#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import ActuatorControlsStatus
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleThrustSetpoint

from utils.NN import BPNN

class OffboardControl(Node):
    def __init__(self):
        # Init node
        super().__init__('actuator_output')
        self.get_logger().info('Node actuator_output Initialized')

        # Init variables
        self.armed = False
        self.motor_pwm = [0, 0, 0, 0]

        # Init joystick control variables
        self.thrust_control = 0.5
        self.thrust_rate = 0.05
        self.roll_control = 0.0
        self.pitch_control = 0.0
        self.yaw_control = 0.0

        # Init qos_profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        self.__init_publisher()
        self.__init_subscriber()

        # Init message variables
        self.vehicle_control_mode = VehicleControlMode()
        self.actuator_controls_status = ActuatorControlsStatus()
        self.manual_control_setpoint = ManualControlSetpoint()
        self.vehicle_torque_setpoint = VehicleTorqueSetpoint()
        self.vehicle_thrust_setpoint = VehicleThrustSetpoint()

        # Init BPNN model
        self.mixer_v4 = BPNN(weight = "model/mixer_v4.3_best_weight.pickle")
    
        # Debugging
        self.counter = 0

        # RUN THE LOOP
        self.timer_period = 0.01 # Seconds
        self.timer = self.create_timer(self.timer_period, self.main_loop)

    # LOCAL FUNCTION
    """ Set QOS Profile for publish and subscribe """
    def __init_qos_profile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

    # SUBSCRIBER NODES
    """ Subscriber initialization """
    def __init_subscriber(self):
        self.status_sub = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_callback, 
            self.qos_profile
        )

        self.actuator_control_sub = self.create_subscription(
            ActuatorControlsStatus, 
            '/fmu/out/actuator_controls_status_0', 
            self.actuator_control_status_callback, 
            self.qos_profile
        )

        self.manual_control_setpoint_sub = self.create_subscription( # Joystick input response
            ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',
            self.manual_control_setpoint_callback,
            self.qos_profile
        )

        self.attitude_control_output_sub = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.vehicle_torque_setpoint_callback,
            self.qos_profile
        )

        self.thrust_control_output_sub = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            self.vehicle_thrust_setpoint_callback,
            self.qos_profile
        )
        
    # PUBLISHER NODES
    """ Publisher initialization """
    def __init_publisher(self):
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            self.qos_profile
            )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            self.qos_profile
            )

    # CALLBACK NODES
    """ Flag Subscriptions (vehicle control mode)"""
    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg

    """ Actuator control status subscription"""
    def actuator_control_status_callback(self, msg):
        self.actuator_control_status = msg

    """ Manual control setpoint subscription """
    def manual_control_setpoint_callback(self, msg):
        self.manual_control_setpoint = msg
        self.thrust_control += self.manual_control_setpoint.throttle * self.thrust_rate

        # Limiter
        if self.thrust_control > 1.0:
            self.thrust_control = 1.0
        elif self.thrust_control < 0.0:
            self.thrust_control = 0.0

    """ Vehicle attitude control output from pixhawk subscription """
    def vehicle_torque_setpoint_callback(self, msg):
        self.vehicle_torque_setpoint = msg

    """ Vehicle thrust control output from pixhawk subscription """
    def vehicle_thrust_setpoint_callback(self, msg):
        self.vehicle_thrust_setpoint = msg

    """ Actuator to mixer """
    def actuator_to_pwm(self, attitude, thrust):

        """
        - attitude  : [-1, 1]
        - thrust    : [ 0, 1]

        KONFIGURASI MOTOR:
        (3) DEPAN KIRI      : CLOCKWISE         (1) DEPAN KANAN     : C-CLOCKWISE
        (2) BELAKANG KIRI   : C-CLOCKWISE       (4) BELAKANG KANAN  : CLOCKWISE

        RETURN: normalized motor pwm value [-1, 1]

        """

        # Pecah attitude ke roll pitch yaw
        roll, pitch, yaw = attitude

        self.motor_pwm[0] = thrust - roll + pitch + yaw
        self.motor_pwm[1] = thrust + roll - pitch + yaw# * 18.0/25.0
        self.motor_pwm[2] = thrust + roll + pitch - yaw
        self.motor_pwm[3] = thrust - roll - pitch - yaw# * 18.0/25.0

        for i in range(len(self.motor_pwm)):
            # Limiter
            if self.motor_pwm[i] > 1.0:
                self.motor_pwm[i] = 1.0
            elif self.motor_pwm[i] < 0.0:
                self.motor_pwm[i] = 0.0

            # Adjust to fit the [-1, 1] actuator as [0, 2000]
            # old_range = (1 - 0)
            # new_range = (1 - (-1))
            # self.motor_pwm[i] = (((self.motor_pwm[i] - 0.0) * new_range) / old_range) + (-1.0)

    def actuator_to_pwm_nn(self):
        input_vector = [
            self.vehicle_torque_setpoint.xyz[0],
            self.vehicle_torque_setpoint.xyz[1],
            self.vehicle_torque_setpoint.xyz[2],
            -self.vehicle_thrust_setpoint.xyz[2]
        ]

        self.motor_pwm = self.mixer_v4.feedForward(input_vector)

        # Adjust to fit the [-1, 1] actuator as [0, 2000]
        old_range = (1 - 0)
        new_range = (1 - (-1))
        self.motor_pwm = (((self.motor_pwm - 0.0) * new_range) / old_range) + (-1.0)
        for n in range(len(self.motor_pwm)):
            if self.motor_pwm[n] < 0.1:
                self.motor_pwm[n] = 0.1
        

    """ Send vehicle commands """
    def VehicleCommand(self, command, param1 = None, param2 = None, param3 = None, param4 = None, param5 = None, param6 = None, param7 = None):
        comm = VehicleCommand()
        if not param1 == None:
            comm.param1 = param1

        if not param2 == None:
            comm.param2 = param2

        if not param3 == None:
            comm.param3 = param3

        if not param4 == None:
            comm.param4 = param4
        
        if not param5 == None:
            comm.param5 = param5

        if not param6 == None:
            comm.param6 = param6

        if not param7 == None:
            comm.param7 = param7

        comm.command = command
        comm.target_system = 1
        comm.target_component = 1
        comm.source_system = 1
        comm.source_component = 1
        comm.from_external = True
        comm.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(comm)

    """ Send an arm command to the vehicle """
    def arm(self):
        self.VehicleCommand(command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True

    """ Send a disarm command to the vehicle """
    def disarm(self):
        self.VehicleCommand(command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 0.0)
        self.get_logger().info('Disarm command sent')

    """ Send actuator control directly """
    def actuator_output(self, motor1 = 0.0, motor2 = 0.0, motor3 = 0.0, motor4 = 0.0):
        self.VehicleCommand(command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1 = motor1, param2 = motor2, param3 = motor3, param4 = motor4)
        # self.get_logger().info(f'Actuator sent: {motor1} {motor2} {motor3} {motor4}')
        print(f'Actuator sent: \nmotor1: {motor1} \nmotor2: {motor2} \nmotor3: {motor3} \nmotor4: {motor4}')

    def main_loop(self):
        self.actuator_to_pwm_nn()
        self.actuator_output(motor1 = self.motor_pwm[0], motor2 = self.motor_pwm[1], motor3 = self.motor_pwm[2], motor4 = self.motor_pwm[3])

def main():
    rclpy.init()
    
    node = OffboardControl()

    # Spin the ros undefinitely with the node
    rclpy.spin(node)

    # Destroy the node when interrupted
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()    
