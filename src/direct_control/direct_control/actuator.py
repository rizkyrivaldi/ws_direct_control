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

class OffboardControl(Node):
    def __init__(self):
        # Init node
        super().__init__('actuator_test_output')
        self.get_logger().info('Node Initialized')

        # Init variables
        self.armed = False

        # Init qos_profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        self.__init_publisher()
        self.__init_subscriber()
    
        # Debugging
        self.counter = 0

        # RUN THE LOOP
        self.timer_period = 0.2 # Seconds
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

    """ Subscriber initialization """
    def __init_subscriber(self):
        # self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.qos_profile)
        pass

    """ Publisher initialization """
    def __init_publisher(self):
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.os_profile)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
        # self.publisher_attitude = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', self.qos_profile)

    # GLOBAL FUNCTION
    """ Send vehicle commands """
    def VehicleCommand(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        comm = VehicleCommand()
        comm.param1 = param1
        comm.param2 = param2
        comm.param3 = param3
        comm.param4 = param4
        comm.param5 = param5
        comm.param6 = param6
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
    def actuator_output(self, roll = 0.0, pitch = 0.0, yaw = 0.0, thrust = 0.0):
        self.VehicleCommand(command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1 = roll, param2 = pitch, param3 = yaw, param4 = thrust)
        self.get_logger().info(f'Actuator sent: {roll} {pitch} {yaw} {thrust}')

    def main_loop(self):
        if not self.armed:
            self.arm()
        self.counter += 1
        self.get_logger().info(f'Counter: {self.counter}')
        self.actuator_output(thrust = 1.0)


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
