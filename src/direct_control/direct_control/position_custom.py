#!/usr/bin/env python3

# Ros imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

# Math imports
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        # Init node
        super().__init__('position_mode')
        self.get_logger().info('Node position_mode Initialized')

        # Init qos_profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        self.__init_publisher()
        self.__init_subscriber()

        # Init message variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Init position setpoints
        self.current_mission = 0
        self.takeoff_height = 8.0
        self.position_setpoint = {
            # Dataset A
            # "takeoff" : [0.0, 0.0, self.takeoff_height],
            # "point_a" : [10.0, 0.0, self.takeoff_height],
            # "point_b" : [10.0, 10.0, self.takeoff_height],
            # "point_c" : [0.0, 10.0, self.takeoff_height],
            # "point_d" : [0.0, 0.0, self.takeoff_height]

            # Dataset B
            # "takeoff" : [0.0, 0.0, self.takeoff_height],
            # "point_a" : [7.0, 0.0, self.takeoff_height],
            # "point_b" : [7.0, 7.0, self.takeoff_height],
            # "point_c" : [0.0, 7.0, self.takeoff_height],
            # "point_d" : [0.0, 0.0, self.takeoff_height]

            # Dataset C
            "takeoff" : [0.0, 0.0, self.takeoff_height],
            "point_a" : [8.0, 0.0, self.takeoff_height],
            "point_b" : [8.0, 8.0, self.takeoff_height],
            "point_c" : [0.0, 8.0, self.takeoff_height],
            "point_d" : [0.0, 0.0, self.takeoff_height]
        }

        # RUN THE LOOP
        self.timer_period = 0.01 # Seconds
        self.timer = self.create_timer(self.timer_period, self.main_loop)

    # Local Variables
    def __init_qos_profile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

    def __init_publisher(self):
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            self.qos_profile
            )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            self.qos_profile
            )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            self.qos_profile
            )

    def __init_subscriber(self):
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, 
            self.qos_profile
            )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            self.qos_profile
            )

    # Callback Functions
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # Publish Functions
    def publish_vehicle_command(self, command, param1 = None, param2 = None, param3 = None, param4 = None, param5 = None, param6 = None, param7 = None):
        msg = VehicleCommand()
        if not param1 == None:
            msg.param1 = param1

        if not param2 == None:
            msg.param2 = param2

        if not param3 == None:
            msg.param3 = param3

        if not param4 == None:
            msg.param4 = param4
        
        if not param5 == None:
            msg.param5 = param5

        if not param6 == None:
            msg.param6 = param6

        if not param7 == None:
            msg.param7 = param7

        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, -z]
        msg.yaw = np.deg2rad(90)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    # Other functions
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # Main loop funtion
    def main_loop(self):
        self.publish_offboard_control_heartbeat_signal()

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.engage_offboard_mode()

        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()

        # Missions
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            # Takeoff
            if -self.vehicle_local_position.z < self.position_setpoint["takeoff"][2] and self.current_mission == 0:
                self.publish_position_setpoint(
                    self.position_setpoint["takeoff"][0],
                    self.position_setpoint["takeoff"][1],
                    self.position_setpoint["takeoff"][2]
                )
            
            elif self.current_mission == 0:
                self.current_mission += 1

            # Go to point A
            if self.vehicle_local_position.x < self.position_setpoint["point_a"][0] and self.current_mission == 1:
                self.publish_position_setpoint(
                    self.position_setpoint["point_a"][0],
                    self.position_setpoint["point_a"][1],
                    self.position_setpoint["point_a"][2]
                )

            elif self.current_mission == 1:
                self.current_mission += 1

            # Go to point B
            if self.vehicle_local_position.y < self.position_setpoint["point_b"][1] and self.current_mission == 2:
                self.publish_position_setpoint(
                    self.position_setpoint["point_b"][0],
                    self.position_setpoint["point_b"][1],
                    self.position_setpoint["point_b"][2]
                )

            elif self.current_mission == 2:
                self.current_mission += 1

            # Go to point C
            if self.vehicle_local_position.x > self.position_setpoint["point_c"][0] and self.current_mission == 3:
                self.publish_position_setpoint(
                    self.position_setpoint["point_c"][0],
                    self.position_setpoint["point_c"][1],
                    self.position_setpoint["point_c"][2]
                )
            
            elif self.current_mission == 3:
                self.current_mission += 1

            # Go to point D
            if self.vehicle_local_position.y > self.position_setpoint["point_d"][1] and self.current_mission == 4:
                self.publish_position_setpoint(
                    self.position_setpoint["point_d"][0],
                    self.position_setpoint["point_d"][1],
                    self.position_setpoint["point_d"][2]
                )

            elif self.current_mission == 4:
                self.current_mission += 1

            # Landing
            if self.current_mission == 5:
                self.land()
                exit(0)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        


    
    