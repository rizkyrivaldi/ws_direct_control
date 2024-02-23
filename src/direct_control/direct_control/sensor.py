#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math

from px4_msgs.msg import VehicleAttitude

class SensorReading(Node):
    def __init__(self):
        # Init node
        super().__init__('sensor_reading')
        self.get_logger().info('Node: sensor_reading Initialized')

        # Init qos_profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        # self.__init_publisher()
        self.__init_subscriber()

        # Init message variables
        self.vehicle_attitude = VehicleAttitude()

        # RUN THE LOOP
        self.timer_period = 0.5 # Seconds
        self.timer = self.create_timer(self.timer_period, self.main_loop)

    def __init_qos_profile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

    def __init_subscriber(self):
        self.vehicle_attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, self.qos_profile)

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude = msg

    def calculate_roll_pitch_yaw(self, q_value, radian = False):
        w, x, y, z = q_value

        # calculate roll
        roll = math.atan2(
            2.0 * ((y * z) + (w * x)),
            w**2 - x**2 - y**2 + z**2
        )

        # calculate pitch
        pitch = math.asin(
            2.0 * ((x * z) - (w * y))
        )

        # calculate yaw
        yaw = math.atan2(
            2.0 * ((x * y) + (w * z)),
            w**2 + x**2 - y**2 - z**2
        )        

        if not radian:
            return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

        else:
            return roll, pitch, yaw

    def main_loop(self):
        
        # Calculate the human readable attitude, convert quaternion to euler
        roll, pitch, yaw = self.calculate_roll_pitch_yaw(self.vehicle_attitude.q)
        self.get_logger().info(f'roll: {roll:.2f}\t pitch: {pitch:.2f}\t yaw: {yaw:.2f}')
    
def main():
    rclpy.init()
    
    node = SensorReading()

    # Spin the ros undefinitely with the node
    rclpy.spin(node)

    # Destroy the node when interrupted
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

            
