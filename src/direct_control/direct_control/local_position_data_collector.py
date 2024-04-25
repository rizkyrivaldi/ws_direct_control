#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math
import pandas as pd
from datetime import datetime
import csv

from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, TrajectorySetpoint

class PositionDataCollector(Node):
    def __init__(self):
        # Init node
        super().__init__('position_reading')
        self.get_logger().info('Node: position_reading Initialized')

        # Init qos_profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        # self.__init_publisher()
        self.__init_subscriber()

        # Init message variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_control_mode = VehicleControlMode()
        self.trajectory_setpoint = TrajectorySetpoint()

        # Init flags
        self.prev_armed = False

        # RUN THE LOOP
        self.timer_period = 0.02 # Seconds
        self.timer = self.create_timer(self.timer_period, self.main_loop)

    def __init_qos_profile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

    def __init_subscriber(self):
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            self.qos_profile
        )

        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            self.qos_profile
        )

        self.trajectory_setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            self.qos_profile
        )

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg

    def trajectory_setpoint_callback(self, msg):
        self.trajectory_setpoint = msg

    def main_loop(self):

        if self.vehicle_control_mode.flag_armed and self.prev_armed == False:
            # Create new csv file based on time
            self.filename = f"datalog/position_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            # Create new dataframe
            self.df = pd.DataFrame(
                columns = [
                    'timestamp',
                    'x_sp',
                    'y_sp',
                    'z_sp',
                    'x', 
                    'y', 
                    'z', 
                ]
            )
            print(f"Flight logging started, dataframe: \n{self.df}")
            self.prev_armed = True

        elif self.vehicle_control_mode.flag_armed and self.prev_armed == True:
            # Apend data to dataframe
            self.df.loc[len(self.df)] = [
                    self.vehicle_local_position.timestamp,
                    self.trajectory_setpoint.position[0],
                    self.trajectory_setpoint.position[1],
                    self.trajectory_setpoint.position[2],
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z
            ]
        
        elif self.vehicle_control_mode.flag_armed == False and self.prev_armed == True:
            self.df.to_csv(self.filename, index = False)
            self.prev_armed = False
            print(f"Logfile saved at {self.filename}")
    
def main():
    rclpy.init()
    
    node = PositionDataCollector()

    # Spin the ros undefinitely with the node
    rclpy.spin(node)
    print("INTERRUPTED!!!!")

    # Destroy the node when interrupted
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

            
