#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math
import pandas as pd
from datetime import datetime
import csv

from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleAttitude, VehicleTorqueSetpoint, VehicleControlMode

class MixerDataCollector(Node):
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
        self.vehicle_attitude_setpoint = VehicleAttitudeSetpoint()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_torque_setpoint = VehicleTorqueSetpoint()
        self.vehicle_control_mode = VehicleControlMode()

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
        self.vehicle_attitude_setpoint_sub = self.create_subscription(
            VehicleAttitudeSetpoint,
            '/fmu/out/vehicle_attitude_setpoint',
            self.vehicle_attitude_setpoint_callback,
            self.qos_profile
        )

        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            self.qos_profile
        )

        self.vehicle_torque_setpoint_sub = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.vehicle_torque_setpoint_callback,
            self.qos_profile
        )

        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            self.qos_profile
        )

    def vehicle_attitude_setpoint_callback(self, msg):
        self.vehicle_attitude_setpoint = msg

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude = msg

    def vehicle_torque_setpoint_callback(self, msg):
        self.vehicle_torque_setpoint = msg

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg

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

        if self.vehicle_control_mode.flag_armed and self.prev_armed == False:
            # Create new csv file based on time
            self.filename = f"datalog/{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            # Create new dataframe
            self.df = pd.DataFrame(
                columns = [
                    'setpoint_roll_sin', 
                    'setpoint_roll_cos', 
                    'setpoint_pitch_sin', 
                    'setpoint_pitch_cos',
                    'setpoint_yaw_sin',
                    'setpoint_yaw_cos',

                    'feedback_roll_sin', 
                    'feedback_roll_cos', 
                    'feedback_pitch_sin', 
                    'feedback_pitch_cos',
                    'feedback_yaw_sin',
                    'feedback_yaw_cos',
                    
                    'torque_setpoint_roll', 
                    'torque_setpoint_pitch', 
                    'torque_setpoint_yaw', 
                ]
            )
            print(f"Flight logging started, dataframe: \n{self.df}")
            self.prev_armed = True

        elif self.vehicle_control_mode.flag_armed and self.prev_armed == True:
            # Calculate roll pitch yaw for each parameter
            roll_setpoint, pitch_setpoint, yaw_setpoint = self.calculate_roll_pitch_yaw(self.vehicle_attitude_setpoint.q_d, radian = True)
            roll_feedback, pitch_feedback, yaw_feedback = self.calculate_roll_pitch_yaw(self.vehicle_attitude.q, radian = True)

            # Apend data to dataframe
            self.df.loc[len(self.df)] = [
                    math.sin(roll_setpoint),
                    math.cos(roll_setpoint),
                    math.sin(pitch_setpoint),
                    math.cos(pitch_setpoint),
                    math.sin(yaw_setpoint),
                    math.cos(yaw_setpoint),

                    math.sin(roll_feedback),
                    math.cos(roll_feedback),
                    math.sin(pitch_feedback),
                    math.cos(pitch_feedback),
                    math.sin(yaw_feedback),
                    math.cos(yaw_feedback),

                    self.vehicle_torque_setpoint.xyz[0],
                    self.vehicle_torque_setpoint.xyz[1],
                    self.vehicle_torque_setpoint.xyz[2]
            ]
        
        elif self.vehicle_control_mode.flag_armed == False and self.prev_armed == True:
            self.df.to_csv(self.filename, index = False)
            self.prev_armed = False
            print(f"Logfile saved at {self.filename}")
    
def main():
    rclpy.init()
    
    node = MixerDataCollector()

    # Spin the ros undefinitely with the node
    rclpy.spin(node)
    print("INTERRUPTED!!!!")

    # Destroy the node when interrupted
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

            
