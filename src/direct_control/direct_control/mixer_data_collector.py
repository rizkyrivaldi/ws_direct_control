#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math
import pandas as pd
from datetime import datetime
import csv

from px4_msgs.msg import VehicleTorqueSetpoint, VehicleThrustSetpoint, ActuatorMotors, ActuatorControlsStatus, VehicleControlMode

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
        self.vehicle_torque_setpoint = VehicleTorqueSetpoint()
        self.vehicle_thrust_setpoint = VehicleThrustSetpoint()
        self.actuator_controls_status = ActuatorControlsStatus()
        self.actuator_motors = ActuatorMotors()
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
        # Vehicle setpoints
        self.vehicle_torque_setpoint_sub = self.create_subscription(
            VehicleTorqueSetpoint, 
            '/fmu/out/vehicle_torque_setpoint', 
            self.vehicle_torque_setpoint_callback, 
            self.qos_profile
        )

        self.vehicle_thrust_setpoint_sub = self.create_subscription(
            VehicleThrustSetpoint, 
            '/fmu/out/vehicle_thrust_setpoint', 
            self.vehicle_thrust_setpoint_callback, 
            self.qos_profile
        )

        self.actuator_controls_status_sub = self.create_subscription(
            ActuatorControlsStatus,
            '/fmu/out/actuator_controls_status_0',
            self.actuator_controls_status_callback,
            self.qos_profile
        )

        self.actuator_motors_sub = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self.actuator_motors_callback,
            self.qos_profile
        )

        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            self.qos_profile
        )

    def vehicle_torque_setpoint_callback(self, msg):
        self.vehicle_torque_setpoint = msg

    def vehicle_thrust_setpoint_callback(self, msg):
        self.vehicle_thrust_setpoint = msg

    def actuator_controls_status_callback(self, msg):
        self.actuator_controls_status = msg
    
    def actuator_motors_callback(self, msg):
        self.actuator_motors = msg

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg

    def main_loop(self):

        if self.vehicle_control_mode.flag_armed and self.prev_armed == False:
            # Create new csv file based on time
            self.filename = f"datalog/{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            # Create new dataframe
            self.df = pd.DataFrame(
                columns = [
                    'setpoint_roll', 
                    'setpoint_pitch', 
                    'setpoint_yaw', 
                    'setpoint_thrust', 
                    'control_power_roll', 
                    'control_power_pitch', 
                    'control_power_yaw', 
                    'motor_output_1', 
                    'motor_output_2', 
                    'motor_output_3', 
                    'motor_output_4'
                ]
            )
            print(f"Flight logging started, dataframe: \n{self.df}")
            self.prev_armed = True

        elif self.vehicle_control_mode.flag_armed and self.prev_armed == True:
            # Apend data to dataframe
            self.df.loc[len(self.df)] = [
                    self.vehicle_torque_setpoint.xyz[0],
                    self.vehicle_torque_setpoint.xyz[1],
                    self.vehicle_torque_setpoint.xyz[2],
                    -self.vehicle_thrust_setpoint.xyz[2],
                    self.actuator_controls_status.control_power[0],
                    self.actuator_controls_status.control_power[1],
                    self.actuator_controls_status.control_power[2],
                    self.actuator_motors.control[0],
                    self.actuator_motors.control[1],
                    self.actuator_motors.control[2],
                    self.actuator_motors.control[3]
            ]
        
        elif self.vehicle_control_mode.flag_armed == False and self.prev_armed == True:
            self.df.to_csv(self.filename, index = False)
            self.prev_armed = False
            print(f"Logfile saved at {self.filename}")

        # Print the vehicle parameters
        # print(self.vehicle_torque_setpoint.xyz)
        # print(self.vehicle_thrust_setpoint.xyz[2])
        # print(self.actuator_controls_status.control_power)
        # print(self.actuator_motors.control[0:4])
    
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

            
