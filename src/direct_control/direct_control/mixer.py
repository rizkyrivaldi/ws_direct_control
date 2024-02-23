#!/usr/bin/ python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class PWMConverter(Node):
    def __init__(self):
        # Init node
        super().__init__('pwm_converter')
        self.get_logger().info('Node pwm_converter Initialized')

        # Init qos profile
        self.__init_qos_profile()

        # Init Subscriber and Publisher
        self.__init_publisher()
        self.__init_subscriber()

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

    def __init_publisher(self):
        pass

    def __init_subscriber(self):
        pass

    def actuator_to_pwm(self, attitude, throttle):

        """
        - attitude  : [-1, 1]
        - throttle  : [ 0, 1]

        KONFIGURASI MOTOR:
        (3) DEPAN KIRI      : CLOCKWISE         (1) DEPAN KANAN     : C-CLOCKWISE
        (2) BELAKANG KIRI   : C-CLOCKWISE       (4) BELAKANG KANAN  : CLOCKWISE

        RETURN: normalized motor pwm value [-1, 1]

        """

        # Pecah attitude ke roll pitch yaw
        roll, pitch, yaw = attitude

        # Inisialisasi variabel list motor
        motor = [0, 0, 0, 0]

        motor[1] = throttle - pitch + roll + yaw
        motor[2] = throttle + pitch - roll + yaw
        motor[3] = throttle - pitch - roll - yaw
        motor[4] = throttle + pitch + roll - yaw

        for pwm in motor:
            if pwm > 1.0:
                pwm = 1.0
            elif pwm < -1.0:
                pwm = -1.0

        # returns actuator value for each motor, not yet converted to real pwm
        return motor

        

    def main_loop(self):
        pass

    