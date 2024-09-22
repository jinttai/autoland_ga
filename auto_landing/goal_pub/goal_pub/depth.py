import rclpy
import copy
import numpy as np
import logging
import os
import math
from datetime import datetime, timedelta
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sklearn.linear_model import LinearRegression

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Float32MultiArray, Bool

hz = 50 #system hz, must be synchronized to the main callback frequency



class BezierControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.count = 0
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/land_full',
            self.land_position_callback,
            qos_profile
        )

        
    def land_position_callback(self, msg):
        print("ok")


def main(args=None):
    rclpy.init(args=args)

    bezier_control = BezierControl()

    rclpy.spin(bezier_control)

    bezier_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
