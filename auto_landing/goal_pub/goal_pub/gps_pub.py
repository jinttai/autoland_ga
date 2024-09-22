import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray
import numpy as np

class EstVel(Node):
    def __init__(self):
        super().__init__('gps_pub')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/land_position',
            qos_profile
        )

        self.timer = self.create_timer(1, self.timer_callback)

        self.velocity = np.array([1.0, 2.0, -1.0])  
        self.position_init = np.array([0.0, 0.0, 0.0])  #lat, lon, alt

    def timer_callback(self):
        msg = Float32MultiArray()

        msg.data = self.position_init.tolist()

        # 메시지 발행
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    est_vel = EstVel()
    rclpy.spin(est_vel)
    est_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
