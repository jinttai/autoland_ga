__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import GimbalManagerSetManualControl

# import math
import math

class GimbalController(Node):
    def __init__(self):
        super().__init__("gimbal_controller")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Gimbal angles in radian
        self.pitchangle = -math.pi/2
        self.yawangle = 0.0
        self.counter = 0

        # Create publishers
        self.gimbal_publisher = self.create_publisher(
            GimbalManagerSetManualControl, '/fmu/in/gimbal_manager_set_manual_control', qos_profile
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = GimbalManagerSetManualControl()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.origin_sysid = 0
        msg.origin_compid = 0
        # msg.target_system = 0
        # msg.target_component = 0
        msg.flags = GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_ROLL_LOCK \
                    + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_PITCH_LOCK \
                    + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        # msg.gimbal_device_id = 0
        msg.pitch = self.pitchangle
        msg.yaw = self.yawangle
        msg.pitch_rate = float('nan')
        msg.yaw_rate = float('nan')
        self.gimbal_publisher.publish(msg)
        self.counter += 1

def main(args = None):
    rclpy.init(args=args)

    gimbal_controller = GimbalController()
    rclpy.spin(gimbal_controller)

    gimbal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)