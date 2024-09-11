import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraTopicRelay(Node):

    def __init__(self):
        super().__init__('camera_topic_relay')

        # /camera/image_raw 토픽을 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # /image_raw 토픽으로 발행
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)

        # /camera/camera_info 토픽을 구독
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # /camera_info 토픽으로 발행
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

    def image_callback(self, msg):
        # 수신한 이미지 메시지를 새로운 토픽으로 발행
        self.image_publisher.publish(msg)

    def camera_info_callback(self, msg):
        # 수신한 카메라 정보 메시지를 새로운 토픽으로 발행
        self.camera_info_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_topic_relay = CameraTopicRelay()

    rclpy.spin(camera_topic_relay)

    camera_topic_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
