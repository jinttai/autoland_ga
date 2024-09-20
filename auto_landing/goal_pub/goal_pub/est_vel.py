import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import numpy as np
from sklearn.linear_model import LinearRegression

class EstVel(Node):
    def __init__(self):
        super().__init__('est_vel')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber와 Publisher 생성
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/land_position',
            self.position_callback,
            qos_profile
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/est_vel',
            qos_profile
        )

        self.clock = self.get_clock()

        # 위치 데이터와 시간을 저장할 리스트 (최대 10개 유지)
        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.x_data_edit = np.array([])
        self.y_data_edit = np.array([])
        self.z_data_edit = np.array([])

        self.est_len = 10 #

        self.time_data = []

        # 속도 데이터
        self.velocity = [0.0, 0.0, 0.0]

    def position_callback(self, msg):
        # 위치 데이터와 현재 시간을 저장
        position_x = msg.data[0]
        position_y = msg.data[1]
        position_z = msg.data[2]
        current_time = self.clock.now().seconds_nanoseconds()[0]  # seconds로 변환

        # 리스트에 저장 (최대 10개만 유지)
        if len(self.x_data) >= self.est_len:
            self.x_data.pop(0)  # 오래된 데이터 제거
            self.y_data.pop(0)
            self.z_data.pop(0)
            self.time_data.pop(0)
        self.x_data.append(position_x)
        self.y_data.append(position_y)
        self.z_data.append(position_z)
        self.time_data.append(current_time)

        self.x_data_edit = np.array(self.x_data) - self.x_data[0]
        self.y_data_edit = np.array(self.y_data) - self.y_data[0]
        self.z_data_edit = np.array(self.z_data) - self.z_data[0]

        # 데이터가 두 개 이상일 때 선형 회귀 수행
        if len(self.x_data_edit) >= 2:
            # 데이터 준비
            times = np.array(self.time_data).reshape(-1, 1)  # 2D 배열로 변환

            # 선형 회귀 모델 생성 및 학습
            model_x = LinearRegression()
            model_x.fit(times, self.x_data_edit)
            model_y = LinearRegression()
            model_y.fit(times, self.y_data_edit)
            model_z = LinearRegression()
            model_z.fit(times, self.z_data_edit)

            # 기울기 (속도)를 추출
            self.velocity[0] = model_x.coef_[0]
            self.velocity[1] = model_y.coef_[0]
            self.velocity[2] = model_z.coef_[0]

            # 속도 데이터를 메시지로 변환하여 발행
            msg = Float32MultiArray()
            msg.data = self.velocity
            self.pub.publish(msg)

            self.get_logger().info('Estimated velocity: %f, %f, %f, %f' % (self.velocity[0], self.velocity[1], self.velocity[2], current_time))

def main(args=None):
    rclpy.init(args=args)
    est_vel = EstVel()
    rclpy.spin(est_vel)
    est_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
