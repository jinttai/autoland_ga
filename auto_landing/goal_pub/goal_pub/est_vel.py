import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray
import numpy as np
from sklearn.linear_model import LinearRegression


from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleGlobalPosition

class EstVel(Node):
    def __init__(self):
        super().__init__('est_vel')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Subscriber와 Publisher 생성
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/land_position',
            self.position_callback,
            qos_profile
        )

        self.vehicle_global_position_sub = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.vehicle_global_position_callback, 
            qos_profile
        )

        self.odometry_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/land_full',
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
        self.vehicle_position = [0.0, 0.0, 0.0]
        self.vehicle_global_position_lla = [0.0, 0.0, 0.0]

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_lla = [msg.lat, msg.lon, msg.alt]

    def vehicle_position_callback(self, msg):
        self.vehicle_position[0] = msg.x
        self.vehicle_position[1] = msg.y
        self.vehicle_position[2] = msg.z


    def position_callback(self, msg):
        # 위치 데이터와 현재 시간을 저장
        R = 6378137.0 # 지구 반지름

        lat = msg.data[0]
        lon = msg.data[1]
        alt = msg.data[2]

        d_lat_rad = math.radians(lat - self.vehicle_global_position_lla[0])
        d_lon_rad = math.radians(lon - self.vehicle_global_position_lla[1])
        lat_ref_rad = math.radians(self.vehicle_global_position_lla[0])
        d_alt = alt - self.vehicle_global_position_lla[2]

        x = d_lat_rad * R
        y = d_lon_rad * math.cos(lat_ref_rad) * R
        

        position_x = self.vehicle_position[0] + x
        position_y = self.vehicle_position[1] + y
        position_z = self.vehicle_position[2] - d_alt # NED frame
        position_xyz = [0.0, 0.0, 0.0]# [position_x, position_y, position_z]
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
        self.time_data_edit = np.array(self.time_data) - self.time_data[0]

        # 데이터가 두 개 이상일 때 선형 회귀 수행
        if len(self.x_data_edit) >= 5:
            # 데이터 준비
            times = np.array(self.time_data_edit).reshape(-1, 1)  # 2D 배열로 변환

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
            xyz_vel = position_xyz + self.velocity
            msg.data = xyz_vel
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    est_vel = EstVel()
    rclpy.spin(est_vel)
    est_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
