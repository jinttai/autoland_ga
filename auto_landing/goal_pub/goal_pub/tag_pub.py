import numpy as np
import rclpy
import logging
import os
from datetime import datetime, timedelta
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sklearn.linear_model import LinearRegression

from tf2_msgs.msg import TFMessage as TfMsg
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand
from std_msgs.msg import Float32MultiArray

def quat2R(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                           
    return rot_matrix

class TagPublisher(Node):
    def __init__(self):
        super().__init__('tag_pose')        
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        
        self.drone_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.att_callback,
            qos_profile
        )

        self.drone_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.gps_callback,
            qos_profile
        )
        
        self.tag_sub = self.create_subscription(
            TfMsg,
            'tf',
            self.tag_callback,
            10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )


        # parameter

        self.xf_pub = self.create_publisher(Float32MultiArray, 'bezier_waypoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        
        self.first = True
        self.yaw = 0
        self.drone_q = [1.0,0.0,0.0,0.0]
        self.drone_world = [0.0, 0.0, 0.0]
        self.waypoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        
        self.camera_position = [-0.1, 0.0, 0.6] # 카메라 위치 (카메라의 중심점) x: 앞으로, y: 오른쪽으로, z: 아래로

        # timer

        self.timer = self.create_timer(1, self.timer_callback)

        # logger

        log_dir = os.path.join(os.getcwd(), 'src/auto_landing/log')
        os.makedirs(log_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        log_file = os.path.join(log_dir,  f'tag_log_{current_time}.txt')
        logging.basicConfig(filename=log_file, level=logging.INFO, format='%(message)s')
        self.logger = logging.getLogger(__name__)

    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        self.logger.info(*args, **kwargs)
    
    def tag_callback(self, msg):
        try:
            transform = msg.transforms[0].transform
            frame_id = msg.transforms[0].child_frame_id
            tag_pose = transform.translation
            tag_body = np.array([-tag_pose.y, tag_pose.x, tag_pose.z])  # 카메라의 위가 앞을 바라보고 있을 때
            self.rotation = quat2R(self.drone_q)
            drone2tag_world = np.matmul(self.rotation,tag_body)
            tag_world = drone2tag_world + self.drone_world + self.camera_position
            current_waypoint = [tag_world[0], tag_world[1], tag_world[2]+0.05, 0., 0., 0.5] 
            self.waypoint = current_waypoint
            self.first = False
            
            # tag보이면 mission에서 offboard로 전환
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                    param1=1.0, # main mode
                    param2=6.0  # offboard mode
                )

            self.print(f"tag_world : {tag_world}    drone_world : {self.drone_world}    id : {frame_id}")

        except Exception as e:
            error = e
    
    def timer_callback(self):
        if self.first == False:
            xf_msg = Float32MultiArray()
            xf_msg.data = self.waypoint
            self.xf_pub.publish(xf_msg)        

    def att_callback(self, msg):
        try:
            self.drone_q = msg.q
        except:
            self.get_logger().info("Oh no,,, att not received")

    def gps_callback(self, msg):
        try:
            self.drone_world = np.array([msg.x, msg.y, msg.z]) # NED frame
            self.yaw = msg.heading
            self.rotation_yaw = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0],
                                          [np.sin(self.yaw), np.cos(self.yaw), 0],
                                          [0, 0, 1]])
        except:
            self.get_logger().info("Oh no,,, position")

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg

    def publish_vehicle_command(self, command, **kwargs):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", float('nan'))
        msg.param2 = kwargs.get("param2", float('nan'))
        msg.param3 = kwargs.get("param3", float('nan'))
        msg.param4 = kwargs.get("param4", float('nan'))
        msg.param5 = kwargs.get("param5", float('nan'))
        msg.param6 = kwargs.get("param6", float('nan'))
        msg.param7 = kwargs.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def get_rotation_matrix(self):
        # Roll (x축) 회전 행렬
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(self.roll), -np.sin(self.roll)],
                        [0, np.sin(self.roll), np.cos(self.roll)]])
        
        # Pitch (y축) 회전 행렬
        R_y = np.array([[np.cos(self.pitch), 0, np.sin(self.pitch)],
                        [0, 1, 0],
                        [-np.sin(self.pitch), 0, np.cos(self.pitch)]])
        
        # Yaw (z축) 회전 행렬
        R_z = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0],
                        [np.sin(self.yaw), np.cos(self.yaw), 0],
                        [0, 0, 1]])

        # 최종 회전 행렬 (Yaw * Roll * Pitch 순서로 곱함)
        R = np.dot(R_z, np.dot(R_x, R_y))
        
        return R            

def main(args=None):
    rclpy.init(args=args)
    tagpublisher = TagPublisher()
    rclpy.spin(tagpublisher)

    tagpublisher.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
