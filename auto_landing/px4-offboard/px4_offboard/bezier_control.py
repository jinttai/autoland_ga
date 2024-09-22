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

class points():
    def __init__(self,xi,xf,vi,vf,hz):
        self.xi = xi
        self.xf = xf
        self.vi = vi
        self.vf = vf
        self.hz = hz
        self.vmax = 1 ##TBD
        self.amax = 2 ##TBD

        (self.t,self.point1,self.point2,self.point3,self.point4) = self.time_calibrate()
        self.count = int(self.t * self.hz)
        self.timecount = 1/(self.count+1e-10)
    def bezier_x(self):
        bezx= np.zeros(self.count)
        for count in range(self.count):
            bezx[count] = (self.point4[0]*(count * self.timecount)**3 +
            3 * self.point3[0]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[0]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[0]*(1-count * self.timecount)**3
            )
        return bezx
    def bezier_y(self):
        bezy= np.zeros(self.count)
        for count in range(self.count):
            bezy[count] = (self.point4[1]*(count * self.timecount)**3 +
            3 * self.point3[1]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[1]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[1]*(1-count * self.timecount)**3
            )
        return bezy
    def bezier_z(self):
        bezz= np.zeros(self.count)
        for count in range(self.count):
            bezz[count] = (self.point4[2]*(count * self.timecount)**3 +
            3 * self.point3[2]*(count * self.timecount)**2 * (1-count * self.timecount)+
            3 * self.point2[2]*(count * self.timecount)**1 * (1-count * self.timecount)**2+
            1 * self.point1[2]*(1-count * self.timecount)**3
            )
        return bezz
    def bezier_vx(self):
        bezvx = np.zeros(self.count)
        for count in range(self.count):
            bezvx[count] = (self.hz * self.timecount) * (3 * self.point4[0] * (count * self.timecount) ** 2 +
                                               6 * self.point3[0] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[0] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[0] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[0] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[0] * (1 - count* self.timecount)**2)
        return bezvx
    def bezier_vy(self):
        bezvy = np.zeros(self.count)
        for count in range(self.count):
            bezvy[count] = (self.hz * self.timecount) * (3 * self.point4[1] * (count * self.timecount) ** 2 +
                                               6 * self.point3[1] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[1] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[1] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[1] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[1] * (1 - count* self.timecount)**2)
        return bezvy
    def bezier_vz(self):
        bezvz = np.zeros(self.count)
        for count in range(self.count):
            bezvz[count] = (self.hz * self.timecount) * (3 * self.point4[2] * (count * self.timecount) ** 2 +
                                               6 * self.point3[2] * (count * self.timecount) * (1 - count * self.timecount) +
                                               3 * self.point2[2] * (1 - count * self.timecount) ** 2 +
                                               -3 * self.point3[2] * (count * self.timecount) ** 2 +
                                               -6 * self.point2[2] * (count * self.timecount) * (1 - count * self.timecount) +
                                               -3 * self.point1[2] * (1 - count* self.timecount)**2)
        return bezvz
    def time_calibrate(self):
        t = np.linalg.norm(self.xf - self.xi)/self.vmax
        flag = 0
        a_max = 0
        while flag == 0:
            point1 = self.xi
            point3 = self.xf - self.vf*t/3
            point4 = self.xf
            # point2 = (self.xi + self.xf * 2)/3
            point2 = self.xi + (self.vi + 0.05 * (np.array([self.xf[0]-self.xi[0],self.xf[1]-self.xi[1],0])) / np.linalg.norm(np.array([self.xf[0]-self.xi[0],self.xf[1]-self.xi[1],0])))*t/3  
            flag = 1
        calibrated = ([t,point1,point2,point3,point4])
        return calibrated


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

        # Subscriber
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.command_sub = self.create_subscription(
            Float32MultiArray,
            '/bezier_waypoint',
            self.point_command_callback,
            10
        )

        self.odometry_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )

        self.vehicle_global_position_sub = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.vehicle_global_position_callback, 
            qos_profile
        )

        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/land_full',
            self.land_position_callback,
            qos_profile
        )

        # Publisher
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        
        # Timer
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        # Parameter
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.delta_t = -1
        self.delta_t_goal = 0
        self.trigger = 0
        self.phase_check = False
        self.detect = 0
        self.loop_on = 1
        self.pub = 0
        self.yaw_start = 0.0
        self.vehicle_length = np.array([0.5, 0.0, 0.0])
        self.vehicle_global_position_NED = [0.0, 0.0, 0.0]
        self.R = np.array([[np.cos(self.yaw_start), -np.sin(self.yaw_start), 0],
                            [np.sin(self.yaw_start), np.cos(self.yaw_start), 0],
                            [0, 0, 1]])
        
        self.clock = self.get_clock()

        #Way point
        self.vehicle_position = np.zeros(3)
        self.vehicle_velocity = np.zeros(3)
        self.vehicle_global_position_lla = np.zeros(3)
        
        self.xf = np.zeros(3)
        self.vf = np.zeros(3)

        self.goal_position = [0.0, 0.0, 0.0] # landing position in gps, x + 0.5, y, z + 0.4
        self.home_position = [0.0, 0.0, 0.0]

        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.x_data_edit = np.array([])
        self.y_data_edit = np.array([])
        self.z_data_edit = np.array([])
        self.time_data = np.array([])

        self.est_len = 10
        self.time_data = []
        self.velocity = np.zeros(3)
        self.vmax = 1
        self.check_get_gps = 0

        """
        Logging setup
        """
        log_dir = os.path.join(os.getcwd(), 'src/auto_landing/log')
        os.makedirs(log_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        log_file = os.path.join(log_dir,  f'log_{current_time}.txt')
        logging.basicConfig(filename=log_file, level=logging.INFO, format='%(message)s')
        self.logger = logging.getLogger(__name__)

    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        self.logger.info(*args, **kwargs)

    """
    Subscriber Callbacks
    """
    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_lla = [msg.lat, msg.lon, msg.alt]

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state        

    def point_command_callback(self, msg):
        self.xf = np.asfarray(msg.data[0:3]) + self.velocity * np.linalg.norm(self.xf - self.vehicle_position) / self.vmax
        self.vf = np.asfarray(msg.data[3:6])
        self.init_position = copy.deepcopy(self.vehicle_position)
        self.detect = 1
        bezier_points = points(self.init_position, self.xf, self.vehicle_velocity, self.vf, hz)
        self.x = bezier_points.bezier_x()
        self.y = bezier_points.bezier_y()
        self.z = bezier_points.bezier_z()
        self.vx = bezier_points.bezier_vx()
        self.vy = bezier_points.bezier_vy()
        self.vz = bezier_points.bezier_vz()
        self.count = bezier_points.count
        self.t = bezier_points.t#
        self.point1 = bezier_points.point1#
        self.point2 = bezier_points.point2#
        self.point3 = bezier_points.point3#
        self.point4 = bezier_points.point4#
        self.delta_t = 0

    def land_position_callback(self, msg):
        try:
            self.check_get_gps = 1
            position_x = msg.data[0]
            position_y = msg.data[1]
            position_z = msg.data[2]

            self.velocity[0] = msg.data[3]
            self.velocity[1] = msg.data[4]
            self.velocity[2] = msg.data[5]
            
            self.goal_position = [position_x, position_y, position_z]
            self.xf_goal = np.asarray(self.goal_position) + self.velocity * np.linalg.norm(self.xf[2] - self.vehicle_position[2]) / self.vmax #하강 시간동안 이동한 착륙 위치 고려
            self.vf_goal = np.asfarray([0.0, 0.0, 0.5])
            self.init_position_goal = copy.deepcopy(self.vehicle_position)
            bezier_points_goal = points(self.init_position_goal, self.xf_goal, self.vehicle_velocity, self.vf_goal, hz)
            self.x_goal = bezier_points_goal.bezier_x()
            self.y_goal = bezier_points_goal.bezier_y()
            self.z_goal = bezier_points_goal.bezier_z()
            self.count_goal = bezier_points_goal.count
            self.t_goal = bezier_points_goal.t    
            self.print(f"goal_position : {self.goal_position},   xf_goal : {self.xf_goal},   vehicle_position : {self.vehicle_position}, t_goal : {self.t_goal}")
        except Exception as e:
            self.print(f"Error : {e}")   

    def vehicle_position_callback(self, msg):
        self.vehicle_position[0] = msg.x
        self.vehicle_position[1] = msg.y
        self.vehicle_position[2] = msg.z

        if self.delta_t > 0 and self.delta_t < self.count-1 :
            self.vehicle_velocity[0] = self.vx[self.delta_t]
            self.vehicle_velocity[1] = self.vy[self.delta_t]
            self.vehicle_velocity[2] = self.vz[self.delta_t]
        else:
            self.vehicle_velocity[0] = msg.vx
            self.vehicle_velocity[1] = msg.vy
            self.vehicle_velocity[2] = msg.vz

    """
    Publisher Callbacks
    """
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
        self.publisher_vehicle_command.publish(msg)

    def publish_offboard_control_mode(self, **kwargs):
        msg = OffboardControlMode()
        msg.position = kwargs.get("position", False)
        msg.velocity = kwargs.get("velocity", False)
        msg.acceleration = kwargs.get("acceleration", False)
        msg.attitude = kwargs.get("attitude", False)
        msg.body_rate = kwargs.get("body_rate", False)
        msg.thrust_and_torque = kwargs.get("thrust_and_torque", False)
        msg.direct_actuator = kwargs.get("direct_actuator", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)

    def land(self):
        self.print("Landing")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.loop_on = 0   

    """
    Main Timer Callback
    """    
    def cmdloop_callback(self):
        if self.loop_on:
            self.publish_offboard_control_mode(position=True)
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.check_get_gps:
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                if self.delta_t == -1 and self.count_goal > int(1/self.timer_period) and self.delta_t_goal + int(1/self.timer_period) < self.count_goal-1: 
                    trajectory_msg.position[0] = self.x_goal[self.delta_t_goal + int(1/self.timer_period)]#np.nan
                    trajectory_msg.position[1] = self.y_goal[self.delta_t_goal + int(1/self.timer_period)]#np.nan
                    trajectory_msg.position[2] = self.z_goal[self.delta_t_goal + int(1/self.timer_period)]#np.nan
                    trajectory_msg.velocity[0] = np.nan #self.vx[self.delta_t] 
                    trajectory_msg.velocity[1] = np.nan #self.vy[self.delta_t]
                    trajectory_msg.velocity[2] = np.nan #self.vz[self.delta_t]
                    trajectory_msg.yaw = self.yaw_start
                    self.delta_t_goal += 1
                    self.publisher_trajectory.publish(trajectory_msg)
                    self.print(f"edge case - count_goal : {self.count_goal},   delta_t_goal : {self.delta_t_goal},   xf_goal : {self.xf_goal},   vehicle_position : {self.vehicle_position}")
                    
                    if self.delta_t_goal + int(1/self.timer_period) >= self.count_goal-1:
                        self.delta_t_goal = 0

                elif self.delta_t == -1 and (self.count_goal <= int(1/self.timer_period) or self.delta_t_goal + int(1/self.timer_period) >= self.count_goal-1):
                    trajectory_msg.position[0] = self.xf_goal[0]#np.nan
                    trajectory_msg.position[1] = self.xf_goal[1]#np.nan
                    trajectory_msg.position[2] = self.xf_goal[2]
                    trajectory_msg.velocity[0] = np.nan #self.vx[self.delta_t] 
                    trajectory_msg.velocity[1] = np.nan #self.vy[self.delta_t]
                    trajectory_msg.velocity[2] = np.nan #self.vz[self.delta_t]
                    trajectory_msg.yaw = self.yaw_start
                    self.publisher_trajectory.publish(trajectory_msg)
                    self.print(f"edge case no bezier - delta_t_goal : {self.delta_t_goal},   xf_goal : {self.xf_goal},   vehicle_position : {self.vehicle_position}")


                elif self.delta_t + int(1/self.timer_period) < self.count-1 and np.linalg.norm(self.vehicle_position[2]-self.xf[2]) > 0.5 and self.detect:   # if receiving command from the bezier curve
                    trajectory_msg.position[0] = self.x[self.delta_t + int(1/self.timer_period)]#np.nan
                    trajectory_msg.position[1] = self.y[self.delta_t + int(1/self.timer_period)]#np.nan
                    trajectory_msg.position[2] = self.z[self.delta_t + int(1/self.timer_period)]#np.nan
                    trajectory_msg.velocity[0] = np.nan #self.vx[self.delta_t] 
                    trajectory_msg.velocity[1] = np.nan #self.vy[self.delta_t]
                    trajectory_msg.velocity[2] = np.nan #self.vz[self.delta_t]
                    trajectory_msg.yaw = self.yaw_start
                    self.delta_t += 1
                    self.delta_t_goal = 0
                    self.publisher_trajectory.publish(trajectory_msg)
                    self.print(f"apriltag bezier - count : {self.count},   delta_t : {self.delta_t},   xf : {self.xf},   vehicle_position : {self.vehicle_position}")

                    if np.linalg.norm(self.vehicle_position[0]-self.xf[0]) < 1.2 and np.linalg.norm(self.vehicle_position[1]-self.xf[1]) < 1.2 and (self.vehicle_position[2]-self.xf[2] > -0.1):
                        self.land()

                elif self.delta_t + int(1/self.timer_period) >= self.count-1 :
                    trajectory_msg.position[0] = self.xf[0]
                    trajectory_msg.position[1] = self.xf[1]
                    trajectory_msg.position[2] = self.xf[2]
                    trajectory_msg.velocity[0] = np.nan #self.vx[self.delta_t] 
                    trajectory_msg.velocity[1] = np.nan #self.vy[self.delta_t]
                    trajectory_msg.velocity[2] = np.nan #self.vz[self.delta_t]
                    trajectory_msg.yaw = self.yaw_start
                    self.publisher_trajectory.publish(trajectory_msg)
                    self.print(f"apriltag no bezier - xf : {self.xf},   vehicle_position : {self.vehicle_position}")

                    if np.linalg.norm(self.vehicle_position[0]-self.xf[0]) < 1.2 and np.linalg.norm(self.vehicle_position[1]-self.xf[1]) < 1.2 and (self.vehicle_position[2]-self.xf[2] > -0.1):
                        self.land()

                    

        

def main(args=None):
    rclpy.init(args=args)

    bezier_control = BezierControl()

    rclpy.spin(bezier_control)

    bezier_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
