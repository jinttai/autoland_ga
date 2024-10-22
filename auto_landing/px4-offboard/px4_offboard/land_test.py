__author__ = "Kyungjun Oh"
__contact__ = "frankok@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import SensorGps
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import message for auto landing, YOLOv5
from std_msgs.msg import Float32MultiArray

# import other libraries
import os
import time
import logging
import numpy as np
from datetime import datetime, timedelta

class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        """
        0. Configure QoS profile for publishing and subscribing
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        """
        1. Constants
        """
        # given constants
        self.camera_to_center = 0.0                             # distance from camera to center of the vehicle
        self.corridor_radius = 2.0

        # acceptance constants
        self.mc_acceptance_radius = 0.3
        self.nearby_acceptance_radius = 30
        self.offboard_acceptance_radius = 10.0                   # mission -> offboard acceptance radius
        self.transition_acceptance_angle = 0.8                   
        self.heading_acceptance_angle = 0.1                      # 0.1 rad = 5.73 deg

        # bezier curve constants
        self.fast_vmax = 5.0
        self.slow_vmax = 3.5
        self.very_slow_vmax = 0.2
        self.max_acceleration = 9.81 * np.tan(10 * np.pi / 180)  # 10 degree tilt angle
        self.mc_start_speed = 0.0001
        self.mc_end_speed = 0.0001
        self.bezier_threshold_speed = 0.7
        self.bezier_minimum_time = 3.0

        # alignment constants
        self.yaw_speed = 0.1                                    # 0.1 rad = 5.73 deg



        """
        2. Logging setup
        """
        log_dir = os.path.join(os.getcwd(), 'src/vehicle_controller/test_nodes/log')
        os.makedirs(log_dir, exist_ok=True)
        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        log_file = os.path.join(log_dir,  f'log_{current_time}.txt')
        logging.basicConfig(filename=log_file, level=logging.INFO, format='%(message)s')
        self.logger = logging.getLogger(__name__)

        # initialize log info & error
        self.log_dict = {
            'auto': [],
            'subphase': [],
            'utc_time': [],
            'pos[0]' : [],
            'pos[1]': [],
            'pos[2]': [],
            'pos_gps[0]': [],
            'pos_gps[1]': [],
            'pos_gps[2]': []
        }
        self.error = [np.inf]

        """
        3. Load waypoints (GPS)
        """
        self.WP = [np.array([0.0, 0.0, 0.0])]
        self.gps_WP = [np.array([0.0, 0.0, 0.0])]
        self.home_position = np.array([0.0, 0.0, 0.0])
        self.start_yaw = 30.0

       
        """
        4. Phase and subphase
        """
        # phase description
        # 0 : before flight
        
        self.phase = 0

       
        self.subphase = 'before flight'

        """
        5. State variables
        """
        # vehicle status
        self.auto = 0                           # 0: manual, 1: auto
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.detect_apriltag = False

        # vehicle position, velocity, and yaw
        self.pos = np.array([0.0, 0.0, 0.0])        # local
        self.pos_gps = np.array([0.0, 0.0, 0.0])    # global
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        
        # goal position and yaw
        self.goal_position = None
        self.goal_yaw = None

        # Bezier curve
        self.num_bezier = 0
        self.bezier_counter = 0
        self.bezier_points = None

        # UTC time
        self.utc_time = 0.0
        self.utc_year = 0
        self.utc_month = 0
        self.utc_day = 0
        self.utc_hour = 0
        self.utc_min = 0
        self.utc_sec = 0
        self.utc_ms = 0

        """
        6. Create Subscribers
        """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile
        )
        self.gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile
        )
        self.tag_subscriber = self.create_subscription(
            Float32MultiArray, 'bezier_waypoint', self.tag_callback, 10
        )

        """
        7. Create Publishers
        """
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.start_yaw_publisher = self.create_publisher(
            Float32MultiArray, '/auto_land_home_info', 10
        )

        """
        8. timer setup
        """
        self.time_period = 0.05     # 20 Hz
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
        
        print("Successfully executed: vehicle_controller")
        print("Start the mission\n")
        self.print("Auto Latitude	Longtitude	 Altitude	  UTC Year	 UTC Month	  UTC Day	  UTC Hour	  UTC Min	  UTC Sec	   UTC ms  WPT")

    
    """
    Services
    """   
    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        self.logger.info(*args, **kwargs)

    def set_home(self, home_position_gps):
        self.home_position = self.pos   # set home position
        self.start_yaw = self.yaw     # set initial yaw
        self.WP.append(np.array([-self.camera_to_center * np.cos(self.start_yaw), -self.camera_to_center * np.sin(self.start_yaw), 0.0]))  # set the camera's position to the home position

    def generate_bezier_curve(self, xi, xf, vmax):
        # reset counter
        self.bezier_counter = 0

        # total time calculation
        total_time = np.linalg.norm(xf - xi) / vmax * 2      # Assume that average velocity = vmax / 2.     real velocity is lower then vmax
        if total_time <= self.bezier_minimum_time:
            total_time = self.bezier_minimum_time

        direction = np.array((xf - xi) / np.linalg.norm(xf - xi))
        vf = self.mc_end_speed * direction
        if np.linalg.norm(self.vel) < self.bezier_threshold_speed:
            vi = self.mc_start_speed * direction
        else:
            vi = self.vel
            self.bezier_counter = int(1 / self.time_period) - 1

        point1 = xi
        point2 = xi + vi * total_time / 3
        point3 = xf - vf * total_time / 3
        point4 = xf

        # Bezier curve
        self.num_bezier = int(total_time / self.time_period)
        bezier = np.linspace(0, 1, self.num_bezier).reshape(-1, 1)
        bezier = point4 * bezier**3 +                             \
                3 * point3 * bezier**2 * (1 - bezier) +           \
                3 * point2 * bezier**1 * (1 - bezier)**2 +        \
                1 * point1 * (1 - bezier)**3
        
        return bezier
    
    def run_bezier_curve(self, bezier_points, goal_yaw=None):
        if goal_yaw is None:
            goal_yaw = self.yaw

        if self.bezier_counter < self.num_bezier:
            self.publish_trajectory_setpoint(
                position_sp = bezier_points[self.bezier_counter],
                yaw_sp = self.yaw + np.sign(np.sin(goal_yaw - self.yaw)) * self.yaw_speed
            )
            self.bezier_counter += 1
        else:
            self.publish_trajectory_setpoint(
                position_sp = bezier_points[-1],        # last point (goal position)
                yaw_sp = self.yaw + np.sign(np.sin(goal_yaw - self.yaw)) * self.yaw_speed
            )

    def make_patrol_trajectory(self, xi, length, vmax):
        # reset counter
        self.patrol_counter = 0

        # total time calculation
        nominal_time = length / vmax    
        if nominal_time <= self.bezier_minimum_time:
            nominal_time = self.bezier_minimum_time
     
        self.patrol_counter = int(1 / self.time_period) - 1

        # linear trajectory
        self.num_lin = int(nominal_time / self.time_period)
        lin_trajectory = np.linspace(0, 1, self.num_lin).reshape(-1, 1)
        lin_trajectory = xi + lin_trajectory * length * np.array([1, 0, 0])
        
        # circle curve
        self.num_circle = int(nominal_time * 2 * 3.14 / self.time_period)
        circle_trajectory = np.array([])
        for i in range(self.num_circle):
            circle_point = xi + length * np.array([np.cos(2 * np.pi * i / self.num_circle), np.sin(2 * np.pi * i / self.num_circle), 0])

            combined_trajectory = np.vstack((lin_trajectory, circle_point))

        return combined_trajectory
    
    """
    Callback functions for the timers
    """    
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(velocity=True)


    def main_timer_callback(self):
        if self.phase == 0:
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.set_home(self.pos_gps)
                self.phase = 1
                self.subphase = 'land_start'
                self.print('\n[phase : 0 -> 1]')
                self.print('[subphase : before flight -> position]\n')

        elif self.phase == 1:
            if self.subphase == 'land_start':
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.bezier_down = self.generate_bezier_curve(self.pos, np.array([self.pos[0],self.pos[1], -1.0]), 1.0)  
                    self.subphase = 'slow_down'   
                    self.print('\n[subphase : land_start -> slow_down]\n')                       
                    # if self.pos[2] > -self.landing_height:
                    #     self.subphase = 'patrol'
                    #     self.patrol_length = 5.0
                    #     self.patrol_trajectory = self.make_patrol_trajectory(self.pos, self.patrol_length, self.fast_vmax)
                    #     self.print('\n[subphase : land_start -> patrol]\n')
                    
            
            # elif self.subphase == 'patrol':
            #     self.run_bezier_curve(self.patrol_trajectory)
            #     if self.detect_apriltag:
            #         self.subphase = 'prepare landing'
            #         self.print('\n[subphase : patrol -> prepare landing]\n')

            elif self.subphase == 'slow_down' :
                self.run_bezier_curve(self.bezier_down)
                if self.detect_apriltag:
                        self.subphase = 'prepare landing'
                        self.print('\n[subphase : slow_down -> prepare landing]\n')

            elif self.subphase == 'prepare landing':
                home_info = Float32MultiArray()
                home_info.data = list(self.home_position) + [self.start_yaw]
                self.start_yaw_publisher.publish(home_info)
                self.subphase = 'auto landing'
                self.print('\n[subphase : prepare landing -> auto landing]\n')

            elif self.subphase == 'auto landing':
                if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.subphase = 'mission complete'
                    self.print('\n[subphase : auto landing -> mission complete]\n')

            elif self.subphase == 'mission complete':
                self.print("\nMission complete")
                self.print("Congratulations!\n")
                self.destroy_node()
                rclpy.shutdown()



    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading
        if self.phase != -1:
            # set position relative to the home position after takeoff
            self.pos = self.pos - self.home_position

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        self.pos_gps = np.array([msg.lat, msg.lon, msg.alt])

    def vehicle_gps_callback(self, msg):
        self.utc_time = msg.time_utc_usec
        self.utc_datetime = datetime(1970, 1, 1) + timedelta(microseconds=self.utc_time)
        self.utc_year = self.utc_datetime.year
        self.utc_month = self.utc_datetime.month
        self.utc_day = self.utc_datetime.day
        self.utc_hour = self.utc_datetime.hour
        self.utc_minute = self.utc_datetime.minute
        self.utc_sec = self.utc_datetime.second
        self.utc_ms = self.utc_datetime.microsecond // 1000  # Convert microseconds to milliseconds
    
    def tag_callback(self, msg):
        self.detect_apriltag = True
    """
    Functions for publishing topics.
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
        self.vehicle_command_publisher.publish(msg)
    
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
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self, **kwargs):
        msg = TrajectorySetpoint()
        msg.position = list( kwargs.get("position_sp", np.nan * np.zeros(3)) + self.home_position )
        msg.velocity = list( kwargs.get("velocity_sp", np.nan * np.zeros(3)) )
        msg.yaw = kwargs.get("yaw_sp", float('nan'))
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


    
    
def main(args = None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
