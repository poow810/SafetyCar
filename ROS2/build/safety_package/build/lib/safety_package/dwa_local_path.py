import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, sqrt, atan2
import numpy as np
from safety_package.dynamic_window_approach import main as dwa_main, Config


class advancedDWALocalPath(Node):

    def __init__(self):
        super().__init__('advanced_dwa_local_path')
        # Publisher and Subscriber 정의
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.global_path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 메시지 초기화 및 플래그 설정
        self.odom_msg = Odometry()
        self.global_path_msg = Path()
        self.lidar_msg = LaserScan()

        self.is_odom = False
        self.is_path = False
        self.is_lidar = False
        self.obstacle = np.array([])
        self.robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0] # 로봇의 현재 위치
        self.goal_pose = [0.0, 0.0]

        # 주기마다 실행되는 타이머 함수 생성
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)
    
    def timer_callback(self):
        if self.is_odom and self.is_path and self.is_lidar:
            # x, config, goal, ob
            # print(f"config: {Config()}")
            # print(f"robot pose: {self.robot_pose}")
            print(self.obstacle)
            u, trajectory = dwa_main(self.robot_pose, Config(), self.goal_pose, self.obstacle)

            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'
            for point in trajectory:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                q = Quaternion.from_euler(0, 0, point[2])
                pose.pose.orientation.x = q.x
                pose.pose.orientation.y = q.y
                pose.pose.orientation.z = q.z
                pose.pose.orientation.w = q.w
                local_path_msg.poses.append(pose)
            print(f"trajectory: {trajectory}")
            self.local_path_pub.publish(local_path_msg)

    def path_callback(self, data):
        self.global_path_msg = data
        self.is_path = True

    def odom_callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z
        self.robot_pose = [position.x, position.y, yaw, linear_x, angular_z]
        self.is_odom = True

    def lidar_callback(self, data):
        ranges = data.ranges
        angle_increment = data.angle_increment
        angle_min = data.angle_min

        obstacle_list = []
        for i, distance in enumerate(ranges):
            if 0.0 < distance < 3.0: # 특정 거리만 
                angle = angle_min + i*angle_increment
                x = self.robot_pose[0] + distance * np.cos(angle + self.robot_pose[2])
                y = self.robot_pose[1] + distance * np.sin(angle + self.robot_pose[2])
                obstacle_list.append([x, y])

        self.obstacle = np.array(obstacle_list)
        self.is_lidar = True

    def goal_callback(self, data):
        self.goal_pose[0] = data.pose.position.x
        self.goal_pose[1] = data.pose.position.y

       
def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw
    
def main(args=None):
    rclpy.init(args=args)
    advanced_dwa_local_path = advancedDWALocalPath()
    rclpy.spin(advanced_dwa_local_path)
    advanced_dwa_local_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
