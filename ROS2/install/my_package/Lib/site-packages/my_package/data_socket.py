import rclpy
import socketio
import os

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from dotenv import load_dotenv
from rclpy.node import Node
from my_package.qos import qos_service, qos_sensor

sio = socketio.Client()
x_offset = -53.97
y_offset = -63.2
x_start_pose = -50.0
y_start_pose = -50.0

global goal_pose
global is_Pose
goal_pose = []
is_Pose = False

@sio.event
def connect() :
    print('connection')

@sio.event
def disconnect() :
    print('disconnection')

@sio.event(namespace='/socketio')
def gridmake(data) :
    global goal_pose
    global is_Pose
    goal_pose = [x_offset+data[1]/100, y_offset+data[0]/100]
    is_Pose = True
    # print(goal_pose)

@sio.event(namespace='/socketio')
def go_home() :
    global goal_pose
    global is_Pose
    goal_pose = [x_start_pose, y_start_pose]
    is_Pose = True
    # print(goal_pose)

class Test(Node):

    def __init__(self):
        super().__init__('test')

        load_dotenv('.env')
        env_var = os.getenv('REACT_APP_PYTHON_URL')
        print(f'{env_var}')
        sio.connect(f'{env_var}', namespaces='/socketio')

        self.goal_pub= self.create_publisher(PoseStamped, 'goal', qos_service)
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback, qos_sensor)
        self.map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback, qos_service)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.odom_msg = Odometry()
        self.goal_msg = PoseStamped()
        self.map_msg = OccupancyGrid()
        self.goal_msg.header.frame_id="map"

        self.is_odom=False
        self.now_pose = [0, 0]

        # sio.emit('send_map', data=self.map_msg.data, namespace='/socketio')



    def timer_callback(self):
        global is_Pose
        global goal_pose
        if self.is_odom :
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            map_point_x= round((x - x_offset) / 0.05)
            map_point_y= round((y - y_offset) / 0.05)
            sio.emit('send_pose', data=[map_point_x, map_point_y], namespace='/socketio')
        if not is_Pose:
            return
        
        if self.now_pose[0]-0.05 < goal_pose[0] < self.now_pose[0]+0.05 and self.now_pose[1]-0.05 < goal_pose[1] < self.now_pose[1]+0.05 :
            return

        self.now_pose = goal_pose
        self.goal_msg.pose.position.x = goal_pose[0]
        self.goal_msg.pose.position.y = goal_pose[1]
        self.goal_pub.publish(self.goal_msg)
    
    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def map_callback(self, msg) :
        self.map_msg = msg

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()