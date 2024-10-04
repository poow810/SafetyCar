import rclpy
import socketio

from rclpy.node import Node

sio = socketio.Client()

global goal_pose
goal_pose = []

@sio.event
def connect() :
    print('connection')

@sio.event
def disconnect() :
    print('disconnection')

@sio.event(namespace='/socketio')
def gridmake(data) :
    global goal_pose
    goal_pose = data
    print(goal_pose)

def get_goal_pose() :
    return goal_pose

class Test(Node):

    def __init__(self):
        super().__init__('test')
        sio.connect('http://localhost:8000/socket', namespaces='/socketio')

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()