import socketio

sio = socketio.Client()

global goal_pose
goal_pose = []

@sio.event
def connect() :
    print('connection')

@sio.event
def disconnect() :
    print('disconnection')

@sio.on('goalPose')
def make_goal_pose(data) :
    global goal_pose
    goal_pose = data

def get_goal_pose() :
    return goal_pose