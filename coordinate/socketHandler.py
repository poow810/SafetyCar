import socketio

sio = socketio.AsyncServer(async_mode='asgi', path='/socket')
socket_app = socketio.ASGIApp(sio)

@sio.event(namespace='/socketio')
async def connect(sid, env) :
    print(str(sid), ' : connect')

@sio.event(namespace='/socketio')
async def disconnect(sid) :
    print(str(sid), ' : disconnect')