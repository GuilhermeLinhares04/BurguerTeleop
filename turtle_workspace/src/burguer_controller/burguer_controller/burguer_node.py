import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import cv2
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import base64
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.abspath(os.path.join(current_dir, '../web/templates'))
static_dir = os.path.abspath(os.path.join(current_dir, '../web/static'))

app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
socketio = SocketIO(app)

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.service = self.create_service(Trigger, 'stop_and_kill', self.handle_stop_service)
        self.running = True

    def send_velocity(self, linear, angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        self.publisher.publish(vel_msg)
        self.get_logger().info(f'Velocidade definida para: linear={linear}, angular={angular}')

    def handle_stop_service(self, request, response):
        self.send_velocity(0, 0)  # Stop the robot
        response.success = True
        response.message = "O robô foi parado e o processo será terminado."
        self.running = False  # Signal to terminate the node
        return response

def video_stream():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Não foi possível abrir a câmera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Não foi possível ler o frame.")
            continue

        _, buffer = cv2.imencode('.jpg', frame)
        frame = base64.b64encode(buffer).decode('utf-8')
        socketio.emit('video_frame', {'image_data': frame})
        socketio.sleep(0.1)

    cap.release()

def key_listener(robot_controller):
    @app.route('/')
    def index():
        return render_template('index.html')

    @socketio.on('control_robot')
    def handle_robot_control(json):
        if json['action'] == 'forward':
            robot_controller.send_velocity(0.1, 0.0)
        elif json['action'] == 'backward':
            robot_controller.send_velocity(-0.1, 0.0)
        elif json['action'] == 'left':
            robot_controller.send_velocity(0.0, 0.1)
        elif json['action'] == 'right':
            robot_controller.send_velocity(0.0, -0.1)
        elif json['action'] == 'stop':
            robot_controller.send_velocity(0.0, 0.0)
        elif json['action'] == 'emergency_stop':
            rclpy.call(robot_controller.service)

    socketio.start_background_task(target=video_stream)
    socketio.run(app, host='0.0.0.0', port=5000)

def main():
    rclpy.init()
    robot_controller = RobotController()
    threading.Thread(target=key_listener, args=(robot_controller,)).start()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
