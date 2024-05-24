import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import os

# Use absolute paths to avoid issues with relative paths
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
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image = None
        self.running = True

    def send_velocity(self, linear, angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        self.publisher.publish(vel_msg)
        self.get_logger().info(f'Velocity set to: linear={linear}, angular={angular}')

    def handle_stop_service(self, request, response):
        self.send_velocity(0, 0)  # Stop the robot
        response.success = True
        response.message = "Robot has been stopped and the process will terminate."
        self.running = False  # Signal to terminate the node
        return response

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ret, jpeg = cv2.imencode('.jpg', self.image)
        self.image = jpeg.tobytes()

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

    def generate():
        while robot_controller.running:
            if robot_controller.image is not None:
                frame = robot_controller.image
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

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
