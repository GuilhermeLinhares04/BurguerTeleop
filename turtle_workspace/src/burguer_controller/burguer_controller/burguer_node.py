#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import curses

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.service = self.create_service(Trigger, 'stop_and_kill', self.handle_stop_service)
        self.vel_msg = Twist()
        self.running = True

    def send_velocity(self, linear, angular):
        self.vel_msg.linear.x = linear
        self.vel_msg.angular.z = angular
        self.publisher.publish(self.vel_msg)
        self.get_logger().info(f'Velocidade definida para: linear={linear}, angular={angular}')

    def handle_stop_service(self, request, response):
        self.send_velocity(0, 0)  # Stop the robot
        response.success = True
        response.message = "O robô foi parado e o processo será terminado."
        self.running = False  # Signal to terminate the node
        return response

def key_listener(win, robot_controller):
    win.nodelay(True)
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)
    key = ""
    while robot_controller.running:
        win.clear()
        win.addstr(0, 0, "Mova o robô com w/a/s/d, 'e' para parar e 'q' para sair", curses.color_pair(1))
        win.addstr(1, 0, f"Velocidade atual - Linear: {robot_controller.vel_msg.linear.x}, Angular: {robot_controller.vel_msg.angular.z}")
        
        try:
            key = win.getkey()
            if key == 'w':
                robot_controller.send_velocity(0.8, 0.0)
            elif key == 's':
                robot_controller.send_velocity(-0.8, 0.0)
            elif key == 'a':
                robot_controller.send_velocity(0.0, 0.8)
            elif key == 'd':
                robot_controller.send_velocity(0.0, -0.8)
            elif key == 'e':
                rclpy.call(robot_controller.service)
            elif key == 'q':
                break
        except Exception as e:
            pass
        win.refresh()

def main():
    rclpy.init()
    robot_controller = RobotController()
    curses.wrapper(key_listener, robot_controller)
    while robot_controller.running:
        rclpy.spin_once(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
