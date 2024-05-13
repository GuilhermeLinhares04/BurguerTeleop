#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.client_spawn = self.create_client(Spawn, 'spawn')
        while not self.client_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço.')
        self.client_kill = self.create_client(Kill, 'kill')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(10.0, self.move_square)
        self.spawn_turtle('turtle1', 5.5, 5.5, 0.0)
    
    def spawn_turtle(self, name, x, y, theta):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = name
        self.future = self.client_spawn.call_async(req)
    
    def move_square(self):
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        self.get_logger().info('Movendo a tartaruga')
        self.timer.reset()

    def kill_turtle(self, name):
        kill = Kill.Request()
        kill.name = name
        self.client_kill.call_async(kill)

def main(args=None):
    rclpy.init(args=args)
    turtle_square = TurtleSquare()
    rclpy.spin(turtle_square)
    turtle_square.kill_turtle('turtle1')
    turtle_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
