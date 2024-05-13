import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist

class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.create_timer(0.5, self.setup_turtle)

    def setup_turtle(self):
        self.get_logger().info('Spawning turtle')
        spawn_req = Spawn.Request()
        spawn_req.x = 5.0
        spawn_req.y = 5.0
        spawn_req.theta = 0.0
        spawn_req.name = 'turtle1'
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(self.on_turtle_spawned)

    def on_turtle_spawned(self, future):
        spawn_response = future.result()
        if spawn_response.name:
            self.get_logger().info(f'Turtle spawned with name: {spawn_response.name}')
            self.set_pen()

    def set_pen(self):
        set_pen_req = SetPen.Request()
        set_pen_req.r = 255
        set_pen_req.g = 0
        set_pen_req.b = 0
        set_pen_req.width = 3
        set_pen_req.off = 0
        self.pen_client.call_async(set_pen_req)
        self.draw_square()

    def draw_square(self):
        move_cmd = Twist()
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        self.get_logger().info('Drawing square')
        self.create_timer(1, self.turn_corner)

    def turn_corner(self):
        turn_cmd = Twist()
        turn_cmd.angular.z = 1.57  # Approximately 90 degrees in radians
        self.publisher.publish(turn_cmd)
        self.create_timer(1, self.stop_moving)

    def stop_moving(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0
        stop_cmd.angular.z = 0
        self.publisher.publish(stop_cmd)
        self.remove_turtle()

    def remove_turtle(self):
        kill_req = Kill.Request()
        kill_req.name = 'turtle1'
        self.kill_client.call_async(kill_req)
        self.get_logger().info('Turtle removed')

def main(args=None):
    rclpy.init(args=args)
    turtle_drawer = TurtleDrawer()
    rclpy.spin(turtle_drawer)
    turtle_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
