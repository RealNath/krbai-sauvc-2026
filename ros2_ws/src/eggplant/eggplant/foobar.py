import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Movement Controller Node has been started.')
        self.state_timer = 0

    def timer_callback(self):
        msg = Twist()
        # Keep robot underwater
        # msg.linear.z = -0.5

        # Sequence logic (timer_period = 0.1s)
        # 1. Go straight (0 - 5s) -> 0 - 50 ticks
        if self.state_timer < 50:
            msg.linear.x = 0.5
            self.get_logger().info('State: Moving Forward 1', throttle_duration_sec=1)
            
        # 2. Turn left 90 deg (5s - 8s) -> 50 - 80 ticks
        # Assuming angular.z = 0.5 rad/s, 90 deg (1.57 rad) takes ~3.14s.
        elif self.state_timer < 80:
            msg.angular.z = 0.5
            self.get_logger().info('State: Turning Left', throttle_duration_sec=1)

        # 3. Move ahead (8s - 13s) -> 80 - 130 ticks
        elif self.state_timer < 130:
            msg.linear.x = 0.5
            self.get_logger().info('State: Moving Forward 2', throttle_duration_sec=1)

        # 4. Turn right 90 deg (13s - 16s) -> 130 - 160 ticks
        elif self.state_timer < 160:
            msg.angular.z = -0.5
            self.get_logger().info('State: Turning Right', throttle_duration_sec=1)

        # 5. Move ahead (16s - 21s) -> 160 - 210 ticks
        elif self.state_timer < 210:
            msg.linear.x = 0.5
            self.get_logger().info('State: Moving Forward 3', throttle_duration_sec=1)
            
        # Stop
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # msg.linear.z = 0.0
            self.get_logger().info('State: Stopped', throttle_duration_sec=1)

        self.publisher_.publish(msg)
        self.state_timer += 1

def main(args=None):
    rclpy.init(args=args)
    movement_controller = MovementController()
    rclpy.spin(movement_controller)
    movement_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
