import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DTControl(Node):
    def __init__(self):
        super().__init__('dt_control')
        
        # Subscribe to messages from main_control
        self.sub = self.create_subscription(
            String,
            '/to_dt_control',
            self.receive_message,
            10
        )
        
        # Optional: publish responses back to main_control
        self.pub = self.create_publisher(String, '/to_main_control', 10)
        
        self.get_logger().info('DT control ready!')

    def receive_message(self, msg):
        self.get_logger().info(f'Received from main_control: "{msg.data}"')
        
        # Send a response
        response = String()
        response.data = f'pong to "{msg.data}"'
        self.pub.publish(response)
        self.get_logger().info(f'Sent response: "{response.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = DTControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
