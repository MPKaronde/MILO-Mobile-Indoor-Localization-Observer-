import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MainControl(Node):
    def __init__(self):
        super().__init__('main_control')
        
        # Publish messages to dt_control
        self.pub = self.create_publisher(String, '/to_dt_control', 10)
        
        # Subscribe to responses from dt_control
        self.sub = self.create_subscription(
            String,
            '/to_main_control',
            self.receive_response,
            10
        )
        self.get_logger().info('Main control ready!')

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: "{message}"')

    def receive_response(self, msg):
        self.get_logger().info(f'Received response: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MainControl()
    
    count = 1
    try:
        while True:
            node.send_message(f'ping #{count}')
            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
