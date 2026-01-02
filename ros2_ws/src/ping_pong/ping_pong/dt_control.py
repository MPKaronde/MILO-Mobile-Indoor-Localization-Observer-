import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import re
import time
import struct

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

        self.CMD_START = 0xDE        # must match Arduino definition
        self.CMD_END   = 0xBE        # must match Arduino definition

        # --- CONFIGURATION ---
        self.PORT = '/dev/ttyUSB0'   # change to your port (COMx on Windows)
        self.BAUDRATE = 9600

        # --- SETUP SERIAL ---
        self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=1)
        time.sleep(2)  # wait for Arduino to reset
        
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

    def parseMessage(self, msg):
        """
        Parses a message of the form:
        <commandID><numParams><Param1>...<ParamN>

        Converts all values to int and sends them to the Arduino.
        """

        # Extract everything inside < >
        tokens = re.findall(r'<([^<>]+)>', msg)

        if len(tokens) < 2:
            self.get_logger().error("Invalid message: missing commandID or numParams")
            return

        try:
            values = [int(t) for t in tokens]
        except ValueError:
            self.get_logger().error("Invalid message: non-integer value found")
            return

        commandID = values[0]
        numParams = values[1]
        params = values[2:]

        if numParams != len(params):
            self.get_logger().error(
                f"Invalid message: numParams={numParams}, but {len(params)} params provided"
            )
            return

        self.get_logger().info(
            f"Parsed commandID={commandID}, numParams={numParams}, params={params}"
        )

        # Send to Arduino
        self.instruct_arduino(commandID, numParams, params)


    # sends a parsed message to the arduino
    def instruct_arduino(self, commandID, numParams, params):
        command_bytes = bytearray()
        command_bytes.append(self.CMD_START)
        command_bytes += struct.pack('<h', commandID) 
        command_bytes += struct.pack('<h', numParams)
        for p in params:
            command_bytes += struct.pack('<h', p)
        command_bytes += struct.pack('<h', self.CMD_END)
        self.ser.write(command_bytes)
        print("Sent command bytes:", list(command_bytes))

    # parse the command to individual vals
    def parse_command(msg):




def main(args=None):
    rclpy.init(args=args)
    node = DTControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
