from rclpy.node import Node
from rclpy.action import ActionServer
from your_package_name.action import DriveCommand  # replace with your package

import serial
import struct
import time

# --- Serial Configuration ---
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
CMD_START = 0xDE
CMD_END   = 0xBE

# CMD_ID constants (should match Arduino)
CMD_PING = 1
DRIVE_MOTOR = 2
DRIVE_POSITION = 3
MIX_DRIVE = 4
TURN_IN_PLACE = 5
DRIVE_DISTANCE = 6

# --- Serial setup ---
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)  # wait for Arduino reset

class DriveActionServer(Node):

    def __init__(self):
        super().__init__('drive_action_server')
        self._action_server = ActionServer(
            self,
            DriveCommand,
            'drive_command',
            self.execute_callback)
        self.get_logger().info("Drive Action Server Initialized")

    def send_command_to_arduino(self, cmd_id, params):
        """Sends a command to the Arduino over serial using the agreed protocol."""
        num_params = len(params)
        command_bytes = bytearray()
        command_bytes.append(CMD_START)
        command_bytes += struct.pack('<h', cmd_id)
        command_bytes += struct.pack('<h', num_params)
        for p in params:
            command_bytes += struct.pack('<h', p)
        command_bytes.append(CMD_END)

        self.get_logger().info(f"Sending bytes: {list(command_bytes)}")
        ser.write(command_bytes)

        # Optionally read Arduino debug response
        response_lines = []
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self.get_logger().info(f"Arduino: {line}")
                response_lines.append(line)
            else:
                break
        return response_lines

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"Received command: ID={goal.cmd_id}, params={goal.params}")

        # send to Arduino
        try:
            response = self.send_command_to_arduino(goal.cmd_id, goal.params)
            goal_handle.succeed()
            result = DriveCommand.Result()
            result.success = True
            result.message = "\n".join(response)
            return result
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
            goal_handle.abort()
            result = DriveCommand.Result()
            result.success = False
            result.message = str(e)
            return result


def main(args=None):
    rclpy.init(args=args)
    action_server = DriveActionServer()
    rclpy.spin(action_server)
    action_server.destroy()
    ser.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
