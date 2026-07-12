#!/usr/bin/env python3

"""
Servo Control Module for 12 Servos.

ROS 2 Humble version.
Requires i2c_pwm_board / i2c_pwm_board_msgs.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from i2c_pwm_board_msgs.msg import Servo, ServoArray, ServoConfig
from i2c_pwm_board_msgs.srv import ServosConfig


NUM_SERVOS = 12

MSG = """
Servo Control Module for 12 Servos.

Enter one of the following options:
-----------------------------
quit: stop and quit the program
oneServo: Move one servo manually, all others will be commanded to their center position
allServos: Move all servos manually together

Keyboard commands for One Servo Control
---------------------------
   q            t   y   u
            f   g       j   k
                b   n   m

  q: Quit current command mode and go back to Option Select
  t: Command servo min value
  y: Command servo center value
  u: Command servo max value
  f: Manually decrease servo command value by 0.10
  g: Manually decrease servo command value by 0.01
  j: Manually increase servo command value by 0.01
  k: Manually increase servo command value by 0.10
  b: Save new min command value
  n: Save new center command value
  m: Save new max command value

CTRL-C to quit
"""


class ServoConvert:
    """
    Encapsulates one servo command value.

    This script uses proportional servo values from -1.0 to 1.0.
    """

    def __init__(self, servo_id=0, center_value=0.0, direction=1):
        self.value = center_value
        self._center = center_value
        self._min = -1.0
        self._max = 1.0
        self._dir = direction
        self.id = servo_id

    def _clamp(self, value):
        return max(-1.0, min(1.0, float(value)))

    def set_value(self, value_in):
        self.value = self._clamp(value_in)

    def set_center(self, center_val):
        self._center = self._clamp(center_val)
        print("Servo %2i center set to %1.2f" % (self.id + 1, self._center))

    def set_max(self, max_val):
        self._max = self._clamp(max_val)
        print("Servo %2i max set to %1.2f" % (self.id + 1, self._max))

    def set_min(self, min_val):
        self._min = self._clamp(min_val)
        print("Servo %2i min set to %1.2f" % (self.id + 1, self._min))


def make_key_dict():
    return {
        "q": None,
        "t": lambda x: x.set_value(x._min),
        "y": lambda x: x.set_value(x._center),
        "u": lambda x: x.set_value(x._max),
        "f": lambda x: x.set_value(x.value - 0.10),
        "g": lambda x: x.set_value(x.value - 0.01),
        "j": lambda x: x.set_value(x.value + 0.01),
        "k": lambda x: x.set_value(x.value + 0.10),
        "b": lambda x: x.set_min(x.value),
        "n": lambda x: x.set_center(x.value),
        "m": lambda x: x.set_max(x.value),
    }


KEY_DICT = make_key_dict()
VALID_CMDS = ["quit", "oneServo", "allServos"]


class SpotMicroServoControl(Node):
    def __init__(self):
        super().__init__("spot_micro_servo_control")

        self.get_logger().info("Setting up the Spot Micro Servo Control Node...")

        self.servos = {}

        for i in range(NUM_SERVOS):
            self.servos[i] = ServoConvert(servo_id=i)

        self.get_logger().info("> Servos correctly initialized")

        self._servo_msg = ServoArray()
        for i in range(NUM_SERVOS):
            servo = Servo()
            servo.servo = i + 1
            servo.value = 0.0
            self._servo_msg.servos.append(servo)

        # Your i2c_pwm_board node launched with argument ['1'] uses suffixed topics.
        self.ros_pub_servo_array = self.create_publisher(
            ServoArray,
            "/servos_proportional_1",
            10,
        )

        self.get_logger().info("> Publisher correctly initialized")

        self.servo_config_client = self.create_client(
            ServosConfig,
            "/config_servos_1",
        )

        self.configure_servos()

        if not sys.stdin.isatty():
            raise RuntimeError(
                "This keyboard node must be run in an interactive terminal. "
                "Use ros2 run, not ros2 launch."
            )

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Initialization complete")

    def configure_servos(self):
        request = ServosConfig.Request()

        for i in range(NUM_SERVOS):
            servo_config = ServoConfig()
            servo_config.center = 300
            servo_config.range = 400
            servo_config.servo = i + 1
            servo_config.direction = 1
            request.servos.append(servo_config)

        self.get_logger().info("> Waiting for /config_servos_1 service...")

        if not self.servo_config_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "> /config_servos_1 service not available. "
                "Make sure i2c_pwm_board is running."
            )
            return

        self.get_logger().info("> /config_servos_1 service found!")

        future = self.servo_config_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed")
            return

        response = future.result()
        print("Config servos done, returned value: %i" % response.error)

    def send_servo_msg(self):
        for _, servo_obj in self.servos.items():
            self._servo_msg.servos[servo_obj.id].servo = servo_obj.id + 1
            self._servo_msg.servos[servo_obj.id].value = servo_obj.value

        self.ros_pub_servo_array.publish(self._servo_msg)

    def reset_all_servos_center(self):
        for servo in self.servos.values():
            servo.value = servo._center

    def get_key(self):
        tty.setraw(sys.stdin.fileno())

        try:
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key

    def print_final_servo_values(self):
        print("Final Servo Values")
        print("--------------------")

        for i in range(NUM_SERVOS):
            print(
                "Servo %2i:   Min: %1.2f,   Center: %1.2f,   Max: %1.2f"
                % (
                    i + 1,
                    self.servos[i]._min,
                    self.servos[i]._center,
                    self.servos[i]._max,
                )
            )

    def prompt_for_servo_number(self):
        while rclpy.ok():
            try:
                user_input = int(
                    input("Which servo to control? Enter a number 1 through 12: ")
                )
            except ValueError:
                print("Invalid servo number entered, try again")
                continue

            if user_input not in range(1, NUM_SERVOS + 1):
                print("Invalid servo number entered, try again")
            else:
                return user_input - 1

        return None

    def run_one_servo_mode(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        servo_index = self.prompt_for_servo_number()

        if servo_index is None:
            return

        print("Enter command, q to go back to option select: ")

        while rclpy.ok():
            user_input = self.get_key()

            if user_input == "q":
                break

            if user_input not in KEY_DICT:
                print("Key not in valid key commands, try again")
                continue

            KEY_DICT[user_input](self.servos[servo_index])
            print(
                "Servo %2i cmd: %1.2f"
                % (servo_index + 1, self.servos[servo_index].value)
            )
            self.send_servo_msg()

    def run_all_servos_mode(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        print("Enter command, q to go back to option select: ")

        while rclpy.ok():
            user_input = self.get_key()

            if user_input == "q":
                break

            if user_input not in KEY_DICT:
                print("Key not in valid key commands, try again")
                continue

            if user_input in ("b", "n", "m"):
                print("Saving values is not supported in all servo control mode")
                continue

            for servo in self.servos.values():
                KEY_DICT[user_input](servo)

            print("All servos commanded")
            self.send_servo_msg()

    def run(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        rate = self.create_rate(10)

        while rclpy.ok():
            print(MSG)
            user_input = input("Command?: ")

            if user_input not in VALID_CMDS:
                print("Valid command not entered, try again...")
                continue

            if user_input == "quit":
                print("Ending program...")
                self.print_final_servo_values()
                break

            if user_input == "oneServo":
                self.run_one_servo_mode()

            elif user_input == "allServos":
                self.run_all_servos_mode()

            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = SpotMicroServoControl()
        node.run()
    except KeyboardInterrupt:
        print("\nCTRL-C received. Exiting.")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()