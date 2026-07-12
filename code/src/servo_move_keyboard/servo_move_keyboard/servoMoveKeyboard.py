#!/usr/bin/env python3

"""Keyboard control for directly commanding servos in ROS 2 Humble."""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from i2c_pwm_board_msgs.msg import Servo, ServoArray


NUM_SERVOS = 12

MSG = """
Servo Control Module for 12 Servos.

Enter one of the following options:
-----------------------------
quit: stop and quit the program
oneServo: Move one servo manually, all others will be commanded to their center position
allServos: Move all servos manually together

Keyboard commands:
-----------------------------
q: return to menu
z: servo min
y: servo center
x: servo max
f: decrease by 10
g: decrease by 1
j: increase by 1
k: increase by 10
b: save current as min
n: save current as center
m: save current as max
"""


class ServoConvert:
    def __init__(self, servo_id=0, center_value=306.0, direction=1):
        self.value = float(center_value)
        self._center = float(center_value)
        self._min = 83.0
        self._max = 520.0
        self._dir = direction
        self.id = servo_id

    def set_value(self, value_in):
        value_in = float(value_in)

        if value_in < 0.0 or value_in > 4095.0:
            print("Servo value not in range [0, 4095]")
            return

        self.value = value_in

    def set_center(self, center_val):
        center_val = float(center_val)

        if 0.0 <= center_val <= 4095.0:
            self._center = center_val
            print(f"Servo {self.id + 1} center set to {self._center}")

    def set_max(self, max_val):
        max_val = float(max_val)

        if 0.0 <= max_val <= 4095.0:
            self._max = max_val
            print(f"Servo {self.id + 1} max set to {self._max}")

    def set_min(self, min_val):
        min_val = float(min_val)

        if 0.0 <= min_val <= 4095.0:
            self._min = min_val
            print(f"Servo {self.id + 1} min set to {self._min}")


KEY_DICT = {
    "q": None,
    "z": lambda x: x.set_value(x._min),
    "y": lambda x: x.set_value(x._center),
    "x": lambda x: x.set_value(x._max),
    "f": lambda x: x.set_value(x.value - 10.0),
    "g": lambda x: x.set_value(x.value - 1.0),
    "j": lambda x: x.set_value(x.value + 1.0),
    "k": lambda x: x.set_value(x.value + 10.0),
    "b": lambda x: x.set_min(x.value),
    "n": lambda x: x.set_center(x.value),
    "m": lambda x: x.set_max(x.value),
}


class SpotMicroServoControl(Node):
    def __init__(self):
        super().__init__("spot_micro_servo_control")

        self.get_logger().info("Setting up servo keyboard control node")

        self.servos = {
            i: ServoConvert(servo_id=i)
            for i in range(NUM_SERVOS)
        }

        self._servo_msg = ServoArray()

        for i in range(NUM_SERVOS):
            servo = Servo()
            servo.servo = i + 1
            servo.value = 0.0
            self._servo_msg.servos.append(servo)

        self.ros_pub_servo_array = self.create_publisher(
            ServoArray,
            "/servos_absolute_1",
            10,
        )

        if not sys.stdin.isatty():
            raise RuntimeError(
                "This keyboard node must be run in an interactive terminal. "
                "Use: ros2 run servo_move_keyboard servo_move_keyboard_node"
            )

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Servo keyboard control node ready")

    def send_servo_msg(self):
        for servo_obj in self.servos.values():
            index = servo_obj.id
            self._servo_msg.servos[index].servo = index + 1
            self._servo_msg.servos[index].value = float(servo_obj.value)

        self.ros_pub_servo_array.publish(self._servo_msg)

    def reset_all_servos_center(self):
        for servo in self.servos.values():
            servo.value = float(servo._center)

    def reset_all_servos_off(self):
        for servo in self.servos.values():
            servo.value = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())

        try:
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings,
            )

        return key

    def prompt_for_servo_number(self):
        while rclpy.ok():
            try:
                servo_number = int(
                    input("Which servo to control? Enter a number 1 through 12: ")
                )
            except ValueError:
                print("Invalid servo number entered, try again")
                continue

            if servo_number < 1 or servo_number > NUM_SERVOS:
                print("Invalid servo number entered, try again")
                continue

            return servo_number - 1

        return None

    def run_one_servo_mode(self):
        self.reset_all_servos_off()
        self.send_servo_msg()

        servo_index = self.prompt_for_servo_number()

        if servo_index is None:
            return

        print("Enter command, q to go back to option select: ")

        while rclpy.ok():
            key = self.get_key()

            if key == "q":
                break

            if key not in KEY_DICT:
                continue

            KEY_DICT[key](self.servos[servo_index])
            print(
                f"Servo {servo_index + 1} command: "
                f"{self.servos[servo_index].value}"
            )
            self.send_servo_msg()

    def run_all_servos_mode(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        print("Enter command, q to go back to option select: ")

        while rclpy.ok():
            key = self.get_key()

            if key == "q":
                break

            if key not in KEY_DICT:
                continue

            if key in ("b", "n", "m"):
                print("Saving min/center/max is disabled in allServos mode")
                continue

            for servo in self.servos.values():
                KEY_DICT[key](servo)

            print("All servos commanded")
            self.send_servo_msg()

    def run(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        while rclpy.ok():
            print(MSG)
            user_input = input("Command?: ")

            if user_input == "quit":
                break

            if user_input == "oneServo":
                self.run_one_servo_mode()

            elif user_input == "allServos":
                self.run_all_servos_mode()

            else:
                print("Invalid command")


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