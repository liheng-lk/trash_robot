import json
import logging
import math
import queue
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial import SerialException
import serial
from std_msgs.msg import Float32

SERIAL_PORT_DEFAULT = "/dev/roarm"
BAUD_RATE_DEFAULT = 115200
SERVO_INIT_CMD = {"T": 605, "cmd": 0}
JOINT_CONTROL_CMD = 102
POSE_QUERY_CMD = 105
LED_CONTROL_CMD = 114
POSE_FEEDBACK_CMD = 1051


class ReadLine:
    def __init__(self, s, timeout=1.0):
        self.buf = bytearray()
        self.s = s
        self.timeout = timeout

    def readline(self):
        end_time = time.monotonic() + self.timeout
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[: i + 1]
            self.buf = self.buf[i + 1 :]
            return r

        while time.monotonic() < end_time:
            in_waiting = self.s.in_waiting
            read_size = max(1, min(512, in_waiting if in_waiting > 0 else 1))
            data = self.s.read(read_size)
            if not data:
                continue

            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[: i + 1]
                self.buf[:] = data[i + 1 :]
                return r
            self.buf.extend(data)

        raise TimeoutError("Timed out waiting for newline on serial port")

    def clear_buffer(self):
        self.buf.clear()
        self.s.reset_input_buffer()


class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger("BaseController")
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=0.2)
        self.rl = ReadLine(self.ser, timeout=0.5)
        self.command_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.command_thread = threading.Thread(
            target=self.process_commands,
            daemon=True,
        )
        self.command_thread.start()
        self.data_buffer = None
        self.base_data = {
            "T": POSE_FEEDBACK_CMD,
            "x": 0,
            "y": 0,
            "z": 0,
            "b": 0,
            "s": 0,
            "e": 0,
            "t": 0,
            "torB": 0,
            "torS": 0,
            "torE": 0,
            "torH": 0,
        }

    def close(self):
        self.stop_event.set()
        self.command_queue.put(None)
        if self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def feedback_data(self):
        line = ""
        try:
            line = self.rl.readline().decode("utf-8").strip()
            if not line:
                return None
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except TimeoutError:
            return None
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()
        return None

    def on_data_received(self):
        try:
            data_read = json.loads(self.rl.readline().decode("utf-8"))
            return data_read
        except Exception as e:
            self.logger.error(f"[base_ctrl.on_data_received] unexpected error: {e}")
            return None

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while not self.stop_event.is_set():
            data = self.command_queue.get()
            if data is None:
                break
            try:
                self.ser.write((json.dumps(data) + "\n").encode("utf-8"))
            except SerialException as e:
                self.logger.error(f"[base_ctrl.process_commands] serial write failed: {e}")
            except Exception as e:
                self.logger.error(f"[base_ctrl.process_commands] unexpected error: {e}")

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)


class RoarmDriver(Node):
    def __init__(self):
        super().__init__("roarm_driver")

        self.declare_parameter("serial_port", SERIAL_PORT_DEFAULT)
        self.declare_parameter("baud_rate", BAUD_RATE_DEFAULT)
        self.declare_parameter("joint_command_min_interval_s", 0.08)
        self.declare_parameter("joint_command_deadband_rad", 0.003)

        self.serial_lock = threading.Lock()
        self.last_joint_command_time = 0.0
        self.last_joint_payload = None
        self.last_pose_query_time = 0.0
        self.last_led_command = None
        self.feedback_controller = None

        serial_port_name = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.joint_command_min_interval_s = (
            self.get_parameter("joint_command_min_interval_s").get_parameter_value().double_value
        )
        self.joint_command_deadband_rad = (
            self.get_parameter("joint_command_deadband_rad").get_parameter_value().double_value
        )

        try:
            self.serial_port = serial.Serial(serial_port_name, baud_rate, timeout=0.2)
            self.rl = ReadLine(self.serial_port, timeout=0.5)
            self.get_logger().info(f"Opened {serial_port_name} at {baud_rate} baud.")

            self._write_json(SERVO_INIT_CMD)
            time.sleep(0.1)
        except SerialException as e:
            self.get_logger().error(f"{serial_port_name}: {e}")
            self.serial_port = None
            return

        self.joint_states_sub = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )
        self.pose_sub = self.create_subscription(Pose, "hand_pose", self.pose_callback, 10)
        self.led_ctrl_sub = self.create_subscription(Float32, "led_ctrl", self.led_ctrl_callback, 10)

    def destroy_node(self):
        if self.feedback_controller is not None:
            self.feedback_controller.close()
            self.feedback_controller = None
        if getattr(self, "serial_port", None) and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

    def _write_json(self, payload):
        if not self.serial_port or not self.serial_port.is_open:
            raise SerialException("Serial port is not open")
        data = (json.dumps(payload) + "\n").encode("utf-8")
        with self.serial_lock:
            self.serial_port.write(data)

    def _get_joint_index(self, names, target):
        try:
            return names.index(target)
        except ValueError:
            return None

    def joint_states_callback(self, msg):
        now = time.monotonic()

        name = msg.name
        position = msg.position

        idx_base = self._get_joint_index(name, "base_link_to_link1")
        idx_shoulder = self._get_joint_index(name, "link1_to_link2")
        idx_elbow = self._get_joint_index(name, "link2_to_link3")
        idx_hand = self._get_joint_index(name, "link3_to_gripper_link")

        if None in (idx_base, idx_shoulder, idx_elbow, idx_hand):
            self.get_logger().warn("JointState is missing expected joint names")
            return

        try:
            payload = {
                "T": JOINT_CONTROL_CMD,
                "base": -position[idx_base],
                "shoulder": -position[idx_shoulder],
                "elbow": position[idx_elbow],
                "hand": math.pi - position[idx_hand],
                "spd": 0,
                "acc": 10,
            }
            command = (
                payload["base"],
                payload["shoulder"],
                payload["elbow"],
                payload["hand"],
            )
            if self.last_joint_payload is not None:
                max_delta = max(abs(a - b) for a, b in zip(command, self.last_joint_payload))
                if max_delta < self.joint_command_deadband_rad:
                    return

            if now - self.last_joint_command_time < self.joint_command_min_interval_s:
                return

            self._write_json(payload)
            self.last_joint_payload = command
            self.last_joint_command_time = now
        except SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in joint_states_callback: {e}")

    def _ensure_feedback_controller(self):
        if self.feedback_controller is None:
            if not self.serial_port:
                raise SerialException("Serial port is not available")
            self.feedback_controller = BaseController(
                self.serial_port.port,
                self.serial_port.baudrate,
            )

    def pose_callback(self, msg):
        del msg
        now = time.monotonic()
        if now - self.last_pose_query_time < 0.1:
            return
        self.last_pose_query_time = now

        try:
            self._write_json({"T": POSE_QUERY_CMD})
            self._ensure_feedback_controller()
            feedback = self.feedback_controller.feedback_data()

            if not feedback or feedback.get("T") != POSE_FEEDBACK_CMD:
                return

            feedback = dict(feedback)
            feedback["x"] /= 1000
            feedback["y"] /= 1000
            feedback["z"] /= 1000

            if any(float(feedback[key]) != 0.0 for key in ("x", "y", "z")):
                self.get_logger().info(f"Received feedback from serial port: {feedback}")
        except TimeoutError:
            self.get_logger().warn("Timed out waiting for pose feedback")
        except SerialException as e:
            self.get_logger().error(f"Error communicating with serial port: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected pose callback error: {e}")

    def led_ctrl_callback(self, msg):
        led_value = int(msg.data)
        if self.last_led_command == led_value:
            return
        self.last_led_command = led_value

        try:
            self._write_json({"T": LED_CONTROL_CMD, "led": led_value})
        except SerialException as e:
            self.get_logger().error(f"LED control write failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in led_ctrl_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    roarm_driver = RoarmDriver()

    try:
        if getattr(roarm_driver, "serial_port", None) and roarm_driver.serial_port.is_open:
            rclpy.spin(roarm_driver)
    finally:
        roarm_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
