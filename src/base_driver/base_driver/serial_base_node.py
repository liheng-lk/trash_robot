# #!/usr/bin/env python3
# import math
# import threading
# import time
# import serial

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist, TransformStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from tf2_ros import TransformBroadcaster


# TX_HEADER = 0x7B
# TX_TAIL = 0x7D
# RX_HEADER = 0x7B
# RX_TAIL = 0x7D
# RX_FRAME_LEN = 24


# def int16_to_bytes_be(value: int):
#     if value < 0:
#         value = (1 << 16) + value
#     return (value >> 8) & 0xFF, value & 0xFF


# def bytes_to_int16_be(high: int, low: int) -> int:
#     value = (high << 8) | low
#     if value & 0x8000:
#         value -= 0x10000
#     return value


# def build_cmd_packet(x_mm_s: int, y_mm_s: int, z_raw: int) -> bytes:
#     xh, xl = int16_to_bytes_be(x_mm_s)
#     yh, yl = int16_to_bytes_be(y_mm_s)
#     zh, zl = int16_to_bytes_be(z_raw)

#     buf = [TX_HEADER, 0x00, 0x00, xh, xl, yh, yl, zh, zl]
#     checksum = 0
#     for b in buf:
#         checksum ^= b
#     buf.append(checksum)
#     buf.append(TX_TAIL)
#     return bytes(buf)


# class SerialBaseNode(Node):
#     def __init__(self):
#         super().__init__('serial_base_node')

#         # ===== Parameters =====
#         self.declare_parameter('port', '/dev/base')
#         self.declare_parameter('baud', 115200)
#         self.declare_parameter('cmd_rate', 20.0)
#         self.declare_parameter('cmd_timeout', 0.3)

#         self.declare_parameter('x_cmd_sign', 1.0)
#         self.declare_parameter('z_cmd_sign', 1.0)

#         self.declare_parameter('x_feedback_sign', 1.0)
#         self.declare_parameter('z_feedback_sign', 1.0)

#         self.declare_parameter('acc_lsb_per_g', 16384.0)
#         self.declare_parameter('gyro_lsb_per_dps', 65.5)

#         # yaw source:
#         # encoder -> only STM32 Z_speed
#         # imu     -> only IMU gyro.z
#         # blend   -> blend encoder + imu
#         self.declare_parameter('yaw_source', 'blend')
#         self.declare_parameter('yaw_blend_alpha', 0.25)   # imu weight
#         self.declare_parameter('imu_yaw_sign', -1.0)

#         # stabilization params
#         self.declare_parameter('imu_lpf_alpha', 0.2)
#         self.declare_parameter('yaw_deadband', 0.01)
#         self.declare_parameter('linear_deadband', 0.005)
#         self.declare_parameter('freeze_yaw_when_stationary', True)

#         port = self.get_parameter('port').value
#         baud = int(self.get_parameter('baud').value)
#         self.cmd_rate = float(self.get_parameter('cmd_rate').value)
#         self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

#         self.x_cmd_sign = float(self.get_parameter('x_cmd_sign').value)
#         self.z_cmd_sign = float(self.get_parameter('z_cmd_sign').value)
#         self.x_feedback_sign = float(self.get_parameter('x_feedback_sign').value)
#         self.z_feedback_sign = float(self.get_parameter('z_feedback_sign').value)

#         self.acc_lsb_per_g = float(self.get_parameter('acc_lsb_per_g').value)
#         self.gyro_lsb_per_dps = float(self.get_parameter('gyro_lsb_per_dps').value)

#         self.yaw_source = str(self.get_parameter('yaw_source').value).strip().lower()
#         self.yaw_blend_alpha = float(self.get_parameter('yaw_blend_alpha').value)
#         self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)

#         self.imu_lpf_alpha = float(self.get_parameter('imu_lpf_alpha').value)
#         self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
#         self.linear_deadband = float(self.get_parameter('linear_deadband').value)
#         self.freeze_yaw_when_stationary = bool(self.get_parameter('freeze_yaw_when_stationary').value)

#         if self.yaw_source not in ('encoder', 'imu', 'blend'):
#             self.get_logger().warn(
#                 f"Invalid yaw_source='{self.yaw_source}', fallback to 'blend'"
#             )
#             self.yaw_source = 'blend'

#         # ===== Serial =====
#         self.ser = serial.Serial(port, baud, timeout=0.02)
#         time.sleep(0.2)
#         self.ser.reset_input_buffer()
#         self.ser.reset_output_buffer()
#         self.get_logger().info(f'Opened serial port {port} @ {baud}')
#         self.get_logger().info(
#             f'yaw_source={self.yaw_source}, '
#             f'yaw_blend_alpha={self.yaw_blend_alpha:.2f}, '
#             f'imu_yaw_sign={self.imu_yaw_sign:.1f}, '
#             f'imu_lpf_alpha={self.imu_lpf_alpha:.2f}, '
#             f'yaw_deadband={self.yaw_deadband:.4f}'
#         )

#         # ===== Command state =====
#         self.cmd_x_mm_s = 0
#         self.cmd_y_mm_s = 0
#         self.cmd_z_raw = 0
#         self.last_cmd_time = time.time()
#         self.timeout_reported = False

#         # ===== Feedback raw =====
#         self.stop_flag = 0

#         self.vx_raw_mm_s = 0
#         self.vy_raw_mm_s = 0
#         self.vz_raw = 0

#         self.ax_raw = 0
#         self.ay_raw = 0
#         self.az_raw = 0

#         self.gx_raw = 0
#         self.gy_raw = 0
#         self.gz_raw = 0

#         self.voltage_v = 0.0

#         # ===== Feedback corrected / SI =====
#         self.vx_m_s = 0.0
#         self.vy_m_s = 0.0

#         # encoder angular velocity from STM32 kinematics
#         self.vz_enc_rad_s = 0.0

#         # imu angular velocity raw/filtered
#         self.gz_rad_s_raw = 0.0
#         self.gz_rad_s = 0.0
#         self.gz_rad_s_filt = 0.0
#         self.imu_filter_initialized = False

#         # selected angular velocity used by odom
#         self.vz_rad_s = 0.0

#         self.ax_m_s2 = 0.0
#         self.ay_m_s2 = 0.0
#         self.az_m_s2 = 0.0
#         self.gx_rad_s = 0.0
#         self.gy_rad_s = 0.0

#         # ===== Odom integration =====
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.last_time = self.get_clock().now()

#         # ===== RX thread =====
#         self.rx_buf = bytearray()
#         self.running = True
#         self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
#         self.rx_thread.start()

#         # ===== ROS interfaces =====
#         self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
#         self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.tx_timer = self.create_timer(1.0 / self.cmd_rate, self.tx_timer_callback)
#         self.pub_timer = self.create_timer(0.02, self.publish_state)  # 50 Hz

#     # =========================
#     # Command handling
#     # =========================
#     def cmd_callback(self, msg: Twist):
#         x_m_s = msg.linear.x * self.x_cmd_sign
#         z_rad_s = msg.angular.z * self.z_cmd_sign

#         self.cmd_x_mm_s = int(x_m_s * 1000.0)
#         self.cmd_y_mm_s = 0
#         self.cmd_z_raw = int(z_rad_s * 1000.0)

#         self.last_cmd_time = time.time()

#     def force_stop(self):
#         self.cmd_x_mm_s = 0
#         self.cmd_y_mm_s = 0
#         self.cmd_z_raw = 0

#     def send_stop_packets(self, count=20, interval=0.03):
#         pkt = build_cmd_packet(0, 0, 0)
#         for _ in range(count):
#             try:
#                 self.ser.write(pkt)
#             except Exception:
#                 pass
#             time.sleep(interval)

#     def tx_timer_callback(self):
#         dt = time.time() - self.last_cmd_time

#         if dt > self.cmd_timeout:
#             self.force_stop()

#             if not self.timeout_reported:
#                 self.get_logger().warn(f'/cmd_vel timeout: {dt:.3f}s, forcing stop')
#                 self.timeout_reported = True

#             try:
#                 pkt = build_cmd_packet(0, 0, 0)
#                 for _ in range(5):
#                     self.ser.write(pkt)
#                     time.sleep(0.01)
#             except Exception as e:
#                 self.get_logger().error(f'serial stop write failed: {e}')
#             return

#         self.timeout_reported = False

#         pkt = build_cmd_packet(self.cmd_x_mm_s, self.cmd_y_mm_s, self.cmd_z_raw)
#         try:
#             self.ser.write(pkt)
#         except Exception as e:
#             self.get_logger().error(f'serial write failed: {e}')

#     # =========================
#     # RX handling
#     # =========================
#     def rx_loop(self):
#         while self.running:
#             try:
#                 n = self.ser.in_waiting
#                 if n > 0:
#                     data = self.ser.read(n)
#                     self.rx_buf.extend(data)
#                     self.extract_frames()
#                 else:
#                     time.sleep(0.005)
#             except Exception as e:
#                 self.get_logger().error(f'rx loop error: {e}')
#                 break

#     def extract_frames(self):
#         while True:
#             if len(self.rx_buf) < RX_FRAME_LEN:
#                 return

#             start = self.rx_buf.find(bytes([RX_HEADER]))
#             if start < 0:
#                 self.rx_buf.clear()
#                 return

#             if start > 0:
#                 del self.rx_buf[:start]

#             if len(self.rx_buf) < RX_FRAME_LEN:
#                 return

#             frame = bytes(self.rx_buf[:RX_FRAME_LEN])

#             if frame[-1] != RX_TAIL:
#                 del self.rx_buf[0]
#                 continue

#             del self.rx_buf[:RX_FRAME_LEN]
#             self.handle_frame(frame)

#     def handle_frame(self, frame: bytes):
#         self.stop_flag = frame[1]

#         self.vx_raw_mm_s = bytes_to_int16_be(frame[2], frame[3])
#         self.vy_raw_mm_s = bytes_to_int16_be(frame[4], frame[5])
#         self.vz_raw = bytes_to_int16_be(frame[6], frame[7])

#         self.ax_raw = bytes_to_int16_be(frame[8], frame[9])
#         self.ay_raw = bytes_to_int16_be(frame[10], frame[11])
#         self.az_raw = bytes_to_int16_be(frame[12], frame[13])

#         self.gx_raw = bytes_to_int16_be(frame[14], frame[15])
#         self.gy_raw = bytes_to_int16_be(frame[16], frame[17])
#         self.gz_raw = bytes_to_int16_be(frame[18], frame[19])

#         voltage_raw = (frame[20] << 8) | frame[21]
#         self.voltage_v = voltage_raw / 1000.0

#         # linear velocity from encoder
#         self.vx_m_s = self.x_feedback_sign * self.vx_raw_mm_s / 1000.0
#         self.vy_m_s = self.vy_raw_mm_s / 1000.0

#         # deadband on tiny linear noise
#         if abs(self.vx_m_s) < self.linear_deadband:
#             self.vx_m_s = 0.0
#         if abs(self.vy_m_s) < self.linear_deadband:
#             self.vy_m_s = 0.0

#         # encoder angular velocity from STM32 vehicle kinematics
#         self.vz_enc_rad_s = self.z_feedback_sign * self.vz_raw / 1000.0


#         # imu unit conversion
#         self.ax_m_s2 = self.ax_raw / self.acc_lsb_per_g * 9.80665
#         self.ay_m_s2 = self.ay_raw / self.acc_lsb_per_g * 9.80665
#         self.az_m_s2 = self.az_raw / self.acc_lsb_per_g * 9.80665

#         self.gx_rad_s = (self.gx_raw / self.gyro_lsb_per_dps) * math.pi / 180.0
#         self.gy_rad_s = (self.gy_raw / self.gyro_lsb_per_dps) * math.pi / 180.0
#         self.gz_rad_s_raw = self.imu_yaw_sign * (self.gz_raw / self.gyro_lsb_per_dps) * math.pi / 180.0

#         # 1st-order low-pass filter for imu gyro.z
#         a = max(0.0, min(1.0, self.imu_lpf_alpha))
#         if not self.imu_filter_initialized:
#             self.gz_rad_s_filt = self.gz_rad_s_raw
#             self.imu_filter_initialized = True
#         else:
#             self.gz_rad_s_filt = a * self.gz_rad_s_raw + (1.0 - a) * self.gz_rad_s_filt

#         self.gz_rad_s = self.gz_rad_s_filt

#         # choose odom yaw source
#         if self.yaw_source == 'encoder':
#             self.vz_rad_s = self.vz_enc_rad_s
#         elif self.yaw_source == 'imu':
#             self.vz_rad_s = self.gz_rad_s
#         else:
#             # blend: encoder as main, imu as auxiliary
#             imu_w = max(0.0, min(1.0, self.yaw_blend_alpha))
#             enc_w = 1.0 - imu_w
#             self.vz_rad_s = enc_w * self.vz_enc_rad_s + imu_w * self.gz_rad_s

#         # yaw deadband to suppress stationary drift
#         if abs(self.vz_rad_s) < self.yaw_deadband:
#             self.vz_rad_s = 0.0

#     # =========================
#     # ROS publish
#     # =========================
#     def publish_state(self):
#         now = self.get_clock().now()
#         dt = (now - self.last_time).nanoseconds * 1e-9
#         if dt <= 0.0:
#             return
#         self.last_time = now

#         # optionally freeze yaw integration when vehicle is effectively stationary
#         if self.freeze_yaw_when_stationary:
#             if abs(self.vx_m_s) < self.linear_deadband and abs(self.vz_rad_s) < self.yaw_deadband:
#                 yaw_rate_for_integration = 0.0
#             else:
#                 yaw_rate_for_integration = self.vz_rad_s
#         else:
#             yaw_rate_for_integration = self.vz_rad_s

#         self.yaw += yaw_rate_for_integration * dt
#         self.x += (self.vx_m_s * math.cos(self.yaw) - self.vy_m_s * math.sin(self.yaw)) * dt
#         self.y += (self.vx_m_s * math.sin(self.yaw) + self.vy_m_s * math.cos(self.yaw)) * dt

#         # ---- IMU message ----
#         imu_msg = Imu()
#         imu_msg.header.stamp = now.to_msg()
#         imu_msg.header.frame_id = 'imu_link'

#         imu_msg.orientation.w = 1.0
#         imu_msg.orientation_covariance = [
#             -1.0, 0.0, 0.0,
#              0.0, -1.0, 0.0,
#              0.0, 0.0, -1.0
#         ]

#         imu_msg.angular_velocity.x = self.gx_rad_s
#         imu_msg.angular_velocity.y = self.gy_rad_s
#         imu_msg.angular_velocity.z = self.gz_rad_s
#         imu_msg.angular_velocity_covariance = [
#             0.02, 0.0, 0.0,
#             0.0, 0.02, 0.0,
#             0.0, 0.0, 0.04
#         ]

#         imu_msg.linear_acceleration.x = self.ax_m_s2
#         imu_msg.linear_acceleration.y = self.ay_m_s2
#         imu_msg.linear_acceleration.z = self.az_m_s2
#         imu_msg.linear_acceleration_covariance = [
#             0.10, 0.0, 0.0,
#             0.0, 0.10, 0.0,
#             0.0, 0.0, 0.20
#         ]
#         self.imu_pub.publish(imu_msg)

#         # ---- Odom message ----
#         odom = Odometry()
#         odom.header.stamp = now.to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
#         odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

#         odom.pose.covariance = [
#             0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.10
#         ]

#         odom.twist.twist.linear.x = self.vx_m_s
#         odom.twist.twist.linear.y = self.vy_m_s
#         odom.twist.twist.linear.z = 0.0
#         odom.twist.twist.angular.x = 0.0
#         odom.twist.twist.angular.y = 0.0
#         odom.twist.twist.angular.z = self.vz_rad_s

#         odom.twist.covariance = [
#             0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.05
#         ]
#         self.odom_pub.publish(odom)

#         # ---- TF ----
#         t = TransformStamped()
#         t.header.stamp = now.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = math.sin(self.yaw / 2.0)
#         t.transform.rotation.w = math.cos(self.yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#     # =========================
#     # Shutdown
#     # =========================
#     def destroy_node(self):
#         self.get_logger().info('Stopping base and closing serial...')
#         self.running = False
#         self.force_stop()
#         self.send_stop_packets(count=20, interval=0.03)
#         try:
#             self.ser.close()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialBaseNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import math
import threading
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


TX_HEADER = 0x7B
TX_TAIL = 0x7D
RX_HEADER = 0x7B
RX_TAIL = 0x7D
RX_FRAME_LEN = 24


def int16_to_bytes_be(value: int):
    if value < 0:
        value = (1 << 16) + value
    return (value >> 8) & 0xFF, value & 0xFF


def bytes_to_int16_be(high: int, low: int) -> int:
    value = (high << 8) | low
    if value & 0x8000:
        value -= 0x10000
    return value


def build_cmd_packet(x_mm_s: int, y_mm_s: int, z_raw: int) -> bytes:
    """
    11-byte command packet:
    [0]  0x7B
    [1]  0x00
    [2]  0x00
    [3]  XH
    [4]  XL
    [5]  YH
    [6]  YL
    [7]  ZH
    [8]  ZL
    [9]  xor checksum of [0:8]
    [10] 0x7D
    """
    xh, xl = int16_to_bytes_be(x_mm_s)
    yh, yl = int16_to_bytes_be(y_mm_s)
    zh, zl = int16_to_bytes_be(z_raw)

    buf = [TX_HEADER, 0x00, 0x00, xh, xl, yh, yl, zh, zl]
    checksum = 0
    for b in buf:
        checksum ^= b
    buf.append(checksum)
    buf.append(TX_TAIL)
    return bytes(buf)


class SerialBaseNode(Node):
    def __init__(self):
        super().__init__('serial_base_node')

        # ===== Parameters =====
        self.declare_parameter('port', '/dev/base')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.3)

        # Host-side command direction correction
        self.declare_parameter('x_cmd_sign', 1.0)
        self.declare_parameter('z_cmd_sign', 1.0)

        # Host-side feedback direction correction
        self.declare_parameter('x_feedback_sign', 1.0)
        self.declare_parameter('z_feedback_sign', 1.0)

        # IMU scale
        self.declare_parameter('acc_lsb_per_g', 16384.0)
        self.declare_parameter('gyro_lsb_per_dps', 65.5)

        # Encoder angular velocity correction scale
        self.declare_parameter('vz_enc_scale', 1.03)

        # yaw source:
        #   encoder -> use STM32 Z_speed only
        #   imu     -> use IMU gyro.z only
        #   blend   -> encoder main + imu assist
        self.declare_parameter('yaw_source', 'encoder')
        self.declare_parameter('yaw_blend_alpha', 0.25)   # imu weight
        self.declare_parameter('imu_yaw_sign', -1.0)

        # filters / deadbands
        self.declare_parameter('imu_lpf_alpha', 0.2)
        self.declare_parameter('yaw_deadband', 0.01)
        self.declare_parameter('linear_deadband', 0.005)
        self.declare_parameter('min_effective_z_cmd', 0.0)
        self.declare_parameter('freeze_yaw_when_stationary', True)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.cmd_rate = float(self.get_parameter('cmd_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.x_cmd_sign = float(self.get_parameter('x_cmd_sign').value)
        self.z_cmd_sign = float(self.get_parameter('z_cmd_sign').value)
        self.x_feedback_sign = float(self.get_parameter('x_feedback_sign').value)
        self.z_feedback_sign = float(self.get_parameter('z_feedback_sign').value)

        self.acc_lsb_per_g = float(self.get_parameter('acc_lsb_per_g').value)
        self.gyro_lsb_per_dps = float(self.get_parameter('gyro_lsb_per_dps').value)

        self.vz_enc_scale = float(self.get_parameter('vz_enc_scale').value)

        self.yaw_source = str(self.get_parameter('yaw_source').value).strip().lower()
        self.yaw_blend_alpha = float(self.get_parameter('yaw_blend_alpha').value)
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)

        self.imu_lpf_alpha = float(self.get_parameter('imu_lpf_alpha').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.min_effective_z_cmd = max(
            0.0, float(self.get_parameter('min_effective_z_cmd').value)
        )
        self.freeze_yaw_when_stationary = bool(
            self.get_parameter('freeze_yaw_when_stationary').value
        )

        if self.yaw_source not in ('encoder', 'imu', 'blend'):
            self.get_logger().warn(
                f"Invalid yaw_source='{self.yaw_source}', fallback to 'encoder'"
            )
            self.yaw_source = 'encoder'

        # ===== Serial =====
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.get_logger().info(f'Opened serial port {port} @ {baud}')
        self.get_logger().info(
            f'yaw_source={self.yaw_source}, '
            f'yaw_blend_alpha={self.yaw_blend_alpha:.2f}, '
            f'vz_enc_scale={self.vz_enc_scale:.3f}, '
            f'imu_yaw_sign={self.imu_yaw_sign:.1f}, '
            f'imu_lpf_alpha={self.imu_lpf_alpha:.2f}, '
            f'yaw_deadband={self.yaw_deadband:.4f}'
        )

        # ===== Command state =====
        self.cmd_x_mm_s = 0
        self.cmd_y_mm_s = 0
        self.cmd_z_raw = 0
        self.last_cmd_time = time.time()
        self.timeout_reported = False

        # ===== Feedback raw =====
        self.stop_flag = 0

        self.vx_raw_mm_s = 0
        self.vy_raw_mm_s = 0
        self.vz_raw = 0  # STM32 vehicle angular speed, *1000

        self.ax_raw = 0
        self.ay_raw = 0
        self.az_raw = 0

        self.gx_raw = 0
        self.gy_raw = 0
        self.gz_raw = 0

        self.voltage_v = 0.0

        # ===== SI values =====
        self.vx_m_s = 0.0
        self.vy_m_s = 0.0

        # encoder angular velocity
        self.vz_enc_rad_s = 0.0

        # imu angular velocity raw / filtered
        self.gz_rad_s_raw = 0.0
        self.gz_rad_s = 0.0
        self.gz_rad_s_filt = 0.0
        self.imu_filter_initialized = False

        # final angular velocity used by odom
        self.vz_rad_s = 0.0

        self.ax_m_s2 = 0.0
        self.ay_m_s2 = 0.0
        self.az_m_s2 = 0.0
        self.gx_rad_s = 0.0
        self.gy_rad_s = 0.0

        # ===== Odom integration =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # ===== RX thread =====
        self.rx_buf = bytearray()
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # ===== ROS interfaces =====
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tx_timer = self.create_timer(1.0 / self.cmd_rate, self.tx_timer_callback)
        self.pub_timer = self.create_timer(0.02, self.publish_state)  # 50 Hz

    # =========================
    # Command handling
    # =========================
    def cmd_callback(self, msg: Twist):
        x_m_s = msg.linear.x * self.x_cmd_sign
        z_rad_s = msg.angular.z * self.z_cmd_sign

        if (
            self.min_effective_z_cmd > 0.0
            and abs(z_rad_s) > 1e-6
            and abs(z_rad_s) < self.min_effective_z_cmd
        ):
            z_rad_s = math.copysign(self.min_effective_z_cmd, z_rad_s)

        self.cmd_x_mm_s = int(x_m_s * 1000.0)
        self.cmd_y_mm_s = 0
        self.cmd_z_raw = int(z_rad_s * 1000.0)

        self.last_cmd_time = time.time()

    def force_stop(self):
        self.cmd_x_mm_s = 0
        self.cmd_y_mm_s = 0
        self.cmd_z_raw = 0

    def send_stop_packets(self, count=20, interval=0.03):
        pkt = build_cmd_packet(0, 0, 0)
        for _ in range(count):
            try:
                self.ser.write(pkt)
            except Exception:
                pass
            time.sleep(interval)

    def tx_timer_callback(self):
        dt = time.time() - self.last_cmd_time

        if dt > self.cmd_timeout:
            self.force_stop()

            if not self.timeout_reported:
                self.get_logger().warn(f'/cmd_vel timeout: {dt:.3f}s, forcing stop')
                self.timeout_reported = True

            try:
                pkt = build_cmd_packet(0, 0, 0)
                for _ in range(5):
                    self.ser.write(pkt)
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'serial stop write failed: {e}')
            return

        self.timeout_reported = False

        pkt = build_cmd_packet(self.cmd_x_mm_s, self.cmd_y_mm_s, self.cmd_z_raw)
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.get_logger().error(f'serial write failed: {e}')

    # =========================
    # RX handling
    # =========================
    def rx_loop(self):
        while self.running:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    self.rx_buf.extend(data)
                    self.extract_frames()
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f'rx loop error: {e}')
                break

    def extract_frames(self):
        while True:
            if len(self.rx_buf) < RX_FRAME_LEN:
                return

            start = self.rx_buf.find(bytes([RX_HEADER]))
            if start < 0:
                self.rx_buf.clear()
                return

            if start > 0:
                del self.rx_buf[:start]

            if len(self.rx_buf) < RX_FRAME_LEN:
                return

            frame = bytes(self.rx_buf[:RX_FRAME_LEN])

            if frame[-1] != RX_TAIL:
                del self.rx_buf[0]
                continue

            del self.rx_buf[:RX_FRAME_LEN]
            self.handle_frame(frame)

    def handle_frame(self, frame: bytes):
        self.stop_flag = frame[1]

        self.vx_raw_mm_s = bytes_to_int16_be(frame[2], frame[3])
        self.vy_raw_mm_s = bytes_to_int16_be(frame[4], frame[5])
        self.vz_raw = bytes_to_int16_be(frame[6], frame[7])

        self.ax_raw = bytes_to_int16_be(frame[8], frame[9])
        self.ay_raw = bytes_to_int16_be(frame[10], frame[11])
        self.az_raw = bytes_to_int16_be(frame[12], frame[13])

        self.gx_raw = bytes_to_int16_be(frame[14], frame[15])
        self.gy_raw = bytes_to_int16_be(frame[16], frame[17])
        self.gz_raw = bytes_to_int16_be(frame[18], frame[19])

        voltage_raw = (frame[20] << 8) | frame[21]
        self.voltage_v = voltage_raw / 1000.0

        # linear velocity from encoder
        self.vx_m_s = self.x_feedback_sign * self.vx_raw_mm_s / 1000.0
        self.vy_m_s = self.vy_raw_mm_s / 1000.0

        # deadband for tiny linear noise
        if abs(self.vx_m_s) < self.linear_deadband:
            self.vx_m_s = 0.0
        if abs(self.vy_m_s) < self.linear_deadband:
            self.vy_m_s = 0.0

        # encoder angular velocity from STM32, with correction scale
        self.vz_enc_rad_s = self.z_feedback_sign * (self.vz_raw / 1000.0) * self.vz_enc_scale

        # imu unit conversion
        self.ax_m_s2 = self.ax_raw / self.acc_lsb_per_g * 9.80665
        self.ay_m_s2 = self.ay_raw / self.acc_lsb_per_g * 9.80665
        self.az_m_s2 = self.az_raw / self.acc_lsb_per_g * 9.80665

        self.gx_rad_s = (self.gx_raw / self.gyro_lsb_per_dps) * math.pi / 180.0
        self.gy_rad_s = (self.gy_raw / self.gyro_lsb_per_dps) * math.pi / 180.0
        self.gz_rad_s_raw = self.imu_yaw_sign * (self.gz_raw / self.gyro_lsb_per_dps) * math.pi / 180.0

        # first-order low-pass filter for gyro.z
        a = max(0.0, min(1.0, self.imu_lpf_alpha))
        if not self.imu_filter_initialized:
            self.gz_rad_s_filt = self.gz_rad_s_raw
            self.imu_filter_initialized = True
        else:
            self.gz_rad_s_filt = a * self.gz_rad_s_raw + (1.0 - a) * self.gz_rad_s_filt

        self.gz_rad_s = self.gz_rad_s_filt

        # select yaw source
        if self.yaw_source == 'encoder':
            self.vz_rad_s = self.vz_enc_rad_s
        elif self.yaw_source == 'imu':
            self.vz_rad_s = self.gz_rad_s
        else:
            imu_w = max(0.0, min(1.0, self.yaw_blend_alpha))
            enc_w = 1.0 - imu_w
            self.vz_rad_s = enc_w * self.vz_enc_rad_s + imu_w * self.gz_rad_s

        # deadband for tiny angular drift
        if abs(self.vz_rad_s) < self.yaw_deadband:
            self.vz_rad_s = 0.0

    # =========================
    # ROS publish
    # =========================
    def publish_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # freeze yaw integration when effectively stationary
        if self.freeze_yaw_when_stationary:
            if abs(self.vx_m_s) < self.linear_deadband and abs(self.vz_rad_s) < self.yaw_deadband:
                yaw_rate_for_integration = 0.0
            else:
                yaw_rate_for_integration = self.vz_rad_s
        else:
            yaw_rate_for_integration = self.vz_rad_s

        self.yaw += yaw_rate_for_integration * dt
        self.x += (self.vx_m_s * math.cos(self.yaw) - self.vy_m_s * math.sin(self.yaw)) * dt
        self.y += (self.vx_m_s * math.sin(self.yaw) + self.vy_m_s * math.cos(self.yaw)) * dt

        # ---- IMU ----
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, -1.0
        ]

        imu_msg.angular_velocity.x = self.gx_rad_s
        imu_msg.angular_velocity.y = self.gy_rad_s
        imu_msg.angular_velocity.z = self.gz_rad_s
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.04
        ]

        imu_msg.linear_acceleration.x = self.ax_m_s2
        imu_msg.linear_acceleration.y = self.ay_m_s2
        imu_msg.linear_acceleration.z = self.az_m_s2
        imu_msg.linear_acceleration_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.20
        ]

        self.imu_pub.publish(imu_msg)

        # ---- Odom ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.10
        ]

        odom.twist.twist.linear.x = self.vx_m_s
        odom.twist.twist.linear.y = self.vy_m_s
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vz_rad_s

        odom.twist.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.odom_pub.publish(odom)

        # ---- TF ----
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # =========================
    # Shutdown
    # =========================
    def destroy_node(self):
        self.get_logger().info('Stopping base and closing serial...')
        self.running = False
        self.force_stop()
        self.send_stop_packets(count=20, interval=0.03)
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()