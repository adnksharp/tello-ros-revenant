#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time

import ament_index_python
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from djitellopy import Tello
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu, Temperature
from std_msgs.msg import Bool, Empty, String

from tello_msg.msg import TelloID, TelloStatus, TelloWifiConfig

# --------------------- Math helpers ---------------------


def euler_to_quaternion(roll, pitch, yaw):
    """(roll, pitch, yaw) in radians -> quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


# --------------------- Tello ROS Node ---------------------


class TelloNode:
    """
    ROS 2 wrapper for Tello using djitellopy.
    - Proper QoS for sensor topics
    - Guards for empty frames
    - Clean shutdown for threads and Tello
    - CameraInfo YAML mapped correctly
    """

    def __init__(self, node: Node):
        self.node = node
        self.running = True

        # ---- Parameters
        self.node.declare_parameter("connect_timeout", 10.0)
        self.node.declare_parameter("tello_ip", "192.168.10.1")
        self.node.declare_parameter("tf_base", "map")
        self.node.declare_parameter("tf_drone", "drone")
        self.node.declare_parameter("tf_pub", False)
        self.node.declare_parameter("camera_info_file", "")

        self.connect_timeout = float(self.node.get_parameter("connect_timeout").value)
        self.tello_ip = str(self.node.get_parameter("tello_ip").value)
        self.tf_base = str(self.node.get_parameter("tf_base").value)
        self.tf_drone = str(self.node.get_parameter("tf_drone").value)
        self.tf_pub = bool(self.node.get_parameter("tf_pub").value)
        self.camera_info_file = str(self.node.get_parameter("camera_info_file").value)

        # ---- Camera info YAML
        self.camera_info_dict = None
        if not self.camera_info_file:
            share_directory = ament_index_python.get_package_share_directory("tello")
            self.camera_info_file = share_directory + "/ost.yaml"
        try:
            with open(self.camera_info_file, "r") as f:
                self.camera_info_dict = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.node.get_logger().warn(
                f"Failed to load camera info YAML '{self.camera_info_file}': {e}"
            )

        # ---- Tello config + connect
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)

        self.connection_status = False
        self.last_connection_check = time.time()
        self.connection_check_interval = 5.0  # Check connection every 5 seconds
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 10

        self.node.get_logger().info("Tello: Connecting to drone")
        self.tello = Tello()
        self.attempt_connection()

        # ---- ROS
        self.setup_publishers()
        self.setup_subscribers()

        # ---- Video + status threads
        self.bridge = CvBridge()
        self.video_thread = self.start_video_capture(rate=1.0 / 30.0)
        self.status_thread = self.start_tello_status(rate=1.0 / 2.0)
        self.odom_thread = self.start_tello_odom(
            rate=1.0 / 2.0
        )  # Further reduced from 5Hz to 2Hz to prevent RViz queue overflow
        self.connection_thread = self.start_connection_monitor(
            rate=1.0 / 5.0
        )  # Monitor connection every 5 seconds

        self.node.get_logger().info("Tello: Driver node ready")

    # ---------------- ROS wiring ----------------

    def sensor_qos(self):
        return QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    def setup_publishers(self):
        sensor_qos = self.sensor_qos()
        default_qos = QoSProfile(depth=10)

        self.pub_image_raw = self.node.create_publisher(Image, "image_raw", default_qos)
        self.pub_camera_info = self.node.create_publisher(
            CameraInfo, "camera_info", sensor_qos
        )
        self.pub_status = self.node.create_publisher(TelloStatus, "status", default_qos)
        self.pub_id = self.node.create_publisher(TelloID, "id", default_qos)
        self.pub_imu = self.node.create_publisher(Imu, "imu", sensor_qos)
        self.pub_battery = self.node.create_publisher(
            BatteryState, "battery", default_qos
        )
        self.pub_temperature = self.node.create_publisher(
            Temperature, "temperature", default_qos
        )
        self.pub_odom = self.node.create_publisher(Odometry, "odom", default_qos)
        self.pub_connection_status = self.node.create_publisher(
            Bool, "connection_status", default_qos
        )

        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)

    def setup_subscribers(self):
        self.sub_emergency = self.node.create_subscription(
            Empty, "emergency", self.cb_emergency, 1
        )
        self.sub_takeoff = self.node.create_subscription(
            Empty, "takeoff", self.cb_takeoff, 1
        )
        self.sub_land = self.node.create_subscription(Empty, "land", self.cb_land, 1)
        self.sub_control = self.node.create_subscription(
            Twist, "control", self.cb_control, 1
        )
        self.sub_flip = self.node.create_subscription(String, "flip", self.cb_flip, 1)
        self.sub_wifi_config = self.node.create_subscription(
            TelloWifiConfig, "wifi_config", self.cb_wifi_config, 1
        )

    # ---------------- Camera Info mapping ----------------

    def make_camera_info_msg(self) -> CameraInfo:
        ci = CameraInfo()
        ci.header.frame_id = self.tf_drone
        d = self.camera_info_dict
        if not d:
            return ci
        try:
            ci.width = int(d.get("image_width", 0))
            ci.height = int(d.get("image_height", 0))
            ci.distortion_model = d.get("distortion_model", "plumb_bob")

            # ROS expects flat lists
            if "distortion_coefficients" in d:
                ci.d = list(d["distortion_coefficients"].get("data", []))
            elif "D" in d:
                ci.d = list(d.get("D", []))

            if "camera_matrix" in d:
                ci.k = list(d["camera_matrix"].get("data", []))
            elif "K" in d:
                ci.k = list(d.get("K", []))

            if "rectification_matrix" in d:
                ci.r = list(d["rectification_matrix"].get("data", []))
            elif "R" in d:
                ci.r = list(d.get("R", []))

            if "projection_matrix" in d:
                ci.p = list(d["projection_matrix"].get("data", []))
            elif "P" in d:
                ci.p = list(d.get("P", []))
        except Exception as e:
            self.node.get_logger().warn(f"CameraInfo parse error: {e}")
        return ci

    # ---------------- Orientation helpers ----------------

    def get_orientation_quaternion(self):
        if self.tello is None:
            return [0.0, 0.0, 0.0, 1.0]
        deg = math.pi / 180.0
        try:
            roll = float(self.tello.get_roll()) * deg
            pitch = float(self.tello.get_pitch()) * deg
            yaw = float(self.tello.get_yaw()) * deg
        except Exception:
            roll = pitch = yaw = 0.0
        return euler_to_quaternion(roll, pitch, yaw)

    # ---------------- Connection Management ----------------

    def attempt_connection(self):
        """Attempt to connect to the Tello drone."""
        try:
            self.node.get_logger().info("Attempting to connect to Tello...")
            self.tello.connect()
            self.tello.streamon()
            self.connection_status = True
            self.reconnect_attempts = 0
            self.node.get_logger().info("Successfully connected to Tello")
        except Exception as e:
            self.node.get_logger().error(f"Failed to connect to Tello: {e}")
            self.connection_status = False
            self.tello = None

    def check_connection(self):
        """Check if the drone is still connected."""
        if self.tello is None:
            self.connection_status = False
            return False

        try:
            # Try to get battery level as a connection test
            self.tello.get_battery()
            self.connection_status = True
            return True
        except Exception as e:
            self.node.get_logger().warn(f"Connection check failed: {e}")
            self.connection_status = False
            return False

    def start_connection_monitor(self, rate=1.0 / 5.0):
        """Start a thread to monitor connection status."""

        def connection_monitor():
            period = rate
            next_ts = time.time()
            while self.running:
                now = time.time()
                if now >= next_ts:
                    # Check connection
                    was_connected = self.connection_status
                    is_connected = self.check_connection()

                    # If we lost connection, try to reconnect
                    if was_connected and not is_connected:
                        self.node.get_logger().warn(
                            "Lost connection to Tello, attempting reconnection..."
                        )
                        self.attempt_connection()

                    # Publish connection status
                    if self.pub_connection_status.get_subscription_count() > 0:
                        msg = Bool()
                        msg.data = self.connection_status
                        self.pub_connection_status.publish(msg)

                    next_ts += period

                sleep = next_ts - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_ts = time.time()

        thread = threading.Thread(target=connection_monitor, daemon=True)
        thread.start()
        return thread

    # ---------------- Threads ----------------

    def start_tello_odom(self, rate=1.0 / 10.0):
        def status_odom():
            period = rate
            next_ts = time.time()
            while self.running:
                now = self.node.get_clock().now().to_msg()

                # TF (optional)
                if self.tf_pub:
                    t = TransformStamped()
                    t.header.stamp = now
                    t.header.frame_id = self.tf_base
                    t.child_frame_id = self.tf_drone
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    try:
                        if self.tello is not None:
                            z = float(self.tello.get_barometer()) / 100.0
                        else:
                            z = 0.0
                    except Exception:
                        z = 0.0
                    t.transform.translation.z = z
                    qx, qy, qz, qw = self.get_orientation_quaternion()
                    t.transform.rotation.x = qx
                    t.transform.rotation.y = qy
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw
                    self.tf_broadcaster.sendTransform(t)

                # IMU
                if self.pub_imu.get_subscription_count() > 0:
                    msg = Imu()
                    msg.header.stamp = now
                    msg.header.frame_id = self.tf_drone
                    try:
                        if self.tello is not None and self.connection_status:
                            msg.linear_acceleration.x = (
                                float(self.tello.get_acceleration_x()) / 100.0
                            )
                            msg.linear_acceleration.y = (
                                float(self.tello.get_acceleration_y()) / 100.0
                            )
                            msg.linear_acceleration.z = (
                                float(self.tello.get_acceleration_z()) / 100.0
                            )
                        else:
                            msg.linear_acceleration.x = 0.0
                            msg.linear_acceleration.y = 0.0
                            msg.linear_acceleration.z = 0.0
                    except Exception:
                        msg.linear_acceleration.x = 0.0
                        msg.linear_acceleration.y = 0.0
                        msg.linear_acceleration.z = 0.0
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw
                    self.pub_imu.publish(msg)

                # Odometry
                if self.pub_odom.get_subscription_count() > 0:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = now
                    odom_msg.header.frame_id = self.tf_base
                    odom_msg.child_frame_id = self.tf_drone
                    odom_msg.pose.pose.orientation.x = qx
                    odom_msg.pose.pose.orientation.y = qy
                    odom_msg.pose.pose.orientation.z = qz
                    odom_msg.pose.pose.orientation.w = qw
                    try:
                        if self.tello is not None and self.connection_status:
                            odom_msg.twist.twist.linear.x = (
                                float(self.tello.get_speed_x()) / 100.0
                            )
                            odom_msg.twist.twist.linear.y = (
                                float(self.tello.get_speed_y()) / 100.0
                            )
                            odom_msg.twist.twist.linear.z = (
                                float(self.tello.get_speed_z()) / 100.0
                            )
                        else:
                            odom_msg.twist.twist.linear.x = 0.0
                            odom_msg.twist.twist.linear.y = 0.0
                            odom_msg.twist.twist.linear.z = 0.0
                    except Exception:
                        odom_msg.twist.twist.linear.x = 0.0
                        odom_msg.twist.twist.linear.y = 0.0
                        odom_msg.twist.twist.linear.z = 0.0
                    self.pub_odom.publish(odom_msg)

                # Rate control
                next_ts += period
                sleep = next_ts - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_ts = time.time()

        thread = threading.Thread(target=status_odom, daemon=True)
        thread.start()
        return thread

    def start_tello_status(self, rate=1.0 / 2.0):
        def status_loop():
            period = rate
            next_ts = time.time()
            while self.running:
                now = self.node.get_clock().now().to_msg()

                # Battery
                if self.pub_battery.get_subscription_count() > 0:
                    msg = BatteryState()
                    msg.header.stamp = now
                    msg.header.frame_id = self.tf_drone
                    try:
                        if self.tello is not None and self.connection_status:
                            level = float(self.tello.get_battery())
                        else:
                            level = 0.0
                    except Exception:
                        level = 0.0
                    msg.percentage = max(
                        0.0, min(1.0, level / 100.0)
                    )  # scale to [0..1]
                    msg.voltage = 3.8
                    msg.design_capacity = 1.1
                    msg.present = True
                    msg.power_supply_technology = (
                        BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
                    )
                    msg.power_supply_status = (
                        BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    )
                    self.pub_battery.publish(msg)

                # Temperature
                if self.pub_temperature.get_subscription_count() > 0:
                    msg = Temperature()
                    msg.header.stamp = now
                    msg.header.frame_id = self.tf_drone
                    try:
                        if self.tello is not None and self.connection_status:
                            msg.temperature = float(self.tello.get_temperature())
                        else:
                            msg.temperature = 0.0
                    except Exception:
                        msg.temperature = 0.0
                    msg.variance = 0.0
                    self.pub_temperature.publish(msg)

                # Connection Status
                if self.pub_connection_status.get_subscription_count() > 0:
                    msg = Bool()
                    msg.data = self.connection_status
                    self.pub_connection_status.publish(msg)

                # Tello Status (custom)
                if self.pub_status.get_subscription_count() > 0:
                    s = TelloStatus()
                    try:
                        if self.tello is not None and self.connection_status:
                            s.acceleration.x = float(self.tello.get_acceleration_x())
                            s.acceleration.y = float(self.tello.get_acceleration_y())
                            s.acceleration.z = float(self.tello.get_acceleration_z())

                            s.speed.x = float(self.tello.get_speed_x())
                            s.speed.y = float(self.tello.get_speed_y())
                            s.speed.z = float(self.tello.get_speed_z())

                            s.pitch = int(self.tello.get_pitch())
                            s.roll = int(self.tello.get_roll())
                            s.yaw = int(self.tello.get_yaw())

                            s.barometer = int(self.tello.get_barometer())
                            s.distance_tof = int(self.tello.get_distance_tof())

                            s.fligth_time = int(self.tello.get_flight_time())
                            s.battery = int(self.tello.get_battery())

                            s.highest_temperature = int(
                                self.tello.get_highest_temperature()
                            )
                            s.lowest_temperature = int(
                                self.tello.get_lowest_temperature()
                            )
                            s.temperature = int(self.tello.get_temperature())

                            s.wifi_snr = int(self.tello.query_wifi_signal_noise_ratio())
                        else:
                            # Set default values when disconnected
                            s.acceleration.x = 0.0
                            s.acceleration.y = 0.0
                            s.acceleration.z = 0.0
                            s.speed.x = 0.0
                            s.speed.y = 0.0
                            s.speed.z = 0.0
                            s.pitch = 0
                            s.roll = 0
                            s.yaw = 0
                            s.barometer = 0
                            s.distance_tof = 0
                            s.fligth_time = 0
                            s.battery = 0
                            s.highest_temperature = 0
                            s.lowest_temperature = 0
                            s.temperature = 0
                            s.wifi_snr = 0
                    except Exception:
                        # Set default values on exception
                        s.acceleration.x = 0.0
                        s.acceleration.y = 0.0
                        s.acceleration.z = 0.0
                        s.speed.x = 0.0
                        s.speed.y = 0.0
                        s.speed.z = 0.0
                        s.pitch = 0
                        s.roll = 0
                        s.yaw = 0
                        s.barometer = 0
                        s.distance_tof = 0
                        s.fligth_time = 0
                        s.battery = 0
                        s.highest_temperature = 0
                        s.lowest_temperature = 0
                        s.temperature = 0
                        s.wifi_snr = 0
                    self.pub_status.publish(s)

                # Tello ID (custom)
                if self.pub_id.get_subscription_count() > 0:
                    tid = TelloID()
                    try:
                        if self.tello is not None and self.connection_status:
                            tid.sdk_version = str(self.tello.query_sdk_version())
                            tid.serial_number = str(self.tello.query_serial_number())
                        else:
                            tid.sdk_version = ""
                            tid.serial_number = ""
                    except Exception:
                        tid.sdk_version = ""
                        tid.serial_number = ""
                    self.pub_id.publish(tid)

                # Camera info
                if self.pub_camera_info.get_subscription_count() > 0:
                    ci = self.make_camera_info_msg()
                    ci.header.stamp = now
                    self.pub_camera_info.publish(ci)

                # Rate control
                next_ts += period
                sleep = next_ts - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_ts = time.time()

        thread = threading.Thread(target=status_loop, daemon=True)
        thread.start()
        return thread

    def start_video_capture(self, rate=1.0 / 30.0):
        def video_capture_thread():
            try:
                period = rate
                next_ts = time.time()
                consecutive_errors = 0
                max_consecutive_errors = 10
                stream_started = False

                while self.running:
                    # Check if tello is available and connected
                    if self.tello is None or not self.connection_status:
                        if stream_started:
                            self.node.get_logger().warn(
                                "Tello disconnected, stopping video stream"
                            )
                            stream_started = False
                        time.sleep(1.0)  # Wait before checking again
                        continue

                    # Start stream if not already started
                    if not stream_started:
                        try:
                            self.tello.streamon()
                            self.node.get_logger().info("Tello: Video stream started")
                            stream_started = True
                            consecutive_errors = 0
                        except Exception as e:
                            self.node.get_logger().error(f"Tello streamon failed: {e}")
                            time.sleep(2.0)
                            continue

                    try:
                        frame_read = self.tello.get_frame_read()
                        frame = frame_read.frame

                        if frame is None:
                            consecutive_errors += 1
                            if consecutive_errors > max_consecutive_errors:
                                self.node.get_logger().warn(
                                    "Too many consecutive frame errors, restarting video stream"
                                )
                                try:
                                    self.tello.streamoff()
                                    time.sleep(1.0)
                                    self.tello.streamon()
                                    consecutive_errors = 0
                                except Exception as e:
                                    self.node.get_logger().error(
                                        f"Failed to restart video stream: {e}"
                                    )
                                continue
                            # quiet skip; avoid spamming log
                            pass
                        else:
                            consecutive_errors = 0
                            if (
                                isinstance(frame, np.ndarray)
                                and frame.ndim == 3
                                and frame.shape[2] == 3
                                and frame.size > 0
                            ):
                                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                                msg.header.stamp = self.node.get_clock().now().to_msg()
                                msg.header.frame_id = self.tf_drone
                                self.pub_image_raw.publish(msg)
                            # else silently skip invalid shapes
                    except Exception as e:
                        consecutive_errors += 1
                        if consecutive_errors > max_consecutive_errors:
                            self.node.get_logger().error(
                                f"Too many video errors ({consecutive_errors}), restarting video stream: {e}"
                            )
                            try:
                                self.tello.streamoff()
                                time.sleep(1.0)
                                self.tello.streamon()
                                consecutive_errors = 0
                            except Exception as e2:
                                self.node.get_logger().error(
                                    f"Failed to restart video stream: {e2}"
                                )
                            continue
                        # Only log every 50th error to reduce spam, but log critical errors immediately
                        if consecutive_errors % 50 == 0:
                            self.node.get_logger().warn(
                                f"Video frame error (count: {consecutive_errors}): {str(e)[:100]}..."
                            )
                        elif "decode_slice_header error" not in str(
                            e
                        ) and "no frame!" not in str(e):
                            # Log non-standard errors immediately
                            self.node.get_logger().warn(f"Unexpected video error: {e}")

                    # Simple rate control
                    next_ts += period
                    sleep = next_ts - time.time()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_ts = time.time()
            except Exception as e:
                self.node.get_logger().error(f"Critical video thread error: {e}")
                # Don't re-raise, just log and exit thread gracefully

        thread = threading.Thread(target=video_capture_thread, daemon=True)
        thread.start()
        return thread  # ---------------- Connection Management ----------------

    def attempt_connection(self):
        """Attempt to connect to the Tello drone"""
        try:
            self.tello.connect()
            self.connection_status = True
            self.reconnect_attempts = 0
            self.node.get_logger().info("Tello: Connected to drone")
            # Publish connection status
            if hasattr(self, "pub_connection_status"):
                msg = Bool()
                msg.data = True
                self.pub_connection_status.publish(msg)
        except Exception as e:
            self.connection_status = False
            self.node.get_logger().error(f"Tello: Failed to connect to drone: {e}")
            self.tello = None
            # Publish connection status
            if hasattr(self, "pub_connection_status"):
                msg = Bool()
                msg.data = False
                self.pub_connection_status.publish(msg)

    def check_connection(self):
        """Check if the drone is still connected"""
        if self.tello is None:
            return False

        try:
            # Try to get battery level as a connection test
            self.tello.get_battery()
            return True
        except Exception:
            return False

    def start_connection_monitor(self, rate=1.0 / 5.0):
        """Start a thread to monitor connection status and attempt reconnection"""

        def connection_monitor_thread():
            period = rate
            next_ts = time.time()

            while self.running:
                current_time = time.time()

                # Check connection status
                was_connected = self.connection_status
                is_connected = self.check_connection()

                if is_connected != was_connected:
                    if is_connected:
                        self.connection_status = True
                        self.node.get_logger().info("Tello: Connection restored")
                        # Publish connection status
                        if hasattr(self, "pub_connection_status"):
                            msg = Bool()
                            msg.data = True
                            self.pub_connection_status.publish(msg)
                    else:
                        self.connection_status = False
                        self.node.get_logger().warn(
                            "Tello: Connection lost, attempting reconnection..."
                        )
                        # Publish connection status
                        if hasattr(self, "pub_connection_status"):
                            msg = Bool()
                            msg.data = False
                            self.pub_connection_status.publish(msg)

                        # Attempt reconnection
                        if self.reconnect_attempts < self.max_reconnect_attempts:
                            self.reconnect_attempts += 1
                            self.node.get_logger().info(
                                f"Tello: Reconnection attempt {self.reconnect_attempts}/{self.max_reconnect_attempts}"
                            )
                            time.sleep(2.0)  # Wait before reconnecting
                            self.attempt_connection()
                        else:
                            self.node.get_logger().error(
                                "Tello: Maximum reconnection attempts reached"
                            )

                # Rate control
                next_ts += period
                sleep = next_ts - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_ts = time.time()

        thread = threading.Thread(target=connection_monitor_thread, daemon=True)
        thread.start()
        return thread

    # ---------------- Shutdown ----------------

    def shutdown(self):
        self.node.get_logger().info("Tello: Shutting down")
        self.running = False
        time.sleep(0.2)  # let loops exit
        if self.tello is not None:
            try:
                # Try to stop video stream gracefully
                self.tello.streamoff()
                self.node.get_logger().info("Tello: Video stream stopped")
            except Exception as e:
                self.node.get_logger().warn(f"Streamoff failed: {e}")
            try:
                # Give some time for video thread to stop
                time.sleep(0.5)
                # Skip tello.end() to avoid thread joining issues in djitellopy
                # The library's end() method tries to join threads from within themselves
                self.node.get_logger().info(
                    "Tello: Skipping tello.end() to avoid thread joining issues"
                )
            except Exception as e:
                self.node.get_logger().warn(f"Tello cleanup warning: {e}")

    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.shutdown()
        rclpy.shutdown()

    # ---------------- Callbacks ----------------

    def cb_emergency(self, _msg: Empty):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring emergency")
            return
        try:
            self.tello.emergency()
        except Exception as e:
            self.node.get_logger().error(f"Emergency failed: {e}")

    def cb_takeoff(self, _msg: Empty):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring takeoff")
            return
        try:
            self.tello.takeoff()
        except Exception as e:
            self.node.get_logger().error(f"Takeoff failed: {e}")

    def cb_land(self, _msg: Empty):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring land")
            return
        try:
            self.tello.land()
        except Exception as e:
            self.node.get_logger().error(f"Land failed: {e}")

    def cb_control(self, msg: Twist):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring control")
            return

        # Map Twist -> send_rc_control(lr, fb, ud, yaw), each in [-100, 100]
        def clamp(v):
            try:
                return int(max(-100, min(100, v)))
            except Exception:
                return 0

        lr = clamp(msg.linear.y)
        fb = clamp(msg.linear.x)
        ud = clamp(msg.linear.z)
        yw = clamp(msg.angular.z)
        try:
            self.tello.send_rc_control(lr, fb, ud, yw)
        except Exception as e:
            self.node.get_logger().error(f"send_rc_control failed: {e}")

    def cb_wifi_config(self, msg: TelloWifiConfig):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring wifi_config")
            return
        try:
            self.tello.set_wifi_credentials(msg.ssid, msg.password)
        except Exception as e:
            self.node.get_logger().error(f"WiFi config failed: {e}")

    def cb_flip(self, msg: String):
        if self.tello is None:
            self.node.get_logger().warn("Tello not connected, ignoring flip")
            return
        try:
            self.tello.flip(msg.data)
        except Exception as e:
            self.node.get_logger().error(f"Flip failed: {e}")


# --------------------- Main ---------------------


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tello")
    drone = TelloNode(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received keyboard interrupt, shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error during spin: {e}")
    finally:
        drone.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
