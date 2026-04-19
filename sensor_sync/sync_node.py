#!/usr/bin/env python3
"""ROS 2 control node for the Teensy-based RGB synchronizer."""

import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from serial import Serial, SerialException
import serial.tools.list_ports
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger

from sensor_sync.protocol import (
    build_mode_command,
    build_status_summary,
    normalize_sync_mode,
    parse_state_line,
)


class SyncControllerNode(Node):
    """Control the Teensy synchronizer over a serial link."""

    def __init__(self):
        super().__init__("sync_controller")

        self.declare_parameter(
            "port",
            "/dev/ttyACM0",
            ParameterDescriptor(description="Teensy serial port"),
        )
        self.declare_parameter(
            "baud",
            115200,
            ParameterDescriptor(description="Serial baud rate"),
        )
        self.declare_parameter(
            "fps",
            1.0,
            ParameterDescriptor(
                description="Stereo RGB target FPS (1-60 with the current 10 ms pulse width)"
            ),
        )
        self.declare_parameter(
            "sync_mode",
            "gps_pps",
            ParameterDescriptor(
                description="Sync mode: gps_pps (default), internal_timer, external_trigger"
            ),
        )
        self.declare_parameter(
            "auto_connect",
            True,
            ParameterDescriptor(description="Auto connect on startup"),
        )

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.fps = self.get_parameter("fps").value
        self.sync_mode = normalize_sync_mode(self.get_parameter("sync_mode").value)
        self.auto_connect = self.get_parameter("auto_connect").value

        self.serial = None
        self.connected = False
        self.running = False
        self.active_mode = self.sync_mode
        self.device_fps = float(self.fps)
        self.pps_locked = False
        self.pps_age_ms = -1
        self.stereo_frames = 0
        self.pps_count = 0
        self.serial_lock = threading.Lock()

        self.read_thread = None
        self.read_running = False

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.srv_start = self.create_service(Trigger, "~/start", self.start_callback)
        self.srv_stop = self.create_service(Trigger, "~/stop", self.stop_callback)
        self.srv_trigger = self.create_service(Trigger, "~/trigger", self.trigger_callback)
        self.srv_connect = self.create_service(Trigger, "~/connect", self.connect_callback)
        self.srv_disconnect = self.create_service(
            Trigger, "~/disconnect", self.disconnect_callback
        )

        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.pub_fps = self.create_publisher(Float32, "~/current_fps", 10)
        self.pub_mode = self.create_publisher(String, "~/active_mode", 10)
        self.pub_pps_locked = self.create_publisher(Bool, "~/pps_locked", 10)
        self.pub_pps_age_ms = self.create_publisher(Int32, "~/pps_age_ms", 10)

        self.status_timer = self.create_timer(1.0, self.publish_status)

        if self.auto_connect:
            self.try_connect()

        self.get_logger().info("Sync Controller Node started")
        self.get_logger().info(f"  Port: {self.port}")
        self.get_logger().info(f"  Mode: {self.sync_mode}")
        self.get_logger().info(f"  FPS: {self.fps}")

    def find_teensy_port(self):
        """Find a likely Teensy serial port."""
        for port in serial.tools.list_ports.comports():
            if port.vid == 0x16C0 and port.pid == 0x0483:
                return port.device
            if "ttyACM" in port.device or "ttyUSB" in port.device:
                return port.device
        return None

    def try_connect(self):
        """Try to connect to the Teensy and apply current config."""
        if self.connected:
            return True

        port = self.port
        if port == "auto" or not port:
            port = self.find_teensy_port()
            if not port:
                self.get_logger().warn("Teensy not found. Waiting...")
                return False

        try:
            self.serial = Serial(port, self.baud, timeout=0.1)
            time.sleep(2.0)
            self.connected = True
            self.port = port

            self.read_running = True
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

            self.get_logger().info(f"Connected to Teensy on {port}")

            if not self.apply_runtime_config(start_sync=True):
                self.disconnect()
                return False

            return True

        except SerialException as exc:
            self.get_logger().error(f"Failed to connect: {exc}")
            return False

    def disconnect(self):
        """Close the Teensy connection."""
        self.read_running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.send_command("STOP")
            time.sleep(0.1)
            with self.serial_lock:
                self.serial.close()

        self.connected = False
        self.running = False
        self.pps_locked = False
        self.pps_age_ms = -1
        self.serial = None
        self.get_logger().info("Disconnected from Teensy")

    def send_command(self, cmd):
        """Send a command to the Teensy."""
        if not self.connected or not self.serial:
            self.get_logger().warn("Not connected to Teensy")
            return False

        try:
            with self.serial_lock:
                self.serial.write(f"{cmd}\n".encode())
            self.get_logger().debug(f"Sent: {cmd}")
            return True
        except SerialException as exc:
            self.get_logger().error(f"Send failed: {exc}")
            self.connected = False
            return False

    def apply_runtime_config(self, start_sync):
        """Apply current mode/FPS to the Teensy and optionally start it."""
        if not self.send_command(build_mode_command(self.sync_mode)):
            return False
        time.sleep(0.05)

        if not self.send_command(f"FPS {self.fps}"):
            return False
        time.sleep(0.05)

        if start_sync:
            if not self.send_command("START"):
                return False
            self.running = True

        self.request_status()
        return True

    def request_status(self):
        """Ask the Teensy for one structured STATE line."""
        if self.connected:
            self.send_command("STATUS")

    def update_device_state(self, parsed_state):
        """Update cached runtime state from a parsed STATE line."""
        if "mode" in parsed_state:
            self.active_mode = parsed_state["mode"]
        if "running" in parsed_state:
            self.running = bool(parsed_state["running"])
        if "fps" in parsed_state:
            self.device_fps = float(parsed_state["fps"])
        if "pps_locked" in parsed_state:
            self.pps_locked = bool(parsed_state["pps_locked"])
        if "pps_age_ms" in parsed_state:
            self.pps_age_ms = int(parsed_state["pps_age_ms"])
        if "stereo_frames" in parsed_state:
            self.stereo_frames = int(parsed_state["stereo_frames"])
        if "pps_count" in parsed_state:
            self.pps_count = int(parsed_state["pps_count"])

    def serial_read_loop(self):
        """Background serial reader."""
        while self.read_running and self.serial:
            try:
                line = ""
                with self.serial_lock:
                    if self.serial and self.serial.in_waiting:
                        line = self.serial.readline().decode(
                            "utf-8", errors="ignore"
                        ).strip()
                if line:
                    parsed = parse_state_line(line)
                    if parsed is not None:
                        self.update_device_state(parsed)
                        self.get_logger().debug(f"State update: {parsed}")
                    else:
                        self.get_logger().debug(f"Teensy: {line}")
            except SerialException:
                self.connected = False
                break
            except Exception as exc:
                self.get_logger().debug(f"Read error: {exc}")

            time.sleep(0.01)

    def on_parameter_change(self, params):
        """Handle runtime parameter changes."""
        for param in params:
            if param.name == "fps":
                new_fps = param.value
                if not (1.0 <= new_fps <= 60.0):
                    message = f"Invalid FPS: {new_fps} (must be 1-60)"
                    self.get_logger().warn(message)
                    return SetParametersResult(successful=False, reason=message)

                self.fps = new_fps
                if self.connected:
                    if not self.send_command(f"FPS {new_fps}"):
                        return SetParametersResult(
                            successful=False,
                            reason="Failed to update FPS on Teensy",
                        )
                    self.request_status()
                self.get_logger().info(f"FPS changed to {new_fps}")

            elif param.name == "sync_mode":
                try:
                    self.sync_mode = normalize_sync_mode(param.value)
                except ValueError as exc:
                    self.get_logger().warn(str(exc))
                    return SetParametersResult(successful=False, reason=str(exc))

                if self.connected:
                    if not self.send_command(build_mode_command(self.sync_mode)):
                        return SetParametersResult(
                            successful=False,
                            reason="Failed to update sync mode on Teensy",
                        )
                    self.request_status()
                self.get_logger().info(f"Sync mode changed to {self.sync_mode}")

            elif param.name == "port":
                self.port = param.value

        return SetParametersResult(successful=True)

    def start_callback(self, request, response):
        """Start synchronization."""
        if not self.connected and not self.try_connect():
            response.success = False
            response.message = "Not connected to Teensy"
            return response

        self.send_command("START")
        self.running = True
        self.request_status()
        response.success = True
        response.message = f"Sync started in {self.sync_mode} mode at {self.fps} Hz"
        self.get_logger().info(response.message)
        return response

    def stop_callback(self, request, response):
        """Stop synchronization."""
        if not self.connected:
            response.success = False
            response.message = "Not connected to Teensy"
            return response

        self.send_command("STOP")
        self.running = False
        self.request_status()
        response.success = True
        response.message = "Sync stopped"
        self.get_logger().info(response.message)
        return response

    def trigger_callback(self, request, response):
        """Send a single RGB trigger pulse."""
        if not self.connected:
            response.success = False
            response.message = "Not connected to Teensy"
            return response

        self.send_command("TRIGGER")
        self.request_status()
        response.success = True
        response.message = "Single trigger sent"
        return response

    def connect_callback(self, request, response):
        """Connect to the Teensy."""
        if self.connected:
            response.success = True
            response.message = f"Already connected to {self.port}"
            return response

        if self.try_connect():
            response.success = True
            response.message = f"Connected to {self.port}"
        else:
            response.success = False
            response.message = "Failed to connect"
        return response

    def disconnect_callback(self, request, response):
        """Disconnect from the Teensy."""
        self.disconnect()
        response.success = True
        response.message = "Disconnected"
        return response

    def publish_status(self):
        """Publish cached runtime state and poll the firmware."""
        self.request_status()

        status_msg = String()
        status_msg.data = build_status_summary(
            connected=self.connected,
            running=self.running,
            mode=self.active_mode,
            fps=self.device_fps,
            pps_locked=self.pps_locked,
            pps_age_ms=self.pps_age_ms,
        )
        self.pub_status.publish(status_msg)

        fps_msg = Float32()
        fps_msg.data = float(self.device_fps)
        self.pub_fps.publish(fps_msg)

        mode_msg = String()
        mode_msg.data = self.active_mode
        self.pub_mode.publish(mode_msg)

        pps_locked_msg = Bool()
        pps_locked_msg.data = bool(self.pps_locked)
        self.pub_pps_locked.publish(pps_locked_msg)

        pps_age_msg = Int32()
        pps_age_msg.data = int(self.pps_age_ms)
        self.pub_pps_age_ms.publish(pps_age_msg)

    def destroy_node(self):
        """Shutdown hook."""
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SyncControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
