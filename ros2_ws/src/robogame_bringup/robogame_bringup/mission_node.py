"""MissionNode — the real-robot entry point (Raspberry Pi).

Assembles the exact same core stack as the Mac mock mission
(via MissionStack); only the leaves differ:

    clock            RealClock                 (was FakeClock)
    transport        SerialTransport -> STM32  (was MemoryTransport + FakeSTM32)
    marker_detector  CameraMarkerDetector      (was MockMarkerDetector)
    block_detector   CameraBlockDetector       (was MockBlockDetector)

Wiring summary (all shared code):

  sensor_msgs/Image (loc cam)  -> image_to_ndarray -> marker adapter
  sensor_msgs/Image (blk cam)  -> image_to_ndarray -> block adapter
  UART (telemetry)             -> McuClient        -> fusion / watchdog
  mission tick (50 Hz timer)   -> stack.tick()     -> HFSM / chassis cmds
  chassis cmds                 -> McuClient        -> UART

Run:
  ros2 run robogame_bringup mission_node --ros-args \
      -p config_dir:=/home/pi/robogame/config \
      -p field_file:=real_field.yaml
"""
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robogame_interfaces.msg import MissionStatus, Pose2D
from robogame_adapters import (
    CameraBlockDetector,
    CameraMarkerDetector,
    image_to_ndarray,
)
from core.mission.stack import MissionStack
from core.utils.clock import RealClock
from core.utils.config import load_yaml
from core.utils.log import get_logger

log = get_logger("bringup.mission_node")


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("robogame_mission")

        # ------------------------------------------------------------ params
        default_cfg = os.environ.get("ROBOGAME_CONFIG_DIR", "./config")
        self.declare_parameter("config_dir", default_cfg)
        self.declare_parameter("field_file", "real_field.yaml")
        self.declare_parameter("control_period", 0.02)      # 50 Hz
        self.declare_parameter("localization_camera_topic",
                               "/camera_localization/image_raw")
        self.declare_parameter("block_camera_topic",
                               "/camera_block/image_raw")
        self.declare_parameter("publish_period", 1.0)       # status 1 Hz

        config_dir = str(self.get_parameter("config_dir").value)
        field_file = str(self.get_parameter("field_file").value)
        period = float(self.get_parameter("control_period").value)
        loc_topic = str(self.get_parameter("localization_camera_topic").value)
        blk_topic = str(self.get_parameter("block_camera_topic").value)

        # ------------------------------------------------------- perception
        perc_cfg = load_yaml(f"{config_dir}/perception.yaml")
        self.marker_detector = CameraMarkerDetector(
            perc_cfg.get("marker_detector"))
        self.block_detector = CameraBlockDetector(
            perc_cfg.get("block_detector", {}))

        # ----------------------------------------------------------- stack
        # transport=None -> SerialTransport from hardware.yaml
        self.clock = RealClock()
        self.stack = MissionStack(
            config_dir, self.clock,
            marker_detector=self.marker_detector,
            block_detector=self.block_detector,
            field_file=field_file)
        self.stack.machine.start()

        # ------------------------------------------------- ROS interface
        self.create_subscription(
            Image, loc_topic, self._on_loc_frame,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, blk_topic, self._on_block_frame,
            qos_profile_sensor_data)

        self.pose_pub = self.create_publisher(Pose2D, "robogame/pose", 10)
        self.status_pub = self.create_publisher(
            MissionStatus, "robogame/mission_status", 10)

        self._control_timer = self.create_timer(period, self._control_tick)
        self._publish_timer = self.create_timer(
            float(self.get_parameter("publish_period").value),
            self._publish_status)

        self.get_logger().info(
            f"mission node up: config={config_dir} field={field_file} "
            f"period={period}s loc_cam={loc_topic} blk_cam={blk_topic}")

    # -------------------------------------------------------- camera input
    def _on_loc_frame(self, msg: Image) -> None:
        try:
            frame = image_to_ndarray(msg)
        except Exception as exc:
            self.get_logger().warning(f"loc frame dropped: {exc}")
            return
        self.marker_detector.on_frame(frame, self.clock.now())

    def _on_block_frame(self, msg: Image) -> None:
        try:
            frame = image_to_ndarray(msg)
        except Exception as exc:
            self.get_logger().warning(f"block frame dropped: {exc}")
            return
        self.block_detector.on_frame(frame, self.clock.now())

    # -------------------------------------------------------- control loop
    def _control_tick(self) -> None:
        markers = self.marker_detector.detect(self.clock.now())
        outcome = self.stack.tick(markers)
        self._publish_pose()
        if outcome == "DONE" or self.stack.match.finished:
            self.get_logger().info("mission finished — halting")
            self._control_timer.cancel()
            self.stack.shutdown()

    # --------------------------------------------------------- publishing
    def _publish_pose(self) -> None:
        snap = self.stack.store.snapshot()
        msg = Pose2D()
        msg.x = snap.robot.x
        msg.y = snap.robot.y
        msg.yaw = snap.robot.yaw
        msg.confidence = snap.robot.localization_confidence
        msg.velocity_vx = snap.robot.vx
        msg.velocity_vy = snap.robot.vy
        msg.velocity_wz = snap.robot.wz
        self.pose_pub.publish(msg)

    def _publish_status(self) -> None:
        snap = self.stack.store.snapshot()
        msg = MissionStatus()
        msg.hfsm_state = self.stack.machine.full_state_name
        msg.current_task = (snap.mission.current_task.value
                            if snap.mission.current_task else "")
        msg.current_skill = snap.mission.current_skill
        msg.retry_count = snap.mission.retry_count
        msg.orange_carried = snap.inventory.orange_count
        msg.purple_carried = snap.inventory.purple_count
        msg.placed_total = snap.building.placed_total
        msg.tower_names = list(snap.building.tower_heights.keys())
        msg.tower_heights = list(snap.building.tower_heights.values())
        msg.tower_purple_top = list(snap.building.tower_purple_top.values())
        msg.match_started = snap.match.started
        msg.match_finished = snap.match.finished
        msg.elapsed_time = snap.match.elapsed_time
        msg.remaining_time = snap.match.remaining_time
        msg.phase = snap.match.phase.value
        msg.localization_confidence = snap.robot.localization_confidence
        msg.watchdog_healthy = snap.hardware.critical_ok()
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self.stack.shutdown()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
