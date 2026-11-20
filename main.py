#!/usr/bin/env python3
"""Stream Astribot sensors and optionally record RGB video and LiDAR clouds.

The robot-side Astribot processes must already be running.  This script
connects through the SDK, activates cameras,
decodes their compressed RGB streams, publishes RGB and depth sensor_msgs/Image
topics, and creates an RGB-colored sensor_msgs/PointCloud2 topic for RViz2.

Run after sourcing ROS 2 and this SDK's environment:

    python3 main.py
    python3 main.py --save
    python3 main.py --debug-save
    python3 main.py --lidar-only --debug-save
    python3 main.py --save --camera-clouds

With --save, recordings are written to recordings/YY-MM-DD-HH-MM-SS/.
Press q to finish recording while streaming continues; press Ctrl-C to quit.
Only RGB images and the two LiDAR point clouds are recorded. Use --depth to
publish depth images; they are not saved. Camera clouds require --camera-clouds and a subscriber.
LiDAR format version 2 stores payloads in chunks; each JSONL index entry gives
the chunk filename, byte offset, length, header, and PointCloud2 schema.

Published topics:
    /sdk/astribot_camera/head_rgbd/color_image
    /sdk/astribot_camera/head_rgbd/depth_image
    /sdk/astribot_camera/head_rgbd/rgb_pointcloud
    /sdk/astribot_camera/torso_rgbd/color_image
    /sdk/astribot_camera/torso_rgbd/depth_image
    /sdk/astribot_camera/torso_rgbd/rgb_pointcloud
    /sdk/astribot_camera/head_stereo/left_image
    /sdk/astribot_camera/head_stereo/right_image
    /sdk/astribot_camera/left_wrist_rgbd/color_image
    /sdk/astribot_camera/right_wrist_rgbd/color_image
    /sdk/astribot_lidar/front_scan
    /sdk/astribot_lidar/back_scan

The robot processes are assumed to be running already. The script warns when
an expected sensor stream has been silent for more than five seconds.
"""

import argparse
import csv
from datetime import datetime
import json
import logging
from pathlib import Path
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from rclpy.logging import LoggingSeverity

from astribot_sdk.core.astribot_api.astribot_client import Astribot
from astribot_sdk.core.astribot_api.astribot_interface import AstribotInterface


CAMERAS = {
    "head_rgbd": {
        "color": "/sdk/astribot_camera/head_rgbd/color_image",
        "depth": "/sdk/astribot_camera/head_rgbd/depth_image",
        "cloud": "/sdk/astribot_camera/head_rgbd/rgb_pointcloud",
    },
    "torso_rgbd": {
        "color": "/sdk/astribot_camera/torso_rgbd/color_image",
        "depth": "/sdk/astribot_camera/torso_rgbd/depth_image",
        "cloud": "/sdk/astribot_camera/torso_rgbd/rgb_pointcloud",
    },
    # head_stereo is the face-facing camera and has no depth stream.
    "head_stereo": {
        "stereo": {
            "left": "/sdk/astribot_camera/head_stereo/left_image",
            "right": "/sdk/astribot_camera/head_stereo/right_image",
        },
    },
    "left_wrist_rgbd": {
        "color": "/sdk/astribot_camera/left_wrist_rgbd/color_image",
        "depth": "/sdk/astribot_camera/left_wrist_rgbd/depth_image",
        "cloud": "/sdk/astribot_camera/left_wrist_rgbd/rgb_pointcloud",
    },
    "right_wrist_rgbd": {
        "color": "/sdk/astribot_camera/right_wrist_rgbd/color_image",
        "depth": "/sdk/astribot_camera/right_wrist_rgbd/depth_image",
        "cloud": "/sdk/astribot_camera/right_wrist_rgbd/rgb_pointcloud",
    },
}

OUTPUT_WIDTH = 320
OUTPUT_HEIGHT = 240

# Replace these with calibrated intrinsics at INTRINSICS_WIDTH x INTRINSICS_HEIGHT.
INTRINSICS = {
    "head_rgbd": (525.0, 525.0, 319.5, 239.5),
    "torso_rgbd": (525.0, 525.0, 319.5, 239.5),
    "left_wrist_rgbd": (525.0, 525.0, 319.5, 239.5),
    "right_wrist_rgbd": (525.0, 525.0, 319.5, 239.5),
}
DEPTH_SCALE_M = 0.001  # uint16 depth values are millimetres
DATA_TIMEOUT_SEC = 5.0
SENSOR_LOG_INTERVAL_SEC = 1.0
LIDAR_TOPICS = {
    "front": ("/livox/lidar_front", "/sdk/astribot_lidar/front_scan"),
    "back": ("/livox/lidar_back", "/sdk/astribot_lidar/back_scan"),
}

VIDEO_FPS = 30.0  # MP4 playback rate; timestamps.csv contains the actual frame times.
RECORD_QUEUE_SIZE = 32
INDEX_FLUSH_SEC = 1.0
LIDAR_CHUNK_BYTES = 64 * 1024 * 1024
INTRINSICS_WIDTH, INTRINSICS_HEIGHT = 640, 480
CLOUD_SYNC_NS = 50_000_000
RECORDINGS_DIR = Path(__file__).resolve().parent / "recordings"


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class SensorRecorder:
    """Write received frames on a worker thread so ROS callbacks stay responsive."""

    def __init__(self, debug_only=False, cameras=None):
        self.debug_only = debug_only
        self.debug_queued = set()
        self.started_monotonic = time.monotonic()
        started = datetime.now().astimezone()
        self.folder = RECORDINGS_DIR / started.strftime("%y-%m-%d-%H-%M-%S")
        self.folder.mkdir(parents=True, exist_ok=False)
        self.queues = {kind: queue.Queue(maxsize=RECORD_QUEUE_SIZE) for kind in ("rgb", "lidar")}
        self.queue_high_water = {kind: 0 for kind in self.queues}
        self.lidar_files = {}
        self.last_drop_warning = 0.0
        self.state_lock = threading.Lock()
        self.active = True
        self.dropped = {}
        self.errors = []
        self.videos = {}
        self.index_files = {}
        self.counts = {}
        self.manifest = {
            "format_version": 2,
            "mode": "debug_snapshots" if debug_only else "recording",
            "lidar_storage": "chunked binary; index file/offset/length identify each payload",
            "started_at": started.isoformat(),
            "status": "recording",
            "timestamp_units": "Unix nanoseconds; stamp_ns comes from the ROS header",
            "video": {"codec": "mp4v", "fps": VIDEO_FPS, "width": OUTPUT_WIDTH,
                      "height": OUTPUT_HEIGHT, "timestamps_authoritative": True},
            "rgb_streams": {},
            "lidar_streams": {},
        }
        for camera, spec in (CAMERAS if cameras is None else cameras).items():
            streams = spec.get("stereo", {"color": spec.get("color")})
            for stream, topic in streams.items():
                key = f"{camera}_{stream}"
                self.manifest["rgb_streams"][key] = {
                    "source_topic": f"/astribot_camera/{camera}/color_compress/compressed",
                    "source_view": stream,
                    "topic": topic,
                    "video": f"rgb/{key}.mp4",
                    "index": f"rgb/{key}_timestamps.csv",
                    "encoding": "bgr8",
                }
                if debug_only:
                    spec = self.manifest["rgb_streams"][key]
                    spec.pop("video")
                    spec["image"] = f"rgb/{key}.png"
                    spec["index"] = f"rgb/{key}.json"
                self.counts[key] = 0
        for name, (input_topic, output_topic) in LIDAR_TOPICS.items():
            self.manifest["lidar_streams"][name] = {
                "source_topic": input_topic,
                "topic": output_topic,
                "data_dir": f"lidar/{name}",
                "index": f"lidar/{name}_index.jsonl",
                "type": "sensor_msgs/msg/PointCloud2",
            }
            self.counts[name] = 0
        if debug_only:
            self.manifest.pop("video")
        self._write_manifest()
        self.workers = [
            threading.Thread(target=self._run_writer, args=(kind,), daemon=True, name=f"record-{kind}")
            for kind in self.queues
        ]
        for worker in self.workers:
            worker.start()

    def _write_manifest(self):
        target = self.folder / "manifest.json"
        temporary = self.folder / "manifest.json.tmp"
        temporary.write_text(json.dumps(self.manifest, indent=2) + "\n", encoding="utf-8")
        temporary.replace(target)

    def _enqueue(self, kind, key, header, payload):
        with self.state_lock:
            if not self.active or (self.debug_only and key in self.debug_queued):
                return
            item = (kind, key, stamp_ns(header), header.frame_id, time.time_ns(), payload)
            try:
                self.queues[kind].put_nowait(item)
                if self.debug_only:
                    self.debug_queued.add(key)
                self.queue_high_water[kind] = max(self.queue_high_water[kind], self.queues[kind].qsize())
            except queue.Full:
                self.dropped[key] = self.dropped.get(key, 0) + 1
                now = time.monotonic()
                if now - self.last_drop_warning >= 5.0:
                    print(f"[WARN] Recording queues full; dropped frames: {self.dropped}", file=sys.stderr)
                    self.last_drop_warning = now

    def record_rgb(self, camera, stream, header, bgr):
        # The producer never modifies a resized frame after handing it off.
        self._enqueue("rgb", f"{camera}_{stream}", header, bgr)

    def record_lidar(self, name, msg):
        if self.debug_only:
            with self.state_lock:
                if name in self.debug_queued or not self.active:
                    return
            if not msg.data or msg.width * msg.height == 0:
                return  # An empty scan does not confirm that sensor values arrived.
        # Copy the raw PointCloud2 payload and its schema for exact ROS replay.
        payload = {
            "data": bytes(msg.data),
            "height": msg.height,
            "width": msg.width,
            "fields": [
                {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                for f in msg.fields
            ],
            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "is_dense": msg.is_dense,
        }
        self._enqueue("lidar", name, msg.header, payload)

    def _write_rgb(self, key, timestamp, frame_id, received_ns, bgr):
        if self.debug_only:
            spec = self.manifest["rgb_streams"][key]
            path = self.folder / spec["image"]
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), bgr):
                raise RuntimeError(f"Cannot save image: {path}")
            metadata = {"stamp_ns": timestamp, "frame_id": frame_id, "received_ns": received_ns,
                        "shape": list(bgr.shape), "dtype": str(bgr.dtype),
                        "min": int(bgr.min()), "max": int(bgr.max()), "mean": float(bgr.mean())}
            (self.folder / spec["index"]).write_text(json.dumps(metadata, indent=2) + "\n")
            self.counts[key] = 1
            print(f"[DEBUG SAVE] {key}: {path}")
            return
        if key not in self.videos:
            spec = self.manifest["rgb_streams"][key]
            video_path = self.folder / spec["video"]
            video_path.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2.VideoWriter(
                str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS,
                (OUTPUT_WIDTH, OUTPUT_HEIGHT),
            )
            if not writer.isOpened():
                writer.release()
                raise RuntimeError(f"Cannot open MP4 writer: {video_path}")
            try:
                index = (self.folder / spec["index"]).open("w", newline="", encoding="utf-8")
            except Exception:
                writer.release()
                raise
            csv_writer = csv.writer(index)
            csv_writer.writerow(("frame_index", "stamp_ns", "frame_id", "received_ns"))
            self.videos[key] = writer
            self.index_files[key] = (index, csv_writer)
        frame_index = self.counts.get(key, 0)
        self.videos[key].write(bgr)
        index, csv_writer = self.index_files[key]
        csv_writer.writerow((frame_index, timestamp, frame_id, received_ns))
        self.counts[key] = frame_index + 1

    def _write_lidar(self, key, timestamp, frame_id, received_ns, payload):
        spec = self.manifest["lidar_streams"][key]
        if key not in self.lidar_files:
            data_dir = self.folder / spec["data_dir"]
            data_dir.mkdir(parents=True, exist_ok=True)
            self.index_files[key] = (self.folder / spec["index"]).open("w", encoding="utf-8")
            self.lidar_files[key] = [0, (data_dir / "chunk_000000.bin").open("wb")]
        chunk, data_file = self.lidar_files[key]
        length = len(payload["data"])
        if data_file.tell() and data_file.tell() + length > LIDAR_CHUNK_BYTES:
            data_file.close()
            chunk += 1
            data_file = (self.folder / spec["data_dir"] / f"chunk_{chunk:06d}.bin").open("wb")
            self.lidar_files[key] = [chunk, data_file]
        offset = data_file.tell()
        data_file.write(payload["data"])
        metadata = {k: v for k, v in payload.items() if k != "data"}
        metadata.update({"file": f"chunk_{chunk:06d}.bin", "offset": offset, "length": length,
                         "stamp_ns": timestamp, "frame_id": frame_id, "received_ns": received_ns})
        self.index_files[key].write(json.dumps(metadata) + "\n")
        self.counts[key] += 1

    def _flush(self, kind):
        # Each worker only touches its own files. Flush cloud data before its index.
        if kind == "lidar":
            for _, data_file in self.lidar_files.values():
                data_file.flush()
        for key in self.manifest[f"{kind}_streams"]:
            entry = self.index_files.get(key)
            if entry is not None:
                (entry[0] if isinstance(entry, tuple) else entry).flush()

    def _record_error(self, key, exc):
        error = f"{key}: {exc}"
        with self.state_lock:
            if error not in self.errors:
                self.errors.append(error)
                print(f"[RECORD ERROR] {error}", file=sys.stderr)

    def _run_writer(self, kind):
        last_flush = time.monotonic()
        failed = set()
        while True:
            try:
                item = self.queues[kind].get(timeout=INDEX_FLUSH_SEC)
            except queue.Empty:
                item = ()
            if item is None:
                break
            if item:
                _, key, timestamp, frame_id, received_ns, payload = item
                if key not in failed:
                    try:
                        if kind == "rgb":
                            self._write_rgb(key, timestamp, frame_id, received_ns, payload)
                        else:
                            self._write_lidar(key, timestamp, frame_id, received_ns, payload)
                    except Exception as exc:
                        failed.add(key)
                        self._record_error(key, exc)
            if time.monotonic() - last_flush >= INDEX_FLUSH_SEC:
                try:
                    self._flush(kind)
                except Exception as exc:
                    self._record_error(kind, exc)
                last_flush = time.monotonic()

    def stop(self):
        with self.state_lock:
            if not self.active:
                return
            self.active = False
        for pending in self.queues.values():
            pending.put(None)
        for worker in self.workers:
            worker.join()
        for key, (_, data_file) in self.lidar_files.items():
            try:
                data_file.close()
            except Exception as exc:
                self._record_error(key, exc)
        for key, writer in self.videos.items():
            try:
                writer.release()
            except Exception as exc:
                self._record_error(key, exc)
        for key, file_entry in self.index_files.items():
            index = file_entry[0] if isinstance(file_entry, tuple) else file_entry
            try:
                index.close()
            except Exception as exc:
                self._record_error(key, exc)
        self.manifest.update({
            "stopped_at": datetime.now().astimezone().isoformat(),
            "status": "complete" if not self.errors and not self.dropped and
                      (not self.debug_only or all(self.counts.values())) else "incomplete",
            "counts": self.counts,
            "missing_streams": [key for key, count in self.counts.items() if count == 0],
            "dropped": self.dropped,
            "queue_high_water": self.queue_high_water,
            "errors": self.errors,
        })
        self._write_manifest()
        print(f"Recording saved to {self.folder}")
        if self.debug_only:
            print(f"[DEBUG SAVE] Missing inputs: {self.manifest['missing_streams']}")
        if self.dropped:
            print(f"[WARN] Recording queue dropped frames: {self.dropped}")


class RawRgbStreamer:
    def __init__(self, launch_robot=False, save=False, camera_clouds=False, depth=False, debug_save=False, cameras=None, pose=True):
        self.cameras = {name: CAMERAS[name] for name in (CAMERAS if cameras is None else cameras)}
        self.launch_robot = launch_robot
        self.save = save or debug_save
        self.debug_save = debug_save
        self.camera_clouds = camera_clouds
        self.depth_enabled = depth or camera_clouds
        self.projection_cache = {}
        self.recorder = None
        # The SDK normally acquires control during construction. This example
        # is sensor-only, so suppress that call while creating the client.
        original_acquire = AstribotInterface.acquire_control_rights
        AstribotInterface.acquire_control_rights = lambda interface, _high: setattr(
            interface, "have_control_rights", False
        )
        try:
            sdk_logger = logging.getLogger("astribot_sensor_sdk")
            sdk_logger.setLevel(logging.WARNING)
            self.robot = Astribot(logger=sdk_logger)
        finally:
            AstribotInterface.acquire_control_rights = original_acquire
        self.node = self.robot.astribot_interface.node
        self.node.get_logger().set_level(LoggingSeverity.WARN)
        self.publishers = {
            camera: {
                **({
                    "color": self.node.create_publisher(Image, spec["color"], qos_profile_sensor_data),
                } if "color" in spec else {
                    stream: self.node.create_publisher(Image, topic, qos_profile_sensor_data)
                    for stream, topic in spec["stereo"].items()
                }),
                **({
                    "depth": self.node.create_publisher(Image, spec["depth"], qos_profile_sensor_data),
                    "cloud": self.node.create_publisher(PointCloud2, spec["cloud"], qos_profile_sensor_data),
                } if "depth" in spec else {}),
            }
            for camera, spec in self.cameras.items()
        }
        # Match 301-get_lidar_scan.py: use the SDK node and depth-10 best effort.
        # The SDK already spins this node on its own executor thread.
        self.lidar_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_subscriptions = []
        self.lidar_publishers = {
            name: self.node.create_publisher(PointCloud2, output_topic, self.lidar_qos)
            for name, (_, output_topic) in LIDAR_TOPICS.items()
        }
        self.subscriptions = []
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_color = {}
        self.latest_color_input_shape = {}
        self.latest_depth = {}
        self.lidar_frames = {name: 0 for name in LIDAR_TOPICS}
        self.last_data = {}
        self.sensor_log_times = {}
        self.watchdog_thread = None
        self.pose_timer = None
        self.pose_error_reported = False
        self.pose_names = [name for name in self.robot.whole_body_names
                           if pose and name not in self.robot.effector_names]
        self.pose_publishers = {
            name: self.node.create_publisher(PoseStamped, f"/sdk/astribot_pose/{name}", qos_profile_sensor_data)
            for name in self.pose_names
        }

    def mark_data(self, stream):
        with self.lock:
            self.last_data[stream] = time.monotonic()

    def log_sensor_input(self, stream, header):
        """Report published messages using ROS timestamps, once per second per stream."""
        now = time.monotonic()
        with self.lock:
            previous = self.sensor_log_times.get(stream)
            if previous is not None and now - previous < SENSOR_LOG_INTERVAL_SEC:
                return
            self.sensor_log_times[stream] = now
        print(f"{stream} published at {header.stamp.sec}.{header.stamp.nanosec:09d}")

    def sdk_image_callback(self, topic, msg, width, height, array):
        """Dispatch the SDK-decoded color and raw depth callbacks by source topic."""
        parts = topic.strip("/").split("/")
        if len(parts) < 3 or parts[0] != "astribot_camera" or parts[1] not in self.cameras:
            return
        camera, kind = parts[1], parts[2]
        if kind == "depth_compress":
            with self.lock:
                shape = self.latest_color_input_shape.get(camera)
            if shape is not None and self.depth_enabled:
                self.publish_depth(camera, msg, shape[1], shape[0], array)
        elif kind == "color_compress" and array is not None and array.ndim == 3:
            if "stereo" in self.cameras[camera]:
                # The robot's driver packs the left/right views into one JPEG.
                if array.shape[1] % 2:
                    return
                midpoint = array.shape[1] // 2
                for stream, view in (("left", array[:, :midpoint]), ("right", array[:, midpoint:])):
                    self.publish_frame(camera, msg, view.shape[1], view.shape[0], view, stream)
            else:
                self.publish_frame(camera, msg, width, height, array)

    def publish_robot_pose(self):
        """Read world-frame poses through the SDK, as in 102-get_cartesian_states.py."""
        try:
            values = self.robot.get_current_cartesian_pose(
                names=self.pose_names, frame=self.robot.world_frame_name,
            )
            if len(values) != len(self.pose_names) or any(len(pose) != 7 for pose in values):
                raise ValueError("SDK returned an incomplete Cartesian pose snapshot")
            # The getter has no source timestamp; stamp this snapshot at read time.
            timestamp = self.node.get_clock().now().to_msg()
            for name, values_for_part in zip(self.pose_names, values):
                pose = PoseStamped()
                pose.header.stamp = timestamp
                pose.header.frame_id = self.robot.world_frame_name
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) = map(float, values_for_part[:3])
                (pose.pose.orientation.x, pose.pose.orientation.y,
                 pose.pose.orientation.z, pose.pose.orientation.w) = map(float, values_for_part[3:])
                self.pose_publishers[name].publish(pose)
            self.pose_error_reported = False
        except Exception as exc:
            if not self.pose_error_reported:
                print(f"[WARN] SDK pose read failed: {exc}", file=sys.stderr)
                self.pose_error_reported = True

    def publish_frame(self, camera, compressed_msg, width, height, bgr, stream="color"):
        """Publish the SDK-decoded BGR ndarray as sensor_msgs/Image."""
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
            return

        input_shape = bgr.shape[:2]
        bgr = cv2.resize(bgr, (OUTPUT_WIDTH, OUTPUT_HEIGHT), interpolation=cv2.INTER_AREA)
        bgr = np.ascontiguousarray(bgr, dtype=np.uint8)
        image = Image()
        image.header = compressed_msg.header
        image.height = int(bgr.shape[0])
        image.width = int(bgr.shape[1])
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = int(bgr.strides[0])
        image.data = bgr.tobytes()
        self.publishers[camera][stream].publish(image)
        if self.recorder is not None and self.recorder.active:
            self.recorder.record_rgb(camera, stream, image.header, bgr)
        self.log_sensor_input(f"{camera}/{stream}", image.header)
        self.mark_data(f"{camera}/{stream}")
        if stream != "color":
            return
        with self.lock:
            self.latest_color_input_shape[camera] = input_shape
            if self.camera_clouds:
                self.latest_color[camera] = (compressed_msg.header, bgr)
        if self.camera_clouds:
            self.try_publish_pointcloud(camera)

    @staticmethod
    def decode_depth(array, width, height):
        """Decode the raw depth payload, which is normally uint16 millimetres."""
        raw = np.asarray(array)
        if raw.dtype == np.uint16 and width and height and raw.size == width * height:
            return raw.reshape(height, width)
        if raw.dtype == np.float32 and width and height and raw.size == width * height:
            return raw.reshape(height, width)
        if width and height and raw.size == width * height * 2:
            return raw.view(np.uint16).reshape(height, width)
        if width and height and raw.size == width * height * 4:
            return raw.view(np.float32).reshape(height, width)
        return None

    def publish_depth(self, camera, compressed_msg, width, height, array):
        with self.lock:
            input_shape = self.latest_color_input_shape.get(camera)
        if input_shape is not None:
            height, width = input_shape
        depth = self.decode_depth(array, width, height)
        if depth is None:
            return
        if depth.shape[:2] != (OUTPUT_HEIGHT, OUTPUT_WIDTH):
            depth = cv2.resize(depth, (OUTPUT_WIDTH, OUTPUT_HEIGHT), interpolation=cv2.INTER_NEAREST)
        depth = np.ascontiguousarray(depth)
        image = Image()
        image.header = compressed_msg.header
        image.height, image.width = depth.shape
        image.encoding = "32FC1" if depth.dtype == np.float32 else "16UC1"
        image.is_bigendian = 0
        image.step = int(depth.strides[0])
        image.data = depth.tobytes()
        self.publishers[camera]["depth"].publish(image)
        self.log_sensor_input(f"{camera}/depth", image.header)
        self.mark_data(f"{camera}/depth")
        if self.camera_clouds:
            with self.lock:
                self.latest_depth[camera] = (compressed_msg.header, depth)
            self.try_publish_pointcloud(camera)

    def try_publish_pointcloud(self, camera):
        with self.lock:
            if not self.publishers[camera]["cloud"].get_subscription_count():
                self.latest_color.pop(camera, None)
                self.latest_depth.pop(camera, None)
                return
            color = self.latest_color.get(camera)
            depth = self.latest_depth.get(camera)
            if color is None or depth is None:
                return
            delta = stamp_ns(color[0]) - stamp_ns(depth[0])
            if abs(delta) > CLOUD_SYNC_NS:
                # Discard the older sample; never reuse a frame in two clouds.
                (self.latest_depth if delta > 0 else self.latest_color).pop(camera, None)
                return
            self.latest_color.pop(camera)
            self.latest_depth.pop(camera)
        self.publish_pointcloud(camera, depth[0], color[1], depth[1])

    def publish_pointcloud(self, camera, header, bgr, depth):
        if bgr.shape[:2] != depth.shape[:2]:
            return
        height, width = depth.shape
        cache_key = (camera, height, width, INTRINSICS[camera])
        if cache_key not in self.projection_cache:
            fx, fy, cx, cy = INTRINSICS[camera]
            scale_x, scale_y = width / INTRINSICS_WIDTH, height / INTRINSICS_HEIGHT
            fx, fy, cx, cy = fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y
            v, u = np.indices((height, width), dtype=np.float32)
            self.projection_cache[cache_key] = ((u - cx) / fx, (v - cy) / fy)
        x_factor, y_factor = self.projection_cache[cache_key]
        z = depth.astype(np.float32)
        if depth.dtype == np.uint16:
            z *= DEPTH_SCALE_M
        valid = np.isfinite(z) & (z > 0)
        points = np.empty((int(valid.sum()), 4), dtype="<f4")
        points[:, 0] = x_factor[valid] * z[valid]
        points[:, 1] = y_factor[valid] * z[valid]
        points[:, 2] = z[valid]
        colors = bgr[valid].astype("<u4")
        rgb = (colors[:, 2] << 16) | (colors[:, 1] << 8) | colors[:, 0]
        points[:, 3] = rgb.astype("<u4", copy=False).view("<f4")
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 16, 16 * len(points)
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.publishers[camera]["cloud"].publish(cloud)

    def lidar_callback(self, name, msg):
        """Relay each Livox scan and report incoming point clouds."""
        self.lidar_frames[name] += 1
        self.mark_data(f"lidar/{name}")
        self.lidar_publishers[name].publish(msg)
        self.log_sensor_input(f"lidar/{name}", msg.header)
        if self.recorder is not None and self.recorder.active:
            self.recorder.record_lidar(name, msg)

    def watchdog(self):
        expected = [
            f"{camera}/{stream}"
            for camera, spec in self.cameras.items()
            for stream in (spec.get("stereo", {}).keys() or ("color",))
        ]
        if self.depth_enabled:
            expected += [f"{camera}/depth" for camera, spec in self.cameras.items() if "depth" in spec]
        expected += [f"lidar/{name}" for name in LIDAR_TOPICS]
        warned = set()
        while not self.stop_event.wait(1.0):
            now = time.monotonic()
            with self.lock:
                last_data = dict(self.last_data)
            for stream in expected:
                last = last_data.get(stream)
                if last is None or now - last > DATA_TIMEOUT_SEC:
                    if stream not in warned:
                        print(f"[WARN] No {stream} data for more than {DATA_TIMEOUT_SEC:.0f}s")
                        warned.add(stream)
                else:
                    warned.discard(stream)

    def start(self):
        print("Connecting to the already launched robot...")
        if self.save:
            self.recorder = SensorRecorder(debug_only=self.debug_save, cameras=self.cameras)
            print(f"Recording in {self.recorder.folder}; press q to stop recording.")

        # Register both callbacks before restarting the driver, as in example 301.
        # Recording is ready before callbacks can receive the first scan.
        for name, (input_topic, output_topic) in LIDAR_TOPICS.items():
            subscription = self.node.create_subscription(
                PointCloud2,
                input_topic,
                lambda msg, n=name: self.lidar_callback(n, msg),
                self.lidar_qos,
            )
            self.lidar_subscriptions.append(subscription)
            print(f"LiDAR topic: {input_topic} -> {output_topic}")
        # Follow 301-get_lidar_scan.py: reset the driver before activating it.
        self.robot.deactivate_lidar()
        if self.stop_event.wait(1.0):
            return
        activations = [("LiDAR", self.robot.activate_lidar)]
        if self.cameras:
            activations.append(("Camera", self.robot.activate_camera))
        for name, activate in activations:
            for attempt in range(3):
                if activate():
                    break
                if self.stop_event.wait(2.0):
                    return
            else:
                print(f"[WARN] {name} activation failed; listening for existing streams anyway.")

        # Give the camera driver time to advertise and begin streaming.
        if self.cameras:
            if self.stop_event.wait(2.0):
                return
            print(f"RGB camera topics (all output images are {OUTPUT_WIDTH}x{OUTPUT_HEIGHT}):")
        for spec in self.cameras.values():
            topics = spec.get("stereo", {"color": spec.get("color")})
            for topic in topics.values():
                print(f"  {topic}")
        # Follow 300-get_images.py. This SDK stores one callback/decode setting,
        # so every registration must use this same dispatcher and need_decode=True.
        for camera, spec in self.cameras.items():
            kinds = ("color", "depth") if self.depth_enabled and "depth" in spec else ("color",)
            for kind in kinds:
                subscription = self.robot.register_image_callback(
                    camera, kind, self.sdk_image_callback, need_decode=True,
                )
                if subscription is None:
                    raise RuntimeError(f"SDK camera subscription failed: {camera}/{kind}")
                self.subscriptions.append(subscription)
        if self.pose_names:
            self.pose_timer = self.node.create_timer(0.1, self.publish_robot_pose)
        self.watchdog_thread = threading.Thread(target=self.watchdog, daemon=True)
        self.watchdog_thread.start()

    def run(self):
        self.start()
        print("Streaming. Press Ctrl-C to quit.")
        terminal_fd = None
        terminal_settings = None
        if self.save and sys.stdin.isatty():
            terminal_fd = sys.stdin.fileno()
            terminal_settings = termios.tcgetattr(terminal_fd)
            tty.setcbreak(terminal_fd)
        elif self.save:
            print("[WARN] No interactive terminal; use Ctrl-C to finish recording and quit.")
        try:
            while not self.stop_event.is_set():
                if self.debug_save and self.recorder is not None and self.recorder.active:
                    if all(self.recorder.counts.values()) or time.monotonic() - self.recorder.started_monotonic >= 30.0:
                        self.recorder.stop()
                        print("Debug capture finished; sensor streaming continues.")
                if terminal_fd is None:
                    self.stop_event.wait(0.2)
                    continue
                readable, _, _ = select.select([terminal_fd], [], [], 0.2)
                if readable and sys.stdin.read(1).lower() == "q":
                    if self.recorder is not None and self.recorder.active:
                        self.recorder.stop()
                        print("Recording stopped; sensor streaming continues. Press Ctrl-C to quit.")
        finally:
            if terminal_settings is not None:
                termios.tcsetattr(terminal_fd, termios.TCSADRAIN, terminal_settings)

    def stop(self):
        self.stop_event.set()
        if self.pose_timer is not None:
            self.node.destroy_timer(self.pose_timer)
            self.pose_timer = None
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        for subscription in self.lidar_subscriptions:
            self.node.destroy_subscription(subscription)
        self.lidar_subscriptions.clear()
        if self.recorder is not None:
            self.recorder.stop()
        self.robot.deactivate_lidar()


def main():
    parser = argparse.ArgumentParser(description="Stream RGB-D, LiDAR, and body state data.")
    parser.add_argument(
        "--launch",
        action="store_true",
        help="Request explicit robot launch/startup mode; read-only mode remains enabled.",
    )
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument(
        "--save", action="store_true",
        help="Record RGB MP4s, frame timestamps, and LiDAR clouds until q or Ctrl-C.",
    )
    parser.add_argument(
        "--camera-clouds", action="store_true",
        help="Generate camera point clouds for subscribers using paired RGB/depth frames.",
    )
    parser.add_argument("--depth", action="store_true", help="Also subscribe to raw depth streams (high bandwidth).")
    recording.add_argument("--debug-save", action="store_true",
                           help="Save one PNG per RGB stream and one scan per LiDAR; finish after 30 seconds.")
    sensors = parser.add_mutually_exclusive_group()
    sensors.add_argument("--cameras", nargs="+", choices=list(CAMERAS), default=list(CAMERAS),
                        help="Camera inputs to subscribe to; both LiDARs remain enabled.")
    sensors.add_argument("--lidar-only", action="store_true",
                         help="Stream both LiDARs without camera activation, image subscriptions, or pose polling.")
    parser.add_argument("--no-pose", action="store_true", help="Disable pose polling and publication.")
    args = parser.parse_args()
    if args.lidar_only and (args.depth or args.camera_clouds):
        parser.error("--lidar-only cannot be combined with --depth or --camera-clouds")
    streamer = RawRgbStreamer(launch_robot=args.launch, save=args.save, camera_clouds=args.camera_clouds, depth=args.depth, debug_save=args.debug_save, cameras=[] if args.lidar_only else args.cameras, pose=not (args.no_pose or args.lidar_only))

    def shutdown(_signum, _frame):
        streamer.stop_event.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        streamer.run()
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
