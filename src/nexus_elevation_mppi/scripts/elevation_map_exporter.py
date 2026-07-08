#!/usr/bin/env python3

import json
import math
import signal
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def decode_multiarray_to_rows_cols(name: str, array_msg: Float32MultiArray) -> np.ndarray:
    data_np = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0 = dims[0].label
        label1 = dims[1].label

        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size or 1
            cols = dims[1].size or (len(data_np) // rows if rows else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order="C")

        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order="F")

    if dims:
        cols = dims[0].size or 1
        rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else len(data_np))
    else:
        cols = int(math.sqrt(len(data_np))) if len(data_np) else 0
        rows = cols

    if rows * cols != data_np.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
    return data_np.reshape((rows, cols), order="C")


class ElevationMapExporter(Node):
    def __init__(self) -> None:
        super().__init__("elevation_map_exporter")

        self.declare_parameter("grid_map_topic", "/elevation_mapping_node/elevation_map")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("elevation_layer", "elevation")
        self.declare_parameter("save_interval_sec", 2.0)
        self.declare_parameter("save_preview", True)

        self.grid_map_topic = str(self.get_parameter("grid_map_topic").value)
        output_dir = str(self.get_parameter("output_dir").value).strip()
        if not output_dir:
            output_dir = str(Path.cwd() / "output" / "elevation_maps")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.elevation_layer = str(self.get_parameter("elevation_layer").value)
        self.save_interval_sec = float(self.get_parameter("save_interval_sec").value)
        self.save_preview = bool(self.get_parameter("save_preview").value)

        self.latest_msg: Optional[GridMap] = None
        self.latest_seq = 0
        self.last_saved_seq = 0
        self.final_saved = False
        self.shutdown_requested = False

        self.sub = self.create_subscription(
            GridMap,
            self.grid_map_topic,
            self.grid_map_callback,
            10,
        )
        self.timer = None
        if self.save_interval_sec > 0.0:
            self.timer = self.create_timer(self.save_interval_sec, self.timer_callback)

        if self.timer is None:
            self.get_logger().info(
                f"Exporting {self.grid_map_topic} to {self.output_dir} on shutdown only"
            )
        else:
            self.get_logger().info(
                f"Exporting {self.grid_map_topic} to {self.output_dir} every {self.save_interval_sec:.2f}s"
            )

    def grid_map_callback(self, msg: GridMap) -> None:
        self.latest_msg = msg
        self.latest_seq += 1

    def timer_callback(self) -> None:
        if self.latest_msg is None or self.latest_seq == self.last_saved_seq:
            return
        self.save_snapshot("latest")

    def request_shutdown(self, reason: str) -> None:
        # Only set the flag here. The actual save_snapshot("final") and
        # rclpy.shutdown() happen in main()'s normal execution context so we
        # never perform file I/O or call the rclpy API from within an
        # async-signal handler (not signal-safe; can re-enter rclpy.spin).
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.get_logger().info(f"Shutting down exporter: {reason}")

    def save_snapshot(self, kind: str) -> None:
        if kind == "final" and self.final_saved:
            return

        msg = self.latest_msg
        if msg is None:
            if kind == "final" and not self.final_saved:
                self.get_logger().warning("No GridMap received before shutdown; nothing to export.")
                self.final_saved = True
            return

        try:
            layer_arrays, metadata = self.decode_grid_map(msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to decode GridMap: {exc}")
            return

        elevation = layer_arrays.get(self.elevation_layer)
        if elevation is None:
            self.get_logger().warning(
                f"GridMap does not contain layer '{self.elevation_layer}', available={list(layer_arrays.keys())}"
            )
            return

        prefix = f"elevation_{kind}"
        npz_path = self.output_dir / f"{prefix}.npz"
        json_path = self.output_dir / f"{prefix}.json"
        pgm_path = self.output_dir / f"{prefix}.pgm"

        npz_payload = {
            "frame_id": np.asarray(msg.header.frame_id),
            "stamp_sec": np.asarray(msg.header.stamp.sec, dtype=np.int64),
            "stamp_nanosec": np.asarray(msg.header.stamp.nanosec, dtype=np.int64),
            "basic_layers": np.asarray(msg.basic_layers, dtype="<U64"),
            "center": np.asarray(metadata["center"], dtype=np.float32),
            "orientation_xyzw": np.asarray(metadata["orientation_xyzw"], dtype=np.float32),
            "resolution": np.asarray(metadata["resolution"], dtype=np.float32),
            "length_x": np.asarray(metadata["length_x"], dtype=np.float32),
            "length_y": np.asarray(metadata["length_y"], dtype=np.float32),
        }
        npz_payload.update(layer_arrays)
        self.write_npz_atomic(npz_path, npz_payload)

        preview_stats = self.write_preview(pgm_path, elevation) if self.save_preview else None
        json_payload = self.build_json_payload(msg, layer_arrays, metadata, preview_stats)
        self.write_json_atomic(json_path, json_payload)

        self.last_saved_seq = self.latest_seq
        if kind == "final":
            self.final_saved = True

        self.get_logger().info(f"Saved {kind} elevation map to {npz_path}")

    def decode_grid_map(self, msg: GridMap) -> Tuple[Dict[str, np.ndarray], Dict[str, object]]:
        if len(msg.layers) != len(msg.data):
            raise ValueError("Mismatch between GridMap layers and data arrays.")

        arrays: Dict[str, np.ndarray] = {}
        for name, array_msg in zip(msg.layers, msg.data):
            arrays[name] = decode_multiarray_to_rows_cols(name, array_msg)

        metadata = {
            "center": [
                float(msg.info.pose.position.x),
                float(msg.info.pose.position.y),
                float(msg.info.pose.position.z),
            ],
            "orientation_xyzw": [
                float(msg.info.pose.orientation.x),
                float(msg.info.pose.orientation.y),
                float(msg.info.pose.orientation.z),
                float(msg.info.pose.orientation.w),
            ],
            "resolution": float(msg.info.resolution),
            "length_x": float(msg.info.length_x),
            "length_y": float(msg.info.length_y),
        }
        return arrays, metadata

    def build_json_payload(
        self,
        msg: GridMap,
        layer_arrays: Dict[str, np.ndarray],
        metadata: Dict[str, object],
        preview_stats: Optional[Dict[str, float]],
    ) -> Dict[str, object]:
        elevation = layer_arrays[self.elevation_layer]
        valid_mask = np.isfinite(elevation)
        valid_values = elevation[valid_mask]
        rows, cols = elevation.shape

        payload: Dict[str, object] = {
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "layers": list(layer_arrays.keys()),
            "basic_layers": list(msg.basic_layers),
            "elevation_layer": self.elevation_layer,
            "shape": {
                "rows": int(rows),
                "cols": int(cols),
            },
            "resolution": metadata["resolution"],
            "length_x": metadata["length_x"],
            "length_y": metadata["length_y"],
            "center": metadata["center"],
            "orientation_xyzw": metadata["orientation_xyzw"],
            "valid_cell_count": int(valid_values.size),
            "nan_cell_count": int(elevation.size - valid_values.size),
        }

        if valid_values.size > 0:
            payload["elevation_min"] = float(np.min(valid_values))
            payload["elevation_max"] = float(np.max(valid_values))
        else:
            payload["elevation_min"] = None
            payload["elevation_max"] = None

        if preview_stats is not None:
            payload["preview"] = preview_stats

        return payload

    def write_npz_atomic(self, path: Path, payload: Dict[str, np.ndarray]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile(prefix=path.stem + ".", suffix=".npz.tmp", dir=path.parent, delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            with tmp_path.open("wb") as handle:
                np.savez_compressed(handle, **payload)
            tmp_path.replace(path)
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

    def write_json_atomic(self, path: Path, payload: Dict[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile(prefix=path.stem + ".", suffix=".json.tmp", dir=path.parent, delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            tmp_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
            tmp_path.replace(path)
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

    def write_preview(self, path: Path, elevation: np.ndarray) -> Dict[str, float]:
        valid_mask = np.isfinite(elevation)
        image = np.zeros(elevation.shape, dtype=np.uint8)

        if np.any(valid_mask):
            valid_values = elevation[valid_mask]
            min_value = float(np.min(valid_values))
            max_value = float(np.max(valid_values))
            if max_value > min_value:
                scaled = (valid_values - min_value) / (max_value - min_value)
                image[valid_mask] = np.clip(np.round(scaled * 255.0), 0, 255).astype(np.uint8)
            else:
                image[valid_mask] = 255
        else:
            min_value = None
            max_value = None

        path.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile(prefix=path.stem + ".", suffix=".pgm.tmp", dir=path.parent, delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            with tmp_path.open("wb") as handle:
                header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
                handle.write(header)
                handle.write(image.tobytes(order="C"))
            tmp_path.replace(path)
        finally:
            if tmp_path.exists():
                tmp_path.unlink()

        return {
            "path": str(path),
            "format": "pgm",
            "min_value": min_value,
            "max_value": max_value,
        }


def main() -> None:
    rclpy.init()
    node = ElevationMapExporter()

    def handle_signal(signum, _frame) -> None:
        node.request_shutdown(f"signal {signum}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        # Poll the shutdown flag instead of rclpy.spin() so the signal
        # handler only sets a flag — the save + shutdown happen here, in
        # normal execution context, not inside the signal handler.
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_snapshot("final")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
