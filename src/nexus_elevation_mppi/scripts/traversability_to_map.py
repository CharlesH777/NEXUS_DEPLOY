#!/usr/bin/env python3
"""
traversability_to_map.py

Subscribes to the elevation_mapping_cupy GridMap, derives a traversability-like
occupancy map directly from the elevation layer, and republishes it as a
nav_msgs/OccupancyGrid for easy RViz viewing.

Visual convention (RViz OccupancyGrid):
  - locally flat / low height range  -> 0   (white / free)
  - locally rough / high height range -> 100 (black / occupied)
  - NaN / unobserved                  -> -1  (gray / unknown)

Coordinate handling:
  grid_map messages use a circular buffer plus a different linearization order
  than nav_msgs/OccupancyGrid. We first unwrap the circular buffer with the
  message start indices, then reorder cells to match the OccupancyGrid
  convention (origin at the bottom-left, x increasing fastest in data[]).
"""

import math
import warnings

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion


def decode_layer(name: str, array_msg) -> np.ndarray:
    """Decode a GridMap Float32MultiArray layer into a (rows, cols) ndarray.

    Mirrors the logic in elevation_map_exporter.py so the orientation matches
    what gets saved to npz.
    """
    data_np = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0, label1 = dims[0].label, dims[1].label
        if label0 == "row_index" and label1 == "column_index":
            rows = dims[0].size or 1
            cols = dims[1].size or (len(data_np) // rows if rows else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout.")
            return data_np.reshape((rows, cols), order="C")
        if label0 == "column_index" and label1 == "row_index":
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout.")
            return data_np.reshape((rows, cols), order="F")

    if dims:
        cols = dims[0].size or 1
        rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else len(data_np))
    else:
        cols = int(math.sqrt(len(data_np))) if len(data_np) else 0
        rows = cols

    if rows * cols != data_np.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout.")
    return data_np.reshape((rows, cols), order="C")


def unwrap_layer(array: np.ndarray, outer_start_index: int, inner_start_index: int) -> np.ndarray:
    """Expand the grid_map circular buffer into an unwrapped matrix."""
    shift = (-int(outer_start_index), -int(inner_start_index))
    if shift == (0, 0):
        return array
    return np.roll(array, shift=shift, axis=(0, 1))


def is_identity_quaternion(quaternion: Quaternion, atol: float = 1e-6) -> bool:
    return (
        abs(float(quaternion.x)) <= atol
        and abs(float(quaternion.y)) <= atol
        and abs(float(quaternion.z)) <= atol
        and abs(float(quaternion.w) - 1.0) <= atol
    )


def quaternion_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
    x_value = float(quaternion.x)
    y_value = float(quaternion.y)
    z_value = float(quaternion.z)
    w_value = float(quaternion.w)

    xx = x_value * x_value
    yy = y_value * y_value
    zz = z_value * z_value
    xy = x_value * y_value
    xz = x_value * z_value
    yz = y_value * z_value
    wx = w_value * x_value
    wy = w_value * y_value
    wz = w_value * z_value

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def local_height_range_map(elevation: np.ndarray, kernel_size: int) -> np.ndarray:
    """Compute per-cell local height range from a square neighborhood."""
    if kernel_size % 2 == 0 or kernel_size < 1:
        raise ValueError("kernel_size must be a positive odd integer.")

    pad = kernel_size // 2
    padded = np.pad(elevation, pad_width=pad, mode="constant", constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    valid_mask = np.isfinite(windows)
    valid_count = np.sum(valid_mask, axis=(-2, -1))

    local_max = np.max(np.where(valid_mask, windows, -np.inf), axis=(-2, -1))
    local_min = np.min(np.where(valid_mask, windows, np.inf), axis=(-2, -1))
    height_range = np.abs(local_max - local_min).astype(np.float32)

    center_valid = np.isfinite(elevation)
    height_range[(valid_count < 2) | (~center_valid)] = np.nan
    return height_range


def roughness_to_occupancy(
    height_range: np.ndarray,
    clear_below_m: float,
    accumulate_from_m: float,
    full_at_m: float,
) -> np.ndarray:
    """Map local height range in meters to occupancy in [0, 100]."""
    if not (0.0 <= clear_below_m <= accumulate_from_m < full_at_m):
        raise ValueError("Expected 0 <= clear_below_m <= accumulate_from_m < full_at_m.")

    occupancy = np.full(height_range.shape, np.nan, dtype=np.float32)
    valid_mask = np.isfinite(height_range)
    if not np.any(valid_mask):
        return occupancy

    values = height_range[valid_mask]
    mapped = np.zeros(values.shape, dtype=np.float32)
    ramp_mask = values >= accumulate_from_m
    if np.any(ramp_mask):
        mapped[ramp_mask] = (
            (values[ramp_mask] - accumulate_from_m) / (full_at_m - accumulate_from_m)
        ) * 100.0
    occupancy[valid_mask] = np.clip(mapped, 0.0, 100.0)
    occupancy[np.isfinite(occupancy) & (height_range < clear_below_m)] = 0.0
    return occupancy


def median_filter_nan(values: np.ndarray, kernel_size: int) -> np.ndarray:
    """Apply a NaN-aware median filter while preserving unknown cells."""
    if kernel_size % 2 == 0 or kernel_size < 1:
        raise ValueError("kernel_size must be a positive odd integer.")

    pad = kernel_size // 2
    padded = np.pad(values, pad_width=pad, mode="constant", constant_values=np.nan)
    windows = np.lib.stride_tricks.sliding_window_view(padded, (kernel_size, kernel_size))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        filtered = np.nanmedian(windows, axis=(-2, -1)).astype(np.float32)
    filtered[~np.isfinite(values)] = np.nan
    return filtered


def grid_map_to_occupancy_data(values: np.ndarray, unknown_value: int) -> np.ndarray:
    """Reorder an unwrapped grid_map matrix into OccupancyGrid data order."""
    out = np.full(values.shape, unknown_value, dtype=np.int8)
    valid_mask = np.isfinite(values)
    if np.any(valid_mask):
        out[valid_mask] = np.rint(np.clip(values[valid_mask], 0.0, 100.0)).astype(np.int8)
    return out.T[::-1, ::-1].flatten(order="C")


def is_zero_time(stamp: Time) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


class TraversabilityToMap(Node):
    def __init__(self) -> None:
        super().__init__("traversability_to_map")

        self.declare_parameter("grid_map_topic", "/elevation_mapping_node/elevation_map")
        self.declare_parameter("map_topic", "/traversability_map")
        self.declare_parameter("layer", "elevation")
        self.declare_parameter("frame_id", "")  # empty = inherit from GridMap
        self.declare_parameter("fallback_frame_id", "world")
        self.declare_parameter("unknown_value", -1)  # OccupancyGrid value for NaN
        self.declare_parameter("kernel_size", 3)
        self.declare_parameter("clear_below_m", 0.04)
        self.declare_parameter("accumulate_from_m", 0.05)
        self.declare_parameter("full_at_m", 0.10)
        self.declare_parameter("median_filter_size", 3)

        grid_topic = str(self.get_parameter("grid_map_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        self.layer = str(self.get_parameter("layer").value)
        self.frame_override = str(self.get_parameter("frame_id").value).strip()
        self.fallback_frame_id = str(self.get_parameter("fallback_frame_id").value).strip()
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.clear_below_m = float(self.get_parameter("clear_below_m").value)
        self.accumulate_from_m = float(self.get_parameter("accumulate_from_m").value)
        self.full_at_m = float(self.get_parameter("full_at_m").value)
        self.median_filter_size = int(self.get_parameter("median_filter_size").value)

        # Latched QoS so RViz receives the last map on connect.
        latch_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, latch_qos)
        self.sub = self.create_subscription(GridMap, grid_topic, self.on_grid, 10)
        self.latest_grid_msg: GridMap | None = None
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self._warned_missing_layer = False
        self._warned_decode = False
        self._warned_frame_override = False
        self._warned_empty_frame = False
        self._warned_empty_source_frame = False
        self._warned_zero_stamp = False
        self._warned_rotated_pose = False

        self.get_logger().info(
            f"traversability_to_map: {grid_topic} (elevation_layer='{self.layer}') -> {map_topic} "
            f"frame_override='{self.frame_override or '<inherit>'}' "
            f"range_kernel={self.kernel_size} thresholds=[<{self.clear_below_m:.3f}m free, "
            f"{self.accumulate_from_m:.3f}m start, {self.full_at_m:.3f}m full] "
            f"median_filter={self.median_filter_size}"
        )

    def on_grid(self, msg: GridMap) -> None:
        self.latest_grid_msg = msg
        self.process_grid(msg)

    def process_grid(self, msg: GridMap) -> None:
        if len(msg.layers) != len(msg.data):
            self.get_logger().warning("Received malformed GridMap message.")
            return

        if self.layer not in msg.layers:
            if not self._warned_missing_layer:
                self.get_logger().warning(
                    f"GridMap has no layer '{self.layer}'. "
                    f"Available: {list(msg.layers)}"
                )
                self._warned_missing_layer = True
            return
        self._warned_missing_layer = False

        idx = list(msg.layers).index(self.layer)
        try:
            arr = decode_layer(self.layer, msg.data[idx])  # (rows, cols), grid_map convention
        except Exception as exc:
            if not self._warned_decode:
                self.get_logger().error(f"Failed to decode layer '{self.layer}': {exc}")
                self._warned_decode = True
            return
        self._warned_decode = False

        elevation = unwrap_layer(arr, msg.outer_start_index, msg.inner_start_index)
        height_cells, width_cells = elevation.shape

        roughness = local_height_range_map(elevation, self.kernel_size)
        occupancy = roughness_to_occupancy(
            roughness,
            clear_below_m=self.clear_below_m,
            accumulate_from_m=self.accumulate_from_m,
            full_at_m=self.full_at_m,
        )
        occupancy = median_filter_nan(occupancy, self.median_filter_size)
        out = grid_map_to_occupancy_data(occupancy, self.unknown_value)

        og = OccupancyGrid()
        og.header = msg.header
        resolved_frame_id = self.frame_override or msg.header.frame_id.strip() or self.fallback_frame_id
        if not resolved_frame_id:
            if not self._warned_empty_frame:
                self.get_logger().error(
                    "GridMap header.frame_id is empty and no frame fallback is configured."
                )
                self._warned_empty_frame = True
            return
        self._warned_empty_frame = False

        if self.frame_override:
            og.header.frame_id = resolved_frame_id
            if not self._warned_frame_override and resolved_frame_id != msg.header.frame_id:
                self.get_logger().warning(
                    "frame_id override only changes the published frame name; "
                    "it does not transform the GridMap pose."
                )
                self._warned_frame_override = True
        else:
            og.header.frame_id = resolved_frame_id
            if not msg.header.frame_id.strip() and not self._warned_empty_source_frame:
                self.get_logger().warning(
                    f"GridMap header.frame_id is empty; using fallback frame '{resolved_frame_id}'."
                )
                self._warned_empty_source_frame = True
            elif msg.header.frame_id.strip():
                self._warned_empty_source_frame = False

        if is_zero_time(og.header.stamp):
            og.header.stamp = self.get_clock().now().to_msg()
            if not self._warned_zero_stamp:
                self.get_logger().warning(
                    "GridMap header.stamp is zero; using the node clock for OccupancyGrid timestamps."
                )
                self._warned_zero_stamp = True
        og.info.resolution = float(msg.info.resolution)
        og.info.width = width_cells
        og.info.height = height_cells
        og.info.map_load_time = og.header.stamp

        cx = float(msg.info.pose.position.x)
        cy = float(msg.info.pose.position.y)
        cz = float(msg.info.pose.position.z)
        half_extent = np.array(
            [
                0.5 * width_cells * og.info.resolution,
                0.5 * height_cells * og.info.resolution,
                0.0,
            ],
            dtype=np.float64,
        )
        rotation = quaternion_to_rotation_matrix(msg.info.pose.orientation)
        origin = np.array([cx, cy, cz], dtype=np.float64) - rotation @ half_extent
        og.info.origin.position.x = float(origin[0])
        og.info.origin.position.y = float(origin[1])
        og.info.origin.position.z = float(origin[2])
        og.info.origin.orientation.x = float(msg.info.pose.orientation.x)
        og.info.origin.orientation.y = float(msg.info.pose.orientation.y)
        og.info.origin.orientation.z = float(msg.info.pose.orientation.z)
        og.info.origin.orientation.w = float(msg.info.pose.orientation.w)

        if not self._warned_rotated_pose and not is_identity_quaternion(msg.info.pose.orientation):
            self.get_logger().warning(
                "Publishing a rotated OccupancyGrid origin from GridMap pose.orientation."
            )
            self._warned_rotated_pose = True

        # int8[] — tolist() returns Python ints, accepted by rclpy.
        og.data = out.tolist()

        self.map_pub.publish(og)

    def on_parameter_change(self, params: list[Parameter]) -> SetParametersResult:
        dynamic_param_names = {
            "kernel_size",
            "clear_below_m",
            "accumulate_from_m",
            "full_at_m",
            "median_filter_size",
        }
        updates = {param.name: param.value for param in params if param.name in dynamic_param_names}
        if not updates:
            return SetParametersResult(successful=True)

        try:
            next_kernel_size = self.kernel_size
            if "kernel_size" in updates:
                next_kernel_size = int(updates["kernel_size"])
            if next_kernel_size < 1 or next_kernel_size % 2 == 0:
                raise ValueError("kernel_size must be a positive odd integer.")

            next_clear_below_m = self.clear_below_m
            if "clear_below_m" in updates:
                next_clear_below_m = float(updates["clear_below_m"])

            next_accumulate_from_m = self.accumulate_from_m
            if "accumulate_from_m" in updates:
                next_accumulate_from_m = float(updates["accumulate_from_m"])

            next_full_at_m = self.full_at_m
            if "full_at_m" in updates:
                next_full_at_m = float(updates["full_at_m"])

            if not (0.0 <= next_clear_below_m <= next_accumulate_from_m < next_full_at_m):
                raise ValueError(
                    "Expected 0 <= clear_below_m <= accumulate_from_m < full_at_m."
                )

            next_median_filter_size = self.median_filter_size
            if "median_filter_size" in updates:
                next_median_filter_size = int(updates["median_filter_size"])
            if next_median_filter_size < 1 or next_median_filter_size % 2 == 0:
                raise ValueError("median_filter_size must be a positive odd integer.")
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self.kernel_size = next_kernel_size
        self.clear_below_m = next_clear_below_m
        self.accumulate_from_m = next_accumulate_from_m
        self.full_at_m = next_full_at_m
        self.median_filter_size = next_median_filter_size

        if self.latest_grid_msg is not None:
            self.process_grid(self.latest_grid_msg)

        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = TraversabilityToMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
