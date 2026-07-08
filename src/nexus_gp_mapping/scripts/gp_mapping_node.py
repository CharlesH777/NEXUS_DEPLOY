#!/usr/bin/env python3

import math
from typing import Optional

import gpytorch
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap, GridMapInfo
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

from gpytorch.distributions import MultivariateNormal
from gpytorch.kernels import InducingPointKernel, RQKernel, ScaleKernel
from gpytorch.means import LinearMean


class SparseGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x: torch.Tensor, train_y: torch.Tensor, likelihood, inducing_points: torch.Tensor) -> None:
        super().__init__(train_x, train_y, likelihood)
        # Linear trend mean: can absorb large-scale slopes so the kernel only
        # needs to model residual roughness. input_size=3 -> [x, y, 1].
        self.mean_module = LinearMean(input_size=3, bias=False)

        # Learnable lengthscale initialised large so the first iteration does
        # not over-smooth steep terrain; Adam will shrink it as needed.
        self.base_covar_module = ScaleKernel(
            RQKernel(lengthscale=torch.tensor([2.0, 2.0]), alpha=torch.tensor([2.0]))
        )
        self.covar_module = InducingPointKernel(
            self.base_covar_module,
            inducing_points=inducing_points,
            likelihood=likelihood,
        )

    def forward(self, x: torch.Tensor) -> MultivariateNormal:
        # Append a constant column so LinearMean acts as w0*x + w1*y + bias.
        x_aug = torch.cat(
            [x, torch.ones(x.shape[0], 1, dtype=x.dtype, device=x.device)], dim=-1
        )
        mean_x = self.mean_module(x_aug)
        covar_x = self.covar_module(x)
        return MultivariateNormal(mean_x, covar_x)


class GPMappingNode(Node):
    def __init__(self) -> None:
        super().__init__("gp_mapping_node")

        self.declare_parameter("input_topic", "/fastlio2/world_cloud")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("center_pose_topic", "/fastlio2/lio_odom")
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("fallback_center_on_cloud", True)
        self.declare_parameter("length_in_x", 10.0)
        self.declare_parameter("length_in_y", 10.0)
        self.declare_parameter("global_length_in_x", 30.0)
        self.declare_parameter("global_length_in_y", 30.0)
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("inducing_points", 500)
        self.declare_parameter("max_sensor_range", 5.0)
        self.declare_parameter("min_points", 200)
        self.declare_parameter("process_period_sec", 3.0)
        self.declare_parameter("training_iterations", 30)
        self.declare_parameter("gp_training_steps", 60)
        self.declare_parameter("robust_fit_iterations", 3)
        self.declare_parameter("ground_seed_cell_size", 0.5)
        self.declare_parameter("ground_cell_keep_size", 0.20)
        self.declare_parameter("robust_residual_threshold", 0.22)
        self.declare_parameter("robust_sigma_multiplier", 0.35)
        self.declare_parameter("ground_lower_margin", 0.30)
        self.declare_parameter("sigma_margin_cap", 2.0)
        self.declare_parameter("floating_reject_margin", 0.55)
        self.declare_parameter("floating_connectivity_radius", 0.60)
        self.declare_parameter("floating_slope_review_radius", 0.90)
        self.declare_parameter("floating_slope_min_neighbors", 18)
        self.declare_parameter("floating_slope_point_plane_tolerance", 0.12)
        self.declare_parameter("floating_slope_neighbor_rms_tolerance", 0.10)
        self.declare_parameter("floating_slope_similarity_tolerance", 0.22)
        self.declare_parameter("floating_slope_height_tolerance", 0.18)
        self.declare_parameter("publish_debug_layers", False)
        self.declare_parameter("visual_z_offset", 0.0)
        self.declare_parameter("retain_unobserved_cells", True)
        self.declare_parameter("retention_observation_radius", 0.35)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_frame_id = str(self.get_parameter("frame_id").value)
        self.center_pose_topic = str(self.get_parameter("center_pose_topic").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.fallback_center_on_cloud = bool(self.get_parameter("fallback_center_on_cloud").value)
        self.x_length = float(self.get_parameter("length_in_x").value)
        self.y_length = float(self.get_parameter("length_in_y").value)
        self.global_x_length = max(self.x_length, float(self.get_parameter("global_length_in_x").value))
        self.global_y_length = max(self.y_length, float(self.get_parameter("global_length_in_y").value))
        self.resolution = float(self.get_parameter("resolution").value)
        self.inducing_points = int(self.get_parameter("inducing_points").value)
        self.max_sensor_range = float(self.get_parameter("max_sensor_range").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.process_period_sec = float(self.get_parameter("process_period_sec").value)
        # training_iterations is kept for backward compatibility with the
        # MAP_SIM_GP_TRAINING_ITERATIONS env var; gp_training_steps is the one
        # actually used by train_gp_model. If the legacy value is set to
        # something other than the default, treat it as an override.
        legacy_iters = int(self.get_parameter("training_iterations").value)
        self.gp_training_steps = max(1, int(self.get_parameter("gp_training_steps").value))
        if legacy_iters != 30 and legacy_iters > 0:
            self.gp_training_steps = legacy_iters
        self.robust_fit_iterations = max(1, int(self.get_parameter("robust_fit_iterations").value))
        self.ground_seed_cell_size = max(0.05, float(self.get_parameter("ground_seed_cell_size").value))
        self.ground_cell_keep_size = max(0.05, float(self.get_parameter("ground_cell_keep_size").value))
        self.robust_residual_threshold = max(
            0.0, float(self.get_parameter("robust_residual_threshold").value)
        )
        self.robust_sigma_multiplier = max(0.0, float(self.get_parameter("robust_sigma_multiplier").value))
        self.ground_lower_margin = max(0.0, float(self.get_parameter("ground_lower_margin").value))
        self.sigma_margin_cap = max(0.0, float(self.get_parameter("sigma_margin_cap").value))
        self.floating_reject_margin = max(0.0, float(self.get_parameter("floating_reject_margin").value))
        self.floating_connectivity_radius = max(
            0.05, float(self.get_parameter("floating_connectivity_radius").value)
        )
        self.floating_slope_review_radius = max(
            0.05, float(self.get_parameter("floating_slope_review_radius").value)
        )
        self.floating_slope_min_neighbors = max(
            3, int(self.get_parameter("floating_slope_min_neighbors").value)
        )
        self.floating_slope_point_plane_tolerance = max(
            0.0, float(self.get_parameter("floating_slope_point_plane_tolerance").value)
        )
        self.floating_slope_neighbor_rms_tolerance = max(
            0.0, float(self.get_parameter("floating_slope_neighbor_rms_tolerance").value)
        )
        self.floating_slope_similarity_tolerance = max(
            0.0, float(self.get_parameter("floating_slope_similarity_tolerance").value)
        )
        self.floating_slope_height_tolerance = max(
            0.0, float(self.get_parameter("floating_slope_height_tolerance").value)
        )
        self.publish_debug_layers = bool(self.get_parameter("publish_debug_layers").value)
        self.visual_z_offset = float(self.get_parameter("visual_z_offset").value)
        self.retain_unobserved_cells = bool(self.get_parameter("retain_unobserved_cells").value)
        self.retention_observation_radius = float(
            self.get_parameter("retention_observation_radius").value
        )

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.latest_cloud: Optional[PointCloud2] = None
        self.processing = False
        self.latest_center_xy: Optional[tuple[float, float]] = None
        self.latest_center_stamp_sec: Optional[float] = None
        self.global_center_xy: Optional[tuple[float, float]] = None
        self.cached_cells: dict[tuple[int, int], tuple[float, float, float]] = {}

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        qos = 10
        self.cloud_sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.pose_sub = None
        if self.center_pose_topic:
            self.pose_sub = self.create_subscription(
                Odometry, self.center_pose_topic, self.pose_callback, qos
            )
        self.elevation_pub = self.create_publisher(PointCloud2, "/gp_navigation/elevation", qos)
        self.magnitude_pub = self.create_publisher(PointCloud2, "/gp_navigation/magnitude", qos)
        self.uncertainty_pub = self.create_publisher(PointCloud2, "/gp_navigation/uncertainty", qos)
        self.grid_map_pub = self.create_publisher(GridMap, "/gp_navigation/elevation_grid_map", qos)

        self.gradx_pub = None
        self.grady_pub = None
        if self.publish_debug_layers:
            self.gradx_pub = self.create_publisher(PointCloud2, "/gp_navigation/x_grad", qos)
            self.grady_pub = self.create_publisher(PointCloud2, "/gp_navigation/y_grad", qos)

        self.timer = self.create_timer(self.process_period_sec, self.process_latest_cloud)

        self.get_logger().info(
            f"GP mapping node ready: input={self.input_topic}, frame={self.output_frame_id}, "
            f"center_pose_topic={self.center_pose_topic or '<none>'}, "
            f"device={self.device.type}, resolution={self.resolution}, "
            f"window=({self.x_length}, {self.y_length}), global=({self.global_x_length}, {self.global_y_length})"
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def pose_callback(self, msg: Odometry) -> None:
        self.latest_center_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.latest_center_stamp_sec = self.stamp_to_sec(msg.header.stamp)

    def resolve_grid_center(self, cloud_msg: PointCloud2, points: np.ndarray) -> tuple[float, float]:
        cloud_stamp_sec = self.stamp_to_sec(cloud_msg.header.stamp)
        if self.latest_center_xy is not None and self.latest_center_stamp_sec is not None:
            diff_sec = abs(cloud_stamp_sec - self.latest_center_stamp_sec)
            if self.pose_timeout_sec <= 0.0 or diff_sec <= self.pose_timeout_sec:
                return self.latest_center_xy

        if self.fallback_center_on_cloud and points.shape[0] > 0:
            center_xy = np.mean(points[:, :2], axis=0)
            return float(center_xy[0]), float(center_xy[1])

        return 0.0, 0.0

    def select_ground_seeds(self, points: np.ndarray) -> np.ndarray:
        """Keep one low point per coarse XY cell for the first GP pass."""
        if points.size == 0:
            return points

        cell_size = max(self.ground_seed_cell_size, self.resolution * 2.0)
        selected: dict[tuple[int, int], tuple[float, float, float]] = {}

        for x_value, y_value, z_value in points:
            cell_key = (
                int(math.floor(float(x_value) / cell_size)),
                int(math.floor(float(y_value) / cell_size)),
            )
            candidate = (float(x_value), float(y_value), float(z_value))
            current = selected.get(cell_key)
            if current is None or candidate[2] < current[2]:
                selected[cell_key] = candidate

        seed_points = np.asarray(list(selected.values()), dtype=np.float32)
        if seed_points.size == 0:
            return points
        return seed_points

    def keep_lowest_points_per_xy_cell(
        self,
        points: np.ndarray,
        candidate_mask: np.ndarray,
        cell_size: float,
    ) -> np.ndarray:
        if points.size == 0 or not np.any(candidate_mask):
            return candidate_mask

        kept_mask = np.zeros_like(candidate_mask, dtype=bool)
        selected_by_cell: dict[tuple[int, int], tuple[int, float]] = {}

        for point_idx in np.flatnonzero(candidate_mask):
            x_value, y_value, z_value = points[point_idx]
            cell_key = (
                int(math.floor(float(x_value) / cell_size)),
                int(math.floor(float(y_value) / cell_size)),
            )
            current = selected_by_cell.get(cell_key)
            if current is None or float(z_value) < current[1]:
                selected_by_cell[cell_key] = (int(point_idx), float(z_value))

        for point_idx, _ in selected_by_cell.values():
            kept_mask[point_idx] = True

        return kept_mask

    def select_inducing_points(self, xy: torch.Tensor) -> torch.Tensor:
        """Farthest-point sampling for spatially uniform inducing points.

        Falls back to random sampling if FPS fails (too few points / CUDA path).
        Guarantees coverage in both x and y, unlike the old argsort-by-x scheme.
        """
        n = xy.shape[0]
        m = min(self.inducing_points, n)
        if m <= 0:
            return xy[:1].clone()
        if m >= n:
            return xy.clone()

        device = xy.device
        start = torch.randint(0, n, (1,), device=device)
        selected_idx = [int(start.item())]
        # Squared distance from each point to the closest already-selected point.
        min_dist_sq = torch.sum((xy - xy[selected_idx[0]]) ** 2, dim=1)

        for _ in range(m - 1):
            next_idx = int(torch.argmax(min_dist_sq).item())
            selected_idx.append(next_idx)
            new_dist_sq = torch.sum((xy - xy[next_idx]) ** 2, dim=1)
            min_dist_sq = torch.minimum(min_dist_sq, new_dist_sq)

        return xy[torch.tensor(selected_idx, device=device, dtype=torch.long)]

    @staticmethod
    def init_linear_mean(model: "SparseGPModel", train_x: torch.Tensor, train_y: torch.Tensor) -> None:
        """Warm-start the LinearMean weights from a least-squares plane fit.

        z ~= w0*x + w1*y + bias. This lets the GP start from a reasonable slope
        instead of forcing the kernel to learn the entire trend in a few steps.
        """
        try:
            design = torch.cat(
                [train_x, torch.ones(train_x.shape[0], 1, dtype=train_x.dtype, device=train_x.device)],
                dim=1,
            )
            sol = torch.linalg.lstsq(design, train_y.unsqueeze(-1)).solution  # (3, 1)
            with torch.no_grad():
                model.mean_module.weights.copy_(sol)
        except Exception:
            pass

    def train_gp_model(self, train_points: np.ndarray):
        train_x_tensor = torch.tensor(train_points[:, :2], dtype=torch.float32, device=self.device)
        train_y_tensor = torch.tensor(train_points[:, 2], dtype=torch.float32, device=self.device)

        inducing = self.select_inducing_points(train_x_tensor).detach()
        likelihood = gpytorch.likelihoods.GaussianLikelihood()
        # Small observation noise so steep-slope points are not treated as noise.
        with torch.no_grad():
            likelihood.noise = 1e-2
        model = SparseGPModel(train_x_tensor, train_y_tensor, likelihood, inducing)
        self.init_linear_mean(model, train_x_tensor, train_y_tensor)
        if self.device.type == "cuda":
            model = model.cuda()
            likelihood = likelihood.cuda()

        model.train()
        likelihood.train()
        optimizer = torch.optim.Adam(model.parameters(), lr=0.05)
        mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

        steps = max(1, self.gp_training_steps)
        for _ in range(steps):
            optimizer.zero_grad()
            output = model(train_x_tensor)
            loss = -mll(output, train_y_tensor).mean()
            loss.backward()
            optimizer.step()

        return model, likelihood

    def predict_mean_variance(
        self,
        model,
        likelihood,
        xy_points: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if xy_points.size == 0:
            return np.empty((0,), dtype=np.float32), np.empty((0,), dtype=np.float32)

        test_x_tensor = torch.tensor(xy_points, dtype=torch.float32, device=self.device)
        model.eval()
        likelihood.eval()
        with torch.no_grad():
            predictions = likelihood(model(test_x_tensor))

        return (
            predictions.mean.detach().cpu().numpy(),
            predictions.variance.detach().cpu().numpy(),
        )

    def compute_disconnected_floating_mask(
        self,
        points: np.ndarray,
        residual: np.ndarray,
        upper_margin: np.ndarray,
        floating_margin: np.ndarray,
        excluded_mask: np.ndarray,
    ) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0,), dtype=bool)

        strict_anchor_margin = np.minimum(upper_margin, self.ground_lower_margin * 0.5)
        anchor_mask = (~excluded_mask) & (residual <= strict_anchor_margin)
        floating_candidate_mask = (~excluded_mask) & (residual > floating_margin)
        relevant_mask = ~excluded_mask

        if not np.any(floating_candidate_mask) or not np.any(anchor_mask):
            return np.zeros((points.shape[0],), dtype=bool)

        cell_size = self.floating_connectivity_radius
        xyz_cells = np.floor(points / cell_size).astype(np.int32)
        cell_index: dict[tuple[int, int, int], list[int]] = {}
        for idx in np.flatnonzero(relevant_mask):
            cell = xyz_cells[idx]
            cell_key = (int(cell[0]), int(cell[1]), int(cell[2]))
            bucket = cell_index.get(cell_key)
            if bucket is None:
                bucket = []
                cell_index[cell_key] = bucket
            bucket.append(int(idx))

        radius_sq = self.floating_connectivity_radius * self.floating_connectivity_radius
        visited = np.zeros((points.shape[0],), dtype=bool)
        disconnected_mask = np.zeros((points.shape[0],), dtype=bool)

        for start_idx in np.flatnonzero(relevant_mask):
            if visited[start_idx]:
                continue

            visited[start_idx] = True
            stack = [int(start_idx)]
            component: list[int] = []
            has_anchor = False

            while stack:
                current_idx = stack.pop()
                component.append(current_idx)
                if anchor_mask[current_idx]:
                    has_anchor = True

                current_cell = xyz_cells[current_idx]
                for d_x in (-1, 0, 1):
                    for d_y in (-1, 0, 1):
                        for d_z in (-1, 0, 1):
                            neighbor_key = (
                                int(current_cell[0] + d_x),
                                int(current_cell[1] + d_y),
                                int(current_cell[2] + d_z),
                            )
                            for neighbor_idx in cell_index.get(neighbor_key, []):
                                if visited[neighbor_idx]:
                                    continue
                                delta = points[neighbor_idx] - points[current_idx]
                                if float(np.dot(delta, delta)) > radius_sq:
                                    continue
                                visited[neighbor_idx] = True
                                stack.append(neighbor_idx)

            anchor_support = int(np.count_nonzero(anchor_mask[component]))
            if anchor_support >= 2:
                continue

            for point_idx in component:
                if floating_candidate_mask[point_idx]:
                    disconnected_mask[point_idx] = True

        return disconnected_mask

    @staticmethod
    def fit_xy_plane(xy: np.ndarray, z_values: np.ndarray) -> np.ndarray:
        design = np.column_stack((xy[:, 0], xy[:, 1], np.ones((xy.shape[0],), dtype=np.float32)))
        coeffs, _, _, _ = np.linalg.lstsq(design, z_values, rcond=None)
        return coeffs.astype(np.float32)

    def recover_slope_consistent_points(
        self,
        points: np.ndarray,
        predicted_mean: np.ndarray,
        residual: np.ndarray,
        floating_mask: np.ndarray,
    ) -> np.ndarray:
        keep_mask = np.zeros((points.shape[0],), dtype=bool)
        candidate_indices = np.flatnonzero(floating_mask)
        if candidate_indices.size == 0:
            return keep_mask

        review_radius_sq = self.floating_slope_review_radius * self.floating_slope_review_radius

        for point_idx in candidate_indices:
            deltas_xy = points[:, :2] - points[point_idx, :2]
            neighbor_mask = np.sum(np.square(deltas_xy), axis=1) <= review_radius_sq
            neighbor_mask &= residual >= -self.ground_lower_margin

            if int(np.count_nonzero(neighbor_mask)) < self.floating_slope_min_neighbors:
                continue

            neighbor_xy = points[neighbor_mask, :2]
            neighbor_z = points[neighbor_mask, 2]
            neighbor_gp = predicted_mean[neighbor_mask]

            raw_plane = self.fit_xy_plane(neighbor_xy, neighbor_z)
            gp_plane = self.fit_xy_plane(neighbor_xy, neighbor_gp)

            raw_plane_neighbor = neighbor_xy[:, 0] * raw_plane[0] + neighbor_xy[:, 1] * raw_plane[1] + raw_plane[2]
            neighbor_rms = float(np.sqrt(np.mean(np.square(neighbor_z - raw_plane_neighbor))))

            point_xy = points[point_idx, :2]
            raw_plane_point = float(point_xy[0] * raw_plane[0] + point_xy[1] * raw_plane[1] + raw_plane[2])
            gp_plane_point = float(point_xy[0] * gp_plane[0] + point_xy[1] * gp_plane[1] + gp_plane[2])

            point_plane_residual = abs(float(points[point_idx, 2]) - raw_plane_point)
            slope_gap = float(np.linalg.norm(raw_plane[:2] - gp_plane[:2]))
            height_gap = abs(raw_plane_point - gp_plane_point)

            if point_plane_residual > self.floating_slope_point_plane_tolerance:
                continue
            if neighbor_rms > self.floating_slope_neighbor_rms_tolerance:
                continue
            if slope_gap > self.floating_slope_similarity_tolerance:
                continue
            if height_gap > self.floating_slope_height_tolerance:
                continue

            keep_mask[point_idx] = True

        return keep_mask

    def fit_gp_robustly(self, candidate_points: np.ndarray):
        raw_points = np.asarray(candidate_points, dtype=np.float32)
        if raw_points.shape[0] < 2:
            raise ValueError("Not enough points to fit GP")

        current_points = raw_points
        if current_points.shape[0] >= 16:
            current_points = self.select_ground_seeds(current_points)

        if current_points.shape[0] < 2:
            current_points = np.asarray(candidate_points, dtype=np.float32)

        model = None
        likelihood = None

        for round_index in range(self.robust_fit_iterations):
            model, likelihood = self.train_gp_model(current_points)
            mean, variance = self.predict_mean_variance(model, likelihood, raw_points[:, :2])
            residual = raw_points[:, 2] - mean
            sigma = np.sqrt(np.maximum(variance, 1e-6))
            sigma_margin = self.robust_sigma_multiplier * np.minimum(sigma, self.sigma_margin_cap)
            upper_margin = self.robust_residual_threshold + sigma_margin
            floating_margin = self.floating_reject_margin + sigma_margin

            high_residual_mask = residual > upper_margin
            low_residual_mask = residual < -self.ground_lower_margin
            floating_mask = self.compute_disconnected_floating_mask(
                raw_points,
                residual,
                upper_margin,
                floating_margin,
                low_residual_mask,
            )
            slope_keep_mask = self.recover_slope_consistent_points(
                raw_points,
                mean,
                residual,
                floating_mask,
            )
            floating_mask &= ~slope_keep_mask
            ground_candidate_mask = (~low_residual_mask) & (~floating_mask)
            lowest_cell_mask = self.keep_lowest_points_per_xy_cell(
                raw_points,
                ground_candidate_mask,
                self.ground_cell_keep_size,
            )
            if np.count_nonzero(lowest_cell_mask) >= 2:
                ground_candidate_mask = lowest_cell_mask

            ground_candidates = raw_points[ground_candidate_mask]
            if ground_candidates.shape[0] < 2:
                self.get_logger().warning(
                    f"GP robust fit round {round_index + 1}/{self.robust_fit_iterations}: only "
                    f"{ground_candidates.shape[0]} ground candidates remain; keeping previous fit set"
                )
                break

            next_points = ground_candidates
            if next_points.shape[0] >= 16:
                next_points = self.select_ground_seeds(next_points)
            if next_points.shape[0] < 2:
                next_points = ground_candidates

            self.get_logger().info(
                f"GP robust fit round {round_index + 1}/{self.robust_fit_iterations}: "
                f"train={current_points.shape[0]}, candidates={ground_candidates.shape[0]}, "
                f"next_fit={next_points.shape[0]}, high_residual={int(np.count_nonzero(high_residual_mask))}, "
                f"floating={int(np.count_nonzero(floating_mask))}, "
                f"slope_kept={int(np.count_nonzero(slope_keep_mask))}, "
                f"below_ground={int(np.count_nonzero(low_residual_mask))}"
            )

            if round_index == self.robust_fit_iterations - 1:
                break

            if next_points.shape == current_points.shape and np.allclose(next_points, current_points):
                break
            current_points = next_points

        if model is None or likelihood is None:
            raise RuntimeError("GP training failed")

        return model, likelihood, current_points

    def process_latest_cloud(self) -> None:
        if self.processing or self.latest_cloud is None:
            return

        self.processing = True
        try:
            self.run_gp_mapping(self.latest_cloud)
        except Exception as exc:
            self.get_logger().error(f"GP mapping failed: {exc}")
        finally:
            self.processing = False

    def run_gp_mapping(self, cloud_msg: PointCloud2) -> None:
        point_iter = point_cloud2.read_points(
            cloud_msg,
            field_names=["x", "y", "z"],
            skip_nans=True,
        )
        raw_points = list(point_iter)
        if not raw_points:
            self.get_logger().warning("Skipping GP mapping: empty point cloud")
            return

        first_point = raw_points[0]
        if hasattr(first_point, "dtype") and getattr(first_point.dtype, "names", None):
            points = np.asarray(
                [(float(p["x"]), float(p["y"]), float(p["z"])) for p in raw_points],
                dtype=np.float32,
            )
        else:
            points = np.asarray(raw_points, dtype=np.float32)

        if points.size == 0:
            self.get_logger().warning("Skipping GP mapping: empty point cloud")
            return

        points = np.asarray(points, dtype=np.float32)
        center_x, center_y = self.resolve_grid_center(cloud_msg, points)
        if self.global_center_xy is None:
            self.global_center_xy = (center_x, center_y)
            self.get_logger().info(
                f"Locked global GP canvas center at ({center_x:.3f}, {center_y:.3f})"
            )

        center_xy = np.asarray([center_x, center_y], dtype=np.float32)
        distances = np.linalg.norm(points[:, :2] - center_xy, axis=1)
        points = points[distances <= self.max_sensor_range]

        if points.shape[0] < self.min_points:
            self.get_logger().warning(
                f"Skipping GP mapping: only {points.shape[0]} points within {self.max_sensor_range}m"
            )
            return

        points = points[np.argsort(points[:, 0])]
        window_grid = self.make_sampling_grid(center_x, center_y, self.x_length, self.y_length)
        model, likelihood, fit_points = self.fit_gp_robustly(points)
        self.get_logger().info(
            f"GP robust fit finished: raw={points.shape[0]}, final_fit={fit_points.shape[0]}, "
            f"window_grid={window_grid.shape[0]}"
        )

        test_x_tensor = torch.tensor(window_grid, dtype=torch.float32, device=self.device, requires_grad=True)

        model.eval()
        likelihood.eval()
        with torch.autograd.set_grad_enabled(True):
            preds = likelihood(model(test_x_tensor))

        mean = preds.mean.detach().cpu().numpy()
        variance = preds.variance.detach().cpu().numpy()

        grad_one_like = torch.ones_like(preds.mean)
        grad_mean = torch.autograd.grad(
            preds.mean,
            test_x_tensor,
            grad_outputs=grad_one_like,
            retain_graph=True,
        )[0].detach().cpu().numpy()
        grad_var = torch.autograd.grad(
            preds.variance,
            test_x_tensor,
            grad_outputs=grad_one_like,
            retain_graph=True,
        )[0].detach().cpu().numpy()

        grad_mean_mag = np.sqrt(np.square(grad_mean[:, 0]) + np.square(grad_mean[:, 1])) - 0.4

        merged_mean, merged_variance, merged_magnitude = self.merge_with_cached_cells(
            window_grid,
            points,
            mean,
            variance,
            grad_mean_mag,
        )
        if self.retain_unobserved_cells:
            mean = merged_mean
            variance = merged_variance
            grad_mean_mag = merged_magnitude

        global_center_x, global_center_y = self.global_center_xy
        global_grid = self.make_sampling_grid(
            global_center_x,
            global_center_y,
            self.global_x_length,
            self.global_y_length,
        )
        global_mean, global_variance, global_magnitude = self.compose_cached_layers(global_grid)

        header = Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = cloud_msg.header.frame_id or self.output_frame_id
        display_mean = global_mean + self.visual_z_offset

        self.elevation_pub.publish(
            self.make_pointcloud(
                header,
                global_grid,
                display_mean,
                global_variance,
            )
        )

        self.magnitude_pub.publish(
            self.make_pointcloud(
                header,
                global_grid,
                display_mean,
                global_magnitude,
            )
        )

        self.uncertainty_pub.publish(
            self.make_pointcloud(
                header,
                global_grid,
                display_mean,
                global_variance,
            )
        )
        self.grid_map_pub.publish(
            self.make_grid_map(
                header,
                global_mean,
                global_variance,
                global_magnitude,
                global_center_x,
                global_center_y,
                self.global_x_length,
                self.global_y_length,
            )
        )

        if self.publish_debug_layers and self.gradx_pub and self.grady_pub:
            self.gradx_pub.publish(
                self.make_pointcloud(header, window_grid, grad_mean[:, 0], grad_var[:, 0])
            )
            self.grady_pub.publish(
                self.make_pointcloud(header, window_grid, grad_mean[:, 1], grad_var[:, 1])
            )

        self.get_logger().info(
            f"Published GP map from {points.shape[0]} points on {self.device.type}: "
            f"window={window_grid.shape[0]} cells, global={global_grid.shape[0]} cells"
        )

    def merge_with_cached_cells(
        self,
        grid_xy: np.ndarray,
        observed_points: np.ndarray,
        mean: np.ndarray,
        variance: np.ndarray,
        magnitude: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        observed_mask = self.compute_observed_mask(
            grid_xy,
            observed_points,
            self.x_length,
            self.y_length,
        )

        merged_mean = np.asarray(mean, dtype=np.float32).copy()
        merged_variance = np.asarray(variance, dtype=np.float32).copy()
        merged_magnitude = np.asarray(magnitude, dtype=np.float32).copy()

        for idx, (x_value, y_value) in enumerate(grid_xy):
            cell_key = self.make_cell_key(float(x_value), float(y_value))
            if observed_mask[idx] or cell_key not in self.cached_cells:
                if not self.is_within_global_canvas(float(x_value), float(y_value)):
                    continue
                self.cached_cells[cell_key] = (
                    float(merged_mean[idx]),
                    float(merged_variance[idx]),
                    float(merged_magnitude[idx]),
                )
                continue

            cached_mean, cached_variance, cached_magnitude = self.cached_cells[cell_key]
            merged_mean[idx] = cached_mean
            merged_variance[idx] = cached_variance
            merged_magnitude[idx] = cached_magnitude

        return merged_mean, merged_variance, merged_magnitude

    def compute_observed_mask(
        self,
        grid_xy: np.ndarray,
        observed_points: np.ndarray,
        length_x: float,
        length_y: float,
    ) -> np.ndarray:
        observed_mask = np.zeros(grid_xy.shape[0], dtype=bool)
        if observed_points.size == 0:
            return observed_mask

        x_min = float(np.min(grid_xy[:, 0]))
        y_min = float(np.min(grid_xy[:, 1]))
        rows = int(round(length_y / self.resolution))
        cols = int(round(length_x / self.resolution))
        cell_radius = max(0, int(math.ceil(self.retention_observation_radius / self.resolution)))

        grid_indices = np.floor((observed_points[:, :2] - np.array([x_min, y_min])) / self.resolution).astype(int)
        valid_mask = (
            (grid_indices[:, 0] >= 0)
            & (grid_indices[:, 0] < cols)
            & (grid_indices[:, 1] >= 0)
            & (grid_indices[:, 1] < rows)
        )
        valid_indices = grid_indices[valid_mask]

        for col_idx, row_idx in valid_indices:
            for d_col in range(-cell_radius, cell_radius + 1):
                for d_row in range(-cell_radius, cell_radius + 1):
                    target_col = col_idx + d_col
                    target_row = row_idx + d_row
                    if target_col < 0 or target_col >= cols or target_row < 0 or target_row >= rows:
                        continue
                    flat_index = target_col * rows + target_row
                    observed_mask[flat_index] = True

        return observed_mask

    def compose_cached_layers(
        self,
        grid_xy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        mean = np.full((grid_xy.shape[0],), np.nan, dtype=np.float32)
        variance = np.full((grid_xy.shape[0],), np.nan, dtype=np.float32)
        magnitude = np.full((grid_xy.shape[0],), np.nan, dtype=np.float32)

        for idx, (x_value, y_value) in enumerate(grid_xy):
            cell_key = self.make_cell_key(float(x_value), float(y_value))
            cached = self.cached_cells.get(cell_key)
            if cached is None:
                continue

            mean[idx], variance[idx], magnitude[idx] = cached

        return mean, variance, magnitude

    def is_within_global_canvas(self, x_value: float, y_value: float) -> bool:
        if self.global_center_xy is None:
            return True

        center_x, center_y = self.global_center_xy
        x_half = self.global_x_length / 2.0
        y_half = self.global_y_length / 2.0
        return (
            center_x - x_half <= x_value < center_x + x_half
            and center_y - y_half <= y_value < center_y + y_half
        )

    def make_cell_key(self, x_value: float, y_value: float) -> tuple[int, int]:
        return (
            int(round(x_value / self.resolution)),
            int(round(y_value / self.resolution)),
        )

    def make_sampling_grid(
        self,
        center_x: float,
        center_y: float,
        length_x: Optional[float] = None,
        length_y: Optional[float] = None,
    ) -> np.ndarray:
        x_length = self.x_length if length_x is None else float(length_x)
        y_length = self.y_length if length_y is None else float(length_y)
        x_half = x_length / 2.0
        y_half = y_length / 2.0
        x_samples = np.arange(center_x - x_half, center_x + x_half, self.resolution, dtype=np.float32)
        y_samples = np.arange(center_y - y_half, center_y + y_half, self.resolution, dtype=np.float32)
        return np.array(np.meshgrid(x_samples, y_samples)).T.reshape(-1, 2)

    def make_grid_map(
        self,
        header: Header,
        mean: np.ndarray,
        variance: np.ndarray,
        magnitude: np.ndarray,
        center_x: float,
        center_y: float,
        length_x: float,
        length_y: float,
    ) -> GridMap:
        rows = int(round(length_y / self.resolution))
        cols = int(round(length_x / self.resolution))

        elevation = np.asarray(mean, dtype=np.float32).reshape((cols, rows), order="C").T
        variance_layer = np.asarray(variance, dtype=np.float32).reshape((cols, rows), order="C").T
        magnitude_layer = np.asarray(magnitude, dtype=np.float32).reshape((cols, rows), order="C").T

        msg = GridMap()
        msg.header = header
        msg.info = GridMapInfo()
        msg.info.resolution = float(self.resolution)
        msg.info.length_x = float(length_x)
        msg.info.length_y = float(length_y)
        msg.info.pose = Pose()
        msg.info.pose.position.x = float(center_x)
        msg.info.pose.position.y = float(center_y)
        msg.info.pose.orientation.w = 1.0
        msg.layers = ["elevation", "variance", "magnitude"]
        msg.basic_layers = ["elevation"]
        msg.data = [
            self.encode_layer_to_multiarray(elevation),
            self.encode_layer_to_multiarray(variance_layer),
            self.encode_layer_to_multiarray(magnitude_layer),
        ]
        msg.outer_start_index = 0
        msg.inner_start_index = 0
        return msg

    def encode_layer_to_multiarray(self, array: np.ndarray) -> Float32MultiArray:
        arr = np.asarray(array, dtype=np.float32)
        rows, cols = arr.shape
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim.append(
            MultiArrayDimension(label="column_index", size=cols, stride=rows * cols)
        )
        msg.layout.dim.append(
            MultiArrayDimension(label="row_index", size=rows, stride=rows)
        )
        msg.data = arr.flatten(order="F").tolist()
        return msg

    def make_pointcloud(
        self,
        header: Header,
        grid_xy: np.ndarray,
        z_values: np.ndarray,
        intensity_values: np.ndarray,
    ) -> PointCloud2:
        z_array = np.asarray(z_values, dtype=np.float32).reshape(-1)
        intensity_array = np.asarray(intensity_values, dtype=np.float32).reshape(-1)
        valid_mask = np.isfinite(z_array) & np.isfinite(intensity_array)
        if not np.any(valid_mask):
            return point_cloud2.create_cloud(header, self.fields, [])

        xyz_i = np.column_stack(
            (
                grid_xy[valid_mask, 0].reshape(-1, 1),
                grid_xy[valid_mask, 1].reshape(-1, 1),
                z_array[valid_mask].reshape(-1, 1),
                intensity_array[valid_mask].reshape(-1, 1),
            )
        )
        return point_cloud2.create_cloud(header, self.fields, xyz_i.tolist())


def main() -> None:
    rclpy.init()
    node = GPMappingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
