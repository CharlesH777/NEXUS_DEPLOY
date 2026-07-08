#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double quaternion_to_yaw(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string normalize_frame_id(const std::string & frame_id, const std::string & fallback = "")
{
  std::string out = frame_id;
  while (!out.empty() && out.front() == '/') {
    out.erase(out.begin());
  }
  return out.empty() ? fallback : out;
}

float normalize_linear(float value, float start, float stop)
{
  if (stop <= start) {
    return value >= stop ? 1.0f : 0.0f;
  }
  const float scaled = (value - start) / (stop - start);
  return std::clamp(scaled, 0.0f, 1.0f);
}

size_t idx2(size_t row, size_t col, size_t cols)
{
  return row * cols + col;
}

int positive_mod(int value, int modulus)
{
  const int result = value % modulus;
  return result < 0 ? result + modulus : result;
}

struct Control
{
  float vx{0.0f};
  float vy{0.0f};
  float wz{0.0f};
};

struct Pose2
{
  float x{0.0f};
  float y{0.0f};
  float yaw{0.0f};
};

struct PlanarTransform
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GoalState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{"world"};
};

struct PoseState
{
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{"world"};
};

struct RobotState
{
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{"world"};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct DecodedLayer
{
  int rows{0};
  int cols{0};
  std::vector<float> values;
};

DecodedLayer decode_multiarray_to_rows_cols(
  const std::string & name,
  const std_msgs::msg::Float32MultiArray & array_msg)
{
  const auto & data = array_msg.data;
  const auto & dims = array_msg.layout.dim;
  int rows = 0;
  int cols = 0;
  std::vector<float> values;

  if (dims.size() >= 2U && !dims[0].label.empty() && !dims[1].label.empty()) {
    const auto & label0 = dims[0].label;
    const auto & label1 = dims[1].label;
    if (label0 == "row_index" && label1 == "column_index") {
      rows = static_cast<int>(dims[0].size != 0U ? dims[0].size : 1U);
      cols = static_cast<int>(dims[1].size != 0U ? dims[1].size : data.size() / rows);
      if (rows * cols != static_cast<int>(data.size())) {
        throw std::runtime_error("Layer '" + name + "' has inconsistent layout metadata.");
      }
      values.assign(data.begin(), data.end());
      return {rows, cols, values};
    }

    if (label0 == "column_index" && label1 == "row_index") {
      cols = static_cast<int>(dims[0].size != 0U ? dims[0].size : 1U);
      rows = static_cast<int>(dims[1].size != 0U ? dims[1].size : data.size() / cols);
      if (rows * cols != static_cast<int>(data.size())) {
        throw std::runtime_error("Layer '" + name + "' has inconsistent layout metadata.");
      }
      values.assign(rows * cols, 0.0f);
      for (int c = 0; c < cols; ++c) {
        for (int r = 0; r < rows; ++r) {
          values[idx2(r, c, cols)] = data[static_cast<size_t>(c * rows + r)];
        }
      }
      return {rows, cols, values};
    }
  }

  if (!dims.empty()) {
    cols = static_cast<int>(dims[0].size != 0U ? dims[0].size : 1U);
    rows = dims.size() > 1U ?
      static_cast<int>(dims[1].size) :
      static_cast<int>(cols != 0 ? data.size() / cols : data.size());
  } else {
    cols = static_cast<int>(std::sqrt(static_cast<double>(data.size())));
    rows = cols;
  }
  if (rows * cols != static_cast<int>(data.size())) {
    throw std::runtime_error("Layer '" + name + "' has inconsistent layout metadata.");
  }
  values.assign(data.begin(), data.end());
  return {rows, cols, values};
}

std::vector<float> roll_layer(
  const std::vector<float> & input,
  int rows,
  int cols,
  int outer_start_index,
  int inner_start_index)
{
  if (outer_start_index == 0 && inner_start_index == 0) {
    return input;
  }
  std::vector<float> output(input.size(), 0.0f);
  for (int r = 0; r < rows; ++r) {
    const int src_r = positive_mod(r + outer_start_index, rows);
    for (int c = 0; c < cols; ++c) {
      const int src_c = positive_mod(c + inner_start_index, cols);
      output[idx2(r, c, cols)] = input[idx2(src_r, src_c, cols)];
    }
  }
  return output;
}

struct MapSnapshot
{
  double stamp_sec{0.0};
  std::string frame_id;
  double center_x{0.0};
  double center_y{0.0};
  double yaw{0.0};
  double length_x{0.0};
  double length_y{0.0};
  double resolution{0.1};
  int rows{0};
  int cols{0};
  std::vector<float> base_cost;
  std::vector<uint8_t> valid_mask;
  std::vector<float> slope_deg;
  std::vector<float> variance;
  std::vector<float> raw_traversability;
  bool has_variance{false};
  bool has_raw_traversability{false};

  bool sample(double x, double y, float * cost, bool * unknown, float * raw_trav) const
  {
    const double dx = x - center_x;
    const double dy = y - center_y;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    const int col = static_cast<int>(std::floor((local_x + 0.5 * length_x) / resolution));
    const int row = static_cast<int>(std::floor((local_y + 0.5 * length_y) / resolution));
    if (row < 0 || row >= rows || col < 0 || col >= cols) {
      *cost = 0.0f;
      *unknown = true;
      if (raw_trav) {
        *raw_trav = std::numeric_limits<float>::quiet_NaN();
      }
      return false;
    }
    const size_t index = idx2(static_cast<size_t>(row), static_cast<size_t>(col), cols);
    *cost = base_cost[index];
    *unknown = valid_mask[index] == 0U;
    if (raw_trav) {
      *raw_trav = has_raw_traversability ? raw_traversability[index] :
        std::numeric_limits<float>::quiet_NaN();
    }
    return true;
  }
};

struct OccupancyMapSnapshot
{
  double stamp_sec{0.0};
  std::string frame_id;
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.1};
  int width{0};
  int height{0};
  double yaw{0.0};
  std::vector<float> occupancy_cost;
  std::vector<uint8_t> valid_mask;
  std::vector<float> traversability;

  bool sample(double x, double y, float * cost, bool * unknown, float * traversability_out) const
  {
    const double dx = x - origin_x;
    const double dy = y - origin_y;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    const int col = static_cast<int>(std::floor(local_x / resolution));
    const int row = static_cast<int>(std::floor(local_y / resolution));
    if (row < 0 || row >= height || col < 0 || col >= width) {
      *cost = 0.0f;
      *unknown = true;
      *traversability_out = std::numeric_limits<float>::quiet_NaN();
      return false;
    }
    const size_t index = idx2(static_cast<size_t>(row), static_cast<size_t>(col), width);
    *cost = occupancy_cost[index];
    *unknown = valid_mask[index] == 0U;
    *traversability_out = traversability[index];
    return true;
  }
};

}  // namespace

class MPPINavigator : public rclcpp::Node
{
public:
  MPPINavigator()
  : Node("mppi_navigator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    readParameters();

    rng_.seed(static_cast<uint32_t>(seed_));
    normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
    control_sequence_.assign(static_cast<size_t>(time_steps_), Control{});
    footprint_offsets_ = buildFootprintOffsets();

    grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      grid_map_topic_, rclcpp::QoS(10),
      std::bind(&MPPINavigator::onGridMap, this, std::placeholders::_1));

    if (!traversability_map_topic_.empty()) {
      auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      traversability_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        traversability_map_topic_, latched_qos,
        std::bind(&MPPINavigator::onTraversabilityMap, this, std::placeholders::_1));
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&MPPINavigator::onOdom, this, std::placeholders::_1));
    world_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      world_pose_topic_, rclcpp::QoS(20),
      std::bind(&MPPINavigator::onWorldPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&MPPINavigator::onGoal, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      reference_path_topic_, rclcpp::QoS(10),
      std::bind(&MPPINavigator::onReferencePath, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, rclcpp::QoS(10));
    optimal_path_pub_ = create_publisher<nav_msgs::msg::Path>(optimal_path_topic_, rclcpp::QoS(10));

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    if (!reference_path_debug_topic_.empty()) {
      reference_path_debug_pub_ = create_publisher<nav_msgs::msg::Path>(
        reference_path_debug_topic_, latched_qos);
    }
    if (!terrain_cost_topic_.empty()) {
      terrain_cost_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        terrain_cost_topic_, latched_qos);
    }

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&MPPINavigator::onParameterChange, this, std::placeholders::_1));

    // Sim-time timer: use rclcpp::create_timer with the node's ROS clock so
    // that the control loop respects the ROS clock (use_sim_time), matching
    // the Python implementation and the cmd_vel_to_swerve bridge.
    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_seconds(1.0 / control_rate_),
      std::bind(&MPPINavigator::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "C++ MPPI navigator ready: goal_topic=%s odom_topic=%s grid_map_topic=%s "
      "traversability_map_topic=%s batch=%d horizon=%d dt=%.2f",
      goal_topic_.c_str(), odom_topic_.c_str(), grid_map_topic_.c_str(),
      traversability_map_topic_.empty() ? "<disabled>" : traversability_map_topic_.c_str(),
      batch_size_, time_steps_, model_dt_);
  }

  ~MPPINavigator() override
  {
    publishStop("MPPI shutdown.");
  }

  void stopNow(const std::string & reason)
  {
    publishStop(reason);
  }

private:
  void declareParameters()
  {
    if (!has_parameter("use_sim_time")) {
      declare_parameter("use_sim_time", false);
    }
    declare_parameter("grid_map_topic", "/elevation_mapping_node/elevation_map");
    declare_parameter("traversability_map_topic", "/traversability_map");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("world_pose_topic", "/pose");
    declare_parameter("goal_topic", "/goal_pose");
    declare_parameter("reference_path_topic", "/mppi/reference_path");
    declare_parameter("reference_path_debug_topic", "/mppi/reference_path_debug");
    declare_parameter("cmd_topic", "/cmd_vel");
    declare_parameter("optimal_path_topic", "/mppi/optimal_path");
    declare_parameter("terrain_cost_topic", "/mppi/terrain_cost_map");
    declare_parameter("control_rate", 20.0);
    declare_parameter("batch_size", 1000);
    declare_parameter("time_steps", 56);
    declare_parameter("model_dt", 0.05);
    declare_parameter("iteration_count", 1);
    declare_parameter("temperature", 0.50);
    declare_parameter("gamma", 0.015);
    declare_parameter("seed", 7);
    declare_parameter("command_sequence_offset", -1);
    declare_parameter("use_savgol_filter", true);
    declare_parameter("min_vx_velocity_threshold", 0.001);
    declare_parameter("min_vy_velocity_threshold", 0.001);
    declare_parameter("min_wz_velocity_threshold", 0.001);
    declare_parameter("vx_max", 1.5);
    declare_parameter("vx_min", -0.3);
    declare_parameter("vy_max", 0.6);
    declare_parameter("wz_max", 1.4);
    declare_parameter("ax_max", 3.0);
    declare_parameter("ay_max", 2.0);
    declare_parameter("awz_max", 3.0);
    declare_parameter("vx_std", 0.50);
    declare_parameter("vy_std", 0.35);
    declare_parameter("wz_std", 0.55);
    declare_parameter("goal_distance_weight", 10.0);
    declare_parameter("goal_progress_weight", 1.5);
    declare_parameter("goal_heading_weight", 1.5);
    declare_parameter("goal_heading_activation_distance", 0.25);
    declare_parameter("path_distance_weight", 1.5);
    declare_parameter("path_align_weight", 1.5);
    declare_parameter("path_follow_offset", 6);
    declare_parameter("path_follow_threshold_to_goal", 1.4);
    declare_parameter("control_effort_weight", 0.05);
    declare_parameter("control_smoothness_weight", 0.18);
    declare_parameter("twirling_weight", 0.005);
    declare_parameter("prefer_forward_weight", 0.25);
    declare_parameter("traversability_cost_weight", 4.0);
    declare_parameter("variance_cost_weight", 0.5);
    declare_parameter("variance_full_scale", 0.05);
    declare_parameter("slope_cost_weight", 8.0);
    declare_parameter("slope_start_deg", 20.0);
    declare_parameter("slope_max_deg", 45.0);
    declare_parameter("collision_cost", 5000.0);
    declare_parameter("fail_on_all_collision", false);
    declare_parameter("unknown_is_obstacle", false);
    declare_parameter("unknown_cost_weight", 0.35);
    declare_parameter("traversability_stop_threshold", 0.05);
    declare_parameter("footprint_radius", 0.38);
    declare_parameter("footprint_sample_count", 8);
    declare_parameter("goal_tolerance_xy", 0.35);
    declare_parameter("goal_tolerance_yaw", 0.35);
    declare_parameter("odom_timeout_sec", 0.5);
    declare_parameter("pose_timeout_sec", 0.5);
    declare_parameter("world_pose_hold_sec", 2.0);
    declare_parameter("map_timeout_sec", 1.0);
    declare_parameter("use_open_loop", true);
    declare_parameter("allow_goal_without_map", false);
    declare_parameter("publish_debug_path", true);
    declare_parameter("clear_reference_path_on_goal", false);
  }

  void readParameters()
  {
    grid_map_topic_ = get_parameter("grid_map_topic").as_string();
    traversability_map_topic_ = get_parameter("traversability_map_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    world_pose_topic_ = get_parameter("world_pose_topic").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    reference_path_topic_ = get_parameter("reference_path_topic").as_string();
    reference_path_debug_topic_ = get_parameter("reference_path_debug_topic").as_string();
    cmd_topic_ = get_parameter("cmd_topic").as_string();
    optimal_path_topic_ = get_parameter("optimal_path_topic").as_string();
    terrain_cost_topic_ = get_parameter("terrain_cost_topic").as_string();

    control_rate_ = std::max(1.0, get_parameter("control_rate").as_double());
    batch_size_ = std::max(32, static_cast<int>(get_parameter("batch_size").as_int()));
    time_steps_ = std::max(5, static_cast<int>(get_parameter("time_steps").as_int()));
    model_dt_ = std::max(0.02, get_parameter("model_dt").as_double());
    iteration_count_ = std::max(1, static_cast<int>(get_parameter("iteration_count").as_int()));
    temperature_ = std::max(1e-3, get_parameter("temperature").as_double());
    gamma_ = std::max(0.0, get_parameter("gamma").as_double());
    seed_ = static_cast<int>(get_parameter("seed").as_int());
    configured_command_sequence_offset_ =
      static_cast<int>(get_parameter("command_sequence_offset").as_int());
    use_savgol_filter_ = get_parameter("use_savgol_filter").as_bool();
    min_vx_velocity_threshold_ =
      std::max(0.0, get_parameter("min_vx_velocity_threshold").as_double());
    min_vy_velocity_threshold_ =
      std::max(0.0, get_parameter("min_vy_velocity_threshold").as_double());
    min_wz_velocity_threshold_ =
      std::max(0.0, get_parameter("min_wz_velocity_threshold").as_double());
    updateControlSequenceOffset();

    vx_max_ = get_parameter("vx_max").as_double();
    vx_min_ = get_parameter("vx_min").as_double();
    vy_max_ = std::abs(get_parameter("vy_max").as_double());
    wz_max_ = std::abs(get_parameter("wz_max").as_double());
    ax_max_ = std::abs(get_parameter("ax_max").as_double());
    ay_max_ = std::abs(get_parameter("ay_max").as_double());
    awz_max_ = std::abs(get_parameter("awz_max").as_double());
    vx_std_ = std::max(1e-3, get_parameter("vx_std").as_double());
    vy_std_ = std::max(1e-3, get_parameter("vy_std").as_double());
    wz_std_ = std::max(1e-3, get_parameter("wz_std").as_double());

    goal_distance_weight_ = std::max(0.0, get_parameter("goal_distance_weight").as_double());
    goal_progress_weight_ = std::max(0.0, get_parameter("goal_progress_weight").as_double());
    goal_heading_weight_ = std::max(0.0, get_parameter("goal_heading_weight").as_double());
    goal_heading_activation_distance_ =
      std::max(1e-3, get_parameter("goal_heading_activation_distance").as_double());
    path_distance_weight_ = std::max(0.0, get_parameter("path_distance_weight").as_double());
    path_align_weight_ = std::max(0.0, get_parameter("path_align_weight").as_double());
    path_follow_offset_ = std::max(0, static_cast<int>(get_parameter("path_follow_offset").as_int()));
    path_follow_threshold_to_goal_ =
      std::max(0.0, get_parameter("path_follow_threshold_to_goal").as_double());
    control_effort_weight_ = std::max(0.0, get_parameter("control_effort_weight").as_double());
    control_smoothness_weight_ =
      std::max(0.0, get_parameter("control_smoothness_weight").as_double());
    twirling_weight_ = std::max(0.0, get_parameter("twirling_weight").as_double());
    prefer_forward_weight_ = std::max(0.0, get_parameter("prefer_forward_weight").as_double());
    traversability_cost_weight_ =
      std::max(0.0, get_parameter("traversability_cost_weight").as_double());
    variance_cost_weight_ = std::max(0.0, get_parameter("variance_cost_weight").as_double());
    variance_full_scale_ = std::max(1e-3, get_parameter("variance_full_scale").as_double());
    slope_cost_weight_ = std::max(0.0, get_parameter("slope_cost_weight").as_double());
    slope_start_deg_ = get_parameter("slope_start_deg").as_double();
    slope_max_deg_ = get_parameter("slope_max_deg").as_double();
    collision_cost_ = std::max(1.0, get_parameter("collision_cost").as_double());
    fail_on_all_collision_ = get_parameter("fail_on_all_collision").as_bool();
    unknown_is_obstacle_ = get_parameter("unknown_is_obstacle").as_bool();
    unknown_cost_weight_ = std::max(0.0, get_parameter("unknown_cost_weight").as_double());
    traversability_stop_threshold_ =
      std::clamp(get_parameter("traversability_stop_threshold").as_double(), 0.0, 1.0);
    footprint_radius_ = std::max(0.0, get_parameter("footprint_radius").as_double());
    footprint_sample_count_ =
      std::max(0, static_cast<int>(get_parameter("footprint_sample_count").as_int()));
    goal_tolerance_xy_ = std::max(0.05, get_parameter("goal_tolerance_xy").as_double());
    goal_tolerance_yaw_ = std::max(0.05, get_parameter("goal_tolerance_yaw").as_double());
    odom_timeout_sec_ = std::max(0.0, get_parameter("odom_timeout_sec").as_double());
    pose_timeout_sec_ = std::max(0.0, get_parameter("pose_timeout_sec").as_double());
    world_pose_hold_sec_ = std::max(0.0, get_parameter("world_pose_hold_sec").as_double());
    map_timeout_sec_ = std::max(0.0, get_parameter("map_timeout_sec").as_double());
    use_open_loop_ = get_parameter("use_open_loop").as_bool();
    allow_goal_without_map_ = get_parameter("allow_goal_without_map").as_bool();
    publish_debug_path_ = get_parameter("publish_debug_path").as_bool();
    clear_reference_path_on_goal_ = get_parameter("clear_reference_path_on_goal").as_bool();
  }

  void updateControlSequenceOffset()
  {
    const double controller_period = 1.0 / std::max(control_rate_, 1e-3);
    constexpr double eps = 1e-6;
    shift_control_sequence_ = std::abs(controller_period - model_dt_) <= eps;

    if (controller_period > model_dt_ + eps) {
      RCLCPP_WARN(
        get_logger(),
        "MPPI controller period %.3fs is greater than model_dt %.3fs; "
        "Nav2 MPPI expects the controller period to be equal to or less than model_dt.",
        controller_period, model_dt_);
    } else if (shift_control_sequence_) {
      RCLCPP_INFO(
        get_logger(),
        "MPPI controller period equals model_dt; control sequence shifting is enabled.");
    }

    if (configured_command_sequence_offset_ >= 0) {
      command_sequence_offset_ = configured_command_sequence_offset_;
    } else {
      command_sequence_offset_ = shift_control_sequence_ ? 1 : 0;
    }
    command_sequence_offset_ = std::max(0, command_sequence_offset_);
  }

  std::vector<std::array<float, 2>> buildFootprintOffsets() const
  {
    std::vector<std::array<float, 2>> offsets;
    offsets.push_back({0.0f, 0.0f});
    if (footprint_radius_ <= 1e-6 || footprint_sample_count_ <= 0) {
      return offsets;
    }
    offsets.reserve(static_cast<size_t>(footprint_sample_count_) + 1U);
    for (int i = 0; i < footprint_sample_count_; ++i) {
      const double angle = 2.0 * kPi * static_cast<double>(i) /
        static_cast<double>(footprint_sample_count_);
      offsets.push_back(
        {static_cast<float>(footprint_radius_ * std::cos(angle)),
          static_cast<float>(footprint_radius_ * std::sin(angle))});
    }
    return offsets;
  }

  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  void onWorldPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_world_pose_ = PoseState{
      nowSec(),
      msg->pose.position.x,
      msg->pose.position.y,
      quaternion_to_yaw(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w),
      normalize_frame_id(msg->header.frame_id, "world")};
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = RobotState{
      nowSec(),
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      quaternion_to_yaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w),
      normalize_frame_id(msg->header.frame_id, "odom"),
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.angular.z};
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const std::string frame_id = normalize_frame_id(msg->header.frame_id, "world");
    if (frame_id != "world" && frame_id != "odom" && frame_id != "map") {
      RCLCPP_WARN(
        get_logger(), "Ignoring goal in unsupported frame '%s'. Expected world/odom/map.",
        frame_id.c_str());
      return;
    }
    goal_state_ = GoalState{
      msg->pose.position.x,
      msg->pose.position.y,
      quaternion_to_yaw(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w),
      frame_id};
    if (clear_reference_path_on_goal_) {
      reference_path_.clear();
      publishReferencePathDebug(nullptr);
    }
    goal_announced_ = false;
  }

  void onReferencePath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      reference_path_.clear();
      publishReferencePathDebug(nullptr);
      return;
    }
    const std::string frame_id = normalize_frame_id(msg->header.frame_id, "world");
    if (frame_id != "world" && frame_id != "odom" && frame_id != "map") {
      RCLCPP_WARN(
        get_logger(), "Ignoring reference path in unsupported frame '%s'. Expected world/odom/map.",
        frame_id.c_str());
      return;
    }
    reference_path_.clear();
    reference_path_.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      reference_path_.push_back(
        {static_cast<float>(pose.pose.position.x), static_cast<float>(pose.pose.position.y)});
    }
    reference_path_frame_id_ = frame_id;
    publishReferencePathDebug(msg.get());
  }

  void onGridMap(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    if (msg->layers.size() != msg->data.size()) {
      RCLCPP_WARN(get_logger(), "Received malformed GridMap message.");
      return;
    }
    const std::string frame_id = normalize_frame_id(msg->header.frame_id);
    if (frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Received GridMap with empty frame_id; ignoring terrain update.");
      return;
    }

    std::unordered_map<std::string, DecodedLayer> layers;
    int rows = 0;
    int cols = 0;
    try {
      for (size_t i = 0; i < msg->layers.size(); ++i) {
        auto layer = decode_multiarray_to_rows_cols(msg->layers[i], msg->data[i]);
        if (msg->outer_start_index != 0U || msg->inner_start_index != 0U) {
          layer.values = roll_layer(
            layer.values,
            layer.rows,
            layer.cols,
            static_cast<int>(msg->outer_start_index),
            static_cast<int>(msg->inner_start_index));
        }
        rows = layer.rows;
        cols = layer.cols;
        layers.emplace(msg->layers[i], std::move(layer));
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "%s", exc.what());
      return;
    }

    auto elevation_it = layers.find("elevation");
    if (elevation_it == layers.end()) {
      RCLCPP_WARN(get_logger(), "GridMap does not include an elevation layer.");
      return;
    }
    const auto & elevation = elevation_it->second.values;
    rows = elevation_it->second.rows;
    cols = elevation_it->second.cols;

    MapSnapshot snapshot;
    snapshot.stamp_sec = nowSec();
    snapshot.frame_id = frame_id;
    snapshot.center_x = msg->info.pose.position.x;
    snapshot.center_y = msg->info.pose.position.y;
    snapshot.yaw = quaternion_to_yaw(
      msg->info.pose.orientation.x,
      msg->info.pose.orientation.y,
      msg->info.pose.orientation.z,
      msg->info.pose.orientation.w);
    snapshot.length_x = msg->info.length_x;
    snapshot.length_y = msg->info.length_y;
    snapshot.resolution = msg->info.resolution;
    snapshot.rows = rows;
    snapshot.cols = cols;
    snapshot.valid_mask.assign(elevation.size(), 0U);
    for (size_t i = 0; i < elevation.size(); ++i) {
      snapshot.valid_mask[i] = std::isfinite(elevation[i]) ? 1U : 0U;
    }
    snapshot.slope_deg = computeSlopeDegrees(elevation, snapshot.valid_mask, rows, cols, snapshot.resolution);

    auto variance_it = layers.find("variance");
    if (variance_it != layers.end()) {
      snapshot.has_variance = true;
      snapshot.variance = std::move(variance_it->second.values);
    }

    auto trav_it = layers.find("traversability");
    if (trav_it != layers.end()) {
      snapshot.has_raw_traversability = true;
      snapshot.raw_traversability = std::move(trav_it->second.values);
    }

    rebuildGridMapCosts(snapshot);
    latest_grid_map_ = std::move(snapshot);
    publishDebugTerrainCostMap();
  }

  std::vector<float> computeSlopeDegrees(
    const std::vector<float> & elevation,
    const std::vector<uint8_t> & valid_mask,
    int rows,
    int cols,
    double resolution) const
  {
    std::vector<float> filled = elevation;
    double sum = 0.0;
    size_t count = 0U;
    for (size_t i = 0; i < elevation.size(); ++i) {
      if (valid_mask[i]) {
        sum += elevation[i];
        ++count;
      }
    }
    const float mean = count > 0U ? static_cast<float>(sum / static_cast<double>(count)) : 0.0f;
    for (size_t i = 0; i < filled.size(); ++i) {
      if (!valid_mask[i]) {
        filled[i] = mean;
      }
    }

    std::vector<uint8_t> known = valid_mask;
    for (int pass = 0; pass < 4; ++pass) {
      bool changed = false;
      auto next = filled;
      auto next_known = known;
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const size_t index = idx2(static_cast<size_t>(r), static_cast<size_t>(c), cols);
          if (known[index]) {
            continue;
          }
          double local_sum = 0.0;
          int local_count = 0;
          const int dr[4] = {-1, 1, 0, 0};
          const int dc[4] = {0, 0, -1, 1};
          for (int k = 0; k < 4; ++k) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
              continue;
            }
            const size_t nidx = idx2(static_cast<size_t>(nr), static_cast<size_t>(nc), cols);
            if (known[nidx]) {
              local_sum += filled[nidx];
              ++local_count;
            }
          }
          if (local_count > 0) {
            next[index] = static_cast<float>(local_sum / local_count);
            next_known[index] = 1U;
            changed = true;
          }
        }
      }
      filled.swap(next);
      known.swap(next_known);
      if (!changed) {
        break;
      }
    }

    std::vector<float> slope(elevation.size(), std::numeric_limits<float>::quiet_NaN());
    for (int r = 0; r < rows; ++r) {
      const int r0 = std::max(0, r - 1);
      const int r1 = std::min(rows - 1, r + 1);
      for (int c = 0; c < cols; ++c) {
        const size_t index = idx2(static_cast<size_t>(r), static_cast<size_t>(c), cols);
        if (!valid_mask[index]) {
          continue;
        }
        const int c0 = std::max(0, c - 1);
        const int c1 = std::min(cols - 1, c + 1);
        const double dy = (filled[idx2(static_cast<size_t>(r1), c, cols)] -
          filled[idx2(static_cast<size_t>(r0), c, cols)]) /
          (static_cast<double>(r1 - r0) * resolution);
        const double dx = (filled[idx2(static_cast<size_t>(r), c1, cols)] -
          filled[idx2(static_cast<size_t>(r), c0, cols)]) /
          (static_cast<double>(c1 - c0) * resolution);
        slope[index] = static_cast<float>(std::atan(std::hypot(dx, dy)) * 180.0 / kPi);
      }
    }
    return slope;
  }

  void rebuildGridMapCosts(MapSnapshot & snapshot)
  {
    snapshot.base_cost.assign(snapshot.slope_deg.size(), 0.0f);
    for (size_t i = 0; i < snapshot.slope_deg.size(); ++i) {
      if (!snapshot.valid_mask[i]) {
        continue;
      }
      const float slope_penalty = normalize_linear(
        snapshot.slope_deg[i],
        static_cast<float>(slope_start_deg_),
        static_cast<float>(slope_max_deg_));
      float cost = static_cast<float>(slope_cost_weight_) * slope_penalty;
      if (snapshot.has_variance && i < snapshot.variance.size()) {
        float variance_penalty = 1.0f;
        if (std::isfinite(snapshot.variance[i])) {
          variance_penalty = std::clamp(
            snapshot.variance[i] / static_cast<float>(variance_full_scale_),
            0.0f,
            1.0f);
        }
        cost += static_cast<float>(variance_cost_weight_) * variance_penalty;
      }
      snapshot.base_cost[i] = cost;
    }
  }

  void onTraversabilityMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int width = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);
    if (width <= 0 || height <= 0) {
      RCLCPP_WARN(get_logger(), "Received empty traversability OccupancyGrid.");
      return;
    }
    const std::string frame_id = normalize_frame_id(msg->header.frame_id);
    if (frame_id.empty()) {
      RCLCPP_WARN(
        get_logger(), "Received traversability OccupancyGrid with empty frame_id; ignoring update.");
      return;
    }
    const int expected = width * height;
    if (static_cast<int>(msg->data.size()) != expected || msg->info.resolution <= 0.0) {
      RCLCPP_WARN(get_logger(), "Received malformed traversability OccupancyGrid.");
      return;
    }

    OccupancyMapSnapshot snapshot;
    snapshot.stamp_sec = nowSec();
    snapshot.frame_id = frame_id;
    snapshot.origin_x = msg->info.origin.position.x;
    snapshot.origin_y = msg->info.origin.position.y;
    snapshot.resolution = msg->info.resolution;
    snapshot.width = width;
    snapshot.height = height;
    snapshot.yaw = quaternion_to_yaw(
      msg->info.origin.orientation.x,
      msg->info.origin.orientation.y,
      msg->info.origin.orientation.z,
      msg->info.origin.orientation.w);
    snapshot.occupancy_cost.assign(static_cast<size_t>(expected), 0.0f);
    snapshot.valid_mask.assign(static_cast<size_t>(expected), 0U);
    snapshot.traversability.assign(
      static_cast<size_t>(expected), std::numeric_limits<float>::quiet_NaN());
    for (int i = 0; i < expected; ++i) {
      const int value = static_cast<int>(msg->data[static_cast<size_t>(i)]);
      if (value < 0) {
        continue;
      }
      snapshot.valid_mask[static_cast<size_t>(i)] = 1U;
      const float cost = std::clamp(static_cast<float>(value) / 100.0f, 0.0f, 1.0f);
      snapshot.occupancy_cost[static_cast<size_t>(i)] = cost;
      snapshot.traversability[static_cast<size_t>(i)] = 1.0f - cost;
    }
    latest_traversability_map_ = std::move(snapshot);
    publishDebugTerrainCostMap();
  }

  void publishReferencePathDebug(const nav_msgs::msg::Path * path_msg)
  {
    if (!reference_path_debug_pub_) {
      return;
    }
    if (path_msg != nullptr) {
      reference_path_debug_pub_->publish(*path_msg);
      return;
    }
    nav_msgs::msg::Path clear_msg;
    clear_msg.header.frame_id = reference_path_frame_id_;
    clear_msg.header.stamp = now();
    reference_path_debug_pub_->publish(clear_msg);
  }

  std::optional<RobotState> resolveRobotState()
  {
    const double now = nowSec();
    const bool odom_fresh = latest_odom_.has_value() &&
      (odom_timeout_sec_ <= 0.0 || (now - latest_odom_->stamp_sec) <= odom_timeout_sec_);

    std::optional<PoseState> pose_state;
    if (latest_world_pose_.has_value()) {
      const double age = now - latest_world_pose_->stamp_sec;
      if (pose_timeout_sec_ <= 0.0 || age <= pose_timeout_sec_) {
        pose_state = latest_world_pose_;
        world_pose_hold_active_ = false;
      } else if (odom_fresh && (world_pose_hold_sec_ <= 0.0 || age <= world_pose_hold_sec_)) {
        pose_state = latest_world_pose_;
        if (!world_pose_hold_active_) {
          RCLCPP_WARN(get_logger(), "world_pose timed out; holding last world pose while using fresh odom twist.");
          world_pose_hold_active_ = true;
        }
      }
    }

    if (!pose_state.has_value() && odom_fresh) {
      world_pose_hold_active_ = false;
      pose_state = PoseState{
        latest_odom_->stamp_sec,
        latest_odom_->x,
        latest_odom_->y,
        latest_odom_->yaw,
        latest_odom_->frame_id};
    }

    if (!pose_state.has_value()) {
      world_pose_hold_active_ = false;
      return std::nullopt;
    }

    if (odom_fresh) {
      return RobotState{
        pose_state->stamp_sec,
        pose_state->x,
        pose_state->y,
        pose_state->yaw,
        pose_state->frame_id,
        latest_odom_->vx,
        latest_odom_->vy,
        latest_odom_->wz};
    }

    if (use_open_loop_) {
      return RobotState{
        pose_state->stamp_sec,
        pose_state->x,
        pose_state->y,
        pose_state->yaw,
        pose_state->frame_id,
        last_command_.vx,
        last_command_.vy,
        last_command_.wz};
    }

    return std::nullopt;
  }

  std::optional<PlanarTransform> lookupPlanarTransform(
    const std::string & target_frame,
    const std::string & source_frame)
  {
    const std::string target = normalize_frame_id(target_frame);
    const std::string source = normalize_frame_id(source_frame);
    if (target.empty() || source.empty()) {
      return std::nullopt;
    }
    if (target == source) {
      return PlanarTransform{};
    }
    const std::string key = target + "<-" + source;
    try {
      auto transform = tf_buffer_.lookupTransform(target, source, tf2::TimePointZero);
      tf_error_cache_.erase(key);
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      return PlanarTransform{
        t.x,
        t.y,
        quaternion_to_yaw(q.x, q.y, q.z, q.w)};
    } catch (const tf2::TransformException & exc) {
      const std::string message = exc.what();
      if (tf_error_cache_[key] != message) {
        RCLCPP_WARN(
          get_logger(), "Missing transform from '%s' to '%s': %s",
          source.c_str(), target.c_str(), message.c_str());
        tf_error_cache_[key] = message;
      }
      return std::nullopt;
    }
  }

  static std::array<double, 2> transformPoint(const PlanarTransform & tf, double x, double y)
  {
    const double c = std::cos(tf.yaw);
    const double s = std::sin(tf.yaw);
    return {tf.x + c * x - s * y, tf.y + s * x + c * y};
  }

  std::optional<GoalState> transformGoalToFrame(const GoalState & goal, const std::string & target_frame)
  {
    auto tf = lookupPlanarTransform(target_frame, goal.frame_id);
    if (!tf.has_value()) {
      return std::nullopt;
    }
    const auto point = transformPoint(*tf, goal.x, goal.y);
    return GoalState{
      point[0],
      point[1],
      wrap_angle(goal.yaw + tf->yaw),
      normalize_frame_id(target_frame)};
  }

  std::vector<std::array<float, 2>> transformPathToFrame(
    const std::vector<std::array<float, 2>> & path,
    const std::string & source_frame,
    const std::string & target_frame)
  {
    auto tf = lookupPlanarTransform(target_frame, source_frame);
    if (!tf.has_value()) {
      return {};
    }
    std::vector<std::array<float, 2>> out;
    out.reserve(path.size());
    for (const auto & point : path) {
      const auto transformed = transformPoint(*tf, point[0], point[1]);
      out.push_back({static_cast<float>(transformed[0]), static_cast<float>(transformed[1])});
    }
    return out;
  }

  bool isGridMapReady()
  {
    if (!latest_grid_map_) {
      return false;
    }
    return map_timeout_sec_ <= 0.0 || (nowSec() - latest_grid_map_->stamp_sec) <= map_timeout_sec_;
  }

  bool isTraversabilityMapReady()
  {
    if (traversability_map_topic_.empty() || !latest_traversability_map_) {
      return false;
    }
    return map_timeout_sec_ <= 0.0 ||
      (nowSec() - latest_traversability_map_->stamp_sec) <= map_timeout_sec_;
  }

  bool isMapReady()
  {
    const bool needs_grid = slope_cost_weight_ > 0.0 || variance_cost_weight_ > 0.0;
    const bool needs_trav = !traversability_map_topic_.empty();
    if (!needs_grid && !needs_trav) {
      return true;
    }
    if (needs_grid && !isGridMapReady()) {
      return false;
    }
    if (needs_trav && !isTraversabilityMapReady()) {
      return false;
    }
    return needs_grid || needs_trav;
  }

  bool canUseMapsInFrame(const std::string & target_frame)
  {
    if (isGridMapReady() && lookupPlanarTransform(latest_grid_map_->frame_id, target_frame) == std::nullopt) {
      return false;
    }
    if (isTraversabilityMapReady() &&
      lookupPlanarTransform(latest_traversability_map_->frame_id, target_frame) == std::nullopt)
    {
      return false;
    }
    return true;
  }

  void publishStop(const std::string & reason)
  {
    publishStopWithMode(reason, true);
  }

  void publishStopWithMode(const std::string & reason, bool hard)
  {
    if (last_stop_reason_ != reason) {
      RCLCPP_INFO(get_logger(), "%s", reason.c_str());
      last_stop_reason_ = reason;
    }
    Control stop{};
    if (!hard) {
      stop = computeSoftStopCommand();
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = stop.vx;
    cmd.linear.y = stop.vy;
    cmd.angular.z = stop.wz;
    cmd_pub_->publish(cmd);
    last_command_ = stop;

    if (hard || (std::abs(stop.vx) <= 1e-4 && std::abs(stop.vy) <= 1e-4 && std::abs(stop.wz) <= 1e-4)) {
      std::fill(control_sequence_.begin(), control_sequence_.end(), Control{});
      control_history_.fill(Control{});
    }
    if (publish_debug_path_) {
      optimal_path_pub_->publish(nav_msgs::msg::Path{});
    }
  }

  Control computeSoftStopCommand() const
  {
    const double stop_dt = std::max(1e-3, 1.0 / control_rate_);
    Control next = last_command_;
    next.vx = decayToZero(next.vx, static_cast<float>(ax_max_ * stop_dt));
    next.vy = decayToZero(next.vy, static_cast<float>(ay_max_ * stop_dt));
    next.wz = decayToZero(next.wz, static_cast<float>(awz_max_ * stop_dt));
    return next;
  }

  static float decayToZero(float value, float max_delta)
  {
    if (std::abs(value) <= 1e-4f) {
      return 0.0f;
    }
    const float mag = std::max(std::abs(value) - max_delta, 0.0f);
    return std::copysign(mag, value);
  }

  void onTimer()
  {
    auto state_opt = resolveRobotState();
    if (!state_opt.has_value()) {
      publishStopWithMode("MPPI waiting for fresh pose/odom.", false);
      return;
    }
    const RobotState state = *state_opt;

    if (!goal_state_.has_value()) {
      publishStopWithMode("MPPI idle: no active goal.", false);
      return;
    }
    if (!allow_goal_without_map_ && !isMapReady()) {
      publishStopWithMode("MPPI waiting for a fresh terrain map.", false);
      return;
    }
    if (!canUseMapsInFrame(state.frame_id)) {
      publishStopWithMode("MPPI waiting for terrain/map frame transforms.", false);
      return;
    }

    auto goal_in_state = transformGoalToFrame(*goal_state_, state.frame_id);
    if (!goal_in_state.has_value()) {
      publishStopWithMode("MPPI waiting for a goal frame transform.", false);
      return;
    }

    std::vector<std::array<float, 2>> reference_path_in_state;
    if (!reference_path_.empty()) {
      reference_path_in_state = transformPathToFrame(
        reference_path_, reference_path_frame_id_, state.frame_id);
    }

    const double goal_distance = std::hypot(goal_in_state->x - state.x, goal_in_state->y - state.y);
    const double goal_heading_error = std::abs(wrap_angle(goal_in_state->yaw - state.yaw));
    if (goal_distance <= goal_tolerance_xy_ && goal_heading_error <= goal_tolerance_yaw_) {
      goal_state_.reset();
      reference_path_.clear();
      publishReferencePathDebug(nullptr);
      publishStop("MPPI goal reached.");
      return;
    }

    if (!goal_announced_) {
      RCLCPP_INFO(
        get_logger(), "MPPI tracking goal at x=%.2f y=%.2f yaw=%.2f rad",
        goal_in_state->x, goal_in_state->y, goal_in_state->yaw);
      goal_announced_ = true;
    }

    auto result = optimizeControls(state, *goal_in_state, reference_path_in_state);
    if (result.failed) {
      publishStopWithMode("MPPI failed: all candidate trajectories are in collision or unknown space.", false);
      return;
    }

    const Control command = sanitizeCommand(selectCommandFromSequence(result.controls));
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.vx;
    twist.linear.y = command.vy;
    twist.angular.z = command.wz;
    cmd_pub_->publish(twist);
    last_command_ = command;
    last_stop_reason_.clear();
    updateControlHistory(command);
    shiftControlSequence();

    if (publish_debug_path_) {
      optimal_path_pub_->publish(buildPathMessage(result.best_trajectory, state.frame_id));
    }
  }

  Control selectCommandFromSequence(const std::vector<Control> & controls) const
  {
    if (controls.empty()) {
      return Control{};
    }
    const size_t offset = std::min(
      static_cast<size_t>(std::max(0, command_sequence_offset_)),
      controls.size() - 1U);
    return controls[offset];
  }

  Control sanitizeCommand(const Control & input) const
  {
    Control command = input;
    command.vx = std::abs(command.vx) < min_vx_velocity_threshold_ ? 0.0f : command.vx;
    command.vy = std::abs(command.vy) < min_vy_velocity_threshold_ ? 0.0f : command.vy;
    command.wz = std::abs(command.wz) < min_wz_velocity_threshold_ ? 0.0f : command.wz;
    command.vx = std::clamp(command.vx, static_cast<float>(vx_min_), static_cast<float>(vx_max_));
    command.vy = std::clamp(command.vy, static_cast<float>(-vy_max_), static_cast<float>(vy_max_));
    command.wz = std::clamp(command.wz, static_cast<float>(-wz_max_), static_cast<float>(wz_max_));
    return command;
  }

  void updateControlHistory(const Control & command)
  {
    for (size_t i = 0; i + 1U < control_history_.size(); ++i) {
      control_history_[i] = control_history_[i + 1U];
    }
    control_history_.back() = command;
  }

  void shiftControlSequence()
  {
    if (!shift_control_sequence_ || control_sequence_.size() <= 1U) {
      return;
    }
    std::rotate(control_sequence_.begin(), control_sequence_.begin() + 1, control_sequence_.end());
    control_sequence_.back() = control_sequence_[control_sequence_.size() - 2U];
  }

  struct OptimizeResult
  {
    std::vector<Control> controls;
    std::vector<Pose2> best_trajectory;
    bool failed{false};
  };

  OptimizeResult optimizeControls(
    const RobotState & state,
    const GoalState & goal,
    const std::vector<std::array<float, 2>> & reference_path)
  {
    std::vector<Control> nominal = control_sequence_;
    if (nominal.empty()) {
      nominal.assign(static_cast<size_t>(time_steps_), Control{});
    }
    applyControlConstraints(nominal, Control{static_cast<float>(state.vx), static_cast<float>(state.vy), static_cast<float>(state.wz)});

    std::vector<Pose2> best_traj;
    bool failed = false;

    for (int iteration = 0; iteration < iteration_count_; ++iteration) {
      // Noise decay: each subsequent iteration explores less, exploiting
      // the refined nominal from the previous iteration.  This lets
      // iteration_count>1 actually improve quality rather than just
      // re-sampling with the same variance.
      const double noise_decay = std::pow(0.5, static_cast<double>(iteration));
      const double vx_noise = vx_std_ * noise_decay;
      const double vy_noise = vy_std_ * noise_decay;
      const double wz_noise = wz_std_ * noise_decay;

      std::vector<Control> candidates(static_cast<size_t>(batch_size_ * time_steps_));
      for (int b = 0; b < batch_size_; ++b) {
        for (int t = 0; t < time_steps_; ++t) {
          const Control base = nominal[static_cast<size_t>(t)];
          candidates[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)] = Control{
            static_cast<float>(base.vx + vx_noise * normal_dist_(rng_)),
            static_cast<float>(base.vy + vy_noise * normal_dist_(rng_)),
            static_cast<float>(base.wz + wz_noise * normal_dist_(rng_))};
        }
      }
      applyControlConstraints(
        candidates,
        Control{static_cast<float>(state.vx), static_cast<float>(state.vy), static_cast<float>(state.wz)});

      auto trajectories = rollout(state, candidates);
      std::vector<float> costs;
      bool all_collide = false;
      evaluateCosts(state, goal, reference_path, trajectories, candidates, nominal, &costs, &all_collide);

      const auto min_it = std::min_element(costs.begin(), costs.end());
      const size_t best_index = static_cast<size_t>(std::distance(costs.begin(), min_it));
      best_traj.assign(
        trajectories.begin() + static_cast<std::ptrdiff_t>(best_index * time_steps_),
        trajectories.begin() + static_cast<std::ptrdiff_t>((best_index + 1U) * time_steps_));

      if (all_collide && fail_on_all_collision_) {
        failed = true;
        std::fill(nominal.begin(), nominal.end(), Control{});
        continue;
      }
      if (all_collide) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "All MPPI candidates touched collision cells; continuing with the lowest-cost trajectory.");
      }
      failed = false;

      const float min_cost = *min_it;
      std::vector<double> weights(costs.size(), 0.0);
      double weight_sum = 0.0;
      for (size_t i = 0; i < costs.size(); ++i) {
        weights[i] = std::exp(-(static_cast<double>(costs[i] - min_cost)) / temperature_);
        weight_sum += weights[i];
      }
      if (weight_sum <= 1e-12) {
        std::fill(weights.begin(), weights.end(), 1.0 / static_cast<double>(weights.size()));
      } else {
        for (auto & weight : weights) {
          weight /= weight_sum;
        }
      }

      std::fill(nominal.begin(), nominal.end(), Control{});
      for (int b = 0; b < batch_size_; ++b) {
        const double w = weights[static_cast<size_t>(b)];
        for (int t = 0; t < time_steps_; ++t) {
          const auto & c = candidates[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)];
          auto & n = nominal[static_cast<size_t>(t)];
          n.vx += static_cast<float>(w * c.vx);
          n.vy += static_cast<float>(w * c.vy);
          n.wz += static_cast<float>(w * c.wz);
        }
      }
      smoothControlSequence(nominal);
      applyControlConstraints(nominal, Control{static_cast<float>(state.vx), static_cast<float>(state.vy), static_cast<float>(state.wz)});
    }

    if (!failed && use_savgol_filter_) {
      applySavitzkyGolayFilter(nominal);
      applyControlConstraints(
        nominal,
        Control{static_cast<float>(state.vx), static_cast<float>(state.vy), static_cast<float>(state.wz)});
    }
    control_sequence_ = nominal;
    if (!failed) {
      best_traj = rollout(state, control_sequence_);
    } else if (best_traj.empty()) {
      best_traj = rollout(state, control_sequence_);
    }
    return OptimizeResult{control_sequence_, best_traj, failed};
  }

  void applyControlConstraints(std::vector<Control> & controls, const Control & current_control) const
  {
    if (controls.empty()) {
      return;
    }
    const bool batched = controls.size() == static_cast<size_t>(batch_size_ * time_steps_);
    const int rows = batched ? batch_size_ : 1;
    const int cols = batched ? time_steps_ : static_cast<int>(controls.size());
    for (int row = 0; row < rows; ++row) {
      Control prev = current_control;
      for (int step = 0; step < cols; ++step) {
        auto & c = controls[idx2(static_cast<size_t>(row), static_cast<size_t>(step), cols)];
        c.vx = std::clamp(c.vx, static_cast<float>(vx_min_), static_cast<float>(vx_max_));
        c.vy = std::clamp(c.vy, static_cast<float>(-vy_max_), static_cast<float>(vy_max_));
        c.wz = std::clamp(c.wz, static_cast<float>(-wz_max_), static_cast<float>(wz_max_));
        c.vx = prev.vx + std::clamp(c.vx - prev.vx, static_cast<float>(-ax_max_ * model_dt_), static_cast<float>(ax_max_ * model_dt_));
        c.vy = prev.vy + std::clamp(c.vy - prev.vy, static_cast<float>(-ay_max_ * model_dt_), static_cast<float>(ay_max_ * model_dt_));
        c.wz = prev.wz + std::clamp(c.wz - prev.wz, static_cast<float>(-awz_max_ * model_dt_), static_cast<float>(awz_max_ * model_dt_));
        c.vx = std::clamp(c.vx, static_cast<float>(vx_min_), static_cast<float>(vx_max_));
        c.vy = std::clamp(c.vy, static_cast<float>(-vy_max_), static_cast<float>(vy_max_));
        c.wz = std::clamp(c.wz, static_cast<float>(-wz_max_), static_cast<float>(wz_max_));
        prev = c;
      }
    }
  }

  void smoothControlSequence(std::vector<Control> & controls) const
  {
    if (controls.size() < 3U) {
      return;
    }
    const auto original = controls;
    for (size_t i = 1; i + 1U < controls.size(); ++i) {
      controls[i].vx = 0.25f * original[i - 1U].vx + 0.5f * original[i].vx + 0.25f * original[i + 1U].vx;
      controls[i].vy = 0.25f * original[i - 1U].vy + 0.5f * original[i].vy + 0.25f * original[i + 1U].vy;
      controls[i].wz = 0.25f * original[i - 1U].wz + 0.5f * original[i].wz + 0.25f * original[i + 1U].wz;
    }
  }

  void applySavitzkyGolayFilter(std::vector<Control> & controls) const
  {
    if (controls.size() < 9U) {
      return;
    }

    constexpr std::array<float, 9> coeffs{
      -21.0f / 231.0f,
      14.0f / 231.0f,
      39.0f / 231.0f,
      54.0f / 231.0f,
      59.0f / 231.0f,
      54.0f / 231.0f,
      39.0f / 231.0f,
      14.0f / 231.0f,
      -21.0f / 231.0f};
    constexpr size_t half_window = 4U;

    std::vector<Control> padded;
    padded.reserve(controls.size() + 2U * half_window);
    for (const auto & hist : control_history_) {
      padded.push_back(hist);
    }
    padded.insert(padded.end(), controls.begin(), controls.end());
    for (size_t i = 0; i < half_window; ++i) {
      padded.push_back(controls.back());
    }

    auto filtered = controls;
    for (size_t i = 0; i < controls.size(); ++i) {
      Control value{};
      for (size_t k = 0; k < coeffs.size(); ++k) {
        const auto & sample = padded[i + k];
        value.vx += coeffs[k] * sample.vx;
        value.vy += coeffs[k] * sample.vy;
        value.wz += coeffs[k] * sample.wz;
      }
      filtered[i] = value;
    }
    controls.swap(filtered);
  }

  std::vector<Pose2> rollout(const RobotState & state, const std::vector<Control> & controls) const
  {
    const bool batched = controls.size() == static_cast<size_t>(batch_size_ * time_steps_);
    const int rows = batched ? batch_size_ : 1;
    const int cols = batched ? time_steps_ : static_cast<int>(controls.size());
    std::vector<Pose2> trajectories(static_cast<size_t>(rows * cols));
    // Midpoint (RK2) integration — more accurate than Euler for the 56-step
    // horizon, especially when yaw changes significantly over the trajectory.
    const double half_dt = 0.5 * model_dt_;
    for (int row = 0; row < rows; ++row) {
      double x = state.x;
      double y = state.y;
      double yaw = state.yaw;
      for (int step = 0; step < cols; ++step) {
        const auto & c = controls[idx2(static_cast<size_t>(row), static_cast<size_t>(step), cols)];
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        // Half-step position update using current heading
        const double x_mid = x + (c.vx * cos_yaw - c.vy * sin_yaw) * half_dt;
        const double y_mid = y + (c.vx * sin_yaw + c.vy * cos_yaw) * half_dt;
        const double yaw_mid = wrap_angle(yaw + c.wz * half_dt);
        // Full-step using midpoint heading
        const double cos_mid = std::cos(yaw_mid);
        const double sin_mid = std::sin(yaw_mid);
        x = x + (c.vx * cos_mid - c.vy * sin_mid) * model_dt_;
        y = y + (c.vx * sin_mid + c.vy * cos_mid) * model_dt_;
        yaw = wrap_angle(yaw + c.wz * model_dt_);
        trajectories[idx2(static_cast<size_t>(row), static_cast<size_t>(step), cols)] =
          Pose2{static_cast<float>(x), static_cast<float>(y), static_cast<float>(yaw)};
      }
    }
    return trajectories;
  }

  void evaluateCosts(
    const RobotState & state,
    const GoalState & goal,
    const std::vector<std::array<float, 2>> & reference_path,
    const std::vector<Pose2> & trajectories,
    const std::vector<Control> & controls,
    const std::vector<Control> & nominal_controls,
    std::vector<float> * costs_out,
    bool * all_collide)
  {
    std::vector<float> costs(static_cast<size_t>(batch_size_), 0.0f);
    if (gamma_ > 0.0) {
      const double sx2 = vx_std_ * vx_std_;
      const double sy2 = vy_std_ * vy_std_;
      const double sw2 = wz_std_ * wz_std_;
      for (int b = 0; b < batch_size_; ++b) {
        double total = 0.0;
        for (int t = 0; t < time_steps_; ++t) {
          const auto & nominal = nominal_controls[static_cast<size_t>(t)];
          const auto & control = controls[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)];
          total += nominal.vx * (control.vx - nominal.vx) / sx2;
          total += nominal.vy * (control.vy - nominal.vy) / sy2;
          total += nominal.wz * (control.wz - nominal.wz) / sw2;
        }
        costs[static_cast<size_t>(b)] += static_cast<float>(gamma_ * total);
      }
    }

    for (int b = 0; b < batch_size_; ++b) {
      double control_energy = 0.0;
      double smoothness = 0.0;
      double twirling = 0.0;
      double reverse = 0.0;
      Control prev{static_cast<float>(state.vx), static_cast<float>(state.vy), static_cast<float>(state.wz)};
      for (int t = 0; t < time_steps_; ++t) {
        const auto & c = controls[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)];
        control_energy += c.vx * c.vx + c.vy * c.vy + c.wz * c.wz;
        smoothness += std::pow(c.vx - prev.vx, 2.0f) + std::pow(c.vy - prev.vy, 2.0f) + std::pow(c.wz - prev.wz, 2.0f);
        twirling += std::abs(c.wz);
        reverse += std::max(-c.vx, 0.0f);
        prev = c;
      }
      float cost = costs[static_cast<size_t>(b)];
      cost += static_cast<float>(control_effort_weight_ * model_dt_ * control_energy);
      cost += static_cast<float>(control_smoothness_weight_ * smoothness);
      cost += static_cast<float>(twirling_weight_ * model_dt_ * twirling);
      cost += static_cast<float>(prefer_forward_weight_ * model_dt_ * reverse);
      costs[static_cast<size_t>(b)] = cost;
    }

    std::vector<uint8_t> collision_mask(static_cast<size_t>(batch_size_), 0U);
    if (isGridMapReady() || isTraversabilityMapReady()) {
      auto terrain_cost = evaluateTerrainCosts(state.frame_id, trajectories, &collision_mask);
      for (int b = 0; b < batch_size_; ++b) {
        costs[static_cast<size_t>(b)] += terrain_cost[static_cast<size_t>(b)];
        if (collision_mask[static_cast<size_t>(b)]) {
          costs[static_cast<size_t>(b)] += static_cast<float>(collision_cost_);
        }
      }
    }
    *all_collide = std::all_of(collision_mask.begin(), collision_mask.end(), [](uint8_t v) {return v != 0U;});

    const double initial_goal_distance = std::hypot(goal.x - state.x, goal.y - state.y);
    for (int b = 0; b < batch_size_; ++b) {
      double final_goal_distance = 0.0;
      double total_progress = 0.0;
      double previous_distance = initial_goal_distance;
      for (int t = 0; t < time_steps_; ++t) {
        const auto & pose = trajectories[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)];
        const double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
        if (t == time_steps_ - 1) {
          final_goal_distance = distance;
        }
        total_progress += std::max(previous_distance - distance, 0.0);
        previous_distance = distance;
      }
      const auto & final_pose = trajectories[idx2(static_cast<size_t>(b), static_cast<size_t>(time_steps_ - 1), time_steps_)];
      const double heading_error = std::abs(wrap_angle(goal.yaw - final_pose.yaw));
      const double heading_gate = std::clamp(1.0 - final_goal_distance / goal_heading_activation_distance_, 0.0, 1.0);
      costs[static_cast<size_t>(b)] += static_cast<float>(
        goal_distance_weight_ * final_goal_distance -
        goal_progress_weight_ * total_progress +
        goal_heading_weight_ * heading_gate * heading_error);

      if (!reference_path.empty() && initial_goal_distance > path_follow_threshold_to_goal_) {
        const auto pruned = pruneReferencePath(state, reference_path);
        if (!pruned.empty()) {
          // Path follow: distance to a point ahead on the path
          const size_t target_index = std::min(
            static_cast<size_t>(path_follow_offset_), pruned.size() - 1U);
          const auto & target = pruned[target_index];
          const double path_distance = std::hypot(final_pose.x - target[0], final_pose.y - target[1]);
          costs[static_cast<size_t>(b)] += static_cast<float>(path_distance_weight_ * path_distance);

          // Path align: perpendicular distance to nearest path segment
          // (mirrors Nav2 PathAlignCritic — penalizes strafing away from the path)
          double min_cross_track = std::numeric_limits<double>::infinity();
          for (size_t i = 0; i + 1U < pruned.size(); ++i) {
            const double seg_dx = pruned[i + 1U][0] - pruned[i][0];
            const double seg_dy = pruned[i + 1U][1] - pruned[i][1];
            const double seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy;
            if (seg_len2 < 1e-9) {
              continue;
            }
            const double t = std::clamp(
              ((final_pose.x - pruned[i][0]) * seg_dx + (final_pose.y - pruned[i][1]) * seg_dy) / seg_len2,
              0.0, 1.0);
            const double proj_x = pruned[i][0] + t * seg_dx;
            const double proj_y = pruned[i][1] + t * seg_dy;
            const double cross_track = std::hypot(final_pose.x - proj_x, final_pose.y - proj_y);
            min_cross_track = std::min(min_cross_track, cross_track);
          }
          if (std::isfinite(min_cross_track)) {
            costs[static_cast<size_t>(b)] += static_cast<float>(path_align_weight_ * min_cross_track);
          }
        }
      }
    }
    *costs_out = std::move(costs);
  }

  std::vector<float> evaluateTerrainCosts(
    const std::string & state_frame_id,
    const std::vector<Pose2> & trajectories,
    std::vector<uint8_t> * collision_mask)
  {
    std::vector<float> terrain_cost(static_cast<size_t>(batch_size_), 0.0f);
    const MapSnapshot * grid = isGridMapReady() ? &(*latest_grid_map_) : nullptr;
    const OccupancyMapSnapshot * trav = isTraversabilityMapReady() ? &(*latest_traversability_map_) : nullptr;

    std::optional<PlanarTransform> grid_tf;
    std::optional<PlanarTransform> trav_tf;
    if (grid != nullptr) {
      grid_tf = lookupPlanarTransform(grid->frame_id, state_frame_id);
      if (!grid_tf.has_value()) {
        std::fill(collision_mask->begin(), collision_mask->end(), 1U);
        std::fill(terrain_cost.begin(), terrain_cost.end(), static_cast<float>(collision_cost_));
        return terrain_cost;
      }
    }
    if (trav != nullptr) {
      trav_tf = lookupPlanarTransform(trav->frame_id, state_frame_id);
      if (!trav_tf.has_value()) {
        std::fill(collision_mask->begin(), collision_mask->end(), 1U);
        std::fill(terrain_cost.begin(), terrain_cost.end(), static_cast<float>(collision_cost_));
        return terrain_cost;
      }
    }

    const size_t fp_count = footprint_offsets_.size();
    for (int b = 0; b < batch_size_; ++b) {
      double total_cost = 0.0;
      bool early_exit = false;
      for (int t = 0; t < time_steps_ && !early_exit; ++t) {
        const auto & pose = trajectories[idx2(static_cast<size_t>(b), static_cast<size_t>(t), time_steps_)];
        const double cy = std::cos(pose.yaw);
        const double sy = std::sin(pose.yaw);
        double step_cost_sum = 0.0;
        int unknown_count = 0;
        for (const auto & offset : footprint_offsets_) {
          const double sample_x = pose.x + cy * offset[0] - sy * offset[1];
          const double sample_y = pose.y + sy * offset[0] + cy * offset[1];
          bool combined_unknown = false;
          float sample_cost = 0.0f;
          bool sample_unknown = false;

          if (grid != nullptr && grid_tf.has_value()) {
            const auto p = transformPoint(*grid_tf, sample_x, sample_y);
            float raw_trav = std::numeric_limits<float>::quiet_NaN();
            grid->sample(p[0], p[1], &sample_cost, &sample_unknown, &raw_trav);
            step_cost_sum += sample_cost;
            combined_unknown = combined_unknown || sample_unknown;
            if (trav == nullptr && grid->has_raw_traversability) {
              float penalty = 1.0f;
              if (std::isfinite(raw_trav)) {
                penalty = std::clamp(1.0f - raw_trav, 0.0f, 1.0f);
                if (raw_trav <= traversability_stop_threshold_) {
                  (*collision_mask)[static_cast<size_t>(b)] = 1U;
                  early_exit = true;
                }
              }
              step_cost_sum += traversability_cost_weight_ * penalty;
            }
          }

          if (trav != nullptr && trav_tf.has_value()) {
            const auto p = transformPoint(*trav_tf, sample_x, sample_y);
            float trav_value = std::numeric_limits<float>::quiet_NaN();
            trav->sample(p[0], p[1], &sample_cost, &sample_unknown, &trav_value);
            step_cost_sum += traversability_cost_weight_ * sample_cost;
            combined_unknown = combined_unknown || sample_unknown;
            if (std::isfinite(trav_value) && trav_value <= traversability_stop_threshold_) {
              (*collision_mask)[static_cast<size_t>(b)] = 1U;
              early_exit = true;
            }
          }

          if (combined_unknown) {
            ++unknown_count;
          }
        }

        const double mean_step_cost = step_cost_sum / std::max<size_t>(fp_count, 1U);
        const double unknown_fraction = static_cast<double>(unknown_count) / std::max<size_t>(fp_count, 1U);
        total_cost += mean_step_cost + unknown_cost_weight_ * unknown_fraction;
        if (unknown_is_obstacle_ && unknown_count > 0) {
          (*collision_mask)[static_cast<size_t>(b)] = 1U;
          early_exit = true;
        }
      }
      // Penalize remaining steps for collided trajectories so they sort lower
      if (early_exit) {
        total_cost += static_cast<float>(collision_cost_);
      }
      terrain_cost[static_cast<size_t>(b)] = static_cast<float>(total_cost * model_dt_);
    }
    return terrain_cost;
  }

  std::vector<std::array<float, 2>> pruneReferencePath(
    const RobotState & state,
    const std::vector<std::array<float, 2>> & path) const
  {
    if (path.size() <= 1U) {
      return path;
    }
    size_t nearest = 0U;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.size(); ++i) {
      const double d = std::hypot(path[i][0] - state.x, path[i][1] - state.y);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    return std::vector<std::array<float, 2>>(path.begin() + static_cast<std::ptrdiff_t>(nearest), path.end());
  }

  nav_msgs::msg::Path buildPathMessage(const std::vector<Pose2> & trajectory, const std::string & frame_id)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = normalize_frame_id(frame_id, "world");
    path.header.stamp = now();
    path.poses.reserve(trajectory.size());
    for (const auto & pose : trajectory) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = path.header;
      pose_msg.pose.position.x = pose.x;
      pose_msg.pose.position.y = pose.y;
      pose_msg.pose.orientation.z = std::sin(0.5 * pose.yaw);
      pose_msg.pose.orientation.w = std::cos(0.5 * pose.yaw);
      path.poses.push_back(pose_msg);
    }
    return path;
  }

  void publishDebugTerrainCostMap()
  {
    if (!terrain_cost_pub_) {
      return;
    }
    auto msg = buildDebugTerrainCostMapMessage();
    if (msg.has_value()) {
      terrain_cost_pub_->publish(*msg);
    }
  }

  std::optional<nav_msgs::msg::OccupancyGrid> buildDebugTerrainCostMapMessage()
  {
    if (!isGridMapReady() && !isTraversabilityMapReady()) {
      return std::nullopt;
    }
    if (isTraversabilityMapReady()) {
      const auto & snapshot = *latest_traversability_map_;
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.stamp = now();
      msg.header.frame_id = snapshot.frame_id;
      msg.info.resolution = snapshot.resolution;
      msg.info.width = static_cast<uint32_t>(snapshot.width);
      msg.info.height = static_cast<uint32_t>(snapshot.height);
      msg.info.map_load_time = msg.header.stamp;
      msg.info.origin.position.x = snapshot.origin_x;
      msg.info.origin.position.y = snapshot.origin_y;
      msg.info.origin.orientation.z = std::sin(0.5 * snapshot.yaw);
      msg.info.origin.orientation.w = std::cos(0.5 * snapshot.yaw);

      const double max_cost = std::max(traversability_cost_weight_ + unknown_cost_weight_, 1e-6);
      msg.data.resize(snapshot.occupancy_cost.size(), -1);
      for (size_t i = 0; i < snapshot.occupancy_cost.size(); ++i) {
        if (!snapshot.valid_mask[i]) {
          msg.data[i] = -1;
          continue;
        }
        const double combined = traversability_cost_weight_ * snapshot.occupancy_cost[i];
        msg.data[i] = static_cast<int8_t>(std::lround(std::clamp((combined / max_cost) * 100.0, 0.0, 100.0)));
      }
      return msg;
    }
    return std::nullopt;
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    try {
      for (const auto & param : params) {
        const auto & name = param.get_name();
        if (name == "batch_size") {
          batch_size_ = std::max(32, static_cast<int>(param.as_int()));
        } else if (name == "time_steps") {
          time_steps_ = std::max(5, static_cast<int>(param.as_int()));
          control_sequence_.assign(static_cast<size_t>(time_steps_), Control{});
        } else if (name == "iteration_count") {
          iteration_count_ = std::max(1, static_cast<int>(param.as_int()));
        } else if (name == "temperature") {
          temperature_ = std::max(1e-3, param.as_double());
        } else if (name == "gamma") {
          gamma_ = std::max(0.0, param.as_double());
        } else if (name == "command_sequence_offset") {
          configured_command_sequence_offset_ = static_cast<int>(param.as_int());
          updateControlSequenceOffset();
        } else if (name == "use_savgol_filter") {
          use_savgol_filter_ = param.as_bool();
        } else if (name == "min_vx_velocity_threshold") {
          min_vx_velocity_threshold_ = std::max(0.0, param.as_double());
        } else if (name == "min_vy_velocity_threshold") {
          min_vy_velocity_threshold_ = std::max(0.0, param.as_double());
        } else if (name == "min_wz_velocity_threshold") {
          min_wz_velocity_threshold_ = std::max(0.0, param.as_double());
        } else if (name == "vx_std") {
          vx_std_ = std::max(1e-3, param.as_double());
        } else if (name == "vy_std") {
          vy_std_ = std::max(1e-3, param.as_double());
        } else if (name == "wz_std") {
          wz_std_ = std::max(1e-3, param.as_double());
        } else if (name == "goal_distance_weight") {
          goal_distance_weight_ = std::max(0.0, param.as_double());
        } else if (name == "goal_progress_weight") {
          goal_progress_weight_ = std::max(0.0, param.as_double());
        } else if (name == "goal_heading_weight") {
          goal_heading_weight_ = std::max(0.0, param.as_double());
        } else if (name == "goal_heading_activation_distance") {
          goal_heading_activation_distance_ = std::max(1e-3, param.as_double());
        } else if (name == "path_distance_weight") {
          path_distance_weight_ = std::max(0.0, param.as_double());
        } else if (name == "path_align_weight") {
          path_align_weight_ = std::max(0.0, param.as_double());
        } else if (name == "path_follow_offset") {
          path_follow_offset_ = std::max(0, static_cast<int>(param.as_int()));
        } else if (name == "path_follow_threshold_to_goal") {
          path_follow_threshold_to_goal_ = std::max(0.0, param.as_double());
        } else if (name == "control_effort_weight") {
          control_effort_weight_ = std::max(0.0, param.as_double());
        } else if (name == "control_smoothness_weight") {
          control_smoothness_weight_ = std::max(0.0, param.as_double());
        } else if (name == "twirling_weight") {
          twirling_weight_ = std::max(0.0, param.as_double());
        } else if (name == "prefer_forward_weight") {
          prefer_forward_weight_ = std::max(0.0, param.as_double());
        } else if (name == "traversability_cost_weight") {
          traversability_cost_weight_ = std::max(0.0, param.as_double());
        } else if (name == "variance_cost_weight") {
          variance_cost_weight_ = std::max(0.0, param.as_double());
        } else if (name == "variance_full_scale") {
          variance_full_scale_ = std::max(1e-3, param.as_double());
        } else if (name == "slope_cost_weight") {
          slope_cost_weight_ = std::max(0.0, param.as_double());
        } else if (name == "slope_start_deg") {
          slope_start_deg_ = param.as_double();
        } else if (name == "slope_max_deg") {
          slope_max_deg_ = param.as_double();
        } else if (name == "collision_cost") {
          collision_cost_ = std::max(1.0, param.as_double());
        } else if (name == "fail_on_all_collision") {
          fail_on_all_collision_ = param.as_bool();
        } else if (name == "unknown_is_obstacle") {
          unknown_is_obstacle_ = param.as_bool();
        } else if (name == "unknown_cost_weight") {
          unknown_cost_weight_ = std::max(0.0, param.as_double());
        } else if (name == "traversability_stop_threshold") {
          traversability_stop_threshold_ = std::clamp(param.as_double(), 0.0, 1.0);
        } else if (name == "footprint_radius") {
          footprint_radius_ = std::max(0.0, param.as_double());
          footprint_offsets_ = buildFootprintOffsets();
        } else if (name == "footprint_sample_count") {
          footprint_sample_count_ = std::max(0, static_cast<int>(param.as_int()));
          footprint_offsets_ = buildFootprintOffsets();
        } else if (name == "clear_reference_path_on_goal") {
          clear_reference_path_on_goal_ = param.as_bool();
        }
      }
      if (slope_max_deg_ <= slope_start_deg_) {
        throw std::runtime_error("slope_max_deg must be greater than slope_start_deg.");
      }
      if (latest_grid_map_) {
        rebuildGridMapCosts(*latest_grid_map_);
      }
      publishDebugTerrainCostMap();
    } catch (const std::exception & exc) {
      result.successful = false;
      result.reason = exc.what();
    }
    return result;
  }

  std::string grid_map_topic_;
  std::string traversability_map_topic_;
  std::string odom_topic_;
  std::string world_pose_topic_;
  std::string goal_topic_;
  std::string reference_path_topic_;
  std::string reference_path_debug_topic_;
  std::string cmd_topic_;
  std::string optimal_path_topic_;
  std::string terrain_cost_topic_;

  double control_rate_{20.0};
  int batch_size_{1000};
  int time_steps_{56};
  double model_dt_{0.05};
  int iteration_count_{1};
  double temperature_{0.5};
  double gamma_{0.015};
  int seed_{7};
  int configured_command_sequence_offset_{-1};
  int command_sequence_offset_{1};
  bool shift_control_sequence_{true};
  bool use_savgol_filter_{true};
  double min_vx_velocity_threshold_{0.001};
  double min_vy_velocity_threshold_{0.001};
  double min_wz_velocity_threshold_{0.001};
  double vx_max_{1.5};
  double vx_min_{-0.3};
  double vy_max_{0.6};
  double wz_max_{1.4};
  double ax_max_{3.0};
  double ay_max_{2.0};
  double awz_max_{3.0};
  double vx_std_{0.5};
  double vy_std_{0.35};
  double wz_std_{0.55};
  double goal_distance_weight_{10.0};
  double goal_progress_weight_{1.5};
  double goal_heading_weight_{1.5};
  double goal_heading_activation_distance_{0.25};
  double path_distance_weight_{1.5};
  double path_align_weight_{1.5};
  int path_follow_offset_{6};
  double path_follow_threshold_to_goal_{1.4};
  double control_effort_weight_{0.05};
  double control_smoothness_weight_{0.18};
  double twirling_weight_{0.005};
  double prefer_forward_weight_{0.25};
  double traversability_cost_weight_{4.0};
  double variance_cost_weight_{0.5};
  double variance_full_scale_{0.05};
  double slope_cost_weight_{8.0};
  double slope_start_deg_{20.0};
  double slope_max_deg_{45.0};
  double collision_cost_{5000.0};
  bool fail_on_all_collision_{false};
  bool unknown_is_obstacle_{false};
  double unknown_cost_weight_{0.35};
  double traversability_stop_threshold_{0.05};
  double footprint_radius_{0.38};
  int footprint_sample_count_{8};
  double goal_tolerance_xy_{0.35};
  double goal_tolerance_yaw_{0.35};
  double odom_timeout_sec_{0.5};
  double pose_timeout_sec_{0.5};
  double world_pose_hold_sec_{2.0};
  double map_timeout_sec_{1.0};
  bool use_open_loop_{true};
  bool allow_goal_without_map_{false};
  bool publish_debug_path_{true};
  bool clear_reference_path_on_goal_{false};

  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unordered_map<std::string, std::string> tf_error_cache_;

  std::optional<MapSnapshot> latest_grid_map_;
  std::optional<OccupancyMapSnapshot> latest_traversability_map_;
  std::optional<RobotState> latest_odom_;
  std::optional<PoseState> latest_world_pose_;
  std::optional<GoalState> goal_state_;
  std::vector<std::array<float, 2>> reference_path_;
  std::string reference_path_frame_id_{"world"};
  std::vector<Control> control_sequence_;
  std::array<Control, 4> control_history_{};
  Control last_command_;
  std::string last_stop_reason_;
  bool goal_announced_{false};
  bool world_pose_hold_active_{false};
  std::vector<std::array<float, 2>> footprint_offsets_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr world_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_cost_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPPINavigator>();
  rclcpp::spin(node);
  node->stopNow("MPPI stopped.");
  rclcpp::shutdown();
  return 0;
}
