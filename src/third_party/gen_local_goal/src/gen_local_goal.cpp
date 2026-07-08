/**
 *  Created by Qingchen Bi on 2022/3/21
 */

#include "gen_local_goal.h"

GenLocalGoal::GenLocalGoal(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  tmp_path_length_ = 3;
  path_resolution_ = 0.3;
  Initialize();
}

void GenLocalGoal::Initialize()
{
  enable_blind_forward_fallback_ =
      node_->declare_parameter<bool>("enable_blind_forward_fallback", enable_blind_forward_fallback_);
  blind_forward_speed_ =
      node_->declare_parameter<double>("blind_forward_speed", blind_forward_speed_);
  enable_scan_recovery_ =
      node_->declare_parameter<bool>("enable_scan_recovery", enable_scan_recovery_);
  scan_recovery_yaw_rate_ =
      node_->declare_parameter<double>("scan_recovery_yaw_rate", scan_recovery_yaw_rate_);
  recovery_trigger_cycles_ =
      node_->declare_parameter<int>("recovery_trigger_cycles", recovery_trigger_cycles_);
  path_stale_timeout_sec_ =
      node_->declare_parameter<double>("path_stale_timeout_sec", path_stale_timeout_sec_);

  execution_timer_ = rclcpp::create_timer(
      node_,
      node_->get_clock(),
      rclcpp::Duration::from_seconds(0.1),
      std::bind(&GenLocalGoal::execute, this));
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(10), std::bind(&GenLocalGoal::MapCallBack, this, std::placeholders::_1));
  global_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(100), std::bind(&GenLocalGoal::PathCallBack, this, std::placeholders::_1));
  pubwaypoint_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("look_ahead_goal", rclcpp::QoS(1));
  pubSpeed_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(5));
  subSpeed_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel2", rclcpp::QoS(100), std::bind(&GenLocalGoal::velCallBack, this, std::placeholders::_1));
  last_waypoint_.point.x = -1;
  last_path_stamp_ = node_->get_clock()->now();
}

void GenLocalGoal::MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
}

void GenLocalGoal::PathCallBack(const nav_msgs::msg::Path::SharedPtr msg)
{
  if(msg->poses.size() > 2)
  {
    path_ = *msg;
    last_path_stamp_ = node_->get_clock()->now();
    has_path_ = true;
  }
  else
  {
    path_.poses.clear();
    tmp_path_.poses.clear();
    has_path_ = false;
    last_waypoint_.point.x = -1;
  }
}

GenLocalGoal::~GenLocalGoal()
{
}

void GenLocalGoal::GetGlobalPlan()
{
  if(map_.info.resolution != 0.0)
    path_resolution_ = map_.info.resolution;
  int local_path_point_num = (int)(tmp_path_length_ / path_resolution_);

  tmp_path_.poses.clear();
  tmp_path_.header.frame_id = "map";
  int ps = path_.poses.size();
  int head_num = std::min(local_path_point_num, ps);
  {
    if(head_num > 2)
    {
      for (int i = 0; i < head_num; i++)
      {
        double x = path_.poses[i].pose.position.x;
        double y = path_.poses[i].pose.position.y;
        geometry_msgs::msg::PoseStamped tmp_p;
        tmp_p.pose.position.x = x;
        tmp_p.pose.position.y = y;
        tmp_p.header.frame_id = "map";
        tmp_path_.poses.push_back(tmp_p);
      }       
    }
 
  }
}

void GenLocalGoal::velCallBack(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  cmd_vel_ = msg->twist;
}

void GenLocalGoal::PublishRecoveryCmd()
{
  geometry_msgs::msg::Twist cmd_vel;

  if (enable_blind_forward_fallback_) {
    cmd_vel.linear.x = blind_forward_speed_;
    cmd_vel.angular.z = 0.0;
  } else if (enable_scan_recovery_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = scan_recovery_yaw_rate_;
  }

  pubSpeed_->publish(cmd_vel);

  if (!recovery_active_) {
    if (enable_blind_forward_fallback_) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Recovery fallback active: publishing blind forward cmd_vel %.2f m/s",
          blind_forward_speed_);
    } else if (enable_scan_recovery_) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Recovery fallback active: no valid exploration path, scanning in place at %.2f rad/s",
          scan_recovery_yaw_rate_);
    } else {
      RCLCPP_WARN(
          node_->get_logger(),
          "Recovery fallback active: no valid exploration path, publishing stop cmd_vel");
    }
    recovery_active_ = true;
  }
}

void GenLocalGoal::execute()
{
  if (has_path_ &&
      (node_->get_clock()->now() - last_path_stamp_).seconds() > path_stale_timeout_sec_)
  {
    path_.poses.clear();
    tmp_path_.poses.clear();
    has_path_ = false;
    last_waypoint_.point.x = -1;
  }

  GetGlobalPlan();

  geometry_msgs::msg::PointStamped waypoint;

  if(!tmp_path_.poses.empty())
  {
    waypoint.point.x = tmp_path_.poses.back().pose.position.x;
    waypoint.point.y = tmp_path_.poses.back().pose.position.y;
    waypoint.header.frame_id = "map";
    pubwaypoint_->publish(waypoint);
    last_waypoint_ = waypoint;
    no_path_count_ = 0;

    if(fabs(cmd_vel_.linear.x) <= 1e-4 && fabs(cmd_vel_.angular.z) <= 1e-4)
    {
      has_path_no_vel_count_++;
    }
    else if(!pub_vel_ing_ && (fabs(cmd_vel_.linear.x) > 1e-4 || fabs(cmd_vel_.angular.z) > 1e-4))
    {
      has_path_no_vel_count_ = 0;
    }
    else if(pub_vel_ing_ && (fabs(cmd_vel_.linear.x) > 1e-4 || fabs(cmd_vel_.angular.z) > 1e-4))
    {
      pub_vel_count_++;
      if(pub_vel_count_ > 30)
      {
        pub_vel_ing_ = false;
        pub_vel_count_ = 0;
      }
    }
    if(has_path_no_vel_count_ > recovery_trigger_cycles_)
    {
      pub_vel_ing_ = true;
      PublishRecoveryCmd();
    }
    else if (recovery_active_)
    {
      recovery_active_ = false;
    }

  }
  else
  {
    no_path_count_++;
    if(no_path_count_ > recovery_trigger_cycles_)
    {
      PublishRecoveryCmd();
    }
    else if (recovery_active_)
    {
      recovery_active_ = false;
    }
  }
}
