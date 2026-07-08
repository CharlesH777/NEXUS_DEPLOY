/**
 *  Created by Qingchen Bi on 2022/3/21
 */

#ifndef GEN_LOCAL_GOAL_H
#define GEN_LOCAL_GOAL_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class GenLocalGoal
{  
public:
  explicit GenLocalGoal(const rclcpp::Node::SharedPtr& node);
  ~GenLocalGoal();
private:
  void execute();
  void Initialize();
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void PathCallBack(const nav_msgs::msg::Path::SharedPtr msg);
  void GetGlobalPlan();
  void velCallBack(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void PublishRecoveryCmd();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubwaypoint_;
  std::string map_topic_ = "/plane_OccMap";
  std::string path_topic_ = "/exporation_path";

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path tmp_path_;

  rclcpp::TimerBase::SharedPtr execution_timer_;

  double tmp_path_length_;
  double path_resolution_;
  geometry_msgs::msg::PointStamped last_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubSpeed_;
  int no_path_count_ = 0;
  
  int has_path_no_vel_count_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subSpeed_;
  geometry_msgs::msg::Twist cmd_vel_;
  bool pub_vel_ing_ = false;
  int pub_vel_count_ = 0;
  bool enable_blind_forward_fallback_ = false;
  double blind_forward_speed_ = 0.5;
  bool enable_scan_recovery_ = true;
  double scan_recovery_yaw_rate_ = 0.6;
  int recovery_trigger_cycles_ = 30;
  double path_stale_timeout_sec_ = 1.5;
  bool recovery_active_ = false;
  bool has_path_ = false;
  rclcpp::Time last_path_stamp_;

};
#endif 
