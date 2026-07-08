
#ifndef __UTILS_H_
#define __UTILS_H_

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include "Eigen/Eigen"
namespace utils_ns
{
struct ViewPoint
{
    geometry_msgs::msg::Point position;
    int indexX = 0;
    int indexY = 0;
    int fartherCentroidX = 0;
    int fartherCentroidY = 0;
    float score = 0.0;
    int disgridnum = 10000;
    int traver_degree = 100;
    double rv_path_cost = 0.0;
};

struct PointInt
{
  PointInt(int _x, int _y)
  {
    x = _x;
    y = _y;
  }

  int x;
  int y;
};

template <typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, const T default_val)
{
  return node->declare_parameter<T>(name, default_val);
}

struct Index2
{
  int x = 0;
  int y = 0;
};

double getAngleRobot(geometry_msgs::msg::Point& p, geometry_msgs::msg::Point& robot);
double getAngleVector(
    geometry_msgs::msg::Point& p1,
    geometry_msgs::msg::Point& p2,
    geometry_msgs::msg::Point& robot);
bool Bresenham(
    int x1, int y1, int x2, int y2,
    std::vector<Eigen::Vector2i> &visited_grid,
    nav_msgs::msg::OccupancyGrid& map);
}
#endif
