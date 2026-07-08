/**
 *  Created by Qingchen Bi on 2023/10/24
 */
#include <rclcpp/rclcpp.hpp>

#include "exploration_planning.h"
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lrae_planner_node");

  auto exploration_planner = std::make_shared<lrae_planner_ns::ExplorationPlanning>(node);
  (void)exploration_planner;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
