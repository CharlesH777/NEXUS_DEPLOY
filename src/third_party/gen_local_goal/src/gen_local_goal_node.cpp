/**
 *  Created by Qingchen Bi on 2022/3/21
 */
#include <rclcpp/rclcpp.hpp>
#include "gen_local_goal.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gen_local_goal_node");
    GenLocalGoal gen_local_goal(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
