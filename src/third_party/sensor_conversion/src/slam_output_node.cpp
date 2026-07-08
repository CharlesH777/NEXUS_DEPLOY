//
// Created by hjl on 2021/9/18.
//

#include "slam_simulation/slam_output.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("slam_sim_output");
    SlamOutput slam_output(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
