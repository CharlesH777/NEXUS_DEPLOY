/**
 *  Created by Qingchen Bi on 2022/11/05
 */
#include <rclcpp/rclcpp.hpp>

#include "vector"
#include <plane.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Traversibility_mapping");
    rclcpp::Rate rate(2.0);
    FitPlane::World world(node, 0.1);
    float plane_size = 0.3; // 0.5
    FitPlane::PlaneMap planemap(&world, plane_size);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();        
    }

    rclcpp::shutdown();
    return 0;
}
