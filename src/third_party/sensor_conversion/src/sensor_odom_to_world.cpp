//
// Created by hjl on 2021/9/4.
//

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SensorOdomToWorld {
public:
    explicit SensorOdomToWorld(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_) {
        world_frame_ = node_->declare_parameter<std::string>("world_frame", "world");
        target_frame_ = node_->declare_parameter<std::string>("target_frame", "base_link");

        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "sensor/sensor_init/odometry", rclcpp::QoS(1),
            std::bind(&SensorOdomToWorld::odomCallback, this, std::placeholders::_1));
        sensor_odom_pub_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("sensor/world/odometry", rclcpp::QoS(1));
        base_odom_pub_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("base_link/world/odometry", rclcpp::QoS(1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &input) {
        sensor_frame_ = input->child_frame_id;
        sensor_init_frame_ = input->header.frame_id;

        if (!is_get_transform_) {
            bool got_world_to_init = false;
            try {
                const auto transform =
                    tf_buffer_.lookupTransform(world_frame_, sensor_init_frame_, tf2::TimePointZero);
                tf2::fromMsg(transform.transform, t_w_s0_);
                got_world_to_init = true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "lookup world->sensor_init failed: %s", ex.what());
            }

            bool got_base_to_sensor = false;
            try {
                const auto transform =
                    tf_buffer_.lookupTransform(target_frame_, sensor_frame_, tf2::TimePointZero);
                tf2::fromMsg(transform.transform, t_b_s_);
                got_base_to_sensor = true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "lookup base->sensor failed: %s", ex.what());
            }

            if (got_world_to_init && got_base_to_sensor) {
                is_get_transform_ = true;
                RCLCPP_INFO(node_->get_logger(), "TF setup complete, publishing odometry");
            }
            return;
        }

        tf2::Quaternion quaternion(
            input->pose.pose.orientation.x,
            input->pose.pose.orientation.y,
            input->pose.pose.orientation.z,
            input->pose.pose.orientation.w);
        tf2::Vector3 vector3(
            input->pose.pose.position.x,
            input->pose.pose.position.y,
            input->pose.pose.position.z);
        tf2::Transform t_s0_si(quaternion, vector3);

        const tf2::Transform t_w_si = t_w_s0_ * t_s0_si;
        const tf2::Transform t_w_bi = t_w_si * t_b_s_.inverse();

        nav_msgs::msg::Odometry sensor_odom_msg;
        sensor_odom_msg.child_frame_id = sensor_frame_;
        sensor_odom_msg.header.frame_id = world_frame_;
        sensor_odom_msg.header.stamp = input->header.stamp;
        sensor_odom_msg.pose.pose.orientation.x = t_w_si.getRotation().getX();
        sensor_odom_msg.pose.pose.orientation.y = t_w_si.getRotation().getY();
        sensor_odom_msg.pose.pose.orientation.z = t_w_si.getRotation().getZ();
        sensor_odom_msg.pose.pose.orientation.w = t_w_si.getRotation().getW();
        sensor_odom_msg.pose.pose.position.x = t_w_si.getOrigin().getX();
        sensor_odom_msg.pose.pose.position.y = t_w_si.getOrigin().getY();
        sensor_odom_msg.pose.pose.position.z = t_w_si.getOrigin().getZ();
        sensor_odom_msg.twist = input->twist;
        sensor_odom_pub_->publish(sensor_odom_msg);

        nav_msgs::msg::Odometry base_odom_msg;
        base_odom_msg.child_frame_id = target_frame_;
        base_odom_msg.header.frame_id = world_frame_;
        base_odom_msg.header.stamp = input->header.stamp;
        base_odom_msg.pose.pose.orientation.x = t_w_bi.getRotation().getX();
        base_odom_msg.pose.pose.orientation.y = t_w_bi.getRotation().getY();
        base_odom_msg.pose.pose.orientation.z = t_w_bi.getRotation().getZ();
        base_odom_msg.pose.pose.orientation.w = t_w_bi.getRotation().getW();
        base_odom_msg.pose.pose.position.x = t_w_bi.getOrigin().getX();
        base_odom_msg.pose.pose.position.y = t_w_bi.getOrigin().getY();
        base_odom_msg.pose.pose.position.z = t_w_bi.getOrigin().getZ();
        base_odom_msg.twist = input->twist;
        base_odom_pub_->publish(base_odom_msg);
    }

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::string sensor_init_frame_;
    std::string world_frame_;
    std::string target_frame_;
    std::string sensor_frame_;

    bool is_get_transform_{false};
    tf2::Transform t_b_s_;
    tf2::Transform t_w_s0_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensor_odom_to_world");
    auto sensor_odom_to_world = std::make_shared<SensorOdomToWorld>(node);
    (void)sensor_odom_to_world;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
