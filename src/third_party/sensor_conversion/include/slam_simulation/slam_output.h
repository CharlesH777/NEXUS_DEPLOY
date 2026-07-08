//
// Created by hjl on 2021/9/18.
// Modified by Qingchen Bi on 2022/11/05
//
#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_H
#define TOPO_PLANNER_WS_SLAM_OUTPUT_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class SlamOutput {
public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Odometry = nav_msgs::msg::Odometry;
    using SyncPolicyLocalCloudOdom =
        message_filters::sync_policies::ApproximateTime<PointCloud2, Odometry>;
    using SynchronizerLocalCloudOdom =
        message_filters::Synchronizer<SyncPolicyLocalCloudOdom>;

    explicit SlamOutput(const rclcpp::Node::SharedPtr &node);

    void pointCloudOdomCallback(
        const PointCloud2::ConstSharedPtr &point_cloud,
        const Odometry::ConstSharedPtr &input);

private:
    void execute();

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr reg_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr dwz_cloud_pub_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    std::shared_ptr<message_filters::Subscriber<PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<Odometry>> local_odom_sub_;
    std::shared_ptr<SynchronizerLocalCloudOdom> sync_local_cloud_odom_;

    std::string frame_id_;
    std::string child_frame_id_;

    bool is_get_first_{false};
    tf2::Transform t_b_w_;
    geometry_msgs::msg::TransformStamped st_b_bi_;

    pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
    float down_voxel_size_{0.1F};

    pcl::PointCloud<pcl::PointXYZ>::Ptr explored_area_cloud_{
        pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    double explored_area_voxel_size_{0.1};
    pcl::VoxelGrid<pcl::PointXYZ> explored_area_dwz_filter_;

    PointCloud2::ConstSharedPtr scan_in_;
};

#endif // TOPO_PLANNER_WS_SLAM_OUTPUT_H
