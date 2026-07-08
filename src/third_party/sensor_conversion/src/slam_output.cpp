//
// Created by hjl on 2021/9/18.
// Modified by Qingchen Bi on 2022/11/05
//

#include "slam_simulation/slam_output.h"

#include <rmw/qos_profiles.h>

SlamOutput::SlamOutput(const rclcpp::Node::SharedPtr &node)
    : node_(node), t_b_w_(tf2::Transform::getIdentity()) {
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = node_->declare_parameter<std::string>("child_frame_id", "sensor");
    down_voxel_size_ = node_->declare_parameter<double>("down_voxel_size", 0.1);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    down_size_filter_.setLeafSize(down_voxel_size_, down_voxel_size_, down_voxel_size_);
    explored_area_dwz_filter_.setLeafSize(
        explored_area_voxel_size_, explored_area_voxel_size_, explored_area_voxel_size_);

    odom_pub_ = node_->create_publisher<Odometry>("odometry_init", rclcpp::QoS(1));
    reg_pub_ = node_->create_publisher<PointCloud2>("registered_scan", rclcpp::QoS(1));
    dwz_cloud_pub_ = node_->create_publisher<PointCloud2>("dwz_scan_cloud", rclcpp::QoS(1));

    local_cloud_sub_ = std::make_shared<message_filters::Subscriber<PointCloud2>>();
    local_cloud_sub_->subscribe(node_, "point_cloud", rmw_qos_profile_sensor_data);
    local_odom_sub_ = std::make_shared<message_filters::Subscriber<Odometry>>();
    local_odom_sub_->subscribe(node_, "odometry", rmw_qos_profile_sensor_data);

    sync_local_cloud_odom_ =
        std::make_shared<SynchronizerLocalCloudOdom>(SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_);
    sync_local_cloud_odom_->registerCallback(
        std::bind(&SlamOutput::pointCloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

    execution_timer_ = rclcpp::create_timer(
        node_,
        node_->get_clock(),
        rclcpp::Duration::from_seconds(0.2),
        std::bind(&SlamOutput::execute, this));
}

void SlamOutput::pointCloudOdomCallback(
    const PointCloud2::ConstSharedPtr &scan_in,
    const Odometry::ConstSharedPtr &input) {
    scan_in_ = scan_in;

    tf2::Quaternion quaternion(
        input->pose.pose.orientation.x,
        input->pose.pose.orientation.y,
        input->pose.pose.orientation.z,
        input->pose.pose.orientation.w);
    tf2::Vector3 vector3(
        input->pose.pose.position.x,
        input->pose.pose.position.y,
        input->pose.pose.position.z);
    tf2::Transform t_w_bi(quaternion, vector3);

    if (!is_get_first_) {
        t_b_w_ = t_w_bi.inverse();
        is_get_first_ = true;
    }

    const tf2::Transform t_b_bi = t_b_w_ * t_w_bi;
    st_b_bi_.header.stamp = scan_in->header.stamp;
    st_b_bi_.header.frame_id = frame_id_;
    st_b_bi_.child_frame_id = child_frame_id_;
    st_b_bi_.transform = tf2::toMsg(t_b_bi);

    broadcaster_->sendTransform(st_b_bi_);

    Odometry odom_msg;
    odom_msg.child_frame_id = child_frame_id_;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.header.stamp = scan_in->header.stamp;
    odom_msg.header.stamp = scan_in->header.stamp;
    odom_msg.pose.pose.orientation.x = t_b_bi.getRotation().getX();
    odom_msg.pose.pose.orientation.y = t_b_bi.getRotation().getY();
    odom_msg.pose.pose.orientation.z = t_b_bi.getRotation().getZ();
    odom_msg.pose.pose.orientation.w = t_b_bi.getRotation().getW();
    odom_msg.pose.pose.position.x = t_b_bi.getOrigin().getX();
    odom_msg.pose.pose.position.y = t_b_bi.getOrigin().getY();
    odom_msg.pose.pose.position.z = t_b_bi.getOrigin().getZ();
    odom_msg.twist = input->twist;

    odom_pub_->publish(odom_msg);
}

void SlamOutput::execute()
{
    if (!is_get_first_ || !scan_in_) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*scan_in_, *scan);

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_data = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<int> scan_index;
    pcl::removeNaNFromPointCloud(*scan, *scan_data, scan_index);

    down_size_filter_.setInputCloud(scan_data);
    pcl::PointCloud<pcl::PointXYZI> scan_dwz;
    down_size_filter_.filter(scan_dwz);

    tf2::Transform t_b_bi;
    tf2::fromMsg(st_b_bi_.transform, t_b_bi);

    Eigen::Matrix4f pose;
    pose << t_b_bi.getBasis()[0][0], t_b_bi.getBasis()[0][1], t_b_bi.getBasis()[0][2], t_b_bi.getOrigin()[0],
            t_b_bi.getBasis()[1][0], t_b_bi.getBasis()[1][1], t_b_bi.getBasis()[1][2], t_b_bi.getOrigin()[1],
            t_b_bi.getBasis()[2][0], t_b_bi.getBasis()[2][1], t_b_bi.getBasis()[2][2], t_b_bi.getOrigin()[2],
            0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto &point : scan_dwz.points) {
        pcl::PointXYZ reg_point;
        reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
        reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
        reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
        registered_scan->points.push_back(reg_point);
    }

    *explored_area_cloud_ += *registered_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_dwz(new pcl::PointCloud<pcl::PointXYZ>());
    explored_area_dwz_filter_.setInputCloud(explored_area_cloud_);
    explored_area_dwz_filter_.filter(*cloud_in_dwz);

    PointCloud2 scan_data_msg;
    pcl::toROSMsg(*cloud_in_dwz, scan_data_msg);
    scan_data_msg.header.stamp = scan_in_->header.stamp;
    scan_data_msg.header.frame_id = frame_id_;
    reg_pub_->publish(scan_data_msg);
}
