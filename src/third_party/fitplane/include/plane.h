/**
 *  Created by Qingchen Bi on 2022/11/05
 *  Several functions refer to PUTN
 */

#ifndef PLANE_H
#define PLANE_H

#include "World.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace FitPlane
{

struct Plane
{
public: 
    Eigen::Vector3d init_coord; // 2D 
    std::vector<Eigen::Vector3d> plane_pts;
    Eigen::Vector3d normal_vector;
    float traversability = 100;
    float plane_height;
    float plane_angle;
}; 

class PlaneMap
{
public:
    PlaneMap(World* world, const float resolution);
    ~PlaneMap();
    bool InitPlaneMap();
    void clearPlaneMap();
    void PointCloudMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_map);

    bool init();
    bool getPlaneMap();
    Plane FitPlane(Eigen::Vector3d& p_surface,World* world,const double &radius);

    void visSurf(const PlaneMap &planemap, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& surf_vis_pub);
    bool pubPlaneGridMap(const PlaneMap &planemap);

    double getAngle(Eigen::Vector3d &plane_vector);

    Plane** Plane_Map_=NULL; 
    // fitplane::PlaneMap PlaneGridMap_; 
    bool has_PlaneMap_=false;
    float resolution_;
    Eigen::Vector3i index_num_;
    Eigen::Vector3d leftdownbound_;
    Eigen::Vector3d rightupbound_;
    World* world_ = NULL;

    // For flat terrain
    // float max_angle_ = 60.0; 
    // float max_flatness_ = 0.7;
    // float w1_ = 0.9;
    
    // For uneven terrain
    float max_angle_ = 40.0;
    float max_flatness_ = 0.01;
    float w1_ = 0.8;

    int width_;
    int height_;
    nav_msgs::msg::OccupancyGrid plane_Occmap_;
    nav_msgs::msg::OccupancyGrid Original_plane_Occmap_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr plane_OccMap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trav_cloud_pub_;

    int occThre_ = 100;
    int flatThre_ = 99;
    int nobs_ = 120;
    
    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + leftdownbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }
    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-leftdownbound_)/resolution_).cast<int>();            
        return index;
    }
    bool isInBorder(const int& x, const int& y)
    {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExploredArea;
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
};

}

#endif
