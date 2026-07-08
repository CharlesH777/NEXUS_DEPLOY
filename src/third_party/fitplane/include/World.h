/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  
 *  Modified by Qingchen Bi on 2022/11/05
 */

#ifndef WORLD_H
#define WORLD_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace FitPlane
{
    class PlaneMap;

struct FitPlaneArg
{
    double w_total_;
    double w_flatness_;
    double w_slope_;
    double w_sparsity_;
    double ratio_max_;
    double ratio_min_;
    double conv_thre_;
};
const float INF= std::numeric_limits<float>::max();
const float PI = 3.14151f;

class World;

/**
 * @brief Class for storing obstacles and world dimension.The information of obstacle is stored in a three-dimensional bool array.
 *        Before using the PF-RRT* algorithm,a suitable grid map must be built
 */
class World
{
public:
    //indicate whether the range of the grid map has been determined
    bool has_map_=false;

    explicit World(const rclcpp::Node::SharedPtr& node, const float &resolution);//, FitPlaneArg fitarg);
    ~World();

    /**
     * @brief Automatically determine the upperbound and lowerbound of the grid map according to the
     *        information of the input point cloud.
     * @param pcl::PointCloud<pcl::PointXYZ> point cloud input
     * @return void
     */
    void initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);

    /**
     * @brief Manually specify the upperbound and lowerbound.
     * @param Vector3d
     * @param Vector3d
     * @return void
     */
    void initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound);
    void setObs(const Eigen::Vector3d &point);

    /**
     * @brief Find the grid closet to the point and return the coordinate of its center
     * @param Vector3d
     * @return Vector3d
     */
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    bool isFree(const Eigen::Vector3d &point);
    bool isFree(const float &coord_x, const float &coord_y, const float &coord_z){return isFree(Eigen::Vector3d(coord_x,coord_y,coord_z));}

    /**
     * @brief Given a 2D coord,start from the lowerbound of the height of the grid map,search upward,
     *        and determine the boundary between the occupied area and the non occupied area as the 
     *        surface point.
     * @param float x(the first dimension)
     * @param float y(the second dimension)
     * @param Vector3d* p_surface(store the result of the projecting)
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool project2surface(const float &x,const float &y,Eigen::Vector3d* p_surface); 
    bool project2surface(const Eigen::Vector3d &p_original,Eigen::Vector3d* p_surface){return project2surface(p_original(0),p_original(1),p_surface);}

     /**
     * @brief Check if there is any obstacle between 2 nodes.
     * @param Node* node_start
     * @param Node* node_end
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    // bool collisionFree(const Node* node_start,const Node* node_end);
    
    /**
     * @brief Check whether the given point is within the range of the grid map
     * @param Eigen::Vector3i(the index value obtained after discretization of the given point)
     * @return bool true(within range),false（out of range)
     */
    bool isInsideBorder(const Eigen::Vector3i &index);
    bool isInsideBorder(const Eigen::Vector3d &point){return isInsideBorder(coord2index(point));}

    /**
     * @brief get the low bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getLowerBound(){return lowerbound_;}

    /**
     * @brief get the up bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getUpperBound(){return upperbound_;}

    /**
     * @brief get resolution of the world
     * @param void
     * @return float
     */
    float getResolution(){return resolution_;} 

    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + lowerbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }

    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-lowerbound_)/resolution_).cast<int>();            
        return index;
    }

    bool ***grid_map_=NULL;

    float resolution_;

    Eigen::Vector3i idx_count_;

    Eigen::Vector3d lowerbound_;
    Eigen::Vector3d upperbound_;

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_map_sub_;
    std::string PointCloud_topic = "/velodyne_points";
    std::string PointCloud_Map_topic = "/laser_cloud_map";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_pub_;
    std::string Grid_Map_topic = "/grid_map";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_map_pub_;
    std::string Plane_Map_topic = "/plane_map";

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::Point ego_position_;
    pcl::PointCloud<pcl::PointXYZ> cloud_near_;
    float minrange_ = 30.0;
    bool use_ex_range_ = false;
    double ex_robot_front_ = 90;
    double ex_robot_back_ = -10;
    double ex_robot_left_ = 90;
    double ex_robot_right_ = -10;
    void clearMap();

    void GetRobotPosition()
    {
        // Guard against TF not yet available on startup
        if (!tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero)) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            return;
        }
        try
        {
            const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            ego_position_.x = transform.transform.translation.x;
            ego_position_.y = transform.transform.translation.y;
            ego_position_.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

};

/**
 * @brief Given a 3D point,extract its x and y coordinates and return a 2D point
 * @param Vector3d 
 * @return Vector2d
 */
inline Eigen::Vector2d project2plane(const Eigen::Vector3d &p){return Eigen::Vector2d(p(0),p(1));}
inline Eigen::Vector2d project2plane(const float &x,const float &y){return Eigen::Vector2d(x,y);}

template <typename T>
void clean_vector(std::vector<T*> &vec)
{
    for(auto &element:vec)
    {
        delete element;
        element=NULL;
    }
    vec.clear();
}

}

#endif
