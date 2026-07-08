/**
 *  Created by Qingchen Bi on 2023/4/11
 */
#include <memory>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

struct mapUpdateData
{
    Eigen::Vector3d robot_coord;
    bool updated = false;
};

class ExplorationMapMerge
{
public:
    ExplorationMapMerge()
        : node_(std::make_shared<rclcpp::Node>("exploration_map_merge")),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_)
    {
        map_w_ = node_->declare_parameter<int>("map_w", 1800);
        map_h_ = node_->declare_parameter<int>("map_h", 1800);
        mapinitox_ = node_->declare_parameter<double>("mapinitox", -90.0);
        mapinitoy_ = node_->declare_parameter<double>("mapinitoy", -90.0);
        merge_size_ = node_->declare_parameter<double>("merge_size", 10.0);
        safe_obs_dis_ = node_->declare_parameter<double>("safe_obs_dis", 20.0);
        robot_clear_radius_ = node_->declare_parameter<double>("robot_clear_radius", 1.0);

        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/plane_OccMap", rclcpp::QoS(1000),
            std::bind(&ExplorationMapMerge::mapCallBack, this, std::placeholders::_1));
        global_map_pub_ =
            node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/globalMap", rclcpp::QoS(10));
        move_inited_pub_ =
            node_->create_publisher<std_msgs::msg::Int8>("/MoveInited", rclcpp::QoS(1));
        robot_position_pub_ =
            node_->create_publisher<geometry_msgs::msg::Point>("/RobotPosition", rclcpp::QoS(1));

        ego_position_last_.x = mapinitox_;
        ego_position_last_.y = mapinitoy_;
        is_move_inited_.data = 0;
    }

    void run()
    {
        // Note: rclcpp::Rate in Humble does not support clock argument.
        // For full sim-time compliance, consider migrating to a timer-based executor loop.
        rclcpp::Rate rate(2.0);
        while (rclcpp::ok()) {
            if (has_map_ && !init_succeeded_) {
                initGlobalMap();
            }

            updateRobotPosition();

            const bool robot_moved =
                std::abs(ego_position_last_.x - ego_position_.x) >= 0.1 ||
                std::abs(ego_position_last_.y - ego_position_.y) >= 0.1 ||
                std::abs(ego_position_last_.z - ego_position_.z) >= 0.1;

            if (init_succeeded_ && (map_dirty_ || robot_moved)) {
                mapMerge();
                map_dirty_ = false;
                is_move_inited_.data = 1;
                move_inited_pub_->publish(is_move_inited_);
                ego_position_last_ = ego_position_;
            }

            if (init_succeeded_) {
                clearRobotFootprint(global_map_data_);
                global_map_pub_->publish(global_map_data_);
            }
            robot_position_pub_->publish(ego_position_);

            rclcpp::spin_some(node_);
            rate.sleep();
        }
    }

private:
    void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
    {
        map_data_ = *msg;
        has_map_ = true;
        map_dirty_ = true;
    }

    void setBound(nav_msgs::msg::OccupancyGrid& global_map_data)
    {
        for (int i = 0; i < static_cast<int>(global_map_data.info.height); i++) {
            global_map_data.data[0 + i * global_map_data.info.width] = 100;
            global_map_data.data[global_map_data.info.width - 1 + i * global_map_data.info.width] = 100;
        }
        for (int j = 0; j < static_cast<int>(global_map_data.info.width); j++) {
            global_map_data.data[j + 0 * global_map_data.info.width] = 100;
            global_map_data.data[((global_map_data.info.width - 1) +
                                  (global_map_data.info.height - 1) * global_map_data.info.width) - j] = 100;
        }
    }

    void initGlobalMap()
    {
        global_map_data_.header.frame_id = map_data_.header.frame_id;
        global_map_data_.header.stamp = map_data_.header.stamp;
        global_map_data_.info.origin.position.x = mapinitox_;
        global_map_data_.info.origin.position.y = mapinitoy_;
        global_map_data_.info.origin.position.z = -0.5;

        global_map_data_.info.resolution = map_data_.info.resolution;
        global_map_data_.info.width = map_w_;
        global_map_data_.info.height = map_h_;
        global_map_data_.data.assign(
            global_map_data_.info.width * global_map_data_.info.height, -1);

        map_update_res_ = global_map_data_.info.resolution;
        map_update_width_ = static_cast<int>(global_map_data_.info.width);
        map_update_height_ = static_cast<int>(global_map_data_.info.height);
        map_update_.assign(
            map_update_width_,
            std::vector<mapUpdateData>(map_update_height_));

        setBound(global_map_data_);
        init_succeeded_ = true;
    }

    void mapMerge()
    {
        int rx = static_cast<int>(mapinitox_);
        int ry = static_cast<int>(mapinitoy_);
        if (ego_position_last_.x != mapinitox_) {
            rx = CONTXY2DISC(ego_position_last_.x - global_map_data_.info.origin.position.x, map_update_res_);
            ry = CONTXY2DISC(ego_position_last_.y - global_map_data_.info.origin.position.y, map_update_res_);
        }

        for (auto &column : map_update_) {
            for (auto &cell : column) {
                cell.updated = false;
            }
        }

        for (int i = 0; i < static_cast<int>(map_data_.info.width); i++) {
            for (int j = 0; j < static_cast<int>(map_data_.info.height); j++) {
                double px = DISCXY2CONT(i, map_data_.info.resolution) + map_data_.info.origin.position.x;
                double py = DISCXY2CONT(j, map_data_.info.resolution) + map_data_.info.origin.position.y;
                if (std::abs(px - ego_position_.x) > merge_size_ ||
                    std::abs(py - ego_position_.y) > merge_size_) {
                    continue;
                }

                int ix = CONTXY2DISC(px - global_map_data_.info.origin.position.x, global_map_data_.info.resolution);
                int iy = CONTXY2DISC(py - global_map_data_.info.origin.position.y, global_map_data_.info.resolution);

                int ux = CONTXY2DISC(px - global_map_data_.info.origin.position.x, map_update_res_);
                int uy = CONTXY2DISC(py - global_map_data_.info.origin.position.y, map_update_res_);

                if (ix < 0 || iy < 0 ||
                    ix >= static_cast<int>(global_map_data_.info.width) ||
                    iy >= static_cast<int>(global_map_data_.info.height) ||
                    ux < 0 || uy < 0 || ux >= map_update_width_ || uy >= map_update_height_) {
                    continue;
                }

                auto updateCell = [&](int update_x, int update_y) {
                    if (map_update_[update_x][update_y].updated) {
                        return;
                    }
                    if (global_map_data_.data[ix + iy * global_map_data_.info.width] == -1) {
                        global_map_data_.data[ix + iy * global_map_data_.info.width] =
                            map_data_.data[i + j * map_data_.info.width];
                    } else if (global_map_data_.data[ix + iy * global_map_data_.info.width] < 90 &&
                               global_map_data_.data[ix + iy * global_map_data_.info.width] >= 0) {
                        if (map_data_.data[i + j * map_data_.info.width] != -1) {
                            global_map_data_.data[ix + iy * global_map_data_.info.width] =
                                static_cast<int>(
                                    0.2 * static_cast<double>(global_map_data_.data[ix + iy * global_map_data_.info.width]) +
                                    0.8 * static_cast<double>(map_data_.data[i + j * map_data_.info.width]));
                        }
                    } else {
                        const double obs_alpha =
                            (std::abs(px - ego_position_.x) >= safe_obs_dis_ ||
                             std::abs(py - ego_position_.y) >= safe_obs_dis_) ? 0.2 : 0.96;
                        const double meas_alpha = 1.0 - obs_alpha;
                        if (map_data_.data[i + j * map_data_.info.width] < 119 &&
                            global_map_data_.data[ix + iy * global_map_data_.info.width] < 119) {
                            if (map_data_.data[i + j * map_data_.info.width] != -1) {
                                global_map_data_.data[ix + iy * global_map_data_.info.width] =
                                    static_cast<int>(
                                        obs_alpha * static_cast<double>(global_map_data_.data[ix + iy * global_map_data_.info.width]) +
                                        meas_alpha * static_cast<double>(map_data_.data[i + j * map_data_.info.width]));
                            }
                        } else {
                            global_map_data_.data[ix + iy * global_map_data_.info.width] = 100;
                        }
                    }
                    map_update_[update_x][update_y].updated = true;
                };

                if (rx == ux && ry == uy && ego_position_last_.x != mapinitox_) {
                    updateCell(rx, ry);
                } else {
                    updateCell(ux, uy);
                }
            }
        }
    }

    void updateRobotPosition()
    {
        // Guard against TF not yet available on startup
        if (!tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero)) {
            return;
        }
        try {
            const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            ego_position_.x = transform.transform.translation.x;
            ego_position_.y = transform.transform.translation.y;
            ego_position_.z = transform.transform.translation.z;
            has_robot_position_ = true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", ex.what());
        }
    }

    void clearRobotFootprint(nav_msgs::msg::OccupancyGrid& map)
    {
        if (!has_robot_position_ || robot_clear_radius_ <= 0.0) {
            return;
        }

        const int robot_ix =
            CONTXY2DISC(ego_position_.x - map.info.origin.position.x, map.info.resolution);
        const int robot_iy =
            CONTXY2DISC(ego_position_.y - map.info.origin.position.y, map.info.resolution);
        const int radius_cells = std::max(
            1, static_cast<int>(std::ceil(robot_clear_radius_ / map.info.resolution)));

        for (int ix = robot_ix - radius_cells; ix <= robot_ix + radius_cells; ix++) {
            for (int iy = robot_iy - radius_cells; iy <= robot_iy + radius_cells; iy++) {
                if (ix < 0 || iy < 0 ||
                    ix >= static_cast<int>(map.info.width) ||
                    iy >= static_cast<int>(map.info.height)) {
                    continue;
                }

                const double cell_x = DISCXY2CONT(ix, map.info.resolution) + map.info.origin.position.x;
                const double cell_y = DISCXY2CONT(iy, map.info.resolution) + map.info.origin.position.y;
                if (std::hypot(cell_x - ego_position_.x, cell_y - ego_position_.y) <= robot_clear_radius_) {
                    map.data[ix + iy * map.info.width] = 0;
                }
            }
        }
    }

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr move_inited_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_pub_;

    std::vector<std::vector<mapUpdateData>> map_update_;
    nav_msgs::msg::OccupancyGrid map_data_;
    nav_msgs::msg::OccupancyGrid global_map_data_;
    bool has_map_ = false;
    bool init_succeeded_ = false;
    bool map_dirty_ = false;
    bool has_robot_position_ = false;

    geometry_msgs::msg::Point ego_position_;
    geometry_msgs::msg::Point ego_position_last_;

    float map_update_res_{};
    int map_update_width_{};
    int map_update_height_{};

    int map_w_;
    int map_h_;
    double mapinitox_;
    double mapinitoy_;
    double merge_size_;
    double safe_obs_dis_;
    double robot_clear_radius_;

    std_msgs::msg::Int8 is_move_inited_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ExplorationMapMerge node;
    node.run();
    rclcpp::shutdown();
    return 0;
}
