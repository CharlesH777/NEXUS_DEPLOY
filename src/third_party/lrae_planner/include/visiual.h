#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
namespace visual
{
    class Marker
    {
    private:
        std::string pub_topic_;
        std::string frame_id_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Node::SharedPtr node_;

    public:
        // static int id_;
        visualization_msgs::msg::Marker marker_;

        explicit Marker(const rclcpp::Node::SharedPtr& node, std::string pub_topic, std::string frame_id)
            : pub_topic_(pub_topic), frame_id_(frame_id)
            , node_(node)
        {
            marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(pub_topic_, rclcpp::QoS(2));
            // id_++;
            marker_.id = 0;
            marker_.action = visualization_msgs::msg::Marker::ADD;
            marker_.pose.orientation.w = 1.0;
            marker_.header.frame_id = frame_id_;
        }

        ~Marker() = default;

        void SetColorRGBA(const std_msgs::msg::ColorRGBA& color)
        {
            marker_.color = color;
        }
        void SetColorRGBA(double r, double g, double b, double a)
        {
            marker_.color.r = r;
            marker_.color.g = g;
            marker_.color.b = b;
            marker_.color.a = a;
        }
        void SetScale(double x, double y, double z)
        {
            marker_.scale.x = x;
            marker_.scale.y = y;
            marker_.scale.z = z;
        }
        void SetType(int type)
        {
            marker_.type = type;
        }
        void SetAction(visualization_msgs::msg::Marker::_action_type action)
        {
            marker_.action = action;
        }
        void Publish()
        {
            marker_.header.stamp = node_->get_clock()->now();
            marker_pub_->publish(marker_);
        }
        void PushBackWinLine(
            geometry_msgs::msg::Point p1,
            geometry_msgs::msg::Point p2,
            geometry_msgs::msg::Point p3,
            geometry_msgs::msg::Point p4)
        {
            marker_.points.push_back(p1);
            marker_.points.push_back(p2);
            marker_.points.push_back(p2);
            marker_.points.push_back(p3);
            marker_.points.push_back(p3);
            marker_.points.push_back(p4);
            marker_.points.push_back(p4);
            marker_.points.push_back(p1);
        }
        void PushBackLine(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
        {
            marker_.points.push_back(p1);
            marker_.points.push_back(p2);
        }
        typedef std::shared_ptr<Marker> Ptr;
    };

}
