#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTFBroadcaster : public rclcpp::Node
{
public:
    OdomToTFBroadcaster()
        : Node("odom_to_tf_broadcaster")
    {
        // Initialize the TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a subscription to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/current_pose", 10, std::bind(&OdomToTFBroadcaster::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Odom to TF Broadcaster started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped t;

        // Set the timestamp to the time of the received message
        t.header.stamp = msg->header.stamp;

        // Set the frame IDs
        t.header.frame_id = "odom";
        t.child_frame_id  = "base_link";

        // Set the translation
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        // Set the rotation
        t.transform.rotation = msg->pose.pose.orientation;

        // Broadcast the transforms
        // tf_broadcaster_->sendTransform(t);

        // Create another TransformStamped message
        geometry_msgs::msg::TransformStamped t_map;

        // Set the timestamp to the time of the received message
        t_map.header.stamp = msg->header.stamp;

        // Set the frame IDs
        t_map.header.frame_id = "map";
        t_map.child_frame_id  = "odom";

        // Set the translation
        t_map.transform.translation.x = 0.0;
        t_map.transform.translation.y = 0.0;
        t_map.transform.translation.z = 0.0;

        // Set the rotation
        t_map.transform.rotation.x = 0.0;
        t_map.transform.rotation.y = 0.0;
        t_map.transform.rotation.z = 0.0;
        t_map.transform.rotation.w = 1.0;

        // Broadcast the transforms
        tf_broadcaster_->sendTransform(t);
        tf_broadcaster_->sendTransform(t_map);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomToTFBroadcaster>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
