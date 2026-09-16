#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>  // OpenCV for image processing
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher(const std::string& yaml_file)
        : Node("map_publisher")
    {
        // Set QoS Profile
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        publisher_       = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_profile);

        // Load map data from YAML file
        load_map(yaml_file);

        // Publish the map
        publish_map();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    std::vector<int8_t>                                        map_data_;
    float                                                      resolution_;
    std::vector<float>                                         origin_;
    float                                                      occupied_thresh_;
    float                                                      free_thresh_;

    void load_map(const std::string& yaml_file)
    {
        YAML::Node  config     = YAML::LoadFile(yaml_file);
        std::string image_path = ament_index_cpp::get_package_share_directory("navigation") + "/map/" + config["image"].as<std::string>();

        resolution_      = config["resolution"].as<float>();
        origin_          = { config["origin"][0].as<float>(), config["origin"][1].as<float>() };
        int negate       = config["negate"].as<int>();
        occupied_thresh_ = config["occupied_thresh"].as<float>();
        free_thresh_     = config["free_thresh"].as<float>();

        // Load and process the image
        cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load image: %s", image_path.c_str());
            return;
        }

        if (negate)
        {
            cv::bitwise_not(image, image);
        }

        cv::flip(image, image, 0);  // Flip to match ROS coordinate system
        map_data_.resize(image.total());
        for (int i = 0; i < image.rows; ++i)
        {
            for (int j = 0; j < image.cols; ++j)
            {
                uint8_t pixel = image.at<uint8_t>(i, j);
                if (pixel > (occupied_thresh_ * 255))
                {
                    map_data_[i * image.cols + j] = 100;  // Occupied
                }
                else if (pixel < (free_thresh_ * 255))
                {
                    map_data_[i * image.cols + j] = 0;  // Free
                }
                else
                {
                    map_data_[i * image.cols + j] = -1;  // Unknown
                }
            }
        }
    }

    void publish_map()
    {
        auto map_msg            = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp    = this->now();
        map_msg.header.frame_id = "map";

        map_msg.info.map_load_time        = this->now();
        map_msg.info.resolution           = resolution_;
        map_msg.info.width                = static_cast<uint32_t>(std::sqrt(map_data_.size()));
        map_msg.info.height               = static_cast<uint32_t>(map_data_.size() / map_msg.info.width);
        map_msg.info.origin.position.x    = origin_[0];
        map_msg.info.origin.position.y    = origin_[1];
        map_msg.info.origin.position.z    = 0.0;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data = map_data_;

        rclcpp::WallRate loop_rate(1);
        while (rclcpp::ok())
        {
            publisher_->publish(map_msg);
            RCLCPP_INFO(this->get_logger(), "Map published");
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string yaml_file = ament_index_cpp::get_package_share_directory("navigation") + "/map/xg_map.yaml";

    auto map_publisher = std::make_shared<MapPublisher>(yaml_file);
    rclcpp::spin(map_publisher);

    rclcpp::shutdown();
    return 0;
}
