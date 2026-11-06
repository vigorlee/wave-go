#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

class CostmapListener : public rclcpp::Node
{
public:
    CostmapListener()
        : Node("costmap_listener")
    {
        global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10, std::bind(&CostmapListener::callbackGlobalCostmap, this, std::placeholders::_1));

        // local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     "/local_costmap/costmap", 10, std::bind(&CostmapListener::callbackLocalCostmap, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Costmap Listener Node started.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;

    void printCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int                        width  = msg->info.width;
        int                        height = msg->info.height;
        const std::vector<int8_t>& data   = msg->data;

        RCLCPP_INFO(this->get_logger(), "Map Info: Width=%d, Height=%d, Resolution=%.2f", width, height, msg->info.resolution);

        for (int y = height - 1; y >= 0; --y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                std::cout << std::setw(3) << static_cast<int>(data[index]) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;
    }

    void callbackGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /global_costmap/costmap");
        printCostmap(msg);
    }

    void callbackLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /local_costmap/costmap");
        printCostmap(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapListener>());
    rclcpp::shutdown();
    return 0;
}