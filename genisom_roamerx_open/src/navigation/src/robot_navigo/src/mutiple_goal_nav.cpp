
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker.hpp"        // Marker消息类型
#include "visualization_msgs/msg/marker_array.hpp"  // MarkerArray消息类型

#include <memory>
#include <vector>

using NavigationAction = nav2_msgs::action::NavigateToPose;

class NavToPoseClient : public rclcpp::Node
{
public:
    using NavigationActionClient     = rclcpp_action::Client<NavigationAction>;
    using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

    NavToPoseClient()
        : Node("nav_to_pose_client")
    {
        action_client_ = rclcpp_action::create_client<NavigationAction>(this, "navigate_to_pose");

        // 初始化Marker发布器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("goal_marker", 10);

        // 添加多个导航目标点到目标队列
        addGoals();
    }

    void sendGoal()
    {
        if (goals_.empty())
        {
            RCLCPP_INFO(get_logger(), "所有目标点已完成导航。");
            return;
        }

        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }

        auto goal_msg = goals_.front();
        goals_.erase(goals_.begin());

        auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();

        send_goal_options.goal_response_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };

        send_goal_options.feedback_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const NavigationAction::Feedback> feedback)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
        };

        send_goal_options.result_callback = [this](const NavigationActionGoalHandle::WrappedResult& result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "目标点导航成功！");
                // 发送下一个目标点
                this->sendGoal();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "目标点导航失败。");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void addGoals()
    {
        // 创建并添加第一个目标点
        NavigationAction::Goal goal_1;
        goal_1.pose.header.frame_id    = "map";
        goal_1.pose.pose.position.x    = 4.0f;
        goal_1.pose.pose.position.y    = 0.0f;
        goal_1.pose.pose.orientation.w = 1.0;  // 设置方向
        goals_.push_back(goal_1);
        publishGoalMarker(goal_1, 0);  // 在RViz中显示目标点

        // 创建并添加第二个目标点
        NavigationAction::Goal goal_2;
        goal_2.pose.header.frame_id    = "map";
        goal_2.pose.pose.position.x    = 6.0f;
        goal_2.pose.pose.position.y    = 0.0f;
        goal_2.pose.pose.orientation.w = 1.0;  // 设置方向
        goals_.push_back(goal_2);
        publishGoalMarker(goal_2, 1);  // 在RViz中显示目标点

        // 继续添加更多目标点...
    }

    void publishGoalMarker(const NavigationAction::Goal& goal, int marker_id)
    {
        // 创建MarkerArray消息
        visualization_msgs::msg::MarkerArray marker_array;

        // 创建箭头Marker
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp    = this->now();
        arrow_marker.ns              = "goal_markers";
        arrow_marker.id              = marker_id;  // 使用唯一ID
        arrow_marker.type            = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action          = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose            = goal.pose.pose;
        arrow_marker.scale.x         = 0.5;  // 箭头长度
        arrow_marker.scale.y         = 0.1;  // 箭头宽度
        arrow_marker.scale.z         = 0.1;  // 箭头高度
        arrow_marker.color.a         = 1.0;  // 透明度
        arrow_marker.color.r         = 0.0;
        arrow_marker.color.g         = 1.0;  // 绿色
        arrow_marker.color.b         = 0.0;

        marker_array.markers.push_back(arrow_marker);  // 添加箭头Marker到MarkerArray

        // 创建文本Marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp    = this->now();
        text_marker.ns              = "goal_markers";
        text_marker.id              = marker_id + 1000;  // 给文本Marker一个不同的ID
        text_marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action          = visualization_msgs::msg::Marker::ADD;
        text_marker.pose            = goal.pose.pose;
        text_marker.pose.position.z += 0.5;  // 将文本位置稍微向上移动，以免与箭头重叠
        text_marker.scale.z = 0.3;           // 文本大小
        text_marker.color.a = 1.0;           // 透明度
        text_marker.color.r = 1.0;           // 红色
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0;
        text_marker.text    = "Goal" + std::to_string(marker_id + 1);  // 显示目标点编号

        marker_array.markers.push_back(text_marker);  // 添加文本Marker到MarkerArray

        // 发布MarkerArray
        marker_pub_->publish(marker_array);
    }

    std::vector<NavigationAction::Goal>                                goals_;  // 目标点队列
    NavigationActionClient::SharedPtr                                  action_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;  // MarkerArray发布器
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    node->sendGoal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}