// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navigo_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace navigo_waypoint_follower
{

    using rcl_interfaces::msg::ParameterType;
    using std::placeholders::_1;

    WaypointFollower::WaypointFollower(const rclcpp::NodeOptions& options)
        : navigo_util::LifecycleNode("waypoint_follower", "", options),
          waypoint_task_executor_loader_("navigo_waypoint_follower", "navigo_core::WaypointTaskExecutor")
    {
        RCLCPP_INFO(get_logger(), "Creating");

        declare_parameter("stop_on_failure", true);
        declare_parameter("loop_rate", 20);
        navigo_util::declare_parameter_if_not_declared(
            this, std::string("waypoint_task_executor_plugin"), rclcpp::ParameterValue(std::string("wait_at_waypoint")));
        navigo_util::declare_parameter_if_not_declared(
            this, std::string("wait_at_waypoint.plugin"), rclcpp::ParameterValue(std::string("navigo_waypoint_follower::WaitAtWaypoint")));
        declare_parameter_if_not_declared(this, std::string("start_navigation_topic"), rclcpp::ParameterValue("/start_navigation"));
    }

    WaypointFollower::~WaypointFollower() {}

    navigo_util::CallbackReturn WaypointFollower::on_configure(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        auto node = shared_from_this();

        const auto interface_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable();

        stop_on_failure_           = get_parameter("stop_on_failure").as_bool();
        loop_rate_                 = get_parameter("loop_rate").as_int();
        waypoint_task_executor_id_ = get_parameter("waypoint_task_executor_plugin").as_string();

        self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

        nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(get_node_base_interface(), get_node_graph_interface(),
            get_node_logging_interface(), get_node_waitables_interface(), "navigate_to_pose", callback_group_);

        action_server_ = std::make_unique<ActionServer>(get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
            get_node_waitables_interface(), "follow_waypoints", std::bind(&WaypointFollower::followWaypoints, this));

        start_navigation_sub_ =
            node->create_subscription<robots_dog_msgs::msg::StartNavigation>(node->get_parameter("start_navigation_topic").as_string(),
                interface_qos, std::bind(&WaypointFollower::onStartNavigationReceived, this, std::placeholders::_1));

        // Behavior Tree XML
        behavior_tree_dir_     = ament_index_cpp::get_package_share_directory("navigo_bt_navigator") + "/behavior_trees/";
        single_point_plan_xml_ = behavior_tree_dir_ + "navigate_through_poses_w_replanning_and_recovery.xml";

        // Load map client
        load_map_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
        load_map_client_         = node->create_client<LoadMapClientT>(
            "/map_server/load_map", rclcpp::ServicesQoS().get_rmw_qos_profile(), load_map_callback_group_);

        try
        {
            waypoint_task_executor_type_ = navigo_util::get_plugin_type_param(this, waypoint_task_executor_id_);
            waypoint_task_executor_      = waypoint_task_executor_loader_.createUniqueInstance(waypoint_task_executor_type_);
            RCLCPP_INFO(get_logger(), "Created waypoint_task_executor : %s of type %s", waypoint_task_executor_id_.c_str(),
                waypoint_task_executor_type_.c_str());
            waypoint_task_executor_->initialize(node, waypoint_task_executor_id_);
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            RCLCPP_FATAL(get_logger(), "Failed to create waypoint_task_executor. Exception: %s", ex.what());
        }

        return navigo_util::CallbackReturn::SUCCESS;
    }

    navigo_util::CallbackReturn WaypointFollower::on_activate(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        action_server_->activate();

        auto node = shared_from_this();
        // Add callback for dynamic parameters
        dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&WaypointFollower::dynamicParametersCallback, this, _1));

        // create bond connection
        createBond();

        return navigo_util::CallbackReturn::SUCCESS;
    }

    navigo_util::CallbackReturn WaypointFollower::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        action_server_->deactivate();
        dyn_params_handler_.reset();

        // destroy bond connection
        destroyBond();

        return navigo_util::CallbackReturn::SUCCESS;
    }

    navigo_util::CallbackReturn WaypointFollower::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        action_server_.reset();
        nav_to_pose_client_.reset();
        start_navigation_sub_.reset();
        self_client_.reset();

        return navigo_util::CallbackReturn::SUCCESS;
    }

    navigo_util::CallbackReturn WaypointFollower::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return navigo_util::CallbackReturn::SUCCESS;
    }

    void WaypointFollower::followWaypoints()
    {
        auto goal     = action_server_->get_current_goal();
        auto feedback = std::make_shared<ActionT::Feedback>();
        auto result   = std::make_shared<ActionT::Result>();

        // Check if request is valid
        if (!action_server_ || !action_server_->is_server_active())
        {
            RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Received follow waypoint request with %i waypoints.", static_cast<int>(goal->poses.size()));

        if (goal->poses.size() == 0)
        {
            action_server_->succeeded_current(result);
            return;
        }

        rclcpp::WallRate r(loop_rate_);
        uint32_t         goal_index = 0;
        bool             new_goal   = true;

        while (rclcpp::ok())
        {
            // Check if asked to stop processing action
            if (action_server_->is_cancel_requested())
            {
                auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
                callback_group_executor_.spin_until_future_complete(cancel_future);
                // for result callback processing
                callback_group_executor_.spin_some();
                action_server_->terminate_all();
                return;
            }

            // Check if asked to process another action
            if (action_server_->is_preempt_requested())
            {
                RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
                goal       = action_server_->accept_pending_goal();
                goal_index = 0;
                new_goal   = true;
            }

            // Check if we need to send a new goal
            if (new_goal)
            {
                new_goal = false;
                ClientT::Goal client_goal;
                client_goal.pose = goal->poses[goal_index];

                auto send_goal_options                   = rclcpp_action::Client<ClientT>::SendGoalOptions();
                send_goal_options.result_callback        = std::bind(&WaypointFollower::resultCallback, this, std::placeholders::_1);
                send_goal_options.goal_response_callback = std::bind(&WaypointFollower::goalResponseCallback, this, std::placeholders::_1);
                future_goal_handle_                      = nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
                current_goal_status_                     = ActionStatus::PROCESSING;
            }

            feedback->current_waypoint = goal_index;
            action_server_->publish_feedback(feedback);

            if (current_goal_status_ == ActionStatus::FAILED)
            {
                failed_ids_.push_back(goal_index);

                if (stop_on_failure_)
                {
                    RCLCPP_WARN(get_logger(),
                        "Failed to process waypoint %i in waypoint "
                        "list and stop on failure is enabled."
                        " Terminating action.",
                        goal_index);
                    result->missed_waypoints = failed_ids_;
                    action_server_->terminate_current(result);
                    failed_ids_.clear();
                    return;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                        "Failed to process waypoint %i,"
                        " moving to next.",
                        goal_index);
                }
            }
            else if (current_goal_status_ == ActionStatus::SUCCEEDED)
            {
                RCLCPP_INFO(get_logger(), "Succeeded processing waypoint %i, processing waypoint task execution", goal_index);
                bool is_task_executed = waypoint_task_executor_->processAtWaypoint(goal->poses[goal_index], goal_index);
                RCLCPP_INFO(get_logger(), "Task execution at waypoint %i %s", goal_index, is_task_executed ? "succeeded" : "failed!");
                // if task execution was failed and stop_on_failure_ is on , terminate action
                if (!is_task_executed && stop_on_failure_)
                {
                    failed_ids_.push_back(goal_index);
                    RCLCPP_WARN(get_logger(),
                        "Failed to execute task at waypoint %i "
                        " stop on failure is enabled."
                        " Terminating action.",
                        goal_index);
                    result->missed_waypoints = failed_ids_;
                    action_server_->terminate_current(result);
                    failed_ids_.clear();
                    return;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                        "Handled task execution on waypoint %i,"
                        " moving to next.",
                        goal_index);
                }
            }

            if (current_goal_status_ != ActionStatus::PROCESSING && current_goal_status_ != ActionStatus::UNKNOWN)
            {
                // Update server state
                goal_index++;
                new_goal = true;
                if (goal_index >= goal->poses.size())
                {
                    RCLCPP_INFO(get_logger(), "Completed all %zu waypoints requested.", goal->poses.size());
                    result->missed_waypoints = failed_ids_;
                    action_server_->succeeded_current(result);
                    failed_ids_.clear();
                    return;
                }
            }
            else
            {
                RCLCPP_INFO_EXPRESSION(
                    get_logger(), (static_cast<int>(now().seconds()) % 30 == 0), "Processing waypoint %i...", goal_index);
            }

            callback_group_executor_.spin_some();
            r.sleep();
        }
    }

    void WaypointFollower::resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result)
    {
        if (result.goal_id != future_goal_handle_.get()->get_goal_id())
        {
            RCLCPP_DEBUG(get_logger(), "Goal IDs do not match for the current goal handle and received result."
                                       "Ignoring likely due to receiving result for an old goal.");
            return;
        }

        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                current_goal_status_ = ActionStatus::SUCCEEDED;
                return;
            case rclcpp_action::ResultCode::ABORTED:
                current_goal_status_ = ActionStatus::FAILED;
                return;
            case rclcpp_action::ResultCode::CANCELED:
                current_goal_status_ = ActionStatus::FAILED;
                return;
            default:
                current_goal_status_ = ActionStatus::UNKNOWN;
                return;
        }
    }

    void WaypointFollower::goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr& goal)
    {
        if (!goal)
        {
            RCLCPP_ERROR(get_logger(), "navigate_to_pose action client failed to send goal to server.");
            current_goal_status_ = ActionStatus::FAILED;
        }
    }

    void WaypointFollower::onStartNavigationReceived(const StartNavigation::SharedPtr msg)
    {
        start_navigation_ = *msg;

        if (StartNavigation::CMD_STANDBY == start_navigation_.cmd)
        {
            onCmdStandByReceived();
        }

        if (StartNavigation::CMD_START == start_navigation_.cmd)
        {
            onCmdStartReceived();
        }

        if (StartNavigation::CMD_STOP == start_navigation_.cmd)
        {
            onCmdStopReceived();
        }
    }

    void WaypointFollower::onCmdStandByReceived()
    {
        if (!start_navigation_.map_path.empty())
        {
            loadMapFromYaml(start_navigation_.map_path);
        }
    }

    void WaypointFollower::onCmdStartReceived()
    {
        if (StartNavigation::FUNCTION_SINGLE_POINT_PLAN == start_navigation_.function_id)
        {
            onSinglePointPlanGoalReceived();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Unknown Function ID.");
        }
    }

    void WaypointFollower::onCmdStopReceived()
    {
        rclcpp::Time goal_expiry_time = now() - std::chrono::milliseconds(10);
        self_client_->async_cancel_goals_before(goal_expiry_time);
        RCLCPP_INFO(get_logger(), "NAVIGATION STATE: CANCELLING.");
    }

    void WaypointFollower::onSinglePointPlanGoalReceived()
    {
        if (StartNavigation::FUNCTION_SINGLE_POINT_PLAN != start_navigation_.function_id)
        {
            return;
        }

        /*--NAVIGATION STATE */
        RCLCPP_INFO(get_logger(), "NAVIGATION STATE: INITIALIZING.");
        /* NAVIGATION STATE--*/

        RCLCPP_INFO(get_logger(), "START PATROL: Initializing");

        if (!start_navigation_.map_path.empty())
        {
            loadMapFromYaml(start_navigation_.map_path);
        }
        else
        {
            if (start_navigation_.goals.empty())
            {
                RCLCPP_INFO(get_logger(), "START SinglePointPlan: Waiting for target goal");
                return;
            }
            else
            {
                ActionT::Goal goal;
                if (!constructFollowWaypointGoal(start_navigation_.function_id, start_navigation_.goals, {}, start_navigation_.speed, goal))
                {
                    RCLCPP_ERROR(get_logger(), "START TRACKING FAILED: Failed to construct follow waypoint goal.");
                    return;
                }
                self_client_->async_send_goal(goal);
            }
        }
    }

    void WaypointFollower::loadMapFromYaml(const std::string& yaml_file)
    {
        auto request     = std::make_shared<LoadMapClientT::Request>();
        request->map_url = yaml_file;
        load_map_client_->async_send_request(request, std::bind(&WaypointFollower::onLoadMapResponseReceived, this, std::placeholders::_1));

        load_map_timeout_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(10.0), std::bind(&WaypointFollower::onLoadMapTimeout, this), load_map_callback_group_);
    }

    void WaypointFollower::onLoadMapResponseReceived(rclcpp::Client<LoadMapClientT>::SharedFuture future)
    {
        load_map_timeout_timer_->cancel();

        auto resp = future.get();
        if (resp->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Load Map succeeded");

            if (StartNavigation::CMD_START == start_navigation_.cmd
                && StartNavigation::FUNCTION_SINGLE_POINT_PLAN == start_navigation_.function_id && !start_navigation_.goals.empty())
            {
                ActionT::Goal goal;
                if (!constructFollowWaypointGoal(start_navigation_.function_id, start_navigation_.goals, {}, start_navigation_.speed, goal))
                {
                    RCLCPP_ERROR(get_logger(), "START TRACKING FAILED: Failed to construct follow waypoint goal.");
                    return;
                }
                self_client_->async_send_goal(goal);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Load Map failed, code=%d", resp->result);
        }
    }

    bool WaypointFollower::constructFollowWaypointGoal(const uint8_t function_id, const std::vector<geometry_msgs::msg::Pose>& poses,
        const std::vector<robots_dog_msgs::msg::Trajectory>& /*reference_paths*/, const geometry_msgs::msg::Vector3& /*desired_velocity*/,
        ActionT::Goal& goal) const
    {
        // common header
        std_msgs::msg::Header header;
        header.stamp    = now();
        header.frame_id = "map";

        // poses
        std::vector<geometry_msgs::msg::PoseStamped> follow_waypoint_poses;
        for (const auto& p : poses)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.pose   = p;
            follow_waypoint_poses.push_back(pose_stamped);
        }
        goal.poses = follow_waypoint_poses;

        // behavior tree
        std::string behavior_tree;
        if (StartNavigation::FUNCTION_SINGLE_POINT_PLAN == function_id)
        {
            behavior_tree = single_point_plan_xml_;
        }
        if (behavior_tree.empty())
        {
            RCLCPP_ERROR(get_logger(), "No behavior tree xml specified.");
            return false;
        }

        return true;
    }

    void WaypointFollower::onLoadMapTimeout() const
    {
        RCLCPP_ERROR(get_logger(), "Load map timeout");
    }

    rcl_interfaces::msg::SetParametersResult WaypointFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        // No locking required as action server is running on same single threaded executor
        rcl_interfaces::msg::SetParametersResult result;

        for (auto parameter : parameters)
        {
            const auto& type = parameter.get_type();
            const auto& name = parameter.get_name();

            if (type == ParameterType::PARAMETER_INTEGER)
            {
                if (name == "loop_rate")
                {
                    loop_rate_ = parameter.as_int();
                }
            }
            else if (type == ParameterType::PARAMETER_BOOL)
            {
                if (name == "stop_on_failure")
                {
                    stop_on_failure_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }

}  // namespace navigo_waypoint_follower

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(navigo_waypoint_follower::WaypointFollower)
