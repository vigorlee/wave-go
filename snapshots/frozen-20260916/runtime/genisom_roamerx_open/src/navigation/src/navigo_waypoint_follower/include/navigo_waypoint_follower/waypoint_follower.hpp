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

#ifndef NAVIGO_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define NAVIGO_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigo_core/waypoint_task_executor.hpp"
#include "navigo_util/lifecycle_node.hpp"
#include "navigo_util/node_utils.hpp"
#include "navigo_util/simple_action_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robots_dog_msgs/msg/start_navigation.hpp"
#include "robots_dog_msgs/msg/trajectory.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace navigo_waypoint_follower
{

    enum class ActionStatus
    {
        UNKNOWN    = 0,
        PROCESSING = 1,
        FAILED     = 2,
        SUCCEEDED  = 3
    };

    /**
     * @class navigo_waypoint_follower::WaypointFollower
     * @brief An action server that uses behavior tree for navigating a robot to its
     * goal position.
     */
    class WaypointFollower : public navigo_util::LifecycleNode
    {
    public:
        using ActionT              = nav2_msgs::action::FollowWaypoints;
        using ClientT              = nav2_msgs::action::NavigateToPose;
        using ActionServer         = navigo_util::SimpleActionServer<ActionT>;
        using ActionClient         = rclcpp_action::Client<ClientT>;
        using StartNavigation      = robots_dog_msgs::msg::StartNavigation;
        using LoadMapClientT       = nav2_msgs::srv::LoadMap;
        using LoadMapServiceClient = rclcpp::Client<LoadMapClientT>;

        /**
         * @brief A constructor for navigo_waypoint_follower::WaypointFollower class
         * @param options Additional options to control creation of the node.
         */
        explicit WaypointFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        /**
         * @brief A destructor for navigo_waypoint_follower::WaypointFollower class
         */
        ~WaypointFollower();

    protected:
        /**
         * @brief Configures member variables
         *
         * Initializes action server for "follow_waypoints"
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        navigo_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Activates action server
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        navigo_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Deactivates action server
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        navigo_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Resets member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        navigo_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Called when in shutdown state
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        navigo_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

        /**
         * @brief Get action name for this navigator
         * @return string Name of action server
         */
        static std::string getName()
        {
            return std::string("follow_waypoints");
        }

        /**
         * @brief Handle the request to load map timeout situation
         */
        void onLoadMapTimeout() const;

        /**
         * @brief Action server callbacks
         */
        void followWaypoints();

        /**
         * @brief Action client result callback
         * @param result Result of action server updated asynchronously
         */
        void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result);

        /**
         * @brief Action client goal response callback
         * @param goal Response of action server updated asynchronously
         */
        void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr& goal);

        void onSinglePointPlanGoalReceived();

        /**
         * @brief Reset the stored start navigation message to its defalut values
         */
        void resetStartNavigation()
        {
            const StartNavigation msg;
            start_navigation_ = msg;
        }

        /**
         * @brief Load an occupancy grid map from a YAML file.
         *
         * This function parses the specified YAML configuration file to extract
         * parameters such as the map image path, resolution, origin, occupied threshold,
         * and free threshold. It then attempts to load the map image and populate
         * a ROS OccupancyGrid message for subsequent publishing.
         *
         * @param yaml_file The filesystem path to the YAML configuration file.
         */
        void loadMapFromYaml(const std::string& yaml_file);
        /**
         * @brief Handle the asynchronous LoadMap service response.
         *
         * This callback is invoked when the LoadMap service returns a response.
         * It checks the result code in the future, logs success or failure,
         * and—on success—triggers any downstream logic such as starting a patrol
         * or republishing the newly loaded map.
         *
         * @param future A shared future containing the LoadMap service response.
         */
        void onLoadMapResponseReceived(rclcpp::Client<LoadMapClientT>::SharedFuture future);

        /**
         * @brief A subscription and callback to handle the topic-based goal published
         * from rviz
         * @param msg Poses received via a topic
         */
        void onStartNavigationReceived(const StartNavigation::SharedPtr msg);

        /**
         * @brief A function to handle cmd stop
         */
        void onCmdStopReceived();

        /**
         * @brief A function to handle cmd standby
         */
        void onCmdStandByReceived();

        /**
         * @brief A function to handle cmd start
         */
        void onCmdStartReceived();

    private:
        /**
         * @brief Populate a FollowWaypoints action goal with the specified parameters.
         *
         * This function fills in the provided ActionT::Goal object so that the robot can execute
         * a “follow waypoints” behavior. It assigns the function identifier, the sequence of
         * target poses, any corresponding reference paths, the desired motion velocity, and the
         * behavior tree configuration to use during execution.
         *
         * @param function_id        A numeric identifier selecting which follow-waypoints variant or mode to run.
         * @param poses              A vector of Pose messages defining the sequence of waypoints the robot should follow.
         * @param reference_paths    A vector of Path messages representing optional reference trajectories for each segment;
         *                           can be empty if not used.
         * @param desired_velocity   A Vector3 message specifying the desired linear velocity components (x, y, z) for the motion.
         * @param[out] goal          The action goal object to be populated with all of the above parameters.
         *
         * @return True if construct goal succeed
         */
        bool constructFollowWaypointGoal(const uint8_t function_id, const std::vector<geometry_msgs::msg::Pose>& poses,
            const std::vector<robots_dog_msgs::msg::Trajectory>& reference_paths, const geometry_msgs::msg::Vector3& desired_velocity,
            ActionT::Goal& goal) const;

        /**
         * @brief Populate a FollowWaypoints action goal with the specified parameters.
         *
         * This function fills in the provided ActionT::Goal object so that the robot can execute
         * a “follow waypoints” behavior. It assigns the function identifier, the sequence of
         * target poses, any corresponding reference paths, the desired motion velocity, and the
         * behavior tree configuration to use during execution.
         *
         * @param function_id        A numeric identifier selecting which follow-waypoints variant or mode to run.
         * @param poses              A vector of Pose messages defining the sequence of waypoints the robot should follow.
         * @param reference_paths    A vector of Path messages representing optional reference trajectories for each segment;
         *                           can be empty if not used.
         * @param desired_velocity   A vector of desired linear velocity components (x, y, z) for the motion.
         * @param[out] goal          The action goal object to be populated with all of the above parameters.
         *
         * @return True if construct goal succeed
         */
        bool constructFollowWaypointGoal(const uint8_t function_id, const std::vector<geometry_msgs::msg::Pose>& poses,
            const std::vector<robots_dog_msgs::msg::Trajectory>& reference_paths, const std::vector<double>& desired_velocity,
            ActionT::Goal& goal) const
        {
            if (desired_velocity.size() != 3)
            {
                RCLCPP_ERROR(get_logger(), "desired_velocity size %lu must be 3", desired_velocity.size());
                return false;
            }

            geometry_msgs::msg::Vector3 desired_velocity_msg;
            desired_velocity_msg.x = desired_velocity[0];
            desired_velocity_msg.y = desired_velocity[1];
            desired_velocity_msg.z = desired_velocity[2];

            return constructFollowWaypointGoal(function_id, poses, reference_paths, desired_velocity_msg, goal);
        }


        /**
         * @brief Callback executed when a parameter change is detected
         * @param event ParameterEvent message
         */
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

        // Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        // Our action server
        std::unique_ptr<ActionServer>                                           action_server_;
        ActionClient::SharedPtr                                                 nav_to_pose_client_;
        rclcpp::CallbackGroup::SharedPtr                                        callback_group_;
        rclcpp::executors::SingleThreadedExecutor                               callback_group_executor_;
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
        bool                                                                    stop_on_failure_;
        ActionStatus                                                            current_goal_status_;
        int                                                                     loop_rate_;
        std::vector<int>                                                        failed_ids_;

        // Navigation Interface
        StartNavigation                                  start_navigation_;
        rclcpp::Subscription<StartNavigation>::SharedPtr start_navigation_sub_;
        rclcpp::TimerBase::SharedPtr                     load_map_timeout_timer_;

        std::string                               behavior_tree_dir_;
        std::string                               single_point_plan_xml_;
        rclcpp_action::Client<ActionT>::SharedPtr self_client_;

        // Load map client
        LoadMapServiceClient::SharedPtr  load_map_client_;
        rclcpp::CallbackGroup::SharedPtr load_map_callback_group_;

        // Task Execution At Waypoint Plugin
        pluginlib::ClassLoader<navigo_core::WaypointTaskExecutor> waypoint_task_executor_loader_;
        pluginlib::UniquePtr<navigo_core::WaypointTaskExecutor>   waypoint_task_executor_;
        std::string                                               waypoint_task_executor_id_;
        std::string                                               waypoint_task_executor_type_;
    };

}  // namespace navigo_waypoint_follower

#endif  // NAVIGO_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
