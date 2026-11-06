cmake_minimum_required(VERSION 3.8)
project(robots_dog_msgs
        VERSION 0.5.0
)

set(CMAKE_CXX_STANDARD 17)
option(USE_VERSION_FILE "use version file" OFF)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
        "msg/BmsCmd.msg"
        "msg/BmsState.msg"
        "msg/Cartesian.msg"
        "msg/CmdVelWithTrajectory.msg"
        "msg/CpuInfo.msg"
        "msg/CustomMsg.msg"
        "msg/CustomPoint.msg"
        "msg/DiskInfo.msg"
        "msg/ElectronicMap.msg"
        "msg/ElectronicObstacle.msg"
        "msg/FaultInfo.msg"
        "msg/FaultInfoMultiArray.msg"
        "msg/Frontier.msg"
        "msg/HighCmd.msg"
        "msg/HighLevelCmd.msg"
        "msg/HighLevelRobotState.msg"
        "msg/HighState.msg"
        "msg/IMU.msg"
        "msg/LED.msg"
        "msg/LegControlData.msg"
        "msg/LinktrackAoaNode0.msg"
        "msg/Localization.msg"
        "msg/LowCmd.msg"
        "msg/LowLevelCmd.msg"
        "msg/LowLevelRobotState.msg"
        "msg/LowState.msg"
        "msg/MonitorInfo.msg"
        "msg/MotorCmd.msg"
        "msg/MotorState.msg"
        "msg/NavigationError.msg"
        "msg/NavigationErrorReport.msg"
        "msg/NavigationErrorSet.msg"
        "msg/NavigationState.msg"
        "msg/NetInfo.msg"
        "msg/Nmea.msg"
        "msg/ObstacleInstance2D.msg"
        "msg/ObstacleScan.msg"
        "msg/PlannerDebugInfo.msg"
        "msg/Pose6D.msg"
        "msg/PredictedPath.msg"
        "msg/PredictedPathArray.msg"
        "msg/PredictedPose.msg"
        "msg/ProcessInfo.msg"
        "msg/RectangleArray.msg"
        "msg/RectangleWithCentre.msg"
        "msg/Relocalization.msg"
        "msg/RslidarPacket.msg"
        "msg/SlamState.msg"
        "msg/StartNavigation.msg"
        "msg/Trajectory.msg"
        "msg/TrajectoryPoint.msg"
        "msg/UniBestNav.msg"
        "msg/UniHeading.msg"
        "msg/UniRtkPvh.msg"
        "msg/UwbPointStamped.msg"
)

set(srv_files
        "srv/GetFrontierCentroids.srv"
        "srv/GetOptimizedPath.srv"
        "srv/IsExplorationGoalValid.srv"
        "srv/IsExplorationGoalsValid.srv"
        "srv/IsPathValid.srv"
        "srv/IsTargetInFov.srv"
        "srv/MapState.srv"
        "srv/RecordPath.srv"
        "srv/RecordKeyPose.srv"
        "srv/LocalizationState.srv"
        "srv/LoadMap.srv"
        "srv/MappingType.srv"
        "srv/UseMapType.srv"
        "srv/SetLocalizationType.srv"
)

set(action_files
        "action/BackUp.action"
        "action/ComputePathsToPoses.action"
        "action/ComputePathThroughPoses.action"
        "action/ComputePathToPose.action"
        "action/ExploreThroughPoses.action"
        "action/FollowPath.action"
        "action/FollowWaypoints.action"
        "action/NavigateThroughPoses.action"
        "action/NavigateToPose.action"
        "action/ProcessInfo.action"
)

# 使用rosidl生成器生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        ${action_files}
        DEPENDENCIES std_msgs nav_msgs geometry_msgs sensor_msgs diagnostic_msgs
)

# 安装其他项目的需求
ament_export_dependencies(rosidl_default_runtime)
install(
        DIRECTORY msg
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY srv
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY action
        DESTINATION share/${PROJECT_NAME}
)
ament_package()

if (USE_VERSION_FILE)
    execute_process(
            COMMAND git log -1 --format=%cd --date=iso
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_LAST_COMMIT_DATE
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX REPLACE " [+-][0-9]{4}$" "" GIT_LAST_COMMIT_DATE_SHORT "${GIT_LAST_COMMIT_DATE}")

    execute_process(
            COMMAND git rev-parse HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_HASH_FULL
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
    )


    include(CMakePackageConfigHelpers)
    set(VERSION_PATH "/opt/robot/robot_version_manager/robots_dog_msgs/")
    configure_package_config_file(
            ${CMAKE_CURRENT_LIST_DIR}/config/version.yaml.in
            ${CMAKE_BINARY_DIR}/version.yaml
            INSTALL_DESTINATION ${VERSION_PATH}
    )

    install(FILES ${CMAKE_BINARY_DIR}/version.yaml
            DESTINATION ${VERSION_PATH})
endif ()

# enabel deb package
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/$ENV{ROS_DISTRO}")
set(CPACK_PACKAGE_NAME robots_dog_msgs)
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}_${PROJECT_VERSION}_${CMAKE_HOST_SYSTEM_PROCESSOR}_$ENV{ROS_DISTRO}_${CMAKE_SYSTEM_NAME})
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "kanonwy")

include(CPack)
