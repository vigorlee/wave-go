#include "navigo_util/path/path_utils.hpp"

#include "navigo_util/coordinate/coordinate.h"

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace navigo_util
{
    namespace path
    {
        void PoseStampedToTrajectoryPoint(
            const geometry_msgs::msg::PoseStamped& pose_stamped, robots_dog_msgs::msg::TrajectoryPoint& trajectory_point)
        {
            trajectory_point.header = pose_stamped.header;
            trajectory_point.x      = pose_stamped.pose.position.x;
            trajectory_point.y      = pose_stamped.pose.position.y;
            trajectory_point.z      = pose_stamped.pose.position.z;
            trajectory_point.r      = navigo_util::coordinate::GetYawFromQuaternion(pose_stamped.pose.orientation);
        }

        void TrajectoryPointToPoseStamped(
            const robots_dog_msgs::msg::TrajectoryPoint& trajectory_point, geometry_msgs::msg::PoseStamped& pose_stamped)
        {
            pose_stamped.header          = trajectory_point.header;
            pose_stamped.pose.position.x = trajectory_point.x;
            pose_stamped.pose.position.y = trajectory_point.y;
            pose_stamped.pose.position.z = trajectory_point.z;
            navigo_util::coordinate::CreateQuaternionFromYaw(pose_stamped.pose.orientation, trajectory_point.r);
        }

        void PathToTrajectory(const nav_msgs::msg::Path& path, robots_dog_msgs::msg::Trajectory& trajectory)
        {
            trajectory.header = path.header;
            trajectory.points.clear();
            for (const auto& p : path.poses)
            {
                robots_dog_msgs::msg::TrajectoryPoint trajectory_point;
                PoseStampedToTrajectoryPoint(p, trajectory_point);
                trajectory.points.push_back(trajectory_point);
            }
        }

        void TrajectoryToPath(const robots_dog_msgs::msg::Trajectory& trajectory, nav_msgs::msg::Path& path)
        {
            path.header = trajectory.header;
            path.poses.clear();
            for (const auto& p : trajectory.points)
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                TrajectoryPointToPoseStamped(p, pose_stamped);
                path.poses.push_back(pose_stamped);
            }
        }

        std::map<std::string, fs::path> findMatchingFiles(const fs::path& base_path, const std::string& extension)
        {
            // Normalize extension to include leading dot
            std::string ext = extension;
            if (!ext.empty() && ext.front() != '.')
            {
                ext.insert(ext.begin(), '.');
            }

            // Extract the filename without extension to serve as the matching target
            auto target = base_path.stem().string();
            // Get the directory containing the base file
            auto parent_dir = base_path.parent_path();

            std::map<std::string, fs::path> matching_files;
            try
            {
                // Iterate over all entries in the parent directory
                for (const auto& entry : fs::directory_iterator(parent_dir))
                {
                    // Check for regular files with the requested extension
                    if (entry.is_regular_file() && entry.path().extension() == ext)
                    {
                        // Get the filename without extension
                        auto filename_stem = entry.path().stem().string();
                        // Does the stem start with our target?
                        if (filename_stem.rfind(target, 0) == 0)
                        {
                            // Determine the suffix (what follows the target in the filename)
                            auto suffix = filename_stem.substr(target.size());
                            if (suffix.empty())
                            {
                                suffix = "original";
                            }
                            // Record the file under the computed suffix
                            matching_files[suffix] = entry.path();
                        }
                    }
                }
            }
            catch (const fs::filesystem_error& e)
            {
                std::cerr << "filesystem_error: " << e.what() << std::endl;
            }

            return matching_files;
        }

        bool selectReferencePath(
            const std::map<std::string, fs::path>& candidate_paths_map, const std::string& extra_info, std::string& selected_path)
        {
            bool     target_path_found = false;
            fs::path target_path;

            // Search for a suffix that contains the extra_info substring
            for (const auto& [suffix, path] : candidate_paths_map)
            {
                if (suffix.find(extra_info) != std::string::npos)
                {
                    target_path       = path;
                    target_path_found = true;
                    break;
                }
            }

            if (!target_path_found)
            {
                std::cerr << "Unable to find reference path matching " << extra_info << std::endl;
                return false;
            }

            // Verify that the selected file actually exists on disk
            if (!fs::exists(target_path))
            {
                std::cerr << "Reference Path " << target_path << " does not exist" << std::endl;
                return false;
            }

            // Convert path to string and log the selection
            selected_path = target_path.string();
            std::cout << "Selected path: " << selected_path << std::endl;

            return true;
        }

        bool parse(
            const std::string& file_path, std::vector<geometry_msgs::msg::PoseStamped>& key_poses, robots_dog_msgs::msg::Trajectory& path)
        {
            std::ifstream infile(file_path);
            if (!infile.is_open())
            {
                std::cerr << "Failed to open file " << file_path << std::endl;
                return false;
            }

            enum class Section
            {
                NONE,
                KEY_POSES,
                PATH
            };
            auto        current_section = Section::NONE;
            std::string line;

            while (std::getline(infile, line))
            {
                if (line.empty())
                    continue;

                if (line.find("key_poses") != std::string::npos)
                {
                    current_section = Section::KEY_POSES;
                    continue;
                }
                if (line.find("path") != std::string::npos)
                {
                    current_section = Section::PATH;
                    continue;
                }

                std::istringstream iss(line);
                double             x, y, yaw;
                if (!(iss >> x >> y >> yaw))
                {
                    continue;
                }

                robots_dog_msgs::msg::TrajectoryPoint p;
                p.x = x;
                p.y = y;
                p.r = yaw;

                if (current_section == Section::KEY_POSES)
                {
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    navigo_util::path::TrajectoryPointToPoseStamped(p, pose_stamped);
                    key_poses.push_back(pose_stamped);
                }
                else if (current_section == Section::PATH)
                {
                    path.points.push_back(p);
                }
            }
            infile.close();

            if (path.points.empty())
            {
                std::cerr << "Empty path" << std::endl;
                return false;
            }

            geometry_msgs::msg::PoseStamped start, end;
            navigo_util::path::TrajectoryPointToPoseStamped(path.points.front(), start);
            navigo_util::path::TrajectoryPointToPoseStamped(path.points.back(), end);

            if (key_poses.empty())
            {
                key_poses.push_back(start);
                key_poses.push_back(end);
            }

            // check if start and end in key poses
            bool start_found = false, end_found = false;
            for (const auto& p : key_poses)
            {
                if (p == start)
                {
                    start_found = true;
                }
                if (p == end)
                {
                    end_found = true;
                }
            }
            if (!start_found)
            {
                std::vector<geometry_msgs::msg::PoseStamped> new_key_poses;
                new_key_poses.push_back(start);
                for (const auto& p : key_poses)
                {
                    new_key_poses.push_back(p);
                }
                key_poses = new_key_poses;
            }
            if (!end_found)
            {
                std::vector<geometry_msgs::msg::PoseStamped> new_key_poses;
                for (const auto& p : key_poses)
                {
                    new_key_poses.push_back(p);
                }
                new_key_poses.push_back(end);
                key_poses = new_key_poses;
            }

            if (key_poses.size() < 2)
            {
                std::cerr << "The num of key poses " << key_poses.size() << " < 2" << std::endl;
                return false;
            }

            return true;
        }

        bool separatePath(const std::vector<geometry_msgs::msg::PoseStamped>& key_poses, const robots_dog_msgs::msg::Trajectory& path,
            std::vector<int>& indices, std::vector<robots_dog_msgs::msg::Trajectory>& paths)
        {
            for (const auto& key_pose : key_poses)
            {
                double min_distance = std::numeric_limits<double>::max();
                int    best_index   = -1;
                for (size_t i = 0; i < path.points.size(); ++i)
                {
                    double dx   = key_pose.pose.position.x - path.points[i].x;
                    double dy   = key_pose.pose.position.y - path.points[i].y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < min_distance)
                    {
                        min_distance = dist;
                        best_index   = static_cast<int>(i);
                    }
                }
                if (best_index != -1)
                {
                    indices.push_back(best_index);
                }
            }
            // remove duplicates and sort
            std::sort(indices.begin(), indices.end());
            indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

            if (indices.size() != key_poses.size())
            {
                std::cerr << "indices size does not match number of key poses" << std::endl;
                return false;
            }

            // separate: evey segment contains both start and end poses
            for (size_t i = 0; i < indices.size() - 1; i++)
            {
                robots_dog_msgs::msg::Trajectory segment;
                for (int j = indices[i]; j <= indices[i + 1]; j++)
                {
                    segment.points.push_back(path.points[j]);
                }
                paths.push_back(segment);
            }

            if (paths.size() != key_poses.size() - 1)
            {
                std::cerr << "paths size does not match number of key poses" << std::endl;
                return false;
            }

            return true;
        }

        BestMatchInfo findBestTrajectoryMatch(const std::vector<robots_dog_msgs::msg::Trajectory>& trajectories,
            const geometry_msgs::msg::PoseStamped& robot_pose, double w_distance, double w_heading, double w_progress, double eps_seg_len2)
        {
            BestMatchInfo best_match{};
            best_match.segment_index = -1;
            best_match.point_index   = -1;
            best_match.t             = 0.0;
            best_match.cost          = std::numeric_limits<double>::infinity();

            // Extract robot yaw and position for reuse
            double      robot_yaw = navigo_util::coordinate::GetYawFromQuaternion(robot_pose.pose.orientation);
            const auto& rp        = robot_pose.pose.position;

            // Iterate over each trajectory
            for (size_t seg_idx = 0; seg_idx < trajectories.size(); ++seg_idx)
            {
                const auto& path = trajectories[seg_idx];
                if (path.points.size() < 2)
                    continue;  // skip too-short trajectories

                size_t num_segments = path.points.size() - 1;

                // Iterate over each segment within the trajectory
                for (size_t i = 0; i < num_segments; ++i)
                {
                    const auto& p1 = path.points[i];
                    const auto& p2 = path.points[i + 1];

                    // Compute segment vector v = p2 - p1
                    double dx    = p2.x - p1.x;
                    double dy    = p2.y - p1.y;
                    double norm2 = dx * dx + dy * dy;

                    double t, proj_x, proj_y;
                    // Handle degenerate (very short) segments as a single point
                    if (norm2 < eps_seg_len2)
                    {
                        t      = 0.0;
                        proj_x = p1.x;
                        proj_y = p1.y;
                    }
                    else
                    {
                        // Vector from p1 to robot
                        double ux = rp.x - p1.x;
                        double uy = rp.y - p1.y;

                        // Project u onto v: t = (u·v) / |v|^2
                        t = (ux * dx + uy * dy) / norm2;

                        // Clamp t to [0,1] and compute projection
                        if (t <= 0.0)
                        {
                            t      = 0.0;
                            proj_x = p1.x;
                            proj_y = p1.y;
                        }
                        else if (t >= 1.0)
                        {
                            t      = 1.0;
                            proj_x = p2.x;
                            proj_y = p2.y;
                        }
                        else
                        {
                            proj_x = p1.x + t * dx;
                            proj_y = p1.y + t * dy;
                        }
                    }

                    // 1) Compute distance error: |robot - proj|
                    double dx_r           = rp.x - proj_x;
                    double dy_r           = rp.y - proj_y;
                    double distance_error = std::hypot(dx_r, dy_r);

                    // 2) Compute heading error: |robot_yaw - segment_yaw|
                    double segment_yaw = std::atan2(dy, dx);
                    double yaw_diff    = robot_yaw - segment_yaw;
                    // Normalize yaw difference to [-π, π]
                    if (yaw_diff > M_PI)
                        yaw_diff -= 2 * M_PI;
                    else if (yaw_diff < -M_PI)
                        yaw_diff += 2 * M_PI;
                    double orientation_error = std::fabs(yaw_diff);

                    // 3) Compute progress-based penalty: later segments penalized more
                    double normalized_progress = (static_cast<double>(i) + t) / static_cast<double>(num_segments);
                    double progress_penalty    = w_progress * normalized_progress;

                    // 4) Compute combined cost
                    double cost = w_distance * distance_error + w_heading * orientation_error + progress_penalty;

                    // Update best_match if this segment is better
                    if (cost < best_match.cost)
                    {
                        best_match.cost          = cost;
                        best_match.segment_index = static_cast<int>(seg_idx);
                        best_match.point_index   = static_cast<int>(i);
                        best_match.t             = t;
                    }
                }
            }

            return best_match;
        }

        std::vector<robots_dog_msgs::msg::Trajectory> trimTrajectorySegments(
            const std::vector<robots_dog_msgs::msg::Trajectory>& trajectories, const BestMatchInfo& best_match)
        {
            std::vector<robots_dog_msgs::msg::Trajectory> trimmed;

            // If no valid match was found, return an empty vector.
            if (best_match.segment_index < 0 || best_match.segment_index >= static_cast<int>(trajectories.size()))
            {
                return trimmed;
            }

            // Create a new version of the best trajectory segment.
            robots_dog_msgs::msg::Trajectory new_best_segment;
            const auto&                      orig_best_segment = trajectories[best_match.segment_index];

            // Set same header as origin
            new_best_segment.header = orig_best_segment.header;

            // Ensure that the best segment contains at least two poses.
            if (orig_best_segment.points.size() < 2 || best_match.point_index < 0
                || best_match.point_index + 1 >= static_cast<int>(orig_best_segment.points.size()))
            {
                return trimmed;
            }

            // Get the two adjacent poses (p1 and p2) where the best match was found.
            const auto&     p1 = orig_best_segment.points[best_match.point_index];
            const auto&     p2 = orig_best_segment.points[best_match.point_index + 1];
            Eigen::Vector2d v(p2.x - p1.x, p2.y - p1.y);

            // Compute the projection point using the stored parameter t.
            Eigen::Vector2d proj = Eigen::Vector2d(p1.x, p1.y) + best_match.t * v;

            // Create a new PoseStamped for the new starting point of the best segment.
            auto trimmed_start = orig_best_segment.points[best_match.point_index];
            trimmed_start.x    = proj.x();
            trimmed_start.y    = proj.y();

            // Build the new best segment starting from the computed projection point.
            new_best_segment.points.push_back(trimmed_start);
            for (size_t j = best_match.point_index + 1; j < orig_best_segment.points.size(); ++j)
            {
                new_best_segment.points.push_back(orig_best_segment.points[j]);
            }

            // The new trimmed trajectories consists of the new best segment followed by all subsequent segments.
            trimmed.push_back(new_best_segment);
            for (size_t seg = best_match.segment_index + 1; seg < trajectories.size(); ++seg)
            {
                trimmed.push_back(trajectories[seg]);
            }
            return trimmed;
        }

        std::vector<geometry_msgs::msg::PoseStamped> trimKeyPoses(
            const std::vector<geometry_msgs::msg::PoseStamped>& key_poses, const BestMatchInfo& best_match)
        {
            std::vector<geometry_msgs::msg::PoseStamped> trimmed_key_poses;
            // Check if the best match index is valid in the context of key_poses.
            if (best_match.segment_index < 0 || best_match.segment_index >= static_cast<int>(key_poses.size()))
            {
                return trimmed_key_poses;
            }

            // Keep only the key poses corresponding to the remaining segments.
            // (Assuming key_poses[i] is the endpoint of trajectory segment i.)
            for (size_t i = best_match.segment_index; i < key_poses.size(); ++i)
            {
                trimmed_key_poses.push_back(key_poses[i]);
            }

            return trimmed_key_poses;
        }

    }  // namespace path
}  // namespace navigo_util