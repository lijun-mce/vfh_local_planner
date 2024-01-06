#ifndef NAV2_VFH_LOCAL_PLANNER__VFH_LOCAL_PLANNER_ROS_H_
#define NAV2_VFH_LOCAL_PLANNER__VFH_LOCAL_PLANNER_ROS_H_

#include <angles/angles.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
// transforms
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

// config and parameters
#include "vfh_local_planner/vfh_config.h"
#include "vfh_local_planner/vfh_local_planner.h"

namespace vfh_local_planner {

class VFHPlannerRos : public nav2_core::Controller {
public:
    VFHPlannerRos();

    ~VFHPlannerRos() override;

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void initialize(nav2_util::LifecycleNode::SharedPtr node);

    /**
     * @brief Cleanup controller state machine
     */
    void cleanup() override;

    /**
     * @brief Activate controller state machine
     */
    void activate() override;

    /**
     * @brief Deactivate controller state machine
     */
    void deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
                                                             nav2_core::GoalChecker* /*goal_checker*/) override;

    /**
     * @brief nav2_core setPlan - Sets the global plan
     * @param path The global plan
     */
    void setPlan(const nav_msgs::msg::Path& path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void setSpeedLimit(const double& speed_limit, const bool& percentage);

    inline double getGoalPositionDistance(const geometry_msgs::msg::PoseStamped& global_pose, double goal_x, double goal_y) {
        return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
    }

    inline double getGoalOrientationAngleDifference(const geometry_msgs::msg::PoseStamped& global_pose, double goal_th) {
        double yaw = tf2::getYaw(global_pose.pose.orientation);
        return angles::shortest_angular_distance(yaw, goal_th);
    }

    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const nav_msgs::msg::Path& global_plan, const geometry_msgs::msg::PoseStamped& global_pose,
                             const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                             std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan) {
        transformed_plan.clear();

        if (global_plan.poses.empty()) {
            RCLCPP_ERROR(logger_, "Received plan with zero length");
            return false;
        }

        const geometry_msgs::msg::PoseStamped& plan_pose = global_plan.poses[0];
        try {
            // get plan_to_global_transform from plan frame to global_frame
            geometry_msgs::msg::TransformStamped plan_to_global_transform =
                tf_->lookupTransform(global_frame, tf2_ros::fromMsg(plan_pose.header.stamp), plan_pose.header.frame_id, tf2::timeFromSec(0),
                                     plan_pose.header.frame_id, tf2::durationFromSec(0.5));

            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::msg::PoseStamped robot_pose = tf_->transform(global_pose, plan_pose.header.frame_id);

            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold =
                std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

            unsigned int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 0;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            while (i < (unsigned int)global_plan.poses.size()) {
                double x_diff = robot_pose.pose.position.x - global_plan.poses[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan.poses[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;
                if (sq_dist <= sq_dist_threshold) {
                    break;
                }
                ++i;
            }

            geometry_msgs::msg::PoseStamped newer_pose;

            // now we'll transform until points are outside of our distance threshold
            while (i < (unsigned int)global_plan.poses.size() && sq_dist <= sq_dist_threshold) {
                const geometry_msgs::msg::PoseStamped& pose = global_plan.poses[i];
                tf2::doTransform(pose, newer_pose, plan_to_global_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan.poses[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan.poses[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                ++i;
            }
        } catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
            return false;
        } catch (tf2::ConnectivityException& ex) {
            RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
            return false;
        } catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
            if (!global_plan.poses.empty())
                RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.poses.size(),
                             global_plan.poses[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }

private:
    int GetPlanPoint(std::vector<geometry_msgs::msg::PoseStamped> transformed_plan, nav_msgs::msg::Path& global_plan,
                     geometry_msgs::msg::PoseStamped current_pose);

    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    // pointer to external objects (do NOT delete object)

    rclcpp_lifecycle::LifecycleNode::WeakPtr nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_; ///<@brief pointer to Transform Listener
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; ///<@brief pointer to costmap
    nav2_costmap_2d::Costmap2D* costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("RegulatedPurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;

    nav_msgs::msg::Path global_plan_;
    // Dynamic parameters handler
    std::mutex mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    std::string global_frame_;

    VFHPlanner vfh_planner;

    VFHConfig::UniquePtr config_;

    int vfh_sections_number;
    int smooth_length;
    double vfh_threshold;
    double yaw_goal_tolerance_, xy_goal_tolerance_;
    double previews_direction;

    // flags
    bool rotating_to_goal_;
    bool xy_goal_latch_;
    bool finding_alternative_way_;
    bool initialized_;
    bool goal_reached_;
};

} // namespace vfh_local_planner

#endif