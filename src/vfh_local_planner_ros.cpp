#include "vfh_local_planner/vfh_local_planner_ros.h"

#include <pluginlib/class_list_macros.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace vfh_local_planner {
VFHPlannerRos::VFHPlannerRos() : costmap_ros_(nullptr), tf_(nullptr), config_(new VFHConfig()), initialized_(false) {}

VFHPlannerRos::~VFHPlannerRos(){};

void VFHPlannerRos::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    nh_ = parent;

    auto node = nh_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;

    initialize(node);
    // visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
    // visualization_->on_configure();
    // planner_->setVisualization(visualization_);

    return;
}

void VFHPlannerRos::initialize(nav2_util::LifecycleNode::SharedPtr node) {
    // check if plugin initialized
    if (!initialized_) {
        RCLCPP_INFO(logger_, "Initializing VFH Planner");

        // declare parameters (ros2-dashing)
        config_->declareParameters(node, plugin_name_);

        // get parameters of TebConfig via the nodehandle and override the default
        // config
        config_->loadRosParamFromNodeHandle(node, plugin_name_);

        xy_goal_latch_ = false;
        rotating_to_goal_ = false;
        finding_alternative_way_ = false;

        global_frame_ = costmap_ros_->getGlobalFrameID();
        costmap_ = costmap_ros_->getCostmap();
        initialized_ = true;

        vfh_planner.Reconfigure(*config_);
        vfh_planner.Initialize(costmap_);
    } else {
        RCLCPP_WARN(logger_, "This planner has already been initialized, doing nothing.");
    }
}

void VFHPlannerRos::setPlan(const nav_msgs::msg::Path &orig_global_plan) {
    // check if plugin is initialized
    if (!initialized_) {
        RCLCPP_ERROR(logger_,
                     "teb_local_planner has not been initialized, please "
                     "call initialize() before using this planner");
        return;
    }

    xy_goal_latch_ = false;
    rotating_to_goal_ = true;
    goal_reached_ = false;
    finding_alternative_way_ = false;

    // store the global plan
    global_plan_.poses.clear();
    global_plan_.poses.reserve(orig_global_plan.poses.size());
    for (const auto &in_pose : orig_global_plan.poses) {
        geometry_msgs::msg::PoseStamped out_pose;
        out_pose.pose = in_pose.pose;
        out_pose.header = orig_global_plan.header;
        global_plan_.poses.push_back(out_pose);
    }

    // we do not clear the local planner here, since setPlan is called frequently
    // whenever the global planner updates the plan. the local planner checks
    // whether it is required to reinitialize the trajectory or not within each
    // velocity computation step.

    return;
}

// 计算速度输出的主函数
geometry_msgs::msg::TwistStamped VFHPlannerRos::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                                        const geometry_msgs::msg::Twist &velocity,
                                                                        nav2_core::GoalChecker *goal_checker) {
    // 判断是否初始化成功
    if (!initialized_) {
        throw nav2_core::PlannerException(
            std::string("vfh_local_planner has not been initialized, please call "
                        "initialize() before using this planner"));
    }
    geometry_msgs::msg::TwistStamped cmd_vel;

    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
        RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    } else {
        config_->config.xy_goal_tolerance = pose_tolerance.position.x;
    }

    // ============================= 1、计算得到全局、局部目标点；需要跟踪的路径段 =================================
    // 全局目标点
    const geometry_msgs::msg::PoseStamped &plan_goal = global_plan_.poses.back();
    // 局部跟踪路径（从全局路径上裁切得到）
    std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
    if (!transformGlobalPlan(*tf_, global_plan_, pose, *costmap_, global_frame_, transformed_plan)) {
        throw nav2_core::PlannerException(std::string("Could not transform the global plan to the frame of the controller"));
    }

    // 局部目标点
    geometry_msgs::msg::PoseStamped intermediary_goal_point;
    if (!transformed_plan.empty()) {
        int point_index = GetPlanPoint(transformed_plan, global_plan_, pose);
        intermediary_goal_point = transformed_plan.at(point_index);
    } else {
        RCLCPP_ERROR(logger_, "Plan is empty");
        if (!config_->config.ignore_global_plan_updates) return cmd_vel;
    }

    // ============================= 2、更新极坐标系直方图 =================================
    // 更新vfh的极坐标系直方图
    if (!vfh_planner.UpdateHistogram(costmap_)) {
        RCLCPP_WARN(logger_, "Could not find clear direction");
        return cmd_vel;
    }

    // ============================= 3、判读是否到达目标点 =================================
    // 检测机器人是否到达目标点、到达目标点是否需要旋转到指定方向
    double goal_x = plan_goal.pose.position.x;
    double goal_y = plan_goal.pose.position.y;
    if (getGoalPositionDistance(pose, goal_x, goal_y) <= config_->config.xy_goal_tolerance || xy_goal_latch_) {
        xy_goal_latch_ = true;
        double goal_th = tf2::getYaw(plan_goal.pose.orientation);
        if (fabs(getGoalOrientationAngleDifference(pose, goal_th)) <= config_->config.yaw_goal_tolerance) {
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            goal_reached_ = true;
            return cmd_vel;
        } else {
            vfh_planner.RotateToGoal(pose, velocity, goal_th, cmd_vel);
            return cmd_vel;
        }
    }

    // ============================= 4、寻找满足条件好的控制方向 =================================
    double direction_to_follow;
    double goal_distance;
    // 是否需要寻找新的方向
    if (finding_alternative_way_) {
        double global_plan_goal_angle = atan2((plan_goal.pose.position.y - pose.pose.position.y), (plan_goal.pose.position.x - pose.pose.position.x));
        goal_distance = 0.2;
        // 全局目标点方向是否可行
        if (vfh_planner.DirectionIsClear(global_plan_goal_angle)) {
            direction_to_follow = global_plan_goal_angle;
            goal_distance = std::min(
                sqrt(pow((plan_goal.pose.position.x - pose.pose.position.x), 2) + pow((plan_goal.pose.position.y - pose.pose.position.y), 2)), 0.3);
        } else {
            // 全局目标点方向不可行的情况下，找到最好的一个山谷方向
            direction_to_follow = vfh_planner.GetNewDirection(global_plan_goal_angle, tf2::getYaw(pose.pose.orientation), previews_direction);
        }
    } else {
        // 首先判断局部目标点方向是否可行
        double intermediary_goal_orientation =
            atan2((intermediary_goal_point.pose.position.y - pose.pose.position.y), (intermediary_goal_point.pose.position.x - pose.pose.position.x));

        // 该方向不可通行
        if (!vfh_planner.DirectionIsClear(intermediary_goal_orientation)) {
            finding_alternative_way_ = true;
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            return cmd_vel;
        } else {
            direction_to_follow = intermediary_goal_orientation;
            goal_distance = sqrt(pow((intermediary_goal_point.pose.position.x - pose.pose.position.x), 2) +
                                 pow((intermediary_goal_point.pose.position.y - pose.pose.position.y), 2));
        }
    }
    // 记录当前运动控制的方向 后面会将方向转换为v,w
    previews_direction = direction_to_follow;

    // ============================= 5、将控制方向转换为v,w =================================
    // 首先旋转到需要控制的目标朝向
    if (rotating_to_goal_) {
        vfh_planner.RotateToGoal(pose, velocity, direction_to_follow, cmd_vel);
        if (fabs(getGoalOrientationAngleDifference(pose, direction_to_follow)) < config_->config.yaw_goal_tolerance) {
            rotating_to_goal_ = false;
        }
        return cmd_vel;
    } else if (fabs(getGoalOrientationAngleDifference(pose, direction_to_follow)) > M_PI / 3) {
        // 二次判断是否需要旋转
        rotating_to_goal_ = true;
        cmd_vel.twist.linear.x = 0.05;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
    }
    // 向着目标点方向移动（计算v,w）
    else {
        vfh_planner.DriveToward(angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), direction_to_follow), goal_distance, cmd_vel);
    }
    return cmd_vel;
}

int VFHPlannerRos::GetPlanPoint(std::vector<geometry_msgs::msg::PoseStamped> transformed_plan, nav_msgs::msg::Path &global_plan,
                                geometry_msgs::msg::PoseStamped current_pose) {
    int point = 0;
    for (int i = 0; i < transformed_plan.size(); i++) {
        double point_distance = sqrt(pow((transformed_plan.at(i).pose.position.x - current_pose.pose.position.x), 2) +
                                     pow((transformed_plan.at(i).pose.position.y - current_pose.pose.position.y), 2));
        if (point_distance < 0.6) {
            point = i;
        } else {
            if (point) break;
        }
    }
    return point;
}

void VFHPlannerRos::setSpeedLimit(const double &speed_limit, const bool &percentage) {
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
        // Restore default value
        config_->config.max_vel_x = config_->config.max_vel_x;
        config_->config.max_vel_th = config_->config.max_vel_th;
        config_->config.min_vel_th = config_->config.min_vel_th;
    } else {
        if (percentage) {
            // Speed limit is expressed in % from maximum speed of robot
            config_->config.max_vel_x = config_->config.max_vel_x * speed_limit / 100.0;
            config_->config.max_vel_th = config_->config.max_vel_th * speed_limit / 100.0;
            config_->config.min_vel_th = config_->config.min_vel_th * speed_limit / 100.0;
        } else {
            // Speed limit is expressed in absolute value
            // double max_speed_xy = std::max(
            //     std::max(config_->config.max_vel_x,config_->config.base_max_vel_x_backwards),config_->config.base_max_vel_y);
            // if (speed_limit < max_speed_xy) {
            //     // Handling components and angular velocity changes:
            //     // Max velocities are being changed in the same proportion
            //     // as absolute linear speed changed in order to preserve
            //     // robot moving trajectories to be the same after speed change.
            //     // G. Doisy: not sure if that's applicable to
            //     base_max_vel_x_backwards. const double ratio = speed_limit /
            //     max_speed_xy; config_->config.max_vel_x =
            //     config_->config.base_max_vel_x * ratio;
            //     config_->config.base_max_vel_x_backwards =
            //     config_->config.base_max_vel_x_backwards * ratio;
            //     config_->config.base_max_vel_y = config_->config.base_max_vel_y *
            //     ratio; config_->config.base_max_vel_theta =
            //     config_->config.base_max_vel_theta * ratio;
            // }
        }
    }
}

void VFHPlannerRos::activate() {
    // visualization_->on_activate();

    return;
}
void VFHPlannerRos::deactivate() {
    // visualization_->on_deactivate();

    return;
}
void VFHPlannerRos::cleanup() {
    // visualization_->on_cleanup();

    return;
}

} // namespace vfh_local_planner

PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlannerRos, nav2_core::Controller)