#ifndef VFH_LOCAL_PLANNER_H_
#define VFH_LOCAL_PLANNER_H_

#include <stdio.h>

#include <vector>

// costmap & geometry
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vfh_local_planner/utils.h"
#include "vfh_local_planner/vfh_config.h"

namespace vfh_local_planner {
class VFHPlanner {
public:
    VFHPlanner();
    ~VFHPlanner();

    void Reconfigure(const VFHConfig& cfg);
    bool Initialize(nav2_costmap_2d::Costmap2D* costmap);
    void Alocate();
    bool UpdateHistogram(nav2_costmap_2d::Costmap2D* costmap);
    void SmoothHistogram();
    void GetCandidateValleys();

    bool RotateToGoal(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::Twist& robot_vel, double goal_th,
                      geometry_msgs::msg::TwistStamped& cmd_vel);
    void DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::msg::TwistStamped& cmd_vel);
    bool DirectionIsClear(double goal_direction);
    double GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction);

private:
    // 地图宽 、 高
    int window_width;
    int window_height;
    // vfh 分割数量
    int vfh_sections_number;
    // 检测范围 增长比例
    double vhf_detection_range, increase_rate;
    int goal_weight, curr_direction_weight, prev_direction_weight;
    int smooth_length;

    // 判断为宽山谷的阈值
    int wide_valley_threshold;
    // 判断为山谷区域最小连续个数
    int very_narrow_valley_threshold;
    double max_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th;
    // 线加速度 、 角加速度
    double acc_lim_x, acc_lim_theta;
    // 控制频率，一般情况为 50hz
    double local_planner_frequence;
    // 线速度
    double cmd_vel_linear_x_;
    // 角速度
    double cmd_vel_angular_z_;

    nav2_costmap_2d::Costmap2D* costmap_;
    std::vector<std::vector<double> > costmap_cells_angle;
    std::vector<std::vector<double> > costmap_cells_distance;
    // 向量直方图
    std::vector<double> vfh_histogram;
    // 判断为山谷的阈值条件
    double vfh_threshold;
    // 山谷区域的候选者
    std::vector<std::vector<int> > candidate_valleys;
    // 需要抛弃的山峰
    std::vector<int> rejected_peaks;
    const VFHConfig* config_{nullptr};
};
} // namespace vfh_local_planner

#endif