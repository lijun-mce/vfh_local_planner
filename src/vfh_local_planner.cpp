#include "vfh_local_planner/vfh_local_planner.h"

namespace vfh_local_planner {
VFHPlanner::VFHPlanner() {}

VFHPlanner::~VFHPlanner() {}

// 读取配置参数
void VFHPlanner::Reconfigure(const VFHConfig &cfg) { config_ = &cfg; }

// 舒适化参数
bool VFHPlanner::Initialize(nav2_costmap_2d::Costmap2D *costmap) {
    window_width = costmap->getSizeInCellsX();
    window_height = costmap->getSizeInCellsY();
    cmd_vel_linear_x_ = 0;
    cmd_vel_angular_z_ = 0;
    Alocate();
    UpdateHistogram(costmap);
    return true;
}

// 分配内存空间
void VFHPlanner::Alocate() {
    vfh_histogram.resize(config_->config.vfh_sections_number, 0);
    // 初始化角度和距离的二维矩阵
    costmap_cells_angle.resize(window_width, std::vector<double>(window_height));
    costmap_cells_distance.resize(window_width, std::vector<double>(window_height));

    for (int y = 0; y < window_height; y++) {
        for (int x = 0; x < window_width; x++) {
            // 以局部地图为中心点初始化 角度和距离 二维矩阵  X轴正方向为0°~360°，顺时针方向旋转
            costmap_cells_angle[x][y] = radToDeg(getCoordinateAngle(x - (window_width / 2), (window_height / 2) - y));
            // 计算得到距离
            costmap_cells_distance[x][y] = getCoordinateDistance(x - (window_width / 2), (window_height / 2) - y);
        }
    }
}

// 基于costmap 更新直方图
bool VFHPlanner::UpdateHistogram(nav2_costmap_2d::Costmap2D *costmap) {
    // 先将直方图初始化为0
    fill(vfh_histogram.begin(), vfh_histogram.end(), 0);

    for (int y = 0; y < window_height; y++) {
        for (int x = 0; x < window_width; x++) {
            // 得到当前直方图的序号（rint 相近取整）
            double cell_sector = rint(costmap_cells_angle[x][y] / (360 / (config_->config.vfh_sections_number - 1)));
            // 得到对应栅格代价值  TODO:值越大越不能通行
            double distance_cost = 100 / (1 + exp((config_->config.increase_rate * costmap_cells_distance[x][y]) -
                                                  (config_->config.increase_rate * config_->config.vhf_detection_range)));
            double magnitude = pow(((costmap->getCost(x, window_height - y - 1)) / 254), 2);
            // 进行累加 得到当前区域的代价值
            vfh_histogram[cell_sector] += magnitude * distance_cost;
        }
    }

    SmoothHistogram();

    GetCandidateValleys();
    // 失败
    if (candidate_valleys.size() < 1) return false;

    return true;
}

// 平滑直方图
void VFHPlanner::SmoothHistogram() {
    std::vector<double> smoothed_histogram(config_->config.vfh_sections_number);
    double smoothed;
    for (int i = 0; i < config_->config.vfh_sections_number; i++) {
        // smooth_length理解为取均值的个数  一般选择5
        smoothed = config_->config.smooth_length * vfh_histogram[i];
        for (int k = 1; k < config_->config.smooth_length - 1; k++) {
            int lower_it = (i - k < 0) ? config_->config.vfh_sections_number - i - k : i - k;
            int upper_it = (i + k > config_->config.vfh_sections_number - 1) ? i + k - config_->config.vfh_sections_number : i + k;

            smoothed += (config_->config.smooth_length - k) * vfh_histogram[lower_it] + (config_->config.smooth_length - k) * vfh_histogram[upper_it];
        }
        smoothed_histogram[i] = smoothed / (2 * config_->config.smooth_length + 1);
    }
    vfh_histogram = smoothed_histogram;
}

// 找出机器人能通过的所有山谷
void VFHPlanner::GetCandidateValleys() {
    candidate_valleys.clear();
    rejected_peaks.clear();
    std::vector<int> valley;
    for (int i = 0; i < config_->config.vfh_sections_number; i++) {
        if (vfh_histogram[i] < config_->config.vfh_threshold) {
            valley.push_back(i);
        } else {
            rejected_peaks.push_back(i);
            if (valley.size() > 0) candidate_valleys.push_back(valley);
            valley.clear();
        }
    }
    // 确保最后一次的山谷区域被推入
    if (valley.size() > 0) candidate_valleys.push_back(valley);

    // 合并起始位置
    if (candidate_valleys.size() > 1) {
        if (candidate_valleys.front().front() == 0 && candidate_valleys.back().back() == config_->config.vfh_sections_number - 1) {
            candidate_valleys.back().insert(candidate_valleys.back().end(), candidate_valleys.front().begin(), candidate_valleys.front().end());
            candidate_valleys.erase(candidate_valleys.begin());
        }
    }
    for (int i = 0; i < candidate_valleys.size(); i++) {
        // 判断为山谷区域最小连续个数
        if (candidate_valleys.at(i).size() <= config_->config.very_narrow_valley_threshold) {
            rejected_peaks.insert(rejected_peaks.end(), candidate_valleys.at(i).begin(), candidate_valleys.at(i).end());
            // 移除当前山谷
            candidate_valleys.erase(candidate_valleys.begin() + i);
        }
    }
}

// 检查给定的方向是否干净
bool VFHPlanner::DirectionIsClear(double goal_direction) {
    int goal_sector = rint(radToDeg(goal_direction) / (360 / (config_->config.vfh_sections_number - 1)));

    for (int k = 0; k <= (config_->config.very_narrow_valley_threshold / 2) - 1; k++) {
        // 上区域得分
        int upper_sector =
            (goal_sector + k > config_->config.vfh_sections_number - 1) ? goal_sector + k - config_->config.vfh_sections_number : goal_sector + k;
        // 下区域得分
        int lower_sector = (goal_sector - k < 0) ? config_->config.vfh_sections_number - goal_sector - k : goal_sector - k;
        // 如果有一个任意一个区域大于容许代价，返回错误（即存在障碍物）
        if (vfh_histogram.at(upper_sector) > config_->config.vfh_threshold || vfh_histogram.at(lower_sector) > config_->config.vfh_threshold)
            return false;
    }
    /*
    bool side_obstructed = false;
    for (int i=0; i < rejected_peaks.size(); i++)
    {
        for (int k=1; k <=
    (int)(config_->config.very_narrow_valley_threshold/2)-1; k++)
        {
            int upper_sector = (goal_sector+k <
    config_->config.vfh_sections_number-1)? goal_sector+k -
    config_->config.vfh_sections_number: goal_sector+k; int lower_sector =
    (goal_sector-k < 0)? config_->config.vfh_sections_number - goal_sector-k:
    goal_sector-k; if ((upper_sector == rejected_peaks.at(i)) || (lower_sector ==
    rejected_peaks.at(i))) return false;
        }

        if (goal_sector == rejected_peaks.at(i))
        {
            return false;
        }
    }
    */
    return true;
}

// 根据目标方向获得新的避障方向 输入:机器人到全局目标点方向、机器人当前方向、前一时刻运动方向
double VFHPlanner::GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction) {
    double goal_diff;
    double curr_direction_diff;
    double prev_direction_diff;
    double direction_cost;
    // 目标点权重  当前方向权重  先前方向权重(只是初始化)
    double smallest_cost =
        (M_PI * config_->config.goal_weight) + (M_PI * config_->config.curr_direction_weight) + (M_PI * config_->config.prev_direction_weight);
    int best_valley;
    bool valley_front;

    for (int i = 0; i < candidate_valleys.size(); i++) {
        // 计算每个山谷的最前面一个方向与三个输入的角度差，然后求代价
        goal_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).front() * (360 / (config_->config.vfh_sections_number - 1))), global_plan_goal_direction));
        curr_direction_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).front() * (360 / (config_->config.vfh_sections_number - 1))), current_robot_direction));
        prev_direction_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).front() * (360 / (config_->config.vfh_sections_number - 1))), previews_direction));
        direction_cost = (goal_diff * config_->config.goal_weight) + (curr_direction_diff * config_->config.curr_direction_weight) +
                         (prev_direction_diff * config_->config.prev_direction_weight);
        if (direction_cost < smallest_cost) {
            smallest_cost = direction_cost;
            best_valley = i;
            valley_front = true;
        }
        // 计算每个山谷的最后面一个方向与三个输入的角度差，然后求代价
        goal_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).back() * (360 / (config_->config.vfh_sections_number - 1))), global_plan_goal_direction));
        curr_direction_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).back() * (360 / (config_->config.vfh_sections_number - 1))), current_robot_direction));
        prev_direction_diff = fabs(angles::shortest_angular_distance(
            degToRad(candidate_valleys.at(i).back() * (360 / (config_->config.vfh_sections_number - 1))), global_plan_goal_direction));
        direction_cost = (goal_diff * config_->config.goal_weight) + (curr_direction_diff * config_->config.curr_direction_weight) +
                         (prev_direction_diff * config_->config.prev_direction_weight);
        if (direction_cost < smallest_cost) {
            smallest_cost = direction_cost;
            best_valley = i;
            valley_front = false;
        }
    }
    // 找到最好的山谷
    int valley_length = candidate_valleys.at(best_valley).size();
    double deviation_angle;
    // 如果该山谷的长度小于 宽山谷的阈值 直接取该山谷的中间方向作为前进方向
    if (valley_length < config_->config.wide_valley_threshold) {
        deviation_angle = candidate_valleys.at(best_valley).at(floor(valley_length / 2)) * (360 / (config_->config.vfh_sections_number - 1));
    } else {
        // 否则从改山谷的前面或者后面开始，选取宽山谷阈值的一半方向作为前进方向
        if (valley_front) {
            deviation_angle = candidate_valleys.at(best_valley).at(floor(config_->config.wide_valley_threshold / 2)) *
                              (360 / (config_->config.vfh_sections_number - 1));
        } else {
            deviation_angle = candidate_valleys.at(best_valley).at(valley_length - 1 - floor(config_->config.wide_valley_threshold / 2)) *
                              (360 / (config_->config.vfh_sections_number - 1));
        }
    }
    return degToRad(deviation_angle);
}

// 向目标方向旋转
bool VFHPlanner::RotateToGoal(const geometry_msgs::msg::PoseStamped &global_pose, const geometry_msgs::msg::Twist &robot_vel, double goal_th,
                              geometry_msgs::msg::TwistStamped &cmd_vel) {
    cmd_vel.twist.linear.x = 0.0;

    double yaw = tf2::getYaw(global_pose.pose.orientation);
    double vel_yaw = robot_vel.angular.z;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(config_->config.max_vel_th, std::max(config_->config.min_in_place_vel_th, ang_diff))
                                         : std::max(config_->config.min_vel_th, std::min(-1.0 * config_->config.min_in_place_vel_th, ang_diff));

    // take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + config_->config.acc_lim_theta / config_->config.local_planner_frequence;
    double min_acc_vel = fabs(vel_yaw) - config_->config.acc_lim_theta / config_->config.local_planner_frequence;

    v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    // we also want to make sure to send a velocity that allows us to stop when we
    // reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * config_->config.acc_lim_theta * fabs(ang_diff));

    v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce config_->config.min_in_place_vel_th.  It is more important than
    // the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0 ? std::min(config_->config.max_vel_th, std::max(config_->config.min_in_place_vel_th, v_theta_samp))
                                      : std::max(config_->config.min_vel_th, std::min(-1.0 * config_->config.min_in_place_vel_th, v_theta_samp));

    cmd_vel_angular_z_ = v_theta_samp;
    cmd_vel.twist.angular.z = v_theta_samp;
    return true;
}

// Get speeds to drive the robot towards the goal
//  第一个参数：from当前位置 to 跟踪位置的正则化角度；
//  第二个参数：目标点的距离；第三个参数：输出的vel
void VFHPlanner::DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::msg::TwistStamped &cmd_vel) {
    double x_speed = std::min(goal_distance, cmd_vel_linear_x_ + config_->config.acc_lim_x / config_->config.local_planner_frequence);

    x_speed = std::min(x_speed, config_->config.max_vel_x);

    double th_speed = (angle_to_goal < 0)
                          ? std::max(angle_to_goal, cmd_vel_angular_z_ - config_->config.acc_lim_theta / config_->config.local_planner_frequence)
                          : std::min(angle_to_goal, cmd_vel_angular_z_ + config_->config.acc_lim_theta / config_->config.local_planner_frequence);

    th_speed = (th_speed > 0) ? std::min(th_speed, config_->config.max_vel_th) : std::max(th_speed, config_->config.min_vel_th);

    cmd_vel_linear_x_ = x_speed;
    cmd_vel_angular_z_ = th_speed;
    cmd_vel.twist.linear.x = x_speed;
    cmd_vel.twist.angular.z = th_speed;
}

} // namespace vfh_local_planner