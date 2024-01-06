
#ifndef VFH_CONFIG_H_
#define VFH_CONFIG_H_

#include <memory>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_2d_utils/parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vfh_local_planner {
class VFHConfig {
public:
    using UniquePtr = std::unique_ptr<VFHConfig>;

    std::string node_name; //!< node name used for parameter event callback

    struct Config {
        int vfh_sections_number;          // Number of sector in the histogram.72, 8, 360
        double vhf_detection_range;       // Obstacle sensitivity distance.
        double increase_rate;             // Magnitude gradient increase rate.0.5, 0, 30.0
        int goal_weight;                  // Weigth of goal direction for the direction cost function.
        int curr_direction_weight;        // Weigth of current direction for the direction cost function.
        int prev_direction_weight;        // Weigth of previews direction for the direction cost function.
        int smooth_length;                // Number of sectors considered in the smoothing.
        int front_angle;                  // Number of sectors that is considered in front of the robot.
        double vfh_threshold;             // Threshold of VFH magnitude.
        int wide_valley_threshold;        // Minimum number of sector of wide valley.
        int very_narrow_valley_threshold; // Minimum number of sector for a valid valley.

        double max_vel_x;  // The maximum x velocity for the robot in m/s.
        double max_vel_th; // The absolute value of the maximum rotational velocity for the robot in rad/s.
        double min_vel_th; // The absolute value of the minimum rotational velocity for the robot in rad/s.

        double min_in_place_vel_th;     // The absolute value of the minimum rotational velocity for the robot when it's stopped in rad/s.
        double acc_lim_x;               // The acceleration limit of the robot in the x direction.
        double acc_lim_theta;           // The acceleration limit of the robot for rotation.
        double xy_goal_tolerance;       // Maximal distance to the goal position.
        double yaw_goal_tolerance;      // Accuracy of the orientation to the goal orientation.
        double local_planner_frequence; // Need to set to the same value as the local planner frequence (controller frequence) in move base.

        bool ignore_global_plan_updates; // Don't follows global plan after enconter obstacle on the path.
        bool restore_defaults;           // Retore to the default configuration.
    } config;

    VFHConfig() {
        config.vfh_sections_number = 72;
        config.vhf_detection_range = 15.0;
        config.increase_rate = 0.5;
        config.goal_weight = 1;
        config.curr_direction_weight = 1;
        config.prev_direction_weight = 1;
        config.smooth_length = 5;
        config.front_angle = 1;
        config.vfh_threshold = 160;
        config.wide_valley_threshold = 12;
        config.very_narrow_valley_threshold = 6;
        config.max_vel_x = 0.7;
        config.max_vel_th = 1.55;
        config.min_vel_th = -1.55;
        config.min_in_place_vel_th = 1.0;
        config.acc_lim_x = 0.5;
        config.acc_lim_theta = 0.6;
        config.xy_goal_tolerance = 0.1;
        config.yaw_goal_tolerance = 0.1;
        config.local_planner_frequence = 5;
        config.ignore_global_plan_updates = true;
        config.restore_defaults = false;
    }

    void declareParameters(const nav2_util::LifecycleNode::SharedPtr, const std::string name);

    /**
     * @brief Load parmeters from the ros param server.
     * @param nh const reference to the local rclcpp::Node::SharedPtr
     */
    void loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name);

    /**
     * @brief Callback executed when a paramter change is detected
     * @param parameters list of changed parameters
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    // /**
    //  * @brief Check parameters and print warnings in case of discrepancies
    //  *
    //  * Call this method whenever parameters are changed using public interfaces to inform the user
    //  * about some improper uses.
    //  */
    // void checkParameters() const;

    // /**
    //  * @brief Check if some deprecated parameters are found and print warnings
    //  * @param nh const reference to the local rclcpp::Node::SharedPtr
    //  */
    // void checkDeprecated(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) const;

    /**
     * @brief Return the internal config mutex
     */
    std::mutex& configMutex() { return config_mutex_; }

private:
    std::mutex config_mutex_; //!< Mutex for config accesses and changes
    rclcpp::Logger logger_{rclcpp::get_logger("VHFPlanner")};
};
} // namespace vfh_local_planner

#endif