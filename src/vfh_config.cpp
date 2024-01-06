
#include "vfh_local_planner/vfh_config.h"

using nav2_util::declare_parameter_if_not_declared;

namespace vfh_local_planner {
void VFHConfig::declareParameters(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) {
    node_name = name;

    declare_parameter_if_not_declared(nh, name + "." + "vfh_sections_number", rclcpp::ParameterValue(config.vfh_sections_number));
    declare_parameter_if_not_declared(nh, name + "." + "vhf_detection_range", rclcpp::ParameterValue(config.vhf_detection_range));
    declare_parameter_if_not_declared(nh, name + "." + "increase_rate", rclcpp::ParameterValue(config.increase_rate));
    declare_parameter_if_not_declared(nh, name + "." + "goal_weight", rclcpp::ParameterValue(config.goal_weight));
    declare_parameter_if_not_declared(nh, name + "." + "curr_direction_weight", rclcpp::ParameterValue(config.curr_direction_weight));
    declare_parameter_if_not_declared(nh, name + "." + "prev_direction_weight", rclcpp::ParameterValue(config.prev_direction_weight));
    declare_parameter_if_not_declared(nh, name + "." + "smooth_length", rclcpp::ParameterValue(config.smooth_length));
    declare_parameter_if_not_declared(nh, name + "." + "front_angle", rclcpp::ParameterValue(config.front_angle));
    declare_parameter_if_not_declared(nh, name + "." + "vfh_threshold", rclcpp::ParameterValue(config.vfh_threshold));
    declare_parameter_if_not_declared(nh, name + "." + "wide_valley_threshold", rclcpp::ParameterValue(config.wide_valley_threshold));
    declare_parameter_if_not_declared(nh, name + "." + "very_narrow_valley_threshold", rclcpp::ParameterValue(config.very_narrow_valley_threshold));
    declare_parameter_if_not_declared(nh, name + "." + "max_vel_x", rclcpp::ParameterValue(config.max_vel_x));
    declare_parameter_if_not_declared(nh, name + "." + "max_vel_th", rclcpp::ParameterValue(config.max_vel_th));
    declare_parameter_if_not_declared(nh, name + "." + "min_in_place_vel_th", rclcpp::ParameterValue(config.min_in_place_vel_th));
    declare_parameter_if_not_declared(nh, name + "." + "acc_lim_x", rclcpp::ParameterValue(config.acc_lim_x));
    declare_parameter_if_not_declared(nh, name + "." + "acc_lim_theta", rclcpp::ParameterValue(config.acc_lim_theta));
    declare_parameter_if_not_declared(nh, name + "." + "xy_goal_tolerance", rclcpp::ParameterValue(config.xy_goal_tolerance));
    declare_parameter_if_not_declared(nh, name + "." + "yaw_goal_tolerance", rclcpp::ParameterValue(config.yaw_goal_tolerance));
    declare_parameter_if_not_declared(nh, name + "." + "local_planner_frequence", rclcpp::ParameterValue(config.local_planner_frequence));
    declare_parameter_if_not_declared(nh, name + "." + "ignore_global_plan_updates", rclcpp::ParameterValue(config.ignore_global_plan_updates));
    declare_parameter_if_not_declared(nh, name + "." + "restore_defaults", rclcpp::ParameterValue(config.restore_defaults));
}

void VFHConfig::loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) {
    nh->get_parameter_or(name + "." + "vfh_sections_number", config.vfh_sections_number, config.vfh_sections_number);
    nh->get_parameter_or(name + "." + "vhf_detection_range", config.vhf_detection_range, config.vhf_detection_range);
    nh->get_parameter_or(name + "." + "increase_rate", config.increase_rate, config.increase_rate);
    nh->get_parameter_or(name + "." + "goal_weight", config.goal_weight, config.goal_weight);
    nh->get_parameter_or(name + "." + "curr_direction_weight", config.curr_direction_weight, config.curr_direction_weight);
    nh->get_parameter_or(name + "." + "prev_direction_weight", config.prev_direction_weight, config.prev_direction_weight);
    nh->get_parameter_or(name + "." + "smooth_length", config.smooth_length, config.smooth_length);
    nh->get_parameter_or(name + "." + "front_angle", config.front_angle, config.front_angle);
    nh->get_parameter_or(name + "." + "vfh_threshold", config.vfh_threshold, config.vfh_threshold);
    nh->get_parameter_or(name + "." + "wide_valley_threshold", config.wide_valley_threshold, config.wide_valley_threshold);
    nh->get_parameter_or(name + "." + "very_narrow_valley_threshold", config.very_narrow_valley_threshold, config.very_narrow_valley_threshold);
    nh->get_parameter_or(name + "." + "max_vel_x", config.max_vel_x, config.max_vel_x);
    nh->get_parameter_or(name + "." + "max_vel_th", config.max_vel_th, config.max_vel_th);
    nh->get_parameter_or(name + "." + "min_in_place_vel_th", config.min_in_place_vel_th, config.min_in_place_vel_th);
    nh->get_parameter_or(name + "." + "acc_lim_x", config.acc_lim_x, config.acc_lim_x);
    nh->get_parameter_or(name + "." + "acc_lim_theta", config.acc_lim_theta, config.acc_lim_theta);
    nh->get_parameter_or(name + "." + "xy_goal_tolerance", config.xy_goal_tolerance, config.xy_goal_tolerance);
    nh->get_parameter_or(name + "." + "yaw_goal_tolerance", config.yaw_goal_tolerance, config.yaw_goal_tolerance);
    nh->get_parameter_or(name + "." + "local_planner_frequence", config.local_planner_frequence, config.local_planner_frequence);
    nh->get_parameter_or(name + "." + "ignore_global_plan_updates", config.ignore_global_plan_updates, config.ignore_global_plan_updates);
    nh->get_parameter_or(name + "." + "restore_defaults", config.restore_defaults, config.restore_defaults);
}

rcl_interfaces::msg::SetParametersResult VFHConfig::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    std::lock_guard<std::mutex> l(config_mutex_);

    for (auto parameter : parameters) {
        const auto& type = parameter.get_type();
        const auto& name = parameter.get_name();

        if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
            if (name == node_name + ".vhf_detection_range") {
                config.vhf_detection_range = parameter.as_double();
            } else if (name == node_name + ".increase_rate") {
                config.increase_rate = parameter.as_double();
            } else if (name == node_name + ".vfh_threshold") {
                config.vfh_threshold = parameter.as_double();
            } else if (name == node_name + ".max_vel_x") {
                config.max_vel_x = parameter.as_double();
            } else if (name == node_name + ".max_vel_th") {
                config.max_vel_th = parameter.as_double();
            } else if (name == node_name + ".min_vel_th") {
                config.min_vel_th = parameter.as_double();
            } else if (name == node_name + ".min_in_place_vel_th") {
                config.min_in_place_vel_th = parameter.as_double();
            } else if (name == node_name + ".acc_lim_x") {
                config.acc_lim_x = parameter.as_double();
            } else if (name == node_name + ".acc_lim_theta") {
                config.acc_lim_theta = parameter.as_double();
            } else if (name == node_name + ".xy_goal_tolerance") {
                config.xy_goal_tolerance = parameter.as_double();
            } else if (name == node_name + ".local_planner_frequence") {
                config.local_planner_frequence = parameter.as_double();
            }
        }

        else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        }

        else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
            if (name == node_name + ".vfh_sections_number") {
                config.vfh_sections_number = parameter.as_int();
            } else if (name == node_name + ".goal_weight") {
                config.goal_weight = parameter.as_int();
            } else if (name == node_name + ".curr_direction_weight") {
                config.curr_direction_weight = parameter.as_int();
            } else if (name == node_name + ".prev_direction_weight") {
                config.prev_direction_weight = parameter.as_int();
            } else if (name == node_name + ".smooth_length") {
                config.smooth_length = parameter.as_int();
            } else if (name == node_name + ".front_angle") {
                config.front_angle = parameter.as_int();
            } else if (name == node_name + ".wide_valley_threshold") {
                config.wide_valley_threshold = parameter.as_int();
            } else if (name == node_name + ".very_narrow_valley_threshold") {
                config.very_narrow_valley_threshold = parameter.as_int();
            }
        }

        else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
            if (name == node_name + ".ignore_global_plan_updates") {
                config.ignore_global_plan_updates = parameter.as_bool();
            } else if (name == node_name + ".restore_defaults") {
                config.restore_defaults = parameter.as_bool();
            }
        }

        else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            // if (name == node_name + ".costmap_converter_plugin") {
            //     obstacles.costmap_converter_plugin = parameter.as_string();
            // }
        }
    }
    // checkParameters();

    result.successful = true;
    return result;
}

} // namespace vfh_local_planner