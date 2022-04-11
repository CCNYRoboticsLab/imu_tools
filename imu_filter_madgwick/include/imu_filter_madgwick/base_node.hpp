#pragma once

#include <rclcpp/rclcpp.hpp>

namespace imu_filter {

class BaseNode : public rclcpp::Node
{
  public:
    explicit BaseNode(std::string name, const rclcpp::NodeOptions &options)
        : Node(name, options)
    {
    }

    typedef struct {
        double from_value;
        double to_value;
        double step;
    } floating_point_range;

    typedef struct {
        int from_value;
        int to_value;
        int step;
    } integer_range;

    // Declare a parameter that has no integer or floating point range
    // constraints
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    // Declare a parameter that has a floating point range constraint
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const floating_point_range fp_range,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;
        descriptor.floating_point_range.resize(1);
        descriptor.floating_point_range[0].from_value = fp_range.from_value;
        descriptor.floating_point_range[0].to_value = fp_range.to_value;
        descriptor.floating_point_range[0].step = fp_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    // Declare a parameter that has an integer range constraint
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const integer_range int_range,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;
        descriptor.integer_range.resize(1);
        descriptor.integer_range[0].from_value = int_range.from_value;
        descriptor.integer_range[0].to_value = int_range.to_value;
        descriptor.integer_range[0].step = int_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }
};

}  // namespace imu_filter
