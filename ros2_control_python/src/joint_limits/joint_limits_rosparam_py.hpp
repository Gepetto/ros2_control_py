#pragma once

// pybind11
#include <pybind11/pybind11.h>

// joint_limits
#include <joint_limits/joint_limits_rosparam.hpp>
// impl_ros2_control_py
#include <impl_ros2_control_py.hpp>

namespace ros2_control_py::bind_joint_limits
{

namespace py = pybind11;
using namespace joint_limits;

inline void init_joint_limits_rosparam([[maybe_unused]] py::module &m)
{
  m.def("declare_parameters", [](const std::string& joint_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf) { return declare_parameters(joint_name, param_itf, logging_itf); });

  m.def("declare_parameters", [](const std::string& joint_name, const rclcpp::Node::SharedPtr& node) { return declare_parameters(joint_name, node); });

  m.def("declare_parameters", [](const std::string& joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr& lifecycle_node) { return declare_parameters(joint_name, lifecycle_node); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf, JointLimits& limits) { return get_joint_limits(joint_name, param_itf, logging_itf, limits); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp::Node::SharedPtr& node, JointLimits& limits) { return get_joint_limits(joint_name, node, limits); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr& lifecycle_node, JointLimits& limits) { return get_joint_limits(joint_name, lifecycle_node, limits); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf, SoftJointLimits& soft_limits) { return get_joint_limits(joint_name, param_itf, logging_itf, soft_limits); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp::Node::SharedPtr& node, SoftJointLimits& soft_limits) { return get_joint_limits(joint_name, node, soft_limits); });

  m.def("get_joint_limits", [](const std::string& joint_name, const rclcpp_lifecycle::LifecycleNode::SharedPtr& lifecycle_node, SoftJointLimits& soft_limits) { return get_joint_limits(joint_name, lifecycle_node, soft_limits); });
}

}
