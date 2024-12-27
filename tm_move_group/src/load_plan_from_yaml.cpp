#include <rclcpp/rclcpp.hpp>  
#include <moveit/move_group_interface/move_group_interface.h>  
#include <yaml-cpp/yaml.h>  
#include <trajectory_msgs/msg/joint_trajectory.hpp>  
  
moveit::planning_interface::MoveGroupInterface::Plan loadPlanFromYAML(const std::string& filename)  
{  
    moveit::planning_interface::MoveGroupInterface::Plan plan;  
    YAML::Node node = YAML::LoadFile(filename);  
  
    const auto& trajectory = node["trajectory"];  
    for (const auto& point_node : trajectory)  
    {  
        trajectory_msgs::msg::JointTrajectoryPoint point;  
        point.positions = point_node["positions"].as<std::vector<double>>();  
        point.velocities = point_node["velocities"].as<std::vector<double>>();  
        point.accelerations = point_node["accelerations"].as<std::vector<double>>();  
        plan.trajectory_.joint_trajectory.points.push_back(point);  
    }  
  
    return plan;  
}  
  
int main(int argc, char** argv)  
{  
    rclcpp::init(argc, argv);  
    auto node = rclcpp::Node::make_shared("plan_executor");  
  
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");  
  
    RCLCPP_INFO(node->get_logger(), "Loading plan from YAML.");  
    auto plan = loadPlanFromYAML("plan.yaml");  
  
    RCLCPP_INFO(node->get_logger(), "Executing plan.");  
    move_group.execute(plan);  
  
    rclcpp::shutdown();  
    return 0;  
}  