#include <rclcpp/rclcpp.hpp>  
#include <moveit/move_group_interface/move_group_interface.h>  
#include <moveit/planning_scene_interface/planning_scene_interface.h>  
#include <moveit_msgs/msg/robot_trajectory.hpp>  
#include <yaml-cpp/yaml.h>  
#include <fstream>  
  
void savePlanToYAML(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& filename)  
{  
    YAML::Emitter out;  
    out << YAML::BeginMap;  
    out << YAML::Key << "trajectory" << YAML::Value << YAML::BeginSeq;  
    for (const auto& point : plan.trajectory_.joint_trajectory.points)  
    {  
        out << YAML::BeginMap;  
        out << YAML::Key << "positions" << YAML::Value << YAML::Flow << point.positions;  
        out << YAML::Key << "velocities" << YAML::Value << YAML::Flow << point.velocities;  
        out << YAML::Key << "accelerations" << YAML::Value << YAML::Flow << point.accelerations;  
        out << YAML::EndMap;  
    }  
    out << YAML::EndSeq;  
    out << YAML::EndMap;  
  
    std::ofstream fout(filename);  
    fout << out.c_str();  
    fout.close();  
}  
  
int main(int argc, char** argv)  
{  
    rclcpp::init(argc, argv);  
    auto node = rclcpp::Node::make_shared("plan_saver");  
  
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");  
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  
    move_group.setPlanningTime(10.0);  
  
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;  
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
  
    if (success)  
    {  
        RCLCPP_INFO(node->get_logger(), "Plan successful. Saving to YAML.");  
        savePlanToYAML(my_plan, "plan.yaml");  
    }  
    else  
    {  
        RCLCPP_ERROR(node->get_logger(), "Failed to plan.");  
    }  
  
    rclcpp::shutdown();  
    return 0;  
}  