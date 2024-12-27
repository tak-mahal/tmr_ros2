/*********************************************************************
 *  run_moveit_cpp.cpp
 * 
 *  Various portions of the code are based on original source from 
 *  PickNik Inc.
 *  and are used in accordance with the following license. */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
   Desc: A simple demo node running MoveItCpp for planning and execution
*/

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {

    // シミュレーションモードか実行モードかを指定
    const bool simulation_mode = true;

    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm("tm12_arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // baseやtsumikiを追加
    moveit_msgs::msg::CollisionObject bpl;
    bpl.id = "base_plate";
    bpl.header.frame_id = "base";
    bpl.primitives.resize(1);
    bpl.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    bpl.primitives[0].dimensions = { 2.0, 2.0, 0.015 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.0075 ;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.orientation = tf2::toMsg(q);
    bpl.pose = pose;
    //planning_scene_interface.applyCollisionObject(bpl);
    bpl.operation = bpl.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(bpl);
    }  // Unlock PlanningScene

    std::ifstream file_pose("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/mtc_tutorial/src/poses.csv");
    std::string line_pose;
    int i = 0;
    while (std::getline(file_pose, line_pose)) {
      std::stringstream ss_pose(line_pose);
      std::string value_pose;
      std::vector<double> pose_values;

      while (std::getline(ss_pose, value_pose, ',')) {
        pose_values.push_back(std::stod(value_pose));
      }

      moveit_msgs::msg::CollisionObject object;
      object.id = "tsumiki_" + std::to_string(i) ;
      object.header.frame_id = "base";
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      object.primitives[0].dimensions = { 0.108, 0.036, 0.018 };

      geometry_msgs::msg::Pose pose;
      pose.position.x = pose_values[0];
      pose.position.y = pose_values[1];
      pose.position.z = pose_values[2] - 0.009 ;
      tf2::Quaternion q;
      q.setRPY(pose_values[3], pose_values[4], pose_values[5]);
      pose.orientation = tf2::toMsg(q);
      object.pose = pose;
      //planning_scene_interface.applyCollisionObject(object);
      object.operation = object.ADD;

      {  // Lock PlanningScene
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(object);
      }  // Unlock PlanningScene

      i++;
    }
  
    // CSVファイルを読み取る
    std::ifstream file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/mtc_tutorial/src/targets.csv");
    std::string line;
    std::vector<int> failed_targets_indices;
    int index = 0;


    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      std::vector<double> pose_values;

      while (std::getline(ss, value, ',')) {
        pose_values.push_back(std::stod(value));
      }

      if (pose_values.size() == 6) {

        //auto stage = std::make_unique<mtc::stages::MoveTo>("move to pose", sampling_planner);
        //stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world";
        target_pose_msg.pose.position.x = pose_values[0];
        target_pose_msg.pose.position.y = pose_values[1];
        target_pose_msg.pose.position.z = pose_values[2];
        tf2::Quaternion q;
        q.setRPY(pose_values[3], pose_values[4], pose_values[5]);
        target_pose_msg.pose.orientation = tf2::toMsg(q);

        // アプローチ用のターゲットBを計算
        geometry_msgs::msg::PoseStamped approach_pose_msg = target_pose_msg;
        approach_pose_msg.pose.position.z += 0.05; // 200mm上方

        auto plan_and_execute = [&](const geometry_msgs::msg::PoseStamped& pose, bool cartesian) {  
            bool success = false;  
  
            if (cartesian) {  
                // PlanningSceneの取得  
                auto planning_scene = moveit_cpp_->getPlanningScene();  
                auto robot_state = planning_scene->getCurrentStateNonConst();  
                const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("tm12_arm");  
  
                // カートesianパスの計算  
                std::vector<robot_state::RobotStatePtr> trajectory;  
                const double jump_threshold = 0.0;  
                const double eef_step = 0.01;  
  
                double fraction = robot_state->computeCartesianPath(  
                    joint_model_group,  
                    trajectory,  
                    robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),  
                    pose.pose,  
                    true,  
                    eef_step,  
                    jump_threshold);  
          
                success = (fraction >= 0.95);  
  
                if (success) {  
                    // TrajectoryをRobotTrajectoryに変換  
                    robot_trajectory::RobotTrajectory rt(robot_state->getRobotModel(), "tm12_arm");  
                    for (const auto& state : trajectory)  
                        rt.addSuffixWayPoint(state, 0.01);  
  
                    // PlanningComponentでexecuteするための準備  
                    moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;  
                    rt.getRobotTrajectoryMsg(robot_trajectory_msg);  
                    arm.setGoal(robot_trajectory_msg);  
                }  
            } else {  
                arm.setGoal(pose, "flange");  
                const auto plan_solution = arm.plan();  
                success = static_cast<bool>(plan_solution);  
            }  
  
            if (success) {  
                const auto plan_solution = arm.plan();  
                if (plan_solution) {  
                    RCLCPP_INFO(LOGGER, "Plan to goal");  
                    arm.execute();  
                } else {  
                    RCLCPP_ERROR(LOGGER, "Planning failed!");  
                }  
            } else {  
                RCLCPP_ERROR(LOGGER, "Cartesian path computation failed!");  
  
                // 失敗した場合、ターゲットをZ軸中心に180度回転させ、回転中心をX方向に+0.044m移動させて再試行  
                geometry_msgs::msg::PoseStamped new_pose = pose;  
                double shift_x = 0.044;  
  
                // 位置を回転中心に合わせてシフト  
                new_pose.pose.position.x -= shift_x;  
  
                // オリエンテーションをZ軸に180度回転  
                tf2::Quaternion q;  
                tf2::fromMsg(new_pose.pose.orientation, q);  
                tf2::Quaternion q_rot;  
                q_rot.setRPY(0, 0, M_PI); // Z軸に180度回転  
                q = q * q_rot;  
                new_pose.pose.orientation = tf2::toMsg(q);  
  
                // 回転後の位置を元に戻す  
                new_pose.pose.position.x += shift_x;  
  
                arm.setGoal(new_pose);  
                const auto new_plan_solution = arm.plan();  
  
                if (new_plan_solution) {  
                    RCLCPP_INFO(LOGGER, "Plan to adjusted goal");  
                    arm.execute();  
                } else {  
                    RCLCPP_ERROR(LOGGER, "Planning failed again after rotation!");  
                    if (simulation_mode) {  
                        failed_targets_indices.push_back(index);  
                        RCLCPP_INFO(node_->get_logger(), "Failed target index: %d", index);  
                    }  
                }  
            }  
        };  

        // アプローチ用のターゲットBに移動（通常のプランニング）
        plan_and_execute(approach_pose_msg, false);

        // ターゲットAに移動（直線運動）
        plan_and_execute(target_pose_msg, true);

        // 再びアプローチ用のターゲットBに戻る（直線運動）
        plan_and_execute(approach_pose_msg, true);

        index++;
    
      }
    }

    if (simulation_mode && !failed_targets_indices.empty()) {
        std::ofstream failed_targets_file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/mtc_tutorial/src/failed_targets.csv");
        if (failed_targets_file.is_open()) {
            for (int failed_index : failed_targets_indices) {
                failed_targets_file << failed_index << std::endl;
            }
            failed_targets_file.close();
            RCLCPP_INFO(node_->get_logger(), "Failed target indices written to file.");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open output CSV file.");
        }
    }

    /*
    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    arm.setGoal("home");

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }
    //*/


    //Below, we simply use a long delay to wait for the previous motion to complete.
    /*rclcpp::sleep_for(std::chrono::seconds(10));   

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal (home)");
    arm.setGoal("home");

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to home");
    plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }*/
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(20));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
