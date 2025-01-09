/*********************************************************************
 *  run_move_group.cpp
 * 
 *  Various portions of the code are based on original source from 
 *  "https://github.com/ros-planning/moveit2/tree/foxy/moveit_demo_nodes/run_move_group/"
 *  and are used in accordance with the following license.
 *  "https://github.com/ros-planning/moveit2/blob/main/LICENSE.txt"
 *********************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html

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

//#include "/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/install/tm_msgs/include/tm_msgs/tm_msgs/srv/set_io.hpp"
#include "tm_msgs/srv/set_io.hpp"
#include "tm_msgs/srv/ask_item.hpp"
#include <yaml-cpp/yaml.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <chrono>

using namespace std::chrono_literals;

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
        out << YAML::Key << "execution_time" << YAML::Value << plan.planning_time_;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();
}

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
        if (node["execution_time"]) {
            plan.planning_time_ = node["execution_time"].as<double>();
        }
    }

    return plan;
}

void saveTargetPoseToYAML(const geometry_msgs::msg::Pose& pose, const std::string& filename)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << pose.position.x;
    out << YAML::Key << "y" << YAML::Value << pose.position.y;
    out << YAML::Key << "z" << YAML::Value << pose.position.z;
    out << YAML::EndMap;
    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << pose.orientation.x;
    out << YAML::Key << "y" << YAML::Value << pose.orientation.y;
    out << YAML::Key << "z" << YAML::Value << pose.orientation.z;
    out << YAML::Key << "w" << YAML::Value << pose.orientation.w;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();
}
geometry_msgs::msg::Pose loadTargetPoseFromYAML(const std::string& filename)
{
    YAML::Node node = YAML::LoadFile(filename);

    geometry_msgs::msg::Pose pose;
    pose.position.x = node["position"]["x"].as<double>();
    pose.position.y = node["position"]["y"].as<double>();
    pose.position.z = node["position"]["z"].as<double>();
    pose.orientation.x = node["orientation"]["x"].as<double>();
    pose.orientation.y = node["orientation"]["y"].as<double>();
    pose.orientation.z = node["orientation"]["z"].as<double>();
    pose.orientation.w = node["orientation"]["w"].as<double>();

    return pose;
}

void set_io(rclcpp::Node::SharedPtr node, int8_t module, int8_t type, int8_t pin, float state) {
  RCLCPP_INFO(node->get_logger(), "before set request");
  auto client = node->create_client<tm_msgs::srv::SetIO>("set_io");
  auto request = std::make_shared<tm_msgs::srv::SetIO::Request>();
  request->module = module;
  request->type = type;
  request->pin = pin;
  request->state = state;

  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available");
    return;
  }
  auto result = client->async_send_request(request);

  // 非同期の結果を待つためにスピンを使う
  while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    rclcpp::spin_some(node);
  }

  if (result.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
    if (result.get()->ok) {
      RCLCPP_INFO(node->get_logger(), "IO set successfully");
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to set IO");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  RCLCPP_INFO(node->get_logger(), "end function");
}

int ask_item(rclcpp::Node::SharedPtr node, const std::string& id, const std::string& item, int wait_time) {
  auto client = node->create_client<tm_msgs::srv::AskItem>("ask_item");
  auto request = std::make_shared<tm_msgs::srv::AskItem::Request>();
  request->id = id;
  request->item = item;
  request->wait_time = wait_time;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return -1;  // エラーを示す値
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // 非同期の結果を待つためにスピンを使う
  while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    rclcpp::spin_some(node);
  }

  if (result.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
    auto getResult = result.get();
    if (getResult->ok) {
      try {
        std::string value_str = getResult->value;
        size_t pos = value_str.find('=');
        if (pos != std::string::npos) {
          std::string number_str = value_str.substr(pos + 1);
          int value = std::stoi(number_str);  // `=`以降の文字列を整数に変換
          RCLCPP_INFO_STREAM(node->get_logger(), "Request successful, extracted value: " << value);
          return value;
        } else {
          RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid format: '=' not found in value");
          return -1;  // エラーを示す値
        }
      } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid argument error in std::stoi: " << e.what());
        return -1;  // エラーを示す値
      } catch (const std::out_of_range &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Out of range error in std::stoi: " << e.what());
        return -1;  // エラーを示す値
      }
    } else {
      RCLCPP_WARN_STREAM(node->get_logger(), "Request not successful");
      return -1;  // エラーを示す値
    }
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to call service");
    return -1;  // エラーを示す値
  }
}

void prompt(const std::string& message)
{
  printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
  fflush(stdout);
  while (std::cin.get() != '\n' && rclcpp::ok())
    ;
}

static const rclcpp::Logger logger = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
 
  int number = 0;
  // コマンドライン引数を処理
  if (argc > 1) {
      //int number;
      std::stringstream ss(argv[1]); // 引数を文字列ストリームに変換
      if (ss >> number) {
          std::cout << "Received integer: " << number << std::endl;
          // 受け取った整数値を使って何らかの処理を行う
          // 例: numberの値を使って計算するなど
      } else {
          std::cerr << "Invalid integer argument: " << argv[1] << std::endl;
      }
  } else {
      std::cerr << "No integer argument provided." << std::endl;
  }

  // シミュレーションモードか実行モードかを指定
  const bool simulation_mode = true;
  const bool use_file = false;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "tmr_arm";

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);
/*
*/
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // add base plate to planning scene
  moveit_msgs::msg::CollisionObject bpl;
  bpl.id = "base_plate";
  bpl.header.frame_id = "base";
  bpl.primitives.resize(1);
  bpl.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  bpl.primitives[0].dimensions = { 2.0, 2.0, 0.015 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = -0.0080 ;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  pose.orientation = tf2::toMsg(q);
  bpl.pose = pose;
  planning_scene_interface.applyCollisionObject(bpl);
  move_group_interface.setSupportSurfaceName("base_plate");

  // add pump rubber cylinder
  moveit_msgs::msg::CollisionObject pr;
  pr.id = "pump_rubber";
  pr.header.frame_id = "flange";
  pr.primitives.resize(1);
  pr.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  pr.primitives[0].dimensions = { 0.020, 0.020 };

  geometry_msgs::msg::Pose pr_pose;
  pr_pose.position.x = 0;
  pr_pose.position.y = -0.044;
  pr_pose.position.z = 0.199 ;
  tf2::Quaternion pr_q;
  pr_q.setRPY(0, 0, 0);
  pr_pose.orientation = tf2::toMsg(pr_q);
  pr.pose = pr_pose;
  planning_scene_interface.applyCollisionObject(pr);

  // add tsumikis to planning scene
  std::ifstream file_pose("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/poses.csv");
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
    object.primitives[0].dimensions = { 0.106, 0.034, 0.016 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_values[0];
    pose.position.y = pose_values[1];
    pose.position.z = pose_values[2] - 0.009 ;
    tf2::Quaternion q;
    q.setRPY(pose_values[3], pose_values[4], pose_values[5]);
    pose.orientation = tf2::toMsg(q);
    object.pose = pose;
    planning_scene_interface.applyCollisionObject(object);

    i++;

  }
  RCLCPP_INFO(node->get_logger(), "tsumiki added");

  // シミュレーションモードの場合に実行速度を高速化
  if (simulation_mode) {
      move_group_interface.setMaxVelocityScalingFactor(1.0); // 速度スケーリングを最大に
      move_group_interface.setMaxAccelerationScalingFactor(1.0); // 加速度スケーリングを最大に
  }else{
      move_group_interface.setMaxVelocityScalingFactor(0.4); // 速度スケーリングを5%に
      move_group_interface.setMaxAccelerationScalingFactor(0.4); // 加速度スケーリングを5%に
  }


  // CSVファイルを読み取る
  //std::ifstream file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/mtc_tutorial/src/targets.csv");
  std::ifstream pick_file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/pick.csv");
  std::ifstream place_file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/place.csv");
  std::ifstream tsumiki_ids_file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/tsumiki_ids.csv");
  std::string pick_line, place_line, id_line;
  std::vector<int> failed_targets_indices;

  std::string plan_folder = "/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/plan/";
  std::string pose_folder = "/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/tmr_ros2/tm_move_group/src/pose/";

  int index = 0;
  // bool tsumiki_on = false;
  // IOの初期地をセット
  set_io(node, tm_msgs::srv::SetIO::Request::MODULE_ENDEFFECTOR, tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT, 0, tm_msgs::srv::SetIO::Request::STATE_ON);
  set_io(node, tm_msgs::srv::SetIO::Request::MODULE_ENDEFFECTOR, tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT, 1, tm_msgs::srv::SetIO::Request::STATE_ON);

  while (rclcpp::ok() && std::getline(pick_file, pick_line) && std::getline(place_file, place_line) && std::getline(tsumiki_ids_file, id_line)) {
    std::stringstream pick_ss(pick_line);
    std::stringstream place_ss(place_line);
    std::stringstream id_ss(id_line);
    //std::string value;
    //std::vector<double> pose_values;
    std::vector<double> pick_pose_values, place_pose_values;
    std::string value;
    int tsumiki_id;

    //while (std::getline(ss, value, ',')) {
    //  pose_values.push_back(std::stod(value));
    //}
    // Pick pose values
    while (std::getline(pick_ss, value, ',')) {
      pick_pose_values.push_back(std::stod(value));
    }

    // Place pose values
    while (std::getline(place_ss, value, ',')) {
      place_pose_values.push_back(std::stod(value));
    }

    // Tsumiki ID
    id_ss >> tsumiki_id;
    std::string object_name = "tsumiki_" + std::to_string(tsumiki_id);


    if (pick_pose_values.size() == 6 && place_pose_values.size() == 6) {

      // Prepare pick pose
      geometry_msgs::msg::PoseStamped pick_pose_msg;
      pick_pose_msg.header.frame_id = "world";
      pick_pose_msg.pose.position.x = pick_pose_values[0];
      pick_pose_msg.pose.position.y = pick_pose_values[1];
      pick_pose_msg.pose.position.z = pick_pose_values[2];
      tf2::Quaternion pick_q;
      pick_q.setRPY(pick_pose_values[3], pick_pose_values[4], pick_pose_values[5]);
      pick_pose_msg.pose.orientation = tf2::toMsg(pick_q);

      // Prepare place pose
      geometry_msgs::msg::PoseStamped place_pose_msg;
      place_pose_msg.header.frame_id = "world";
      place_pose_msg.pose.position.x = place_pose_values[0];
      place_pose_msg.pose.position.y = place_pose_values[1];
      place_pose_msg.pose.position.z = place_pose_values[2];
      tf2::Quaternion place_q;
      place_q.setRPY(place_pose_values[3], place_pose_values[4], place_pose_values[5]);
      place_pose_msg.pose.orientation = tf2::toMsg(place_q);

      // アプローチ用のターゲットBを計算
      geometry_msgs::msg::PoseStamped pick_approach_pose_msg = pick_pose_msg;
      pick_approach_pose_msg.pose.position.z += 0.04; // 200mm上方

      geometry_msgs::msg::PoseStamped place_approach_pose_msg = place_pose_msg;
      place_approach_pose_msg.pose.position.z += 0.02; // 200mm上方

      //pick_pose_msg.pose.position.z += -195;
      //place_pose_msg.pose.position.z += -195;
      //pick_pose_msg.pose.position.x += 0.044;
      //place_pose_msg.pose.position.x += 0.044;

      auto plan_and_execute_try_all = [&](const geometry_msgs::msg::PoseStamped& pose, bool cartesian, int indices1, int indices2, std::string plan_file, std::string pose_file) {
        move_group_interface.setPoseTarget(pose, "flange");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success;
        // プランナーのリストを定義
        std::vector<std::string> planners = {
          "RRTConnectkConfigDefault",
          "SBLkConfigDefault",
          "ESTkConfigDefault",
          "LBKPIECEkConfigDefault",
          "BKPIECEkConfigDefault",
          "KPIECEkConfigDefault",
          "RRTstarkConfigDefault",
          "RRTkConfigDefault",
          "TRRTkConfigDefault",
          "PRMkConfigDefault",
          "PRMstarkConfigDefault",
          "FMTkConfigDefault",
          "BFMTkConfigDefault",
          "PDSTkConfigDefault",
          "STRIDEkConfigDefault",
          "BiTRRTkConfigDefault",
          "LBTRRTkConfigDefault",
          "BiESTkConfigDefault",
          "ProjESTkConfigDefault",
          "LazyPRMkConfigDefault",
          "LazyPRMstarkConfigDefault",
          "SPARSkConfigDefault",
          "SPARStwokConfigDefault"
        };

        if (cartesian) {
          moveit_msgs::msg::RobotTrajectory trajectory;
          success = (move_group_interface.computeCartesianPath(
                      {pose.pose}, 0.01, 0.0, trajectory, true) >= 0.95);

          if (success) {
            plan.trajectory_ = trajectory;

          }
        } else {

            for (const auto& planner : planners) {
              move_group_interface.setPlannerId(planner);
              RCLCPP_INFO(node->get_logger(), "Trying planner: %s", planner.c_str());

              success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

              if (success) {
                  RCLCPP_INFO(node->get_logger(), "Planning succeeded with planner: %s", planner.c_str());
                  break; // 成功したらループを抜ける
              } else {
                  RCLCPP_WARN(node->get_logger(), "Planning failed with planner: %s", planner.c_str());
              }
            }

        }

        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Planning failed with all planners. id: %d - %d", indices1, indices2);
            return false;
        } else {
            bool exec_success = (move_group_interface.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            savePlanToYAML(plan, plan_file);
            saveTargetPoseToYAML(pose.pose, pose_file);
            RCLCPP_INFO(node->get_logger(), "plan and pose saved for id: %d - %d", indices1, indices2);
            return exec_success;
        }

      };

      RCLCPP_INFO(node->get_logger(), "before planning");
    
      if (index >= number){
        // 衝突回避用先端ゴムをアタッチ
        move_group_interface.attachObject("pump_rubber");

        // アプローチ用のターゲットBに移動（通常のプランニング）
        std::string plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(1) + ".yaml";
        std::string pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(1) + ".yaml";
        std::ifstream plan_file1(plan_path);
        std::ifstream pose_file1(pose_path);

        if (use_file && plan_file1 && pose_file1){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 1);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
            //plan_and_execute(pick_approach_pose_msg, false, index, 1, plan_path, pose_path);
            bool pe1_success = plan_and_execute_try_all(pick_approach_pose_msg, false, index, 1, plan_path, pose_path);
        }
        // 衝突回避用先端ゴムをデタッチ
        move_group_interface.detachObject("pump_rubber");
        planning_scene_interface.removeCollisionObjects({"pump_rubber"});

        // ターゲットAに移動（直線運動）
        plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(2) + ".yaml";
        pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(2) + ".yaml";
        std::ifstream plan_file2(plan_path);
        std::ifstream pose_file2(pose_path);
        if (use_file && plan_file2 && pose_file2){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 2);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
            //plan_and_execute(pick_pose_msg, true, index, 2, plan_path, pose_path);
            //bool ct_success = false;
            bool ct_success = plan_and_execute_try_all(pick_pose_msg, true, index, 2, plan_path, pose_path);
            if (!ct_success){

                geometry_msgs::msg::PoseStamped new_pick_app_pose = pick_approach_pose_msg;
    
                // オリエンテーションをZ軸に180度回転
                tf2::Quaternion q_app;
                tf2::fromMsg(new_pick_app_pose.pose.orientation, q_app);
                tf2::Quaternion q_rot_app;
                q_rot_app.setRPY(0, 0, M_PI); // Z軸に180度回転
                q_app = q_app * q_rot_app;
                new_pick_app_pose.pose.orientation = tf2::toMsg(q_app);
    
                // 元の座標系のY方向に0.088移動
                tf2::Vector3 translation_vector(0, 0.088, 0);
                tf2::Transform transform(q_app, tf2::Vector3(0, 0, 0));
                tf2::Vector3 rotated_translation = transform * translation_vector;
                new_pick_app_pose.pose.position.x += rotated_translation.x();
                new_pick_app_pose.pose.position.y += rotated_translation.y();
                new_pick_app_pose.pose.position.z += rotated_translation.z();

                ct_success = plan_and_execute_try_all(new_pick_app_pose, false, index, 1, plan_path, pose_path);
                if (ct_success) {
                    RCLCPP_INFO(node->get_logger(), "recovery succeeded for id: %d - %d", index, 1);
                    pick_approach_pose_msg = new_pick_app_pose;
                    geometry_msgs::msg::PoseStamped new_pick_pose = pick_pose_msg;

                    // オリエンテーションをZ軸に180度回転
                    tf2::Quaternion q;
                    tf2::fromMsg(new_pick_pose.pose.orientation, q);
                    tf2::Quaternion q_rot;
                    q_rot.setRPY(0, 0, M_PI); // Z軸に180度回転
                    q = q * q_rot;
                    new_pick_pose.pose.orientation = tf2::toMsg(q);

                    // 元の座標系のY方向に0.088移動
                    transform.setRotation(q);
                    rotated_translation = transform * translation_vector;
                    new_pick_pose.pose.position.x += rotated_translation.x();
                    new_pick_pose.pose.position.y += rotated_translation.y();
                    new_pick_pose.pose.position.z += rotated_translation.z();

                    moveit_msgs::msg::RobotTrajectory new_trajectory;
                    ct_success = plan_and_execute_try_all(new_pick_pose, true, index, 2, plan_path, pose_path); 
                    if (ct_success) {
                        RCLCPP_INFO(node->get_logger(), "recovery succeeded for id: %d - %d", index, 2);
                        pick_pose_msg = new_pick_pose;
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Planning failed after 180 degrees rotation.");
                        rclcpp::shutdown();  // ROS2ノードをシャットダウン
                        std::exit(EXIT_FAILURE);  // プログラムを終了

                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Planning failed after 180 degrees rotation.");
                    rclcpp::shutdown();  // ROS2ノードをシャットダウン
                    std::exit(EXIT_FAILURE);  // プログラムを終了
                }

            }
        }
        // つかむ
        set_io(node, tm_msgs::srv::SetIO::Request::MODULE_ENDEFFECTOR, tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT, 1, tm_msgs::srv::SetIO::Request::STATE_OFF);
        // 1秒待機
        if (!simulation_mode){
            rclcpp::sleep_for(1s);

            int tsumiki_on = ask_item(node, "demo", "End_DI0", 1);
            int attempt_count = 0;
            const int max_attempts = 20;

            if (tsumiki_on != 1) {
               geometry_msgs::msg::PoseStamped new_pick_pose_msg = pick_pose_msg;
               while (tsumiki_on != 1 && attempt_count < max_attempts) {
                    // pick_pose_msgをZ軸方向に-1mm（0.001m）下げる
                    new_pick_pose_msg.pose.position.z -= 0.001;

                    // ロボットを動かす
                    plan_and_execute_try_all(new_pick_pose_msg, true, index, 2, plan_path, pose_path);

                    // 吸着確認
                    tsumiki_on = ask_item(node, "demo", "End_DI0", 1);

                    // ログ出力
                    RCLCPP_INFO_STREAM(node->get_logger(), "Attempt " << (attempt_count + 1) << ": Lowering Z: " << new_pick_pose_msg.pose.position.z);
                    RCLCPP_INFO_STREAM(node->get_logger(), "Tsumiki on status: " << tsumiki_on);

                    // カウンタをインクリメント
                    attempt_count++;
                }
                // Place時の衝突を避けるために、元の位置に戻す
                //pick_pose_msg.pose.position.z += 0.001 * attempt_count;
                plan_and_execute_try_all(pick_pose_msg, true, index, 2, plan_path, pose_path);
                // 

           }
           if (tsumiki_on != 1) {
                RCLCPP_WARN_STREAM(node->get_logger(), "Failed to pick up item after " << max_attempts << " attempts.");
           } 
        }
        move_group_interface.attachObject(object_name);

        // 再びアプローチ用のターゲットBに戻る（直線運動）
        plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(3) + ".yaml";
        pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(3) + ".yaml";
        std::ifstream plan_file3(plan_path);
        std::ifstream pose_file3(pose_path);
        if (use_file && plan_file3 && pose_file3){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 3);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
            //plan_and_execute(pick_approach_pose_msg, true, index, 3, plan_path, pose_path);
            plan_and_execute_try_all(pick_approach_pose_msg, true, index, 3, plan_path, pose_path);
        }

        // アプローチ用のターゲットBに移動（通常のプランニング）
        plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(4) + ".yaml";
        pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(4) + ".yaml";
        std::ifstream plan_file4(plan_path);
        std::ifstream pose_file4(pose_path);
        if (use_file && plan_file4 && pose_file4){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 4);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
           //plan_and_execute(place_approach_pose_msg, false, index ,4, plan_path, pose_path);
           plan_and_execute_try_all(place_approach_pose_msg, false, index ,4, plan_path, pose_path);
        }
        /// Place時の直線移動をなくしてみる
        // ターゲットAに移動（直線運動）
        plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(5) + ".yaml";
        pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(5) + ".yaml";
        std::ifstream plan_file5(plan_path);
        std::ifstream pose_file5(pose_path);
        if (use_file && plan_file5 && pose_file5){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 5);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
            //plan_and_execute(place_pose_msg, true, index, 5, plan_path, pose_path);

            bool ct2_success = plan_and_execute_try_all(place_pose_msg, true, index, 5, plan_path, pose_path);
            if (!ct2_success){

                geometry_msgs::msg::PoseStamped new_place_app_pose = place_approach_pose_msg;

                // オリエンテーションをZ軸に180度回転
                tf2::Quaternion q_app;
                tf2::fromMsg(new_place_app_pose.pose.orientation, q_app);
                tf2::Quaternion q_rot_app;
                q_rot_app.setRPY(0, 0, M_PI); // Z軸に180度回転
                q_app = q_app * q_rot_app;
                new_place_app_pose.pose.orientation = tf2::toMsg(q_app);

                // 元の座標系のY方向に0.088移動
                tf2::Vector3 translation_vector(0, 0.088, 0);
                tf2::Transform transform(q_app, tf2::Vector3(0, 0, 0));
                tf2::Vector3 rotated_translation = transform * translation_vector;
                new_place_app_pose.pose.position.x += rotated_translation.x();
                new_place_app_pose.pose.position.y += rotated_translation.y();
                new_place_app_pose.pose.position.z += rotated_translation.z();

                ct2_success = plan_and_execute_try_all(new_place_app_pose, false, index, 4, plan_path, pose_path);
                if (ct2_success) {
                    RCLCPP_INFO(node->get_logger(), "recovery succeeded for id: %d - %d", index, 4);
                    place_approach_pose_msg = new_place_app_pose;
                    geometry_msgs::msg::PoseStamped new_place_pose = place_pose_msg;

                    // オリエンテーションをZ軸に180度回転
                    tf2::Quaternion q;
                    tf2::fromMsg(new_place_pose.pose.orientation, q);
                    tf2::Quaternion q_rot;
                    q_rot.setRPY(0, 0, M_PI); // Z軸に180度回転
                    q = q * q_rot;
                    new_place_pose.pose.orientation = tf2::toMsg(q);

                    // 元の座標系のY方向に0.088移動
                    transform.setRotation(q);
                    rotated_translation = transform * translation_vector;
                    new_place_pose.pose.position.x += rotated_translation.x();
                    new_place_pose.pose.position.y += rotated_translation.y();
                    new_place_pose.pose.position.z += rotated_translation.z();

                    moveit_msgs::msg::RobotTrajectory new_trajectory;
                    ct2_success = plan_and_execute_try_all(new_place_pose, true, index, 5, plan_path, pose_path);
                    if (ct2_success) {
                        RCLCPP_INFO(node->get_logger(), "recovery succeeded for id: %d - %d", index, 5);
                        place_pose_msg = new_place_pose;
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Planning failed after 180 degrees rotation.");
                        rclcpp::shutdown();  // ROS2ノードをシャットダウン
                        std::exit(EXIT_FAILURE);  // プログラムを終了

                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Planning failed after 180 degrees rotation.");
                    rclcpp::shutdown();  // ROS2ノードをシャットダウン
                    std::exit(EXIT_FAILURE);  // プログラムを終了

                }

            }

        }
      
        // 離す
        set_io(node, tm_msgs::srv::SetIO::Request::MODULE_ENDEFFECTOR, tm_msgs::srv::SetIO::Request::TYPE_DIGITAL_OUT, 1, tm_msgs::srv::SetIO::Request::STATE_ON);

        if (!simulation_mode) {
           rclcpp::sleep_for(1s);

        }
        move_group_interface.detachObject(object_name);
      
        // 再度アプローチ用のターゲットBに移動（直線運動）
        plan_path = plan_folder + "plan_" + std::to_string(index) + "_" + std::to_string(6) + ".yaml";
        pose_path = pose_folder + "target_pose_" + std::to_string(index) + "_" + std::to_string(6) + ".yaml";
        std::ifstream plan_file6(plan_path);
        std::ifstream pose_file6(pose_path);
        if (use_file && plan_file6 && pose_file6){
            RCLCPP_INFO(node->get_logger(), "execute from file %d - %d", index, 6);
            auto plan = loadPlanFromYAML(plan_path);
            move_group_interface.execute(plan);
        } else {
            //plan_and_execute(place_approach_pose_msg, true, index, 6, plan_path, pose_path);
            plan_and_execute_try_all(place_approach_pose_msg, true, index, 6, plan_path, pose_path);
        }
      
        // add pump rubber cylinder
        moveit_msgs::msg::CollisionObject pr2;
        pr2.id = "pump_rubber";
        pr2.header.frame_id = "flange";
        pr2.primitives.resize(1);
        pr2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        pr2.primitives[0].dimensions = { 0.020, 0.020 };

        geometry_msgs::msg::Pose pr2_pose;
        pr2_pose.position.x = 0;
        pr2_pose.position.y = -0.044;
        pr2_pose.position.z = 0.199 ;
        tf2::Quaternion pr2_q;
        pr2_q.setRPY(0, 0, 0);
        pr2_pose.orientation = tf2::toMsg(pr2_q);
        pr2.pose = pr2_pose;
        planning_scene_interface.applyCollisionObject(pr2);


      } else{

        std::map<std::string, moveit_msgs::msg::CollisionObject> collision_objects_map = planning_scene_interface.getObjects({object_name});
        moveit_msgs::msg::CollisionObject collision_object = collision_objects_map[object_name];
        // 現在のPoseを取得
        geometry_msgs::msg::Pose current_pose = place_pose_msg.pose;

        // tf2::Transformを使って現在のPoseを変換
        tf2::Transform current_transform;
        tf2::fromMsg(current_pose, current_transform);

        // 移動させたいオフセット（例えば、X方向に0.1、Y方向に0.2、Z方向に0.3）
        tf2::Vector3 translation_offset(0, -0.044, 0.199+0.009);

       // オフセットを現在の座標系に対して適用
        tf2::Transform offset_transform(tf2::Quaternion::getIdentity(), translation_offset);
        tf2::Transform new_transform = current_transform * offset_transform;

        // 新しいPoseを設定
        geometry_msgs::msg::Pose new_pose;
        new_pose.position.x = new_transform.getOrigin().x();
        new_pose.position.y = new_transform.getOrigin().y();
        new_pose.position.z = new_transform.getOrigin().z();
        new_pose.orientation.x = new_transform.getRotation().x();
        new_pose.orientation.y = new_transform.getRotation().y();
        new_pose.orientation.z = new_transform.getRotation().z();
        new_pose.orientation.w = new_transform.getRotation().w();
 
        // Collision ObjectのPoseを変更
        collision_object.pose = new_pose;

        // 変更したCollision Objectを更新する
        planning_scene_interface.applyCollisionObject(collision_object);

      }
    
      index++;
      /*

      */
    }
    //rclcpp::spin_some(node); // ノードをスピンしてコールバックを処理
  }

  if (simulation_mode && !failed_targets_indices.empty()) {
      std::ofstream failed_targets_file("/home/tak-mahal/IsaacSim-ros_workspaces/humble_ws/src/mtc_tutorial/src/failed_targets.csv");
      if (failed_targets_file.is_open()) {
          for (int failed_index : failed_targets_indices) {
              failed_targets_file << failed_index << std::endl;
          }
          failed_targets_file.close();
          RCLCPP_INFO(node->get_logger(), "Failed target indices written to file.");
      } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to open output CSV file.");
      }
  }


  /*
  */
  rclcpp::shutdown();
  return 0;
}
