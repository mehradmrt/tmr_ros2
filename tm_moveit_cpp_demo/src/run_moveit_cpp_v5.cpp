#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <rviz_visual_tools/rviz_visual_tools.hpp>
// #include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include "onrobot_rg_msgs/srv/set_command.hpp"
#include "custom_interfaces/srv/get_spectrum.hpp"
#include "custom_interfaces/msg/nano_spec.hpp"
#include "custom_interfaces/msg/leaf_pose_arrays.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <filesystem>
using namespace std::chrono_literals;
namespace fs = std::filesystem;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , tfBuffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
    , tfListener_(*tfBuffer_)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    , onrobot_rg_set_command_client_(node_->create_client<onrobot_rg_msgs::srv::SetCommand>("/onrobot_rg/set_command"))
    // , get_spectrum_client_(node_->create_client<custom_interfaces::srv::GetSpectrum>("/get_spectrum"))
    
  {

    leaf_pose_arrays_subscriber_ = node_->create_subscription<custom_interfaces::msg::LeafPoseArrays>(
        "/target_leaves_multi_pose", 10, std::bind(&MoveItCppDemo::leafPoseArraysCallback, this, std::placeholders::_1));

    spectral_data_subscriber_ = node_->create_subscription<custom_interfaces::msg::NanoSpec>(
        "/spectral_data", 10, [this](const custom_interfaces::msg::NanoSpec::SharedPtr msg) {
            spectralDataCallback(msg, "leaf_x", "pose_x"); 
        });

    spec_service_trigger_publisher_ = node_->create_publisher<std_msgs::msg::String>("spec_service_trigger", 10);
       
  }

  void run()
  { 

    // ----------------------------------------------------
    // -------- Initializing the planning pipeline --------
    // ----------------------------------------------------

    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitorNonConst()->setPlanningScenePublishingFrequency(100);
    
    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("tmr_arm", moveit_cpp_);
    
    // auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>("tmr_arm", moveit_cpp_);
    auto robot_first_state = moveit_cpp_->getCurrentState();
    // auto robot_model_ptr = moveit_cpp_->getRobotModel();
    // auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup("tmr_arm");
    // auto planning_scene_monitor = moveit_cpp_->getPlanningSceneMonitor();
    // planning_scene_monitor->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // moveit_cpp::PlanningComponent::PlanRequestParameters params;
    // params.max_velocity_scaling_factor = 1.0;
    // params.max_acceleration_scaling_factor = 1.0;


    // ------------------------------------------
    // -------- Adding collision objects --------
    // ------------------------------------------

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.80, 0.90 , 0.4 };

    tf2::Quaternion box_quat;
    // box_quat.setRPY(0, 0, 0); 
    box_quat.setRPY(0, 0, -M_PI / 4); //Rotate -45 degrees around Z-axis

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.25;  
    box_pose.position.y = 0.0; //0.25;
    box_pose.position.z = -0.15;
    box_pose.orientation = tf2::toMsg(box_quat);

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the wall behind the robot as a constraint to avoid collision
    moveit_msgs::msg::CollisionObject collision_wall;
    collision_wall.header.frame_id = "base";
    collision_wall.id = "wall";

    shape_msgs::msg::SolidPrimitive wall;
    wall.type = wall.BOX;
    wall.dimensions = { 0.01, 1.50, 1.0 };
    

    tf2::Quaternion wall_quat;
    // wall_quat.setRPY(0, 0, 0);
    wall_quat.setRPY(0, 0, -M_PI/4);
    
    geometry_msgs::msg::Pose wall_pose;
    wall_pose.position.x = -0.85;  
    wall_pose.position.y = 0.0;
    wall_pose.position.z = 0.50;
    wall_pose.orientation = tf2::toMsg(wall_quat);

    collision_wall.primitives.push_back(wall);
    collision_wall.primitive_poses.push_back(wall_pose);
    collision_wall.operation = collision_wall.ADD;

    {
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
      scene->processCollisionObjectMsg(collision_wall);
    }  

    std::string outcome_log = "Planning outcomes for all points:\n";
    for (size_t k = 0; k < all_target_poses_gripper_[0].size(); ++k)
    {
      moveit_msgs::msg::CollisionObject collision_leaf;
      collision_leaf.header.frame_id = "gripper";
      collision_leaf.id = "leaf_" + std::to_string(k+1);

      shape_msgs::msg::SolidPrimitive leaf_box;
      leaf_box.type = leaf_box.BOX;
      leaf_box.dimensions = { 0.01, 0.12, 0.07 };

      geometry_msgs::msg::Pose leaf_box_pose = all_target_poses_gripper_[0][k]; 

      collision_leaf.primitives.push_back(leaf_box);
      collision_leaf.primitive_poses.push_back(leaf_box_pose);
      collision_leaf.operation = collision_leaf.ADD;
      {
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(collision_leaf);
      }
    }
    
    // ------------------------------------------
    // ------------ Planning Begins -------------
    // ------------------------------------------

    performServiceCalls("reference","reference");
    for (size_t i = 0; i < all_target_poses_[0].size(); ++i)
    {
      bool planFound = false;
      for (size_t j = 0; j < all_target_poses_.size(); ++j)
      {
        const auto& pose = all_target_poses_[j][i];
        RCLCPP_INFO(LOGGER, "Setting goal for leaf %zu with pose %zu", i+1, j+1);
        arm.setGoal(pose, "gripper");

        RCLCPP_INFO(LOGGER, "Planning to goal for leaf %zu, with pose %zu", i+1, j+1);
        auto plan_solution = arm.plan();

        if(plan_solution) 
        { 
          outcome_log += "Leaf " + std::to_string(i+1) + ", Pose "+ std::to_string(j+1) +": SUCCESS\n";
          RCLCPP_INFO(LOGGER, "Plan found for leaf %zu", i+1);
          arm.execute();
          bool goalReached = false;
          while (rclcpp::ok())
          {
            if (PoseCompare("gripper", "link_0", pose, 0.001, 0.05)) 
            {
              RCLCPP_INFO(LOGGER, "Gripper reached the goal for leaf %zu, with pose %zu.", i+1, j+1);
              bool goalReached = true;

              performServiceCalls("Leaf_" + std::to_string(i+1), "Pose_" + std::to_string(j+1));
              break;
            }
            else 
            {
              RCLCPP_INFO(LOGGER, "Gripper has not reached the goal for leaf %zu, with pose %zu yet.", i+1, j+1);
              rclcpp::sleep_for(std::chrono::seconds(3));
            }
          }

          rclcpp::sleep_for(std::chrono::seconds(2));

          planFound = true;
          rclcpp::sleep_for(std::chrono::seconds(1));
          break;

        }
        else
        {
          RCLCPP_INFO(LOGGER, "No plan found for leaf %zu, with pose %zu", i+1, j+1);
          outcome_log += "Leaf " + std::to_string(i+1) + ", Pose "+ std::to_string(j+1) +": FAILED\n";
        }
      }
      RCLCPP_INFO(LOGGER, "%s", outcome_log.c_str());
    }

    arm.setGoal(*robot_first_state);
    auto return_plan_solution = arm.plan();
    if (return_plan_solution)
    {
      RCLCPP_INFO(LOGGER, "Returning to initial state.");
      arm.execute();

      RCLCPP_INFO(LOGGER, "Waiting for robot to reach first state...");
      bool FirstStateReached = false;

      while (rclcpp::ok())
      { 
        auto current_state = moveit_cpp_->getCurrentState();
        if (isRobotBack(current_state, robot_first_state)) 
        { 
          RCLCPP_INFO(LOGGER, "Robot reached its first state.");
          rclcpp::sleep_for(std::chrono::seconds(3));
          FirstStateReached = true;
          break;
        } 
        else 
        {
          RCLCPP_INFO(LOGGER, "Robot has NOT reached its first state.");
          rclcpp::sleep_for(std::chrono::seconds(3));
        }
      }
    }
  }





  bool PoseCompare(const std::string& target_frame, const std::string& reference_frame, const geometry_msgs::msg::PoseStamped& goal_pose, double position_tolerance, double orientation_tolerance) 
  {
    geometry_msgs::msg::PoseStamped current_pose;
    try {
        auto transformStamped = tfBuffer_->lookupTransform(reference_frame, target_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
        current_pose.header.stamp = node_->get_clock()->now();
        current_pose.header.frame_id = reference_frame;
        current_pose.pose.position.x = transformStamped.transform.translation.x;
        current_pose.pose.position.y = transformStamped.transform.translation.y;
        current_pose.pose.position.z = transformStamped.transform.translation.z;
        current_pose.pose.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not get current pose: %s", ex.what());
        return false;
    }

    double dx = current_pose.pose.position.x - goal_pose.pose.position.x;
    double dy = current_pose.pose.position.y - goal_pose.pose.position.y;
    double dz = current_pose.pose.position.z - goal_pose.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion current_orientation, goal_orientation;
    tf2::fromMsg(current_pose.pose.orientation, current_orientation);
    tf2::fromMsg(goal_pose.pose.orientation, goal_orientation);
    double angle_diff = current_orientation.angleShortestPath(goal_orientation);

    if (distance < position_tolerance && angle_diff < orientation_tolerance) {
        RCLCPP_INFO(node_->get_logger(), "Tolerance reached.");
        return true;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Tolerance not reached. Position: %f > %f, Orientation: %f > %f", distance, position_tolerance, angle_diff, orientation_tolerance);
      return false;
    }

  }

  bool isRobotHome(const moveit::core::RobotStatePtr& robot_state, const std::string& home_position_name) 
  {
      const std::string group_name = "tmr_arm";
      const auto* joint_model_group = robot_state->getJointModelGroup(group_name);
      double tolerance = 0.002;
      
      std::map<std::string, double> target_positions;
      joint_model_group->getVariableDefaultPositions(home_position_name, target_positions);

      std::vector<double> diffs;
      diffs.clear();
      for (const auto& joint_target : target_positions) {
          double current_position = *robot_state->getJointPositions(joint_target.first);
          diffs.push_back(current_position - joint_target.second);
      }

      double total_difference = std::sqrt(std::inner_product(diffs.begin(), diffs.end(), diffs.begin(), 0.0));

      if (total_difference < tolerance) {
          RCLCPP_INFO(node_->get_logger(), "Robot is at '%s' position within a tolerance of %f.", home_position_name.c_str(), tolerance);
          return true;
      } else {
          RCLCPP_INFO(node_->get_logger(), "Robot is not at '%s' position, tolerance exceeded: %f > %f", home_position_name.c_str(), total_difference, tolerance);
          return false;
      }
  }

  bool isRobotBack(const moveit::core::RobotStatePtr& current_state, 
                 const moveit::core::RobotStatePtr& initial_state, 
                 double tolerance = 0.002) 
  {
    const moveit::core::RobotModelConstPtr& robot_model = current_state->getRobotModel();
    const std::vector<std::string>& joint_group_names = robot_model->getJointModelGroupNames();

    for (const auto& group_name : joint_group_names) {
        const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);
        std::vector<double> current_positions, initial_positions;
        current_state->copyJointGroupPositions(joint_model_group, current_positions);
        initial_state->copyJointGroupPositions(joint_model_group, initial_positions);

        for (size_t i = 0; i < current_positions.size(); ++i) {
            if (std::abs(current_positions[i] - initial_positions[i]) > tolerance) {
                return false;
            }
        }
    }
    return true;
  }

  void performServiceCalls(const std::string& leaf_x, const std::string& pose_x) 
  {
    if (!onrobot_rg_set_command_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(LOGGER, "/onrobot_rg/set_command service not available.");
        return;
    }
 
    auto request_rg = std::make_shared<onrobot_rg_msgs::srv::SetCommand::Request>();
    request_rg->command = 'c';
    auto future_rg = onrobot_rg_set_command_client_->async_send_request(request_rg);
    RCLCPP_INFO(LOGGER, "GRIPPER CLOESE");
    rclcpp::sleep_for(std::chrono::seconds(3)); 
    
    spectrum_var_ = false;
    NanoSpecTrigger(leaf_x, pose_x);
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < 5s) {
      rclcpp::spin_some(node_);
      if (spectrum_var_) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!spectrum_var_) {
      RCLCPP_WARN(LOGGER, "No spectral data received within 3 seconds.");
    }
    RCLCPP_INFO(LOGGER, "SPECTRAL DATA PROCESSED");
    rclcpp::sleep_for(std::chrono::seconds(5));

    request_rg->command = 'o';
    auto future_rg_second = onrobot_rg_set_command_client_->async_send_request(request_rg);
    RCLCPP_INFO(LOGGER, "GRIPPER OPENED");
    rclcpp::sleep_for(std::chrono::seconds(1)); 
  }

  void NanoSpecTrigger(const std::string& leaf_x, const std::string& pose_x)
  {
    spectral_data_subscriber_ = node_->create_subscription<custom_interfaces::msg::NanoSpec>(
    "/spectral_data", 10, [this, leaf_x, pose_x](const custom_interfaces::msg::NanoSpec::SharedPtr msg) {
        spectralDataCallback(msg, leaf_x, pose_x);
    });

    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = "trigger";
    spec_service_trigger_publisher_->publish(*message);
    RCLCPP_INFO(LOGGER, "Trigger sent to spec_service_trigger topic.");
    
  }

  void spectralDataCallback(const custom_interfaces::msg::NanoSpec::SharedPtr msg, const std::string& leaf_x, const std::string& pose_x)
  {
    std::thread([this, msg, leaf_x, pose_x]() {
      RCLCPP_INFO(LOGGER, "Processing spectral data in a separate thread.");

      std::string wavelengths_str = "Wavelengths: ";
      for (const auto& wavelength : msg->wavelengths)
      {
          wavelengths_str += std::to_string(wavelength) + " ";
      }
      RCLCPP_INFO(LOGGER, wavelengths_str.c_str());

      std::string spectrum_str = "Spectrum: ";
      for (const auto& spectrum_value : msg->spectrum)
      {
          spectrum_str += std::to_string(spectrum_value) + " ";
      }
      RCLCPP_INFO(LOGGER, spectrum_str.c_str());

      save_to_json(leaf_x, pose_x, msg->wavelengths, msg->spectrum);
      spectrum_var_ = true;
    }).detach();
  }

  void save_to_json( const std::string& leaf_x, const std::string& pose_x, const std::vector<uint16_t>& wavelengths, const std::vector<double>& spectrum) {
    nlohmann::json j;
    j["leaf_number"] = leaf_x;
    j["pose_number"] = pose_x;
    j["wavelengths"] = wavelengths;
    j["spectrum"] = spectrum;

    auto today = std::chrono::system_clock::now();
    std::time_t today_time = std::chrono::system_clock::to_time_t(today);
    std::stringstream date_stream;
    date_stream << std::put_time(std::localtime(&today_time), "%m-%d-%Y");
    std::string date_str = date_stream.str();

    std::string current_path = fs::current_path().string();
    RCLCPP_INFO(LOGGER, "Current working directory: %s", current_path.c_str());

    std::string base_path = current_path + "/runs/results/" + date_str;
    if (!fs::exists(base_path)) {
        RCLCPP_ERROR(LOGGER, "Directory does not exist: %s", base_path.c_str());
        return;
    }

    int latest_results_number = -1;
    std::string latest_results_folder;
    for (const auto& entry : fs::directory_iterator(base_path)) {
        if (entry.is_directory() && entry.path().filename().string().find("results") == 0) {
            int results_number = std::stoi(entry.path().filename().string().substr(7));
            if (results_number > latest_results_number) {
                latest_results_number = results_number;
                latest_results_folder = entry.path().string();
            }
        }
    }

    if (latest_results_folder.empty()) {
        RCLCPP_ERROR(LOGGER, "No valid results folder found in: %s", base_path.c_str());
        return;
    }

    
    std::string file_name = leaf_x + ".json";
    std::string file_path = latest_results_folder + "/" + file_name;
    std::ofstream file(file_path);
    if (file.is_open()) {
      file << j.dump(4) << std::endl;
      file.close();
      RCLCPP_INFO(LOGGER, "Saved JSON to: %s", file_path.c_str());
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to open file: %s", file_path.c_str());
    }
  }
  
  
  void processNewData()
  {
      if (packages_received_)
      {
          run();
          packages_received_ = false;
      }
      else
      {
      RCLCPP_INFO(LOGGER, "waiting for the targets to be available.....");
      rclcpp::sleep_for(std::chrono::seconds(1));
      }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  
  rclcpp::Subscription<custom_interfaces::msg::LeafPoseArrays>::SharedPtr leaf_pose_arrays_subscriber_;
  rclcpp::Subscription<custom_interfaces::msg::NanoSpec>::SharedPtr spectral_data_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spec_service_trigger_publisher_;

  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> all_target_poses_;
  std::vector<std::vector<geometry_msgs::msg::Pose>> all_target_poses_gripper_;

  rclcpp::Client<onrobot_rg_msgs::srv::SetCommand>::SharedPtr onrobot_rg_set_command_client_;
  // rclcpp::Client<custom_interfaces::srv::GetSpectrum>::SharedPtr get_spectrum_client_;

  bool packages_received_ = false;
  bool spectrum_var_ = false;

  geometry_msgs::msg::PoseStamped transformPose(const geometry_msgs::msg::PoseStamped& input_pose, const std::string& target_frame)
  {
    geometry_msgs::msg::PoseStamped output_pose;

    try
    {
      output_pose = tfBuffer_->transform(input_pose, target_frame, tf2::durationFromSec(0.1));
      RCLCPP_INFO(node_->get_logger(), "Transformed Pose in frame [%s]: Position - x: [%f], y: [%f], z: [%f]; Orientation - x: [%f], y: [%f], z: [%f], w: [%f]", 
            output_pose.header.frame_id.c_str(),
            output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z,
            output_pose.pose.orientation.x, output_pose.pose.orientation.y, output_pose.pose.orientation.z, output_pose.pose.orientation.w);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "TF2 exception: %s", ex.what());
    }

    return output_pose;
  }

  void leafPoseArraysCallback(const custom_interfaces::msg::LeafPoseArrays::SharedPtr msg) 
  {
    all_target_poses_.clear();
    all_target_poses_gripper_.clear();

    std::vector<std::vector<geometry_msgs::msg::Pose>> all_poses = {
      msg->poses1, msg->poses2, msg->poses3, msg->poses4, msg->poses5};

    all_target_poses_gripper_ = all_poses;

    for (const auto& pose_array : all_poses) 
    {
      std::vector<geometry_msgs::msg::PoseStamped> transformed_poses_array;
      for (const auto& pose : pose_array) 
      {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header = msg->header;
        auto transformed_pose = transformPose(pose_stamped, "link_0");
        transformed_poses_array.push_back(transformed_pose);
      }
      all_target_poses_.push_back(transformed_poses_array);
    }
    packages_received_ = true;
  }
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
  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(node);
    // std::this_thread::sleep_for(std::chrono::seconds(8)); 
    demo.processNewData();
    std::this_thread::sleep_for(std::chrono::seconds(3)); 
  }
  return 0;
}
