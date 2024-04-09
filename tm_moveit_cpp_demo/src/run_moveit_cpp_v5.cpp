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

// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <rviz_visual_tools/rviz_visual_tools.hpp>

// #include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include "onrobot_rg_msgs/srv/set_command.hpp"
#include "custom_interfaces/srv/get_spectrum.hpp"
#include "custom_interfaces/msg/leaf_pose_arrays.hpp"



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
    , get_spectrum_client_(node_->create_client<custom_interfaces::srv::GetSpectrum>("/get_spectrum"))
  {
    // target_leaves_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    //         "/target_leaves", 10, std::bind(&MoveItCppDemo::targetLeavesCallback, this, std::placeholders::_1));

    leaf_pose_arrays_subscriber_ = node_->create_subscription<custom_interfaces::msg::LeafPoseArrays>(
        "/target_leaves_multi_pose", 10, std::bind(&MoveItCppDemo::leafPoseArraysCallback, this, std::placeholders::_1));
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

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


    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools(node_, "link_0", "moveit_cpp_",
    //                                                 moveit_cpp_->getPlanningSceneMonitor());
    // visual_tools.deleteAllMarkers();
    // visual_tools.loadRemoteControl();

    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();



    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 1.0, 0.8, 0.4 };

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // quat.setRPY(0, 0, -M_PI / 4); Rotate -45 degrees around Z-axis

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.35;  
    box_pose.position.y = 0.0; //0.25;
    box_pose.position.z = -0.15;
    box_pose.orientation = tf2::toMsg(quat);

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    std::vector<int> plan_outcomes(target_poses_.size(), 9);  
    std::string outcome_log = "Planning outcomes for all points:\n";
    for (size_t i = 0; i < target_poses_.size(); ++i)
    {
      moveit_msgs::msg::CollisionObject collision_leaf;
      collision_leaf.header.frame_id = "link_0";
      collision_leaf.id = "leaf_" + std::to_string(i);

      shape_msgs::msg::SolidPrimitive leaf_box;
      leaf_box.type = leaf_box.BOX;
      leaf_box.dimensions = { 0.01, 0.1, 0.05 };

      geometry_msgs::msg::Pose leaf_box_pose = target_poses_[i].pose; 

      collision_leaf.primitives.push_back(leaf_box);
      collision_leaf.primitive_poses.push_back(leaf_box_pose);
      collision_leaf.operation = collision_leaf.ADD;
      {  
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(collision_leaf);
      }   
    }

    // arm.setGoal("home2");
    // RCLCPP_INFO(LOGGER, "Plan to home2");
    // auto home_solution = arm.plan();
    // if (home_solution)
    // {
    //   RCLCPP_INFO(LOGGER, "Executing movement to 'home2'");
    //   arm.execute();

    //   RCLCPP_INFO(LOGGER, "Waiting for robot to reach 'home2'...");
    //   bool HomeReached = false;

    //   while (rclcpp::ok())
    //   { 
    //     auto robot_state = moveit_cpp_->getCurrentState();
    //     if (isRobotHome(robot_state, "home2")) 
    //     { 
    //       RCLCPP_INFO(LOGGER, "Robot has reached 'home2' position.");
    //       rclcpp::sleep_for(std::chrono::seconds(3));
    //       HomeReached = true;
    //       break;
    //     } 
    //     else 
    //     {
    //       rclcpp::sleep_for(std::chrono::seconds(3));
    //     }
    //   }
    // }


    performServiceCalls();
    for (size_t i = 0; i < target_poses_.size(); ++i)
    {
      bool planFound = false;
      std::vector<std::vector<geometry_msgs::msg::PoseStamped>> allPoses = { {target_poses_[i]} };
      if (i < alternative_target_poses_.size()) 
      {
          for (auto& alternative_poses : alternative_target_poses_[i]) {
              allPoses.push_back({alternative_poses}); 
          }
      }

      for (auto& poses : allPoses) 
        {
          for (auto& pose : poses) 
          {
            RCLCPP_INFO(LOGGER, "Setting goal for point %zu", i);
            arm.setGoal(pose, "gripper");
            
            RCLCPP_INFO(LOGGER, "Planning to goal for point %zu", i);
            auto plan_solution = arm.plan();

              if(plan_solution) 
              {
                  RCLCPP_INFO(LOGGER, "Plan found for point %zu", i);
                  arm.execute();
                  while (rclcpp::ok())
                  {
                    if (PoseCompare("gripper", "link_0", pose, 0.001, 0.05)) 
                    { 
                      RCLCPP_INFO(LOGGER, "Gripper reached the goal for point %zu.", i);
                      goalReached = true;

                      rclcpp::sleep_for(std::chrono::seconds(6));
                      performServiceCalls();
                      break;
                    } 
                    else 
                    {
                      RCLCPP_INFO(LOGGER, "Gripper has not reached the goal for point %zu yet.", i);
                      rclcpp::sleep_for(std::chrono::seconds(3));
                    }
                    
                  }
                  planFound = true;
                  break; 
              }
          }
          if (planFound) break;
        }

        
        

        rclcpp::sleep_for(std::chrono::seconds(3));

        // arm.setGoal("home2");
        // RCLCPP_INFO(LOGGER, "Plan to home2");
        // auto home_solution = arm.plan();
        // if (home_solution)
        // {
        //   RCLCPP_INFO(LOGGER, "Executing movement to 'home2'");
        //   arm.execute();

        //   RCLCPP_INFO(LOGGER, "Waiting for robot to reach 'home2'...");
        //   bool HomeReached = false;

        //   while (rclcpp::ok())
        //   { 
        //     auto robot_state = moveit_cpp_->getCurrentState();
        //     if (isRobotHome(robot_state, "home2")) 
        //     { 
        //       RCLCPP_INFO(LOGGER, "Robot has reached 'home2' position.");
        //       rclcpp::sleep_for(std::chrono::seconds(3));
        //       HomeReached = true;
        //       break;
        //     } 
        //     else 
        //     {
        //       rclcpp::sleep_for(std::chrono::seconds(3));
        //     }
        //   }
        // }

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
      else
      {
        RCLCPP_INFO(LOGGER, "No plan found for point %zu", i);
        plan_outcomes[i] = 0;
        outcome_log += "Point " + std::to_string(i) + ": Failure\n";
      }
      
      rclcpp::sleep_for(std::chrono::seconds(3));
    }
    RCLCPP_INFO(LOGGER, "%s", outcome_log.c_str());
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

  bool isRobotHome(const moveit::core::RobotStatePtr& robot_state, const std::string& home_position_name) {
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
                 double tolerance = 0.002) {
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

  void performServiceCalls() {
    if (!onrobot_rg_set_command_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "/onrobot_rg/set_command service not available.");
        return;
    }
    if (!get_spectrum_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "/get_spectrum service not available.");
        return;
    }
    
    auto request_rg = std::make_shared<onrobot_rg_msgs::srv::SetCommand::Request>();
    request_rg->command = 'c';
    auto future_rg = onrobot_rg_set_command_client_->async_send_request(request_rg);
    rclcpp::sleep_for(std::chrono::seconds(3)); 

    auto request_spectrum = std::make_shared<custom_interfaces::srv::GetSpectrum::Request>();
    auto future_spectrum = get_spectrum_client_->async_send_request(request_spectrum);
    rclcpp::sleep_for(std::chrono::seconds(3)); 

    request_rg->command = 'o';
    auto future_rg_second = onrobot_rg_set_command_client_->async_send_request(request_rg);
    // rclcpp::sleep_for(std::chrono::seconds(3)); 
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
  
  // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_leaves_subscriber_;
  rclcpp::Subscription<custom_interfaces::msg::LeafPoseArrays>::SharedPtr leaf_pose_arrays_subscriber_;


  std::vector<geometry_msgs::msg::PoseStamped> target_poses_;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> alternative_target_poses_; 

  rclcpp::Client<onrobot_rg_msgs::srv::SetCommand>::SharedPtr onrobot_rg_set_command_client_;
  rclcpp::Client<custom_interfaces::srv::GetSpectrum>::SharedPtr get_spectrum_client_;

  bool packages_received_ = false;

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

  // void targetLeavesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  // { 
  //   target_poses_.clear();
  //   std::vector<geometry_msgs::msg::PoseStamped> target_poses;
  //   for (const auto& pose : msg->poses)
  //   {
  //     geometry_msgs::msg::PoseStamped pose_stamped;
  //     pose_stamped.pose = pose;
  //     pose_stamped.header.frame_id = "gripper";
  //     // pose_stamped.header.frame_id = "camera_depth_optical_frame";
  //     pose_stamped.header.stamp = node_->get_clock()->now();

  //     auto transformed_pose = transformPose(pose_stamped, "link_0");
  //     target_poses_.push_back(transformed_pose);
  //   }

  //   packages_received_ = true;
  // }

  void leafPoseArraysCallback(const custom_interfaces::msg::LeafPoseArrays::SharedPtr msg)
  {
    target_poses_.clear();
    alternative_target_poses_.clear();

    for (const auto& pose : msg->Poses1.poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header = msg->header; 
        auto transformed_pose = transformPose(pose_stamped, "link_0"); 
        target_poses_.push_back(transformed_pose);
    }

    std::vector<geometry_msgs::msg::PoseArray> alternative_poses = {msg->Poses2, msg->Poses3, msg->Poses4, msg->Poses5};
    
    for (auto& pose_array : alternative_poses) {
        std::vector<geometry_msgs::msg::PoseStamped> current_alternatives;
        for (const auto& pose : pose_array.poses) {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          pose_stamped.header = msg->header; 
          auto transformed_pose = transformPose(pose_stamped, "link_0"); 
          current_alternatives.push_back(transformed_pose);
        }
        alternative_target_poses_.push_back(current_alternatives);
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
  while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        // std::this_thread::sleep_for(std::chrono::seconds(8)); 
        demo.processNewData();
        std::this_thread::sleep_for(std::chrono::seconds(5)); 
    }
  return 0;
}
 