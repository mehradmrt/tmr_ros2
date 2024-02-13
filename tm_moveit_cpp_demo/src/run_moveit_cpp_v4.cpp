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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , tfBuffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
    , tfListener_(*tfBuffer_)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
    target_leaves_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/target_leaves", 10, std::bind(&MoveItCppDemo::targetLeavesCallback, this, std::placeholders::_1));
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

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base";
    collision_object.id = "box";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 1.0, 0.8, 0.4 };

    tf2::Quaternion quat;
    quat.setRPY(0, 0, -M_PI / 4);  // Rotate -45 degrees around Z-axis

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.25;  
    box_pose.position.y = 0.25;
    box_pose.position.z = -0.2;
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
      RCLCPP_INFO(LOGGER, "Setting goal for point %zu", i);
      arm.setGoal(target_poses_[i], "gripper");

      RCLCPP_INFO(LOGGER, "Planning to goal for point %zu", i);
      auto plan_solution = arm.plan();

      if (plan_solution)
      {
        RCLCPP_INFO(LOGGER, "Plan found for point %zu", i);
        plan_outcomes[i] = 1;  
        outcome_log += "Point " + std::to_string(i) + ": Success\n";
        arm.execute();

        arm.setGoal(*robot_first_state);
        auto return_plan_solution = arm.plan();
        if (return_plan_solution)
        {
            RCLCPP_INFO(LOGGER, "Returning to initial state.");
            arm.execute();
        }
        else
        {
            RCLCPP_WARN(LOGGER, "Failed to plan back to initial state.");
        }
      }
      else
      {
        RCLCPP_INFO(LOGGER, "No plan found for point %zu", i);
        plan_outcomes[i] = 0;
        outcome_log += "Point " + std::to_string(i) + ": Failure\n";
      }
      
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
    RCLCPP_INFO(LOGGER, "%s", outcome_log.c_str());
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
  
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_leaves_subscriber_;
  std::vector<geometry_msgs::msg::PoseStamped> target_poses_;

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

  void targetLeavesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  { 
    target_poses_.clear();
    std::vector<geometry_msgs::msg::PoseStamped> target_poses;
    for (const auto& pose : msg->poses)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      pose_stamped.header.frame_id = "gripper";
      pose_stamped.header.stamp = node_->get_clock()->now();

      auto transformed_pose = transformPose(pose_stamped, "link_0");
      target_poses_.push_back(transformed_pose);
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
        demo.processNewData();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small sleep to prevent busy waiting
    }
  return 0;
}
 