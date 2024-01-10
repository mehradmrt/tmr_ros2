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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node),
    robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1)),
    tfBuffer_(std::make_shared<rclcpp::Clock>()),
    tfListener_(tfBuffer_)
  {
    this->pose_array_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/target_leaves", 10, std::bind(&MoveItCppDemo::poseArrayCallback, this, std::placeholders::_1));
  }
  
  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("tmr_arm", moveit_cpp_);

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
 
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // checkPlannability(arm);
    // if (!base_pose_array_.poses.empty()) 
    // {
    approachable_targets_.poses.clear(); 
    std::string result = std::to_string(base_pose_array_.poses.size()) + " targets: ";
    for (const auto& pose : base_pose_array_.poses) {
        geometry_msgs::msg::PoseStamped target;
        target.pose = pose;
        arm.setGoal(target, "link_6"); 

        auto plan_solution = arm.plan();
        if (plan_solution) {
            result += "Yes, ";
            approachable_targets_.poses.push_back(pose); 
        } else {
            result += "No, ";
        }
    }

    result.pop_back(); 
    result.pop_back();
    RCLCPP_INFO(LOGGER, "Plannability %s", result.c_str());

    rclcpp::sleep_for(std::chrono::seconds(5));


    for (const auto& app_pose : approachable_targets_.poses) 
    {
      geometry_msgs::msg::PoseStamped app_target;
      app_target.pose = app_pose;
      arm.setGoal(app_target, "link_6"); 

      auto plan_solution = arm.plan();

      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
      rclcpp::sleep_for(std::chrono::seconds(10));
    }
    // geometry_msgs::msg::PoseStamped target;
    // target.pose.position.x = 0;
    // target.pose.position.y = 0;
    // target.pose.position.z =  0.1;  
 
    // // Set the pose goal
    // arm.setGoal(target, "link_6"); 

    // // Run actual plan
    // RCLCPP_INFO(LOGGER, "Plan to goal");
    // auto plan_solution = arm.plan();
    // if (plan_solution)
    // {
    //   RCLCPP_INFO(LOGGER, "arm.execute()");
    //   // arm.execute();
    // }
  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
  {
    base_pose_array_.poses.clear();

    for (const auto& tool_pose : msg->poses) 
    {
      try {
        geometry_msgs::msg::PoseStamped input_pose;
        input_pose.pose = tool_pose;
        input_pose.header.frame_id = "link_6";
        input_pose.header.stamp = node_->get_clock()->now();

        geometry_msgs::msg::PoseStamped new_pose_;
        if (tfBuffer_.canTransform("link_0", "link_6", tf2::TimePointZero)) 
        {
          new_pose_ = tfBuffer_.transform(input_pose, "flange", tf2::durationFromSec((0.1)));
          base_pose_array_.poses.push_back(new_pose_.pose);
          RCLCPP_INFO(LOGGER, "TF2 transformation with Position: [%.2f, %.2f, %.2f], Orientation: [%.2f, %.2f, %.2f, %.2f]", 
            new_pose_.pose.position.x, 
            new_pose_.pose.position.y, 
            new_pose_.pose.position.z, 
            new_pose_.pose.orientation.x, 
            new_pose_.pose.orientation.y, 
            new_pose_.pose.orientation.z, 
            new_pose_.pose.orientation.w);
        }
      } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(node_->get_logger(), "TF2 exception: %s", ex.what());
      }
    }
    run();
  }

  void checkPlannability(moveit::planning_interface::PlanningComponent& arm) {
      approachable_targets_.poses.clear(); 
      std::string result = std::to_string(base_pose_array_.poses.size()) + " targets: ";
      for (const auto& pose : base_pose_array_.poses) {
          geometry_msgs::msg::PoseStamped target;
          target.pose = pose;
          arm.setGoal(target, "link_6"); 

          auto plan_solution = arm.plan();
          if (plan_solution) {
              result += "Yes, ";
              approachable_targets_.poses.push_back(pose); 
          } else {
              result += "No, ";
          }
      }
      
      if (!base_pose_array_.poses.empty()) {
          result.pop_back(); 
          result.pop_back();
      }

      RCLCPP_INFO(LOGGER, "%s", result.c_str());
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_subscriber_;
  geometry_msgs::msg::PoseArray base_pose_array_;
  geometry_msgs::msg::PoseArray approachable_targets_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
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
  // std::thread run_demo([&demo]() {
  //   // Let RViz initialize before running demo
  //   // TODO (henningkayser): use lifecycle events to launch node
  //   rclcpp::sleep_for(std::chrono::seconds(5));
  //   demo.run();
    
  // });

  rclcpp::spin(node);
  // run_demo.join();

  return 0;
}
 

 