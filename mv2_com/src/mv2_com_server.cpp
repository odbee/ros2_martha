#include "rclcpp/rclcpp.hpp"
#include "mv2_com_interfaces/srv/move_pose.hpp"

#include <memory>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_action");

class RobotInstance {
  public:
    RobotInstance(){
      planning_components=nullptr;
      moveit_cpp_ptr=nullptr;
      joint_model_group_ptr=nullptr;

    };
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components;
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr;
    const moveit::core::JointModelGroup* joint_model_group_ptr;

};


RobotInstance panda;

void add(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
          std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response)
{
    response->success=true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: SUCCESS");

  auto start_state = *(panda.moveit_cpp_ptr->getCurrentState());
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.w = (float) rand()/RAND_MAX;
  start_pose.position.x = 0.55;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.6;
   
  start_state.setFromIK(panda.joint_model_group_ptr, start_pose);

  panda.planning_components->setStartState(start_state);
}

void plan(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
          std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response)
{
    response->success=true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: SUCCESS");

  panda.planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "panda_link0";
  target_pose1.pose.orientation.w = (float) rand()/RAND_MAX;
  target_pose1.pose.position.x = (float) rand()/RAND_MAX;
  target_pose1.pose.position.y = -0.2;
  target_pose1.pose.position.z = (float) rand()/RAND_MAX;
  panda.planning_components->setGoal(target_pose1, "panda_link8");

  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = panda.planning_components->plan();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");
  node_options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("movepose_server", "", node_options);

  rclcpp::Service<mv2_com_interfaces::srv::MovePose>::SharedPtr service =
    node->create_service<mv2_com_interfaces::srv::MovePose>("movepose", &plan);


  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string LOGNAME = "moveit_action";

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
  panda.moveit_cpp_ptr= moveit_cpp_ptr;
  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  panda.planning_components= planning_components;

  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
  panda.joint_model_group_ptr= joint_model_group_ptr;


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "moveit_action",
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



  RCLCPP_INFO(LOGGER, "Ready to MOVE .");

  rclcpp::spin(node);
  rclcpp::shutdown();
}