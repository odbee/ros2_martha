#include "rclcpp/rclcpp.hpp"
#include "mv2_com_interfaces/srv/move_pose.hpp"

#include <memory>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/srv/trigger.hpp"

namespace rvt = rviz_visual_tools;




static const rclcpp::Logger LOGGER = rclcpp::get_logger("babybaby");


class RobotServices : public rclcpp::Node {
public:
  RobotServices() : Node("movepose_serviceholder") { 
    auto planning_group_desc = rcl_interfaces::msg::ParameterDescriptor{};
    planning_group_desc.description = "Planning Group for move_group execution";

    this->declare_parameter("planning_group", "PLEASE_DECLARE_PLANNING_GROUP_IN_LAUNCH_FILE", planning_group_desc);
    
    PLANNING_GROUP=this->get_parameter("planning_group").as_string();

    auto base_frame_desc = rcl_interfaces::msg::ParameterDescriptor{};
    base_frame_desc.description = "Planning Group for move_group execution";

    this->declare_parameter("base_frame", "PLEASE_DECLARE_BASE_FRAME_IN_LAUNCH_FILE", base_frame_desc);
    
    BASE_FRAME=this->get_parameter("base_frame").as_string();



    planning_scene_interface=nullptr;
    move_group=nullptr;
    joint_model_group_ptr=nullptr; 

  }
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  const moveit::core::JointModelGroup* joint_model_group_ptr;
  std::string  PLANNING_GROUP;
  std::string  BASE_FRAME;
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;

};

void plan_move_robot(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
          std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response,std::shared_ptr<RobotServices> RobotSer)
  {

    // const moveit::core::JointModelGroup* joint_model_group =
    //   RobotSer->move_group->getCurrentState()->getJointModelGroup(RobotSer->PLANNING_GROUP);

    RobotSer->move_group->setPoseTarget(request->pose);

    RCLCPP_INFO(LOGGER, "received pose: pos:(%.2f,%.2f,%.2f), orient:(%.2f,%.2f,%.2f,%.2f) ",
      request->pose.position.x,request->pose.position.y,request->pose.position.z,
      request->pose.orientation.x,request->pose.orientation.y,request->pose.orientation.z,request->pose.orientation.w);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (RobotSer->move_group->plan(RobotSer->move_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool movesuccess = (RobotSer->move_group->execute(RobotSer->move_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    response->success=success;
  }


void plan_robot(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
          std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response,std::shared_ptr<RobotServices> RobotSer)
  {

    // const moveit::core::JointModelGroup* joint_model_group =
    //   RobotSer->move_group->getCurrentState()->getJointModelGroup(RobotSer->PLANNING_GROUP);

    RobotSer->move_group->setPoseTarget(request->pose);

    RCLCPP_INFO(LOGGER, "received pose: pos:(%.2f,%.2f,%.2f), orient:(%.2f,%.2f,%.2f,%.2f) ",
      request->pose.position.x,request->pose.position.y,request->pose.position.z,
      request->pose.orientation.x,request->pose.orientation.y,request->pose.orientation.z,request->pose.orientation.w);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (RobotSer->move_group->plan(RobotSer->move_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    response->success=success;
  }

void move_robot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response,std::shared_ptr<RobotServices> RobotSer)
  {

    // const moveit::core::JointModelGroup* joint_model_group =
    //   RobotSer->move_group->getCurrentState()->getJointModelGroup(RobotSer->PLANNING_GROUP);

    // RobotSer->move_group->move();


    // bool success = (RobotSer->move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);

    bool success = (RobotSer->move_group->execute(RobotSer->move_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    response->success=success;
  }


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> move_group_node = rclcpp::Node::make_shared("movepose_executor", "", node_options);
  std::shared_ptr<RobotServices> RS =      std::make_shared<RobotServices>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  rclcpp::Service<mv2_com_interfaces::srv::MovePose>::SharedPtr plan_move_service_ =
     RS->create_service<mv2_com_interfaces::srv::MovePose>("plan_move_robot",
     std::bind(&plan_move_robot, std::placeholders::_1, std::placeholders::_2, RS));
  rclcpp::Service<mv2_com_interfaces::srv::MovePose>::SharedPtr plan_service_ =
     RS->create_service<mv2_com_interfaces::srv::MovePose>("plan_robot",
     std::bind(&plan_robot, std::placeholders::_1, std::placeholders::_2, RS));
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_service_ =
    RS->create_service<std_srvs::srv::Trigger>("move_robot",
    std::bind(&move_robot, std::placeholders::_1, std::placeholders::_2, RS));

  executor.add_node(RS);


  // std::thread([&executor]() { executor.spin(); }).detach();


  RCLCPP_INFO(LOGGER, "done spinning");

  RS->move_group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, RS->PLANNING_GROUP);
  executor.spin();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const moveit::core::JointModelGroup* joint_model_group =
    //   RS->move_group->getCurrentState()->getJointModelGroup(RS->PLANNING_GROUP);



  namespace rvt = rviz_visual_tools;



  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
                                                     RS->move_group->getRobotModel());





  // rclcpp::spin(move_group_node);



  RCLCPP_INFO(LOGGER, "Ready to MOVE .");
  while(rclcpp::ok()){

  }
  rclcpp::shutdown();
}