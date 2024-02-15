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


namespace rvt = rviz_visual_tools;




static const rclcpp::Logger LOGGER = rclcpp::get_logger("babybaby");
std::string PlanningGroup="ur_manipulator";


class RobotServices : public rclcpp::Node {
public:
  RobotServices(float sleep_timer) : Node("movepose_serviceholder") {    
    PLANNING_GROUP="";
    planning_scene_interface=nullptr;
    move_group=nullptr;
    joint_model_group_ptr=nullptr;
    this->wait_time = sleep_timer;
    // service_ = 
    //     this->create_service<mv2_com_interfaces::srv::MovePose>("add_two_ints",std::bind(&RobotServices::add, this, std::placeholders::_1, std::placeholders::_2));

  }
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  const moveit::core::JointModelGroup* joint_model_group_ptr;
  std::string  PLANNING_GROUP;

private:
  // void add(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
  //         std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response)
  // {
  //   request==request;
  //   //   // Raw pointers are frequently used to refer to the planning group for improved performance.
  //   // const moveit::core::JointModelGroup* joint_model_group =
  //   //   move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //   // geometry_msgs::msg::Pose target_pose1;
  //   // target_pose1.orientation.w = (float) rand()/RAND_MAX;
  //   // target_pose1.position.x = 0.55;
  //   // target_pose1.position.y = 0.0;
  //   // target_pose1.position.z = 0.6;
  //   // move_group->setPoseTarget(target_pose1);

  //   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //   // bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  //   // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  //   response->success=true;
  // }
  // rclcpp::Service<mv2_com_interfaces::srv::MovePose>::SharedPtr service_;
  float wait_time;
};


// class RobotInstance {
//   public:
//     RobotInstance(){
//       PLANNING_GROUP="";
//       planning_scene_interface=nullptr;
//       move_group=nullptr;
//       joint_model_group_ptr=nullptr;


//     };
//     std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
//     const moveit::core::JointModelGroup* joint_model_group_ptr;
//     std::string  PLANNING_GROUP;

// };

// RobotServices RS;


void move_robot(const std::shared_ptr<mv2_com_interfaces::srv::MovePose::Request> request,
          std::shared_ptr<mv2_com_interfaces::srv::MovePose::Response>      response,std::shared_ptr<RobotServices> RobotSer)
  {
    // request==request;
      // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
      RobotSer->move_group->getCurrentState()->getJointModelGroup(RobotSer->PLANNING_GROUP);
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = (float) rand()/RAND_MAX;
    target_pose1.position.x = 0.55;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.6;
    RobotSer->move_group->setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (RobotSer->move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

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
  std::shared_ptr<RobotServices> RS =      std::make_shared<RobotServices>(0.1);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  rclcpp::Service<mv2_com_interfaces::srv::MovePose>::SharedPtr service_ = RS->create_service<mv2_com_interfaces::srv::MovePose>("aaaaaaa",std::bind(&move_robot, std::placeholders::_1, std::placeholders::_2, RS));
  executor.add_node(RS);


  // std::thread([&executor]() { executor.spin(); }).detach();


  RCLCPP_INFO(LOGGER, "done spinning");


  RS->PLANNING_GROUP = PlanningGroup;

  RS->move_group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, RS->PLANNING_GROUP);
  executor.spin();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group =
      RS->move_group->getCurrentState()->getJointModelGroup(RS->PLANNING_GROUP);



  namespace rvt = rviz_visual_tools;



  // moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
  //                                                     RS->move_group->getRobotModel());





  // rclcpp::spin(move_group_node);



  RCLCPP_INFO(LOGGER, "Ready to MOVE .");
  while(rclcpp::ok()){

  }
  rclcpp::shutdown();
}