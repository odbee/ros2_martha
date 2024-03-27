#include "rclcpp/rclcpp.hpp"
#include "igus_axis_interfaces/srv/send_float.hpp"
// #include "igus_axis_driver/D1.h"
#include "../hardware/include/igus_axis_driver/D1.h"
#include <memory>


void MoveAbs(const std::shared_ptr<igus_axis_interfaces::srv::SendFloat::Request> request,
          std::shared_ptr<igus_axis_interfaces::srv::SendFloat::Response>      response,std::shared_ptr<D1> axis)
{

  axis->profilePositionAbs(request->pos*1000, 50, 200);
  // response->sum = request->a + request->b;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
  //               request->a, request->b);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  response->success=true;

}

void Home(const std::shared_ptr<igus_axis_interfaces::srv::SendFloat::Request> request,
          std::shared_ptr<igus_axis_interfaces::srv::SendFloat::Response>      response,std::shared_ptr<D1> axis)
{

  axis->homing(50, 1, 100);
  response->success=true;
  response->message="successfully homed";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "succesfully homed");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_axis_server");
  std::shared_ptr<D1> xAxis =      std::make_shared<D1>();
  std::string ip="192.168.0.10";
  int port=502;
  
  xAxis->startConnection(ip, port);
  rclcpp::Service<igus_axis_interfaces::srv::SendFloat>::SharedPtr move_service =
    node->create_service<igus_axis_interfaces::srv::SendFloat>("move_abs_axis",std::bind(&MoveAbs, std::placeholders::_1, std::placeholders::_2, xAxis));
  rclcpp::Service<igus_axis_interfaces::srv::SendFloat>::SharedPtr home_service =
    node->create_service<igus_axis_interfaces::srv::SendFloat>("home_axis",std::bind(&Home, std::placeholders::_1, std::placeholders::_2, xAxis));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Interact with Axis.");

  rclcpp::spin(node);
  
  rclcpp::shutdown();
}