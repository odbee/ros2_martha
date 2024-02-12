#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from mv2_com_interfaces.srv import MovePose
from rclpy.node import Node

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped




class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(MovePose, 'movepose', self.add_two_ints_callback)
            # instantiate MoveItPy instance and get planning component
        
        
        self.logger = get_logger("moveit_py.pose_goal")
        self.panda = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.logger.info("MoveItPy instance created")



    def add_two_ints_callback(self, request, response):
        response.success = True
        request
        self.get_logger().info('received incoming move pose command')

        # set plan start state to current state
        self.panda_arm.set_start_state_to_current_state()


        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")

        # plan to goal
        plan_and_execute(self.panda, self.panda_arm, self.logger, sleep_time=0)

        return response


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()

    minimal_service = MinimalService()
    rclpy.spin(minimal_service)









if __name__ == "__main__":
    main()