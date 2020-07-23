#!/usr/bin/env python
import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from holding_mode_service_wrapper import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

CONTROLLER_NS_PARAMETER = '/controller_ns_string'
CONTROLLER_NAME_PARAMTER = '/controller_name_string'

WAIT_FOR_SERVICE_TIMEOUT_S = 10
DEFAULT_TRAJECTORY_DURATION_S = 10

FOLLOW_JOINT_TRAJECTORY_SUFFIX = "/follow_joint_trajectory"
JOINT_NAMES_SUFFIX = "/joint_names"

def init_traj_goal(joint_names):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    return goal

def add_traj_point(goal, goal_position, goal_velocity=[], goal_acceleration=[], time_from_last=0.1):
    point = JointTrajectoryPoint()
    point.positions = goal_position
    point.velocities = goal_velocity
    point.accelerations = goal_acceleration
    point.time_from_start = rospy.Duration(time_from_last)
    if len(goal.trajectory.points) > 0:
        point.time_from_start += goal.trajectory.points[-1].time_from_start

    goal.trajectory.points.append(point)

def perform_test():
    # initialize
    controller_ns = rospy.get_param(CONTROLLER_NS_PARAMETER)
    controller_name = rospy.get_param(CONTROLLER_NAME_PARAMTER)

    joint_names = rospy.get_param(controller_ns + JOINT_NAMES_SUFFIX)

    action_name = controller_ns + "/" + controller_name + FOLLOW_JOINT_TRAJECTORY_SUFFIX
    traj_client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
    traj_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVICE_TIMEOUT_S))

    hold_srv = HoldingModeServiceWrapper(controller_ns, controller_name)
    hold_srv.request_default_mode()

    # let's go
    goal = init_traj_goal(joint_names)
    add_traj_point(goal, [0.0, 0.0], goal_velocity=[0.0, 0.0], time_from_last=0.0)
    add_traj_point(goal, [0.01, 0.01], goal_velocity=[0.1, 0.1], time_from_last=0.1)
    add_traj_point(goal, [0.03, 0.03], goal_velocity=[0.2, 0.2], time_from_last=0.1)
    add_traj_point(goal, [0.06, 0.06], goal_velocity=[0.3, 0.3], time_from_last=0.1)

    traj_client.send_goal_and_wait(goal)

if __name__ == "__main__":
    rospy.init_node('test_limits')
    perform_test()
