/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Trigger.h>

#include <hardware_interface/joint_command_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>
#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#include "pjtc_manager_mock.h"
#include "pjtc_test_helper.h"
#include "robot_driver_mock.h"
#include "trajectory_action_client_wrapper.h"

namespace pilz_joint_trajectory_controller_test
{
static const std::string CONTROLLER_NAMESPACE{ "/controller_ns" };
static const std::string TRAJECTORY_ACTION{ "/follow_joint_trajectory" };
static const std::string HOLD_SERVICE{ "/hold" };
static const std::string UNHOLD_SERVICE{ "/unhold" };
static const std::string IS_EXECUTING_SERVICE{ "/is_executing" };
static const std::string TRAJECTORY_COMMAND_TOPIC{ "/command" };
static const std::string CONTROLLER_JOINT_NAMES_PARAM{ "/controller_joint_names" };

using namespace pilz_joint_trajectory_controller;

using HWInterface = hardware_interface::PositionJointInterface;
using Segment = trajectory_interface::QuinticSplineSegment<double>;
using PJTCManager = PJTCManagerMock<Segment, HWInterface>;
using RobotDriver = RobotDriverMock<PJTCManager>;
using Controller = pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface>;
using ControllerPtr = std::shared_ptr<Controller>;

/**
 * @brief Test fixture class for the unit-test of the PilzJointTrajectoryController.
 *
 * A minimal test-robot is used to initialize the controller.
 *
 * @note ros::Time is set to use simulated time.
 */
class PilzJointTrajectoryControllerTest : public testing::Test
{
protected:
  void SetUp() override;

  testing::AssertionResult isControllerInHoldMode();
  testing::AssertionResult isControllerInUnholdMode();

protected:
  RobotDriver robot_driver_{ CONTROLLER_NAMESPACE };
  std::shared_ptr<PJTCManager> manager_;
  TrajectoryActionClientWrapper action_client_{ CONTROLLER_NAMESPACE + TRAJECTORY_ACTION };
  ros::NodeHandle controller_nh_{ CONTROLLER_NAMESPACE };
  ros::AsyncSpinner spinner_{ 2 };
};

void PilzJointTrajectoryControllerTest::SetUp()
{
  spinner_.start();
  manager_ = robot_driver_.getManager();
  setControllerParameters(CONTROLLER_NAMESPACE);
  startSimTime();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::isControllerInHoldMode()
{
  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);
  if (!action_client_.waitForActionResult())
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in hold mode.";
  }
  if (action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::INVALID_GOAL)
  {
    return testing::AssertionFailure() << "Error code is " << action_client_.getResult()->error_code
                                       << ", should have been INVALID_GOAL";
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::isControllerInUnholdMode()
{
  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);
  if (!action_client_.waitForActionResult([this]() { robot_driver_.update(); }))
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in unhold mode.";
  }
  if (action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
  {
    return testing::AssertionFailure() << "Error code is " << action_client_.getResult()->error_code
                                       << ", should have been SUCCESSFUL";
  }
  return testing::AssertionSuccess();
}

//////////////////////////////////
//  Testing of Initialization   //
//////////////////////////////////

/**
 * @brief Test if test environemt is initialized successfully.
 */
TEST_F(PilzJointTrajectoryControllerTest, testInitializiation)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + HOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + UNHOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + IS_EXECUTING_SERVICE, true));

  EXPECT_TRUE(action_client_.waitForActionServer());
}

/////////////////////////////////////////
//    Testing of additional methods    //
/////////////////////////////////////////

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants are called.
 */
TEST_F(PilzJointTrajectoryControllerTest, testD0Destructor)
{
  ControllerPtr controller{ new Controller() };
  SUCCEED();
}

/**
 * @brief Test if acceleration limits are correctly received.
 */
TEST_F(PilzJointTrajectoryControllerTest, testGetJointAccelerationLimits)
{
  // test setup
  ros::NodeHandle nh{ "~" };
  std::vector<std::string> joint_names;
  ASSERT_TRUE(nh.getParam(CONTROLLER_JOINT_NAMES_PARAM, joint_names));
  ASSERT_FALSE(joint_names.empty());

  // test with existing acc limits
  std::vector<double> acceleration_limits = getJointAccelerationLimits(nh, joint_names);
  EXPECT_EQ(joint_names.size(), acceleration_limits.size());
  EXPECT_FLOAT_EQ(acceleration_limits.at(0), 3.49);
  EXPECT_FLOAT_EQ(acceleration_limits.at(1), 3.49);  // as of pilz_control/test/config/joint_limits.yaml

  // testing behaviour if `has_acceleration_limits` is false
  std::vector<std::string> joint_names_has_acc_lim_false = { "joint_with_has_acc_lim_false" };
  std::vector<double> acceleration_limits_has_acc_lim_false =
      getJointAccelerationLimits(nh, joint_names_has_acc_lim_false);
  EXPECT_EQ(joint_names_has_acc_lim_false.size(), acceleration_limits_has_acc_lim_false.size());
  EXPECT_FLOAT_EQ(acceleration_limits_has_acc_lim_false.at(0), 0);
}

/**
 * @brief Test if an exception is thrown if the acceleration limits can not be found.
 */
TEST_F(PilzJointTrajectoryControllerTest, testGetJointAccelerationLimitsException)
{
  ros::NodeHandle nh{ "~" };

  // testing behaviour if acc limit can not be read
  std::vector<std::string> joint_names_no_acc_limit = { "joint_with_undefined_max_acc" };
  EXPECT_THROW(getJointAccelerationLimits(nh, joint_names_no_acc_limit), ros::InvalidParameterException);

  // testing behaviour if `has_acceleration_limits` is undefined
  std::vector<std::string> joint_names_has_acc_lim_undefined = { "joint_with_undefined_has_acc_lim" };
  EXPECT_THROW(getJointAccelerationLimits(nh, joint_names_has_acc_lim_undefined), ros::InvalidParameterException);
}

/////////////////////////////////////
//    Testing of hold and unhold   //
/////////////////////////////////////

/**
 * @tests{end_holding,
 * Tests unholding before the controller is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdFailureWhenNotStarted)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_FALSE(resp.success);
}

/**
 * @tests{hold_at_controller_start,
 * Tests that the controller is holded once it is started.
 * }
 * @tests{no_execution_during_hold,
 * Tests that the controller is holded once it is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldAtStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";
  ASSERT_TRUE(action_client_.waitForActionServer());

  manager_->startController();

  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{end_holding,
 * Tests unholding the controller at different time points after the controller is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdSuccess)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";
  ASSERT_TRUE(action_client_.waitForActionServer());

  manager_->startController();

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_FALSE(resp.success);

  ros::Duration stop_duration{ STOP_TRAJECTORY_DURATION_SEC + 2 * DEFAULT_UPDATE_PERIOD_SEC };
  progressInTime(stop_duration);
  robot_driver_.update();

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());
}

/**
 * @tests{start_holding,
 * Tests holding the controller.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldSuccessAfterUnhold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{start_holding,
 * Tests triggering hold two times successively.
 * }
 * @tests{no_execution_during_hold,
 * Tests triggering hold two times successively.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleHoldSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());

  EXPECT_TRUE(manager_->triggerHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{end_holding,
 * Tests triggering unhold two times successively.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());
}

/**
 * @tests{end_holding,
 * Tests holding and unholding the controller repeatedly.
 * }
 * @tests{start_holding,
 * Tests holding and unholding the controller repeatedly.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding and unholding the controller repeatedly.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testRepeatHoldAndUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  const unsigned int number_of_iterations{ 3U };
  for (unsigned int i = 0; i < number_of_iterations; ++i)
  {
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    // run async such that stop trajectory can be executed in the meantime
    std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
    EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(isControllerInHoldMode());

    EXPECT_TRUE(manager_->triggerUnHold(req, resp));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(isControllerInUnholdMode());
  }
}

/**
 * @tests{start_holding,
 * Tests holding the controller with a running trajectory.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller with a running trajectory.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldDuringGoalExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());

  EXPECT_TRUE(action_client_.waitForActionResult());
  EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
}

/**
 * @brief Cancel a goal after a hold was requested and before the next update is performed.
 *
 * This test was written to obtain full line coverage.
 */
TEST_F(PilzJointTrajectoryControllerTest, testGoalCancellingDuringHold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  // Sleep to make sure hold was triggered
  std::this_thread::sleep_for(HOLD_TIMEOUT);
  action_client_.cancelGoal();
  EXPECT_TRUE(action_client_.waitForActionResult());
  robot_driver_.update();
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

/////////////////////////////////////////////////////////////////////////////////////////////
//    Testing the correct handling of trajectories that violate the acceleration limits    //
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Send a trajectory that has too high acceleration and make sure controller does not execute it.
 */
TEST_F(PilzJointTrajectoryControllerTest, testTrajectoryWithTooHighAcceleration)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  // unhold controller and record start pose;
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());

  // Make sure, robot moves for slow motion
  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_, ros::Duration(DEFAULT_GOAL_DURATION_SEC)) };
  action_client_.sendGoal(goal);
  EXPECT_TRUE(manager_->controller_->is_executing());
  action_client_.waitForActionResult();
  EXPECT_TRUE(updateUntilNoRobotMotion<RobotDriver>(&robot_driver_));

  // Now sending a quicker motion which should trigger the acceleration limit and not move the robot
  goal = generateSimpleGoal<RobotDriver>(&robot_driver_, ros::Duration(DEFAULT_GOAL_DURATION_SEC), 1E4);
  EXPECT_TRUE(updateUntilNoRobotMotion<RobotDriver>(&robot_driver_));
  action_client_.sendGoal(goal);
  EXPECT_FALSE(manager_->controller_->is_executing());
  action_client_.waitForActionResult();
  EXPECT_TRUE(updateUntilNoRobotMotion<RobotDriver>(&robot_driver_));
}

//////////////////////////////////////////////////////////////////////////
//    Parameterized tests for the "is-executing-check" functionality    //
//////////////////////////////////////////////////////////////////////////

//! The return value indicates if the call was successful (in case of a service callback), the actual result
//! of the is-executing-check is assigned (via reference) to the second argument.
using InvokeIsExecuting = std::function<testing::AssertionResult(const ControllerPtr&, bool&)>;

static testing::AssertionResult InvokeIsExecutingMethod(const ControllerPtr& controller, bool& result)
{
  result = controller->is_executing();
  return testing::AssertionSuccess();
}

static testing::AssertionResult InvokeIsExecutingServiceCallback(const ControllerPtr& controller, bool& result)
{
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  if (!controller->handleIsExecutingRequest(req, resp))
  {
    return testing::AssertionFailure() << "Callback of is_executing returned false unexpectedly.";
  }

  result = resp.success;
  return testing::AssertionSuccess();
}

/**
 * @brief For testing both the isExecuting method and the is_executing service callback we use parameterized tests.
 */
class PilzJointTrajectoryControllerIsExecutingTest : public testing::Test,
                                                     public testing::WithParamInterface<InvokeIsExecuting>
{
protected:
  void SetUp() override;

  testing::AssertionResult invokeIsExecuting(bool& result);

protected:
  RobotDriver robot_driver_{ CONTROLLER_NAMESPACE };
  std::shared_ptr<PJTCManager> manager_;
  TrajectoryActionClientWrapper action_client_{ CONTROLLER_NAMESPACE + TRAJECTORY_ACTION };
  ros::NodeHandle controller_nh_{ CONTROLLER_NAMESPACE };
  ros::AsyncSpinner spinner_{ 2 };
};

void PilzJointTrajectoryControllerIsExecutingTest::SetUp()
{
  spinner_.start();
  manager_ = robot_driver_.getManager();
  setControllerParameters(CONTROLLER_NAMESPACE);
  startSimTime();
}

testing::AssertionResult PilzJointTrajectoryControllerIsExecutingTest::invokeIsExecuting(bool& result)
{
  auto invoke_is_executing{ GetParam() };
  return invoke_is_executing(manager_->controller_, result);
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotStarted)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

/**
 * @brief Check that a fake start (simply setting state to RUNNING) doesn't trigger that is_executing() returns true.
 *
 * Increases line coverage.
 */
TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testFakeStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  manager_->controller_->state_ = Controller::RUNNING;
  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  manager_->startController();

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution at start.";

  ros::Duration stop_duration{ STOP_TRAJECTORY_DURATION_SEC + 2 * DEFAULT_UPDATE_PERIOD_SEC };
  progressInTime(stop_duration);
  robot_driver_.update();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotExecutingAfterUnhold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testActionGoalExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect action goal execution";

  progressInTime(getGoalDuration(goal) + ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
  robot_driver_.update();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testTrajCommandExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  ros::NodeHandle nh{ "~" };
  ros::Publisher trajectory_command_publisher =
      nh.advertise<trajectory_msgs::JointTrajectory>(CONTROLLER_NAMESPACE + TRAJECTORY_COMMAND_TOPIC, 1);

  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  trajectory_command_publisher.publish(goal.trajectory);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect trajectory command execution";

  progressInTime(getGoalDuration(goal) + ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
  robot_driver_.update();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtHold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateSimpleGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  EXPECT_TRUE(action_client_.waitForActionResult());
  // the following fails due to https://github.com/ros-controls/ros_controllers/issues/174
  // EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);

  robot_driver_.update();

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution";

  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

INSTANTIATE_TEST_CASE_P(MethodAndServiceCallback, PilzJointTrajectoryControllerIsExecutingTest,
                        testing::Values(InvokeIsExecutingMethod, InvokeIsExecutingServiceCallback));

}  // namespace pilz_joint_trajectory_controller_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
