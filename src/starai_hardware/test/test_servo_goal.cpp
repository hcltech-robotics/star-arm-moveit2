#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "starai_hardware/servo_goal.hpp"

using starai_hardware::compute_servo_goal;
using starai_hardware::JointMap;

namespace
{
constexpr double kDeg = M_PI / 180.0;
const JointMap kArm{kDeg, 0.0, -1.0, 1.0};  // rad joint, limits +-1 rad
constexpr double kLead = 0.03;
}  // namespace

TEST(ServoGoal, TrackingCruisesAtCommandedSpeed)
{
  // At 0.2 rad, moving 1 rad/s, servo exactly on the setpoint.
  const double q = 0.2, v = 1.0;
  const auto g = compute_servo_goal(q, v, q / kDeg, kArm, kLead, kLead, 2.0, 360.0);
  EXPECT_NEAR(g.goal_deg, (q + v * kLead) / kDeg, 1e-9);
  EXPECT_NEAR(g.speed_dps, v / kDeg, 1e-9);  // ~57.3 deg/s
}

TEST(ServoGoal, LaggingServoSpeedsUpLeadingServoSlowsDown)
{
  const double q = 0.2, v = 1.0;
  const auto on = compute_servo_goal(q, v, q / kDeg, kArm, kLead, kLead, 2.0, 360.0);
  const auto behind = compute_servo_goal(q, v, q / kDeg - 2.0, kArm, kLead, kLead, 2.0, 360.0);
  const auto ahead = compute_servo_goal(q, v, q / kDeg + 1.0, kArm, kLead, kLead, 2.0, 360.0);
  EXPECT_GT(behind.speed_dps, on.speed_dps);
  EXPECT_LT(ahead.speed_dps, on.speed_dps);
  EXPECT_DOUBLE_EQ(behind.goal_deg, on.goal_deg);  // the goal does not depend on the measurement
}

TEST(ServoGoal, GoalNeverLeavesTheLimits)
{
  // Setpoint already past the limit, and the lead would push it further.
  const auto g = compute_servo_goal(1.2, 3.0, 50.0, kArm, kLead, kLead, 2.0, 360.0);
  EXPECT_NEAR(g.goal_deg, 1.0 / kDeg, 1e-9);
  const auto g2 = compute_servo_goal(-0.99, -3.0, -50.0, kArm, kLead, kLead, 2.0, 360.0);
  EXPECT_NEAR(g2.goal_deg, -1.0 / kDeg, 1e-9);
}

TEST(ServoGoal, SpeedIsClampedAndOffsetScaleApplied)
{
  // Gripper mapping: metres, 0.000261 m per servo degree, open (0) at 100 deg.
  const JointMap gripper{0.000261, 100.0, -0.026, 0.0};
  const auto g = compute_servo_goal(-0.01305, 0.0, 100.0, gripper, kLead, kLead, 2.0, 200.0);
  EXPECT_NEAR(g.goal_deg, 50.0, 1e-9);
  EXPECT_DOUBLE_EQ(g.speed_dps, 200.0);  // 50 deg gap in 30 ms would be 1667 deg/s
  const auto idle = compute_servo_goal(-0.01305, 0.0, 50.0, gripper, kLead, kLead, 2.0, 200.0);
  EXPECT_DOUBLE_EQ(idle.speed_dps, 2.0);  // never sends speed 0
}

TEST(ServoGoal, UnlimitedJointsAreNotClamped)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const JointMap free{kDeg, 0.0, nan, nan};
  const auto g = compute_servo_goal(10.0, 0.0, 10.0 / kDeg, free, kLead, kLead, 2.0, 360.0);
  EXPECT_NEAR(g.goal_deg, 10.0 / kDeg, 1e-9);
}

TEST(ServoGoal, HoldingJointIsNotResent)
{
  using starai_hardware::should_send;
  bool retried = false;
  EXPECT_TRUE(should_send(true, 10.0, 10.0, 1.5, false, false, retried));   // new goal
  EXPECT_FALSE(should_send(false, 10.0, 10.9, 1.5, true, false, retried));  // sag: leave it
  EXPECT_FALSE(should_send(false, 10.0, 15.0, 1.5, false, false, retried)); // retry not due
  EXPECT_TRUE(should_send(false, 10.0, 15.0, 1.5, true, false, retried));   // lost frame
  EXPECT_TRUE(should_send(false, 10.0, 15.0, 1.5, true, false, retried));   // ...every interval
}

TEST(ServoGoal, BlockedGripperIsResentOnce)
{
  using starai_hardware::should_send;
  bool retried = false;
  // Fingers closed on an object, 30 servo deg short of the goal.
  EXPECT_TRUE(should_send(true, 0.0, 30.0, 1.5, false, true, retried));   // the close command
  EXPECT_TRUE(should_send(false, 0.0, 30.0, 1.5, true, true, retried));   // one retry
  EXPECT_FALSE(should_send(false, 0.0, 30.0, 1.5, true, true, retried));  // then no re-squeeze
  EXPECT_FALSE(should_send(false, 0.0, 30.0, 1.5, true, true, retried));
  EXPECT_TRUE(should_send(true, 100.0, 30.0, 1.5, false, true, retried)); // open: new goal
  EXPECT_FALSE(retried);                                                  // retry allowed again
}

TEST(ServoGoal, TrimDoesNotWindUpAtALimit)
{
  using starai_hardware::limit_trim;
  using starai_hardware::update_trim;
  // Setpoint at the lower limit (-1 rad); a parking stop holds the servo 0.8 deg short.
  const double q = -1.0, stop_deg = q / kDeg + 0.8;
  double trim = 0.0;
  for (int k = 0; k < 500; ++k) {
    trim = limit_trim(update_trim(trim, q / kDeg - stop_deg, 0.01, 1.0, 0.2, 3.0), q, kArm);
  }
  EXPECT_NEAR(trim, 0.0, 1e-9);  // would be -3 deg without limit_trim
  // Inside the limits the trim is untouched; near one it is cut to what is left.
  EXPECT_NEAR(limit_trim(-2.0, 0.0, kArm), -2.0, 1e-9);
  EXPECT_NEAR(limit_trim(-2.0, -1.0 + 0.5 * kDeg, kArm), -0.5, 1e-9);
}

TEST(ServoGoal, OutsideLimitsMarginIsInServoDegrees)
{
  using starai_hardware::outside_limits;
  EXPECT_FALSE(outside_limits(1.0 + 2.0 * kDeg, kArm, 3.0));  // 2 deg past: nudge back
  EXPECT_TRUE(outside_limits(1.0 + 4.0 * kDeg, kArm, 3.0));   // 4 deg past: refuse
  const JointMap gripper{0.000261, 100.0, -0.026, 0.0};
  EXPECT_TRUE(outside_limits(-0.0275, gripper, 3.0));          // 1.5 mm = 5.7 servo deg
}

TEST(ServoGoal, GentleCorrectionNearTheEndOfAMove)
{
  // Setpoint creeping at 2 deg/s, servo stuck 0.9 deg behind in its own deadband.
  const double q = 0.0, v = 2.0 * kDeg, behind = -0.9;
  const auto g = compute_servo_goal(q, v, behind, kArm, kLead, 0.2, 0.5, 360.0);
  EXPECT_NEAR(g.speed_dps, 2.0 + 0.9 / 0.2, 1e-9);  // 6.5 deg/s, not 2 + 0.9 / 0.03 = 32
  // Ahead of the setpoint: slow down, but never below the minimum speed.
  const auto a = compute_servo_goal(q, v, 0.9, kArm, kLead, 0.2, 0.5, 360.0);
  EXPECT_DOUBLE_EQ(a.speed_dps, 0.5);
}

TEST(ServoGoal, TrimConvergesIgnoresNoiseAndIsCapped)
{
  using starai_hardware::update_trim;
  double trim = 0.0;
  // Joint sags 0.7 deg; the servo follows the trimmed goal, so the error shrinks as trim grows.
  for (int k = 0; k < 500; ++k) {  // 5 s at 100 Hz, time constant 1 s
    trim = update_trim(trim, 0.7 - trim, 0.01, 1.0, 0.2, 3.0);
  }
  EXPECT_NEAR(trim, 0.5, 0.01);  // stops once the error is inside the 0.2 deg deadband
  EXPECT_DOUBLE_EQ(update_trim(0.3, 0.15, 0.01, 1.0, 0.2, 3.0), 0.3);  // encoder noise: no change
  EXPECT_DOUBLE_EQ(update_trim(2.99, 5.0, 0.01, 0.1, 0.2, 3.0), 3.0);  // capped
  EXPECT_DOUBLE_EQ(update_trim(0.0, 5.0, 0.01, 0.0, 0.2, 3.0), 0.0);   // off
}
