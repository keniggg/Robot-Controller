#include <gtest/gtest.h>

#include "alicia_d_driver/actuation_confirmation.hpp"
#include "alicia_d_driver/endpoint_trim_continuity.hpp"
#include "alicia_d_driver/endpoint_trim_driver_admission.hpp"
#include "alicia_d_driver/gui_direct_hold.hpp"
#include "alicia_d_driver/sdk_position_codec.hpp"
#include "alicia_d_driver/joint_feedback_recovery.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

ActuationConfirmationConfig test_config()
{
    ActuationConfirmationConfig config;
    config.sync_tolerance_rad = 0.05;
    config.command_probe_min_delta_rad = 0.02;
    config.measured_response_min_delta_rad = 0.003;
    config.response_timeout_sec = 1.0;
    config.confirmation_freshness_sec = 2.0;
    return config;
}

std::vector<double> joints(
    double joint1 = 0.0,
    double joint2 = 0.0,
    double joint3 = 0.0,
    double joint4 = 0.0,
    double joint5 = 0.0,
    double joint6 = 0.0
)
{
    return {joint1, joint2, joint3, joint4, joint5, joint6};
}

double degrees(double value)
{
    return value * M_PI / 180.0;
}

double driver_sdk_quantize(double radians)
{
    return sdk_joint_position_decode(sdk_joint_position_encode(radians));
}

EndpointTrimConfig endpoint_trim_config()
{
    return EndpointTrimConfig();
}

std::vector<uint8_t> all_stable_joints()
{
    return {1, 1, 1, 1, 1, 1};
}

void expect_vectors_near(
    const std::vector<double>& actual,
    const std::vector<double>& expected,
    double tolerance
)
{
    ASSERT_EQ(actual.size(), expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        EXPECT_NEAR(actual[i], expected[i], tolerance) << "joint " << i;
    }
}

}  // namespace

TEST(ActuationConfirmationTest, PositiveEnableIsPendingNotConfirmed)
{
    ActuationConfirmation confirmation(test_config());

    confirmation.reset_for_positive_enable(10.0);

    EXPECT_EQ(confirmation.state(), ActuationState::PENDING);
    EXPECT_FALSE(confirmation.motion_confirmed(10.1));
    EXPECT_EQ(
        confirmation.status_text(),
        "PENDING:POSITIVE_ENABLE_REQUESTED"
    );
}

TEST(GuiControlOwnershipTest, CheckedModeExclusivelyAdmitsSliderCommands)
{
    EXPECT_TRUE(joint_source_allowed_by_control_mode(true, "gui_direct"));
    EXPECT_TRUE(joint_source_allowed_by_control_mode(true, "gui_direct_sync"));
    for (const std::string source : {"", "motion_gateway", "ros_control", "task"}) {
        EXPECT_FALSE(joint_source_allowed_by_control_mode(true, source));
        EXPECT_TRUE(joint_source_allowed_by_control_mode(false, source));
    }
    EXPECT_FALSE(joint_source_allowed_by_control_mode(false, "gui_direct"));
    EXPECT_FALSE(joint_source_allowed_by_control_mode(false, "gui_direct_sync"));
}

TEST(ActuationConfirmationTest, RequiresFreshFeedbackAndNearFeedbackSync)
{
    ActuationConfirmation confirmation(test_config());
    std::string reason;
    confirmation.reset_for_positive_enable(10.0);

    EXPECT_FALSE(
        confirmation.admit_command(joints(0.0, 0.01), 10.1, &reason)
    );
    EXPECT_EQ(reason, "FEEDBACK_REQUIRED_FOR_SYNC");
    EXPECT_FALSE(confirmation.synchronized());

    confirmation.note_feedback(joints(), 10.2);
    EXPECT_FALSE(
        confirmation.admit_command(joints(0.0, 0.20), 10.3, &reason)
    );
    EXPECT_EQ(reason, "STALE_COMMAND_AFTER_RECONNECT");
    EXPECT_FALSE(confirmation.synchronized());

    EXPECT_TRUE(
        confirmation.admit_command(joints(0.0, 0.01), 10.4, &reason)
    );
    EXPECT_TRUE(confirmation.synchronized());
    EXPECT_TRUE(reason.empty());
}

TEST(GuiControlOwnershipTest, September11RestartBridgeCannotSupplyMeasuredHold)
{
    const auto measured = joints(-1.5201749607946704, -0.17640779060684875,
        0.18100973297050565, -1.3054176504906807, 0.2423689644859313,
        -0.2316310989707318);
    const auto old_bridge = joints(-1.5199479757853986, -0.19938344973840005,
        0.2000837867708086, -1.2868773658785526, 0.24417114956652705,
        -0.2501198781127774);
    // The reconnect allowance alone admits this 0.023 rad bridge sample;
    // automatic ownership must first require a near-zero-motion hold.
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(measured, 10.1);
    ASSERT_TRUE(confirmation.admit_command(old_bridge, 10.2, nullptr));
    EXPECT_FALSE(controller_handoff_matches_feedback(old_bridge, measured, true));
    EXPECT_TRUE(controller_handoff_matches_feedback(measured, measured, true));
}

TEST(ActuationConfirmationTest, DirectionalEncoderResponseConfirmsActuation)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(
        confirmation.admit_command(joints(0.0, 0.01), 10.2, nullptr)
    );

    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);
    confirmation.note_feedback(joints(0.0, 0.006), 10.4);

    EXPECT_EQ(confirmation.state(), ActuationState::CONFIRMED);
    EXPECT_TRUE(confirmation.motion_confirmed(10.5));
    EXPECT_EQ(
        confirmation.status_text(),
        "CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE"
    );
}

TEST(ActuationConfirmationTest, RepeatedEnablePreservesOnlyFreshConnectedConfirmation)
{
    ActuationConfirmation confirmation(test_config());
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 10.0));
    confirmation.reset_for_positive_enable(10.0);
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 10.1));
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);
    confirmation.note_feedback(joints(0.0, 0.006), 10.4);

    EXPECT_TRUE(confirmation.can_preserve_positive_enable(true, true, 10.5));
    EXPECT_TRUE(confirmation.synchronized());
    EXPECT_TRUE(confirmation.motion_confirmed(10.5));
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(false, true, 10.5));
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, false, 10.5));
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 12.5));
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 10.0));
    confirmation.mark_unconfirmed("DISCONTINUOUS_FEEDBACK_RECOVERY", 10.6);
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 10.7));
    confirmation.mark_overheat_blocked("TEMPERATURE", 10.8);
    EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, 10.9));
}

TEST(JointFeedbackRecoveryTest, September11RepeatedJoint3SampleAgreesWithHeldCommand)
{
    // Captured delta 0.147262, previous error 0.135398, candidate error
    // 0.011864 rad: the old 0.1323 rad polling margin rejected 273 frames.
    const auto previous = joints(-1.922078, 0.443320, -0.004602, -0.005, -0.30833, -0.001534);
    const auto candidate = joints(-1.922078, 0.443320, -0.151864, -0.005, -0.30833, -0.001534);
    const auto command = joints(-1.922078, 0.443320, -0.140000, -0.005, -0.30833, -0.001534);
    EXPECT_TRUE(alicia_d_driver::command_consistent_feedback_recovery(
        previous, candidate, command, 0.05));
}

TEST(JointFeedbackRecoveryTest, MovingTowardCommandWithoutReachingItDoesNotRecover)
{
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.0, 0.2), joints(0.0, 0.5), 0.05));
}

TEST(JointFeedbackRecoveryTest, CorruptionAwayFromHeldCommandDoesNotRecover)
{
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.0, -2.4), joints(0.0, 0.1), 0.05));
    // Correcting the largest jump cannot conceal another inconsistent joint.
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.5, -0.2), joints(0.5, 0.0), 0.05));
}

TEST(JointFeedbackRecoveryTest, SmallUnchangedOrInvalidSamplesDoNotSupplyRecoveryEvidence)
{
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(), joints(), 0.05));
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.01), joints(0.01), 0.05));
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.2), {}, 0.05));
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(std::numeric_limits<double>::quiet_NaN()), joints(), 0.05));
    EXPECT_FALSE(alicia_d_driver::command_consistent_feedback_recovery(
        joints(), joints(0.2), joints(0.2), 0.0));
}

TEST(ActuationConfirmationTest, SmoothFollowingConfirmsAccumulatedCommandResponse)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    for (int step = 1; step <= 25; ++step) {
        const double stamp = 10.2 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.001 * step), stamp);
        confirmation.note_feedback(joints(0.001 * step - 0.0005), stamp + 0.01);
        confirmation.update(stamp + 0.02);
    }
    EXPECT_TRUE(confirmation.motion_confirmed(12.8));
}

TEST(ActuationConfirmationTest, SmoothResponseUsesDisplacementDespiteFixedEncoderOffset)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(-0.008), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    for (int step = 1; step <= 25; ++step) {
        const double stamp = 10.2 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.001 * step), stamp);
        confirmation.note_feedback(joints(0.001 * step - 0.008), stamp + 0.01);
        confirmation.update(stamp + 0.02);
    }
    EXPECT_TRUE(confirmation.motion_confirmed(12.8));
}

TEST(ActuationConfirmationTest, SmoothCommandWithoutResponseStillTimesOut)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    for (int step = 1; step <= 25; ++step) {
        const double stamp = 10.2 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.001 * step), stamp);
        confirmation.note_feedback(joints(), stamp + 0.01);
    }
    confirmation.update(13.5);
    EXPECT_FALSE(confirmation.motion_confirmed(13.5));
    EXPECT_EQ(confirmation.status_text(), "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT");
}

TEST(ActuationConfirmationTest, CommandGapDropsAccumulationBaseline)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.01), 10.3);
    confirmation.note_feedback(joints(0.01), 10.4);
    confirmation.note_feedback(joints(0.02), 12.0);
    confirmation.note_streamed_target(joints(0.025), 12.1);
    confirmation.note_feedback(joints(0.025), 12.2);
    EXPECT_FALSE(confirmation.motion_confirmed(12.3));
    confirmation.mark_unconfirmed("DISCONTINUOUS_FEEDBACK_RECOVERY", 12.4);
    confirmation.note_streamed_target(joints(0.026), 12.5);
    confirmation.note_feedback(joints(0.026), 12.6);
    EXPECT_FALSE(confirmation.motion_confirmed(12.6));
}

TEST(ActuationConfirmationTest, MotionBeforeACommandCannotSupplyAccumulatedResponse)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(), 10.3);
    confirmation.note_feedback(joints(0.025), 10.4);
    confirmation.note_streamed_target(joints(0.025), 10.5);
    confirmation.note_feedback(joints(0.025), 10.6);
    EXPECT_FALSE(confirmation.motion_confirmed(10.7));
}

TEST(ActuationConfirmationTest, OwnershipChangeDiscardsUnfinishedProbe)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.025), 10.3);
    confirmation.reset_command_synchronization();
    confirmation.note_feedback(joints(0.01), 10.4);
    EXPECT_FALSE(confirmation.motion_confirmed(10.5));
    EXPECT_FALSE(confirmation.synchronized());
}

TEST(ActuationConfirmationTest, OwnershipHandoffClearsOldProbeButPreservesSynchronization)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(.025), 10.3);
    confirmation.reset_motion_observation();
    confirmation.note_feedback(joints(.01), 10.4);
    EXPECT_FALSE(confirmation.motion_confirmed(10.5)); // Old motion is not confirmation.
    EXPECT_TRUE(confirmation.synchronized());
    EXPECT_EQ(confirmation.status_text(), "PENDING:COMMAND_SYNCHRONIZED");
    confirmation.note_streamed_target(joints(.05), 10.6);
    confirmation.note_feedback(joints(.02), 10.7);
    ASSERT_TRUE(confirmation.motion_confirmed(10.8));
    confirmation.reset_motion_observation();
    EXPECT_TRUE(confirmation.motion_confirmed(10.8));
    EXPECT_TRUE(confirmation.synchronized());
    // A slider goal outside reconnect tolerance remains admissible: no torque reset.
    EXPECT_TRUE(confirmation.admit_command(joints(.4), 10.9, nullptr));
}

TEST(ActuationConfirmationTest, ZeroResponseTimesOutUnconfirmed)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);

    confirmation.note_feedback(joints(), 10.9);
    confirmation.update(11.31);

    EXPECT_EQ(confirmation.state(), ActuationState::UNCONFIRMED);
    EXPECT_FALSE(confirmation.motion_confirmed(11.31));
    EXPECT_EQ(
        confirmation.status_text(),
        "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT"
    );
}

TEST(ActuationConfirmationTest, OppositeAndUnrelatedMotionDoNotConfirm)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);

    confirmation.note_feedback(joints(0.0, -0.006), 10.4);
    EXPECT_EQ(confirmation.state(), ActuationState::PENDING);
    confirmation.note_feedback(joints(0.0, 0.0, 0.006), 10.5);
    EXPECT_EQ(confirmation.state(), ActuationState::PENDING);

    confirmation.update(11.31);
    EXPECT_EQ(confirmation.state(), ActuationState::UNCONFIRMED);
}

TEST(ActuationConfirmationTest, RepeatedKeepaliveDoesNotResetProbeDeadline)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);

    confirmation.note_streamed_target(joints(0.0, 0.10), 11.2);
    confirmation.update(11.31);

    EXPECT_EQ(confirmation.state(), ActuationState::UNCONFIRMED);
    EXPECT_EQ(
        confirmation.status_text(),
        "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT"
    );
}

TEST(ActuationConfirmationTest, TimedOutProbeCannotRearmFromRetainedTargets)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.025), 10.3);
    confirmation.update(11.31);

    // The September 11 slider probe kept streaming after its timeout. That
    // keepalive must not hide the failure from the GUI's next explicit action.
    for (int step = 0; step <= 30; ++step) {
        const double stamp = 11.4 + 0.1 * step;
        confirmation.note_feedback(joints(), stamp);
        confirmation.note_streamed_target(joints(0.0, 0.025), stamp);
        confirmation.update(stamp);
        EXPECT_EQ(confirmation.status_text(), "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT");
    }
    // Late movement alone cannot authorize a discarded full slider target.
    confirmation.note_feedback(joints(0.0, 0.006), 14.5);
    EXPECT_FALSE(confirmation.motion_confirmed(14.5));
}

TEST(ActuationConfirmationTest, NewPositiveEnableCanRecoverTimedOutProbe)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.025), 10.3);
    confirmation.update(11.31);

    confirmation.reset_for_positive_enable(12.0);
    EXPECT_FALSE(confirmation.admit_command(joints(), 12.01, nullptr));
    confirmation.note_feedback(joints(), 12.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 12.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.025), 12.3);
    confirmation.note_feedback(joints(0.0, 0.006), 12.4);
    EXPECT_TRUE(confirmation.motion_confirmed(12.4));
}

TEST(ActuationConfirmationTest, FreshFeedbackKeepsConfirmationAlive)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);
    confirmation.note_feedback(joints(0.0, 0.006), 10.4);

    confirmation.note_feedback(joints(0.0, 0.006), 12.2);
    confirmation.update(14.0);
    EXPECT_TRUE(confirmation.motion_confirmed(14.0));

    confirmation.update(14.21);
    EXPECT_EQ(confirmation.state(), ActuationState::UNCONFIRMED);
    EXPECT_EQ(
        confirmation.status_text(),
        "UNCONFIRMED:CONFIRMED_FEEDBACK_STALE"
    );
}

TEST(ActuationConfirmationTest, ConfirmedStreamLosesResponseDespiteFreshFeedback)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.025), 10.3);
    confirmation.note_feedback(joints(0.025), 10.4);
    ASSERT_TRUE(confirmation.motion_confirmed(10.4));

    // The SDK keeps accepting a 0.02 rad/s stream while all encoders stop.
    // Repeated packets are fresh telemetry, not fresh actuation evidence.
    bool response_lost = false;
    for (int step = 0; step <= 40; ++step) {
        const double stamp = 10.5 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.025 + 0.002 * step), stamp);
        confirmation.note_feedback(joints(0.025), stamp + 0.01);
        confirmation.update(stamp + 0.02);
        if (!confirmation.motion_confirmed(stamp + 0.02)) {
            EXPECT_EQ(confirmation.status_text(), "UNCONFIRMED:ENCODER_RESPONSE_LOST");
            EXPECT_FALSE(confirmation.can_preserve_positive_enable(true, true, stamp + 0.02));
            EXPECT_TRUE(confirmation.synchronized());
            response_lost = true;
        } else {
            EXPECT_FALSE(response_lost) << "An old stream cannot restore a failed confirmation";
        }
    }
    EXPECT_TRUE(response_lost);
}

TEST(ActuationConfirmationTest, ConfirmedSmoothFollowingRemainsConfirmed)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.025), 10.3);
    confirmation.note_feedback(joints(0.025), 10.4);
    for (int step = 0; step <= 150; ++step) {
        const double stamp = 10.5 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.05 + 0.002 * step), stamp);
        confirmation.note_feedback(joints(driver_sdk_quantize(0.025 + 0.002 * step)), stamp + 0.01);
        confirmation.update(stamp + 0.02);
        EXPECT_TRUE(confirmation.motion_confirmed(stamp + 0.02)) << step;
    }
}

TEST(ActuationConfirmationTest, ConfirmedStationaryKeepaliveWithOffsetIsNotMotion)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.05), 10.3);
    confirmation.note_feedback(joints(0.025), 10.4);
    for (int step = 0; step <= 100; ++step) {
        const double stamp = 10.5 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.05), stamp);
        confirmation.note_feedback(joints(0.025), stamp + 0.01);
        confirmation.update(stamp + 0.02);
        EXPECT_TRUE(confirmation.motion_confirmed(stamp + 0.02));
    }
}

TEST(ActuationConfirmationTest, ConfirmedStreamDoesNotAcceptUnrelatedJointMotion)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.025), 10.3);
    confirmation.note_feedback(joints(0.025), 10.4);
    bool response_lost = false;
    for (int step = 0; step <= 40; ++step) {
        const double stamp = 10.5 + 0.1 * step;
        confirmation.note_streamed_target(joints(0.025 + 0.002 * step), stamp);
        confirmation.note_feedback(joints(0.025, 0.004 * step), stamp + 0.01);
        confirmation.update(stamp + 0.02);
        if (!confirmation.motion_confirmed(stamp + 0.02)) {
            response_lost = true;
            break;
        }
    }
    EXPECT_TRUE(response_lost);
}

TEST(ActuationConfirmationTest, September13SettlingThenReverseUsesNewMotionBaseline)
{
    // Reduced wire/encoder count sequence from the 01:29 manual Joint5
    // reversal. Encoders moved 2084 -> 2056, yet the old detector waited
    // for them to cross a baseline from the preceding forward movement.
    const double quantum = 2.0 * M_PI / 4096.0;
    const auto q = [quantum](int count) {
        return joints(0., 0., 0., 0., (count - 2048) * quantum);
    };
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(q(1965), 10.1);
    ASSERT_TRUE(confirmation.admit_command(q(1965), 10.2, nullptr));
    confirmation.note_streamed_target(q(2031), 10.3);
    confirmation.note_feedback(q(1973), 10.4);
    ASSERT_TRUE(confirmation.motion_confirmed(10.4));
    confirmation.note_streamed_target(q(2071), 10.5);
    confirmation.note_feedback(q(1988), 10.6);
    confirmation.note_streamed_target(q(2092), 10.7);
    confirmation.note_feedback(q(2003), 10.8);
    confirmation.note_streamed_target(q(2092), 10.9);
    confirmation.note_feedback(q(2084), 11.0);
    confirmation.note_streamed_target(q(2092), 11.1);
    confirmation.note_streamed_target(q(2085), 11.2);
    confirmation.note_streamed_target(q(2079), 11.3);
    confirmation.note_streamed_target(q(2071), 11.4);
    confirmation.note_feedback(q(2082), 11.401);
    confirmation.note_streamed_target(q(2058), 11.5);
    for (int i = 0; i < 10; ++i) {
        const double now = 11.6 + .1 * i;
        confirmation.note_feedback(q(2056), now);
        confirmation.note_streamed_target(q(2058), now + .01);
        confirmation.update(now + .02);
    }
    EXPECT_TRUE(confirmation.motion_confirmed(12.52));
    EXPECT_EQ(confirmation.state(), ActuationState::CONFIRMED);
}

TEST(ActuationConfirmationTest, PriorSettlingCannotConfirmAStalledNewCommand)
{
    for (int response_kind = 0; response_kind < 3; ++response_kind) {
        SCOPED_TRACE(response_kind); // stationary, wrong direction, other axis
        ActuationConfirmation confirmation(test_config());
        confirmation.reset_for_positive_enable(10.0);
        confirmation.note_feedback(joints(), 10.1);
        ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
        confirmation.note_streamed_target(joints(.05), 10.3);
        confirmation.note_feedback(joints(.01), 10.4);
        ASSERT_TRUE(confirmation.motion_confirmed(10.4));
        confirmation.note_streamed_target(joints(.05), 10.5);
        confirmation.note_feedback(joints(.045), 10.6);
        confirmation.note_streamed_target(joints(.05), 10.7);
        confirmation.note_streamed_target(joints(.08), 10.8);
        for (int i = 0; i < 12; ++i) {
            const double now = 10.9 + .1 * i;
            confirmation.note_feedback(joints(
                response_kind == 1 ? .04 : .045,
                response_kind == 2 ? .02 : 0.), now);
            confirmation.note_streamed_target(joints(.08), now + .01);
            confirmation.update(now + .02);
        }
        EXPECT_EQ(confirmation.status_text(), "UNCONFIRMED:ENCODER_RESPONSE_LOST");
    }
}

TEST(ActuationConfirmationTest, OverheatBlocksWithoutAConfirmedState)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);

    confirmation.mark_overheat_blocked(
        "SUSTAINED_SAME_CHANNEL_TEMPERATURE",
        10.2
    );

    EXPECT_EQ(confirmation.state(), ActuationState::OVERHEAT_BLOCKED);
    EXPECT_FALSE(confirmation.motion_confirmed(10.3));
    EXPECT_EQ(
        confirmation.status_text(),
        "OVERHEAT_BLOCKED:SUSTAINED_SAME_CHANNEL_TEMPERATURE"
    );
}

TEST(ActuationConfirmationTest, FailedPositiveFrameIsExplicitlyUnconfirmed)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);

    confirmation.mark_unconfirmed("TORQUE_ON_WRITE_FAILED", 10.1);

    EXPECT_EQ(confirmation.state(), ActuationState::UNCONFIRMED);
    EXPECT_FALSE(confirmation.motion_confirmed(10.2));
    EXPECT_EQ(
        confirmation.status_text(),
        "UNCONFIRMED:TORQUE_ON_WRITE_FAILED"
    );
}

TEST(ActuationConfirmationTest, EnableResetDropsSyncProbeAndPriorConfirmation)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);
    confirmation.note_feedback(joints(), 10.1);
    ASSERT_TRUE(confirmation.admit_command(joints(), 10.2, nullptr));
    confirmation.note_streamed_target(joints(0.0, 0.10), 10.3);
    confirmation.note_feedback(joints(0.0, 0.006), 10.4);
    ASSERT_EQ(confirmation.state(), ActuationState::CONFIRMED);

    confirmation.reset_for_positive_enable(20.0);

    EXPECT_EQ(confirmation.state(), ActuationState::PENDING);
    EXPECT_FALSE(confirmation.synchronized());
    EXPECT_FALSE(confirmation.motion_confirmed(20.1));
    std::string reason;
    EXPECT_FALSE(
        confirmation.admit_command(joints(0.0, 0.01), 20.2, &reason)
    );
    EXPECT_EQ(reason, "FEEDBACK_REQUIRED_FOR_SYNC");
}

TEST(ActuationConfirmationTest, ExplicitDisableIsTheDisabledTransition)
{
    ActuationConfirmation confirmation(test_config());
    confirmation.reset_for_positive_enable(10.0);

    confirmation.set_disabled(10.1);

    EXPECT_EQ(confirmation.state(), ActuationState::DISABLED);
    EXPECT_FALSE(confirmation.motion_confirmed(10.2));
    EXPECT_EQ(confirmation.status_text(), "DISABLED:EXPLICIT_TORQUE_OFF");
}

TEST(GuiDirectHoldTest, PreservesRecentStreamedSetpointsForUneditedJoints)
{
    const std::vector<double> measured =
        joints(0.0, 0.555, -0.262, 0.0, -0.324, 0.046);
    const std::vector<double> streamed =
        joints(0.0, 0.560, -0.253, 0.0, -0.321, 0.049);

    EXPECT_EQ(
        select_gui_direct_hold_reference(measured, streamed, true),
        streamed
    );
}

TEST(GuiDirectHoldTest, FallsBackToFeedbackWithoutAUsableStreamedSetpoint)
{
    const std::vector<double> measured =
        joints(0.0, 0.555, -0.262, 0.0, -0.324, 0.046);

    EXPECT_EQ(
        select_gui_direct_hold_reference(measured, joints(), false),
        measured
    );
    EXPECT_EQ(
        select_gui_direct_hold_reference(
            measured,
            std::vector<double>{0.0, 0.1},
            true
        ),
        measured
    );
}

TEST(GuiDirectHoldTest, ChangingSliderPreservesEarlierUnfinishedManualGoals)
{
    const auto measured = joints(0.0, 0.1, 0.2, 0.3, 0.4, 0.5);
    const auto streamed = joints(0.01, 0.11, 0.21, 0.31, 0.41, 0.51);
    auto target = select_gui_direct_command_reference(measured, streamed, true, {});
    target[0] = 0.8;  // Explicit J1 goal, not yet reached by SDK interpolation.
    auto next = select_gui_direct_command_reference(measured, streamed, true, target);
    next[1] = -0.6;   // Next message only edits J2; J1 must continue to 0.8.
    EXPECT_DOUBLE_EQ(next[0], 0.8);
    EXPECT_DOUBLE_EQ(next[1], -0.6);
    for (size_t i = 2; i < 6; ++i) EXPECT_DOUBLE_EQ(next[i], streamed[i]);
    // Neither encoder drift nor an idle/expired keepalive replaces admitted goals.
    EXPECT_EQ(select_gui_direct_command_reference(joints(), {}, false, next), next);
}

TEST(GuiDirectHoldTest, SixEditedChannelsCanRetainSixConcurrentGoals)
{
    auto target = joints();
    for (size_t i = 0; i < 6; ++i) {
        target = select_gui_direct_command_reference(joints(), joints(), true, target);
        target[i] = 0.1 * (i + 1);
    }
    for (size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(target[i], 0.1 * (i + 1));
}

TEST(GuiDirectHoldTest, NewOwnershipEpochDoesNotResumeOldManualOrAutomaticGoals)
{
    const auto wire = joints(.01, .02, .03, .04, .05, .06);
    EXPECT_EQ(select_gui_direct_command_reference(joints(), wire, true, {}), wire);
    // An actuation/reconnect reset invalidates both manual and transmitted caches.
    const auto feedback = joints(.3, .2, .1);
    EXPECT_EQ(select_gui_direct_command_reference(feedback, {}, false, {}), feedback);
    auto malformed = joints();
    malformed[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(select_gui_direct_command_reference(feedback, wire, true, malformed), wire);
}

TEST(SdkPositionCodecTest, EveryJointEncoderCountRoundTripsWithoutRatcheting)
{
    size_t legacy_losses = 0;
    for (uint16_t count = 0; count < 4096; ++count) {
        const double q = sdk_joint_position_decode(count);
        const int legacy = static_cast<int>((q * 180.0 / M_PI + 180.0) / 360.0 * 4096.0);
        if (legacy != count) ++legacy_losses;
        ASSERT_EQ(sdk_joint_position_encode(q), count) << count;
    }
    EXPECT_GT(legacy_losses, 0u);  // Regression reproduces the old truncation bug.
    EXPECT_EQ(sdk_joint_position_encode(-4.0), 0);
    EXPECT_EQ(sdk_joint_position_encode(4.0), 4095);
}

TEST(SdkPositionCodecTest, EveryGripperCountRoundTripsAndKeepsBounds)
{
    for (uint16_t count = 0; count <= 1000; ++count) {
        ASSERT_EQ(sdk_gripper_position_encode(sdk_gripper_position_decode(count)), count);
    }
    EXPECT_EQ(sdk_gripper_position_encode(-1.0), 0);
    EXPECT_EQ(sdk_gripper_position_encode(3.0), 1000);
}

TEST(SdkPositionCodecTest, OwnershipHoldPreservesExactTransmittedCountsNotEncoders)
{
    std::vector<uint8_t> frame(34, 0);
    frame[0] = 0xAA; frame[1] = 0x06; frame[2] = 0x03; frame[3] = 0x1C; frame[33] = 0xFF;
    const std::vector<uint16_t> counts{18, 19, 45, 2423, 1875, 2045, 995};
    for (size_t i = 0; i < 7; ++i) {
        frame[4 + 4*i] = counts[i] & 0xFF;
        frame[5 + 4*i] = counts[i] >> 8;
    }
    auto hold = joints(); // Stale/fresh encoders cannot change a valid wire hold.
    double gripper = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, hold, gripper));
    for (size_t i = 0; i < 6; ++i) EXPECT_EQ(sdk_joint_position_encode(hold[i]), counts[i]);
    EXPECT_EQ(sdk_gripper_position_encode(gripper), 995);
    // Both directions and repeated mode toggles preserve the same counts.
    for (int toggle = 0; toggle < 20; ++toggle) {
        ASSERT_TRUE(decode_retained_sdk_hold(frame, hold, gripper));
        for (size_t i = 0; i < 6; ++i) EXPECT_EQ(sdk_joint_position_encode(hold[i]), counts[i]);
    }
}

TEST(SdkPositionCodecTest, NoPriorOrInvalidWireFrameCannotAuthorizeAnOwnershipStream)
{
    auto held = joints(.5);
    double gripper = .7;
    EXPECT_FALSE(decode_retained_sdk_hold({}, held, gripper));
    EXPECT_EQ(held, joints(.5));
    EXPECT_DOUBLE_EQ(gripper, .7);
    EXPECT_FALSE(decode_retained_sdk_hold(std::vector<uint8_t>(34, 0), held, gripper));
}

TEST(EndpointTrimContinuityTest, SerializesLiveCorrectionAndDelaysLeaseRelease)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.max_step_rad = degrees(2.0);
    config.max_total_trim_rad = degrees(4.0);
    EndpointTrimContinuity trim(config);
    const std::vector<double> reference =
        joints(0.0, 0.555, -0.262, 0.0, -0.324, 0.046);
    const std::vector<double> first_measured =
        joints(0.0, 0.555 - degrees(1.3), -0.262, 0.0, -0.324, 0.046);
    const std::vector<double> second_measured =
        joints(0.0, 0.555 - degrees(1.0), -0.262, 0.0, -0.324, 0.046);

    trim.activate(reference, joints(), 10.0);
    const EndpointTrimDecision first = trim.request_correction(
        first_measured, all_stable_joints(), 1.0, 10.01
    );
    ASSERT_EQ(first.phase, EndpointTrimPhase::WAITING_RESPONSE);
    EXPECT_NEAR(first.applied_step[1], degrees(1.3), 1e-12);

    EndpointTrimContinuity hypothetical(config);
    hypothetical.activate(reference, first.offsets, 10.01);
    const EndpointTrimDecision would_reach = hypothetical.request_correction(
        second_measured, all_stable_joints(), 1.0, 10.02
    );
    EXPECT_NEAR(
        would_reach.composed_target[1] - reference[1],
        degrees(2.3),
        1e-12
    );

    const EndpointTrimDecision second = trim.request_correction(
        second_measured, all_stable_joints(), 1.0, 10.02
    );
    EXPECT_EQ(second.phase, EndpointTrimPhase::WAITING_RESPONSE);
    EXPECT_FALSE(second.command_changed);
    EXPECT_EQ(second.code, "ENDPOINT_TRIM_RESPONSE_OUTSTANDING");
    expect_vectors_near(second.composed_target, first.composed_target, 1e-12);

    const EndpointTrimDecision release = trim.request_release(
        first_measured, true, 10.38
    );
    EXPECT_EQ(release.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_FALSE(release.release_completed);
    EXPECT_EQ(release.applied_step, joints());
    expect_vectors_near(release.composed_target, first.composed_target, 1e-12);
}

TEST(EndpointTrimContinuityTest, ClampsEveryAppliedCorrectionStep)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(1.0, -1.0, 1.0, -1.0, 1.0, -1.0), joints(), 0.0);
    const EndpointTrimDecision decision = trim.request_correction(
        joints(), all_stable_joints(), 1.0, 0.01
    );

    ASSERT_EQ(decision.phase, EndpointTrimPhase::WAITING_RESPONSE);
    for (const double step : decision.applied_step) {
        EXPECT_LE(std::abs(step), config.max_step_rad);
    }
}

TEST(EndpointTrimContinuityTest, RequiresNewDirectionalFeedbackForStableResponse)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.settle_error_rad = 10.0 * config.sdk_quantum_rad;
    EndpointTrimContinuity trim(config);
    const std::vector<double> reference = joints();
    const std::vector<double> baseline =
        joints(-2.0 * config.response_min_rad);

    trim.activate(reference, joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 1.0
    );
    ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);

    EXPECT_EQ(
        trim.note_feedback(
            joints(-3.0 * config.response_min_rad), 1.1, 1.1
        ).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(joints(), 1.0, 1.2).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(joints(), 1.3, 1.3).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(joints(), 1.3 + config.stable_sec, 1.3 + config.stable_sec)
            .phase,
        EndpointTrimPhase::ACTIVE_READY
    );
}

TEST(EndpointTrimContinuityTest, DeployedResponseMinimumRejectsOneQuantumUntilTwoQuantumResponseSettles)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.sdk_quantum_rad = 2.0 * M_PI / 4096.0;
    config.max_step_rad = 8.0 * M_PI / 4096.0;
    config.response_min_rad = 4.0 * M_PI / 4096.0;
    config.response_deadline_sec = 1.0;
    config.stable_sec = 0.30;
    config.settle_error_rad = 4.0 * M_PI / 4096.0;
    EndpointTrimContinuity trim(config);

    const double one_quantum = config.response_min_rad / 2.0;
    const std::vector<double> baseline = joints(-config.response_min_rad);
    const std::vector<double> one_quantum_response =
        joints(-config.response_min_rad + one_quantum);
    const std::vector<double> two_quantum_response = joints();

    trim.activate(joints(), joints(-one_quantum), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 1.0
    );
    ASSERT_EQ(
        correction.phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    ASSERT_LE(
        std::abs(one_quantum_response[0] - correction.composed_target[0]),
        config.settle_error_rad
    );

    EXPECT_EQ(
        trim.note_feedback(one_quantum_response, 1.1, 1.1).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(one_quantum_response, 1.41, 1.41).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(two_quantum_response, 1.5, 1.5).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(two_quantum_response, 1.79, 1.79).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(two_quantum_response, 1.81, 1.81).phase,
        EndpointTrimPhase::ACTIVE_READY
    );
}

TEST(EndpointTrimContinuityTest,
     EverySdkTwoCountResponseSettlesInBothDirections)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.max_step_rad = 4.0 * config.sdk_quantum_rad;
    config.response_min_rad = 2.0 * config.sdk_quantum_rad;
    config.settle_error_rad = 2.0 * config.sdk_quantum_rad;
    for (int lower_count = 0; lower_count < 4094; ++lower_count) {
        for (const int direction : {-1, 1}) {
            const int baseline_count = lower_count + (direction < 0 ? 2 : 0);
            const int response_count = baseline_count + 2 * direction;
            SCOPED_TRACE(::testing::Message()
                         << "SDK baseline=" << baseline_count
                         << " response=" << response_count);
            const std::vector<double> baseline =
                joints(sdk_joint_position_decode(baseline_count));
            const std::vector<double> response =
                joints(sdk_joint_position_decode(response_count));
            EndpointTrimContinuity trim(config);
            trim.activate(response, joints(), 0.0);
            ASSERT_EQ(
                trim.request_correction(baseline, all_stable_joints(), 1.0, 1.0)
                    .phase,
                EndpointTrimPhase::WAITING_RESPONSE
            );
            ASSERT_EQ(
                trim.note_feedback(response, 1.1, 1.1).phase,
                EndpointTrimPhase::WAITING_RESPONSE
            );
            ASSERT_EQ(
                trim.note_feedback(response, 1.41, 1.41).phase,
                EndpointTrimPhase::ACTIVE_READY
            );
        }
    }
}

TEST(EndpointTrimContinuityTest,
     EverySdkOneCountResponseIsRejectedInBothDirections)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.response_min_rad = 2.0 * config.sdk_quantum_rad;
    for (int lower_count = 0; lower_count < 4095; ++lower_count) {
        for (const int direction : {-1, 1}) {
            const int baseline_count = lower_count + (direction < 0 ? 1 : 0);
            const int response_count = baseline_count + direction;
            SCOPED_TRACE(::testing::Message()
                         << "SDK baseline=" << baseline_count
                         << " response=" << response_count);
            const std::vector<double> baseline =
                joints(sdk_joint_position_decode(baseline_count));
            const std::vector<double> response =
                joints(sdk_joint_position_decode(response_count));
            EndpointTrimContinuity trim(config);
            trim.activate(response, joints(), 0.0);
            ASSERT_EQ(
                trim.request_correction(baseline, all_stable_joints(), 1.0, 1.0)
                    .phase,
                EndpointTrimPhase::WAITING_RESPONSE
            );
            ASSERT_EQ(
                trim.note_feedback(response, 1.1, 1.1).phase,
                EndpointTrimPhase::WAITING_RESPONSE
            );
            ASSERT_EQ(
                trim.note_feedback(response, 1.41, 1.41).phase,
                EndpointTrimPhase::WAITING_RESPONSE
            );
            ASSERT_EQ(trim.update(2.0).phase, EndpointTrimPhase::FAULT);
            ASSERT_EQ(trim.state().code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
        }
    }
}

TEST(EndpointTrimContinuityTest,
     EverySdkTwoCountRequestRejectsZeroAndOppositeDirectionalFeedback)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.response_min_rad = 2.0 * config.sdk_quantum_rad;
    for (int lower_count = 0; lower_count < 4094; ++lower_count) {
        for (const int direction : {-1, 1}) {
            const int baseline_count = lower_count + (direction < 0 ? 2 : 0);
            const std::vector<double> baseline =
                joints(sdk_joint_position_decode(baseline_count));
            const std::vector<double> reference = joints(
                sdk_joint_position_decode(baseline_count + 2 * direction)
            );
            for (const int opposite_count_delta : {0, 1, 2}) {
                const int response_count =
                    baseline_count - direction * opposite_count_delta;
                if (response_count < 0 || response_count > 4095) {
                    continue;
                }
                SCOPED_TRACE(::testing::Message()
                             << "SDK baseline=" << baseline_count
                             << " direction=" << direction
                             << " response=" << response_count);
                const std::vector<double> response =
                    joints(sdk_joint_position_decode(response_count));
                EndpointTrimContinuity trim(config);
                trim.activate(reference, joints(), 0.0);
                ASSERT_EQ(
                    trim.request_correction(
                        baseline, all_stable_joints(), 1.0, 1.0
                    ).phase,
                    EndpointTrimPhase::WAITING_RESPONSE
                );
                ASSERT_EQ(
                    trim.note_feedback(response, 1.1, 1.1).phase,
                    EndpointTrimPhase::WAITING_RESPONSE
                );
                ASSERT_EQ(
                    trim.note_feedback(response, 1.41, 1.41).phase,
                    EndpointTrimPhase::WAITING_RESPONSE
                );
            }
        }
    }
}

TEST(EndpointTrimContinuityTest,
     ResponseRoundoffDoesNotAdmitResolvableSubthresholdDisplacement)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.response_min_rad = 2.0 * config.sdk_quantum_rad;
    const double shortfall_rad = 1e-8 * config.sdk_quantum_rad;
    for (const int direction : {-1, 1}) {
        const std::vector<double> baseline = joints(sdk_joint_position_decode(1000));
        const std::vector<double> reference = joints(
            baseline[0] + direction * config.response_min_rad
        );
        const std::vector<double> response = joints(
            reference[0] - direction * shortfall_rad
        );
        EndpointTrimContinuity trim(config);
        trim.activate(reference, joints(), 0.0);
        ASSERT_EQ(
            trim.request_correction(baseline, all_stable_joints(), 1.0, 1.0).phase,
            EndpointTrimPhase::WAITING_RESPONSE
        );
        ASSERT_EQ(
            trim.note_feedback(response, 1.1, 1.1).phase,
            EndpointTrimPhase::WAITING_RESPONSE
        );
        ASSERT_EQ(
            trim.note_feedback(response, 1.41, 1.41).phase,
            EndpointTrimPhase::WAITING_RESPONSE
        );
    }
}

TEST(EndpointTrimContinuityTest,
     DeployedIncrementResponseConvergesFixedBiasWithoutReversal)
{
    EndpointTrimConfig config = endpoint_trim_config();
    config.sdk_quantum_rad = 2.0 * M_PI / 4096.0;
    config.max_step_rad = 4.0 * config.sdk_quantum_rad;
    config.response_min_rad = 2.0 * config.sdk_quantum_rad;
    config.settle_error_rad = 2.0 * config.sdk_quantum_rad;
    config.stable_sec = 0.30;
    config.response_deadline_sec = 1.0;
    EndpointTrimContinuity trim(config);
    const double quantum = config.sdk_quantum_rad;
    const std::vector<double> reference = joints();
    const double fixed_bias = degrees(1.3);
    std::vector<double> measured = joints(-fixed_bias);
    double previous_residual = fixed_bias;

    trim.activate(reference, joints(), 0.0);
    std::vector<double> final_target;
    for (size_t iteration = 0; iteration < 4; ++iteration) {
        const double request_time = 1.0 + 0.6 * iteration;
        const EndpointTrimDecision correction = trim.request_correction(
            measured, all_stable_joints(), 1.0, request_time
        );
        ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);
        ASSERT_TRUE(correction.command_changed);
        EXPECT_GT(correction.applied_step[0], 0.0);
        EXPECT_LE(correction.applied_step[0], 4.0 * quantum + 1e-12);
        if (iteration < 3) {
            EXPECT_NEAR(correction.applied_step[0], 4.0 * quantum, 1e-12);
        } else {
            EXPECT_NEAR(
                correction.applied_step[0], fixed_bias - 12.0 * quantum, 1e-12
            );
        }
        for (size_t joint = 1; joint < correction.applied_step.size(); ++joint) {
            EXPECT_EQ(correction.applied_step[joint], 0.0);
        }

        measured = joints(correction.composed_target[0] - fixed_bias);
        const double residual = std::abs(reference[0] - measured[0]);
        EXPECT_LT(residual, previous_residual);
        previous_residual = residual;
        final_target = correction.composed_target;

        const double settle_start = request_time + 0.05;
        EXPECT_EQ(
            trim.note_feedback(measured, settle_start, settle_start).phase,
            EndpointTrimPhase::WAITING_RESPONSE
        );
        EXPECT_EQ(
            trim.note_feedback(
                measured,
                settle_start + config.stable_sec - 0.01,
                settle_start + config.stable_sec - 0.01
            ).phase,
            EndpointTrimPhase::WAITING_RESPONSE
        );
        EXPECT_EQ(
            trim.note_feedback(
                measured,
                settle_start + config.stable_sec + 0.01,
                settle_start + config.stable_sec + 0.01
            ).phase,
            EndpointTrimPhase::ACTIVE_READY
        );
    }

    EXPECT_LE(std::abs(measured[0] - reference[0]), 1e-12);
    const EndpointTrimDecision next = trim.request_correction(
        measured, all_stable_joints(), 1.0, 3.5
    );
    EXPECT_EQ(next.phase, EndpointTrimPhase::ACTIVE_READY);
    EXPECT_FALSE(next.command_changed);
    EXPECT_EQ(next.applied_step, joints());
    EXPECT_EQ(next.composed_target, final_target);
}

TEST(EndpointTrimContinuityTest, PendingReleaseCompletesAfterStableResponse)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> reference = joints();
    const std::vector<double> baseline =
        joints(-2.0 * config.response_min_rad);

    trim.activate(reference, joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    EXPECT_EQ(
        trim.request_release(baseline, true, 0.11).phase,
        EndpointTrimPhase::PENDING_RELEASE
    );
    trim.note_feedback(joints(), 0.2, 0.2);
    const EndpointTrimDecision completed =
        trim.note_feedback(joints(), 0.2 + config.stable_sec, 0.2 + config.stable_sec);

    EXPECT_TRUE(completed.release_completed);
    EXPECT_EQ(completed.phase, EndpointTrimPhase::QUIESCENT);
    for (size_t i = 0; i < preserved.size(); ++i) {
        EXPECT_LE(
            std::abs(completed.reference[i] + completed.offsets[i] - preserved[i]),
            config.sdk_quantum_rad
        );
    }
}

TEST(EndpointTrimContinuityTest, RepeatedReleaseRequestRemainsPending)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> baseline =
        joints(-2.0 * config.response_min_rad);

    trim.activate(joints(), joints(), 0.0);
    trim.request_correction(baseline, all_stable_joints(), 1.0, 0.1);
    const EndpointTrimDecision first = trim.request_release(
        baseline, true, 0.2
    );
    const EndpointTrimDecision repeated = trim.request_release(
        joints(), true, 0.3
    );

    ASSERT_EQ(first.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_EQ(repeated.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_FALSE(repeated.release_completed);
    EXPECT_EQ(repeated.applied_step, joints());
    EXPECT_EQ(repeated.composed_target, first.composed_target);
}

TEST(EndpointTrimContinuityTest, PendingReleaseTimesOutWithoutChangingTarget)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> reference = joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
    const std::vector<double> offsets = joints(0.01, -0.01, 0.01, -0.01, 0.01, -0.01);

    trim.activate(reference, offsets, 0.0);
    const EndpointTrimDecision correction =
        trim.request_correction(joints(), all_stable_joints(), 1.0, 0.1);
    const std::vector<double> preserved = correction.composed_target;
    trim.request_release(joints(), false, 0.2);
    const EndpointTrimDecision timed_out =
        trim.update(0.1 + config.response_deadline_sec);

    EXPECT_TRUE(timed_out.release_completed);
    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_EQ(timed_out.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(timed_out.reference, preserved);
    EXPECT_EQ(timed_out.offsets, joints());
    EXPECT_EQ(timed_out.composed_target, preserved);
}

TEST(EndpointTrimContinuityTest, TimeoutIgnoresFreshReleaseFeedback)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> reference =
        joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);

    trim.activate(reference, joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    trim.request_release(joints(), true, 0.2);
    const EndpointTrimDecision timed_out = trim.update(
        0.1 + config.response_deadline_sec
    );

    EXPECT_TRUE(timed_out.release_completed);
    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_EQ(timed_out.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(timed_out.reference, preserved);
    EXPECT_EQ(timed_out.offsets, joints());
    EXPECT_EQ(timed_out.composed_target, preserved);
}

TEST(EndpointTrimContinuityTest, TimeoutWithoutReleasePermanentlyBlocksCorrection)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(-config.max_step_rad), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    const EndpointTrimDecision timed_out = trim.update(
        0.1 + config.response_deadline_sec
    );
    const EndpointTrimDecision retry = trim.request_correction(
        joints(-2.0 * config.max_step_rad),
        all_stable_joints(),
        1.0,
        0.2 + config.response_deadline_sec
    );
    const EndpointTrimDecision reactivated = trim.activate(
        joints(0.5), joints(), 0.3 + config.response_deadline_sec
    );

    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(timed_out.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(timed_out.composed_target, preserved);
    EXPECT_EQ(retry.phase, EndpointTrimPhase::FAULT);
    EXPECT_FALSE(retry.command_changed);
    EXPECT_EQ(retry.composed_target, preserved);
    EXPECT_EQ(reactivated.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(reactivated.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_FALSE(reactivated.command_changed);
    EXPECT_EQ(reactivated.composed_target, preserved);
}

TEST(EndpointTrimContinuityTest,
     TaskHandoffCannotClearStickyFaultButExplicitGuiHandoffCan)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(-config.max_step_rad), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    const EndpointTrimDecision timed_out = trim.update(
        0.1 + config.response_deadline_sec
    );
    ASSERT_EQ(timed_out.phase, EndpointTrimPhase::FAULT);

    const EndpointTrimDecision task_handoff = trim.task_controller_handoff(
        joints(0.5), 0.2 + config.response_deadline_sec
    );
    EXPECT_EQ(task_handoff.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(task_handoff.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_FALSE(task_handoff.command_changed);
    EXPECT_EQ(task_handoff.applied_step, joints());
    EXPECT_EQ(task_handoff.composed_target, preserved);

    const std::vector<double> gui_target =
        joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
    const EndpointTrimDecision gui_handoff = trim.explicit_gui_handoff(
        gui_target, 0.3 + config.response_deadline_sec
    );
    EXPECT_EQ(gui_handoff.phase, EndpointTrimPhase::IDLE);
    EXPECT_TRUE(gui_handoff.command_changed);
    EXPECT_TRUE(gui_handoff.code.empty());
    EXPECT_EQ(gui_handoff.reference, gui_target);
    EXPECT_EQ(gui_handoff.offsets, joints());
    EXPECT_EQ(gui_handoff.composed_target, gui_target);
}

TEST(EndpointTrimContinuityTest, MalformedTaskHandoffFailsClosed)
{
    EndpointTrimContinuity trim(endpoint_trim_config());
    trim.explicit_gui_handoff(joints(0.2), 0.0);
    const EndpointTrimDecision malformed = trim.task_controller_handoff(
        std::vector<double>{0.1}, 0.1
    );

    EXPECT_EQ(malformed.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(malformed.code, "ENDPOINT_TRIM_CONTINUITY_VIOLATION");
    EXPECT_FALSE(malformed.command_changed);
}

TEST(EndpointTrimContinuityTest, ActivateCannotBypassOutstandingResponseOrRelease)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(-config.max_step_rad), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    const EndpointTrimDecision waiting_activate = trim.activate(
        joints(0.5), joints(), 0.2
    );

    EXPECT_EQ(waiting_activate.phase, EndpointTrimPhase::WAITING_RESPONSE);
    EXPECT_FALSE(waiting_activate.command_changed);
    EXPECT_EQ(
        waiting_activate.code,
        "ENDPOINT_TRIM_RESPONSE_OUTSTANDING"
    );
    EXPECT_EQ(waiting_activate.composed_target, preserved);

    ASSERT_EQ(
        trim.request_release(joints(), false, 0.3).phase,
        EndpointTrimPhase::PENDING_RELEASE
    );
    const EndpointTrimDecision pending_activate = trim.activate(
        joints(-0.5), joints(), 0.4
    );
    EXPECT_EQ(pending_activate.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_FALSE(pending_activate.command_changed);
    EXPECT_EQ(pending_activate.composed_target, preserved);

    const EndpointTrimDecision timed_out = trim.update(
        0.1 + config.response_deadline_sec
    );
    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(timed_out.release_completed);
    EXPECT_EQ(timed_out.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(timed_out.composed_target, preserved);
}

TEST(EndpointTrimContinuityTest, FeedbackProcessedAfterDeadlineCannotSettle)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(-config.max_step_rad), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = correction.composed_target;
    trim.request_release(joints(), false, 0.15);
    trim.note_feedback(correction.composed_target, 0.2, 0.2);
    const EndpointTrimDecision late = trim.note_feedback(
        correction.composed_target,
        0.2 + config.stable_sec,
        0.2 + config.response_deadline_sec
    );

    EXPECT_EQ(late.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(late.release_completed);
    EXPECT_EQ(late.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(late.reference, preserved);
    EXPECT_EQ(late.offsets, joints());
    EXPECT_EQ(late.composed_target, preserved);
}

TEST(EndpointTrimContinuityTest, SettledFeedbackIsNotFreshForLaterRelease)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> baseline = joints(-config.max_step_rad);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> settled = joints();
    trim.note_feedback(settled, 0.2, 0.2);
    ASSERT_EQ(
        trim.note_feedback(
            settled,
            0.2 + config.stable_sec,
            0.2 + config.stable_sec
        ).phase,
        EndpointTrimPhase::ACTIVE_READY
    );

    const EndpointTrimDecision released = trim.request_release(
        joints(-0.05), false, 0.6
    );

    EXPECT_EQ(released.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(released.release_completed);
    EXPECT_EQ(released.reference, correction.composed_target);
    EXPECT_EQ(released.offsets, joints());
    EXPECT_EQ(released.composed_target, correction.composed_target);
}

TEST(EndpointTrimContinuityTest, SdkQuantizationMatchesDriverFormula)
{
    EndpointTrimConfig config = endpoint_trim_config();

    for (int milliradians = -120; milliradians <= 120; ++milliradians) {
        SCOPED_TRACE(milliradians);
        const double offset = static_cast<double>(milliradians) / 1000.0;
        EndpointTrimContinuity trim(config);
        trim.activate(
            std::vector<double>(config.joint_count, offset),
            joints(),
            0.0
        );
        const EndpointTrimDecision released = trim.request_release(
            joints(), true, 0.1
        );

        ASSERT_EQ(released.phase, EndpointTrimPhase::QUIESCENT);
        ASSERT_EQ(released.offsets.size(), config.joint_count);
        for (const double actual : released.offsets) {
            EXPECT_NEAR(actual, driver_sdk_quantize(offset), 1e-12);
        }
    }
}

TEST(EndpointTrimContinuityTest, RejectsMalformedOrNonfiniteInput)
{
    EndpointTrimContinuity wrong_size(endpoint_trim_config());
    const EndpointTrimDecision malformed = wrong_size.activate(
        std::vector<double>{0.0, 0.1}, joints(), 0.0
    );
    EXPECT_EQ(malformed.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(malformed.code, "ENDPOINT_TRIM_CONTINUITY_VIOLATION");

    EndpointTrimContinuity nonfinite(endpoint_trim_config());
    const EndpointTrimDecision invalid = nonfinite.activate(
        joints(),
        joints(0.0, std::numeric_limits<double>::infinity()),
        0.0
    );
    EXPECT_EQ(invalid.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(invalid.code, "ENDPOINT_TRIM_CONTINUITY_VIOLATION");

    EndpointTrimContinuity malformed_correction(endpoint_trim_config());
    malformed_correction.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision wrong_mask =
        malformed_correction.request_correction(
            joints(), std::vector<uint8_t>{1, 1}, 1.0, 0.1
        );
    EXPECT_EQ(wrong_mask.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(wrong_mask.code, "ENDPOINT_TRIM_CONTINUITY_VIOLATION");

    EndpointTrimContinuity nonfinite_feedback(endpoint_trim_config());
    nonfinite_feedback.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision bad_feedback = nonfinite_feedback.note_feedback(
        joints(0.0, std::numeric_limits<double>::quiet_NaN()), 0.1, 0.1
    );
    EXPECT_EQ(bad_feedback.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(bad_feedback.code, "ENDPOINT_TRIM_CONTINUITY_VIOLATION");
}

TEST(EndpointTrimContinuityTest, SaturatesFiniteCorrectionArithmetic)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const double maximum = std::numeric_limits<double>::max();

    trim.activate(joints(maximum), joints(), 0.0);
    const EndpointTrimDecision decision = trim.request_correction(
        joints(-maximum), all_stable_joints(), 1.0, 0.1
    );

    ASSERT_EQ(decision.phase, EndpointTrimPhase::ACTIVE_READY);
    EXPECT_TRUE(std::isfinite(decision.composed_target[0]));
    EXPECT_EQ(decision.composed_target[0], maximum);
    EXPECT_EQ(decision.applied_step[0], config.max_step_rad);
}

TEST(EndpointTrimContinuityTest, ExplicitGuiHandoffClearsTaskTrimAndPreservesTarget)
{
    EndpointTrimContinuity trim(endpoint_trim_config());
    trim.activate(joints(), joints(), 0.0);
    trim.request_correction(
        joints(-0.02), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> gui_target =
        joints(0.4, -0.3, 0.2, -0.1, 0.05, -0.04);

    const EndpointTrimDecision handoff = trim.explicit_gui_handoff(gui_target, 0.2);

    EXPECT_EQ(handoff.phase, EndpointTrimPhase::IDLE);
    EXPECT_EQ(handoff.reference, gui_target);
    EXPECT_EQ(handoff.offsets, joints());
    EXPECT_EQ(handoff.composed_target, gui_target);
    EXPECT_TRUE(handoff.command_changed);
}

TEST(EndpointTrimDriverAdmissionTest,
     RepeatedControllerCommandAfterReleasePreservesComposedTarget)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    EndpointTrimCommandOrder order(config.joint_count, config.sdk_quantum_rad);
    const std::vector<double> upstream =
        joints(0.10, 0.20, 0.30, 0.40, 0.50, 0.60);

    ASSERT_TRUE(order.observe_upstream_command(
        upstream,
        EndpointTrimCommandSource::TASK_CONTROLLER
    ));
    trim.task_controller_handoff(order.upstream_target(), 0.0);
    order.mark_command_applied();
    const EndpointTrimDecision active = trim.activate(
        upstream,
        joints(0.02, -0.02, 0.02, -0.02, 0.02, -0.02),
        0.1
    );
    const std::vector<double> preserved = active.composed_target;
    const std::vector<double> release_feedback =
        joints(0.09, 0.19, 0.29, 0.39, 0.49, 0.59);
    const EndpointTrimDecision released = trim.request_release(
        release_feedback, true, 0.2
    );
    order.note_release();

    ASSERT_EQ(released.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_EQ(released.reference, release_feedback);
    expect_vectors_near(
        released.composed_target,
        preserved,
        config.sdk_quantum_rad
    );
    const std::vector<double> rebased_target = released.composed_target;
    EXPECT_FALSE(order.observe_upstream_command(
        upstream,
        EndpointTrimCommandSource::TASK_CONTROLLER
    ));
    EXPECT_FALSE(order.newer_task_command_requires_handoff());
    EXPECT_EQ(trim.state().composed_target, rebased_target);
}

TEST(EndpointTrimDriverAdmissionTest,
     NewGuiCommandAfterServiceReleaseIsAuthoritative)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    EndpointTrimCommandOrder order(config.joint_count, config.sdk_quantum_rad);
    const std::vector<double> task_target = joints(0.1);
    const std::vector<double> gui_target =
        joints(-0.4, 0.3, -0.2, 0.1, -0.05, 0.04);

    order.observe_upstream_command(
        task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER
    );
    trim.task_controller_handoff(order.upstream_target(), 0.0);
    order.mark_command_applied();
    trim.activate(task_target, joints(0.01), 0.1);
    trim.request_release(joints(0.1), true, 0.2);
    order.note_release();
    const uint64_t release_generation = order.release_generation();

    ASSERT_TRUE(order.observe_upstream_command(
        gui_target,
        EndpointTrimCommandSource::GUI_DIRECT_EDIT,
        0.3
    ));
    EXPECT_GT(order.command_generation(), release_generation);
    ASSERT_TRUE(order.newer_explicit_gui_command_requires_handoff());
    const EndpointTrimDecision handoff = trim.explicit_gui_handoff(
        order.upstream_target(),
        0.3
    );
    order.mark_command_applied();

    EXPECT_EQ(handoff.phase, EndpointTrimPhase::IDLE);
    EXPECT_EQ(handoff.composed_target, gui_target);
    EXPECT_FALSE(order.newer_explicit_gui_command_requires_handoff());
}

TEST(EndpointTrimDriverAdmissionTest,
     RetainedStateResetCannotSelectPriorTrimTarget)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    EndpointTrimCommandOrder order(config.joint_count, config.sdk_quantum_rad);
    const std::vector<double> upstream = joints(0.1);
    const std::vector<double> safe_fallback = joints(-0.2);

    order.observe_upstream_command(
        upstream,
        EndpointTrimCommandSource::TASK_CONTROLLER
    );
    trim.activate(upstream, joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(0.08), all_stable_joints(), 1.0, 0.1
    );
    ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);
    ASSERT_NE(correction.composed_target, safe_fallback);

    trim = EndpointTrimContinuity(config);
    order.reset();

    EXPECT_EQ(trim.state().phase, EndpointTrimPhase::IDLE);
    EXPECT_TRUE(trim.state().composed_target.empty());
    EXPECT_EQ(
        endpoint_trim_stream_target(safe_fallback, trim.state()),
        safe_fallback
    );
    EXPECT_FALSE(order.has_unapplied_command());
}

TEST(EndpointTrimDriverAdmissionTest,
     BlockedTransmissionCannotAdmitCorrectionOrMutateTarget)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    const std::vector<EndpointTrimTransmissionGate> blocked = {
        {false, false, false, true, true, false},
        {true, true, false, true, true, false},
        {true, false, true, true, true, false},
        {true, false, false, true, false, true},
        {true, false, false, true, true, true},
    };

    for (const EndpointTrimTransmissionGate& gate : blocked) {
        EndpointTrimContinuity trim(config);
        const EndpointTrimDecision active =
            trim.activate(joints(0.1), joints(), 0.0);
        const std::vector<double> before = active.composed_target;
        if (gate.allows_correction()) {
            trim.request_correction(
                joints(0.08), all_stable_joints(), 1.0, 0.1
            );
        }
        EXPECT_EQ(trim.state().phase, EndpointTrimPhase::ACTIVE_READY);
        EXPECT_EQ(trim.state().composed_target, before);
    }
}

TEST(EndpointTrimDriverAdmissionTest,
     FailedWriteAfterAdmissionFailsClosedWithoutTargetReversal)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    trim.activate(joints(0.1), joints(), 0.0);
    const EndpointTrimDecision admitted = trim.request_correction(
        joints(0.08), all_stable_joints(), 1.0, 0.1
    );
    const std::vector<double> preserved = admitted.composed_target;

    // Model write_raw_frame() == false: no feedback acknowledgement arrives.
    const EndpointTrimDecision timed_out =
        trim.update(0.1 + config.response_deadline_sec);
    const EndpointTrimDecision retry = trim.request_correction(
        joints(0.06), all_stable_joints(), 1.0,
        0.2 + config.response_deadline_sec
    );

    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(timed_out.composed_target, preserved);
    EXPECT_EQ(retry.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(retry.composed_target, preserved);
    EXPECT_FALSE(retry.command_changed);
}

TEST(EndpointTrimOwnershipTransitionTest,
     ServiceReleaseDuringWaitingResponseStaysPendingUntilSettled)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> baseline = joints(-config.max_step_rad);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 0.1
    );
    ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);
    const std::vector<double> preserved = correction.composed_target;

    const EndpointTrimDecision pending = trim.request_release(
        baseline, true, 0.15
    );
    EXPECT_EQ(pending.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_FALSE(pending.release_completed);
    EXPECT_EQ(pending.composed_target, preserved);

    trim.note_feedback(joints(), 0.2, 0.2);
    const EndpointTrimDecision settled = trim.note_feedback(
        joints(),
        0.2 + config.stable_sec,
        0.2 + config.stable_sec
    );
    EXPECT_EQ(settled.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(settled.release_completed);
    EXPECT_TRUE(settled.code.empty());
    expect_vectors_near(
        settled.composed_target,
        preserved,
        config.sdk_quantum_rad
    );
}

TEST(EndpointTrimOwnershipTransitionTest,
     LeaseExpiryDuringWaitingResponseUsesPendingReleasePath)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);

    trim.activate(joints(0.1), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        joints(0.1 - config.max_step_rad),
        all_stable_joints(),
        1.0,
        0.1
    );
    ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);

    // Lease expiry is deliberately the same coordinator request as service
    // release; the node source contract verifies that shared routing.
    const EndpointTrimDecision pending = trim.request_release(
        joints(), false, 0.2
    );
    EXPECT_EQ(pending.phase, EndpointTrimPhase::PENDING_RELEASE);
    EXPECT_FALSE(pending.release_completed);
    EXPECT_EQ(pending.composed_target, correction.composed_target);

    const EndpointTrimDecision timed_out = trim.update(
        0.1 + config.response_deadline_sec
    );
    EXPECT_EQ(timed_out.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(timed_out.release_completed);
    EXPECT_EQ(timed_out.code, "ENDPOINT_TRIM_RESPONSE_TIMEOUT");
    EXPECT_EQ(timed_out.composed_target, correction.composed_target);
}

TEST(EndpointTrimOwnershipTransitionTest,
     StaleReleaseFeedbackPreservesTheComposedTarget)
{
    EndpointTrimContinuity trim(endpoint_trim_config());
    const EndpointTrimDecision active = trim.activate(
        joints(0.1), joints(0.01), 0.0
    );

    const EndpointTrimDecision released = trim.request_release(
        joints(-0.4), false, 0.1
    );

    EXPECT_EQ(released.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(released.release_completed);
    EXPECT_EQ(released.reference, active.composed_target);
    EXPECT_EQ(released.offsets, joints());
    EXPECT_EQ(released.composed_target, active.composed_target);
}

TEST(EndpointTrimOwnershipTransitionTest,
     FreshReleaseFeedbackRebasesWithoutACommandJump)
{
    const EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const EndpointTrimDecision active = trim.activate(
        joints(0.1), joints(0.01), 0.0
    );
    const std::vector<double> measured = joints(0.095);

    const EndpointTrimDecision released = trim.request_release(
        measured, true, 0.1
    );

    EXPECT_EQ(released.phase, EndpointTrimPhase::QUIESCENT);
    EXPECT_TRUE(released.release_completed);
    EXPECT_TRUE(released.code.empty());
    EXPECT_EQ(released.reference, measured);
    expect_vectors_near(
        released.composed_target,
        active.composed_target,
        config.sdk_quantum_rad
    );
}

TEST(EndpointTrimOwnershipTransitionTest,
     ExplicitGuiCommandDuringPendingReleaseIsImmediatelyAuthoritative)
{
    EndpointTrimContinuity trim(endpoint_trim_config());
    trim.activate(joints(0.1), joints(), 0.0);
    trim.request_correction(
        joints(0.08), all_stable_joints(), 1.0, 0.1
    );
    ASSERT_EQ(
        trim.request_release(joints(0.08), true, 0.15).phase,
        EndpointTrimPhase::PENDING_RELEASE
    );
    const std::vector<double> gui_target =
        joints(-0.4, 0.3, -0.2, 0.1, -0.05, 0.04);

    const EndpointTrimDecision handoff =
        trim.explicit_gui_handoff(gui_target, 0.2);

    EXPECT_EQ(handoff.phase, EndpointTrimPhase::IDLE);
    EXPECT_TRUE(handoff.command_changed);
    EXPECT_EQ(handoff.reference, gui_target);
    EXPECT_EQ(handoff.offsets, joints());
    EXPECT_EQ(handoff.composed_target, gui_target);
    EXPECT_FALSE(handoff.release_completed);
    EXPECT_TRUE(handoff.code.empty());
}

TEST(EndpointTrimDriverOrchestrationTest,
     ExplicitGuiAuthorityRejectsAStaleTaskCommand)
{
    EndpointTrimCommandOrder order(
        endpoint_trim_config().joint_count,
        endpoint_trim_config().sdk_quantum_rad
    );
    const std::vector<double> gui_target = joints(0.4);
    const std::vector<double> stale_task_target = joints(-0.3);

    ASSERT_TRUE(order.observe_upstream_command(
        gui_target,
        EndpointTrimCommandSource::GUI_DIRECT_EDIT,
        10.0
    ));
    ASSERT_TRUE(order.last_observation_accepted());
    order.mark_command_applied();

    EXPECT_FALSE(order.observe_upstream_command(
        stale_task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER,
        10.10
    ));
    EXPECT_FALSE(order.last_observation_accepted());
    EXPECT_EQ(order.upstream_target(), gui_target);
    EXPECT_EQ(
        order.authoritative_source(),
        EndpointTrimCommandSource::GUI_DIRECT_EDIT
    );
    EXPECT_FALSE(order.newer_task_command_requires_handoff());
}

TEST(EndpointTrimDriverOrchestrationTest,
     TaskControllerResumesOnlyAfterCommittedGuiHoldoffExpires)
{
    EndpointTrimCommandOrder order(
        endpoint_trim_config().joint_count,
        endpoint_trim_config().sdk_quantum_rad
    );
    const std::vector<double> gui_target = joints(0.4);
    const std::vector<double> task_target = joints(-0.3);

    ASSERT_TRUE(order.observe_upstream_command(
        gui_target,
        EndpointTrimCommandSource::GUI_DIRECT_EDIT,
        20.0
    ));
    order.mark_command_applied();

    // Match the existing gui_direct guard: the 0.25 s boundary itself is
    // still held, and the first time after it restores task authority.
    EXPECT_FALSE(order.observe_upstream_command(
        task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER,
        20.249
    ));
    EXPECT_EQ(order.upstream_target(), gui_target);
    EXPECT_FALSE(order.observe_upstream_command(
        task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER,
        20.250
    ));
    EXPECT_TRUE(order.observe_upstream_command(
        task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER,
        20.251
    ));
    EXPECT_TRUE(order.last_observation_accepted());
    EXPECT_EQ(order.upstream_target(), task_target);
    EXPECT_EQ(
        order.authoritative_source(),
        EndpointTrimCommandSource::TASK_CONTROLLER
    );
    EXPECT_TRUE(order.newer_task_command_requires_handoff());
    EXPECT_FALSE(order.newer_explicit_gui_command_requires_handoff());
}

TEST(EndpointTrimDriverOrchestrationTest,
     GuiDirectSyncClosesTheCommittedTaskHoldoff)
{
    EndpointTrimCommandOrder order(
        endpoint_trim_config().joint_count,
        endpoint_trim_config().sdk_quantum_rad
    );
    const std::vector<double> gui_target = joints(0.4);
    const std::vector<double> sync_target = joints(0.41);
    const std::vector<double> task_target = joints(-0.3);

    ASSERT_TRUE(order.observe_upstream_command(
        gui_target,
        EndpointTrimCommandSource::GUI_DIRECT_EDIT,
        30.0
    ));
    order.mark_command_applied();
    ASSERT_TRUE(order.observe_upstream_command(
        sync_target,
        EndpointTrimCommandSource::GUI_DIRECT_SYNC,
        30.01
    ));
    order.mark_command_applied();

    EXPECT_TRUE(order.observe_upstream_command(
        task_target,
        EndpointTrimCommandSource::TASK_CONTROLLER,
        30.01
    ));
    EXPECT_EQ(
        order.authoritative_source(),
        EndpointTrimCommandSource::TASK_CONTROLLER
    );
    EXPECT_TRUE(order.newer_task_command_requires_handoff());
}

TEST(EndpointTrimDriverOrchestrationTest,
     ReleaseDoesNotPromoteAnUnappliedTaskCommandToGui)
{
    EndpointTrimCommandOrder order(
        endpoint_trim_config().joint_count,
        endpoint_trim_config().sdk_quantum_rad
    );
    ASSERT_TRUE(order.observe_upstream_command(
        joints(0.2),
        EndpointTrimCommandSource::TASK_CONTROLLER
    ));
    ASSERT_TRUE(order.has_unapplied_command());

    order.note_release();

    EXPECT_FALSE(order.newer_explicit_gui_command_requires_handoff());
    EXPECT_FALSE(order.newer_task_command_requires_handoff());
    EXPECT_EQ(
        order.authoritative_source(),
        EndpointTrimCommandSource::TASK_CONTROLLER
    );
}

TEST(EndpointTrimDriverOrchestrationTest,
     ImmediateReleaseEventSurvivesSameTickNoopFeedbackAndConsumesOnce)
{
    EndpointTrimContinuity trim(endpoint_trim_config());
    EndpointTrimCommandOrder order;
    const EndpointTrimDecision active =
        trim.activate(joints(0.1), joints(0.01), 0.0);
    const EndpointTrimDecision released =
        trim.request_release(joints(0.095), true, 0.1);
    ASSERT_TRUE(released.release_completed);

    EXPECT_EQ(
        order.record_release(active.phase, released, true),
        EndpointTrimReleaseStatus::COMPLETED
    );
    const EndpointTrimDecision after_feedback =
        trim.note_feedback(joints(0.095), 0.11, 0.11);
    EXPECT_FALSE(after_feedback.release_completed);
    ASSERT_TRUE(order.has_terminal_release_event());
    EXPECT_EQ(
        order.consume_terminal_release_code(),
        "ENDPOINT_TRIM_RELEASE_SETTLED"
    );
    EXPECT_FALSE(order.has_terminal_release_event());
    EXPECT_TRUE(order.consume_terminal_release_code().empty());

    const uint64_t completed_generation = order.release_generation();
    EXPECT_EQ(
        order.record_release(
            EndpointTrimPhase::QUIESCENT,
            released,
            true
        ),
        EndpointTrimReleaseStatus::NOOP
    );
    EXPECT_EQ(order.release_generation(), completed_generation);
    order.capture_terminal_release(released);
    EXPECT_FALSE(order.has_terminal_release_event());

    EndpointTrimContinuity faulted(endpoint_trim_config());
    const EndpointTrimDecision fault = faulted.activate(
        std::vector<double>{0.1}, joints(), 0.0
    );
    ASSERT_EQ(fault.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(
        order.record_release(fault.phase, fault, true),
        EndpointTrimReleaseStatus::REJECTED
    );
    EXPECT_EQ(order.release_generation(), completed_generation);
    order.capture_terminal_release(released);
    EXPECT_FALSE(order.has_terminal_release_event());
}

TEST(EndpointTrimDriverOrchestrationTest,
     ReleaseStatusDistinguishesPendingCompletedNoopAndRejected)
{
    EndpointTrimCommandOrder pending_order;
    EndpointTrimContinuity pending_trim(endpoint_trim_config());
    pending_trim.activate(joints(), joints(), 0.0);
    pending_trim.request_correction(
        joints(-0.02), all_stable_joints(), 1.0, 0.1
    );
    const EndpointTrimPhase waiting_phase = pending_trim.state().phase;
    const EndpointTrimDecision pending =
        pending_trim.request_release(joints(-0.02), true, 0.2);
    EXPECT_EQ(
        pending_order.record_release(waiting_phase, pending, true),
        EndpointTrimReleaseStatus::PENDING
    );
    EXPECT_FALSE(pending_order.has_terminal_release_event());

    EndpointTrimCommandOrder completed_order;
    EndpointTrimContinuity completed_trim(endpoint_trim_config());
    const EndpointTrimDecision active =
        completed_trim.activate(joints(0.1), joints(), 0.0);
    const EndpointTrimDecision completed =
        completed_trim.request_release(joints(0.1), true, 0.1);
    EXPECT_EQ(
        completed_order.record_release(active.phase, completed, true),
        EndpointTrimReleaseStatus::COMPLETED
    );

    EndpointTrimCommandOrder noop_order;
    EndpointTrimContinuity noop_trim(endpoint_trim_config());
    EXPECT_EQ(
        noop_order.record_release(
            EndpointTrimPhase::IDLE,
            noop_trim.state(),
            false
        ),
        EndpointTrimReleaseStatus::NOOP
    );
    const EndpointTrimDecision quiescent =
        completed_trim.request_release(joints(0.1), true, 0.2);
    EXPECT_EQ(
        noop_order.record_release(
            EndpointTrimPhase::QUIESCENT,
            quiescent,
            true
        ),
        EndpointTrimReleaseStatus::NOOP
    );

    EndpointTrimCommandOrder rejected_order;
    EndpointTrimContinuity faulted(endpoint_trim_config());
    const EndpointTrimDecision fault = faulted.activate(
        std::vector<double>{0.1}, joints(), 0.0
    );
    ASSERT_EQ(fault.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(
        rejected_order.record_release(fault.phase, fault, true),
        EndpointTrimReleaseStatus::REJECTED
    );
}

TEST(EndpointTrimDriverOrchestrationTest,
     ReleaseRoutingEligibilityCannotFaultAnInactiveCoordinator)
{
    EXPECT_FALSE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::IDLE
    ));
    EXPECT_FALSE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::QUIESCENT
    ));
    EXPECT_FALSE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::FAULT
    ));
    EXPECT_TRUE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::ACTIVE_READY
    ));
    EXPECT_TRUE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::WAITING_RESPONSE
    ));
    EXPECT_TRUE(endpoint_trim_release_request_allowed(
        EndpointTrimPhase::PENDING_RELEASE
    ));

    EndpointTrimContinuity idle(endpoint_trim_config());
    idle.explicit_gui_handoff(joints(0.2), 0.0);
    ASSERT_EQ(idle.state().phase, EndpointTrimPhase::IDLE);
    if (endpoint_trim_release_request_allowed(idle.state().phase)) {
        idle.request_release(joints(0.2), true, 0.1);
    }
    EXPECT_EQ(idle.state().phase, EndpointTrimPhase::IDLE);
    EXPECT_EQ(idle.state().composed_target, joints(0.2));
}

TEST(GuiControlOwnershipTest, ManualReleaseRequiresFreshCompleteMatchedFeedback)
{
    const std::vector<double> measured{-1.95, 0.63, -0.23, -0.15, -0.42, 0.15};
    EXPECT_TRUE(controller_handoff_matches_feedback(measured, measured, true));
    EXPECT_FALSE(controller_handoff_matches_feedback(measured, {}, true));
    EXPECT_FALSE(controller_handoff_matches_feedback(measured, measured, false));
    EXPECT_FALSE(controller_handoff_matches_feedback({}, {}, true));
    auto stale_target = measured;
    stale_target[1] -= 0.52;
    EXPECT_FALSE(controller_handoff_matches_feedback(stale_target, measured, true));
    auto nearby_target = measured;
    nearby_target[2] += 0.002;
    EXPECT_TRUE(controller_handoff_matches_feedback(nearby_target, measured, true));
    nearby_target[2] += 0.002;
    EXPECT_FALSE(controller_handoff_matches_feedback(nearby_target, measured, true));
    nearby_target[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller_handoff_matches_feedback(nearby_target, measured, true));
    EXPECT_FALSE(controller_handoff_matches_feedback(measured, nearby_target, true));
}

namespace
{
std::vector<uint8_t> handoff_test_frame(const std::vector<uint16_t>& counts)
{
    std::vector<uint8_t> frame(34, 0);
    frame[0] = 0xAA; frame[1] = 0x06; frame[2] = 0x03; frame[3] = 0x1C; frame[33] = 0xFF;
    for (size_t i = 0; i < counts.size(); ++i) {
        frame[4 + 4*i] = counts[i] & 0xFF;
        frame[5 + 4*i] = counts[i] >> 8;
    }
    return frame;
}
}

TEST(GuiControlOwnershipTest, FrozenWireHandoffPreservesBaselineDespiteMeasuredResidual)
{
    const auto frame = handoff_test_frame({797, 1964, 2183, 2009, 1895, 2095, 995});
    std::vector<double> wire; double grip = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
    auto actual = wire;
    actual[1] = sdk_joint_position_decode(1956);
    actual[2] = sdk_joint_position_decode(2172);
    ASSERT_FALSE(controller_handoff_matches_feedback(wire, actual, true));
    EXPECT_TRUE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, actual, true, frame, true));
    // Repeating feedback would be a physical reference change, not a handoff.
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        actual, grip, actual, true, frame, true));
    auto fractional_same_words = wire;
    for (double& q : fractional_same_words) q += 0.1 * 2.0 * M_PI / 4096.0;
    EXPECT_TRUE(controller_handoff_matches_frozen_sdk_or_initial(
        fractional_same_words, grip, actual, true, frame, true));
}

TEST(GuiControlOwnershipTest, EveryChangedJointPositionWordRejectsFrozenHandoff)
{
    // All 4096 codes, both directions, every arm axis: not even one count
    // can be authorized as a no-motion ownership transition.
    for (size_t joint = 0; joint < 6; ++joint) {
        for (uint16_t count = 0; count <= 4095; ++count) {
            std::vector<uint16_t> counts(7, 2048); counts[6] = 995; counts[joint] = count;
            const auto frame = handoff_test_frame(counts);
            std::vector<double> wire; double grip = 0.0;
            ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
            EXPECT_TRUE(controller_handoff_matches_frozen_sdk_or_initial(
                wire, grip, wire, true, frame, true));
            for (int direction : {-1, 1}) {
                const int changed = static_cast<int>(count) + direction;
                if (changed < 0 || changed > 4095) continue;
                auto target = wire; target[joint] = sdk_joint_position_decode(changed);
                EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
                    target, grip, wire, true, frame, true));
            }
        }
    }
}

TEST(GuiControlOwnershipTest, GripperWordCannotChangeDuringFrozenArmHandoff)
{
    const auto frame = handoff_test_frame({2048, 2048, 2048, 2048, 2048, 2048, 995});
    std::vector<double> wire; double grip = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
    for (uint16_t count = 0; count <= 1000; ++count) {
        EXPECT_EQ(controller_handoff_matches_frozen_sdk_or_initial(
            wire, sdk_gripper_position_decode(count), wire, true, frame, true), count == 995);
    }
}

TEST(GuiControlOwnershipTest, FrozenHandoffRequiresFreshCompleteFeedbackAndValidWrittenFrame)
{
    const auto frame = handoff_test_frame({2048, 2048, 2048, 2048, 2048, 2048, 995});
    std::vector<double> wire; double grip = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, wire, false, frame, true));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, {}, true, frame, true));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, wire, true, frame, false));
    auto invalid = frame; invalid[4] = 0xFF; invalid[5] = 0xFF;
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, wire, true, invalid, true));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, wire, true, std::vector<uint8_t>(34, 0), true));
    auto bad_feedback = wire; bad_feedback[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, grip, bad_feedback, true, frame, true));
    auto bad_command = wire; bad_command[2] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        bad_command, grip, wire, true, frame, true));
    bad_command = wire; bad_command[1] = 4.0;
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        bad_command, grip, wire, true, frame, true));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        wire, std::numeric_limits<double>::quiet_NaN(), wire, true, frame, true));
}

TEST(GuiControlOwnershipTest, ResetWithoutWireRetainsInitialFeedbackHandshakeNotOldGoal)
{
    const auto frame = handoff_test_frame({2048, 2058, 2058, 2048, 2048, 2048, 995});
    std::vector<double> previous; double grip = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, previous, grip));
    const auto fresh = joints();
    const std::vector<uint8_t> cleared_after_power_reset;
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        previous, grip, fresh, true, cleared_after_power_reset, false));
    EXPECT_TRUE(controller_handoff_matches_frozen_sdk_or_initial(
        fresh, grip, fresh, true, cleared_after_power_reset, false));
    EXPECT_FALSE(controller_handoff_matches_frozen_sdk_or_initial(
        fresh, grip, fresh, false, cleared_after_power_reset, false));
}

TEST(GuiControlOwnershipTest, ReferenceSyncRequiresCurrentResetAndOwnershipEpoch)
{
    EXPECT_TRUE(reference_sync_stamp_in_current_epoch(120, 100, 110, 130));
    EXPECT_TRUE(reference_sync_stamp_in_current_epoch(110, 100, 110, 110));
    EXPECT_TRUE(reference_sync_stamp_in_current_epoch(100, 100, 0, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(0, 100, 110, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(120, 0, 110, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(99, 100, 0, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(109, 100, 110, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(131, 100, 110, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(120, 125, 110, 130));
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(120, 100, 110, 90));
    // Preserve nanosecond boundaries at real ROS epoch magnitudes.
    const uint64_t epoch = 1789200000000000000ULL;
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(epoch - 1, epoch, 0, epoch + 1));
    EXPECT_TRUE(reference_sync_stamp_in_current_epoch(epoch, epoch, 0, epoch + 1));
    EXPECT_FALSE(joint_source_allowed_by_control_mode(true, "motion_gateway_reference_sync"));
    EXPECT_TRUE(joint_source_allowed_by_control_mode(false, "motion_gateway_reference_sync"));
}

TEST(GuiControlOwnershipTest, ReferenceEpochChangesOnlyOnResetAndNeverReusesAStamp)
{
    EXPECT_EQ(reference_epoch_next_stamp_ns(100, 0), 100U);
    EXPECT_EQ(reference_epoch_next_stamp_ns(100, 100), 101U);
    EXPECT_EQ(reference_epoch_next_stamp_ns(90, 100), 101U);
    EXPECT_EQ(reference_epoch_next_stamp_ns(130, 100), 130U);
    EXPECT_EQ(reference_epoch_next_stamp_ns(0, 0), 1U);
    const uint64_t epoch = 1789200000000000000ULL;
    EXPECT_EQ(reference_epoch_next_stamp_ns(epoch, epoch), epoch + 1);
    // On clock reversal the future barrier rejects reference admission; it
    // does not make a previous command new by relaxing the timestamp check.
    EXPECT_FALSE(reference_sync_stamp_in_current_epoch(90,
        reference_epoch_next_stamp_ns(90, 100), 0, 90));
}

TEST(GuiControlOwnershipTest, ReferenceSyncNoWireRequiresTrueInitializationAndOriginalTolerance)
{
    const auto actual = joints();
    for (const std::string& status : {"PENDING:POSITIVE_ENABLE_REQUESTED", "PENDING:COMMAND_SYNCHRONIZED"}) {
        EXPECT_TRUE(reference_sync_matches_successful_sdk_or_initial(
            actual, 0.2, actual, true, {}, false, status));
        auto near = actual; near[1] = 0.003;
        EXPECT_TRUE(reference_sync_matches_successful_sdk_or_initial(
            near, 0.2, actual, true, {}, false, status));
        near[1] = std::nextafter(0.003, 1.0);
        EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
            near, 0.2, actual, true, {}, false, status));
    }
    for (const std::string& status : {"", "UNCONFIRMED:SERIAL_RECONNECTED_TORQUE_UNCHANGED",
            "CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE", "PENDING:AWAITING_ENCODER_RESPONSE",
            "PENDING:ENCODER_RESPONSE_TIMEOUT", "DISABLED:EXPLICIT_TORQUE_OFF"}) {
        EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
            actual, 0.2, actual, true, {}, false, status));
    }
    EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
        actual, 0.2, actual, false, {}, false, "PENDING:POSITIVE_ENABLE_REQUESTED"));
}

TEST(GuiControlOwnershipTest, ReferenceSyncAlwaysChecksCurrentSuccessfulWordsAfterTrackingRace)
{
    const std::string tracking = "CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE";
    for (size_t joint = 0; joint < 6; ++joint) {
        for (uint16_t count = 0; count <= 4095; ++count) {
            std::vector<uint16_t> counts(7, 2048); counts[6] = 995; counts[joint] = count;
            const auto frame = handoff_test_frame(counts);
            std::vector<double> wire; double grip = 0.0;
            ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
            EXPECT_TRUE(reference_sync_matches_successful_sdk_or_initial(
                wire, grip, wire, true, frame, true, tracking));
            for (int direction : {-1, 1}) {
                const int changed = static_cast<int>(count) + direction;
                if (changed < 0 || changed > 4095) continue;
                auto queued = wire; queued[joint] = sdk_joint_position_decode(changed);
                EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
                    queued, grip, wire, true, frame, true, tracking));
            }
        }
    }
}

TEST(GuiControlOwnershipTest, ReferenceSyncPreservesGripperAndRejectsInvalidSuccessfulEvidence)
{
    const auto frame = handoff_test_frame({797, 1964, 2183, 2009, 1895, 2095, 995});
    std::vector<double> wire; double grip = 0.0;
    ASSERT_TRUE(decode_retained_sdk_hold(frame, wire, grip));
    auto actual = wire; actual[1] -= 8 * 2.0 * M_PI / 4096.0;
    const std::string pending = "PENDING:COMMAND_SYNCHRONIZED";
    EXPECT_TRUE(reference_sync_matches_successful_sdk_or_initial(
        wire, grip, actual, true, frame, true, pending));
    for (uint16_t code = 0; code <= 1000; ++code) {
        EXPECT_EQ(reference_sync_matches_successful_sdk_or_initial(
            wire, sdk_gripper_position_decode(code), actual, true, frame, true, pending), code == 995);
    }
    EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
        wire, grip, actual, true, frame, false, pending));
    EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
        wire, grip, actual, false, frame, true, pending));
    EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
        wire, grip, actual, true, std::vector<uint8_t>(34, 0), true, pending));
    EXPECT_FALSE(reference_sync_matches_successful_sdk_or_initial(
        wire, grip, actual, true, frame, true, "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT"));
}

TEST(EndpointTrimPerJoint, ChangesOnlyTheConfiguredAxisLimit)
{
    auto config = endpoint_trim_config();
    const double quantum = config.sdk_quantum_rad;
    config.max_step_rad_by_joint = {4*quantum, 7*quantum, 4*quantum,
                                    4*quantum, 4*quantum, 4*quantum};
    EndpointTrimContinuity trim(config);
    trim.activate(std::vector<double>(6, 20*quantum), joints(), 0.0);
    const auto step = trim.request_correction(joints(), all_stable_joints(), 1.0, .1);
    ASSERT_EQ(step.phase, EndpointTrimPhase::WAITING_RESPONSE);
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_NEAR(step.applied_step[i], (i == 1 ? 7 : 4)*quantum, 1e-14);
    }
}

TEST(EndpointTrimPerJoint, SevenCountStepAcceptsFiveCountResponseAtEncoderCoordinates)
{
    auto config = endpoint_trim_config();
    const double quantum = config.sdk_quantum_rad;
    config.response_min_rad = 2*quantum;
    config.max_step_rad_by_joint = {4*quantum, 7*quantum, 4*quantum,
                                    4*quantum, 4*quantum, 4*quantum};
    for (int count : {100, 785, 1446, 2048, 2322, 4000}) {
        EndpointTrimContinuity trim(config);
        auto baseline = joints();
        baseline[1] = sdk_joint_position_decode(count);
        auto reference = baseline;
        reference[1] += 20*quantum;
        trim.activate(reference, joints(), 0.0);
        trim.request_correction(baseline, {0,1,0,0,0,0}, 1.0, 1.0);
        auto measured = baseline;
        measured[1] = sdk_joint_position_decode(count+5);
        trim.note_feedback(measured, 1.1, 1.1);
        EXPECT_EQ(trim.note_feedback(measured, 1.5, 1.5).phase,
                  EndpointTrimPhase::ACTIVE_READY) << count;
    }
}

TEST(EndpointTrimPerJoint, NoResponseStillFaultsWithoutAnotherIncrement)
{
    auto config = endpoint_trim_config();
    const double quantum = config.sdk_quantum_rad;
    config.max_step_rad_by_joint = {4*quantum, 7*quantum, 4*quantum,
                                    4*quantum, 4*quantum, 4*quantum};
    EndpointTrimContinuity trim(config);
    auto reference = joints(); reference[1] = 20*quantum;
    trim.activate(reference, joints(), 0.0);
    auto step = trim.request_correction(joints(), {0,1,0,0,0,0}, 1.0, .1);
    trim.note_feedback(joints(), .8, .8);
    EXPECT_EQ(trim.update(1.2).phase, EndpointTrimPhase::FAULT);
    auto retry = trim.request_correction(joints(), {0,1,0,0,0,0}, 1.0, 1.3);
    EXPECT_EQ(retry.phase, EndpointTrimPhase::FAULT);
    EXPECT_EQ(retry.composed_target, step.composed_target);
}

TEST(EndpointTrimPerJoint, ThreeCountResponseShortfallRemainsRejected)
{
    auto config = endpoint_trim_config();
    const double quantum = config.sdk_quantum_rad;
    config.max_step_rad_by_joint = {4*quantum, 7*quantum, 4*quantum,
                                    4*quantum, 4*quantum, 4*quantum};
    EndpointTrimContinuity trim(config);
    auto reference = joints(); reference[1] = 20*quantum;
    trim.activate(reference, joints(), 0.0);
    trim.request_correction(joints(), {0,1,0,0,0,0}, 1.0, .1);
    auto measured = joints(); measured[1] = 4*quantum;
    trim.note_feedback(measured, .2, .2);
    EXPECT_EQ(trim.note_feedback(measured, .7, .7).phase,
              EndpointTrimPhase::WAITING_RESPONSE);
    EXPECT_EQ(trim.update(1.2).phase, EndpointTrimPhase::FAULT);
}

TEST(EndpointTrimPerJoint, InvalidPerAxisConfigurationFailsClosed)
{
    for (const auto& limits : std::vector<std::vector<double>>{
             {0.01}, {0.,0.,0.,0.,0.,0.}, std::vector<double>(6, .03),
             std::vector<double>(6, std::numeric_limits<double>::quiet_NaN())}) {
        auto config = endpoint_trim_config();
        config.max_step_rad_by_joint = limits;
        EndpointTrimContinuity trim(config);
        EXPECT_EQ(trim.activate(joints(), joints(), 0.0).phase, EndpointTrimPhase::FAULT);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
