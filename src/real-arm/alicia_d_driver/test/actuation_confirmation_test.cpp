#include <gtest/gtest.h>

#include "alicia_d_driver/actuation_confirmation.hpp"
#include "alicia_d_driver/endpoint_trim_continuity.hpp"

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

TEST(EndpointTrimContinuityTest, SettlesAgainstTheComposedCommandTarget)
{
    EndpointTrimConfig config = endpoint_trim_config();
    EndpointTrimContinuity trim(config);
    const std::vector<double> baseline = joints(-config.max_step_rad);

    trim.activate(joints(), joints(), 0.0);
    const EndpointTrimDecision correction = trim.request_correction(
        baseline, all_stable_joints(), 1.0, 0.1
    );
    ASSERT_EQ(correction.phase, EndpointTrimPhase::WAITING_RESPONSE);

    EXPECT_EQ(
        trim.note_feedback(correction.composed_target, 0.2, 0.2).phase,
        EndpointTrimPhase::WAITING_RESPONSE
    );
    EXPECT_EQ(
        trim.note_feedback(
            correction.composed_target,
            0.2 + config.stable_sec,
            0.2 + config.stable_sec
        ).phase,
        EndpointTrimPhase::ACTIVE_READY
    );
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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
