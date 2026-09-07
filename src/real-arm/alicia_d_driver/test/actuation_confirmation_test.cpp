#include <gtest/gtest.h>

#include "alicia_d_driver/actuation_confirmation.hpp"
#include "alicia_d_driver/endpoint_trim_continuity.hpp"
#include "alicia_d_driver/endpoint_trim_driver_admission.hpp"
#include "alicia_d_driver/gui_direct_hold.hpp"

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
    const double angle_deg = radians * 180.0 / M_PI;
    const int hardware_value = static_cast<int>(
        (angle_deg + 180.0) / 360.0 * 4096.0
    );
    const double decoded_deg = -180.0 +
        (static_cast<double>(hardware_value) / 4096.0) * 360.0;
    return decoded_deg * M_PI / 180.0;
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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
