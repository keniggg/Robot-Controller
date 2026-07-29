#include <gtest/gtest.h>

#include "alicia_d_driver/actuation_confirmation.hpp"

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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
