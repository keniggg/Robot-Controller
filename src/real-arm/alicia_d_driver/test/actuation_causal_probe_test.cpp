#include <gtest/gtest.h>
#include "alicia_d_driver/actuation_confirmation.hpp"
#include <cmath>
#include <limits>
#include <vector>

namespace {
constexpr double Q = 2.0 * M_PI / 4096.0;
ActuationConfirmationConfig causal_config() {
    ActuationConfirmationConfig c;
    c.command_probe_min_delta_rad = .01;
    c.measured_response_min_delta_rad = .003;
    c.response_timeout_sec = 1.;
    c.confirmation_freshness_sec = 2.;
    return c;
}
std::vector<double> zero() { return std::vector<double>(6, 0.); }
std::vector<double> at(unsigned axis, double value) {
    auto q = zero(); q[axis] = value; return q;
}
ActuationConfirmation committed_hold(const std::vector<double>& hold = zero()) {
    ActuationConfirmation c(causal_config());
    c.reset_for_positive_enable(10.);
    c.note_feedback(hold, 10.01);
    EXPECT_TRUE(c.admit_command(hold, 10.02, nullptr));
    // Explicit successful physical hold write, never fabricated by admission.
    c.note_committed_reference(hold, 10.03);
    return c;
}
ActuationConfirmation confirmed_then_settled() {
    auto c = committed_hold();
    c.note_streamed_target(at(0, .03), 10.1);
    c.note_feedback(at(0, .006), 10.11);
    EXPECT_TRUE(c.motion_confirmed(10.11));
    c.note_streamed_target(at(0, .03), 10.12);
    c.note_feedback(at(0, .03), 10.2);
    c.note_streamed_target(at(0, .03), 10.21);
    return c;
}
}

TEST(CausalProbe, FirstUnreferencedWriteCannotInferMotionFromEncoderOffset) {
    ActuationConfirmation c(causal_config());
    c.reset_for_positive_enable(10.);
    c.note_feedback(zero(), 10.01);
    ASSERT_TRUE(c.admit_command(at(0, .03), 10.02, nullptr));
    c.note_streamed_target(at(0, .03), 10.03);
    c.note_feedback(at(0, .006), 10.04);
    EXPECT_FALSE(c.motion_confirmed(10.04));
    c.note_streamed_target(at(0, .03), 10.05);
    c.note_feedback(at(0, .006), 10.06);
    c.update(11.2);
    EXPECT_EQ(c.status_text(), "PENDING:COMMAND_SYNCHRONIZED");
    // The first subsequent real step has a successful SDK origin.
    c.note_feedback(at(0, .006), 11.21);
    c.note_streamed_target(at(0, .045), 11.22);
    c.note_feedback(at(0, .010), 11.23);
    EXPECT_TRUE(c.motion_confirmed(11.23));
}

TEST(CausalProbe, FirstRealStepFromCommittedHoldRequiresNewDirectionalFeedback) {
    for (unsigned axis = 0; axis < 6; ++axis) for (double sign : {-1., 1.}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(sign);
        auto c = committed_hold();
        c.note_streamed_target(at(axis, sign*.012), 10.1);
        c.note_feedback(at(axis, sign*.004), 10.1);
        EXPECT_FALSE(c.motion_confirmed(10.1));
        c.note_feedback(at(axis, sign*.004), 10.11);
        // Equal-source-time movement is pre-command evidence, not a response.
        EXPECT_FALSE(c.motion_confirmed(10.11));
        c.note_feedback(at(axis, sign*.008), 10.12);
        EXPECT_TRUE(c.motion_confirmed(10.12));
    }
}

TEST(CausalProbe, PendingFixedOffsetAndKeepalivesNeverCreateNewIntent) {
    for (unsigned axis = 0; axis < 6; ++axis) for (double sign : {-1., 1.}) {
        auto c = committed_hold();
        for (int i=0; i<50; ++i) {
            const double t=10.1+.1*i;
            c.note_feedback(at(axis, sign*.025), t);
            c.note_streamed_target(zero(), t+.01);
            c.update(t+.02);
            EXPECT_EQ(c.status_text(), "PENDING:COMMAND_SYNCHRONIZED");
        }
    }
}

TEST(CausalProbe, OffsetAndFeedbackAheadCannotSelectHeldAxisOrEraseCommandAccumulation) {
    for (unsigned moving=0; moving<6; ++moving) for (unsigned held=0; held<6; ++held) {
        if (moving==held) continue;
        for (double sign : {-1., 1.}) for (bool responds : {false, true}) {
            SCOPED_TRACE(moving);
            SCOPED_TRACE(held);
            SCOPED_TRACE(sign);
            SCOPED_TRACE(responds);
            auto c=committed_hold();
            auto measured=zero(); measured[moving]=sign*2*Q; measured[held]=-sign*9*Q;
            c.note_feedback(measured, 10.07);
            c.note_streamed_target(zero(), 10.08);
            for (int n=1; n<=8; ++n) {
                const double t=10.1+.05*n;
                c.note_streamed_target(at(moving, sign*n*Q), t);
                measured[moving]=sign*(2+(responds?n:0))*Q;
                // Uncommanded-axis movement can never substitute for response.
                measured[held]=-sign*(9+(n%2)*3)*Q;
                c.note_feedback(measured, t+.01);
            }
            EXPECT_EQ(c.motion_confirmed(10.52), responds);
            if (!responds) {
                c.note_feedback(measured, 11.44);
                c.update(11.46);
                EXPECT_TRUE(c.initial_probe_failed());
                EXPECT_EQ(c.status_text(), "UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT");
            }
        }
    }
}

TEST(CausalProbe, SmoothStreamRetainsCausalMovementButNeedsFrameAfterThresholdCommit) {
    auto c=committed_hold();
    for(int n=1;n<=6;++n) {
        c.note_streamed_target(at(4, -n*Q), 10.1+.05*n);
        c.note_feedback(at(4, -n*Q), 10.11+.05*n);
    }
    c.note_streamed_target(at(4, -7*Q), 10.45);
    EXPECT_FALSE(c.motion_confirmed(10.45));
    c.note_feedback(at(4, -7*Q), 10.45);
    EXPECT_FALSE(c.motion_confirmed(10.45));
    c.note_feedback(at(4, -7*Q), 10.46);
    EXPECT_TRUE(c.motion_confirmed(10.46));
}

TEST(CausalProbe, LateAxisUsesItsActualCommittedDirectionNotEarlierSmallSign) {
    for(bool responds:{false,true}) {
        auto c=committed_hold();
        auto target=at(0,.012); target[3]=.001;
        c.note_streamed_target(target,10.1);
        c.note_feedback(zero(),10.2);
        target[3]=-.012;
        c.note_streamed_target(target,10.8);
        c.note_feedback(at(3,responds?-.004:0.),10.81);
        c.update(11.11);
        EXPECT_EQ(c.motion_confirmed(11.11),responds);
        if(!responds) EXPECT_TRUE(c.initial_probe_failed());
    }
}

TEST(CausalProbe, LateAxisCannotCreditItsPrecommandMotionOrRenewDeadline) {
    auto c=committed_hold();
    c.note_streamed_target(at(0,.012),10.1);
    c.note_feedback(at(2,-.02),10.2);
    auto target=at(0,.012);target[2]=-.012;
    c.note_streamed_target(target,10.9);
    c.note_feedback(at(2,-.02),10.91);
    EXPECT_FALSE(c.motion_confirmed(10.91));
    c.note_feedback(at(2,-.024),11.101);
    c.update(11.102);
    EXPECT_TRUE(c.initial_probe_failed());
}

TEST(CausalProbe, SubthresholdReversalRequiresResponseToItsNewCommittedDirection) {
    for (unsigned axis=0; axis<6; ++axis) for (double sign:{-1.,1.}) {
        for (bool late_join:{false,true}) for (bool responds:{false,true}) {
            SCOPED_TRACE(axis);
            SCOPED_TRACE(sign);
            SCOPED_TRACE(late_join);
            SCOPED_TRACE(responds);
            auto c=committed_hold();
            auto target=at(axis,-sign*.001);
            if (late_join) target[(axis+1)%6]=.012;
            c.note_streamed_target(target,10.1);
            const auto prior_motion=at(axis,sign*.0031);
            c.note_feedback(prior_motion,10.2);
            target[axis]=sign*.012;
            c.note_streamed_target(target,10.8);
            ASSERT_EQ(c.status_text(),"PENDING:AWAITING_ENCODER_RESPONSE");
            c.note_feedback(prior_motion,10.81);
            EXPECT_FALSE(c.motion_confirmed(10.81));
            if (responds) c.note_feedback(at(axis,sign*.0062),10.82);
            const double after_deadline=late_join?11.11:11.81;
            c.update(after_deadline);
            EXPECT_EQ(c.motion_confirmed(after_deadline),responds);
            if (!responds) EXPECT_TRUE(c.initial_probe_failed());
        }
    }
}

TEST(CausalProbe, SubthresholdReversalKeepsOriginalLateJoinDeadline) {
    for (unsigned axis=0; axis<6; ++axis) for (double sign:{-1.,1.}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(sign);
        auto c=committed_hold();
        auto target=at(axis,-sign*.001);
        target[(axis+1)%6]=.012;
        c.note_streamed_target(target,10.1);
        c.note_feedback(at(axis,sign*.0031),10.2);
        target[axis]=sign*.012;
        c.note_streamed_target(target,10.8);
        c.note_feedback(at(axis,sign*.0031),10.81);
        // A real response after the original 10.1 + 1.0 deadline is late,
        // even though it is within one second of this axis's reversal.
        c.note_feedback(at(axis,sign*.0062),11.11);
        c.update(11.12);
        EXPECT_EQ(c.status_text(),"UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT");
        EXPECT_TRUE(c.initial_probe_failed());
    }
}

TEST(CausalProbe, SubthresholdReversalAccumulatesFromActualSdkTurningPoint) {
    for (unsigned axis=0; axis<6; ++axis) for (double sign:{-1.,1.}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(sign);
        auto c=committed_hold();
        c.note_streamed_target(at(axis,sign*.009),10.1);
        c.note_feedback(at(axis,-sign*.0031),10.2);
        c.note_streamed_target(at(axis,sign*.006),10.3);
        c.note_feedback(at(axis,-sign*.0031),10.4);
        // .014 rad reversed SDK displacement crosses the threshold although
        // the absolute target is only .005 from the older SDK origin.
        c.note_streamed_target(at(axis,-sign*.005),10.8);
        ASSERT_EQ(c.status_text(),"PENDING:AWAITING_ENCODER_RESPONSE");
        c.note_feedback(at(axis,-sign*.0031),10.81);
        EXPECT_FALSE(c.motion_confirmed(10.81));
        c.note_feedback(at(axis,-sign*.0062),10.82);
        EXPECT_TRUE(c.motion_confirmed(10.82));
    }
}

TEST(CausalProbe, DelayedPrecommandSampleRefinesBaselineAndCannotBecomeLaterResponse) {
    auto c=committed_hold();
    c.note_streamed_target(at(1,.012),10.2);
    c.note_feedback(at(1,.004),10.19);
    c.note_feedback(at(1,.004),10.21);
    EXPECT_FALSE(c.motion_confirmed(10.21));
    c.note_feedback(at(1,.008),10.22);
    EXPECT_TRUE(c.motion_confirmed(10.22));
}

TEST(CausalProbe, FeedbackAfterInterpolationTickIsMeasuredBeforeActualCommit) {
    auto c=committed_hold();
    // Interpolation started earlier; only the later successful-write stamp
    // establishes causality. Feedback received meanwhile is the baseline.
    c.note_feedback(at(1,.004),10.19);
    c.note_streamed_target(at(1,.012),10.2);
    c.note_feedback(at(1,.004),10.21);
    EXPECT_FALSE(c.motion_confirmed(10.21));
    c.note_feedback(at(1,.008),10.22);
    EXPECT_TRUE(c.motion_confirmed(10.22));
}

TEST(CausalProbe, AdmissionAndFeedbackWithoutSuccessfulSdkWriteNeverConfirm) {
    auto c=committed_hold();
    ASSERT_TRUE(c.admit_command(at(2,.03),10.1,nullptr));
    // Simulated write failure: no successful-write evidence API is invoked.
    c.note_feedback(at(2,.02),10.2);
    c.update(11.2);
    EXPECT_FALSE(c.motion_confirmed(11.2));
    EXPECT_EQ(c.status_text(),"PENDING:COMMAND_SYNCHRONIZED");
    c.note_streamed_target(at(2,.03),11.21);
    c.note_feedback(at(2,.024),11.22);
    EXPECT_TRUE(c.motion_confirmed(11.22));
}

TEST(CausalProbe, ReferenceOnlyWritesCannotAuthorizeResponseOrRenewActiveDeadline) {
    auto c=committed_hold();
    c.note_committed_reference(at(5,.03),10.1);
    c.note_feedback(at(5,.006),10.2);
    EXPECT_FALSE(c.motion_confirmed(10.2));
    c.note_streamed_target(at(5,.045),10.3);
    for(int i=0;i<8;++i) {
        c.note_committed_reference(at(5,.045),10.4+.1*i);
        c.note_feedback(at(5,.006),10.41+.1*i);
    }
    c.update(11.31);
    EXPECT_EQ(c.status_text(),"UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT");
}

TEST(CausalProbe, OwnerResetDiscardsOldResponseButRetainsSuccessfulSdkOrigin) {
    auto c=committed_hold();
    c.note_streamed_target(at(2,.025),10.1);
    c.reset_motion_observation();
    c.note_feedback(at(2,.006),10.2);
    c.note_committed_reference(at(2,.025),10.21);
    c.note_streamed_target(at(2,.025),10.3);
    c.note_feedback(at(2,.012),10.4);
    EXPECT_FALSE(c.motion_confirmed(10.4));
    c.note_streamed_target(at(2,.04),10.5);
    c.note_feedback(at(2,.016),10.51);
    EXPECT_TRUE(c.motion_confirmed(10.51));
}

TEST(CausalProbe, PositiveEnableEpochRequiresNewCommittedReferenceAndNewResponse) {
    auto c=committed_hold();
    c.note_streamed_target(at(0,.012),10.1);
    c.reset_for_positive_enable(10.2);
    c.note_feedback(at(0,.004),10.3);
    ASSERT_TRUE(c.admit_command(at(0,.004),10.31,nullptr));
    c.note_streamed_target(at(0,.012),10.4); // New epoch's first actual write: reference only.
    c.note_feedback(at(0,.008),10.5);
    EXPECT_FALSE(c.motion_confirmed(10.5));
    c.note_streamed_target(at(0,.024),10.6);
    c.note_feedback(at(0,.012),10.61);
    EXPECT_TRUE(c.motion_confirmed(10.61));
}

TEST(CausalProbe, ConfirmedHoldOffsetRemainsConfirmedAndNewStepNeedsNewResponse) {
    for(double sign:{-1.,1.}) for(bool responds:{false,true}) {
        auto c=committed_hold();
        c.note_streamed_target(at(1,sign*.03),10.1);
        c.note_feedback(at(1,sign*.004),10.11);
        ASSERT_TRUE(c.motion_confirmed(10.11));
        for(int n=0;n<30;++n) {
            const double t=10.2+.1*n;
            c.note_streamed_target(at(1,sign*.03),t);
            c.note_feedback(at(1,sign*.004),t+.01);
            c.update(t+.02);
            EXPECT_TRUE(c.motion_confirmed(t+.02));
        }
        c.note_streamed_target(at(1,sign*.045),13.2);
        c.note_feedback(at(1,sign*(responds?.008:.004)),13.21);
        c.note_feedback(at(1,sign*(responds?.008:.004)),14.19);
        c.update(14.21);
        EXPECT_EQ(c.motion_confirmed(14.21),responds);
        if(!responds) EXPECT_EQ(c.status_text(),"UNCONFIRMED:ENCODER_RESPONSE_LOST");
    }
}

TEST(CausalProbe, GenuineReversalRequiresNewResponseAndDoesNotExtendWindow) {
    for(int response=0;response<3;++response) {
        auto c=confirmed_then_settled();
        c.note_streamed_target(at(0,.045),10.3);
        c.note_streamed_target(zero(),10.4);
        c.note_feedback(at(0,response==1?.024:(response==2?.036:.03)),10.5);
        c.note_feedback(at(0,response==1?.024:(response==2?.036:.03)),11.29);
        c.update(11.31);
        EXPECT_EQ(c.motion_confirmed(11.31),response==1);
    }
}

TEST(CausalProbe, RepeatedReversalsAndLateAxesCannotExtendOriginalDeadline) {
    auto c=confirmed_then_settled();
    c.note_streamed_target(at(0,.045),10.3);
    for(int n=0;n<8;++n) {
        auto target=at(0,n%2?.06:0.); target[4]=n>=6?.02:0.;
        c.note_streamed_target(target,10.4+.1*n);
        c.note_feedback(at(0,.03),10.41+.1*n);
    }
    c.update(11.31);
    EXPECT_EQ(c.status_text(),"UNCONFIRMED:ENCODER_RESPONSE_LOST");
}

TEST(CausalProbe, StaleFeedbackCannotAdmitResponseAndHeldTargetCannotRepairMissingEvidence) {
    auto c=committed_hold();
    c.note_streamed_target(at(3,.012),13.); // No fresh measurement for this successful write.
    c.note_feedback(at(3,.004),13.01);
    c.note_streamed_target(at(3,.012),13.02);
    c.note_feedback(at(3,.008),13.03);
    EXPECT_FALSE(c.motion_confirmed(13.03));
    c.note_streamed_target(at(3,.024),13.1);
    c.note_feedback(at(3,.012),13.11);
    EXPECT_TRUE(c.motion_confirmed(13.11));
}

TEST(CausalProbe, InvalidReferenceOrFeedbackCannotManufactureConfirmation) {
    auto c=committed_hold();
    auto bad=zero(); bad[2]=std::numeric_limits<double>::quiet_NaN();
    c.note_committed_reference(bad,10.1);
    c.note_streamed_target(at(2,.012),10.2);
    c.note_feedback(bad,10.21);
    c.note_feedback(std::vector<double>(5,.01),10.22);
    c.note_feedback(at(2,.004),std::numeric_limits<double>::infinity());
    EXPECT_FALSE(c.motion_confirmed(10.3));
    c.note_feedback(zero(),11.19);
    c.update(11.21);
    EXPECT_TRUE(c.initial_probe_failed());
}

TEST(CausalProbe, BackwardSdkStampCannotProvideAnIntentOrOverwriteCommittedReference) {
    auto c=committed_hold();
    c.note_streamed_target(at(2,.012),10.0);
    c.note_feedback(at(2,.004),10.1);
    EXPECT_FALSE(c.motion_confirmed(10.1));
    c.note_streamed_target(at(2,.012),10.2);
    c.note_feedback(at(2,.004),10.21);
    EXPECT_FALSE(c.motion_confirmed(10.21));
    c.note_feedback(at(2,.008),10.22);
    EXPECT_TRUE(c.motion_confirmed(10.22));
}

TEST(CausalProbe, InitialFailureCannotRearmThroughHoldsOwnerResetOrLateResponse) {
    auto c=committed_hold();
    c.note_streamed_target(at(0,.012),10.1);
    c.update(11.11);
    ASSERT_TRUE(c.initial_probe_failed());
    c.note_committed_reference(at(0,.012),11.2);
    c.reset_motion_observation();
    c.note_feedback(at(0,.012),11.3);
    c.note_streamed_target(at(0,.03),11.4);
    EXPECT_FALSE(c.motion_confirmed(11.4));
    EXPECT_FALSE(c.admit_command(at(0,.03),11.5,nullptr));
    EXPECT_TRUE(c.initial_probe_failed());
}
