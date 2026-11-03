#include <gtest/gtest.h>
#include "alicia_d_driver/gui_direct_profile.hpp"
#include "alicia_d_driver/actuation_confirmation.hpp"
#include <limits>

namespace {
const double quantum = 2.0 * M_PI / 4096.0;
std::vector<double> seed() {
    std::vector<double> q;
    for (uint16_t word : {820, 2430, 1961, 1978, 1771, 2077}) q.push_back(sdk_joint_position_decode(word));
    return q;
}
void finish(GuiDirectProfile& p) { for (int i=0; i<4000; ++i) p.advance(.02); }
}

TEST(GuiDirectProfileTest, InvalidLimitsAndSourcesNeverInitialize) {
    GuiDirectProfileLimits limits;
    limits.jerk = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(GuiDirectProfile p(limits), std::invalid_argument);
    GuiDirectProfile p;
    EXPECT_FALSE(p.seed({0.0}));
    auto q=seed(); q[3]=std::numeric_limits<double>::infinity();
    EXPECT_FALSE(p.seed(q)); EXPECT_FALSE(p.initialized());
    EXPECT_FALSE(p.request(4,.1));
}

TEST(GuiDirectProfileTest, SixDegreeTargetHasBoundedMonotonicCurveAndExactFinalWords) {
    GuiDirectProfile p;
    auto q=seed(); ASSERT_TRUE(p.seed(q));
    const double target=q[4]-6.0*M_PI/180.0;
    ASSERT_TRUE(p.request(4,target));
    EXPECT_LT(p.axis(4).duration,1.1);
    EXPECT_GT(p.axis(4).duration,.7);
    EXPECT_LE(p.axis(4).velocity_bound,.23*(1+1e-10));
    EXPECT_LE(p.axis(4).acceleration_bound,1.0*(1+1e-10));
    EXPECT_LE(p.axis(4).jerk_bound,10.0*(1+1e-10));
    uint16_t previous=sdk_joint_position_encode(q[4]);
    for (int tick=0; tick<200; ++tick) {
        p.advance(.02);
        auto s=p.sample(4);
        auto word=sdk_joint_position_encode(s.position);
        EXPECT_LE(word,previous);
        EXPECT_LE((previous-word)*quantum,.23*.02+quantum+1e-12);
        EXPECT_GE(s.position,p.axis(4).lower-1e-12);
        EXPECT_LE(s.position,p.axis(4).upper+1e-12);
        EXPECT_LE(std::abs(s.velocity),.23+1e-10);
        EXPECT_LE(std::abs(s.acceleration),1.0+1e-10);
        EXPECT_LE(std::abs(s.jerk),10.0+1e-10);
        for (size_t i=0;i<6;++i) { if(i!=4) { EXPECT_EQ(sdk_joint_position_encode(p.sample(i).position),sdk_joint_position_encode(q[i])); } }
        previous=word;
    }
    EXPECT_EQ(previous,sdk_joint_position_encode(target));
    EXPECT_DOUBLE_EQ(p.sample(4).velocity,0.0);
    EXPECT_DOUBLE_EQ(p.sample(4).acceleration,0.0);
}

TEST(GuiDirectProfileTest, LargeDeltaAndLastQuantumTerminateWithoutHunting) {
    GuiDirectProfile p; auto q=seed(); q[0]=sdk_joint_position_decode(1);
    ASSERT_TRUE(p.seed(q)); ASSERT_TRUE(p.request(0,sdk_joint_position_decode(4094)));
    EXPECT_LE(p.axis(0).duration,60.0);
    finish(p); EXPECT_EQ(sdk_joint_position_encode(p.sample(0).position),4094);
    ASSERT_TRUE(p.request(0,sdk_joint_position_decode(4095)));
    finish(p); EXPECT_EQ(sdk_joint_position_encode(p.sample(0).position),4095);
    const auto completed=p.axis(0);
    for(int i=0;i<50;++i) { EXPECT_TRUE(p.request(0,M_PI)); p.advance(.02); }
    EXPECT_DOUBLE_EQ(p.axis(0).duration,completed.duration);
    EXPECT_DOUBLE_EQ(p.axis(0).elapsed,completed.elapsed);
    EXPECT_EQ(p.axis(0).target_word,4095);
}

TEST(GuiDirectProfileTest, RepeatedSubQuantumTargetDoesNotRestartAndOtherEditDoesNotReplan) {
    GuiDirectProfile p; auto q=seed(); ASSERT_TRUE(p.seed(q));
    ASSERT_TRUE(p.request(4,q[4]+.1));
    for(int i=0;i<10;++i)p.advance(.02);
    const auto first=p.axis(4); const auto s=p.sample(4);
    ASSERT_TRUE(p.request(4,sdk_joint_position_decode(first.target_word)+.1*quantum));
    EXPECT_EQ(p.axis(4).control,first.control);
    EXPECT_DOUBLE_EQ(p.axis(4).elapsed,first.elapsed);
    ASSERT_TRUE(p.request(1,q[1]+.05));
    EXPECT_EQ(p.axis(4).control,first.control);
    EXPECT_DOUBLE_EQ(p.sample(4).position,s.position);
    EXPECT_DOUBLE_EQ(p.sample(4).velocity,s.velocity);
}

TEST(GuiDirectProfileTest, FeasibleRetargetPreservesPositionVelocityAcceleration) {
    GuiDirectProfile p; auto q=seed(); ASSERT_TRUE(p.seed(q));
    ASSERT_TRUE(p.request(4,q[4]+.1));
    for(int i=0;i<12;++i)p.advance(.02);
    const auto before=p.sample(4);
    ASSERT_TRUE(p.request(4,q[4]+.15));
    const auto after=p.sample(4);
    EXPECT_NEAR(after.position,before.position,1e-12);
    EXPECT_NEAR(after.velocity,before.velocity,1e-12);
    EXPECT_NEAR(after.acceleration,before.acceleration,1e-12);
    finish(p);
    EXPECT_EQ(sdk_joint_position_encode(p.sample(4).position),sdk_joint_position_encode(q[4]+.15));
}

TEST(GuiDirectProfileTest, ReversalEitherBrakesWithinAuthorizedEnvelopeOrRejectsWithoutQueue) {
    for(int phase : {1,5,10,20,30,40}) {
        GuiDirectProfile p; auto q=seed(); ASSERT_TRUE(p.seed(q));
        ASSERT_TRUE(p.request(4,q[4]+.1));
        for(int i=0;i<phase;++i)p.advance(.02);
        const auto before=p.sample(4); const auto axis=p.axis(4);
        const double next=q[4]-.1;
        if (p.request(4,next)) {
            EXPECT_NEAR(p.sample(4).position,before.position,1e-12);
            EXPECT_NEAR(p.sample(4).velocity,before.velocity,1e-12);
            EXPECT_NEAR(p.sample(4).acceleration,before.acceleration,1e-12);
            for(int i=0;i<500;++i) {
                p.advance(.02); auto s=p.sample(4);
                EXPECT_GE(s.position,sdk_joint_position_decode(sdk_joint_position_encode(next))-1e-12);
                EXPECT_LE(s.position,sdk_joint_position_decode(axis.target_word)+1e-12);
                EXPECT_LE(std::abs(s.velocity),.23+1e-10);
                EXPECT_LE(std::abs(s.acceleration),1.0+1e-10);
                EXPECT_LE(std::abs(s.jerk),10.0+1e-10);
            }
        } else {
            EXPECT_EQ(p.axis(4).control,axis.control);
            EXPECT_DOUBLE_EQ(p.axis(4).elapsed,axis.elapsed);
            finish(p); EXPECT_EQ(p.axis(4).target_word,axis.target_word);
        }
    }
}

TEST(GuiDirectProfileTest, InfeasibleNewTargetCannotReplaceAlreadyAdmittedGoal) {
    GuiDirectProfileLimits limits; limits.maximum_duration=1.0;
    GuiDirectProfile p(limits); auto q=seed(); ASSERT_TRUE(p.seed(q));
    ASSERT_TRUE(p.request(4,q[4]+.025)); p.advance(.02);
    const auto before=p.axis(4);
    EXPECT_FALSE(p.request(4,2.0));
    EXPECT_EQ(p.axis(4).control,before.control);
    EXPECT_DOUBLE_EQ(p.axis(4).elapsed,before.elapsed);
    finish(p); EXPECT_EQ(p.axis(4).target_word,before.target_word);
}

TEST(GuiDirectProfileTest, WriteFailureAndClockStallCannotAdvanceToUnsubmittedGoal) {
    GuiDirectProfile committed; auto q=seed(); ASSERT_TRUE(committed.seed(q));
    ASSERT_TRUE(committed.request(4,q[4]+.1));
    auto failed_write=committed; failed_write.advance(.02);
    EXPECT_DOUBLE_EQ(committed.axis(4).elapsed,0.0);
    auto after_stall=committed; after_stall.advance(600.0);
    EXPECT_DOUBLE_EQ(after_stall.axis(4).elapsed,.02);
    EXPECT_DOUBLE_EQ(after_stall.sample(4).position,failed_write.sample(4).position);
    committed=after_stall; // only successful write/identical already-sent words commit
    EXPECT_DOUBLE_EQ(committed.axis(4).elapsed,.02);
    committed.advance(-1); committed.advance(std::numeric_limits<double>::quiet_NaN());
    EXPECT_DOUBLE_EQ(committed.axis(4).elapsed,.02);
    committed.clear(); EXPECT_FALSE(committed.initialized());
    EXPECT_TRUE(committed.positions().empty());
}

TEST(GuiDirectProfileTest, InitialProbeUsesOriginalResponseAndTimeoutGates) {
    for(bool measured_motion : {false,true}) {
        GuiDirectProfile p; auto q=std::vector<double>(6,0.0); ASSERT_TRUE(p.seed(q));
        ASSERT_TRUE(p.request(5,.025)); EXPECT_LT(p.axis(5).duration,.7);
        ActuationConfirmation confirmation;
        confirmation.reset_for_positive_enable(10.0);
        confirmation.note_feedback(q,10.01);
        std::string reason; ASSERT_TRUE(confirmation.admit_command(q,10.02,&reason));
        confirmation.note_streamed_target(q,10.02);
        for(int tick=1; tick<=100; ++tick) {
            const double stamp=10.02+.02*tick;
            p.advance(.02);
            auto wire=p.positions();
            for(double& v:wire) v=sdk_joint_position_decode(sdk_joint_position_encode(v));
            confirmation.note_streamed_target(wire,stamp);
            if(tick%5==0) confirmation.note_feedback(measured_motion?wire:q,stamp+.001);
            confirmation.update(stamp+.001);
        }
        if(measured_motion) EXPECT_EQ(confirmation.state(),ActuationState::CONFIRMED);
        else { EXPECT_TRUE(confirmation.initial_probe_failed()); EXPECT_EQ(confirmation.state(),ActuationState::UNCONFIRMED); }
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
