#include <gtest/gtest.h>
#include "alicia_d_driver/physical_reference_contract.hpp"
using namespace alicia_d_driver;

// Historical numbers are a fixture, never a branch in production admission.
// The test certifies software coordinates and transaction rules, not actuation.
TEST(PhysicalReferenceContract, CompleteSixAxisRecordKeepsRawAndPhysicalErrorsDistinct) {
    const ReferenceWords wire{{846,1747,2698,2003,1339,757}};
    const ReferenceWords encoder{{845,1714,2695,1999,1338,762}};
    const std::array<int,6> offset{{-6,13,11,-5,-4,-4}};
    const std::array<int,6> physical_error{{7,20,-8,9,5,-1}};
    const double q=2*M_PI/4096.;
    PhysicalReferenceContract c;ReferenceBinding b;ReferenceEvidence e;
    ReferenceClock now{10000000000ULL,100.};
    e.connected=e.confirmed=e.pending_complete=e.action_idle=true;e.manual=false;
    e.epoch=1000000000ULL;e.sdk=wire;e.controller=c.decoded(wire);e.encoder=e.controller;
    e.sdk_stamp=e.encoder_stamp=e.controller_stamp=9990000000ULL;
    e.stable_since=9600000000ULL;e.sdk_samples=e.encoder_samples=5;
    b.id="record-fixture";b.owner="one-task";b.episode="nominal-pregrasp";
    b.epoch=e.epoch;b.generation=2;b.plan=std::string(64,'a');b.model=std::string(64,'b');
    b.baseline=wire;b.sdk_stamp=e.sdk_stamp;b.encoder_stamp=e.encoder_stamp;b.issued=now.ros_ns;
    b.deadline=b.episode_deadline=now.ros_ns+200000000000ULL;b.duration=b.episode_duration=200.;
    for(size_t i=0;i<6;++i){b.bias[i]=offset[i]*q;e.encoder[i]-=b.bias[i];}
    ASSERT_TRUE(c.prepare(b,1,now,e).ok);
    // PREPARE is output-gated; changing only software coordinates cannot write.
    ASSERT_TRUE(c.gated());now.ros_ns+=1000000ULL;now.monotonic+=.001;
    e.controller_stamp=e.sdk_stamp=e.encoder_stamp=now.ros_ns;e.controller=c.physical_baseline();
    ASSERT_EQ(c.words(c.map(e.controller)),wire);ASSERT_TRUE(c.commit(2,b.id,b.owner,now,e).ok);
    e.encoder=c.decoded(encoder);
    EXPECT_TRUE(c.physical_following(e));
    for(size_t i=0;i<6;++i)EXPECT_NEAR(e.controller[i]-e.encoder[i],physical_error[i]*q,1e-12);
    EXPECT_NEAR(c.decoded(wire)[1]-e.encoder[1],33*q,1e-12);
    EXPECT_GT(std::abs(c.decoded(wire)[1]-e.encoder[1]),.035);
    EXPECT_LT(std::abs(e.controller[1]-e.encoder[1]),.035);
}

TEST(PhysicalReferenceContract, EverySdkCellIsRejectedIfMapWouldClip) {
    PhysicalReferenceContract c;
    const double q=2*M_PI/4096.;
    for(int direction:{-1,1}) {
        ReferenceVector physical{};physical.fill(sdk_joint_position_decode(direction<0?0:4095));
        c.binding.bias.fill(direction*q);
        EXPECT_FALSE(c.finite(c.map(physical)));
    }
}

TEST(PhysicalReferenceContract, EncoderFeedbackIsNeverACommandSpaceAccessor) {
    ReferenceEvidence evidence;evidence.controller.fill(0.);evidence.encoder.fill(.036);
    EXPECT_FALSE(PhysicalReferenceContract::physical_following(evidence));
    evidence.encoder.fill(.035);EXPECT_TRUE(PhysicalReferenceContract::physical_following(evidence));
    evidence.encoder[3]=NAN;EXPECT_FALSE(PhysicalReferenceContract::physical_following(evidence));
}

TEST(PhysicalReferenceContract, RosClockAdvanceCannotStartBeforeOriginalMonotonicStart) {
    auto local=[](){PhysicalReferenceContract c;c.phase=ReferencePhase::EXECUTING;
        c.binding.id="binding";c.binding.owner="owner";c.binding.epoch=1;
        c.binding.issued=1;c.binding.deadline=c.binding.episode_deadline=20000000000ULL;
        c.monotonic_deadline=110.;c.goal.token="goal";c.goal_observed=true;
        c.goal.start=10900000000ULL;c.goal.end=12000000000ULL;
        c.goal_monotonic_start=101.;c.goal_monotonic_end=102.;return c;};
    ReferenceEvidence e;e.connected=e.confirmed=true;e.manual=false;e.epoch=1;
    e.sdk_stamp=e.encoder_stamp=11000000000ULL;e.sdk.fill(2048);
    ReferenceVector changed{};changed.fill(2*M_PI/4096.);
    auto early=local();auto result=early.admit_command("binding","owner","goal",1,1,
        changed,changed,{11000000000ULL,100.5},e);
    EXPECT_FALSE(result.ok);EXPECT_EQ(result.code,"REFERENCE_COMMAND_BEFORE_TRAJECTORY_START");
    auto ontime=local();EXPECT_TRUE(ontime.admit_command("binding","owner","goal",1,1,
        changed,changed,{11000000000ULL,101.1},e).ok);
}

TEST(PhysicalReferenceContract, ObservedGoalCannotBeOverwrittenByLaterIdleStatus) {
    ReferenceObservedGoals pending;ASSERT_TRUE(pending.idle());
    ASSERT_TRUE(pending.observe("legacy-new-goal",17).ok);
    const bool fresh_status_reports_idle=true;
    EXPECT_FALSE(fresh_status_reports_idle && pending.idle());
    ASSERT_TRUE(pending.terminal("legacy-new-goal",17,3).ok);
    EXPECT_TRUE(fresh_status_reports_idle && pending.idle());
}

TEST(PhysicalReferenceContract, OnlyExactIdentityAndTerminalStatusRemovePendingGoal) {
    ReferenceObservedGoals pending;ASSERT_TRUE(pending.observe("first",17).ok);
    ASSERT_TRUE(pending.observe("second",19).ok);
    EXPECT_FALSE(pending.terminal("wrong",17,3).ok);
    EXPECT_FALSE(pending.terminal("first",18,3).ok);
    EXPECT_FALSE(pending.terminal("first",17,1).ok);EXPECT_EQ(pending.size(),2u);
    EXPECT_TRUE(pending.terminal("first",17,4).ok);EXPECT_FALSE(pending.idle());
    EXPECT_FALSE(pending.terminal("first",17,4).ok);EXPECT_EQ(pending.size(),1u);
    EXPECT_TRUE(pending.terminal("second",19,2).ok);EXPECT_TRUE(pending.idle());
}

TEST(PhysicalReferenceContract, DuplicateObservedGoalCannotBecomeFreshAfterTerminal) {
    ReferenceObservedGoals pending;ASSERT_TRUE(pending.observe("same",17).ok);
    EXPECT_FALSE(pending.observe("same",18).ok);EXPECT_EQ(pending.size(),1u);
    EXPECT_TRUE(pending.terminal("same",17,3).ok);EXPECT_FALSE(pending.idle());
    EXPECT_FALSE(pending.observe("new",20).ok);
}

TEST(PhysicalReferenceContract, PendingCapacityNeverDiscardsUnfinishedGoal) {
    ReferenceObservedGoals pending;
    for(size_t i=0;i<ReferenceObservedGoals::maximum_pending;++i)
        ASSERT_TRUE(pending.observe("goal-"+std::to_string(i),i+1).ok);
    EXPECT_FALSE(pending.observe("overflow",1000).ok);
    EXPECT_EQ(pending.size(),size_t(ReferenceObservedGoals::maximum_pending));
    for(size_t i=0;i<ReferenceObservedGoals::maximum_pending;++i)
        ASSERT_TRUE(pending.terminal("goal-"+std::to_string(i),i+1,3).ok);
    EXPECT_EQ(pending.size(),0u);EXPECT_FALSE(pending.idle());
}

TEST(PhysicalReferenceContract, PendingGripperInterpolationCannotClaimCompletedWireHold) {
    const double written=sdk_gripper_position_decode(500);
    EXPECT_TRUE(reference_gripper_hold_complete(written,written,written,0.));
    EXPECT_FALSE(reference_gripper_hold_complete(written,sdk_gripper_position_decode(501),written,0.));
    EXPECT_FALSE(reference_gripper_hold_complete(written,written,sdk_gripper_position_decode(499),0.));
    EXPECT_FALSE(reference_gripper_hold_complete(written,written,written,.001));
    EXPECT_FALSE(reference_gripper_hold_complete(written,NAN,written,0.));
}

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
