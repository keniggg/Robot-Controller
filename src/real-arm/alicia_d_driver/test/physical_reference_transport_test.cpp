#include <gtest/gtest.h>
#include <fstream>
#include <regex>
#include "alicia_d_driver/physical_reference_transport.hpp"
using namespace alicia_d_driver;
namespace {
struct Fixture {
    SetPhysicalReference::Request request;ReferenceVector bias{};
    Fixture() {
        bias[1]=13.*2*M_PI/4096.;request.goal_token="one-use-goal";
        request.proof_sha256=std::string(64,'c');request.stop_duration_sec=.5;
        auto& a=request.action_goal;a.header.stamp=ros::Time(10,17);
        a.goal_id.id="/motion_gateway-exact-1";a.goal_id.stamp=ros::Time(10,11);
        a.goal.goal_time_tolerance=ros::Duration(8.);
        auto& wire=request.wire_trajectory;wire.header.stamp=ros::Time(11,0);
        wire.header.frame_id="base_link";wire.header.seq=7;
        for(size_t i=0;i<6;++i) {
            wire.joint_names.push_back("Joint"+std::to_string(i+1));
            control_msgs::JointTolerance t;t.name=wire.joint_names.back();t.position=.09;
            a.goal.path_tolerance.push_back(t);t.position=.035;t.velocity=.03;
            a.goal.goal_tolerance.push_back(t);
        }
        for(int i=0;i<2;++i) {
            trajectory_msgs::JointTrajectoryPoint p;
            p.positions={-.4,-.49451705649463357,.5,-.1,.2,-.6};
            p.positions[0]+=.01*i;p.velocities.assign(6,0.);p.accelerations.assign(6,0.);
            p.time_from_start=ros::Duration(double(i));wire.points.push_back(p);
        }
        a.goal.trajectory=wire;
        for(auto& p:a.goal.trajectory.points)for(size_t j=0;j<6;++j)p.positions[j]-=bias[j];
        hashes();
    }
    void hashes(){request.wire_trajectory_sha256=reference_sha256(request.wire_trajectory);
        request.physical_trajectory_sha256=reference_sha256(request.action_goal.goal.trajectory);}
    bool valid(){ReferenceGoal g;return bind_physical_reference_goal(request,bias,.5,g).ok;}
};
}
TEST(PhysicalReferenceTransport, HistoricalWireBytesSurviveFloatingPointRoundtrip) {
    Fixture f;ASSERT_TRUE(f.valid());
    const auto& a=f.request.action_goal.goal.trajectory;
    auto roundtrip=a;for(auto& p:roundtrip.points)for(size_t j=0;j<6;++j)p.positions[j]+=f.bias[j];
    EXPECT_NE(roundtrip.points[0].positions[1],f.request.wire_trajectory.points[0].positions[1]);
    EXPECT_NE(reference_sha256(roundtrip),f.request.wire_trajectory_sha256);
    EXPECT_EQ(f.request.wire_trajectory.points[0].positions[1],-.49451705649463357);
}
TEST(PhysicalReferenceTransport, OnlyOuterPublisherSequenceIsTransportMetadata) {
    Fixture f;auto a=f.request.action_goal;auto b=a;b.header.seq=137;
    EXPECT_EQ(reference_action_envelope(a),reference_action_envelope(b));
    b=a;b.header.stamp.nsec++;EXPECT_NE(reference_action_envelope(a),reference_action_envelope(b));
    b=a;b.goal_id.stamp.nsec++;EXPECT_NE(reference_action_envelope(a),reference_action_envelope(b));
    b=a;b.goal_id.id+="other";EXPECT_NE(reference_action_envelope(a),reference_action_envelope(b));
    b=a;b.goal.trajectory.header.seq++;EXPECT_NE(reference_action_envelope(a),reference_action_envelope(b));
}
TEST(PhysicalReferenceTransport, RehashedWireCannotChangePhysicalMapping) {
    Fixture f;f.request.wire_trajectory.points[1].positions[1]+=.002;f.hashes();EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, RehashedPhysicalCannotHideChangedReference) {
    Fixture f;f.request.action_goal.goal.trajectory.points[1].positions[1]+=.002;f.hashes();EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, BothPathsBindHeadersTimesAndDerivatives) {
    Fixture f;f.request.wire_trajectory.header.stamp.nsec++;f.hashes();EXPECT_FALSE(f.valid());
    f=Fixture();f.request.wire_trajectory.points[1].time_from_start.nsec++;f.hashes();EXPECT_FALSE(f.valid());
    f=Fixture();f.request.wire_trajectory.points[1].velocities[3]=.01;f.hashes();EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, OriginalJointAndStopBoundsRemainRequired) {
    Fixture f;f.request.action_goal.goal.goal_tolerance[1].position=.0507;EXPECT_FALSE(f.valid());
    f=Fixture();f.request.action_goal.goal.path_tolerance[1].position=.1;EXPECT_FALSE(f.valid());
    f=Fixture();f.request.action_goal.goal.goal_tolerance[1].name="Joint1";EXPECT_FALSE(f.valid());
    f=Fixture();f.request.stop_duration_sec=.6;EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, MappedSaturationCannotPassByRehashingBothPaths) {
    Fixture f;f.request.wire_trajectory.points[1].positions[1]=M_PI;
    f.request.action_goal.goal.trajectory.points[1].positions[1]=M_PI-f.bias[1];f.hashes();EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, NonfiniteDerivativeIsRejectedWithoutTransport) {
    Fixture f;f.request.wire_trajectory.points[1].accelerations[0]=NAN;
    f.request.action_goal.goal.trajectory.points[1].accelerations[0]=NAN;f.hashes();EXPECT_FALSE(f.valid());
}
TEST(PhysicalReferenceTransport, CommitWaitsForActualPeerPhaseWithoutChangingLocalLease) {
    PhysicalReferenceContract local;local.phase=ReferencePhase::ACTIVE_IDLE;
    local.binding.id="binding";local.binding.owner="owner";local.binding.epoch=1;
    local.monotonic_deadline=30.;local.request_sequence=2;
    PhysicalReferenceState peer;peer.binding_id="binding";peer.owner_token="owner";
    peer.epoch_ns=1;peer.header.stamp=ros::Time(10,0);peer.evidence_valid=true;
    peer.phase=PhysicalReferenceState::PREPARED;peer.output_gated=true;
    const ReferenceClock now{10000000000ULL,10.};
    EXPECT_FALSE(reference_peer_allows_output(peer,local,now));
    EXPECT_EQ(local.phase,ReferencePhase::ACTIVE_IDLE);EXPECT_EQ(local.monotonic_deadline,30.);
    EXPECT_EQ(local.request_sequence,2u);
    peer.phase=PhysicalReferenceState::ACTIVE_IDLE;peer.output_gated=false;
    EXPECT_TRUE(reference_peer_allows_output(peer,local,now));
    peer.phase=PhysicalReferenceState::FAULT;EXPECT_FALSE(reference_peer_allows_output(peer,local,now));
    peer.phase=PhysicalReferenceState::ACTIVE_IDLE;peer.owner_token="other";
    EXPECT_FALSE(reference_peer_allows_output(peer,local,now));
}
TEST(PhysicalReferenceTransport, ExecutionWaitsForSameActualGoalAndFreshPeerReceipt) {
    PhysicalReferenceContract local;local.phase=ReferencePhase::EXECUTING;
    local.binding.id="binding";local.binding.owner="owner";local.binding.epoch=1;
    local.goal.id="goal";local.goal.token="one-use";local.goal.stamp=9000000000ULL;
    PhysicalReferenceState peer;peer.binding_id="binding";peer.owner_token="owner";peer.epoch_ns=1;
    peer.header.stamp=ros::Time(10,0);peer.evidence_valid=true;peer.phase=PhysicalReferenceState::EXECUTING;
    peer.goal_observed=true;peer.action_goal_id="goal";peer.goal_token="one-use";peer.action_goal_stamp_ns=9000000000ULL;
    const ReferenceClock now{10000000000ULL,10.};
    EXPECT_TRUE(reference_peer_allows_output(peer,local,now));
    peer.goal_observed=false;EXPECT_FALSE(reference_peer_allows_output(peer,local,now));peer.goal_observed=true;
    peer.action_goal_stamp_ns++;EXPECT_FALSE(reference_peer_allows_output(peer,local,now));peer.action_goal_stamp_ns--;
    peer.goal_token="replay";EXPECT_FALSE(reference_peer_allows_output(peer,local,now));peer.goal_token="one-use";
    peer.header.stamp=ros::Time(9,0);EXPECT_FALSE(reference_peer_allows_output(peer,local,now));
}
TEST(PhysicalReferenceTransport, ProductionCallerIdentitiesMatchLaunchAndAcceptanceOwners) {
    const std::string root=ALICIA_REFERENCE_WORKSPACE_ROOT;
    auto read=[&](const std::string& relative){std::ifstream stream(root+"/"+relative);
        EXPECT_TRUE(stream.good())<<relative;return std::string(std::istreambuf_iterator<char>(stream),{});};
    auto node=[&](const std::string& text,const std::string& executable){
        const std::regex tag("<node\\b[^>]*\\btype\\s*=\\s*\""+executable+"\"[^>]*>");
        std::smatch match;EXPECT_TRUE(std::regex_search(text,match,tag))<<executable;
        if(match.empty())return std::string();const std::string element=match.str();
        const std::regex name("\\bname\\s*=\\s*\"([^\"]+)\"");
        EXPECT_TRUE(std::regex_search(element,match,name));
        return match.size()>1?"/"+match[1].str():std::string();};
    const auto driver_launch=read("src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch");
    EXPECT_EQ(node(driver_launch,"alicia_d_driver_node"),reference_driver_node);
    EXPECT_EQ(node(driver_launch,"alicia_d_hw_interface_node"),reference_hardware_node);
    const auto gateway_launch=read("src/alicia_flexible_grasp_supervisor/launch/bringup.launch");
    EXPECT_EQ(node(gateway_launch,"motion_gateway_node.py"),reference_gateway_node);
    const auto acceptance=read("tools/full_stack_start.py");
    const std::regex owner("['\"]/alicia_d/accepted_joint_states['\"]\\s*:\\s*['\"]"
        +std::string(reference_driver_node)+"['\"]");
    EXPECT_TRUE(std::regex_search(acceptance,owner));
    EXPECT_NE(acceptance.find("result['"+std::string(reference_gateway_node)+"']"),std::string::npos);
}
TEST(PhysicalReferenceTransport, TopicNamespacesAndServiceNamesAreNotNodeIdentities) {
    EXPECT_TRUE(reference_driver_caller("/alicia_d_driver_node"));
    EXPECT_FALSE(reference_driver_caller("/alicia_d"));
    EXPECT_FALSE(reference_driver_caller("/alicia_d/physical_reference"));
    EXPECT_TRUE(reference_hardware_caller("/bessica_d_hw_interface"));
    EXPECT_FALSE(reference_hardware_caller("/alicia_controller"));
    EXPECT_TRUE(reference_gateway_caller("/motion_gateway"));
    EXPECT_FALSE(reference_gateway_caller("/move_group_commander_wrappers_123"));
    EXPECT_FALSE(reference_gateway_caller("/motion_gateway/physical_reference"));
}
int main(int argc,char** argv) {::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
