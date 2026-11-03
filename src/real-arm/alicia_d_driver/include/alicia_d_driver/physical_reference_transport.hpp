#pragma once
// Typed wire-format validation only: no NodeHandle, ROS master or device I/O.
#include "alicia_d_driver/physical_reference_contract.hpp"
#include <alicia_d_driver/SetPhysicalReference.h>
#include <alicia_d_driver/PhysicalReferenceState.h>
#include <ros/serialization.h>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>

namespace alicia_d_driver {
template<class T> inline std::vector<uint8_t> reference_serialized(const T& msg) {
    std::vector<uint8_t> b(ros::serialization::serializationLength(msg));
    ros::serialization::OStream stream(b.data(),b.size());
    ros::serialization::serialize(stream,msg);return b;
}
template<class T> inline std::string reference_sha256(const T& msg) {
    auto b=reference_serialized(msg);unsigned char out[SHA256_DIGEST_LENGTH];
    SHA256(b.data(),b.size(),out);std::ostringstream s;
    for(auto v:out)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);return s.str();
}
inline std::vector<uint8_t> reference_action_envelope(control_msgs::FollowJointTrajectoryActionGoal goal) {
    // rospy Publisher rewrites this one transport counter immediately before
    // serialization. All control identity, stamps, nested headers and goal
    // bytes remain exact; phase/id ownership independently rejects replay.
    goal.header.seq=0;return reference_serialized(goal);
}
inline bool reference_peer_allows_output(const PhysicalReferenceState& peer,
        const PhysicalReferenceContract& local,const ReferenceClock& now) {
    if(peer.binding_id!=local.binding.id || peer.owner_token!=local.binding.owner
       || peer.epoch_ns!=local.binding.epoch || !peer.evidence_valid || peer.output_gated
       || !PhysicalReferenceContract::fresh(peer.header.stamp.toNSec(),now))return false;
    if(local.phase==ReferencePhase::ACTIVE_IDLE)
        return peer.phase==PhysicalReferenceState::ACTIVE_IDLE;
    if(local.phase==ReferencePhase::EXECUTING)
        return peer.phase==PhysicalReferenceState::EXECUTING && peer.goal_observed
            && peer.action_goal_id==local.goal.id && peer.action_goal_stamp_ns==local.goal.stamp
            && peer.goal_token==local.goal.token;
    return false;
}
inline ReferenceOutcome bind_physical_reference_goal(const SetPhysicalReference::Request& r,
        const ReferenceVector& bias,double configured_stop,ReferenceGoal& g) {
    const auto& tr=r.action_goal.goal.trajectory;const auto& wire=r.wire_trajectory;
    auto fail=[](){return ReferenceOutcome{false,"REFERENCE_ARM_TRAJECTORY_BINDING_INVALID"};};
    if(tr.joint_names.size()!=6 || tr.points.size()<2 || wire.joint_names!=tr.joint_names
       || wire.points.size()!=tr.points.size() || reference_serialized(wire.header)!=reference_serialized(tr.header)
       || !std::isfinite(configured_stop) || configured_stop<=0. || configured_stop>30.
       || !std::isfinite(r.stop_duration_sec) || r.stop_duration_sec!=configured_stop)return fail();
    for(size_t i=0;i<6;++i)if(tr.joint_names[i]!="Joint"+std::to_string(i+1))return fail();
    const double grace=r.action_goal.goal.goal_time_tolerance.toSec();
    if(!std::isfinite(grace)||grace<=0.||grace>30.
       || r.action_goal.goal.path_tolerance.size()!=6 || r.action_goal.goal.goal_tolerance.size()!=6)return fail();
    std::set<std::string> path_names,goal_names;
    for(const auto& tol:r.action_goal.goal.path_tolerance) {
        path_names.insert(tol.name);
        if(!std::isfinite(tol.position)||tol.position<=0.||tol.position>.09
           || tol.velocity!=0.||tol.acceleration!=0.)return fail();
    }
    for(const auto& tol:r.action_goal.goal.goal_tolerance) {
        goal_names.insert(tol.name);
        if(!std::isfinite(tol.position)||tol.position<=0.||tol.position>.035
           || !std::isfinite(tol.velocity)||tol.velocity<0.||tol.velocity>.03
           || tol.acceleration!=0.)return fail();
    }
    if(path_names.size()!=6||goal_names.size()!=6)return fail();
    for(size_t j=0;j<6;++j)if(!path_names.count("Joint"+std::to_string(j+1))
        || !goal_names.count("Joint"+std::to_string(j+1)))return fail();
    double previous=-1.;
    for(size_t i=0;i<tr.points.size();++i) {
        const auto& point=tr.points[i];const auto& wire_point=wire.points[i];
        if(point.positions.size()!=6 || wire_point.positions.size()!=6
           || !std::isfinite(point.time_from_start.toSec()) || point.time_from_start.toSec()<0.
           || point.time_from_start.toSec()<=previous || point.time_from_start!=wire_point.time_from_start
           || point.velocities!=wire_point.velocities || point.accelerations!=wire_point.accelerations
           || point.effort!=wire_point.effort)return fail();
        for(const auto* values:{&point.velocities,&point.accelerations,&point.effort})
            if(!values->empty() && (values->size()!=6 || std::any_of(values->begin(),values->end(),[](double v){return !std::isfinite(v);})))return fail();
        previous=point.time_from_start.toSec();ReferenceVector wire_q{},physical{},mapped{};
        for(size_t j=0;j<6;++j) {
            wire_q[j]=wire_point.positions[j];physical[j]=point.positions[j];
            if(physical[j]!=wire_q[j]-bias[j])return fail();
            mapped[j]=physical[j]+bias[j];
        }
        if(!PhysicalReferenceContract::finite(wire_q)||!PhysicalReferenceContract::finite(physical)
           ||!PhysicalReferenceContract::finite(mapped)
           ||PhysicalReferenceContract::words(mapped)!=PhysicalReferenceContract::words(wire_q))return fail();
    }
    if(reference_sha256(tr)!=r.physical_trajectory_sha256 || reference_sha256(wire)!=r.wire_trajectory_sha256)return fail();
    g.token=r.goal_token;g.id=r.action_goal.goal_id.id;g.stamp=r.action_goal.goal_id.stamp.toNSec();
    g.start=tr.header.stamp.toNSec();g.end=g.start+tr.points.back().time_from_start.toNSec()
        +r.action_goal.goal.goal_time_tolerance.toNSec()+static_cast<uint64_t>(std::ceil(configured_stop*1e9));
    g.proof=r.proof_sha256;g.physical_digest=r.physical_trajectory_sha256;g.wire_digest=r.wire_trajectory_sha256;
    g.envelope=reference_action_envelope(r.action_goal);return {true,"REFERENCE_ARM_TRAJECTORIES_BOUND"};
}
} // namespace alicia_d_driver
