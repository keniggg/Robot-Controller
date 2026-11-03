#pragma once
// Pure transaction kernel. No ROS, serial I/O, torque operation or clock reads.
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <algorithm>
#include "alicia_d_driver/sdk_position_codec.hpp"

namespace alicia_d_driver {
// Node identities from the production bringup launches, not topic namespaces.
constexpr char reference_driver_node[]="/alicia_d_driver_node";
constexpr char reference_hardware_node[]="/bessica_d_hw_interface";
constexpr char reference_gateway_node[]="/motion_gateway";
inline bool reference_driver_caller(const std::string& caller){return caller==reference_driver_node;}
inline bool reference_hardware_caller(const std::string& caller){return caller==reference_hardware_node;}
inline bool reference_gateway_caller(const std::string& caller){return caller==reference_gateway_node;}
using ReferenceVector = std::array<double, 6>;
using ReferenceWords = std::array<uint16_t, 6>;
enum class ReferencePhase : uint8_t {
    LEGACY=0, PREPARED=1, ACTIVE_IDLE=2, ARMED=3, EXECUTING=4,
    RELEASE_PENDING=5, FAULT=6, MANUAL=7
};
struct ReferenceClock { uint64_t ros_ns=0; double monotonic=0.; };
struct ReferenceBinding {
    std::string id, previous_id, owner, plan, model, episode;
    uint64_t epoch=0, generation=0, sdk_stamp=0, encoder_stamp=0;
    uint64_t issued=0, deadline=0, episode_deadline=0;
    double duration=0., episode_duration=0.;
    ReferenceVector bias{};
    ReferenceWords baseline{};
};
struct ReferenceGoal {
    std::string token, id, proof, physical_digest, wire_digest;
    uint64_t stamp=0, start=0, end=0;
    std::vector<uint8_t> envelope;
};
struct ReferenceEvidence {
    bool connected=false, confirmed=false, manual=true, manual_reference=false;
    bool pending_complete=false, action_idle=false;
    uint64_t epoch=0, sdk_stamp=0, encoder_stamp=0, controller_stamp=0;
    uint64_t stable_since=0;
    size_t sdk_samples=0, encoder_samples=0;
    ReferenceWords sdk{};
    ReferenceVector encoder{}, controller{}, velocity{}, acceleration{};
};
struct ReferenceOutcome { bool ok; std::string code; };

inline bool reference_gripper_hold_complete(double written,double target,double current,double velocity) {
    for(double value:{written,target,current})
        if(!std::isfinite(value) || value<0. || value>sdk_gripper_position_decode(1000))return false;
    return std::isfinite(velocity) && velocity==0.
        && sdk_gripper_position_encode(written)==sdk_gripper_position_encode(target)
        && sdk_gripper_position_encode(written)==sdk_gripper_position_encode(current);
}

// Status and goal/result travel on separate ROS connections. A fresh idle
// status cannot erase a goal already observed on the goal connection.
class ReferenceObservedGoals {
public:
    static constexpr size_t maximum_pending=64;
    ReferenceOutcome observe(const std::string& id,uint64_t stamp) {
        if(blocked_)return {false,"REFERENCE_OBSERVED_GOALS_BLOCKED"};
        if(id.empty() || id.size()>1024 || stamp==0 || pending_.count(id)
           || pending_.size()>=maximum_pending) {
            blocked_=true;return {false,"REFERENCE_OBSERVED_GOAL_INVALID_REPLAY_OR_CAPACITY"};
        }
        pending_.emplace(id,stamp);return {true,"REFERENCE_GOAL_PENDING_TERMINAL"};
    }
    ReferenceOutcome terminal(const std::string& id,uint64_t stamp,int status) {
        const auto found=pending_.find(id);
        if(found==pending_.end() || found->second!=stamp
           || (status!=2 && status!=3 && status!=4 && status!=5 && status!=8 && status!=9))
            return {false,"REFERENCE_OBSERVED_TERMINAL_IDENTITY_OR_STATUS"};
        pending_.erase(found);return {true,"REFERENCE_OBSERVED_GOAL_TERMINATED"};
    }
    bool idle() const {return !blocked_ && pending_.empty();}
    size_t size() const {return pending_.size();}
private:
    std::map<std::string,uint64_t> pending_;
    // Never discard unknown in-flight work to make room. Restart is required
    // after malformed/replayed/overflowed evidence; time cannot clear this.
    bool blocked_=false;
};

class PhysicalReferenceContract {
public:
    ReferencePhase phase=ReferencePhase::LEGACY;
    ReferenceBinding binding;
    ReferenceGoal goal;
    uint64_t request_sequence=0, command_sequence=0, prepared_ns=0;
    double monotonic_deadline=0., outer_monotonic_deadline=0.;
    double goal_monotonic_start=0.,goal_monotonic_end=0.;
    bool goal_observed=false, goal_terminal=false;
    int goal_status=0, goal_error=0;
    std::string code="LEGACY";

    static bool finite(const ReferenceVector& q) {
        for (double v:q) if (!std::isfinite(v) || (v < sdk_joint_position_decode(0) || v > sdk_joint_position_decode(4095))) return false;
        return true;
    }
    static ReferenceWords words(const ReferenceVector& q) {
        ReferenceWords out{};
        for (size_t i=0;i<6;++i) out[i]=sdk_joint_position_encode(q[i]);
        return out;
    }
    static ReferenceVector decoded(const ReferenceWords& q) {
        ReferenceVector out{};
        for (size_t i=0;i<6;++i) out[i]=sdk_joint_position_decode(q[i]);
        return out;
    }
    static bool digest(const std::string& s) {
        return s.size()==64 && s.find_first_not_of("0123456789abcdef")==std::string::npos;
    }
    static bool fresh(uint64_t stamp,const ReferenceClock& now) {
        return stamp>0 && stamp<=now.ros_ns && now.ros_ns-stamp<=500000000ULL;
    }
    static bool stable(const ReferenceEvidence& e,const ReferenceClock& now) {
        if (!e.connected || !e.confirmed || e.manual || !e.pending_complete || !e.action_idle
            || !fresh(e.sdk_stamp,now) || !fresh(e.encoder_stamp,now)
            || !fresh(e.controller_stamp,now) || e.stable_since==0
            || std::min(e.sdk_stamp,e.encoder_stamp)<e.stable_since+300000000ULL
            || e.sdk_samples<3 || e.encoder_samples<3 || !finite(e.encoder)
            || !finite(e.controller)) return false;
        for(size_t i=0;i<6;++i)
            if (!std::isfinite(e.velocity[i]) || !std::isfinite(e.acceleration[i])
                || e.velocity[i]!=0. || e.acceleration[i]!=0.) return false;
        return true;
    }
    static bool physical_following(const ReferenceEvidence& e) {
        for(size_t i=0;i<6;++i)
            if(!std::isfinite(e.controller[i]) || !std::isfinite(e.encoder[i])
               || std::abs(e.controller[i]-e.encoder[i])>.035)return false;
        return true;
    }
    ReferenceOutcome fault(const std::string& why,bool manual=false) {
        released_success_=false;manual_cleanup_=false;
        if (phase!=ReferencePhase::LEGACY) {
            phase=manual ? ReferencePhase::MANUAL:ReferencePhase::FAULT;
            code=why;
        }
        return {false,why};
    }
    ReferenceOutcome check(const ReferenceClock& now,const ReferenceEvidence& e) {
        if (phase==ReferencePhase::LEGACY) return {true,"LEGACY"};
        if (manual_cleanup_ && phase==ReferencePhase::RELEASE_PENDING) {
            if(manual_clearable(now,e))return {true,"REFERENCE_MANUAL_SOFTWARE_CLEANUP"};
            return fault("REFERENCE_MANUAL_CLEANUP_SOURCE_CHANGED");
        }
        if (phase==ReferencePhase::FAULT || phase==ReferencePhase::MANUAL) return {false,code};
        if (!std::isfinite(now.monotonic) || now.ros_ns<binding.issued
            || now.ros_ns>=binding.deadline || now.ros_ns>=binding.episode_deadline || now.monotonic>=monotonic_deadline)
            return fault("REFERENCE_LEASE_EXPIRED");
        if (e.manual) return fault("REFERENCE_MANUAL_TAKEOVER",true);
        if (!e.connected || !e.confirmed || e.epoch!=binding.epoch)
            return fault("REFERENCE_DRIVER_EPOCH_OR_CONNECTION_CHANGED");
        if (!fresh(e.sdk_stamp,now) || !fresh(e.encoder_stamp,now)
            || e.sdk_stamp<binding.epoch || e.encoder_stamp<binding.epoch)
            return fault("REFERENCE_SOURCE_STALE");
        if(phase==ReferencePhase::ARMED && (now.ros_ns>=goal.start || now.monotonic>=goal_monotonic_start))
            return fault("REFERENCE_GOAL_NOT_OBSERVED_BEFORE_START");
        if(phase==ReferencePhase::EXECUTING && !goal_terminal
           && (now.ros_ns>=goal.end || now.monotonic>=goal_monotonic_end))
            return fault("REFERENCE_GOAL_RESULT_DEADLINE");
        return {true,"REFERENCE_LIVE"};
    }
    bool legacy_allowed() const { return phase==ReferencePhase::LEGACY; }
    bool gated() const {
        return phase!=ReferencePhase::ACTIVE_IDLE && phase!=ReferencePhase::EXECUTING;
    }
    ReferenceVector map(const ReferenceVector& physical) const {
        ReferenceVector out{};
        for(size_t i=0;i<6;++i) out[i]=physical[i]+binding.bias[i];
        return out;
    }
    ReferenceVector physical_baseline() const {
        ReferenceVector q=decoded(binding.baseline);
        for(size_t i=0;i<6;++i) q[i]-=binding.bias[i];
        return q;
    }
    ReferenceOutcome prepare(const ReferenceBinding& next,uint64_t seq,
                              const ReferenceClock& now,const ReferenceEvidence& e) {
        // Retry never renews either clock. A consumed binding id never re-arms.
        if (!std::isfinite(now.monotonic) || now.ros_ns==0) return {false,"REFERENCE_PREPARE_CLOCK_INVALID"};
        if (used_bindings_.count(next.id)) return {false,"REFERENCE_BINDING_REPLAY"};
        const bool same_owner_history=!binding.id.empty() && next.owner==binding.owner
            && next.epoch==binding.epoch && next.generation==binding.generation;
        if(same_owner_history && next.previous_id!=binding.id)
            return {false,"REFERENCE_PREDECESSOR_OMITTED"};
        const bool rollover=(phase==ReferencePhase::LEGACY && released_success_) && goal_terminal
            && goal_status==3 && goal_error==0 && next.previous_id==binding.id;
        if (phase!=ReferencePhase::LEGACY && !rollover)
            return {false,"REFERENCE_PREPARE_PHASE_INVALID"};
        if (!stable(e,now) || next.baseline!=e.sdk || next.epoch!=e.epoch)
            return {false,"REFERENCE_PREPARE_NOT_STATIONARY"};
        if (next.id.empty() || next.owner.empty() || seq==0 || next.generation==0
            || next.episode.empty() || used_episodes_.count(next.episode)
            || next.epoch==0 || next.sdk_stamp<next.epoch || next.encoder_stamp<next.epoch
            || !digest(next.plan) || !digest(next.model) || !finite(next.bias)
            || !fresh(next.sdk_stamp,now) || !fresh(next.encoder_stamp,now)
            || next.sdk_stamp!=e.sdk_stamp || next.encoder_stamp!=e.encoder_stamp
            || next.issued==0 || next.issued>now.ros_ns || now.ros_ns-next.issued>500000000ULL
            || next.deadline<=now.ros_ns || !std::isfinite(next.duration) || next.duration<=0.
            || next.episode_deadline<=now.ros_ns || next.episode_deadline>next.deadline
            || !std::isfinite(next.episode_duration) || next.episode_duration<=0.)
            return {false,"REFERENCE_PREPARE_BINDING_INVALID"};
        if (!rollover && !next.previous_id.empty()) return {false,"REFERENCE_PREDECESSOR_INVALID"};
        if (rollover && (next.owner!=binding.owner || next.epoch!=binding.epoch
            || next.generation!=binding.generation || next.plan!=binding.plan || next.model!=binding.model
            || next.deadline!=binding.deadline || now.monotonic>=outer_monotonic_deadline))
            return {false,"REFERENCE_PREDECESSOR_CHANGED"};
        const auto wire=decoded(e.sdk);
        for(size_t i=0;i<6;++i) {
            if (std::abs(next.bias[i])>.09 || next.baseline[i]>4095)
                return {false,"REFERENCE_BIAS_OR_WORD_RANGE_INVALID"};
            // Bias is measured and quantized, not chosen to hide a failed goal.
            if (std::abs((wire[i]-e.encoder[i])-next.bias[i])>1e-10)
                return {false,"REFERENCE_BIAS_NOT_SOURCE_BOUND"};
            if (!rollover && std::abs(next.bias[i])>.035)
                return {false,"REFERENCE_INITIAL_FOLLOWING_EXCEEDED"};
        }
        double expiry=now.monotonic+std::min(next.duration,(next.deadline-now.ros_ns)*1e-9);
        if (rollover) expiry=std::min(expiry,outer_monotonic_deadline);
        outer_monotonic_deadline=expiry;
        monotonic_deadline=std::min(expiry,now.monotonic+std::min(next.episode_duration,
            (next.episode_deadline-now.ros_ns)*1e-9));
        binding=next; request_sequence=seq; command_sequence=0;prepared_ns=now.ros_ns;
        used_bindings_.insert(next.id);used_episodes_.insert(next.episode); phase=ReferencePhase::PREPARED;
        goal=ReferenceGoal{};goal_observed=false;goal_terminal=false;goal_status=0;goal_error=0;
        code="REFERENCE_PREPARED_OUTPUT_GATED";return {true,code};
    }
    ReferenceOutcome mutate(uint64_t seq,const std::string& id,const std::string& owner,
                            const ReferenceClock& now,const ReferenceEvidence& e) {
        auto live=check(now,e);if(!live.ok)return live;
        if(id!=binding.id || owner!=binding.owner || seq<=request_sequence)
            return {false,"REFERENCE_REQUEST_IDENTITY_OR_REPLAY"};
        request_sequence=seq;return {true,"REFERENCE_REQUEST_BOUND"};
    }
    ReferenceOutcome commit(uint64_t seq,const std::string& id,const std::string& owner,
                            const ReferenceClock& now,const ReferenceEvidence& e) {
        auto ok=mutate(seq,id,owner,now,e);if(!ok.ok)return ok;
        if(phase!=ReferencePhase::PREPARED || !stable(e,now) || e.sdk!=binding.baseline
           || e.controller_stamp<=prepared_ns || !finite(map(e.controller)) || words(map(e.controller))!=binding.baseline)
            return fault("REFERENCE_COMMIT_NOT_SAME_WIRE");
        phase=ReferencePhase::ACTIVE_IDLE;code="REFERENCE_COMMITTED_SAME_WIRE";return {true,code};
    }
    ReferenceOutcome arm(const ReferenceGoal& next,uint64_t seq,const std::string& id,
                         const std::string& owner,const ReferenceClock& now,const ReferenceEvidence& e) {
        auto ok=mutate(seq,id,owner,now,e);if(!ok.ok)return ok;
        if(goal_terminal && (goal_status!=3 || goal_error!=0))
            return fault("REFERENCE_FAILED_EPISODE_CANNOT_REARM");
        if(phase!=ReferencePhase::ACTIVE_IDLE || !stable(e,now) || !physical_following(e)
           || !finite(map(e.controller)) || words(map(e.controller))!=e.sdk || next.id.empty() || next.token.empty()
           || used_goals_.count(next.id) || used_goal_tokens_.count(next.token)
           || !digest(next.proof) || !digest(next.physical_digest) || !digest(next.wire_digest)
           || next.stamp==0 || next.stamp>now.ros_ns || now.ros_ns-next.stamp>500000000ULL
           || next.start<=now.ros_ns || next.end<=next.start || next.end>=std::min(binding.deadline,binding.episode_deadline)
           || next.envelope.empty()
           || now.monotonic+(next.end-now.ros_ns)*1e-9>=monotonic_deadline)
            return fault("REFERENCE_ARM_INVALID");
        goal=next;goal_monotonic_start=now.monotonic+(next.start-now.ros_ns)*1e-9;
        goal_monotonic_end=now.monotonic+(next.end-now.ros_ns)*1e-9;
        used_goals_.insert(next.id);used_goal_tokens_.insert(next.token);
        goal_observed=false;goal_terminal=false;goal_status=0;goal_error=0;
        phase=ReferencePhase::ARMED;code="REFERENCE_GOAL_ARMED";return {true,code};
    }
    ReferenceOutcome observe_goal(const std::string& caller,const std::string& id,uint64_t stamp,
                                  const std::vector<uint8_t>& envelope,const ReferenceClock& now) {
        if(phase==ReferencePhase::LEGACY){released_success_=false;return {true,"LEGACY_GOAL"};}
        if(phase!=ReferencePhase::ARMED || !reference_gateway_caller(caller) || id!=goal.id
           || stamp!=goal.stamp || envelope!=goal.envelope || now.ros_ns>=goal.start
           || !std::isfinite(now.monotonic) || now.monotonic>=goal_monotonic_start)
            return fault("REFERENCE_ACTION_IDENTITY_REPLAY_OR_LATE");
        goal_observed=true;phase=ReferencePhase::EXECUTING;
        code="REFERENCE_ACTION_OBSERVED";return {true,code};
    }
    ReferenceOutcome terminal(const std::string& id,int status,int error) {
        if(phase==ReferencePhase::LEGACY)return {true,"LEGACY_TERMINAL"};
        if(!goal_observed || id!=goal.id || goal_terminal)
            return fault("REFERENCE_TERMINAL_IDENTITY_OR_REPLAY");
        if(status!=2 && status!=3 && status!=4 && status!=5 && status!=8 && status!=9)
            return fault("REFERENCE_TERMINAL_STATUS_INVALID");
        goal_terminal=true;goal_status=status;goal_error=error;
        // Do not clear the map: the ordinary JTC stopping hold still uses it.
        code="REFERENCE_ACTION_TERMINAL_MAP_RETAINED";return {true,code};
    }
    ReferenceOutcome admit_command(const std::string& id,const std::string& owner,
             const std::string& token,uint64_t epoch,uint64_t seq,
             const ReferenceVector& physical,const ReferenceVector& wire,
             const ReferenceClock& now,const ReferenceEvidence& e) {
        auto live=check(now,e);if(!live.ok)return live;
        if(gated() || id!=binding.id || owner!=binding.owner || epoch!=binding.epoch
           || seq<=command_sequence || !finite(physical) || !finite(wire)
           || !finite(map(physical)) || words(map(physical))!=words(wire)) return fault("REFERENCE_MAPPED_COMMAND_INVALID");
        if(phase==ReferencePhase::EXECUTING && (!goal_observed || token!=goal.token))
            return fault("REFERENCE_COMMAND_GOAL_UNBOUND");
        if(phase==ReferencePhase::ACTIVE_IDLE && (!token.empty() || words(wire)!=e.sdk))
            return fault("REFERENCE_IDLE_COMMAND_CHANGED");
        if(phase==ReferencePhase::EXECUTING
           && (now.ros_ns<goal.start || now.monotonic<goal_monotonic_start) && words(wire)!=e.sdk)
            return fault("REFERENCE_COMMAND_BEFORE_TRAJECTORY_START");
        command_sequence=seq;return {true,"REFERENCE_MAPPED_COMMAND_ADMITTED"};
    }
    ReferenceOutcome quiesce(uint64_t seq,const std::string& id,const std::string& owner,
                              const ReferenceClock& now,const ReferenceEvidence& e) {
        auto ok=mutate(seq,id,owner,now,e);if(!ok.ok)return ok;
        if(phase!=ReferencePhase::EXECUTING || !goal_terminal || !stable(e,now)
           || (goal_status==3 && goal_error==0 && !physical_following(e))
           || !finite(map(e.controller)) || words(map(e.controller))!=e.sdk) return fault("REFERENCE_QUIESCE_NOT_CONFIRMED");
        phase=ReferencePhase::ACTIVE_IDLE;code="REFERENCE_QUIESCENT_MAP_RETAINED";return {true,code};
    }
    ReferenceOutcome release_prepare(uint64_t seq,const std::string& id,const std::string& owner,
                                      const ReferenceClock& now,const ReferenceEvidence& e) {
        if(phase==ReferencePhase::FAULT || phase==ReferencePhase::MANUAL) {
            if(id!=binding.id || owner!=binding.owner || seq<=request_sequence || !manual_clearable(now,e))
                return {false,"REFERENCE_MANUAL_CLEANUP_NOT_CONFIRMED"};
            request_sequence=seq;binding.baseline=e.sdk;prepared_ns=now.ros_ns;
            manual_cleanup_=true;released_success_=false;phase=ReferencePhase::RELEASE_PENDING;
            code="REFERENCE_MANUAL_RELEASE_OUTPUT_GATED";return {true,code};
        }
        auto ok=mutate(seq,id,owner,now,e);if(!ok.ok)return ok;
        if(phase!=ReferencePhase::ACTIVE_IDLE || !stable(e,now) || words(map(e.controller))!=e.sdk)
            return fault("REFERENCE_RELEASE_NOT_STATIONARY");
        binding.baseline=e.sdk;prepared_ns=now.ros_ns;
        phase=ReferencePhase::RELEASE_PENDING;code="REFERENCE_RELEASE_OUTPUT_GATED";return {true,code};
    }
    ReferenceOutcome release_commit(uint64_t seq,const std::string& id,const std::string& owner,
                                     const ReferenceClock& now,const ReferenceEvidence& e) {
        if(manual_cleanup_) {
            if(phase!=ReferencePhase::RELEASE_PENDING || id!=binding.id || owner!=binding.owner
               || seq<=request_sequence || !manual_clearable(now,e) || e.sdk!=binding.baseline
               || e.controller_stamp<=prepared_ns || words(e.controller)!=e.sdk)
                return fault("REFERENCE_MANUAL_RELEASE_NOT_SAME_WIRE");
            request_sequence=seq;manual_cleanup_=false;released_success_=false;
            phase=ReferencePhase::LEGACY;code="REFERENCE_MANUAL_SOFTWARE_RELEASED";return {true,code};
        }
        auto ok=mutate(seq,id,owner,now,e);if(!ok.ok)return ok;
        if(phase!=ReferencePhase::RELEASE_PENDING || !stable(e,now) || e.sdk!=binding.baseline
           || e.controller_stamp<=prepared_ns || words(e.controller)!=e.sdk)
            return fault("REFERENCE_RELEASE_NOT_SAME_WIRE");
        released_success_=goal_terminal && goal_status==3 && goal_error==0;
        phase=ReferencePhase::LEGACY;code="REFERENCE_RELEASED_SAME_WIRE";return {true,code};
    }
private:
    bool manual_clearable(const ReferenceClock& now,const ReferenceEvidence& e) const {
        if(binding.id.empty() || !e.manual || !e.manual_reference || e.epoch!=binding.epoch
           || !std::isfinite(now.monotonic) || now.ros_ns==0)return false;
        auto view=e;view.manual=false;return stable(view,now);
    }
    bool released_success_=false,manual_cleanup_=false;
    std::set<std::string> used_bindings_,used_goals_,used_goal_tokens_,used_episodes_;
};
} // namespace alicia_d_driver
