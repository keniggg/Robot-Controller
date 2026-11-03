#pragma once
#include "alicia_d_driver/physical_reference_contract.hpp"
#include "alicia_d_driver/physical_reference_transport.hpp"
#include <alicia_d_driver/PhysicalReferenceCommand.h>
#include <alicia_d_driver/PhysicalReferenceState.h>
#include <alicia_d_driver/SetPhysicalReference.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <openssl/sha.h>
#include <chrono>
#include <deque>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace alicia_d_driver {
// The caller's ownership mutex always precedes data_mutex_. Call *_locked only
// while holding that mutex. No callbacks mutate controller feedback or torque.
class PhysicalReferenceRos {
public:
    using Guard=std::function<void(const std::function<void()>&)>;
    using Environment=std::function<void(ReferenceEvidence&)>;
    using FaultHold=std::function<void()>;
    PhysicalReferenceRos(ros::NodeHandle& nh,bool driver,const Guard& guard,
                         const Environment& environment,const FaultHold& hold)
      : driver_(driver),guard_(guard),environment_(environment),fault_hold_(hold) {
        const std::string base=driver ? "/alicia_d/physical_reference":"/bessica_d_hw_interface/physical_reference";
        publisher_=nh.advertise<PhysicalReferenceState>(base+"_state",10,false);
        service_=nh.advertiseService(base,&PhysicalReferenceRos::service,this);
        sdk_sub_=nh.subscribe("/alicia_d/sdk_command",40,&PhysicalReferenceRos::sdk,this);
        encoder_sub_=nh.subscribe("/alicia_d/accepted_joint_states",40,&PhysicalReferenceRos::encoder,this);
        reference_sub_=nh.subscribe("/alicia_d/control_reference",40,&PhysicalReferenceRos::reference,this);
        epoch_sub_=nh.subscribe("/alicia_d/control_reference_epoch",5,&PhysicalReferenceRos::epoch,this);
        status_sub_=nh.subscribe("/alicia_d/actuation_status",10,&PhysicalReferenceRos::status,this);
        gui_sub_=nh.subscribe("/gui/joint_direct_mode",5,&PhysicalReferenceRos::manual,this);
        controller_sub_=nh.subscribe("/alicia_controller/state",40,&PhysicalReferenceRos::controller,this);
        goals_sub_=nh.subscribe("/alicia_controller/follow_joint_trajectory/goal",10,&PhysicalReferenceRos::action_goal,this);
        results_sub_=nh.subscribe("/alicia_controller/follow_joint_trajectory/result",10,&PhysicalReferenceRos::action_result,this);
        actions_sub_=nh.subscribe("/alicia_controller/follow_joint_trajectory/status",10,&PhysicalReferenceRos::actions,this);
        if(!driver) driver_state_sub_=nh.subscribe("/alicia_d/physical_reference_state",20,&PhysicalReferenceRos::driver_state,this);
        timer_=nh.createTimer(ros::Duration(.02),&PhysicalReferenceRos::timer,this);
    }
    static ReferenceClock clock() {
        return {ros::Time::now().toNSec(),std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count()};
    }
    template<class T> static std::vector<uint8_t> bytes(const T& msg) {
        std::vector<uint8_t> b(ros::serialization::serializationLength(msg));
        ros::serialization::OStream stream(b.data(),b.size());ros::serialization::serialize(stream,msg);return b;
    }
    template<class T> static std::string digest(const T& msg) {
        auto b=bytes(msg);unsigned char out[SHA256_DIGEST_LENGTH];SHA256(b.data(),b.size(),out);
        std::ostringstream s;for(auto v:out)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);return s.str();
    }
    bool legacy_allowed_locked() {
        std::lock_guard<std::mutex> lock(data_mutex_);return core_.legacy_allowed();
    }
    uint64_t legacy_barrier_locked() {
        std::lock_guard<std::mutex> lock(data_mutex_);return legacy_barrier_ns_;
    }
    void revoke_locked(const std::string& why,bool manual=false) {
        std::lock_guard<std::mutex> lock(data_mutex_);core_.fault(why,manual);
    }
    // Called immediately before an SDK write as well as from the 50 Hz watchdog.
    bool healthy_locked() {
        bool fault=false;{
            std::lock_guard<std::mutex> lock(data_mutex_);auto now=clock();auto e=evidence(now);
            const auto before=core_.phase;core_.check(now,e);
            fault=core_.phase==ReferencePhase::FAULT;
        }if(fault)fault_hold_();
        std::lock_guard<std::mutex> lock(data_mutex_);
        return core_.phase!=ReferencePhase::FAULT && core_.phase!=ReferencePhase::MANUAL;
    }
    bool encode_locked(const ReferenceVector& physical,PhysicalReferenceCommand& out) {
        std::lock_guard<std::mutex> lock(data_mutex_);auto now=clock();auto e=evidence(now);
        if(core_.legacy_allowed())return false;
        if(!core_.check(now,e).ok || core_.gated())return false;
        if(!driver_ && !reference_peer_allows_output(driver_state_,core_,now)) {
            if(core_.phase==ReferencePhase::EXECUTING && !core_.goal_terminal
               && (now.ros_ns>=core_.goal.start || now.monotonic>=core_.goal_monotonic_start))
                core_.fault("REFERENCE_PEER_GOAL_NOT_ACKNOWLEDGED_BEFORE_START");
            return false;
        }
        const auto wire=core_.map(physical);const auto seq=core_.command_sequence+1;
        const auto token=core_.phase==ReferencePhase::EXECUTING?core_.goal.token:std::string();
        auto ok=core_.admit_command(core_.binding.id,core_.binding.owner,token,core_.binding.epoch,
                                   seq,physical,wire,now,e);
        if(!ok.ok)return false;
        out.header.stamp=ros::Time::now();out.header.frame_id="physical_reference_v1";
        out.binding_id=core_.binding.id;out.owner_token=core_.binding.owner;out.goal_token=token;
        out.epoch_ns=core_.binding.epoch;out.sequence=seq;
        std::copy(physical.begin(),physical.end(),out.physical_positions_rad.begin());
        std::copy(wire.begin(),wire.end(),out.wire_positions_rad.begin());return true;
    }
    bool admit_locked(const PhysicalReferenceCommand& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);auto now=clock();auto e=evidence(now);
        if(msg.header.frame_id!="physical_reference_v1" || !PhysicalReferenceContract::fresh(msg.header.stamp.toNSec(),now))
            return core_.fault("REFERENCE_COMMAND_STAMP_INVALID").ok;
        ReferenceVector physical{},wire{};
        std::copy(msg.physical_positions_rad.begin(),msg.physical_positions_rad.end(),physical.begin());
        std::copy(msg.wire_positions_rad.begin(),msg.wire_positions_rad.end(),wire.begin());
        return core_.admit_command(msg.binding_id,msg.owner_token,msg.goal_token,msg.epoch_ns,msg.sequence,
                                   physical,wire,now,e).ok;
    }
private:
    struct Sample { uint64_t stamp; ReferenceVector q; };
    bool driver_;
    Guard guard_;Environment environment_;FaultHold fault_hold_;
    std::mutex data_mutex_;
    PhysicalReferenceContract core_;
    ReferenceObservedGoals observed_goals_;
    std::deque<Sample> sdk_,encoder_;
    std::string reference_mode_;uint64_t reference_stamp_=0;ReferenceWords reference_words_{};
    uint64_t epoch_=0,status_stamp_=0,actions_stamp_=0,legacy_barrier_ns_=0;
    bool manual_=true,confirmed_=false,action_idle_=false;
    double goal_stop_duration_=0.;uint32_t action_transport_sequence_=0;
    control_msgs::JointTrajectoryControllerState controller_;
    PhysicalReferenceState driver_state_;
    ros::Publisher publisher_;
    ros::ServiceServer service_;
    ros::Subscriber sdk_sub_,encoder_sub_,reference_sub_,epoch_sub_,status_sub_,gui_sub_,controller_sub_,goals_sub_,results_sub_,actions_sub_,driver_state_sub_;
    ros::Timer timer_;
    static bool became_fault(ReferencePhase before,ReferencePhase after) {
        return before!=after && (after==ReferencePhase::FAULT || after==ReferencePhase::MANUAL);
    }
    static bool canonical(const std::vector<std::string>& names) {
        if(names.size()!=6)return false;
        for(size_t i=0;i<6;++i)if(names[i]!="Joint"+std::to_string(i+1))return false;
        return true;
    }
    static bool ordered(const std::vector<std::string>& names,const std::vector<double>& values,ReferenceVector& out,bool gripper=false) {
        if(names.size()!=values.size() || names.size()!=(gripper?7:6))return false;
        std::set<std::string> seen;
        for(size_t i=0;i<names.size();++i) {
            if(!seen.insert(names[i]).second || !std::isfinite(values[i]))return false;
            if(names[i]=="right_finger" && gripper)continue;
            bool found=false;for(size_t j=0;j<6;++j)if(names[i]=="Joint"+std::to_string(j+1)){out[j]=values[i];found=true;break;}
            if(!found)return false;
        }return PhysicalReferenceContract::finite(out);
    }
    ReferenceEvidence evidence(const ReferenceClock& now) {
        ReferenceEvidence e;e.epoch=epoch_;e.manual=manual_;e.confirmed=confirmed_;
        e.connected=!sdk_.empty()&&!encoder_.empty();
        e.action_idle=action_idle_&&observed_goals_.idle()&&PhysicalReferenceContract::fresh(actions_stamp_,now);
        if(!sdk_.empty()){e.sdk_stamp=sdk_.back().stamp;e.sdk=PhysicalReferenceContract::words(sdk_.back().q);}
        if(!encoder_.empty()){e.encoder_stamp=encoder_.back().stamp;e.encoder=encoder_.back().q;}
        e.manual_reference=reference_mode_=="sdk_manual" && reference_stamp_==e.sdk_stamp
            && reference_stamp_>=e.epoch && reference_words_==e.sdk
            && PhysicalReferenceContract::fresh(reference_stamp_,now);
        e.controller_stamp=controller_.header.stamp.toNSec();
        if(!ordered(controller_.joint_names,controller_.desired.positions,e.controller)
           || controller_.desired.velocities.size()!=6 || controller_.desired.accelerations.size()!=6)
            e.controller_stamp=0;
        else {
            ordered_derivatives(controller_.desired.velocities,e.velocity);
            ordered_derivatives(controller_.desired.accelerations,e.acceleration);
        }
        if(e.connected) {
            const uint64_t end=std::min(e.sdk_stamp,e.encoder_stamp);
            uint64_t first_sdk=e.sdk_stamp,first_encoder=e.encoder_stamp;
            for(auto i=sdk_.rbegin();i!=sdk_.rend();++i) {
                if(PhysicalReferenceContract::words(i->q)!=e.sdk || first_sdk-i->stamp>250000000ULL)break;
                first_sdk=i->stamp;++e.sdk_samples;
            }
            ReferenceVector low=e.encoder,high=e.encoder;
            for(auto i=encoder_.rbegin();i!=encoder_.rend();++i) {
                if(first_encoder-i->stamp>250000000ULL)break;
                bool stable=true;for(size_t j=0;j<6;++j){low[j]=std::min(low[j],i->q[j]);high[j]=std::max(high[j],i->q[j]);if(high[j]-low[j]>2*M_PI/4096+1e-12)stable=false;}
                if(!stable)break;first_encoder=i->stamp;++e.encoder_samples;
            }
            e.stable_since=std::max(first_sdk,first_encoder);
            if(e.stable_since>end)e.stable_since=0;
        }
        if(!driver_) {
            ReferenceWords peer_words{};std::copy(driver_state_.sdk_counts.begin(),driver_state_.sdk_counts.end(),peer_words.begin());
            e.pending_complete=driver_state_.pending_complete && driver_state_.evidence_valid
                && peer_words==e.sdk && driver_state_.epoch_ns==e.epoch
                && PhysicalReferenceContract::fresh(driver_state_.header.stamp.toNSec(),now);
        }
        environment_(e);return e;
    }
    void ordered_derivatives(const std::vector<double>& v,ReferenceVector& out) {
        for(size_t i=0;i<6;++i) {
            auto p=std::find(controller_.joint_names.begin(),controller_.joint_names.end(),"Joint"+std::to_string(i+1));
            out[i]=(p==controller_.joint_names.end())?NAN:v[p-controller_.joint_names.begin()];
        }
    }
    PhysicalReferenceState state(const ReferenceClock& now,const ReferenceEvidence& e) {
        PhysicalReferenceState s;s.header.stamp=ros::Time::now();s.header.frame_id="physical_reference_v1";
        s.evidence_valid=e.connected&&e.confirmed&&PhysicalReferenceContract::fresh(e.sdk_stamp,now)&&PhysicalReferenceContract::fresh(e.encoder_stamp,now);
        s.pending_complete=e.pending_complete;auto stationary_view=e;stationary_view.manual=false;
        s.stationary=PhysicalReferenceContract::stable(stationary_view,now);s.manual_reference=e.manual_reference;
        s.phase=static_cast<uint8_t>(core_.phase);s.code=core_.code;
        const auto& b=core_.binding;s.binding_id=b.id;s.previous_binding_id=b.previous_id;s.owner_token=b.owner;
        s.goal_token=core_.goal.token;s.episode_id=b.episode;s.action_goal_id=core_.goal.id;s.action_goal_stamp_ns=core_.goal.stamp;s.action_transport_sequence=action_transport_sequence_;
        s.epoch_ns=e.epoch;s.selection_generation=b.generation;s.request_sequence=core_.request_sequence;s.last_command_sequence=core_.command_sequence;
        s.plan_sha256=b.plan;s.model_sha256=b.model;s.proof_sha256=core_.goal.proof;
        s.physical_trajectory_sha256=core_.goal.physical_digest;s.wire_trajectory_sha256=core_.goal.wire_digest;
        std::copy(b.bias.begin(),b.bias.end(),s.bias_rad.begin());std::copy(e.controller.begin(),e.controller.end(),s.physical_reference_rad.begin());
        auto wire=PhysicalReferenceContract::decoded(e.sdk);std::copy(wire.begin(),wire.end(),s.wire_positions_rad.begin());
        std::copy(e.sdk.begin(),e.sdk.end(),s.sdk_counts.begin());s.sdk_stamp_ns=e.sdk_stamp;s.encoder_stamp_ns=e.encoder_stamp;s.controller_stamp_ns=e.controller_stamp;
        s.prepared_ns=core_.prepared_ns;s.deadline_ns=b.deadline;s.episode_deadline_ns=b.episode_deadline;s.outer_monotonic_remaining_sec=std::max(0.,core_.outer_monotonic_deadline-now.monotonic);s.monotonic_remaining_sec=std::max(0.,core_.monotonic_deadline-now.monotonic);
        s.stop_duration_sec=goal_stop_duration_;s.goal_observed=core_.goal_observed;s.goal_terminal=core_.goal_terminal;s.goal_status=core_.goal_status;s.goal_error_code=core_.goal_error;
        s.output_gated=!core_.legacy_allowed()&&core_.gated();return s;
    }
    bool service(ros::ServiceEvent<SetPhysicalReference::Request,SetPhysicalReference::Response>& event) {
        auto& r=event.getRequest();auto& out=event.getResponse();
        const auto header=event.getConnectionHeader();auto caller=header.find("callerid");
        const bool authorized=caller!=header.end()&&reference_gateway_caller(caller->second);
        guard_([&]{bool new_fault=false;{
            std::lock_guard<std::mutex> lock(data_mutex_);auto now=clock();auto e=evidence(now);const auto before=core_.phase;
            ReferenceOutcome result{false,"REFERENCE_OPERATION_INVALID"};
            const bool descriptor_matches=r.binding_id==core_.binding.id && r.owner_token==core_.binding.owner
                && r.previous_binding_id==core_.binding.previous_id && r.epoch_ns==core_.binding.epoch
                && r.selection_generation==core_.binding.generation && r.plan_sha256==core_.binding.plan
                && r.model_sha256==core_.binding.model && r.episode_id==core_.binding.episode
                && r.deadline_ns==core_.binding.deadline && r.episode_deadline_ns==core_.binding.episode_deadline
                && std::equal(r.bias_rad.begin(),r.bias_rad.end(),core_.binding.bias.begin());
            if(r.operation!=r.QUERY && !authorized)result={false,"REFERENCE_MUTATION_CALLER_REJECTED"};
            else if(r.operation!=r.QUERY && r.operation!=r.PREPARE && !descriptor_matches)
                result={false,"REFERENCE_BOUND_DESCRIPTOR_CHANGED"};
            else if(r.operation==r.QUERY) {auto probe=core_;result=probe.check(now,e);}
            else if(r.operation==r.PREPARE) {
                ReferenceBinding b;b.id=r.binding_id;b.previous_id=r.previous_binding_id;b.owner=r.owner_token;b.epoch=r.epoch_ns;b.generation=r.selection_generation;
                b.plan=r.plan_sha256;b.model=r.model_sha256;b.sdk_stamp=r.source_sdk_stamp_ns;b.encoder_stamp=r.source_encoder_stamp_ns;
                b.issued=r.issued_ns;b.deadline=r.deadline_ns;b.duration=r.maximum_duration_sec;
                b.episode=r.episode_id;b.episode_deadline=r.episode_deadline_ns;b.episode_duration=r.episode_duration_sec;
                std::copy(r.bias_rad.begin(),r.bias_rad.end(),b.bias.begin());std::copy(r.baseline_sdk_counts.begin(),r.baseline_sdk_counts.end(),b.baseline.begin());
                bool found_sdk=false,found_encoder=false;
                for(const auto& row:sdk_)if(row.stamp==b.sdk_stamp && PhysicalReferenceContract::words(row.q)==e.sdk){e.sdk_stamp=row.stamp;found_sdk=true;}
                for(const auto& row:encoder_)if(row.stamp==b.encoder_stamp){e.encoder_stamp=row.stamp;e.encoder=row.q;found_encoder=true;}
                if(!driver_ && (driver_state_.binding_id!=r.binding_id || driver_state_.phase!=PhysicalReferenceState::PREPARED))result={false,"REFERENCE_DRIVER_NOT_PREPARED"};
                else if(!found_sdk||!found_encoder)result={false,"REFERENCE_SOURCE_SAMPLE_NOT_RETAINED"};
                else result=core_.prepare(b,r.request_sequence,now,e);
            } else if(r.operation==r.COMMIT)result=core_.commit(r.request_sequence,r.binding_id,r.owner_token,now,e);
            else if(r.operation==r.ARM) {
                double configured_stop=0.;ReferenceGoal g;
                ReferenceOutcome validated{false,"REFERENCE_STOP_CONFIGURATION_UNAVAILABLE"};
                if(ros::param::get("/alicia_controller/stop_trajectory_duration",configured_stop))
                    validated=bind_physical_reference_goal(r,core_.binding.bias,configured_stop,g);
                if(!validated.ok)result=core_.fault(validated.code);
                else {result=core_.arm(g,r.request_sequence,r.binding_id,r.owner_token,now,e);
                    if(result.ok)goal_stop_duration_=configured_stop;}
            } else if(r.operation==r.QUIESCE)result=core_.quiesce(r.request_sequence,r.binding_id,r.owner_token,now,e);
            else if(r.operation==r.RELEASE_PREPARE)result=core_.release_prepare(r.request_sequence,r.binding_id,r.owner_token,now,e);
            else if(r.operation==r.RELEASE_COMMIT) {
                result=core_.release_commit(r.request_sequence,r.binding_id,r.owner_token,now,e);
                if(result.ok)legacy_barrier_ns_=now.ros_ns;
            }
            else if(r.operation==r.REVOKE){result=core_.mutate(r.request_sequence,r.binding_id,r.owner_token,now,e);if(result.ok){core_.fault("REFERENCE_EXPLICIT_REVOKE");result={true,"REFERENCE_EXPLICIT_REVOKE"};}}
            out.success=result.ok;out.code=result.code;out.state=state(now,e);publisher_.publish(out.state);new_fault=became_fault(before,core_.phase);
        }if(new_fault)fault_hold_();});return true;
    }
    template<class T> static std::string caller(const ros::MessageEvent<T const>& event) {
        auto h=event.getConnectionHeaderPtr();if(!h)return {};auto i=h->find("callerid");return i==h->end()?std::string():i->second;
    }
    void sample(const ros::MessageEvent<sensor_msgs::JointState const>& event,bool is_sdk) {
        auto msg=event.getMessage();if(!reference_driver_caller(caller(event)))return;
        ReferenceVector q{};if(msg->header.frame_id!=(is_sdk?"sdk_transmitted":"sdk_measured") || !ordered(msg->name,msg->position,q,!is_sdk))return;
        guard_([&]{std::lock_guard<std::mutex> lock(data_mutex_);auto& rows=is_sdk?sdk_:encoder_;
            if(!rows.empty()&&msg->header.stamp.toNSec()<=rows.back().stamp){core_.fault("REFERENCE_SOURCE_STAMP_REPLAY");return;}
            rows.push_back({msg->header.stamp.toNSec(),q});while(rows.size()>128)rows.pop_front();});
    }
    void sdk(const ros::MessageEvent<sensor_msgs::JointState const>& e){sample(e,true);}
    void encoder(const ros::MessageEvent<sensor_msgs::JointState const>& e){sample(e,false);}
    void reference(const ros::MessageEvent<sensor_msgs::JointState const>& event) {
        if(!reference_driver_caller(caller(event)))return;auto msg=event.getMessage();ReferenceVector q{};
        guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);
            // Every later reference replaces the old manual claim; never filter by mode.
            reference_mode_=msg->header.frame_id;reference_stamp_=msg->header.stamp.toNSec();
            if(!ordered(msg->name,msg->position,q,true)){reference_mode_="invalid";return;}
            reference_words_=PhysicalReferenceContract::words(q);
        });
    }
    void epoch(const std_msgs::Header::ConstPtr& e){guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);epoch_=e->stamp.toNSec();});}
    void status(const std_msgs::String::ConstPtr& e){guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);confirmed_=e->data.compare(0,10,"CONFIRMED:")==0;status_stamp_=ros::Time::now().toNSec();});}
    void manual(const std_msgs::Bool::ConstPtr& e){guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);const bool changed=manual_!=e->data;manual_=e->data;if(changed&&manual_)core_.fault("REFERENCE_MANUAL_TAKEOVER",true);});}
    void controller(const ros::MessageEvent<control_msgs::JointTrajectoryControllerState const>& e){if(!reference_hardware_caller(caller(e)))return;guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);controller_=*e.getMessage();});}
    void driver_state(const ros::MessageEvent<PhysicalReferenceState const>& event){
        if(!reference_driver_caller(caller(event)))return;
        guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);const auto& state=*event.getMessage();
            if(state.header.stamp>=driver_state_.header.stamp)driver_state_=state;});
    }
    void actions(const ros::MessageEvent<actionlib_msgs::GoalStatusArray const>& e){if(!reference_hardware_caller(caller(e)))return;guard_([&]{std::lock_guard<std::mutex> l(data_mutex_);actions_stamp_=e.getMessage()->header.stamp.toNSec();action_idle_=true;for(const auto& s:e.getMessage()->status_list)if(s.status==0||s.status==1||s.status==6||s.status==7)action_idle_=false;});}
    void action_goal(const ros::MessageEvent<control_msgs::FollowJointTrajectoryActionGoal const>& event){
        guard_([&]{bool new_fault=false;{
            std::lock_guard<std::mutex> lock(data_mutex_);const auto before=core_.phase;
            const auto& goal=*event.getMessage();action_transport_sequence_=goal.header.seq;
            const auto tracked=observed_goals_.observe(goal.goal_id.id,goal.goal_id.stamp.toNSec());
            if(!tracked.ok)core_.fault(tracked.code);
            else core_.observe_goal(caller(event),goal.goal_id.id,goal.goal_id.stamp.toNSec(),
                                    reference_action_envelope(goal),clock());
            new_fault=became_fault(before,core_.phase);
        }if(new_fault)fault_hold_();});
    }
    void action_result(const ros::MessageEvent<control_msgs::FollowJointTrajectoryActionResult const>& event){
        if(!reference_hardware_caller(caller(event)))return;
        guard_([&]{bool new_fault=false;{
            std::lock_guard<std::mutex> lock(data_mutex_);const auto before=core_.phase;
            const auto& result=*event.getMessage();
            const auto tracked=observed_goals_.terminal(result.status.goal_id.id,
                result.status.goal_id.stamp.toNSec(),result.status.status);
            if(!tracked.ok && core_.phase!=ReferencePhase::LEGACY)
                core_.fault("REFERENCE_TERMINAL_IDENTITY_OR_REPLAY");
            else core_.terminal(result.status.goal_id.id,result.status.status,result.result.error_code);
            new_fault=became_fault(before,core_.phase);
        }if(new_fault)fault_hold_();});
    }
    void timer(const ros::TimerEvent&){guard_([&]{healthy_locked();std::lock_guard<std::mutex> l(data_mutex_);auto now=clock();auto e=evidence(now);publisher_.publish(state(now,e));});}
};
} // namespace alicia_d_driver
