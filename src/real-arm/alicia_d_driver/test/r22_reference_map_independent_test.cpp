// Independent pure transaction tests; no ROS, serial connection or device I/O.
#include "alicia_d_driver/physical_reference_contract.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace alicia_d_driver;
namespace {
constexpr double Q=2.*M_PI/4096.;
void required(bool ok) { if(!ok) throw std::runtime_error("fixture setup rejected"); }
struct Case {
    PhysicalReferenceContract c;
    ReferenceClock now{10000000000ULL,100.};
    ReferenceEvidence e;
    ReferenceBinding b;
    ReferenceGoal g;
    Case() {
        e.connected=e.confirmed=e.pending_complete=e.action_idle=true;e.manual=false;
        e.epoch=1000000000ULL;e.sdk_stamp=e.encoder_stamp=e.controller_stamp=9990000000ULL;
        e.stable_since=9600000000ULL;e.sdk_samples=e.encoder_samples=5;
        e.sdk.fill(2048);e.sdk[1]=1747;e.controller=PhysicalReferenceContract::decoded(e.sdk);
        e.encoder=e.controller;e.encoder[1]-=13*Q;
        b.id="binding-1";b.owner="owner-1";b.epoch=e.epoch;b.generation=1;
        b.plan=std::string(64,'a');b.model=std::string(64,'b');b.bias[1]=13*Q;
        b.baseline=e.sdk;b.sdk_stamp=e.sdk_stamp;b.encoder_stamp=e.encoder_stamp;
        b.issued=now.ros_ns;b.deadline=now.ros_ns+60000000000ULL;b.duration=60.;
        b.episode="nominal-pregrasp";b.episode_deadline=b.deadline;b.episode_duration=b.duration;
    }
    void tick(uint64_t ns=1000000ULL) {
        now.ros_ns+=ns;now.monotonic+=ns*1e-9;
        e.sdk_stamp=e.encoder_stamp=e.controller_stamp=now.ros_ns;
    }
    void committed() {
        required(c.prepare(b,1,now,e).ok);tick();e.controller=c.physical_baseline();
        required(c.commit(2,b.id,b.owner,now,e).ok);
    }
    void executing() {
        committed();g.id="action-1";g.token="goal-1";g.proof=std::string(64,'c');
        g.physical_digest=std::string(64,'d');g.wire_digest=std::string(64,'e');
        g.stamp=now.ros_ns;g.start=now.ros_ns+200000000ULL;g.end=g.start+2000000000ULL;
        g.envelope={1,2,3,4};required(c.arm(g,3,b.id,b.owner,now,e).ok);
        required(c.observe_goal("/motion_gateway",g.id,g.stamp,g.envelope,now).ok);
    }
    void stopped(int status=3,int error=0,int new_bias_counts=33) {
        executing();tick(2600000000ULL);
        e.encoder=PhysicalReferenceContract::decoded(e.sdk);e.encoder[1]-=new_bias_counts*Q;
        e.stable_since=now.ros_ns-400000000ULL;
        required(c.terminal(g.id,status,error).ok);
        required(c.quiesce(4,b.id,b.owner,now,e).ok);
    }
    ReferenceBinding next_binding(int bias_counts=33) {
        auto n=b;n.id="binding-2";n.previous_id=b.id;n.bias[1]=bias_counts*Q;
        n.episode="next-approach";
        n.baseline=e.sdk;n.sdk_stamp=e.sdk_stamp;n.encoder_stamp=e.encoder_stamp;
        n.issued=now.ros_ns;return n;
    }
    void released() {
        required(c.release_prepare(5,b.id,b.owner,now,e).ok);tick();
        e.controller=c.decoded(e.sdk);
        required(c.release_commit(6,b.id,b.owner,now,e).ok);
    }
};
int pass=0,fail=0;
void test(const char* name,const std::function<bool()>& f) {
    bool ok=false;try{ok=f();}catch(const std::exception& x){std::cerr<<name<<": "<<x.what()<<"\n";}
    std::cout<<"{\"case\":\""<<name<<"\",\"passed\":"<<(ok?"true":"false")<<"}\n";
    if(ok)++pass;else ++fail;
}
}
int main() {
    test("initial_commit_preserves_exact_wire",[]{Case x;x.committed();
        return x.c.words(x.c.map(x.e.controller))==x.e.sdk&&!x.c.legacy_allowed();});
    test("changed_commit_is_gated_and_retains_bias",[]{Case x;required(x.c.prepare(x.b,1,x.now,x.e).ok);
        x.tick();x.e.controller=x.c.physical_baseline();x.e.controller[1]+=Q;
        auto r=x.c.commit(2,x.b.id,x.b.owner,x.now,x.e);
        return !r.ok&&x.c.gated()&&!x.c.legacy_allowed()&&x.c.binding.bias[1]==13*Q;});
    test("active_episode_cannot_reestimate_bias",[]{Case x;x.executing();auto n=x.next_binding();
        return !x.c.prepare(n,4,x.now,x.e).ok&&x.c.binding.id==x.b.id;});
    test("failed_episode_cannot_be_relabelled_next_episode",[]{Case x;x.stopped(4,-5);auto n=x.next_binding();
        return !x.c.prepare(n,5,x.now,x.e).ok&&x.c.goal_error==-5&&x.c.binding.bias[1]==13*Q;});
    test("normal_next_episode_rebinds_fresh_bias_with_same_wire",[]{Case x;x.stopped();x.released();auto old=x.e.sdk;
        auto n=x.next_binding();required(x.c.prepare(n,7,x.now,x.e).ok);
        if(!x.c.gated()||x.c.words(x.c.map(x.c.physical_baseline()))!=old)return false;
        x.tick();x.e.controller=x.c.physical_baseline();required(x.c.commit(8,n.id,n.owner,x.now,x.e).ok);
        return x.c.words(x.c.map(x.e.controller))==old&&x.c.binding.bias[1]==33*Q;});
    test("wrong_goal_envelope_faults_without_bias_reset",[]{Case x;x.committed();x.g.id="g";x.g.token="t";
        x.g.proof=x.g.physical_digest=x.g.wire_digest=std::string(64,'a');x.g.stamp=x.now.ros_ns;
        x.g.start=x.now.ros_ns+100000000ULL;x.g.end=x.g.start+1000000000ULL;x.g.envelope={1};
        required(x.c.arm(x.g,3,x.b.id,x.b.owner,x.now,x.e).ok);
        return !x.c.observe_goal("/motion_gateway","g",x.g.stamp,{2},x.now).ok
            &&x.c.gated()&&!x.c.legacy_allowed()&&x.c.binding.bias[1]==13*Q;});
    test("manual_takeover_never_releases_legacy_automatically",[]{Case x;x.executing();x.e.manual=true;
        return !x.c.check(x.now,x.e).ok&&x.c.gated()&&!x.c.legacy_allowed()&&x.c.binding.bias[1]==13*Q;});
    test("epoch_change_gates_without_bias_reset",[]{Case x;x.executing();++x.e.epoch;
        return !x.c.check(x.now,x.e).ok&&x.c.gated()&&x.c.binding.bias[1]==13*Q;});
    test("monotonic_expiry_gates_even_if_ros_clock_stops",[]{Case x;x.executing();x.now.monotonic+=61.;
        return !x.c.check(x.now,x.e).ok&&x.c.gated()&&!x.c.legacy_allowed();});
    test("duplicate_command_sequence_is_rejected",[]{Case x;x.executing();auto p=x.c.physical_baseline();
        auto w=x.c.decoded(x.e.sdk);required(x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok);
        return !x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    test("mapped_domain_overflow_cannot_hide_behind_sdk_clipping",[]{Case x;x.executing();x.tick(250000000ULL);
        auto p=x.c.physical_baseline();auto w=x.c.decoded(x.e.sdk);p[1]=M_PI;w[1]=M_PI;
        return !x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    test("zero_epoch_is_not_a_valid_prepare",[]{Case x;x.e.epoch=0;x.b.epoch=0;
        return !x.c.prepare(x.b,1,x.now,x.e).ok;});
    test("source_predating_epoch_cannot_prepare",[]{Case x;x.e.epoch=x.now.ros_ns-1;x.b.epoch=x.e.epoch;
        return !x.c.prepare(x.b,1,x.now,x.e).ok;});
    test("nonfinite_monotonic_clock_cannot_prepare",[]{Case x;x.now.monotonic=std::numeric_limits<double>::quiet_NaN();
        return !x.c.prepare(x.b,1,x.now,x.e).ok;});
    test("rollover_source_bias_still_capped_at_point09",[]{Case x;x.executing();x.tick(2600000000ULL);
        x.e.encoder=x.c.decoded(x.e.sdk);x.e.encoder[1]-=60*Q;
        x.e.stable_since=x.now.ros_ns-400000000ULL;
        required(x.c.terminal(x.g.id,3,0).ok);
        const auto stopped=x.c.quiesce(4,x.b.id,x.b.owner,x.now,x.e);
        // The strengthened successful-terminal physical-following gate may
        // correctly reject this impossible-success fixture before rebinding.
        if(!stopped.ok)return stopped.code=="REFERENCE_QUIESCE_NOT_CONFIRMED"
            &&x.c.gated()&&!x.c.legacy_allowed();
        x.released();auto n=x.next_binding(60);
        return !x.c.prepare(n,7,x.now,x.e).ok;});
    test("prestart_command_cannot_move_away_from_confirmed_hold",[]{Case x;x.executing();
        auto p=x.c.physical_baseline();p[0]+=Q;auto w=x.c.map(p);
        return !x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    test("completed_same_episode_cannot_reset_its_local_budget",[]{Case x;x.stopped();x.released();
        auto n=x.next_binding();n.episode=x.b.episode;
        return !x.c.prepare(n,7,x.now,x.e).ok;});
    test("same_owner_cannot_omit_failed_lineage_to_start_fresh",[]{Case x;x.stopped(4,-5,13);x.released();
        auto n=x.next_binding(13);n.previous_id.clear();
        return !x.c.prepare(n,7,x.now,x.e).ok;});
    test("arm_complete_goal_must_fit_effective_monotonic_budget",[]{Case x;x.b.episode_duration=1.;x.committed();
        ReferenceGoal g;g.id="too-long";g.token="too-long-token";
        g.proof=g.physical_digest=g.wire_digest=std::string(64,'a');g.stamp=x.now.ros_ns;
        g.start=x.now.ros_ns+200000000ULL;g.end=g.start+2000000000ULL;g.envelope={1};
        return !x.c.arm(g,3,x.b.id,x.b.owner,x.now,x.e).ok;});
    test("expired_manual_software_cleanup_preserves_wire_without_renewing_motion",[]{Case x;x.executing();
        x.tick(61000000000ULL);required(!x.c.check(x.now,x.e).ok);
        const double expired=x.c.monotonic_deadline;
        x.e.manual=x.e.manual_reference=true;
        required(x.c.release_prepare(5,x.b.id,x.b.owner,x.now,x.e).ok);
        auto query=x.c;required(query.check(x.now,x.e).ok);
        if(query.phase!=ReferencePhase::RELEASE_PENDING||!query.gated())return false;
        x.tick();x.e.controller=x.c.decoded(x.e.sdk);
        required(x.c.release_commit(6,x.b.id,x.b.owner,x.now,x.e).ok);
        return x.c.legacy_allowed()&&x.c.monotonic_deadline==expired
            &&x.c.words(x.e.controller)==x.e.sdk
            &&!x.c.arm(x.g,7,x.b.id,x.b.owner,x.now,x.e).ok;});
    test("manual_cleanup_requires_fresh_explicit_manual_reference",[]{Case x;x.executing();
        x.e.manual=true;required(!x.c.check(x.now,x.e).ok);x.e.manual_reference=false;
        return !x.c.release_prepare(5,x.b.id,x.b.owner,x.now,x.e).ok&&!x.c.legacy_allowed();});
    test("manual_cleanup_cannot_cross_enable_epoch",[]{Case x;x.executing();
        x.e.manual=x.e.manual_reference=true;required(!x.c.check(x.now,x.e).ok);++x.e.epoch;
        return !x.c.release_prepare(5,x.b.id,x.b.owner,x.now,x.e).ok&&!x.c.legacy_allowed();});
    test("manual_cleanup_cannot_release_a_different_active_binding",[]{Case x;x.committed();
        x.e.manual=x.e.manual_reference=true;
        return !x.c.release_prepare(5,"old-binding","old-owner",x.now,x.e).ok
            &&!x.c.legacy_allowed()&&x.c.binding.id==x.b.id;});
    test("manual_cleanup_commit_must_keep_exact_wire",[]{Case x;x.executing();
        x.e.manual=x.e.manual_reference=true;required(!x.c.check(x.now,x.e).ok);
        required(x.c.release_prepare(5,x.b.id,x.b.owner,x.now,x.e).ok);x.tick();
        x.e.controller=x.c.decoded(x.e.sdk);x.e.controller[0]+=Q;
        return !x.c.release_commit(6,x.b.id,x.b.owner,x.now,x.e).ok&&x.c.gated()&&!x.c.legacy_allowed();});
    test("mapped_upper_codec_clipping_region_is_rejected",[]{Case x;x.executing();x.tick(250000000ULL);
        auto w=x.c.decoded(x.e.sdk);w[1]=M_PI-.75*Q;
        auto p=w;p[1]-=x.b.bias[1];
        return !x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    test("mapped_exact_positive_pi_cannot_claim_half_count_quantization",[]{Case x;x.executing();x.tick(250000000ULL);
        auto w=x.c.decoded(x.e.sdk);w[1]=M_PI;
        auto p=w;p[1]-=x.b.bias[1];
        return !x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    test("mapped_last_decodable_sdk_value_remains_supported",[]{Case x;x.executing();x.tick(250000000ULL);
        auto w=x.c.decoded(x.e.sdk);w[1]=sdk_joint_position_decode(4095);
        auto p=w;p[1]-=x.b.bias[1];
        return x.c.admit_command(x.b.id,x.b.owner,x.g.token,x.b.epoch,1,p,w,x.now,x.e).ok;});
    std::cout<<"{\"passed\":"<<pass<<",\"failed\":"<<fail<<"}\n";
    return fail?1:0;
}
