"""Initial observation preparation can wait; it never revives hard authority."""
from copy import deepcopy
import json
import math
import threading
import time
import uuid

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from .initial_preflight_attempt import InitialPreflightAttemptMixin
from .observe_return_task import positive_ns
from .initial_preflight_recovery import InitialPreflightRecoveryCollector
from .unknown_pending_identity import support_identity_deadline

REQUEST_TOPIC='/grasp_mode/initial_preflight_request'
ACK_TOPIC='/grasp_mode/initial_preflight_ack'
PENDING='TARGET_MEASUREMENT_PENDING'


class _StillPending(Exception):
    pass


class InitialPreflightTaskMixin(InitialPreflightAttemptMixin):
    def _initialize_initial_preflight(self):
        super()._initialize_initial_preflight()
        self._initial_preflight_condition=threading.Condition(threading.RLock())
        self._initial_preflight_collector=None
        self._initial_preflight_source_objects={}
        self._initial_preflight_token=None
        self._initial_preflight_acks={}
        self._initial_preflight_pending=None
        self._initial_preflight_generation=0
        self._initial_preflight_last_path=None
        self._initial_preflight_replan=False
        self._initial_preflight_proven_gaps={}

    def _setup_initial_preflight_io(self):
        super()._setup_initial_preflight_io()
        self._initial_preflight_request_pub=rospy.Publisher(REQUEST_TOPIC,String,queue_size=8,latch=True)
        self._initial_preflight_ack_sub=rospy.Subscriber(ACK_TOPIC,String,self._initial_preflight_ack,queue_size=16)
        self._initial_preflight_mask_sub=rospy.Subscriber('/perception/object_mask',Image,
            self._initial_preflight_mask,queue_size=16)

    def _initial_preflight_selection(self, selection):
        self._initial_preflight_collector=None
        if selection.get('mode')=='unknown' and selection.get('strategy')=='observe_return':
            config=rospy.get_param('/unknown_perception',{})
            from .tabletop import Config
            defaults=Config()
            policy={k:float(config.get(k,getattr(defaults,k))) for k in (
                'support_max_normal_change_deg','support_max_local_shift_m','support_threshold_m','support_min_ratio')}
            self._initial_preflight_collector=InitialPreflightRecoveryCollector(selection,
                min(float(config.get('max_frame_age_sec',.8)),.8),policy)
        self._initial_preflight_source_objects={}
        self._initial_preflight_token=None
        self._initial_preflight_pending=None
        self._initial_preflight_generation=0
        self._initial_preflight_last_path=None
        self._initial_preflight_replan=False
        self._initial_preflight_proven_gaps={}

    def _initial_preflight_ack(self,msg):
        try:
            ack=json.loads(msg.data)
            if not isinstance(ack,dict) or not isinstance(ack.get('request_id'),str):return
            with self._initial_preflight_condition:
                self._initial_preflight_acks[ack['request_id']]=ack
                while len(self._initial_preflight_acks)>32:
                    del self._initial_preflight_acks[next(iter(self._initial_preflight_acks))]
                self._initial_preflight_condition.notify_all()
        except (ValueError,TypeError,AttributeError):return

    def _initial_preflight_rpc(self,action):
        token=self._initial_preflight_token
        if token is None:raise ValueError('INITIAL_PREFLIGHT_TOKEN_MISSING')
        now=rospy.Time.now().to_nsec()
        request=dict(token['request'],action=action,request_id=uuid.uuid4().hex,stamp_ns=now)
        self._initial_preflight_request_pub.publish(String(data=json.dumps(request,sort_keys=True)))
        deadline=min(token['monotonic_deadline'],time.monotonic()+.8)
        while time.monotonic()<deadline and rospy.Time.now().to_nsec()<request['deadline_ns']:
            with self._initial_preflight_condition:
                ack=deepcopy(self._initial_preflight_acks.get(request['request_id']))
                if ack is None:
                    self._initial_preflight_condition.wait(.01);continue
            if (ack.get('version')!=1 or ack.get('token_id')!=request['token_id']
                    or ack.get('action')!=action or type(ack.get('stamp_ns')) is not int
                    or not now<=ack['stamp_ns']<=rospy.Time.now().to_nsec()):
                raise ValueError('INITIAL_PREFLIGHT_ACK_BINDING_INVALID')
            if ack.get('state')=='pending' and action=='commit':raise _StillPending()
            if ack.get('accepted') is not True:
                raise ValueError('INITIAL_PREFLIGHT_REJECTED: '+str(ack.get('reason')))
            return ack
        raise ValueError('INITIAL_PREFLIGHT_ACK_TIMEOUT')

    def _initial_preflight_arm(self,plan,gcfg):
        record=self._initial_preflight_attempt
        c=self._initial_preflight_collector
        if (record is None or record['physical_action_submitted'] or record['disqualifications']
                or c is None or getattr(self,'_execution_authority_revoked',False)):
            return False
        with c.lock:
            reference=deepcopy(c.references.get(c.latest_joined_ns))
            if c.fatal or reference is None:raise ValueError('INITIAL_PREFLIGHT_REFERENCE_MISSING: '+c.fatal)
        now=rospy.Time.now().to_nsec()
        if not now<reference['deadline_ns']:raise ValueError('INITIAL_PREFLIGHT_REFERENCE_STALE')
        report=self._observe_return_report(plan)
        metadata=report['observe_return']
        if metadata.get('phase')!='observation_prefix':return False
        deadline=plan.header.stamp.to_nsec()+round(self._configured_plan_validity(gcfg)*1e9)
        if not now<deadline:raise ValueError('INITIAL_PREFLIGHT_PLAN_EXPIRED')
        request=dict(version=1,token_id=record['attempt_id'],selection=deepcopy(self._mode_bound_selection),
            plan_id=plan.plan_id,target_track_id=plan.target_track_id,target_epoch=metadata['target_epoch'],
            control_epoch_ns=record['entry_control_epoch_ns'],q_station_rad=deepcopy(record['entry_q_rad']),
            anchor_binding=reference['binding'],deadline_ns=deadline,physical_action_submitted=False)
        self._initial_preflight_token=dict(request=request,state='registering',
            monotonic_deadline=time.monotonic()+(deadline-now)/1e9)
        ack=self._initial_preflight_rpc('arm')
        status=ack.get('reference_status')
        if (not isinstance(status,dict) or status.get('source_stamp_ns')!=ack.get('reference_source_ns')
                or ack.get('state') not in ('armed','pending')):
            raise ValueError('INITIAL_PREFLIGHT_ARM_REFERENCE_INVALID')
        # The ACK is only a reference pointer; wait for our own exact three
        # source messages. It cannot supply a missing mask or ObjectPose.
        wait_end=min(self._initial_preflight_token['monotonic_deadline'],time.monotonic()+.8)
        while time.monotonic()<wait_end:
            with c.lock:local=deepcopy(c.references.get(ack['reference_source_ns']))
            if local is not None:
                if local['binding']!=request['anchor_binding']:
                    raise ValueError('INITIAL_PREFLIGHT_ARM_REFERENCE_CHANGED')
                break
            with self._initial_preflight_condition:self._initial_preflight_condition.wait(.01)
        else:raise ValueError('INITIAL_PREFLIGHT_ARM_SOURCE_JOIN_MISSING')
        with self._grasp6d_plan_guard():
            if getattr(self,'_execution_authority_revoked',False):
                raise ValueError('INITIAL_PREFLIGHT_HARD_AUTHORITY_ALREADY_REVOKED')
            self._initial_preflight_token['state']='armed'
        return True

    def _initial_preflight_scope(self,allow_registering=False):
        token=self._initial_preflight_token
        record=self._initial_preflight_attempt
        states=('armed','registering') if allow_registering else ('armed',)
        return bool(token and token['state'] in states and record
            and not record['physical_action_submitted'] and not record['disqualifications']
            and getattr(self,'active',False) and not getattr(self,'_execution_authority_revoked',False)
            and self._mode_bound_selection==token['request']['selection']
            and getattr(getattr(self,'_bound_execution_plan',None),'plan_id',None)==token['request']['plan_id'])

    def _initial_preflight_abort(self,reason):
        self._initial_preflight_reject(reason)
        self._execution_authority_revoked=True
        self._last_execution_plan_event='INITIAL_PREFLIGHT_INVALID: '+reason
        if self._initial_preflight_token:self._initial_preflight_token['state']='revoked'

    def _initial_preflight_check(self):
        token=self._initial_preflight_token
        if not token or token['state'] not in ('armed','registering'):return False
        if not self._initial_preflight_scope(True):
            self._initial_preflight_abort('initial_station_authority_changed');return False
        token=self._initial_preflight_token;now=rospy.Time.now().to_nsec()
        if (now>=token['request']['deadline_ns'] or time.monotonic()>=token['monotonic_deadline']
                or self._following_epoch_ns!=token['request']['control_epoch_ns']
                or rospy.get_param_cached('/gui/joint_direct_mode',False) is not False):
            self._initial_preflight_abort('initial_station_context_changed_or_expired');return False
        c=self._initial_preflight_collector
        if c is None or c.fatal:
            self._initial_preflight_abort('initial_station_source_invalid');return False
        return True

    def _initial_preflight_quarantine(self,ns,reference_ns=None,notice=False):
        c=self._initial_preflight_collector;now=rospy.Time.now().to_nsec()
        with c.lock:
            if reference_ns is None:
                candidates=[n for n in c.references if n<ns]
                if not candidates:return False
                reference_ns=max(candidates)
            reference=deepcopy(c.references.get(reference_ns))
        old=self._initial_preflight_pending
        deadline=(old['deadline_ns'] if old and old['reference_source_ns']==reference_ns
                  else (reference or {}).get('deadline_ns',0))
        if (reference is None or not reference_ns<ns<=now<deadline
                or ns>=reference['deadline_ns']
                or reference['binding']!=self._initial_preflight_token['request']['anchor_binding']):
            return False
        if old is None:
            self._initial_preflight_generation+=1
            self._initial_preflight_pending=dict(reference_source_ns=reference_ns,
                source_ns=ns,source_stamps={ns},notices=set(),deadline_ns=reference['deadline_ns'],
                monotonic_deadline=time.monotonic()+(reference['deadline_ns']-now)/1e9)
        else:
            if old['reference_source_ns']!=reference_ns:return False
            old['source_stamps'].add(ns);old['source_ns']=max(ns,old['source_ns'])
        if notice:self._initial_preflight_pending['notices'].add(ns)
        self.latest_obj=None;self.latest_obj_time=None;self.latest_target_geometry=None
        self._initial_preflight_replan=True
        return True

    def obj_cb(self,msg):
        received_ns=rospy.Time.now().to_nsec()
        c=self._initial_preflight_collector
        repeated_proven=False
        if c is not None:
            ns=msg.header.stamp.to_nsec()
            repeated_proven=(not msg.detected and ns in self._initial_preflight_proven_gaps
                and self._initial_preflight_source_objects.get(ns)==msg)
            c.object(msg,rospy.Time.now().to_nsec())
            self._initial_preflight_source_objects[ns]=deepcopy(msg)
            while len(self._initial_preflight_source_objects)>64:
                del self._initial_preflight_source_objects[min(self._initial_preflight_source_objects)]
        token=self._initial_preflight_token
        preparing=bool(token and token['state'] in ('armed','registering'))
        if not preparing:
            if (repeated_proven and not c.fatal and token
                    and self._mode_bound_selection==token['request']['selection']):
                return
        else:
            with self._grasp6d_plan_guard():
                if self._initial_preflight_superseded_object(msg,received_ns):
                    return
                if (repeated_proven and not c.fatal and self._initial_preflight_token
                        and self._mode_bound_selection==self._initial_preflight_token['request']['selection']):
                    return
                if self._initial_preflight_check():
                    if not msg.detected:
                        ns=msg.header.stamp.to_nsec()
                        if ns in self._initial_preflight_proven_gaps:return
                        if self._initial_preflight_quarantine(ns):
                            self._initial_preflight_refresh();return
                    elif self._initial_preflight_pending is not None:
                        self._initial_preflight_refresh();return
        result=super().obj_cb(msg)
        self._initial_preflight_refresh()
        return result

    def _initial_preflight_superseded_object(self,msg,received_ns):
        """Discard only a timely old positive delayed by our motion lock.

        This is not measured geometry, a freshness extension, or restoration
        of revoked authority. False/invalid/new sources keep the original path.
        """
        token=self._initial_preflight_token
        record=self._initial_preflight_attempt
        if (not token or token['state']!='committed' or not record
                or record['ended_ns'] is not None or not record['physical_action_submitted']
                or any(v!='measured_pose_changed' for v in record['disqualifications'])
                or getattr(self,'_execution_authority_revoked',False)
                or getattr(msg,'detected',False) is not True):
            return False
        submitted=token.get('motion_request')
        current=getattr(self,'_observation_measurement_request',None)
        plan=getattr(self,'_bound_execution_plan',None)
        try:
            gcfg=rospy.get_param_cached('/grasp',{})
            ns=self._object_source_stamp(msg).to_nsec();now=rospy.Time.now().to_nsec()
            validity_ns=round(self._configured_target_observation_validity(gcfg)*1e9)
            if (not plan or not submitted
                    or not plan.header.stamp.to_nsec()<=ns<submitted['stamp_ns']
                    or not 0<=received_ns-ns<=validity_ns
                    or not now-ns>validity_ns or now<=received_ns):
                return False
            if (not current or current['state'] not in ('executing','requalifying','qualified')
                    or submitted['state']!='executing' or submitted['motion_kind']!='observation'
                    or any(current[k]!=submitted[k] for k in ('lease_id','selection','session_id',
                        'view_plan_id','target_epoch','target_track_id','control_epoch_ns','plan_sha256'))
                    or submitted['selection']!=token['request']['selection']
                    or submitted['view_plan_id']!=token['request']['plan_id']
                    or submitted['target_track_id']!=token['request']['target_track_id']
                    or submitted['control_epoch_ns']!=token['request']['control_epoch_ns']
                    or submitted['plan_sha256']!=committed_plan_digest(plan)
                    or record['plan_digest']!=self._execution_plan_digest(plan)
                    or record['plan_digest']!=self._bound_execution_plan_digest):
                return False
            session=getattr(self,'_observe_return_session',None) or {}
            if (not getattr(self,'active',False) or not getattr(self,'_mode_ready',False)
                    or self._mode_selection!=submitted['selection']
                    or self._mode_bound_selection!=submitted['selection']
                    or self._following_epoch_ns!=submitted['control_epoch_ns']
                    or rospy.get_param_cached('/gui/joint_direct_mode',False) is not False
                    or session.get('phase')!='observing'
                    or any(session.get(k)!=submitted[k] for k in ('session_id','view_plan_id','target_epoch','target_track_id'))
                    or now>=positive_ns(session['deadline_ns']) or now>=current['deadline_ns']
                    or (current['state']=='requalifying' and now>=current['capture_deadline_ns'])
                    or not self._observed_target_matches_plan(plan,msg,gcfg)):
                return False
            confidence=float(getattr(msg,'confidence',0.) or 0.)
            if not math.isfinite(confidence) or confidence<self._cfg_float(gcfg,'min_object_confidence',.5):
                return False
            center=self._plan_geometry_center_xyz(plan);xyz=self._object_xyz(msg)
            if math.sqrt(sum((a-b)**2 for a,b in zip(center,xyz)))>self._configured_target_max_drift(gcfg):
                return False
            previous=getattr(self,'latest_obj',None);stamp=getattr(self,'latest_obj_time',None)
            if previous is not None and stamp is not None:
                window=self._cfg_float(gcfg,'object_jump_filter_window_sec',4.)
                limit=self._cfg_float(gcfg,'max_object_jump_m',.12)
                if (limit>0. and (received_ns-stamp.to_nsec())/1e9<=window
                        and self._object_distance(previous,msg)>limit):return False
            c=self._initial_preflight_collector
            if c is not None:
                with c.lock:
                    if c.fatal:return False
                    reference=deepcopy(c.references.get(ns))
                if reference is not None and reference['binding']!=token['request']['anchor_binding']:
                    return False
            # Only this rare lock-delayed source reaches the authoritative
            # owner check. Normal committed callbacks retain the original path.
            self._observation_measurement_live(current)
            if (getattr(self,'_execution_authority_revoked',False)
                    or self._following_epoch_ns!=submitted['control_epoch_ns']
                    or any(v!='measured_pose_changed' for v in record['disqualifications'])):
                return False
            token.setdefault('superseded_object_sources',[]).append(ns)
            token['superseded_object_sources']=token['superseded_object_sources'][-16:]
            rospy.loginfo('Discarded pre-motion object delayed by action lock: %s',json.dumps(dict(
                source_ns=ns,received_ns=received_ns,checked_ns=now,lease_id=current['lease_id'],
                metric_geometry_authority=False,execution_authorized=False),sort_keys=True))
            return True
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError):
            return False

    def _initial_preflight_mask(self,msg):
        c=self._initial_preflight_collector
        if c is not None:c.mask(msg,rospy.Time.now().to_nsec())
        self._initial_preflight_refresh()

    def _initial_preflight_status(self,status):
        c=self._initial_preflight_collector
        if c is not None:c.status(status,rospy.Time.now().to_nsec())
        self._initial_preflight_refresh()

    def target_geometry_cb(self,msg):
        if not msg.valid and str(msg.failure_reason).partition(':')[0]==PENDING:
            with self._grasp6d_plan_guard():
                if self._initial_preflight_token:
                    try:
                        payload=json.loads(msg.failure_reason.partition(':')[2])
                        if set(payload)!={'initial_preflight'}:raise ValueError('payload')
                        p=payload['initial_preflight']
                        if set(p)!={'token_id','reference_source_ns','pending_source_ns'}:raise ValueError('fields')
                        request=self._initial_preflight_token['request'];ns=p['pending_source_ns']
                        if (p['token_id']!=request['token_id'] or msg.target_track_id!=request['target_track_id']
                                or msg.header.frame_id!='base_link' or msg.header.stamp.to_nsec()!=ns):
                            raise ValueError('binding')
                        if self._initial_preflight_proven_gaps.get(ns)==p['reference_source_ns']:return
                        if self._initial_preflight_check() and self._initial_preflight_quarantine(ns,p['reference_source_ns'],True):
                            self._initial_preflight_refresh();return
                    except (ValueError,TypeError,KeyError,AttributeError):pass
        return super().target_geometry_cb(msg)

    def _initial_preflight_refresh(self):
        token=self._initial_preflight_token
        if not token or token['state'] not in ('armed','registering'):
            return
        with self._grasp6d_plan_guard():
            if self._initial_preflight_check() and self._initial_preflight_pending is not None:
                pending=self._initial_preflight_pending;c=self._initial_preflight_collector
                now=rospy.Time.now().to_nsec()
                with c.lock:
                    try:
                        for ns in pending['source_stamps']:
                            gap=c.gaps.get(ns)
                            if gap is not None:
                                support_identity_deadline(gap['reference'], now)
                    except (ValueError, KeyError, TypeError):
                        self._initial_preflight_abort('initial_support_recovery_expired')
                        return
                    gaps=[deepcopy(c.gaps.get(ns)) for ns in pending['source_stamps']]
                    candidates=[deepcopy(row) for row in c.recoveries.values()
                        if row['reference_source_ns']==pending['reference_source_ns']
                        and row['source_ns']>pending['source_ns']]
                complete=bool(gaps and all(gap is not None for gap in gaps))
                if complete:
                    try:
                        deadlines=[support_identity_deadline(gap['reference'], now) for gap in gaps]
                    except (ValueError, KeyError, TypeError):
                        self._initial_preflight_abort('initial_support_recovery_expired')
                        return
                    new_deadline=min(deadlines)
                    # Receipt validation established completion before the old
                    # geometry expired. This is the original fixed deadline.
                    if new_deadline!=pending['deadline_ns']:
                        pending['monotonic_deadline']+= (new_deadline-pending['deadline_ns'])/1e9
                        pending['deadline_ns']=new_deadline
                if now>=pending['deadline_ns'] or time.monotonic()>=pending['monotonic_deadline']:
                    self._initial_preflight_abort('initial_support_recovery_expired')
                elif complete and pending['notices'] and candidates:
                    recovered=max(candidates,key=lambda row:row['source_ns']);ns=recovered['source_ns']
                    obj=self._initial_preflight_source_objects.get(ns)
                    if (obj is not None and obj.detected and 0<=now-ns<800000000
                            and not getattr(self,'_execution_authority_revoked',False)):
                        # Admit the actual new ObjectPose using its unchanged
                        # source stamp; never republish or relabel the old one.
                        super().obj_cb(deepcopy(obj))
                        if (not getattr(self,'_execution_authority_revoked',False)
                                and getattr(self,'latest_obj',None) is not None):
                            for gap_ns in pending['source_stamps']:
                                self._initial_preflight_proven_gaps[gap_ns]=pending['reference_source_ns']
                            self._initial_preflight_pending=None
                            self._initial_preflight_generation+=1
        with self._initial_preflight_condition:self._initial_preflight_condition.notify_all()

    def _initial_preflight_wait(self):
        while True:
            self._initial_preflight_refresh()
            with self._grasp6d_plan_guard():
                if not self._initial_preflight_check():
                    return bool(self._initial_preflight_pending is None
                        and getattr(self,'active',False)
                        and not getattr(self,'_execution_authority_revoked',False))
                if self._initial_preflight_pending is None:return True
            with self._initial_preflight_condition:self._initial_preflight_condition.wait(.01)

    def _initial_preflight_wrap_move(self,move_pose):
        def planned(pose,execute):
            if execute or not self._initial_preflight_scope():return move_pose(pose,execute)
            self._initial_preflight_last_path=(move_pose,deepcopy(pose))
            while True:
                if not self._initial_preflight_wait():raise ValueError('INITIAL_PREFLIGHT_RECOVERY_FAILED')
                generation=self._initial_preflight_generation
                response=move_pose(pose,False)
                if not self._initial_preflight_wait():raise ValueError('INITIAL_PREFLIGHT_RECOVERY_FAILED')
                with self._grasp6d_plan_guard():
                    if generation==self._initial_preflight_generation:
                        self._initial_preflight_replan=False
                        return response
        return planned

    def _initial_preflight_ready(self):
        if not self._initial_preflight_wait():return False
        if self._initial_preflight_scope() and self._initial_preflight_replan:
            if self._initial_preflight_last_path is None:return True
            move,pose=self._initial_preflight_last_path
            result=self._initial_preflight_wrap_move(move)(pose,False)
            if not result.success:
                self._initial_preflight_abort('fresh_preflight_failed');return False
        return not getattr(self,'_execution_authority_revoked',False)

    def _execution_checkpoint(self,plan,gcfg,stage_label):
        if self._initial_preflight_token and self._initial_preflight_token['state']=='armed':
            self._initial_preflight_ready()
        return super()._execution_checkpoint(plan,gcfg,stage_label)

    def _validate_bound_plan_locked(self,plan,gcfg):
        if self._initial_preflight_token and self._initial_preflight_token['state']=='registering':
            return self._station_task_result(False,PENDING,'initial observation registration is not acknowledged')
        if self._initial_preflight_token and self._initial_preflight_token['state']=='armed':
            self._initial_preflight_check()
        if self._initial_preflight_scope() and self._initial_preflight_pending is not None:
            return self._station_task_result(False,PENDING,'initial observation has no current measured geometry')
        return super()._validate_bound_plan_locked(plan,gcfg)

    def _initial_preflight_commit(self):
        ack=self._initial_preflight_rpc('commit')
        if ack.get('state')!='committed':raise ValueError('INITIAL_PREFLIGHT_COMMIT_NOT_CONFIRMED')
        with self._initial_preflight_lock:
            self._initial_preflight_submission_context()
            self._initial_preflight_token['state']='committed'

    def _initial_preflight_submission_context(self):
        token=self._initial_preflight_token;record=self._initial_preflight_attempt
        if (not token or not record or record['ended_ns'] is not None
                or record['disqualifications'] or record['physical_action_submitted']
                or getattr(self,'_execution_authority_revoked',False)
                or not getattr(self,'active',False)
                or self._following_epoch_ns!=token['request']['control_epoch_ns']
                or self._mode_bound_selection!=token['request']['selection']
                or getattr(getattr(self,'_bound_execution_plan',None),'plan_id',None)!=token['request']['plan_id']
                or self._initial_preflight_pending is not None
                or self._initial_preflight_replan
                or rospy.Time.now().to_nsec()>=token['request']['deadline_ns']
                or time.monotonic()>=token['monotonic_deadline']):
            self._initial_preflight_abort('initial_submission_context_changed')
            raise ValueError('INITIAL_PREFLIGHT_SUBMISSION_CONTEXT_CHANGED')

    def _initial_preflight_mark_action(self,label):
        with self._initial_preflight_lock:
            record=self._initial_preflight_attempt
            if record is not None:
                record['physical_action_submitted']=True
                if len(record['submissions'])<16:
                    record['submissions'].append(dict(stage=label,stamp_ns=rospy.Time.now().to_nsec()))

    def _initial_preflight_wrap_strict(self,execute_pose):
        def submitted(*args,**kwargs):
            token=self._initial_preflight_token
            if token and token['state']=='retired_for_gripper':
                # The original checked opening path has ended preparation.
                # All subsequent movement uses the existing execution gates.
                return execute_pose(*args,**kwargs)
            with self._initial_preflight_lock:
                if not token or token['state']!='committed':
                    raise ValueError('INITIAL_PREFLIGHT_STRICT_SUBMISSION_NOT_COMMITTED')
                # This closure is reused by later supplementary views and
                # CONTACT. Their original gates own motion after preparation
                # was irreversibly closed by the first actual submission.
                if not self._initial_preflight_attempt['physical_action_submitted']:
                    self._initial_preflight_submission_context()
                    self._initial_preflight_mark_action('strict observation submission')
            return execute_pose(*args,**kwargs)
        return submitted

    def _invoke_plan_bound_action(self,plan,gcfg,stage_label,action):
        if self._initial_preflight_token and self._initial_preflight_token['state']=='armed':
            self._initial_preflight_check()
        if not self._initial_preflight_scope():
            return super()._invoke_plan_bound_action(plan,gcfg,stage_label,action)
        while True:
            self._initial_preflight_ready()
            def commit_action():
                if stage_label=='6D pregrasp':
                    # The existing strict wrapper first obtains the dual
                    # motion-measurement ACK. Its hook commits below.
                    return action()
                self._initial_preflight_commit()
                validation=self._validate_bound_plan_locked(plan,gcfg)
                if not validation.ok:raise ValueError(validation.code+': '+validation.reason)
                self._initial_preflight_mark_action(stage_label)
                return action()
            try:
                # Skip only the diagnostic submission wrapper: mark the real
                # boundary above, after commit acknowledgement, not before it.
                return super(InitialPreflightAttemptMixin,self)._invoke_plan_bound_action(
                    plan,gcfg,stage_label,commit_action)
            except _StillPending:
                with self._initial_preflight_condition:self._initial_preflight_condition.wait(.01)

    def _observation_measurement_begin(self,plan,motion_kind='observation'):
        scoped=self._initial_preflight_scope()
        result=super()._observation_measurement_begin(plan,motion_kind)
        if scoped:
            try:
                self._initial_preflight_commit()
                self._initial_preflight_token['motion_request']=deepcopy(result)
            except _StillPending:
                # The existing movement-measurement lease has already been
                # admitted. It is not a new preparation window; do not retry
                # a begin with that active lease or swallow its rejection.
                raise ValueError('INITIAL_PREFLIGHT_PENDING_AFTER_MOVEMENT_LEASE')
        return result

    def _command_gripper_position(self,set_gripper,position,label,wait_sec,
            execution_plan=None,gcfg=None,skip_if_reached=False,reached_tolerance=0.0):
        if (self._initial_preflight_scope() and execution_plan is not None and skip_if_reached
                and self._gripper_position_reached(position,reached_tolerance)):
            return self._execution_checkpoint(execution_plan,gcfg or {},label+' already reached')
        if self._initial_preflight_scope():
            self._initial_preflight_ready()
            ack=self._initial_preflight_rpc('revoke')
            if ack.get('state')!='revoked':raise ValueError('INITIAL_PREFLIGHT_REVOKE_NOT_CONFIRMED')
            self._initial_preflight_token['state']='retired_for_gripper'
        return super()._command_gripper_position(set_gripper,position,label,wait_sec,
            execution_plan=execution_plan,gcfg=gcfg,skip_if_reached=skip_if_reached,
            reached_tolerance=reached_tolerance)

    def _initial_preflight_finish(self,result):
        token=self._initial_preflight_token
        if token and token['state'] not in ('committed','revoked','retired_for_gripper'):
            request=dict(token['request'],action='revoke',request_id=uuid.uuid4().hex,
                stamp_ns=rospy.Time.now().to_nsec())
            self._initial_preflight_request_pub.publish(String(data=json.dumps(request,sort_keys=True)))
            token['state']='revoked'
        return super()._initial_preflight_finish(result)
