#!/usr/bin/env python3
"""Pure offline candidate geometry and interpretation for J2/J3 counterrotation.

No ROS imports, publishers, services, device handles, files or parameter writes.
Callers must reacquire live evidence and independently enforce current control,
MoveIt, storage and execution contracts. Returned numerical candidates are not
execution authorization. Only fresh source/feedback/SDK bindings are accepted.
"""
import math
import numpy as np

QUANTUM = 2.*math.pi/4096.
BOARD_CORNERS = np.array([[x,y,0.,1.] for x in (0.,.18) for y in (0.,.135)])


def _vector(value,n,label):
    a=np.asarray(value,dtype=float)
    if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError(label+' must be finite length '+str(n))
    return a.copy()


def _transform(value,label):
    a=np.asarray(value,dtype=float)
    if (a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1],atol=1e-10)
        or not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-7)
        or abs(np.linalg.det(a[:3,:3])-1.)>1e-7):raise ValueError(label+' must be a rigid transform')
    return a.copy()


def rotation_angle(rotation):
    return math.acos(float(np.clip((np.trace(rotation)-1.)/2.,-1.,1.)))


def words(q):
    return [int(math.floor((float(v)+math.pi)/QUANTUM+.5)) for v in q[:6]]


def validate_binding(binding):
    required=('now_ns','source_stamp_ns','accepted_stamp_ns','sdk_stamp_ns','epoch_ns',
              'accepted_epoch_ns','sdk_epoch_ns','camera_fingerprint','handeye_sha256')
    if any(k not in binding for k in required):raise ValueError('missing fresh-evidence binding')
    now=int(binding['now_ns']);epoch=int(binding['epoch_ns'])
    if epoch<=0 or epoch!=binding['accepted_epoch_ns'] or epoch!=binding['sdk_epoch_ns']:
        raise ValueError('driver epoch mismatch')
    for key,age in (('source_stamp_ns',350_000_000),('accepted_stamp_ns',300_000_000),('sdk_stamp_ns',300_000_000)):
        if not epoch<=int(binding[key])<=now or now-int(binding[key])>age:
            raise ValueError(key+' is stale, future or before epoch')
    if abs(int(binding['accepted_stamp_ns'])-int(binding['source_stamp_ns']))>100_000_000:
        raise ValueError('camera/accepted separation exceeds 100 ms')
    if not binding['camera_fingerprint'] or len(str(binding['handeye_sha256']))!=64:
        raise ValueError('camera/hand-eye identity unavailable')


def coupled_targets(sdk_q,requested_deg):
    q=_vector(sdk_q,7,'sdk_q')
    if not 2.<=float(requested_deg)<=4.:raise ValueError('requested step must be 2..4 degrees')
    count=int(math.floor(math.radians(float(requested_deg))/QUANTUM+1e-12))
    if not 22<=count<=45:raise ValueError('counterrotation must use 22..45 SDK words')
    answer=[]
    for sign in (1,-1):
        target=q.copy();target[1]+=sign*count*QUANTUM;target[2]-=sign*count*QUANTUM
        delta=np.array(words(target))-np.array(words(q))
        expected=np.array([0,sign*count,-sign*count,0,0,0])
        if not np.array_equal(delta,expected):raise ValueError('SDK quantization did not preserve opposite word increments')
        answer.append(target)
    return answer,count


def evaluate_counterrotation_candidates(measured_q,sdk_q,X,Y,binding,fk,
        gripper_corners_base,project_camera_points,image_size,joint_lower,joint_upper,
        requested_degrees=(2.,3.,4.),response_bound_deg=4.8,grid_step_deg=.3):
    """Evaluate nominal full segments plus asynchronous J2/J3 response envelope.

    fk(q6) -> base_T_tool. gripper_corners_base(tool_T, opening) -> N x 3
    conservative CAD corners. project_camera_points(camera_link_points) -> N x 2
    pixels using the supplied *current* SDK camera model (including distortion).
    No collision-service call is hidden in this function.

    The response rectangle is fixed about the INITIAL measured q2/q3 for all
    four legs. Dense samples cover independent axis lag, not only a diagonal.
    Samples do not constitute a continuous collision or visibility certificate.
    Returned states still require fresh whole-arm MoveIt validation by caller.
    """
    validate_binding(binding)
    measured=_vector(measured_q,7,'measured_q');reference=_vector(sdk_q,7,'sdk_q')
    x=_transform(X,'X');y=_transform(Y,'Y')
    lo=_vector(joint_lower,6,'joint_lower');hi=_vector(joint_upper,6,'joint_upper')
    if np.any(lo>hi):raise ValueError('reversed joint limits')
    if not 0<=measured[6]<=.05 or not 0<=reference[6]<=.05:
        raise ValueError('measured and held gripper openings must be valid')
    if float(response_bound_deg)!=4.8:raise ValueError('existing 4.8-degree response envelope must not be loosened or narrowed')
    if not 0<float(grid_step_deg)<=.3:raise ValueError('grid step must be 0..0.3 degrees')
    width,height=map(int,image_size)
    normal=y[:3,2].copy()
    if normal[2]<0:normal=-normal
    offset=-float(normal@y[:3,3])
    bound=math.radians(response_bound_deg)
    absolute_lower=measured[:6].copy();absolute_upper=measured[:6].copy()
    for j in (1,2):absolute_lower[j]-=bound;absolute_upper[j]+=bound
    # Held axes can move from actual feedback to their already-held SDK target.
    held=[0,3,4,5]
    absolute_lower[held]=np.minimum(measured[held],reference[held])
    absolute_upper[held]=np.maximum(measured[held],reference[held])
    if np.any(absolute_lower<lo) or np.any(absolute_upper>hi):
        raise ValueError('original absolute response envelope exceeds joint limits')
    checks=[]
    def check(q,label):
        if np.any(q<lo) or np.any(q>hi):raise ValueError('candidate exceeds joint limits')
        tool=_transform(fk(q),'fk')
        cad=np.concatenate([np.asarray(gripper_corners_base(tool,float(opening)),dtype=float)
                            for opening in (measured[6],reference[6])],axis=0)
        if cad.ndim!=2 or cad.shape[1]!=3 or len(cad)==0 or not np.isfinite(cad).all():
            raise ValueError('finite full gripper CAD corners required')
        clearance=float(np.min(cad@normal+offset))
        camera=(np.linalg.inv(tool@x)@y@BOARD_CORNERS.T).T[:,:3]
        forward=float(np.min(camera[:,0]))
        if forward<=0.:edge=-math.inf
        else:
            uv=np.asarray(project_camera_points(camera),dtype=float)
            if uv.shape!=(4,2) or not np.isfinite(uv).all():raise ValueError('current camera projection unavailable')
            edge=float(min(uv[:,0].min(),width-1-uv[:,0].max(),uv[:,1].min(),height-1-uv[:,1].max()))
        row=dict(kind=label,q_rad=q.tolist(),plane_clearance_m=clearance,full_board_edge_px=edge,
                 board_forward_m=forward,geometric_sample_pass=bool(clearance>=.04 and edge>=20. and forward>=.12))
        checks.append(row)
        return row
    steps=int(math.ceil(2.*response_bound_deg/float(grid_step_deg)))+1
    # Three values sample the held-axis bridge: baseline, midpoint and SDK hold.
    # This deliberately exposes any old SDK/accepted offset instead of silently
    # pretending held SDK words imply no actual movement of the other axes.
    for bridge in (0.,.5,1.):
        hold=measured[:6]*(1.-bridge)+reference[:6]*bridge
        for q2 in np.linspace(absolute_lower[1],absolute_upper[1],steps):
            for q3 in np.linspace(absolute_lower[2],absolute_upper[2],steps):
                q=hold.copy();q[1]=q2;q[2]=q3;check(q,'asynchronous_absolute_response_envelope')
    envelope_count=len(checks)
    envelope_ok=all(c['geometric_sample_pass'] for c in checks)
    candidates=[]
    for requested in requested_degrees:
        signed,count=coupled_targets(reference,requested)
        inside=all(np.all(t[:6]>=absolute_lower) and np.all(t[:6]<=absolute_upper) for t in signed)
        start=len(checks)
        endpoints=[reference,signed[0],reference,signed[1],reference]
        current=measured
        for leg,end in enumerate(endpoints[1:]):
            span=float(np.max(np.abs(end[:6]-current[:6])))
            n=max(17,int(math.ceil(span/math.radians(grid_step_deg)))+1)
            for alpha in np.linspace(0.,1.,n):check(current[:6]*(1.-alpha)+end[:6]*alpha,'nominal_leg_%d'%leg)
            current=end
        rows=checks[start:]
        positive=_transform(fk(signed[0][:6]),'positive_fk');negative=_transform(fk(signed[1][:6]),'negative_fk')
        relative=np.linalg.inv(negative)@positive
        candidates.append(dict(requested_deg=float(requested),sdk_step_words=count,
            quantized_deg=math.degrees(count*QUANTUM),inside_absolute_envelope=bool(inside),
            geometric_samples_pass=bool(inside and envelope_ok and all(c['geometric_sample_pass'] for c in rows)),
            numerical_sdk_endpoints=[t.tolist() for t in endpoints],
            fixed_four_leg_order=['positive','original_sdk','negative','original_sdk'],
            predicted_plus_minus_translation_mm=float(np.linalg.norm(relative[:3,3])*1000),
            predicted_plus_minus_rotation_deg=math.degrees(rotation_angle(relative[:3,:3])),
            nominal_minimum_plane_m=min(c['plane_clearance_m'] for c in rows),
            nominal_minimum_board_margin_px=min(c['full_board_edge_px'] for c in rows),
            nominal_sample_range=[start,len(checks)]))
    return dict(schema=1,binding=dict(binding),execution_authorized=False,
        requires_live_moveit_and_revalidation=True,source_is_fresh_only_relative_to_supplied_now=True,
        fixed_absolute_lower_rad=absolute_lower.tolist(),fixed_absolute_upper_rad=absolute_upper.tolist(),
        response_bound_deg=response_bound_deg,envelope_grid_per_axis=steps,envelope_samples=envelope_count,
        envelope_pass=envelope_ok,
        envelope_minimum_plane_m=min(c['plane_clearance_m'] for c in checks[:envelope_count]),
        envelope_minimum_board_margin_px=min(c['full_board_edge_px'] for c in checks[:envelope_count]),
        candidates=candidates,samples=checks,
        limits=['Dense geometry samples are not a continuous collision or visibility certificate.',
                'The 4.8-degree envelope is a planned observed-response bound, not a servo guarantee.',
                'No epoch, board, image or SDK reference from an old experiment may authorize execution.',
                'Each leg must reacquire its own geometry and independently validate the unchanged original absolute envelope.',
                'Actual feedback, not opposing SDK increments, determines rotation cancellation.'])


def evaluate_measured_pair(q_before,q_after,camera_board_before,camera_board_after,
                           fk,fixed_X,translation_hypothesis_bound_m=.1):
    """Camera/FK invariants plus explicit sensitivity to residual rotation.

    `translation_hypothesis_bound_m` is an assumed bound on an alternative
    hand-eye *translation correction*, never a measured accuracy guarantee.
    Caller must provide stationary multi-frame mean poses and record spread.
    """
    qa=_vector(q_before,6,'q_before');qb=_vector(q_after,6,'q_after')
    ca=_transform(camera_board_before,'C_before');cb=_transform(camera_board_after,'C_after')
    x=_transform(fixed_X,'X')
    a=np.linalg.inv(_transform(fk(qa),'fk_before'))@_transform(fk(qb),'fk_after')
    b=ca@np.linalg.inv(cb)
    theta=rotation_angle(a[:3,:3]);ctheta=rotation_angle(b[:3,:3])
    gain=2.*math.sin(theta/2.)
    bound=float(translation_hypothesis_bound_m)
    if not math.isfinite(bound) or bound<0.:raise ValueError('finite nonnegative hypothesis bound required')
    # A X = X B => t_A - R_X t_B = (I-R_A) t_X.
    residual=a[:3,3]+a[:3,:3]@x[:3,3]-x[:3,3]-x[:3,:3]@b[:3,3]
    delta=qb-qa
    return dict(actual_joint_delta_deg=np.degrees(delta).tolist(),
        actual_joint23_sum_delta_deg=math.degrees(float(delta[1]+delta[2])),
        fk_relative_rotation_deg=math.degrees(theta),camera_relative_rotation_deg=math.degrees(ctheta),
        camera_minus_fk_rotation_deg=math.degrees(ctheta-theta),
        fk_translation_norm_mm=float(np.linalg.norm(a[:3,3])*1000),
        camera_translation_norm_mm=float(np.linalg.norm(b[:3,3])*1000),
        invariant_length_difference_mm=float((np.linalg.norm(a[:3,3])-np.linalg.norm(b[:3,3]))*1000),
        AX_minus_XB_translation_mm=(residual*1000).tolist(),
        translation_error_gain=gain,sensitivity_mm_per_10mm_translation_correction=gain*10.,
        translation_hypothesis_bound_m=bound,maximum_additional_fixed_translation_leak_mm=gain*bound*1000,
        using_current_X_norm_rotation_leak_mm=gain*float(np.linalg.norm(x[:3,3]))*1000,
        one_encoder_word_deg=math.degrees(QUANTUM),
        actual_fk_rotation_within_one_word=bool(theta<=QUANTUM+1e-10),
        interpretation='At zero actual relative rotation, any constant hand-eye translation cancels; translation length is also invariant to constant hand-eye rotation. Nonzero rotation has the reported lever-arm dependence. RGB scale/systematic error and robot angle truth remain separate uncertainties.')
