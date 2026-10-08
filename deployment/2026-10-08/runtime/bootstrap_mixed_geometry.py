"""Pure continuous geometry bound for the explicit initial-sync L1 response.

No ROS I/O. This certifies modeled gripper/support and full-board projection,
conditional on the measured six-axis response remaining in the stated L1 ball.
It is not a whole-arm collision or hardware stopping guarantee.
"""
import math
import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners
from alicia_flexible_grasp.robot.observation_path_guard import _axis_rotation


def _interval(lo, hi=None):
    """Outward enclosure includes real arithmetic and SDK float32 rounding."""
    if hi is None:
        hi = lo
    if not np.isfinite([lo, hi]).all() or lo > hi:
        raise RuntimeError('invalid projection interval')
    low = min(float(lo), float(np.nextafter(np.float32(lo), np.float32(-np.inf))))
    high = max(float(hi), float(np.nextafter(np.float32(hi), np.float32(np.inf))))
    if not np.isfinite([low, high]).all():
        raise RuntimeError('projection interval overflow')
    return low, high


def _add(a, b):
    return _interval(a[0]+b[0], a[1]+b[1])


def _mul(a, b):
    values = [x*y for x in a for y in b]
    return _interval(min(values), max(values))


def _square(a):
    return _interval(0. if a[0] <= 0. <= a[1] else min(v*v for v in a),
                     max(v*v for v in a))


def _scale(value, interval):
    return _mul(_interval(float(value)), interval)


def _project_normalized_rectangle(model, x, y):
    """Official SDK 2.56.5 Brown polynomial with outward float intervals."""
    if model.backend_sdk_version != '2.56.5':
        raise RuntimeError('continuous projection requires verified SDK 2.56.5')
    ci, intr = model.evidence['color_intrinsics'], model.intrinsics
    kind = ci['model']
    if kind not in ('none', 'inverse_brown_conrady', 'modified_brown_conrady', 'brown_conrady'):
        raise RuntimeError('unsupported continuous projection model')
    if (intr.model != getattr(model.sdk.distortion, kind)
            or intr.width != ci['width'] or intr.height != ci['height']):
        raise RuntimeError('projection intrinsics differ from bound evidence')
    if (not np.isfinite([intr.fx, intr.fy, intr.ppx, intr.ppy, *intr.coeffs]).all()
            or min(intr.fx, intr.fy) <= 0):
        raise RuntimeError('invalid projection intrinsics')
    if kind != 'none':
        k = list(map(float, intr.coeffs))
        r2 = _add(_square(x), _square(y))
        f = _add(_add(_add(_interval(1.), _scale(k[0], r2)),
                      _mul(_scale(k[1], r2), r2)),
                 _mul(_mul(_scale(k[4], r2), r2), r2))
        xf, yf = _mul(x, f), _mul(y, f)
        tx, ty = (xf, yf) if kind in ('inverse_brown_conrady', 'modified_brown_conrady') else (x, y)
        dx = _add(_add(xf, _mul(_scale(2.*k[2], tx), ty)),
                  _scale(k[3], _add(r2, _scale(2., _square(tx)))))
        dy = _add(_add(yf, _mul(_scale(2.*k[3], tx), ty)),
                  _scale(k[2], _add(r2, _scale(2., _square(ty)))))
        x, y = dx, dy
    return np.asarray([_add(_scale(intr.fx, x), _interval(intr.ppx)),
                       _add(_scale(intr.fy, y), _interval(intr.ppy))])



def _normalized_box(optical_lo, optical_hi):
    if (np.shape(optical_lo)!=(3,) or np.shape(optical_hi)!=(3,)
            or not np.isfinite([optical_lo,optical_hi]).all()
            or np.any(np.asarray(optical_lo)>optical_hi) or optical_lo[2]<=0):
        raise RuntimeError('invalid positive-depth projection box')
    coordinates=[_interval(a,b) for a,b in zip(optical_lo,optical_hi)]
    if coordinates[2][0]<=0:raise RuntimeError('rounded depth interval includes zero')
    inverse_z=_interval(1./coordinates[2][1],1./coordinates[2][0])
    return [_mul(c,inverse_z) for c in coordinates[:2]]


def projection_rectangle(model, optical_lo, optical_hi):
    """Compatible full optical XYZ-box projection; all interior points covered."""
    x,y=_normalized_box(optical_lo,optical_hi)
    return _project_normalized_rectangle(model,x,y)


def projection_corner_boxes(model, camera_lower, camera_upper):
    """Full convex board projection from four camera_link corner enclosures.

    At every fixed posture, the board point P=sum(lambda_i*P_i) has normalized
    optical coordinates sum(w_i * (x_i/z_i,y_i/z_i)), where w_i=lambda_i*z_i /
    sum(lambda_j*z_j). Positive depths make these weights nonnegative and sum
    to one. Thus normalized extrema lie within the four corner intervals.
    Normalize each corner BEFORE taking the union, then bound SDK distortion
    over the ENTIRE normalized rectangle. This is not corner-only pixel testing.
    Corner identities must match between lower/upper and all merged postures.
    """
    lower,upper=np.asarray(camera_lower,dtype=float),np.asarray(camera_upper,dtype=float)
    if lower.shape!=(4,3) or upper.shape!=(4,3):
        raise RuntimeError('invalid four-corner positive-depth enclosures')
    return projection_corner_box_batch(model,lower[None,...],upper[None,...])


def _array_interval(lower,upper):
    """Vector counterpart of _interval: enclose real and float32 endpoints."""
    lo=np.minimum(lower,np.nextafter(np.asarray(lower,dtype=np.float32),np.float32(-np.inf)).astype(float))
    hi=np.maximum(upper,np.nextafter(np.asarray(upper,dtype=np.float32),np.float32(np.inf)).astype(float))
    if not np.isfinite([lo,hi]).all():raise RuntimeError('corner interval overflow')
    return lo,hi


def projection_corner_box_batch(model,camera_lower,camera_upper):
    """Nx4x3 cell/corner enclosures; normalize before any cross-cell union.

    Vectorized outward float32 intervals preserve the same per-corner positive
    depth convex-hull proof as projection_corner_boxes. Only the normalized
    x/y intervals are unioned across corners AND cells; SDK Brown polynomial
    interval evaluation then happens once. No motion sampling is used here.
    """
    lower,upper=np.asarray(camera_lower,dtype=float),np.asarray(camera_upper,dtype=float)
    if (lower.ndim!=3 or lower.shape[1:]!=(4,3) or lower.shape[0]<1
            or upper.shape!=lower.shape or not np.isfinite([lower,upper]).all()
            or np.any(lower>upper) or np.any(lower[:,:,0]<=0)):
        raise RuntimeError('invalid batched four-corner positive-depth enclosures')
    optical_lo=np.stack((-upper[:,:,1],-upper[:,:,2],lower[:,:,0]),axis=2)
    optical_hi=np.stack((-lower[:,:,1],-lower[:,:,2],upper[:,:,0]),axis=2)
    lo,hi=_array_interval(optical_lo,optical_hi)
    if np.any(lo[:,:,2]<=0):raise RuntimeError('rounded corner depth includes zero')
    invlo,invhi=_array_interval(1./hi[:,:,2],1./lo[:,:,2])
    products=np.stack((lo[:,:,:2]*invlo[:,:,None],lo[:,:,:2]*invhi[:,:,None],
                       hi[:,:,:2]*invlo[:,:,None],hi[:,:,:2]*invhi[:,:,None]),axis=0)
    xylo,xyhi=_array_interval(products.min(axis=0),products.max(axis=0))
    x=(float(xylo[:,:,0].min()),float(xyhi[:,:,0].max()))
    y=(float(xylo[:,:,1].min()),float(xyhi[:,:,1].max()))
    return _project_normalized_rectangle(model,x,y)


def _rigid(value):
    a = np.asarray(value, dtype=float)
    if (a.shape != (4, 4) or not np.isfinite(a).all()
            or not np.allclose(a[3], [0, 0, 0, 1], rtol=0, atol=1e-10)
            or not np.allclose(a[:3, :3].T@a[:3, :3], np.eye(3), rtol=0, atol=1e-8)
            or abs(np.linalg.det(a[:3, :3])-1.) > 1e-8):
        raise RuntimeError('continuous geometry requires rigid transforms')
    return a


def _continuous_geometry(fk, gripper, measured, geometry, lower, upper, bound_deg):
    """Enclose q=q0+d for ALL sum(abs(d))<=rho, not only six axis rays.

    Serial revolute point speed satisfies |dp/dq_i|<=R_i at every posture.
    Integrating along q0+t*d gives displacement <=rho*max(R_i). Camera
    board-camera coordinates use an exact first derivative plus a global
    second-order Taylor remainder, separately for each coordinate. The full
    board is a convex hull: corner interval extrema enclose every interior
    point at every posture in the ball, before interval SDK projection.
    """
    measured = np.asarray(measured, dtype=float)
    if (measured.shape != (7,) or not np.isfinite(measured).all()
            or not 0 <= measured[6] <= gripper.max_inner_gap_m
            or not math.isfinite(bound_deg) or not 0. < bound_deg <= 4.8):
        raise RuntimeError('invalid continuous geometry inputs')
    rho = math.radians(bound_deg)
    lo = np.maximum(np.asarray(lower, dtype=float), fk.limits[:, 0])
    hi = np.minimum(np.asarray(upper, dtype=float), fk.limits[:, 1])
    if (lo.shape != (6,) or hi.shape != (6,) or not np.isfinite([lo, hi]).all()
            or np.any(measured[:6]-rho < lo) or np.any(measured[:6]+rho > hi)):
        raise RuntimeError('mixed initial sync ball exceeds unchanged joint limits')
    tool, x, y = _rigid(fk(measured[:6])), _rigid(geometry['X']), _rigid(geometry['Y'])
    normal = y[:3, 2].copy()
    if normal[2] < 0:
        normal *= -1
    normal /= np.linalg.norm(normal)
    d = -float(normal@y[:3, 3])
    local = np.concatenate([_box_corners(c, np.eye(3), size)
        for _, c, size in _stage_boxes(np.eye(4), gripper, float(measured[6]), 'y', 'z')])
    nominal = (local@tool[:3, :3].T+tool[:3, 3])@normal+d
    point_allowances = np.asarray([rho*max(fk.corner_motion_radii(p)) for p in local])
    # This numerical padding tightens physical gates; it never expands them.
    clearance = float(np.min(nominal-point_allowances)-1e-9)
    camera_translation_bound = rho*max(fk.corner_motion_radii(x[:3, 3]))
    taylor = board_camera_taylor_box(fk, measured[:6], x, y, rho)
    a, b = np.asarray(taylor['lower_xyz_m']), np.asarray(taylor['upper_xyz_m'])
    camera_vector_bound = taylor['maximum_vector_displacement_upper_bound_m']
    if a[0] < .12:
        raise RuntimeError('mixed initial sync board depth lower bound below 120 mm')
    # camera_link (+x forward, +y left, +z up) -> optical coordinates.
    pixel = projection_corner_boxes(geometry['model'], taylor['corner_lower_xyz_m'],
                                    taylor['corner_upper_xyz_m'])
    ci = geometry['model'].evidence['color_intrinsics']
    edge = float(min(pixel[0, 0], ci['width']-1-pixel[0, 1],
                     pixel[1, 0], ci['height']-1-pixel[1, 1]))
    if clearance < .04 or edge < 20.:
        raise RuntimeError('continuous mixed initial sync violates 40 mm plane / 20 px board margin '
                           '(lower bounds %.6f m, %.3f px)' % (clearance, edge))
    return dict(kind='continuous_six_axis_L1_initial_sync_geometry',
        response_total_l1_bound_deg=float(bound_deg),
        minimum_plane_clearance_lower_bound_m=clearance,
        minimum_full_board_edge_lower_bound_px=edge,
        minimum_board_depth_lower_bound_m=float(a[0]),
        camera_translation_upper_bound_m=float(camera_translation_bound),
        full_board_camera_vector_displacement_upper_bound_m=camera_vector_bound,
        full_board_pixel_rectangle_uv=pixel.tolist(),
        camera_coordinate_taylor_bound=taylor,
        robot_description_sha256=fk.model_sha256,
        camera_model_fingerprint=geometry['model'].fingerprint,
        proof='global CAD radii + cornerwise inverse-chain Taylor bounds + positive-depth normalized convex hull + outward SDK polynomial intervals',
        whole_arm_collision_certified=False, hardware_tracking_or_stopping_certified=False)


def board_camera_taylor_box(fk, q, x, y, rho):
    """Full-board coordinate enclosure for the WHOLE joint L1 ball.

    Write C(q)^-1 P as a sum of rotated constant vectors by expanding the
    inverse serial transforms. Each term contains a given joint rotation at
    most once. Every first, repeated or mixed second rotation derivative has
    operator norm <=1, including interleaved fixed rotations. Therefore EVERY
    component of EVERY Hessian entry is bounded by
      S(P) = |P| + sum(|origin_translation|) + |X_translation|.
    The L1 Taylor remainder is at most S(P)*rho^2/2 in each coordinate.
    The exact derivative is -R_camera.T @ [axis_i x (P-origin_i)].

    P in the board interior is a convex combination of its corners. The exact
    coordinate at any q has that same combination; thus enclosing every
    corner coordinate over all q also encloses the entire board interior.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all() or not math.isfinite(rho) or not 0. < rho <= math.radians(4.8):
        raise RuntimeError('invalid board Taylor radius or joint vector')
    x, y = _rigid(x), _rigid(y)
    camera = _rigid(fk(q))@x
    board = np.asarray([[cx, cy, 0., 1.] for cx in (0., .18) for cy in (0., .135)])
    world = (y@board.T).T[:, :3]
    nominal = (np.linalg.inv(camera)@y@board.T).T[:, :3]
    jacobian = np.zeros((4, 3, 6))
    pose = np.eye(4)
    for origin, index, axis in fk.chain:
        pose = pose@origin
        if index is not None:
            world_axis = pose[:3, :3]@axis
            jacobian[:, :, index] = -np.cross(world_axis, world-pose[:3, 3])@camera[:3, :3]
            pose[:3, :3] = pose[:3, :3]@_axis_rotation(axis, q[index])
    constant_sum = (sum(float(np.linalg.norm(origin[:3, 3])) for origin, _, _ in fk.chain)
                    + float(np.linalg.norm(x[:3, 3])))
    hessian_bounds = np.linalg.norm(world, axis=1)+constant_sum
    first = rho*np.max(np.abs(jacobian), axis=2)
    remainder = .5*hessian_bounds*rho*rho
    allowance = first+remainder[:, None]+1e-9
    return dict(corner_lower_xyz_m=(nominal-allowance).tolist(),
        corner_upper_xyz_m=(nominal+allowance).tolist(),
        lower_xyz_m=np.min(nominal-allowance,axis=0).tolist(),
        upper_xyz_m=np.max(nominal+allowance,axis=0).tolist(),
        corner_nominal_xyz_m=nominal.tolist(),
        corner_jacobian_xyz_per_joint=jacobian.tolist(),
        corner_first_order_coordinate_allowance_m=first.tolist(),
        corner_second_order_coordinate_allowance_m=remainder.tolist(),
        corner_hessian_entry_vector_norm_bound_m=hessian_bounds.tolist(),
        maximum_vector_displacement_upper_bound_m=float(np.max(np.linalg.norm(allowance,axis=1))),
        joint_l1_radius_rad=float(rho))


def mixed_sync_geometry(fk, gripper, measured, geometry, lower, upper, bound_deg):
    """Original initial-sync policy remains exactly 2 or 4.8 degrees."""
    if bound_deg not in (2., 4.8):
        raise RuntimeError('invalid explicit initial sync geometry inputs')
    return _continuous_geometry(fk,gripper,measured,geometry,lower,upper,bound_deg)


def certify_local_cell_geometry(fk, gripper, measured, geometry, lower, upper, l1_radius_deg):
    """Pure local-cell proof; NEVER authorizes a command or widens CLI policy.

    Caller must enclose the ENTIRE joint cell (including unedited-axis error
    allowances) in sum(abs(q-measured)) <= radians(l1_radius_deg).
    Fresh scene binding, actual MoveIt path checks, ownership and feedback
    enforcement remain the caller's responsibility. Conservative failure
    raises RuntimeError instead of relaxing any geometric threshold.
    """
    if not math.isfinite(l1_radius_deg) or not 0. < l1_radius_deg <= 2.:
        raise RuntimeError('local cell L1 radius must be positive and at most 2 degrees')
    proof = _continuous_geometry(fk,gripper,measured,geometry,lower,upper,l1_radius_deg)
    proof['kind'] = 'continuous_six_axis_L1_local_cell_geometry'
    return proof


def _joint_box_half_widths(half_width_rad):
    half=np.asarray(half_width_rad,dtype=float)
    if (half.shape!=(6,) or not np.isfinite(half).all() or np.any(half<0.)
            or float(np.sum(half))>math.radians(2.)):
        raise RuntimeError('joint box needs six finite nonnegative half-widths with sum at most 2 degrees')
    return half.copy()


def board_camera_box_taylor_bound(fk,q,x,y,half_width_rad):
    """Whole explicit joint BOX, with no conversion to a larger L1 first order.

    For |dq_i|<=h_i, each coordinate's exact linear support is sum |J_i|h_i.
    The same globally bounded Hessian vector used by the L1 proof bounds the
    integral remainder by .5*S*(sum h_i)^2. This covers every continuous point
    of the box. Board interior then follows by the existing positive-depth
    normalized convex-hull argument, not by sampling its corners alone.
    """
    half=_joint_box_half_widths(half_width_rad);rho=float(np.sum(half))
    # Reuse only the exact derivatives and global Hessian constants. Existing
    # L1 callers and their first-order allowance remain completely unchanged.
    t=board_camera_taylor_box(fk,q,x,y,max(rho,1e-12))
    nominal=np.asarray(t['corner_nominal_xyz_m'])
    jacobian=np.asarray(t['corner_jacobian_xyz_per_joint'])
    hessian=np.asarray(t['corner_hessian_entry_vector_norm_bound_m'])
    first=np.einsum('pki,i->pk',np.abs(jacobian),half)
    remainder=.5*hessian*rho*rho
    allowance=first+remainder[:,None]+1e-9
    t.update(corner_lower_xyz_m=(nominal-allowance).tolist(),
        corner_upper_xyz_m=(nominal+allowance).tolist(),
        lower_xyz_m=np.min(nominal-allowance,axis=0).tolist(),
        upper_xyz_m=np.max(nominal+allowance,axis=0).tolist(),
        corner_first_order_coordinate_allowance_m=first.tolist(),
        corner_second_order_coordinate_allowance_m=remainder.tolist(),
        maximum_vector_displacement_upper_bound_m=float(np.max(np.linalg.norm(allowance,axis=1))),
        joint_half_width_rad=half.tolist(),joint_l1_radius_rad=rho,
        domain='explicit_axis_aligned_joint_box')
    return t


def certify_joint_box_geometry(fk,gripper,center,geometry,lower,upper,half_width_rad):
    """Pure complete six-axis box proof; thresholds/feedback policy unchanged.

    Every modeled CAD corner displacement is <=sum_i h_i*R_i, since each
    global R_i bounds point speed at every pose and integration follows the
    straight joint-space segment. Limits test the real box, not a larger ball.
    The caller still owns fresh sources, whole-arm MoveIt and feedback bounds.
    """
    half=_joint_box_half_widths(half_width_rad);center=np.asarray(center,dtype=float)
    if (center.shape!=(7,) or not np.isfinite(center).all()
            or not 0<=center[6]<=gripper.max_inner_gap_m):
        raise RuntimeError('invalid joint box center or gripper opening')
    lower,upper=np.asarray(lower,dtype=float),np.asarray(upper,dtype=float)
    if lower.shape!=(6,) or upper.shape!=(6,) or not np.isfinite([lower,upper]).all():
        raise RuntimeError('invalid joint box limits')
    lo=np.maximum(lower,fk.limits[:,0]);hi=np.minimum(upper,fk.limits[:,1])
    if np.any(lo>hi) or np.any(center[:6]-half<lo) or np.any(center[:6]+half>hi):
        raise RuntimeError('explicit joint box exceeds unchanged joint limits')
    tool,x,y=_rigid(fk(center[:6])),_rigid(geometry['X']),_rigid(geometry['Y'])
    normal=y[:3,2].copy()
    if normal[2]<0:normal=-normal
    normal/=np.linalg.norm(normal);offset=-float(normal@y[:3,3])
    local=np.concatenate([_box_corners(c,np.eye(3),size)
        for _,c,size in _stage_boxes(np.eye(4),gripper,float(center[6]),'y','z')])
    nominal=(local@tool[:3,:3].T+tool[:3,3])@normal+offset
    allowances=np.asarray([np.dot(fk.corner_motion_radii(p),half) for p in local])
    clearance=float(np.min(nominal-allowances)-1e-9)
    t=board_camera_box_taylor_bound(fk,center[:6],x,y,half)
    depth=float(t['lower_xyz_m'][0])
    if depth<.12:raise RuntimeError('explicit joint box board depth lower bound below 120 mm')
    pixel=projection_corner_boxes(geometry['model'],t['corner_lower_xyz_m'],t['corner_upper_xyz_m'])
    ci=geometry['model'].evidence['color_intrinsics']
    edge=float(min(pixel[0,0],ci['width']-1-pixel[0,1],pixel[1,0],ci['height']-1-pixel[1,1]))
    if clearance<.04 or edge<20.:
        raise RuntimeError('continuous explicit joint box violates 40 mm plane / 20 px board margin '
            '(lower bounds %.6f m, %.3f px)'%(clearance,edge))
    return dict(kind='continuous_six_axis_explicit_joint_box_geometry',
        joint_half_width_rad=half.tolist(),joint_box_lower_rad=(center[:6]-half).tolist(),
        joint_box_upper_rad=(center[:6]+half).tolist(),
        response_total_l1_bound_deg=math.degrees(float(np.sum(half))),
        minimum_plane_clearance_lower_bound_m=clearance,
        minimum_full_board_edge_lower_bound_px=edge,minimum_board_depth_lower_bound_m=depth,
        camera_translation_upper_bound_m=float(np.dot(fk.corner_motion_radii(x[:3,3]),half)),
        full_board_camera_vector_displacement_upper_bound_m=t['maximum_vector_displacement_upper_bound_m'],
        full_board_pixel_rectangle_uv=pixel.tolist(),camera_coordinate_taylor_bound=t,
        robot_description_sha256=fk.model_sha256,camera_model_fingerprint=geometry['model'].fingerprint,
        proof='explicit per-axis linear supports + global integral Hessian remainder + CAD integrated per-axis radii + positive-depth normalized convex hull + outward SDK projection',
        whole_arm_collision_certified=False,hardware_tracking_or_stopping_certified=False)
