from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys

import numpy as np
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.grasp.grasp6d_candidate_selector import Grasp6DCandidate


@dataclass
class DependencyStatus:
    available: bool
    missing: list
    message: str


@dataclass
class RuntimeReport:
    ready: bool
    present: dict
    missing: list
    versions: dict
    message: str


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float


@dataclass
class GraspNetInput:
    model_input: dict
    scene_points: np.ndarray
    scene_colors: np.ndarray


class Grasp6DBackendUnavailable(RuntimeError):
    pass


def check_grasp6d_dependencies(importer=None):
    importer = importer or __import__
    missing = []
    for module_name in ('open3d', 'MinkowskiEngine', 'graspnetAPI'):
        try:
            importer(module_name)
        except Exception:
            missing.append(module_name)
    return DependencyStatus(
        available=not missing,
        missing=missing,
        message='ok' if not missing else 'missing 6D grasp dependencies: %s' % ', '.join(missing),
    )


def inspect_grasp6d_runtime(root=None, checkpoint_path='', importer=None):
    root = Path(root).expanduser() if root else default_grasp6d_root()
    importer = importer or __import__
    missing = []
    present = {}
    versions = {}

    for module_name in ('numpy', 'scipy', 'torch', 'open3d', 'MinkowskiEngine', 'graspnetAPI', 'tqdm', 'tensorboard'):
        try:
            module = importer(module_name)
            present[module_name] = True
            versions[module_name] = str(getattr(module, '__version__', 'unknown'))
        except Exception:
            present[module_name] = False
            missing.append('missing python module: %s' % module_name)

    for rel_path in (
        'models/graspnet.py',
        'utils/data_utils.py',
        'utils/collision_detector.py',
    ):
        exists = (root / rel_path).exists()
        present[rel_path] = exists
        if not exists:
            missing.append('missing source file: %s' % rel_path)

    for rel_path in ('dataset', 'pointnet2'):
        exists = (root / rel_path).is_dir()
        present[rel_path] = exists
        if not exists:
            missing.append('missing source dir: %s' % rel_path)

    checkpoint = Path(checkpoint_path).expanduser() if checkpoint_path else None
    if checkpoint is None:
        present['checkpoint'] = False
        missing.append('missing checkpoint_path')
    else:
        present['checkpoint'] = checkpoint.exists()
        if not checkpoint.exists():
            missing.append('checkpoint not found: %s' % checkpoint)

    try:
        torch = importer('torch')
        cuda_available = bool(torch.cuda.is_available())
        versions['torch_cuda_available'] = str(cuda_available)
        versions['torch_cuda_version'] = str(getattr(torch.version, 'cuda', None))
    except Exception:
        versions['torch_cuda_available'] = 'unknown'
        versions['torch_cuda_version'] = 'unknown'

    return RuntimeReport(
        ready=not missing,
        present=present,
        missing=missing,
        versions=versions,
        message='6D grasp runtime ready' if not missing else '; '.join(missing),
    )


def build_graspnet_input_from_rgbd(
    color_bgr,
    depth_raw,
    intrinsics,
    workspace_mask=None,
    num_points=50000,
    voxel_size=0.005,
    rng=None,
    grasp6d_root=None,
):
    data_utils = _load_grasp6d_data_utils(grasp6d_root)
    depth = np.asarray(depth_raw).astype(np.float32).copy()
    color = np.asarray(color_bgr).astype(np.float32) / 255.0
    color = color[:, :, ::-1]
    if workspace_mask is not None:
        mask = np.asarray(workspace_mask).astype(bool)
        depth[~mask] = 0.0
        color[~mask] = 0.0
    camera = data_utils.CameraInfo(
        int(intrinsics.width),
        int(intrinsics.height),
        float(intrinsics.fx),
        float(intrinsics.fy),
        float(intrinsics.cx),
        float(intrinsics.cy),
        1.0 / float(intrinsics.depth_scale),
    )
    cloud = data_utils.create_point_cloud_from_depth_image(depth, camera, organized=True)
    valid_mask = depth > 0.0
    scene_points = cloud[valid_mask].astype(np.float32)
    scene_colors = color[valid_mask].astype(np.float32)
    if scene_points.size == 0:
        raise ValueError('no valid depth points in 6D grasp workspace')
    indices = _sample_indices(len(scene_points), int(num_points), rng)
    sampled = scene_points[indices].astype(np.float32)
    model_input = {
        'point_clouds': sampled,
        'coors': (sampled / float(voxel_size)).astype(np.float32),
        'feats': np.ones_like(sampled).astype(np.float32),
    }
    return GraspNetInput(model_input=model_input, scene_points=scene_points, scene_colors=scene_colors)


class AliciaGrasp6DBackend:
    def __init__(
        self,
        root=None,
        checkpoint_path='',
        seed_feat_dim=512,
        collision_thresh=0.05,
        collision_voxel_size=0.01,
        device='cpu',
    ):
        self.root = Path(root).expanduser() if root else default_grasp6d_root()
        self.checkpoint_path = str(checkpoint_path or '')
        self.seed_feat_dim = int(seed_feat_dim)
        self.collision_thresh = float(collision_thresh)
        self.collision_voxel_size = float(collision_voxel_size)
        self.device_name = str(device or 'cpu')
        self._loaded = False

    def load(self):
        status = check_grasp6d_dependencies()
        if not status.available:
            raise Grasp6DBackendUnavailable(status.message)
        if not self.checkpoint_path:
            raise Grasp6DBackendUnavailable('missing /grasp_6d/checkpoint_path')
        checkpoint = Path(self.checkpoint_path).expanduser()
        if not checkpoint.exists():
            raise Grasp6DBackendUnavailable('6D grasp checkpoint not found: %s' % checkpoint)

        self._install_grasp6d_paths()
        try:
            import torch
            from graspnetAPI.graspnet_eval import GraspGroup
            from models.graspnet import GraspNet, pred_decode
            from dataset.graspnet_dataset import minkowski_collate_fn
            from utils.collision_detector import ModelFreeCollisionDetector
        except Exception as exc:
            raise Grasp6DBackendUnavailable('failed to import alicia_d_grasp_6d runtime: %s' % exc) from exc

        self.torch = torch
        self.GraspGroup = GraspGroup
        self.pred_decode = pred_decode
        self.minkowski_collate_fn = minkowski_collate_fn
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self.device = torch.device('cuda:0' if self.device_name.startswith('cuda') and torch.cuda.is_available() else 'cpu')
        self.net = GraspNet(seed_feat_dim=self.seed_feat_dim, is_training=False)
        self.net.to(self.device)
        ckpt = torch.load(str(checkpoint), map_location=self.device)
        self.net.load_state_dict(ckpt['model_state_dict'])
        self.net.eval()
        self._loaded = True
        return self

    def predict_candidates(self, graspnet_input):
        if not self._loaded:
            self.load()
        batch_data = self.minkowski_collate_fn([graspnet_input.model_input])
        for key in batch_data:
            if 'list' in key:
                for i in range(len(batch_data[key])):
                    for j in range(len(batch_data[key][i])):
                        batch_data[key][i][j] = batch_data[key][i][j].to(self.device)
            else:
                batch_data[key] = batch_data[key].to(self.device)

        with self.torch.no_grad():
            end_points = self.net(batch_data)
            grasp_preds = self.pred_decode(end_points)
        preds = grasp_preds[0].detach().cpu().numpy()
        grasp_group = self.GraspGroup(preds)
        if len(grasp_group) == 0:
            return []
        grasp_group.nms()
        if self.collision_thresh >= 0.0:
            detector = self.ModelFreeCollisionDetector(
                graspnet_input.scene_points,
                voxel_size=self.collision_voxel_size,
            )
            collision_mask = detector.detect(grasp_group, collision_thresh=self.collision_thresh)
            grasp_group = grasp_group[~collision_mask]
        if len(grasp_group) == 0:
            return []
        grasp_group.sort_by_score()
        return [_candidate_from_grasp(grasp) for grasp in grasp_group]

    def _install_grasp6d_paths(self):
        for path in (self.root, self.root / 'utils'):
            text = str(path)
            if text not in sys.path:
                sys.path.insert(0, text)


def default_grasp6d_root():
    return Path(__file__).resolve().parents[5] / 'src' / 'real-arm' / 'alicia_d_grasp_6d'


def _candidate_from_grasp(grasp):
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = np.asarray(grasp.rotation_matrix, dtype=float)
    quat = quaternion_from_matrix(mat)
    return Grasp6DCandidate(
        score=float(grasp.score),
        collision_free=True,
        reachable=True,
        tactile_score=1.0,
        pose_camera=(np.asarray(grasp.translation, dtype=float), quat),
        width_m=float(getattr(grasp, 'width', 0.0) or 0.0),
    )


def _sample_indices(point_count, num_points, rng):
    rng = rng or np.random.default_rng()
    if point_count >= num_points:
        return rng.choice(point_count, num_points, replace=False)
    base = np.arange(point_count)
    extra = rng.choice(point_count, num_points - point_count, replace=True)
    return np.concatenate([base, extra], axis=0)


def _load_grasp6d_data_utils(grasp6d_root=None):
    root = Path(grasp6d_root) if grasp6d_root is not None else default_grasp6d_root()
    path = root / 'utils' / 'data_utils.py'
    spec = importlib.util.spec_from_file_location('alicia_d_grasp_6d_data_utils', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
