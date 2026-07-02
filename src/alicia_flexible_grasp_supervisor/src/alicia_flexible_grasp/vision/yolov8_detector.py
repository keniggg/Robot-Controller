import numpy as np


class YOLOv8ObjectDetector:
    def __init__(self, model_path='yolov8n.pt', target_class='', conf=0.35,
                 iou=0.45, device='cpu', imgsz=None, model_backend=None, model_loader=None):
        self.model_path = str(model_path or 'yolov8n.pt')
        self.target_classes = self._normalize_targets(target_class)
        self.conf = float(conf)
        self.iou = float(iou)
        self.device = str(device or 'cpu')
        self.imgsz = self._positive_int(imgsz)
        self.model = model_backend if model_backend is not None else self._load_model(model_loader)

    def detect(self, bgr, preferred_uv=None, max_preferred_distance_px=None):
        kwargs = {
            'conf': self.conf,
            'iou': self.iou,
            'device': self.device,
            'verbose': False,
        }
        if self.imgsz is not None:
            kwargs['imgsz'] = self.imgsz
        results = self.model.predict(bgr, **kwargs)
        candidates = []
        for result in results or []:
            candidates.extend(self._result_candidates(result, bgr.shape))
        if not candidates:
            return None, None
        return self._choose_candidate(candidates, preferred_uv, max_preferred_distance_px), None

    def _load_model(self, model_loader):
        try:
            if model_loader is None:
                from ultralytics import YOLO
                model_loader = YOLO
            return model_loader(self.model_path)
        except ImportError as exc:
            raise RuntimeError(
                'YOLOv8 requires ultralytics/torch. Install with: '
                'pip3 install ultralytics torch torchvision'
            ) from exc
        except Exception as exc:
            raise RuntimeError('Failed to load YOLOv8 model %s: %s' % (self.model_path, exc)) from exc

    def _result_candidates(self, result, image_shape):
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return []
        xyxy = self._to_numpy(getattr(boxes, 'xyxy', []))
        confs = self._to_numpy(getattr(boxes, 'conf', []))
        classes = self._to_numpy(getattr(boxes, 'cls', []))
        names = getattr(result, 'names', None) or getattr(self.model, 'names', {})
        frame_area = float(image_shape[0] * image_shape[1])
        candidates = []
        for index, box in enumerate(xyxy):
            if len(box) < 4:
                continue
            confidence = float(confs[index]) if index < len(confs) else 0.0
            class_id = int(classes[index]) if index < len(classes) else -1
            label = self._class_name(names, class_id)
            if self.target_classes and label.lower() not in self.target_classes:
                continue
            x0, y0, x1, y1 = [float(v) for v in box[:4]]
            x = max(0, int(round(x0)))
            y = max(0, int(round(y0)))
            width = max(0, int(round(x1 - x0)))
            height = max(0, int(round(y1 - y0)))
            if width <= 0 or height <= 0:
                continue
            area = float(width * height)
            candidates.append({
                'u': int(round(x + width * 0.5)),
                'v': int(round(y + height * 0.5)),
                'bbox': (x, y, width, height),
                'area': area,
                'confidence': confidence,
                'score': confidence * max(1.0, area / max(1.0, frame_area)),
                'label': label,
                'class_id': class_id,
            })
        return candidates

    def _choose_candidate(self, candidates, preferred_uv, max_preferred_distance_px):
        preferred = self._preferred_point(preferred_uv)
        max_distance = self._positive_float(max_preferred_distance_px)
        if preferred is not None and max_distance is not None:
            px, py = preferred
            near = []
            for candidate in candidates:
                distance = float(np.hypot(candidate['u'] - px, candidate['v'] - py))
                if distance <= max_distance:
                    near.append(candidate)
            if near:
                return max(near, key=lambda item: item['confidence'])
        return max(candidates, key=lambda item: item['confidence'])

    @staticmethod
    def _to_numpy(value):
        if hasattr(value, 'cpu'):
            value = value.cpu()
        if hasattr(value, 'numpy'):
            value = value.numpy()
        return np.asarray(value)

    @staticmethod
    def _class_name(names, class_id):
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        try:
            return str(names[class_id])
        except Exception:
            return str(class_id)

    @staticmethod
    def _normalize_targets(value):
        if value is None:
            return set()
        if isinstance(value, str):
            items = value.replace(';', ',').split(',')
        else:
            items = list(value)
        return set(str(item).strip().lower() for item in items if str(item).strip())

    @staticmethod
    def _preferred_point(value):
        if value is None:
            return None
        try:
            u, v = value
            return float(u), float(v)
        except Exception:
            return None

    @staticmethod
    def _positive_float(value):
        if value is None:
            return None
        try:
            value = float(value)
        except Exception:
            return None
        return value if value > 0.0 else None

    @staticmethod
    def _positive_int(value):
        if value is None:
            return None
        try:
            value = int(value)
        except Exception:
            return None
        return value if value > 0 else None
