import numpy as np


class YOLOv8ObjectDetector:
    def __init__(self, model_path='yolov8n.pt', target_class='', conf=0.35,
                 iou=0.45, device='cpu', imgsz=None, expected_task='detect',
                 require_instance_mask=False, model_backend=None, model_loader=None):
        self.model_path = str(model_path or 'yolov8n.pt')
        self.target_classes = self._normalize_targets(target_class)
        self.conf = float(conf)
        self.iou = float(iou)
        self.device = str(device or 'cpu')
        self.imgsz = self._positive_int(imgsz)
        self.model = model_backend if model_backend is not None else self._load_model(model_loader)
        self.expected_task = str(expected_task or 'detect').strip().lower()
        self.require_instance_mask = bool(require_instance_mask)
        actual_task = str(getattr(self.model, 'task', '') or '').strip().lower()
        if actual_task and actual_task != self.expected_task:
            raise RuntimeError(
                'YOLO checkpoint task mismatch: expected %s, got %s'
                % (self.expected_task, actual_task)
            )

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
        chosen = self._choose_candidate(candidates, preferred_uv, max_preferred_distance_px)
        mask = chosen.get('mask')
        if self.require_instance_mask and (mask is None or not chosen.get('mask_consistent', False)):
            return None, None
        return chosen, mask

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
            instance_mask = (
                self._instance_mask(result, index, image_shape)
                if self.expected_task == 'segment' else None
            )
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
            bbox = (x, y, width, height)
            mask_area, mask_centroid = self._mask_metrics(instance_mask)
            mask_consistent = self._mask_is_consistent(
                instance_mask, bbox, mask_area, mask_centroid
            )
            if not mask_consistent:
                instance_mask = None
                mask_area = 0
                mask_centroid = None
            candidates.append({
                'u': int(round(x + width * 0.5)),
                'v': int(round(y + height * 0.5)),
                'bbox': bbox,
                'area': area,
                'confidence': confidence,
                'score': confidence * max(1.0, area / max(1.0, frame_area)),
                'label': label,
                'class_id': class_id,
                'instance_index': index,
                'mask': instance_mask,
                'mask_area': mask_area,
                'mask_centroid': mask_centroid,
                'mask_consistent': mask_consistent,
            })
        return candidates

    @classmethod
    def _instance_mask(cls, result, index, image_shape):
        masks = getattr(result, 'masks', None)
        if masks is None:
            return None
        try:
            data = cls._to_numpy(getattr(masks, 'data', []))
        except (TypeError, ValueError, OverflowError, RuntimeError):
            return None
        if (
            data.ndim != 3
            or index < 0
            or index >= data.shape[0]
            or data.shape[1] <= 0
            or data.shape[2] <= 0
        ):
            return None
        try:
            mask = np.asarray(data[index], dtype=np.float32)
        except (TypeError, ValueError, OverflowError):
            return None
        if not np.all(np.isfinite(mask)):
            return None
        restored = cls._restore_mask(mask, image_shape[:2])
        if restored is None:
            return None
        binary = np.where(restored >= 0.5, 255, 0).astype(np.uint8)
        return binary if np.any(binary) else None

    @staticmethod
    def _restore_mask(mask, original_shape):
        import cv2
        try:
            mask = np.asarray(mask, dtype=np.float32)
            dst_h, dst_w = [int(value) for value in original_shape[:2]]
        except (TypeError, ValueError, OverflowError):
            return None
        if (
            mask.ndim != 2
            or mask.shape[0] <= 0
            or mask.shape[1] <= 0
            or dst_h <= 0
            or dst_w <= 0
        ):
            return None
        src_h, src_w = mask.shape[:2]
        if abs((src_w / float(src_h)) - (dst_w / float(dst_h))) > 1e-3:
            gain = min(src_w / float(dst_w), src_h / float(dst_h))
            used_w = int(round(dst_w * gain))
            used_h = int(round(dst_h * gain))
            if used_w <= 0 or used_h <= 0:
                return None
            left = max(0, (src_w - used_w) // 2)
            top = max(0, (src_h - used_h) // 2)
            mask = mask[top:top + used_h, left:left + used_w]
        if mask.size == 0:
            return None
        try:
            return cv2.resize(mask, (dst_w, dst_h), interpolation=cv2.INTER_NEAREST)
        except cv2.error:
            return None

    @staticmethod
    def _mask_metrics(mask):
        if mask is None:
            return 0, None
        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            return 0, None
        return int(len(xs)), (int(round(float(np.mean(xs)))), int(round(float(np.mean(ys)))))

    @staticmethod
    def _mask_is_consistent(mask, bbox, mask_area, mask_centroid):
        if mask is None or mask_area <= 0 or mask_centroid is None:
            return False
        x, y, width, height = bbox
        ys, xs = np.nonzero(mask)
        inside = (
            (xs >= x) & (xs < x + width)
            & (ys >= y) & (ys < y + height)
        )
        if float(np.count_nonzero(inside)) / float(mask_area) < 0.80:
            return False
        centroid_x, centroid_y = mask_centroid
        return (
            x - 2 <= centroid_x <= x + width + 2
            and y - 2 <= centroid_y <= y + height + 2
        )

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
