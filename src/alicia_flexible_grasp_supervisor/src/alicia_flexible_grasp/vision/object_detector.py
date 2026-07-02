import cv2
import numpy as np

class HSVObjectDetector:
    def __init__(self, lower=(35,40,40), upper=(85,255,255), min_area=300, hsv_ranges=None, shape='any'):
        self.ranges = []
        if hsv_ranges:
            for lower_value, upper_value in hsv_ranges:
                self.ranges.append((
                    np.array(lower_value, dtype=np.uint8),
                    np.array(upper_value, dtype=np.uint8),
                ))
        if not self.ranges:
            self.ranges.append((
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            ))
        self.min_area = float(min_area)
        self.shape = str(shape or 'any').lower()

    def detect(self, bgr, preferred_uv=None, max_preferred_distance_px=None):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask
        candidates = []
        for contour in contours:
            candidate = self._make_candidate(contour, bgr.shape)
            if candidate is not None:
                candidates.append(candidate)
        if not candidates:
            return None, mask
        chosen = self._choose_candidate(candidates, preferred_uv, max_preferred_distance_px)
        return chosen, mask

    def _make_candidate(self, contour, image_shape):
        area = cv2.contourArea(contour)
        if area < self.min_area:
            return None
        matches, shape_score = self._shape_score(contour)
        if not matches:
            return None
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None
        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])
        x, y, w, h = cv2.boundingRect(contour)
        frame_area = float(image_shape[0] * image_shape[1])
        confidence = min(1.0, (area / frame_area) * max(0.25, shape_score))
        return {
            'u': u,
            'v': v,
            'bbox': (x, y, w, h),
            'area': area,
            'confidence': confidence,
            'score': area * max(0.25, shape_score),
            'shape_score': shape_score,
        }

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
                return max(near, key=lambda item: item['score'])
        return max(candidates, key=lambda item: item['score'])

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

    def _shape_matches(self, contour):
        return self._shape_score(contour)[0]

    def _shape_score(self, contour):
        if self.shape in ('', 'any', 'all', 'object', 'target'):
            return True, 1.0
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6 or area <= 1e-6:
            return False, 0.0
        approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
        vertices = len(approx)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        x, y, w, h = cv2.boundingRect(contour)
        aspect = float(w) / float(h) if h else 0.0

        if self.shape == 'circle':
            (_, _), radius = cv2.minEnclosingCircle(contour)
            enclosing_area = np.pi * radius * radius if radius > 1e-6 else 0.0
            fill_ratio = area / enclosing_area if enclosing_area > 1e-6 else 0.0
            near_square_bounds = 0.72 <= aspect <= 1.30
            if not (near_square_bounds and circularity > 0.40 and fill_ratio > 0.55 and vertices >= 5):
                return False, 0.0
            aspect_score = max(0.0, 1.0 - abs(aspect - 1.0) / 0.30)
            circularity_score = max(0.0, min(1.0, (circularity - 0.40) / 0.50))
            fill_score = max(0.0, min(1.0, (fill_ratio - 0.55) / 0.35))
            score = 0.45 * aspect_score + 0.25 * fill_score + 0.30 * circularity_score
            return True, score
        if self.shape == 'triangle':
            return vertices == 3, 1.0 if vertices == 3 else 0.0
        if self.shape == 'square':
            matched = vertices == 4 and 0.75 <= aspect <= 1.33
            return matched, 1.0 if matched else 0.0
        if self.shape == 'rectangle':
            return vertices == 4, 1.0 if vertices == 4 else 0.0
        return True, 1.0
