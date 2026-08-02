"""Cheburashka detector adapted from the supplied cv.py prototype.

It joins nearby brown/orange components before applying the area threshold.
The returned pixel is scaled back to the original camera image; object_finder
then projects it through TF and converts it to a field cell.
"""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .base import DetectionResult, ObjectDetector, register_detector


@register_detector
class CheburashkaHsvDetector(ObjectDetector):
    name = "cheburashka_hsv"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        self.brown_low = tuple(self.params.get("cheburashka_brown_low", (0, 134, 32)))
        self.brown_high = tuple(self.params.get("cheburashka_brown_high", (25, 255, 118)))
        self.orange_low = tuple(self.params.get("cheburashka_orange_low", (6, 126, 199)))
        self.orange_high = tuple(self.params.get("cheburashka_orange_high", (18, 255, 255)))
        self.min_area = int(self.params.get("cheburashka_min_area_px", 5000))
        self.merge_distance = float(self.params.get("cheburashka_merge_distance_px", 20))
        self.width = int(self.params.get("cheburashka_width", 640))
        self.height = int(self.params.get("cheburashka_height", 480))

    def detect(self, image) -> DetectionResult | None:
        if image is None or image.size == 0:
            return None
        original_h, original_w = image.shape[:2]
        frame = cv2.resize(image, (self.width, self.height))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array(self.brown_low, np.uint8), np.array(self.brown_high, np.uint8)),
            cv2.inRange(hsv, np.array(self.orange_low, np.uint8), np.array(self.orange_high, np.uint8)),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        parts = []
        for index in range(1, count):
            x, y, w, h, area = stats[index]
            if area:
                parts.append((int(x), int(y), int(w), int(h), int(area), *centroids[index]))
        if not parts:
            return None

        parent = list(range(len(parts)))
        def root(i: int) -> int:
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        def distance(a, b) -> float:
            ax, ay, aw, ah, *_ = a; bx, by, bw, bh, *_ = b
            dx = max(0, ax - (bx + bw), bx - (ax + aw))
            dy = max(0, ay - (by + bh), by - (ay + ah))
            return float(np.hypot(dx, dy))
        for i in range(len(parts)):
            for j in range(i + 1, len(parts)):
                if distance(parts[i], parts[j]) <= self.merge_distance:
                    parent[root(j)] = root(i)

        groups: dict[int, list[tuple]] = {}
        for i, part in enumerate(parts):
            groups.setdefault(root(i), []).append(part)
        best = None
        for group in groups.values():
            area = sum(item[4] for item in group)
            if area < self.min_area:
                continue
            x1, y1 = min(i[0] for i in group), min(i[1] for i in group)
            x2 = max(i[0] + i[2] for i in group); y2 = max(i[1] + i[3] for i in group)
            cx = sum(i[5] * i[4] for i in group) / area
            cy = sum(i[6] * i[4] for i in group) / area
            candidate = (area, x1, y1, x2 - x1, y2 - y1, cx, cy)
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is None:
            return None
        area, x, y, w, h, cx, cy = best
        sx, sy = original_w / self.width, original_h / self.height
        confidence = min(1.0, 0.5 + 0.5 * area / max(1, self.min_area * 2))
        return DetectionResult(pixel=(cx * sx, cy * sy), confidence=confidence,
            bbox=(round(x * sx), round(y * sy), round(w * sx), round(h * sy)),
            label="cheburashka", debug={"area_px": area, "components": len(groups)})
