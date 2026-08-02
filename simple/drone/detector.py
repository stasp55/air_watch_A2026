"""Cheburashka detector adapted from the supplied cv.py prototype.

It joins nearby brown/orange components before applying the area threshold.
The returned pixel is scaled back to the original camera image; object_finder
then projects it through TF and converts it to a field cell.
"""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

def detect(image, params: dict[str, Any], debug: bool = False) -> dict | None:
    """Return pixel, confidence and diagnostics for one BGR camera frame."""
    detector = _Detector(params)
    return detector.detect(image, debug)


class _Detector:

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        params = dict(params or {})
        self.brown_low = tuple(params.get("cheburashka_brown_low", (0, 134, 32)))
        self.brown_high = tuple(params.get("cheburashka_brown_high", (25, 255, 118)))
        self.orange_low = tuple(params.get("cheburashka_orange_low", (6, 126, 199)))
        self.orange_high = tuple(params.get("cheburashka_orange_high", (18, 255, 255)))
        # Keep the interpolated threshold as a float: component areas are
        # integers, but the decision boundary must vary continuously by height.
        self.min_area = float(params.get("cheburashka_min_area_px", 5000))
        self.merge_distance = float(params.get("cheburashka_merge_distance_px", 20))
        self.width = int(params.get("cheburashka_width", 640))
        self.height = int(params.get("cheburashka_height", 480))

    def detect(self, image, debug: bool = False) -> dict | None:
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
            return {"found": False, "debug_image": frame.copy()} if debug else None

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
        canvas = frame.copy() if debug else None
        best = None
        for group in groups.values():
            area = sum(item[4] for item in group)
            # Bounding coordinates are needed both for diagnostic drawing and
            # for the candidate; compute them before either branch uses them.
            x1, y1 = min(i[0] for i in group), min(i[1] for i in group)
            x2 = max(i[0] + i[2] for i in group)
            y2 = max(i[1] + i[3] for i in group)
            if debug:
                cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,165,255),1)
            if area < self.min_area:
                continue
            cx = sum(i[5] * i[4] for i in group) / area
            cy = sum(i[6] * i[4] for i in group) / area
            candidate = (area, x1, y1, x2 - x1, y2 - y1, cx, cy)
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is None:
            return {"found": False, "debug_image": canvas, "components": len(groups)} if debug else None
        area, x, y, w, h, cx, cy = best
        sx, sy = original_w / self.width, original_h / self.height
        confidence = min(1.0, 0.5 + 0.5 * area / max(1, self.min_area * 2))
        result = {"found": True, "pixel": (cx * sx, cy * sy), "confidence": confidence,
            "bbox": (round(x * sx), round(y * sy), round(w * sx), round(h * sy)),
            "area_px": area, "components": len(groups)}
        if debug:
            cv2.rectangle(canvas,(x,y),(x+w,y+h),(0,255,0),2); cv2.circle(canvas,(round(cx),round(cy)),5,(0,0,255),-1)
            result["debug_image"] = canvas
        return result
