"""Pure-Python filtering for stable, one-shot ArUco detections."""

from typing import Iterable, Optional, Set


class StableMarkerGate:
    """Confirm one marker across frames while the arm is ready to move."""

    def __init__(self, allowed_ids: Iterable[int], required_frames: int) -> None:
        self.allowed_ids: Set[int] = {int(value) for value in allowed_ids}
        if not self.allowed_ids:
            raise ValueError("allowed_ids에는 하나 이상의 마커 ID가 필요합니다.")
        self.required_frames = int(required_frames)
        if self.required_frames < 1:
            raise ValueError("required_frames는 1 이상이어야 합니다.")

        self.ready = False
        self.armed = False
        self.published_ids: Set[int] = set()
        self.candidate: Optional[int] = None
        self.consecutive_frames = 0

    def set_ready(self, ready: bool) -> None:
        """Arm on a transition into READY and disarm while the arm is moving."""

        ready = bool(ready)
        if ready and not self.ready:
            self.armed = True
            self._reset_candidate()
        elif not ready:
            self.armed = False
            self._reset_candidate()
        self.ready = ready

    def observe(self, detected_ids: Iterable[int]) -> Optional[int]:
        """Return an ID once it is stable enough to publish, otherwise ``None``."""

        if not self.ready or not self.armed:
            self._reset_candidate()
            return None

        candidates = sorted(
            {
                int(value)
                for value in detected_ids
                if int(value) in self.allowed_ids
                and int(value) not in self.published_ids
            }
        )
        if not candidates:
            self._reset_candidate()
            return None

        selected = candidates[0]
        if selected == self.candidate:
            self.consecutive_frames += 1
        else:
            self.candidate = selected
            self.consecutive_frames = 1

        if self.consecutive_frames < self.required_frames:
            return None

        self.published_ids.add(selected)
        self.armed = False
        self._reset_candidate()
        return selected

    def _reset_candidate(self) -> None:
        self.candidate = None
        self.consecutive_frames = 0
