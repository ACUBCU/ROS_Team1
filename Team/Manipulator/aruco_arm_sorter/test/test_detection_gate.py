from aruco_arm_sorter.detection_gate import StableMarkerGate


def test_requires_consecutive_frames_before_publish():
    gate = StableMarkerGate([0, 1], required_frames=3)
    gate.set_ready(True)
    assert gate.observe([0]) is None
    assert gate.observe([0]) is None
    assert gate.observe([0]) == 0
    assert gate.observe([0]) is None


def test_arm_status_gates_detection_and_rearms_on_ready_transition():
    gate = StableMarkerGate([0, 1], required_frames=2)
    assert gate.observe([0]) is None
    gate.set_ready(True)
    assert gate.observe([0]) is None
    gate.set_ready(False)
    assert gate.observe([0]) is None
    gate.set_ready(True)
    assert gate.observe([1]) is None
    assert gate.observe([1]) == 1


def test_already_published_id_does_not_block_next_marker():
    gate = StableMarkerGate([0, 1], required_frames=1)
    gate.set_ready(True)
    assert gate.observe([0, 1]) == 0
    gate.set_ready(False)
    gate.set_ready(True)
    assert gate.observe([0, 1]) == 1


def test_unsupported_id_resets_candidate():
    gate = StableMarkerGate([0, 1], required_frames=2)
    gate.set_ready(True)
    assert gate.observe([0]) is None
    assert gate.observe([42]) is None
    assert gate.observe([0]) is None
