import importlib.util
import math
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'go_to_pose', Path(__file__).parent.parent / 'scripts' / 'go_to_pose.py')
go_to_pose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(go_to_pose)

LIMIT = math.radians(15)
JOINTS = ['joint1', 'joint2']
REST = [0.0, -1.5743]


def test_near_pose_moves():
    state = {'joint1': math.radians(10), 'joint2': -1.5743 + math.radians(-14)}
    assert go_to_pose.too_far(JOINTS, REST, state, LIMIT) == []


def test_far_pose_is_refused():
    state = {'joint1': 0.0, 'joint2': 0.0}  # stretched out: 90 deg from rest
    assert [j for j, _ in go_to_pose.too_far(JOINTS, REST, state, LIMIT)] == ['joint2']
