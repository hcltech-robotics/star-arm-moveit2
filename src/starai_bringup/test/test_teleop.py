import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'teleop', Path(__file__).parent.parent / 'scripts' / 'teleop.py')
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


def test_step_is_limited_per_joint():
    # Far target: each joint moves at most one step, in its own direction.
    assert teleop.step_toward([0.0, 0.0, 1.0], [1.0, -0.005, 1.0], 0.01) == [0.01, -0.005, 1.0]


def test_gripper_maps_and_clamps():
    m = lambda x: teleop.map_gripper(x, 0.0, -0.02, 0.0, -0.03)  # noqa: E731
    assert m(-0.01) == pytest.approx(-0.015)   # half closed
    assert m(0.005) == 0.0                     # past open
    assert m(-0.05) == pytest.approx(-0.03)    # past closed
