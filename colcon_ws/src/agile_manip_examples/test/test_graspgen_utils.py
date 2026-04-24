"""Tests for small helpers in graspgen_utils.

Only ``load_grasp_scores`` is covered here -- the rest of the module
talks to ROS publishers / TF and is out of scope for a pure-Python
unit test. Heavy ROS imports are stubbed the same way they are in
test_planning_helpers so this file runs on a plain host.
"""

import sys
import textwrap
import types
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Stub ROS dependencies so graspgen_utils can import on the host.
# ---------------------------------------------------------------------------
def _ensure_module(name):
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    return module


geom = _ensure_module('geometry_msgs')
geom_msg = _ensure_module('geometry_msgs.msg')
geom_msg.Pose = type('Pose', (), {})
geom_msg.TransformStamped = type('TransformStamped', (), {})
geom.msg = geom_msg

vis = _ensure_module('visualization_msgs')
vis_msg = _ensure_module('visualization_msgs.msg')
vis_msg.MarkerArray = type('MarkerArray', (), {})
vis.msg = vis_msg

rclpy = _ensure_module('rclpy')
rclpy_duration = _ensure_module('rclpy.duration')
rclpy_duration.Duration = lambda *_, **__: None
rclpy_qos = _ensure_module('rclpy.qos')
for name in ('DurabilityPolicy', 'HistoryPolicy', 'QoSProfile', 'ReliabilityPolicy'):
    setattr(rclpy_qos, name, type(name, (), {}))
rclpy_time = _ensure_module('rclpy.time')
rclpy_time.Time = lambda *_, **__: None

tf2_ros = _ensure_module('tf2_ros')
tf2_ros.TransformException = Exception


SRC_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC_ROOT))
pkg = _ensure_module('agile_manip_examples')
pkg.__path__ = [str(SRC_ROOT / 'agile_manip_examples')]

from agile_manip_examples import graspgen_utils  # noqa: E402


# ---------------------------------------------------------------------------
# load_grasp_scores
# ---------------------------------------------------------------------------

VALID_YAML = textwrap.dedent("""
    format: isaac_grasp
    format_version: 1.0
    grasps:
      grasp_0:
        confidence: 0.95
      grasp_1:
        confidence: 0.42
      grasp_2:
        position: [0.0, 0.0, 0.0]
""").lstrip()


def test_load_grasp_scores_happy_path(tmp_path):
    p = tmp_path / 'scores.yaml'
    p.write_text(VALID_YAML)
    scores = graspgen_utils.load_grasp_scores(str(p))
    assert scores == {'grasp_0': 0.95, 'grasp_1': 0.42}


def test_load_grasp_scores_empty_path_returns_empty():
    assert graspgen_utils.load_grasp_scores('') == {}


def test_load_grasp_scores_missing_file_returns_empty(tmp_path):
    assert graspgen_utils.load_grasp_scores(str(tmp_path / 'nope.yaml')) == {}


def test_load_grasp_scores_regex_fallback_on_broken_yaml(tmp_path):
    # Upstream ``save_to_isaac_grasp_format`` occasionally emits list
    # entries whose indentation PyYAML cannot parse. Simulate that by
    # writing a file that is structurally valid for the regex but
    # rejected by yaml.safe_load (tab/space indentation mixed).
    broken = (
        'format: isaac_grasp\n'
        'grasps:\n'
        '  grasp_0:\n'
        '    confidence: 0.91\n'
        '\tbroken: [-0.1, -0.2]\n'        # mixing tab + space breaks YAML
        '  grasp_1:\n'
        '    confidence: 0.55\n'
    )
    p = tmp_path / 'broken.yaml'
    p.write_text(broken)
    scores = graspgen_utils.load_grasp_scores(str(p))
    assert scores == {'grasp_0': 0.91, 'grasp_1': 0.55}


def test_load_grasp_scores_fallback_handles_scientific_notation(tmp_path):
    # The fallback regex must accept scientific-notation floats because
    # GraspGen occasionally emits them for very low-confidence grasps.
    broken = (
        'grasps:\n'
        '  grasp_0:\n'
        '    confidence: 1.23e-04\n'
        '\tjunk: [-inf]\n'
        '  grasp_1:\n'
        '    confidence: -5.5E+1\n'
    )
    p = tmp_path / 'scinot.yaml'
    p.write_text(broken)
    scores = graspgen_utils.load_grasp_scores(str(p))
    assert scores == pytest.approx({'grasp_0': 1.23e-4, 'grasp_1': -55.0})


def test_load_grasp_scores_skips_entries_without_confidence(tmp_path):
    p = tmp_path / 'partial.yaml'
    p.write_text(
        'grasps:\n'
        '  grasp_0:\n'
        '    confidence: 0.7\n'
        '  grasp_1:\n'
        '    position: [0,0,0]\n'
    )
    scores = graspgen_utils.load_grasp_scores(str(p))
    assert scores == {'grasp_0': 0.7}
