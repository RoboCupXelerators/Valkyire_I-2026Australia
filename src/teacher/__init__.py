"""Teacher debugging utilities for MLP data collection.

The teacher package keeps rule-policy decisions observable and replayable.
It is intentionally independent from the low-level executor so the same
decision stream can later train or validate an MLP policy.
"""

from .rule_teacher import RuleTeacher, TeacherConfig, TeacherDecision
from .scenarios import TeacherScenario, default_teacher_scenarios, load_teacher_scenarios
from .trace import TeacherTraceRecorder
from .badcase import TeacherBadcaseDetector

__all__ = [
    "RuleTeacher",
    "TeacherConfig",
    "TeacherDecision",
    "TeacherScenario",
    "default_teacher_scenarios",
    "load_teacher_scenarios",
    "TeacherTraceRecorder",
    "TeacherBadcaseDetector",
]
