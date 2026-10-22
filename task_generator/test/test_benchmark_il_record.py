#! /usr/bin/env python3
"""
A benchmark relaunch keeps IL recording on (record_il.launch) when the
launch it replaces recorded, with one session folder per stage.

The module is loaded with its ROS and task_generator imports stubbed,
so no ROS master or built workspace is needed.
"""

import importlib.util
import os
import sys
import unittest
from unittest import mock

MODULE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "task_generator",
    "tasks",
    "modules",
    "benchmark.py",
)

STUBBED_MODULES = [
    "rospkg",
    "rospy",
    "yaml",
    "task_generator",
    "task_generator.constants",
    "task_generator.shared",
    "task_generator.tasks",
    "task_generator.tasks.modules",
    "task_generator.tasks.task_factory",
    "arena_evaluation_msgs",
    "arena_evaluation_msgs.srv",
]


def load_benchmark_module():
    stubs = {name: mock.MagicMock(name=name) for name in STUBBED_MODULES}
    for name, module in stubs.items():
        parent, _, child = name.rpartition(".")
        if parent:
            setattr(stubs[parent], child, module)

    with mock.patch.dict(sys.modules, stubs), \
            mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:11311"}):
        spec = importlib.util.spec_from_file_location("benchmark_under_test", MODULE_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class TestBenchmarkILRecord(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.benchmark = load_benchmark_module()

    def _args(self, params):
        def get(cast, name, default=None):
            return params.get(name, default)
        with mock.patch.object(self.benchmark, "rosparam_get", side_effect=get):
            return self.benchmark._il_record_args("basic_26-10-21_22-28-06", "gru_learn_scenario_2")

    def test_recording_launch_is_forwarded(self):
        args = self._args({
            "/il_session_recorder/data_root": "/home/abdel/il_data",
            "/il_session_recorder/operator": "ewan",
        })
        self.assertEqual([
            "record_il:=true",
            "il_data_root:=/home/abdel/il_data",
            "il_operator:=ewan",
            "il_session:=basic_26-10-21_22-28-06_gru_learn_scenario_2",
        ], args)

    def test_session_is_a_plain_folder_name(self):
        session = self._args({"/il_session_recorder/data_root": "/d"})[-1].split(":=", 1)[1]
        self.assertNotIn("/", session)

    def test_no_recording_without_recorder(self):
        self.assertEqual([], self._args({}))


if __name__ == "__main__":
    unittest.main()
