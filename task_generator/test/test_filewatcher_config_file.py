#! /usr/bin/env python3
"""
The filewatcher reads the task config file that the benchmark module writes
(per ROS master port), and the shared task_generator.yaml otherwise.

The script is loaded with its ROS imports stubbed, so no ROS master or built
workspace is needed.
"""

import importlib.util
import os
import sys
import unittest
from unittest import mock

SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "filewatcher.py"
)

STUBBED_MODULES = [
    "rospkg",
    "rospy",
    "yaml",
    "watchdog",
    "watchdog.observers",
    "watchdog.events",
    "dynamic_reconfigure",
    "dynamic_reconfigure.client",
    "task_generator",
    "task_generator.cfg",
]


def load_filewatcher():
    stubs = {name: mock.MagicMock(name=name) for name in STUBBED_MODULES}
    for name, module in stubs.items():
        parent, _, child = name.rpartition(".")
        if parent:
            setattr(stubs[parent], child, module)

    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location("filewatcher_under_test", SCRIPT_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class TestFilewatcherConfigFile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fw = load_filewatcher()

    def test_benchmark_reads_the_file_of_its_master(self):
        self.assertEqual("task_generator_11311.yaml",
                         self.fw.task_config_name("benchmark", "http://localhost:11311/"))
        self.assertEqual("task_generator_11312.yaml",
                         self.fw.task_config_name("rviz_ui,benchmark", "http://localhost:11312"))

    def test_same_name_as_benchmark_writes(self):
        # benchmark.py: f"task_generator_{port}.yaml" with port from ROS_MASTER_URI
        uri = "http://sunstreaker:11391/"
        port = uri.split(":")[-1].rstrip("/")
        self.assertEqual(f"task_generator_{port}.yaml", self.fw.task_config_name("benchmark", uri))

    def test_other_launches_read_the_shared_file(self):
        for modules in ("", "staged", "dynamic_map,rviz_ui"):
            self.assertEqual("task_generator.yaml",
                             self.fw.task_config_name(modules, "http://localhost:11311"))


if __name__ == "__main__":
    unittest.main()
