#! /usr/bin/env python3
"""
reset_task() must pause the simulator once and resume it exactly once,
right after the task reset and before /scenario_reset is published.

The node module is loaded with its ROS and task_generator imports stubbed,
so no ROS master or built workspace is needed.
"""

import importlib.util
import os
import sys
import unittest
from unittest import mock

NODE_PATH = os.environ.get(
    "TASK_GENERATOR_NODE_PATH",
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "task_generator",
        "task_generator_node.py",
    ),
)

STUBBED_MODULES = [
    "rospkg",
    "rospy",
    "yaml",
    "task_generator",
    "task_generator.constants",
    "task_generator.manager",
    "task_generator.manager.entity_manager",
    "task_generator.manager.entity_manager.entity_manager",
    "task_generator.manager.entity_manager.flatland_manager",
    "task_generator.manager.entity_manager.pedsim_manager",
    "task_generator.manager.world_manager",
    "task_generator.manager.obstacle_manager",
    "task_generator.manager.robot_manager",
    "task_generator.manager.utils",
    "task_generator.shared",
    "task_generator.simulators",
    "task_generator.simulators.base_simulator",
    "task_generator.simulators.flatland_simulator",
    "task_generator.simulators.gazebo_simulator",
    "task_generator.simulators.simulator_factory",
    "task_generator.tasks",
    "task_generator.tasks.task_factory",
    "task_generator.utils",
    "map_distance_server",
    "map_distance_server.srv",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
]


def load_node_module():
    stubs = {name: mock.MagicMock(name=name) for name in STUBBED_MODULES}
    for name, module in stubs.items():
        parent, _, child = name.rpartition(".")
        if parent:
            setattr(stubs[parent], child, module)
    # as in a running node; reset_task stops early once rospy is shut down
    stubs["rospy"].is_shutdown.return_value = False

    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(
            "task_generator_node_under_test", NODE_PATH
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class TestDoubleResume(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = load_node_module()

    def _run_reset(self):
        calls = mock.MagicMock()
        generator = mock.MagicMock()
        generator._env_wrapper = calls.env
        generator._task = calls.task
        generator._pub_scenario_reset = calls.pub_scenario_reset
        generator._number_of_resets = 3
        generator._task.reset.return_value = False

        with mock.patch("builtins.print"):
            self.node.TaskGenerator.reset_task(generator)

        names = [c[0] for c in calls.mock_calls]
        return generator, names

    def test_resume_called_once(self):
        _, names = self._run_reset()
        self.assertEqual(names.count("env.after_reset_task"), 1, names)
        self.assertEqual(names.count("env.before_reset_task"), 1, names)

    def test_resume_order(self):
        _, names = self._run_reset()
        self.assertLess(
            names.index("env.before_reset_task"), names.index("task.reset")
        )
        self.assertLess(
            names.index("task.reset"), names.index("env.after_reset_task")
        )
        self.assertLess(
            names.index("env.after_reset_task"),
            names.index("pub_scenario_reset.publish"),
        )

    def test_scenario_reset_unchanged(self):
        generator, names = self._run_reset()
        self.assertEqual(names.count("pub_scenario_reset.publish"), 1, names)
        generator._pub_scenario_reset.publish.assert_called_once_with(3)
        self.assertEqual(generator._number_of_resets, 4)

    def test_no_new_episode_when_the_run_ended_in_the_reset(self):
        # the benchmark ends or restarts the run in before_reset (IL fix 51)
        rospy = self.node.rospy
        rospy.is_shutdown.return_value = True
        try:
            generator, names = self._run_reset()
        finally:
            rospy.is_shutdown.return_value = False
        self.assertNotIn("pub_scenario_reset.publish", names)
        self.assertNotIn("env.after_reset_task", names)
        self.assertEqual(generator._number_of_resets, 3)


if __name__ == "__main__":
    unittest.main()
