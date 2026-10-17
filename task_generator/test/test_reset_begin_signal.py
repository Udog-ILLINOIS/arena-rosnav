#! /usr/bin/env python3
"""
reset_task() must publish /scenario_reset_begin before the simulator is
paused, carrying the id that the matching /scenario_reset will carry.
The publisher is created latched, next to scenario_reset, and only
outside train mode.

The node module is loaded with its ROS and task_generator imports stubbed,
so no ROS master or built workspace is needed.
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_double_resume import load_node_module  # noqa: E402


class TestResetBeginSignal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = load_node_module()

    def _make_generator(self, number_of_resets):
        calls = mock.MagicMock()
        generator = mock.MagicMock()
        generator._env_wrapper = calls.env
        generator._task = calls.task
        generator._pub_scenario_reset_begin = calls.pub_scenario_reset_begin
        generator._pub_scenario_reset = calls.pub_scenario_reset
        generator._number_of_resets = number_of_resets
        generator._task.reset.return_value = False
        return generator, calls

    def _reset(self, generator):
        with mock.patch("builtins.print"):
            self.node.TaskGenerator.reset_task(generator)

    def test_begin_published_once_before_pause(self):
        generator, calls = self._make_generator(3)
        self._reset(generator)
        names = [c[0] for c in calls.mock_calls]
        self.assertEqual(names.count("pub_scenario_reset_begin.publish"), 1, names)
        self.assertEqual(names[0], "pub_scenario_reset_begin.publish", names)
        self.assertLess(
            names.index("pub_scenario_reset_begin.publish"),
            names.index("env.before_reset_task"),
        )
        self.assertLess(
            names.index("pub_scenario_reset_begin.publish"),
            names.index("pub_scenario_reset.publish"),
        )

    def test_begin_id_matches_scenario_reset_id(self):
        generator, _ = self._make_generator(0)
        for expected in range(3):
            generator._pub_scenario_reset_begin.reset_mock()
            generator._pub_scenario_reset.reset_mock()
            self._reset(generator)
            generator._pub_scenario_reset_begin.publish.assert_called_once_with(
                expected
            )
            generator._pub_scenario_reset.publish.assert_called_once_with(expected)
        self.assertEqual(generator._number_of_resets, 3)

    def _construct(self, train_mode):
        node = self.node
        rospy = node.rospy
        rospy.reset_mock()

        def fake_rosparam_get(cast, name, default=None):
            if name == "train_mode":
                return train_mode
            return default

        with mock.patch.object(
            node, "rosparam_get", side_effect=fake_rosparam_get
        ), mock.patch.object(
            node.TaskGenerator, "_get_predefined_task"
        ), mock.patch.object(
            node.TaskGenerator, "reset_task"
        ):
            node.TaskGenerator()
        return rospy.Publisher.call_args_list

    def test_publisher_latched_outside_train_mode(self):
        publishers = self._construct(train_mode=False)
        begin = [c for c in publishers if c[0][0] == "scenario_reset_begin"]
        self.assertEqual(len(begin), 1, publishers)
        args, kwargs = begin[0]
        self.assertIs(args[1], self.node.std_msgs.Int16)
        self.assertTrue(kwargs.get("latch"))
        reset = [c for c in publishers if c[0][0] == "scenario_reset"]
        self.assertEqual(len(reset), 1, publishers)
        self.assertTrue(reset[0][1].get("latch"))

    def test_no_publishers_in_train_mode(self):
        publishers = self._construct(train_mode=True)
        self.assertEqual(publishers, [])


if __name__ == "__main__":
    unittest.main()
