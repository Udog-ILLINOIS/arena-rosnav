#! /usr/bin/env python3
"""
reset_task() runs one reset at a time: the status timer and the reset_task
service run on different threads. A timer tick during a running reset is
skipped instead of starting a second reset with the same begin id.
"""

import os
import sys
import threading
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_double_resume import load_node_module  # noqa: E402


class TestResetLock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = load_node_module()

    def _generator(self):
        generator = mock.MagicMock()
        generator._reset_lock = threading.RLock()
        generator._number_of_resets = 0
        generator._task.reset.return_value = False
        generator.reset_task = lambda **kw: self.node.TaskGenerator.reset_task(generator, **kw)
        return generator

    @staticmethod
    def held_elsewhere(lock):
        # RLock has no locked(); try from another thread
        result = []
        t = threading.Thread(target=lambda: result.append(not lock.acquire(blocking=False) or lock.release()))
        t.start()
        t.join()
        return result[0]

    def test_reset_holds_the_lock(self):
        generator = self._generator()
        held = []
        generator._env_wrapper.before_reset_task.side_effect = lambda: held.append(self.held_elsewhere(generator._reset_lock))
        with mock.patch("builtins.print"):
            self.node.TaskGenerator.reset_task(generator)
        self.assertEqual(held, [True])
        self.assertFalse(self.held_elsewhere(generator._reset_lock))

    def test_status_tick_during_reset_is_skipped(self):
        generator = self._generator()
        generator._task.is_done = True
        running = threading.Event()
        done = threading.Event()

        def reset_running():  # another thread holds the lock
            with generator._reset_lock:
                running.set()
                done.wait(2)
        holder = threading.Thread(target=reset_running)
        holder.start()
        running.wait(2)
        with mock.patch("builtins.print"):
            self.node.TaskGenerator._check_task_status(generator)
        generator._pub_scenario_reset_begin.publish.assert_not_called()
        done.set()
        holder.join(2)

        with mock.patch("builtins.print"):
            self.node.TaskGenerator._check_task_status(generator)
        generator._pub_scenario_reset_begin.publish.assert_called_once_with(0)
        self.assertFalse(self.held_elsewhere(generator._reset_lock))

    def test_concurrent_resets_get_distinct_ids(self):
        generator = self._generator()
        started = threading.Event()
        release = threading.Event()

        def slow_pause():
            started.set()
            release.wait(2)
        generator._env_wrapper.before_reset_task.side_effect = slow_pause

        with mock.patch("builtins.print"):
            first = threading.Thread(target=self.node.TaskGenerator.reset_task, args=(generator,))
            first.start()
            started.wait(2)
            generator._env_wrapper.before_reset_task.side_effect = None
            second = threading.Thread(target=self.node.TaskGenerator.reset_task, args=(generator,))
            second.start()
            release.set()
            first.join(2)
            second.join(2)

        ids = [c.args[0] for c in generator._pub_scenario_reset_begin.publish.call_args_list]
        self.assertEqual(ids, [0, 1])


if __name__ == "__main__":
    unittest.main()
