#! /usr/bin/env python3
"""
A benchmark relaunch keeps the pedestrian speed (ped_vmax) of the launch it
replaces.
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_benchmark_il_record import load_benchmark_module


class TestBenchmarkPedVmax(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.benchmark = load_benchmark_module()

    def _args(self, params):
        def get(cast, name, default=None):
            return cast(params[name]) if name in params else default
        with mock.patch.object(self.benchmark, "rosparam_get", side_effect=get):
            return self.benchmark._ped_vmax_args()

    def test_speed_is_forwarded(self):
        self.assertEqual(["ped_vmax:=0.8"], self._args({"ped_vmax": 0.8}))

    def test_nothing_without_speed(self):
        self.assertEqual([], self._args({}))


if __name__ == "__main__":
    unittest.main()
