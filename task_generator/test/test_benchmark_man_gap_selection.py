#! /usr/bin/env python3
"""
A benchmark relaunch keeps the manual gap selection settings
(man_gap_max_bearing_deg, man_gap_switch_ratio) of the launch it replaces.
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_benchmark_il_record import load_benchmark_module


class TestBenchmarkManGapSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.benchmark = load_benchmark_module()

    def _args(self, params):
        def get(cast, name, default=None):
            return cast(params[name]) if name in params else default
        with mock.patch.object(self.benchmark, "rosparam_get", side_effect=get):
            return self.benchmark._man_gap_selection_args()

    def test_settings_are_forwarded(self):
        self.assertEqual(["man_gap_max_bearing_deg:=45.0", "man_gap_switch_ratio:=0.8"],
                         self._args({"man_gap_max_bearing_deg": 45, "man_gap_switch_ratio": 0.8}))

    def test_nothing_without_settings(self):
        self.assertEqual([], self._args({}))


if __name__ == "__main__":
    unittest.main()
