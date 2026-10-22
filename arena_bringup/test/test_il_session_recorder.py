#!/usr/bin/env python3
"""Issue 07: il_session_recorder topic list, rosbag options, session naming and data_root checks.
No ROS master needed (rospy and rospkg are stubbed)."""

import datetime
import importlib.machinery
import importlib.util
import os
import subprocess
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "il_session_recorder")
LAUNCH = os.path.join(HERE, "..", "launch", "record_il.launch")


def load_script():
    stubs = {
        "rospy": types.ModuleType("rospy"),
        "rospkg": types.ModuleType("rospkg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    stubs["std_msgs.msg"].Int16 = object
    stubs["rospkg"].ResourceNotFound = KeyError
    with mock.patch.dict(sys.modules, stubs):
        loader = importlib.machinery.SourceFileLoader("il_session_recorder", SCRIPT)
        spec = importlib.util.spec_from_loader(loader.name, loader)
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
    return module


class SessionRecorderTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.m = load_script()

    def test_topics_are_namespaced(self):
        topics = self.m.il_topics("rto")
        for topic in ["/rto/manual_gap_sample", "/rto/gap_selection_cursor", "/rto/traj_creation_point",
                      "/rto/manual_traj_selection", "/rto/scan", "/rto/odom", "/rto/acc", "/rto/cmd_vel",
                      "/rto/move_base_simple/goal", "/rto/move_base_flex/NavfnROS/plan", "/rto/curr_exec_dg_traj",
                      "/tf", "/tf_static", "/map", "/scenario_reset", "/scenario_reset_begin",
                      "/pedsim_simulator/pedsim_agents_data"]:
            self.assertIn(topic, topics)
        self.assertEqual(self.m.il_topics("/rto/"), topics)
        self.assertEqual(len(topics), len(set(topics)))

    def test_no_marker_topics(self):
        self.assertFalse([t for t in self.m.il_topics("rto") if "marker" in t])

    def test_rosbag_options(self):
        cmd = self.m.rosbag_command(["/a", "/b"], 1024)
        self.assertEqual(cmd[:2], ["rosbag", "record"])
        for option in ["--lz4", "--split", "--size=1024", "--repeat-latched"]:
            self.assertIn(option, cmd)
        self.assertEqual(cmd[cmd.index("-O") + 1], "session.bag")
        self.assertIn("/a", cmd)
        self.assertIn("/b", cmd)

    def test_rosbag_supports_the_options(self):
        try:
            help_text = subprocess.run(["rosbag", "record", "--help"], stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, universal_newlines=True).stdout
        except FileNotFoundError:
            self.skipTest("rosbag not on PATH")
        for option in ["--lz4", "--split", "--size", "--repeat-latched"]:
            self.assertIn(option, help_text)

    def test_session_name(self):
        self.assertEqual(self.m.session_name("pilot01"), "pilot01")
        self.assertEqual(self.m.session_name("", datetime.datetime(2026, 10, 5, 9, 3, 7)), "2026-10-05_09-03-07")

    def test_data_root_checks(self):
        with tempfile.TemporaryDirectory() as repo:
            self.assertIn("not set", self.m.check_data_root("", [repo]))
            self.assertIn("absolute", self.m.check_data_root("data", [repo]))
            self.assertIn("inside the git repo", self.m.check_data_root(os.path.join(repo, "data"), [repo]))
            self.assertIn("inside the git repo", self.m.check_data_root(repo, [repo]))
            self.assertIsNone(self.m.check_data_root(repo + "_data", [repo]))


class LaunchFileTest(unittest.TestCase):

    def test_data_root_has_no_default(self):
        args = {a.get("name"): a for a in ET.parse(LAUNCH).getroot().iter("arg")}
        self.assertIn("data_root", args)
        self.assertIsNone(args["data_root"].get("default"))
        self.assertIsNone(args["data_root"].get("value"))

    def test_runs_the_recorder(self):
        nodes = [n for n in ET.parse(LAUNCH).getroot().iter("node")]
        self.assertEqual([(n.get("pkg"), n.get("type")) for n in nodes], [("arena_bringup", "il_session_recorder")])

    def test_script_is_executable(self):
        self.assertTrue(os.access(SCRIPT, os.X_OK))


if __name__ == "__main__":
    unittest.main()
