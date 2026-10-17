#!/usr/bin/env python3
"""Checks that start_trials.launch and start_arena.launch start exactly one
RViz node, with the arena.rviz config, for every headless level, both
simulators and both show_rviz values.

Uses `roslaunch --nodes` / `roslaunch --args`, which only resolve the launch
tree and do not need a ROS master.
"""

import os
import shutil
import subprocess
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(PKG_DIR, "launch")
ENTRY_POINTS = ["start_trials.launch", "start_arena.launch"]
SIMULATORS = ["flatland", "gazebo"]
HEADLESS = ["0", "1", "2"]
SHOW_RVIZ = ["true", "false"]


def _env():
    env = dict(os.environ)
    # make nested $(find arena_bringup) resolve to this source tree
    src_root = os.path.dirname(PKG_DIR)
    env["ROS_PACKAGE_PATH"] = src_root + os.pathsep + env.get("ROS_PACKAGE_PATH", "")
    return env


def _roslaunch(*args):
    return subprocess.run(
        ["roslaunch"] + list(args),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        env=_env(),
        timeout=120,
    )


def _available():
    if shutil.which("roslaunch") is None or shutil.which("rospack") is None:
        return False
    try:
        out = subprocess.run(
            ["rospack", "find", "dynamic_gap"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=_env(),
            timeout=60,
        )
    except Exception:
        return False
    return out.returncode == 0


@unittest.skipUnless(_available(), "roslaunch / ROS workspace not available")
class TestSingleRviz(unittest.TestCase):
    def test_rviz_node_set(self):
        for launch in ENTRY_POINTS:
            path = os.path.join(LAUNCH_DIR, launch)
            for sim in SIMULATORS:
                for headless in HEADLESS:
                    for show_rviz in SHOW_RVIZ:
                        launch_args = [
                            "simulator:=" + sim,
                            "headless:=" + headless,
                            "show_rviz:=" + show_rviz,
                        ]
                        with self.subTest(launch=launch, simulator=sim,
                                          headless=headless, show_rviz=show_rviz):
                            res = _roslaunch("--nodes", path, *launch_args)
                            self.assertEqual(res.returncode, 0, res.stdout)
                            nodes = [l.strip() for l in res.stdout.splitlines()
                                     if l.strip().startswith("/")]
                            rviz_nodes = [n for n in nodes if "rviz" in n
                                          and n != "/visualize_robot_model"]
                            self.assertEqual(rviz_nodes, ["/rviz"], res.stdout)
                            cmd = _roslaunch("--args", "/rviz", path, *launch_args)
                            self.assertEqual(cmd.returncode, 0, cmd.stdout)
                            self.assertIn("dynamic_gap/rviz/arena.rviz", cmd.stdout)


if __name__ == "__main__":
    unittest.main()
