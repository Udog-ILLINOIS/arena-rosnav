#! /usr/bin/env python3
"""
A benchmark stage's scenario is used from the stage's first episode on (IL fix 51).

Stage changes on the same map are not relaunched: the benchmark writes the next
stage's task config and the filewatcher applies it. That used to happen after the
reset that starts the new stage, so every stage ran one episode late (the first
stage twice, the last never). The modules are loaded with ROS stubbed, so no
master or built workspace is needed. The benchmark sets the stage's config on the
server itself, and the task factory gives it to the task modes at each reset (their
reconfigure callbacks are asynchronous).
"""

import importlib.util
import os
import sys
import types
import unittest
from unittest import mock

TASK_GENERATOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "task_generator")

STUBBED_MODULES = [
    "rospkg",
    "rospy",
    "yaml",
    "std_msgs",
    "std_msgs.msg",
    "rosgraph_msgs",
    "rosgraph_msgs.msg",
    "training",
    "training.srv",
    "task_generator.manager",
    "task_generator.manager.obstacle_manager",
    "task_generator.manager.robot_manager",
    "task_generator.manager.world_manager",
    "task_generator.utils",
    "task_generator.cfg",
    "task_generator.task_config",
    # registered by task_factory's relative imports at its end
    "task_generator.tasks.obstacles.random",
    "task_generator.tasks.obstacles.scenario",
    "task_generator.tasks.obstacles.parametrized",
    "task_generator.tasks.robots.random",
    "task_generator.tasks.robots.guided",
    "task_generator.tasks.robots.explore",
    "task_generator.tasks.robots.scenario",
    "task_generator.tasks.modules.clear_forbidden_zones",
    "task_generator.tasks.modules.dynamic_map",
    "task_generator.tasks.modules.rviz_ui",
    "task_generator.tasks.modules.staged",
    "task_generator.tasks.modules.benchmark",
    "dynamic_reconfigure",
    "dynamic_reconfigure.client",
    "task_generator",
    "task_generator.constants",
    "task_generator.shared",
    "task_generator.tasks",
    "task_generator.tasks.modules",
    "task_generator.tasks.obstacles",
    "task_generator.tasks.robots",
    "task_generator.tasks.task_factory",
    "arena_evaluation_msgs",
    "arena_evaluation_msgs.srv",
]


class _Namespace(str):
    def __call__(self, *args):
        return _Namespace(os.path.join(self, *args))


def _base(name):
    return type(name, (), {"NODE_CONFIGURATION": _Namespace("/task_generator_server"),
                           "__init__": lambda self, **kwargs: None,
                           "reset": lambda self, **kwargs: None})


def load(*path):
    """Load a task_generator module with its imports stubbed and real base classes."""
    stubs = {name: mock.MagicMock(name=name) for name in STUBBED_MODULES}
    for name, module in stubs.items():
        parent, _, child = name.rpartition(".")
        if parent:
            setattr(stubs[parent], child, module)
    identity = lambda *_: (lambda cls: cls)
    factory = stubs["task_generator.tasks.task_factory"].TaskFactory
    factory.register_module = factory.register_obstacles = factory.register_robots = identity
    stubs["task_generator.tasks.modules"].TM_Module = _base("TM_Module")
    stubs["task_generator.tasks.obstacles"].TM_Obstacles = _base("TM_Obstacles")
    stubs["task_generator.tasks.robots"].TM_Robots = _base("TM_Robots")
    stubs["task_generator.shared"].Namespace = _Namespace
    reconfigurable = _base("Reconfigurable")
    stubs["task_generator.tasks"].Reconfigurable = reconfigurable
    stubs["task_generator.tasks"].Task = type("Task", (), {})

    with mock.patch.dict(sys.modules, stubs), \
            mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:11311"}):
        # a package-qualified name, so relative imports resolve to the stubs
        name = ".".join(["task_generator", *path])[:-len(".py")]
        spec = importlib.util.spec_from_file_location(name, os.path.join(TASK_GENERATOR, *path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class TestStageTiming(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.benchmark = load("tasks", "modules", "benchmark.py")

    def _run(self, episodes):
        """Scenario file active at each reset of a run of stages with these episode counts."""
        b = self.benchmark
        stages = [
            b.Suite.Stage(name=f"s{i}", episodes=n, robot="rto", map="gru_learn",
                          tm_robots="scenario", tm_obstacles="scenario",
                          config={"SCENARIO": {"file": f"{i}.json"}}, seed=1, timeout=90)
            for i, n in enumerate(episodes)
        ]
        bench = object.__new__(b.Mod_Benchmark)
        bench._config = types.SimpleNamespace(suite=types.SimpleNamespace(scale_episodes=1))
        bench._suite = b.Suite(name="suite", stages=stages)
        bench._contest = mock.MagicMock(max_index=0)
        bench._contest_index, bench._suite_index = b.Contest.Index(0), b.Suite.Index(0)
        bench._requires_restart = False
        bench._runid = "run"

        active = {"file": stages[0].config["SCENARIO"]["file"], "done": False}

        def reincarnate(self):
            # what the non-restart branch leaves behind: the stage's file applied, episode 0
            active["file"] = self._suite.config(self._suite_index).config["SCENARIO"]["file"]
            self._episode = 0

        def complete():
            active["done"] = True

        get = lambda cast, name, default=None: default
        with mock.patch.object(b, "rosparam_get", side_effect=get), \
                mock.patch.object(b.Mod_Benchmark, "_reincarnate", reincarnate), \
                mock.patch.object(b.Mod_Benchmark, "_logger", mock.MagicMock()), \
                mock.patch.object(b.Mod_Benchmark, "_log_episode", lambda self: None), \
                mock.patch.object(b.Mod_Benchmark, "_log_suite", lambda self: None), \
                mock.patch.object(b.Mod_Benchmark, "_log_contest", lambda self: None), \
                mock.patch.object(b.Mod_Benchmark, "_taskgen_restore", lambda *a, **k: None), \
                mock.patch.object(b.Mod_Benchmark, "_suicide", lambda self: complete()), \
                mock.patch.object(b.Mod_Benchmark, "_start_stage", lambda self, config: None), \
                mock.patch("builtins.open", mock.mock_open()), \
                mock.patch.object(b.os, "remove"):
            bench._episode = -1  # as after a (re)launch
            used = []
            for _ in range(sum(episodes) + 3):
                bench.before_reset()
                if active["done"]:
                    break
                used.append(active["file"])  # the task modes read the config here
                bench.after_reset()
        return used, active["done"]

    def test_each_stage_runs_its_own_scenario(self):
        used, done = self._run([1, 1, 1, 1])
        self.assertEqual(["0.json", "1.json", "2.json", "3.json"], used)
        self.assertTrue(done)

    def test_episode_counts_per_stage(self):
        used, done = self._run([2, 1, 3])
        self.assertEqual(["0.json", "0.json", "1.json", "2.json", "2.json", "2.json"], used)
        self.assertTrue(done)


class TestApplyTaskConfig(unittest.TestCase):
    """The benchmark sets the stage's whole config on the server itself, synchronously."""

    @classmethod
    def setUpClass(cls):
        cls.benchmark = load("tasks", "modules", "benchmark.py")

    def setUp(self):
        self.benchmark.rospy.reset_mock()
        self.client = mock.MagicMock()
        self.benchmark.dynamic_reconfigure.client.Client.return_value = self.client
        self.bench = object.__new__(self.benchmark.Mod_Benchmark)

    def test_whole_config_is_applied(self):
        written = {"SCENARIO": {"file": "CS2.json"}, "RANDOM": {"seed": 7, "static": {"min": 1}}}
        with mock.patch.object(self.benchmark, "task_config_params",
                               side_effect=lambda content, description: {"from": content}) as params:
            self.bench._apply_task_config(written)
        params.assert_called_once()
        self.assertIs(written, params.call_args[0][0])  # base values included, not only the stage's
        self.client.update_configuration.assert_called_once_with({"from": written})
        self.benchmark.rospy.logerr.assert_not_called()

    def test_failure_is_reported(self):
        self.client.update_configuration.side_effect = RuntimeError("server gone")
        with mock.patch.object(self.benchmark, "task_config_params", return_value={}):
            self.bench._apply_task_config({})
        self.benchmark.rospy.logerr.assert_called_once()

    def test_task_mode_change_requires_restart(self):
        b = self.benchmark
        stage = lambda name, tm: b.Suite.Stage(name=name, episodes=1, robot="rto", map="gru_learn",
                                               tm_robots="scenario", tm_obstacles=tm,
                                               config={}, seed=1, timeout=90)
        for tm, restart in (("scenario", False), ("random", True)):
            bench = object.__new__(b.Mod_Benchmark)
            bench._suite = b.Suite(name="suite", stages=[stage("a", "scenario"), stage("b", tm)])
            bench._suite_index = b.Suite.Index(0)
            bench._requires_restart = False
            with mock.patch.object(b.Mod_Benchmark, "_reincarnate", lambda self: None), \
                    mock.patch.object(b.Mod_Benchmark, "_logger", mock.MagicMock()), \
                    mock.patch.object(b.Mod_Benchmark, "_log_suite", lambda self: None):
                bench.suite_index = 1
            self.assertEqual(restart, bench._requires_restart, tm)


class TestTaskConfigParams(unittest.TestCase):
    """The shared config-to-parameter mapping (filewatcher and benchmark)."""

    @classmethod
    def setUpClass(cls):
        cls.task_config = load("task_config.py")

    def test_mapping(self):
        description = {
            "parameters": [{"name": "timeout"}, {"name": "episodes"}],
            "groups": [{"parameters": [{"name": "SCENARIO_file"}, {"name": "RANDOM_static_models"},
                                       {"name": "RANDOM_dynamic_models"}, {"name": "RANDOM_seed"},
                                       {"name": "PARAMETRIZED_file"}]}],
        }
        content = {"timeout": 90, "SCENARIO": {"file": "CS2.json"},
                   "RANDOM": {"seed": 3, "static": {"models": ["shelf", "table"]}, "dynamic": {"models": [1, 2]}}}
        self.assertEqual(
            {"timeout": 90, "SCENARIO_file": "CS2.json", "RANDOM_static_models": "shelf;table",
             "RANDOM_dynamic_models": "1;2", "RANDOM_seed": 3},
            self.task_config.task_config_params(content, description))

    def test_dicts_as_json(self):
        self.assertEqual('{"a": 1}', self.task_config.encode({"a": 1}))


class TestTaskConfigWrittenOnce(unittest.TestCase):
    """The stage's config is overlaid on the backup and written in one go."""

    def test_overlay_on_base(self):
        import tempfile
        import yaml as real_yaml
        benchmark = load("tasks", "modules", "benchmark.py")
        benchmark.yaml = real_yaml
        with tempfile.TemporaryDirectory() as tmp:
            base, current = os.path.join(tmp, "base.yaml"), os.path.join(tmp, "current.yaml")
            with open(base, "w") as f:
                real_yaml.dump({"SCENARIO": {"file": "default.json"}, "RANDOM": {"seed": 0, "static": {"min": 1}}}, f)
            with open(current, "w") as f:
                real_yaml.dump({"SCENARIO": {"file": "CS1.json"}, "leftover": True}, f)
            writes = []
            real_open = open

            def tracking_open(path, mode="r", *a, **k):
                if "w" in mode:
                    writes.append(path)
                return real_open(path, mode, *a, **k)

            with mock.patch.object(benchmark.Mod_Benchmark, "TASK_GENERATOR_CONFIG", current), \
                    mock.patch("builtins.open", tracking_open):
                benchmark.Mod_Benchmark._taskgen_write(
                    {"RANDOM": {"seed": 7}}, {"SCENARIO": {"file": "CS2.json"}}, base=base)
            with real_open(current) as f:
                written = real_yaml.safe_load(f)
        self.assertEqual([current], writes)
        self.assertEqual({"SCENARIO": {"file": "CS2.json"}, "RANDOM": {"seed": 7, "static": {"min": 1}}}, written)


class TestSyncTaskConfigAtReset(unittest.TestCase):
    """The task modes get the server's config at reset, before they place anything."""

    @classmethod
    def setUpClass(cls):
        cls.factory_module = load("tasks", "task_factory.py")

    def setUp(self):
        m = self.factory_module
        m.rospy.reset_mock()
        m.rospy.is_shutdown.return_value = False
        self.task = object.__new__(m.TaskFactory.combine([]))
        self.robots, self.obstacles = mock.MagicMock(), mock.MagicMock()
        self.task._CombinedTask__tm_robots = self.robots
        self.task._CombinedTask__tm_obstacles = self.obstacles
        self.task._CombinedTask__synced_config = None
        self.server = {"SCENARIO_file": "CS1.json"}
        m.rospy.get_param.side_effect = lambda name, default=None: dict(self.server)

    def test_applies_new_config_once(self):
        self.task._sync_task_config()
        self.task._sync_task_config()
        self.server["SCENARIO_file"] = "CS2.json"
        self.task._sync_task_config()
        self.assertEqual([mock.call({"SCENARIO_file": "CS1.json"}), mock.call({"SCENARIO_file": "CS2.json"})],
                         self.obstacles.reconfigure.call_args_list)
        self.assertEqual(2, self.robots.reconfigure.call_count)

    def test_a_failing_task_mode_keeps_its_config(self):
        self.robots.reconfigure.side_effect = FileNotFoundError("no such scenario")
        self.task._sync_task_config()
        self.obstacles.reconfigure.assert_called_once()
        self.factory_module.rospy.logerr.assert_called_once()

    def _reset(self, module):
        t = self.task
        t._train_mode = True
        t._CombinedTask__modules = [module]
        t._CombinedTask__reset_start = mock.MagicMock()
        t._CombinedTask__reset_end = mock.MagicMock()
        t.obstacle_manager = mock.MagicMock()
        t.clock = mock.MagicMock()
        self.obstacles.reset.return_value = ([], [])
        t._reset_task()
        return t

    def test_reset_syncs_after_the_modules(self):
        order = []
        module = mock.MagicMock()
        module.before_reset.side_effect = lambda: order.append("before_reset")
        self.obstacles.reconfigure.side_effect = lambda c: order.append("reconfigure")
        self.obstacles.reset.side_effect = lambda **k: (order.append("reset"), ([], []))[1]
        self._reset(module)
        self.assertEqual(["before_reset", "reconfigure", "reset"], order)

    def test_no_placement_after_a_module_ended_the_run(self):
        module, later = mock.MagicMock(), mock.MagicMock()
        module.before_reset.side_effect = lambda: setattr(self.factory_module.rospy.is_shutdown, "return_value", True)
        t = self.task
        t._train_mode = True
        t._CombinedTask__modules = [module, later]
        t._CombinedTask__reset_start = mock.MagicMock()
        t._CombinedTask__reset_end = mock.MagicMock()
        t.obstacle_manager = mock.MagicMock()
        t._reset_task()
        later.before_reset.assert_not_called()
        self.robots.reset.assert_not_called()
        self.obstacles.reset.assert_not_called()
        module.after_reset.assert_not_called()
        # the benchmark stopped the master: nothing more is sent to it
        t._CombinedTask__reset_end.publish.assert_not_called()

    def test_config_is_synced_before_the_task_modes(self):
        order = []
        constants = self.factory_module.constants
        constants.reconfigure.side_effect = lambda c: order.append(("Config", c))
        self.obstacles.reconfigure.side_effect = lambda c: order.append(("obstacles", c))
        try:
            self.task._sync_task_config()
        finally:
            constants.reconfigure.side_effect = None
        self.assertEqual([("Config", self.server), ("obstacles", self.server)], order)

    def test_no_parameter_set_after_the_run_ended(self):
        m = self.factory_module
        t = self.task
        t._train_mode = False
        t.PARAM_RESETTING = "resetting"
        t._CombinedTask__reset_mutex = False
        t._reset_task = lambda: setattr(m.rospy.is_shutdown, "return_value", True)
        t._mutex_reset_task()
        self.assertEqual([mock.call(t.PARAM_RESETTING, True)], m.rospy.set_param.call_args_list)
        self.assertFalse(t._CombinedTask__reset_mutex)


class TestStageStart(unittest.TestCase):
    """What a stage start writes and applies: its timeout (so stages on the same map get
    it) and its seed, from which the RNG restarts."""

    @classmethod
    def setUpClass(cls):
        cls.benchmark = load("tasks", "modules", "benchmark.py")

    def _bench(self, timeout, config=None):
        b = self.benchmark
        stage = b.Suite.Stage(name="s", episodes=2, robot="rto", map="gru_learn", tm_robots="scenario",
                              tm_obstacles="scenario", config=config or {"SCENARIO": {"file": "CS5.json"}},
                              seed=1, timeout=timeout)
        bench = object.__new__(b.Mod_Benchmark)
        bench._config = types.SimpleNamespace(suite=types.SimpleNamespace(scale_episodes=1),
                                              general=types.SimpleNamespace(simulator="flatland"))
        bench._suite = b.Suite(name="suite", stages=[stage])
        bench._contest = mock.MagicMock()
        bench._contest_index, bench._suite_index = b.Contest.Index(0), b.Suite.Index(0)
        bench._requires_restart = False
        bench._runid, bench._headless, bench._run_timeout = "run", 1, 300.0
        return bench

    def _reincarnate(self, timeout, restart=False, written=None, applied=True):
        b = self.benchmark
        bench = self._bench(timeout)
        bench._requires_restart = restart
        b.rospkg.RosPack.return_value.get_path.return_value = "/task_generator"
        with mock.patch.object(b.Mod_Benchmark, "_taskgen_write", return_value=written or {"written": 1}) as write, \
                mock.patch.object(b.Mod_Benchmark, "_apply_task_config", return_value=applied) as apply, \
                mock.patch.object(b, "reseed") as reseed, \
                mock.patch.object(b, "_get_rosmaster_pid", return_value=1), \
                mock.patch.object(b.subprocess, "Popen") as popen, \
                mock.patch.object(b.Mod_Benchmark, "_suicide", lambda self: None), \
                mock.patch.object(b.Mod_Benchmark, "_logger", mock.MagicMock()), \
                mock.patch.object(b.Mod_Benchmark, "_log_episode", lambda self: None), \
                mock.patch.object(b, "rosparam_get", side_effect=lambda cast, name, default=None: default), \
                mock.patch("builtins.open", mock.mock_open()):
            bench._reincarnate()
        return bench, write.call_args[0][0], apply, reseed, popen

    def test_timeout_written(self):
        _, overlay, _, _, _ = self._reincarnate(120)
        self.assertEqual(120, overlay["timeout"])

    def test_no_timeout_keeps_the_runs(self):
        # the launch's timeout (300 here), also after a stage with another one
        _, overlay, _, _, _ = self._reincarnate(None)
        self.assertEqual(300.0, overlay["timeout"])
        for timeout, arg in ((120, ["timeout:=120"]), (None, ["timeout:=300.0"])):
            *_, popen = self._reincarnate(timeout, restart=True)
            self.assertEqual(arg, [a for a in popen.call_args[0][0] if a.startswith("timeout:=")])

    def test_seed_set_by_the_stage_config(self):
        *_, reseed, _ = self._reincarnate(120, written={"RANDOM": {"seed": 42}})
        reseed.assert_called_once_with(42)

    def test_config_not_applied_stops_the_run(self):
        b = self.benchmark
        with mock.patch.object(b.Mod_Benchmark, "_suicide") as suicide:
            b.rospy.reset_mock()
            bench = self._bench(120)
            with mock.patch.object(b.Mod_Benchmark, "_apply_task_config", return_value=False), \
                    mock.patch.object(b, "reseed") as reseed, \
                    mock.patch.object(b.Mod_Benchmark, "_logger", mock.MagicMock()):
                bench._start_stage({})
        reseed.assert_not_called()
        # the whole launch stops, not only the task generator (it is not required)
        suicide.assert_called_once()
        b.rospy.logfatal.assert_called_once()

    def test_timeout_of_the_launch(self):
        b = self.benchmark
        bench = self._bench(None)
        for params, expected in (({"benchmark_run_timeout": 240.0, "/task_generator_server/timeout": 90.0}, 240.0),
                                 ({"/task_generator_server/timeout": 90.0}, 90.0), ({}, -1.0)):
            with mock.patch.object(b, "rosparam_get",
                                   side_effect=lambda cast, name, default=None: params.get(name, default)):
                self.assertEqual(expected, bench._launch_timeout())

    def test_run_timeout_kept_through_a_relaunch(self):
        b = self.benchmark
        bench = self._bench(None)
        lock = mock.mock_open()
        with mock.patch.object(b.Mod_Benchmark, "_taskgen_write", return_value={}), \
                mock.patch.object(b.Mod_Benchmark, "_start_stage"), \
                mock.patch.object(b.Mod_Benchmark, "_log_episode", lambda self: None), \
                mock.patch.object(b, "rosparam_get", side_effect=lambda cast, name, default=None: default), \
                mock.patch("builtins.open", lock):
            bench._reincarnate()
        self.assertEqual("run 0 0 1 300.0", lock().write.call_args_list[0][0][0])
        # an older lock file without it: none (the launch's is used)
        for text, run_timeout in (("run 0 3 1 300.0", 300.0), ("run 0 3 1", None)):
            with mock.patch("builtins.open", mock.mock_open(read_data=text)):
                self.assertEqual(("run", 0, 3, 1, run_timeout), b.Mod_Benchmark._resume())

    def test_recorder_moves_after_the_reset(self):
        b = self.benchmark
        b.rospy.reset_mock()
        bench, *_ = self._reincarnate(120)
        b.rospy.ServiceProxy.assert_not_called()  # before_reset: nothing placed yet
        bench.after_reset()
        b.rospy.ServiceProxy.assert_called_once_with("/rto/change_directory", mock.ANY)
        bench.after_reset()  # the next episode of the stage
        b.rospy.ServiceProxy.assert_called_once()

    def test_stage_start_applies_config_and_restarts_the_rng(self):
        bench, overlay, apply, reseed, _ = self._reincarnate(120)
        apply.assert_called_once_with({"written": 1})
        reseed.assert_called_once_with(overlay["RANDOM"]["seed"])
        self.assertEqual(bench._stage_seed(), overlay["RANDOM"]["seed"])

    def test_first_reset_after_a_relaunch_starts_the_stage(self):
        b = self.benchmark
        bench = self._bench(120)
        bench._episode_index = -1  # as after a (re)launch
        b.yaml.load.return_value = {"SCENARIO": {"file": "CS5.json"}}
        with mock.patch.object(b.Mod_Benchmark, "_start_stage") as start, \
                mock.patch.object(b.Mod_Benchmark, "_log_episode", lambda self: None), \
                mock.patch.object(b, "rosparam_get", side_effect=lambda cast, name, default=None: default), \
                mock.patch("builtins.open", mock.mock_open()):
            bench.before_reset()
            bench.before_reset()
        start.assert_called_once_with({"SCENARIO": {"file": "CS5.json"}})
        self.assertEqual(1, bench._episode)


class TestConstantsReconfigure(unittest.TestCase):
    """The RNG is recreated only when the seed changes: the same config arrives several times."""

    @classmethod
    def setUpClass(cls):
        cls.constants = load("constants.py")

    def _config(self, seed):
        return {"RANDOM_seed": seed, "episodes": -1, "goal_radius": 0.5,
                "goal_tolerance_angle": 1.0, "timeout": 120}

    def test_same_seed_keeps_the_sequence(self):
        c = self.constants
        c.reseed(0)
        c.reconfigure(self._config(5))
        first = c.Config.General.RNG.random()
        c.reconfigure(self._config(5))  # the filewatcher's copy of the same config
        second = c.Config.General.RNG.random()
        c.reconfigure(self._config(6))
        c.reconfigure(self._config(5))
        self.assertNotEqual(first, second)
        self.assertEqual(first, c.Config.General.RNG.random())
        self.assertEqual(120, c.Config.Robot.TIMEOUT)

    def test_reseed_restarts_an_unchanged_seed(self):
        c = self.constants
        c.reseed(0)
        c.reconfigure(self._config(5))
        first = c.Config.General.RNG.random()
        c.reseed(5)  # next stage, same seed
        self.assertEqual(first, c.Config.General.RNG.random())
        c.reconfigure(self._config(5))  # its config arriving later changes nothing
        self.assertNotEqual(first, c.Config.General.RNG.random())


if __name__ == "__main__":
    unittest.main()
