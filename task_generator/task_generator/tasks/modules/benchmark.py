import datetime
import hashlib
import json
import pathlib
from task_generator.constants import Constants, reseed
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory
from task_generator.task_config import task_config_params
from task_generator.cfg import TaskGeneratorConfig

import typing
import os
import yaml
import subprocess

import rospkg
import rospy
import arena_evaluation_msgs.srv as arena_evaluation_srvs
import dynamic_reconfigure.client

import logging

port = os.environ["ROS_MASTER_URI"].split(":")[-1].rstrip("/")

def _get_rosmaster_pid() -> int:
    try:
        port = os.environ.get("ROS_MASTER_URI", "http://localhost:11311").split(":")[-1].rstrip("/")
        output = subprocess.check_output(["ps", "-C", "rosmaster", "-o", "pid,args", "h"]).decode()
        for line in output.strip().split("\n"):
            if f"-p {port}" in line:
                return int(line.strip().split()[0])
        raise RuntimeError("could not find rosmaster pid")
    except Exception as e:
        raise RuntimeError("could not determine rosmaster pid") from e

def _il_record_args(runid: str, stage_name: str) -> typing.List[str]:
    """
    record_il.launch arguments for a relaunch, empty if this launch does not record.
    Each stage records into its own session, named after the run and the stage.
    """
    data_root = rosparam_get(str, "/il_session_recorder/data_root", "")
    if not data_root:
        return []
    return [
        "record_il:=true",
        f"il_data_root:={data_root}",
        f"il_operator:={rosparam_get(str, '/il_session_recorder/operator', '')}",
        f"il_session:={runid}_{stage_name}",
    ]

def _ped_vmax_args() -> typing.List[str]:
    """
    ped_vmax launch argument for a relaunch, empty if this launch did not set it.
    """
    ped_vmax = rosparam_get(float, "ped_vmax", None)
    if ped_vmax is None:
        return []
    return [f"ped_vmax:={ped_vmax}"]

def _man_gap_selection_args() -> typing.List[str]:
    """
    Manual gap selection launch arguments for a relaunch, empty for those this launch did not set.
    """
    args = []
    for name in ("man_gap_max_bearing_deg", "man_gap_switch_ratio"):
        value = rosparam_get(float, name, None)
        if value is not None:
            args.append(f"{name}:={value}")
    return args

def _learned_gap_scorer_args() -> typing.List[str]:
    """
    Learned gap scorer launch arguments for a relaunch, empty if this launch did not turn it on.
    """
    if not rosparam_get(bool, "learned_gap_scorer", False):
        return []
    return ["learned_gap_scorer:=true",
            f"learned_gap_scorer_path:={rosparam_get(str, 'learned_gap_scorer_path', '')}"]

def _il_goal_cursor_args() -> typing.List[str]:
    """
    Automatic goal-cursor operator launch argument for a relaunch, empty if this launch did not turn it on.
    """
    if not rosparam_get(bool, "il_goal_cursor", False):
        return []
    return ["il_goal_cursor:=true"]

class _Config(typing.NamedTuple):

    @classmethod
    def parse(cls, obj: typing.Dict):
        return cls(
            suite = cls.Suite(**obj["suite"]),
            contest = cls.Contest(**obj["contest"]),
            general = cls.General(**obj["general"])
        )

    class Suite(typing.NamedTuple):
        config: str
        scale_episodes: float = 1

    class Contest(typing.NamedTuple):
        config: str

    class General(typing.NamedTuple):
        simulator: str

    suite: Suite
    contest: Contest
    general: General


class Suite(typing.NamedTuple):

    @classmethod
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name = name,
            stages = [
                cls.Stage.parse(stage)
                for stage
                in obj["stages"]
            ]
        )

    class Index(int): ...

    class Stage(typing.NamedTuple):
        name: str
        episodes: int
        robot: str
        map: str
        tm_robots: Constants.TaskMode.TM_Robots
        tm_obstacles: Constants.TaskMode.TM_Obstacles
        config: typing.Dict

        seed: int
        timeout: typing.Optional[float]

        @classmethod
        def hash(cls, obj: typing.Dict) -> int:
            """
            hash json-serializable object to non-negative int32
            """
            return 0x7f_ff_ff_ff & int.from_bytes(
                hashlib.sha1(json.dumps(obj).encode()).digest()[-4:],
                byteorder="big"
            )

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Suite.Stage":
            # none: the run's timeout (the launch's)
            obj.setdefault("timeout", None)
            obj.setdefault("seed", cls.hash(obj))
            return cls(**obj)
        

    name: str
    stages: typing.List[Stage]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> Index:
        return self.Index(len(self.stages)-1)
    
    def config(self, index: Index) -> Stage:
        return self.stages[index]

class Contest(typing.NamedTuple):

    @classmethod
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name = name,
            contestants = [
                cls.Contestant.parse(contestant)
                for contestant
                in obj["contestants"]
            ]
        )

    class Index(int): ...

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Contest.Contestant":
            obj.setdefault("inter_planner", "bypass")
            return cls(**obj)

    name: str
    contestants: typing.List[Contestant]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> Index:
        return self.Index(len(self.contestants)-1)
    
    def config(self, index: Index) -> Contestant:
        return self.contestants[index]

@TaskFactory.register_module(Constants.TaskMode.TM_Module.BENCHMARK)
class Mod_Benchmark(TM_Module):

    DIR: Namespace = Namespace(
        os.path.join(
                rospkg.RosPack().get_path("arena_bringup"),
                "configs",
                "benchmark"
            )
        )
    LOCK_FILE = f"resume_{port}.lock"
    LOG_DIR = DIR("logs")
    TASK_GENERATOR_CONFIG = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        f"task_generator_{port}.yaml"
    )
    TASK_GENERATOR_CONFIG_BKUP = TASK_GENERATOR_CONFIG + ".bkup"

    _config: _Config
    _suite: Suite
    _contest: Contest
    _episode_index: int

    _runid: str
    _contest_index: Contest.Index
    _suite_index: Suite.Index
    _headless: int
    _run_timeout: float  # the launch's timeout, for stages without one

    _requires_restart: bool
    _pending_record_dir: typing.Optional[typing.Tuple[str, str]] = None  # (robot, dir) for after_reset

    # CONFIGURATION

    @classmethod
    def _load_config(cls) -> _Config:
        with open(cls.DIR("config.yaml")) as f:
            return _Config.parse(yaml.load(f, yaml.FullLoader))

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        with open(cls.DIR("contests", contest)) as f:
            return Contest.parse(pathlib.Path(contest).stem, yaml.load(f, yaml.FullLoader))
        
    @classmethod
    def _load_suite(cls, suite: str) -> Suite:
        with open(cls.DIR("suites", suite)) as f:
            return Suite.parse(pathlib.Path(suite).stem, yaml.load(f, yaml.FullLoader))
    
    @classmethod
    def _resume(cls) -> typing.Tuple[str, Contest.Index, Suite.Index, int, typing.Optional[float]]:
        with open(cls.DIR(cls.LOCK_FILE)) as f:
            runid, contest, suite, headless, *run_timeout = f.read().split(" ")
        return runid, Contest.Index(contest), Suite.Index(suite), int(headless), float(run_timeout[0]) if run_timeout else None


    @classmethod
    def _taskgen_backup(cls):
        if os.path.exists(bkup_file := cls.TASK_GENERATOR_CONFIG + ".bkup"): return
        with open(cls.TASK_GENERATOR_CONFIG) as fr:
            with open(bkup_file, "w") as fw:
                fw.write(fr.read())

    @classmethod
    def _taskgen_write(cls, *configs: typing.Dict, base: typing.Optional[str] = None) -> typing.Dict:
        """
        Overlay configs on base (default: the current task config), write the result
        to the task config file and return it.
        """

        def overwrite(source: typing.Dict, target: typing.Dict):
            for k,v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        with open(base or cls.TASK_GENERATOR_CONFIG, "r") as f:
            joint_config = yaml.load(f, yaml.FullLoader)

        for config in configs:
            overwrite(config, joint_config)        
        
        # yaml has problems with r+
        with open(cls.TASK_GENERATOR_CONFIG, "w") as f:
            yaml.dump(joint_config, f)

        return joint_config

    @classmethod
    def _taskgen_restore(cls, cleanup: bool = False):
        with open(cls.TASK_GENERATOR_CONFIG_BKUP) as fr:
            with open(cls.TASK_GENERATOR_CONFIG, "w") as fw:
                fw.write(fr.read())
        if cleanup:
            os.remove(cls.TASK_GENERATOR_CONFIG_BKUP)
        

    # RUNTIME

    def __init__(self, **kwargs):
        
        self._config = self._load_config()
        suite_passed = rosparam_get(str, "benchmark_suite", self._config.suite.config)
        rospy.loginfo(f"LOADING SUITE: {suite_passed}")
        self._suite = self._load_suite(suite_passed)
        self._contest = self._load_contest(self._config.contest.config)

        self._requires_restart = False

        # first run
        if not rosparam_get(bool, "benchmark_resume", False):
            self._taskgen_backup()
            self._runid = f"{self._contest.name}_{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}"
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            self._headless = rosparam_get(int, "headless", 1)
            self._run_timeout = self._launch_timeout()

            self._taskgen_backup()
            with open(self.TASK_GENERATOR_CONFIG_BKUP) as f:
                base_config = f.read() 

            os.makedirs(self.LOG_DIR, exist_ok=True)
            with open(self.LOG_DIR(f"{self._runid}.log"), "w") as f:
                f.write(f"run {self._runid}\n")
                f.write(f"of contest {self._contest.name} with {len(self._contest.contestants)} contestants\n")
                f.write(f"on suite {self._suite.name} with {len(self._suite.stages)} stages\n")
                f.write(f"total of {len(self._contest.contestants) * sum([int(self._config.suite.scale_episodes * self._suite.config(Suite.Index(index)).episodes) for index in range(self._suite.min_index, self._suite.max_index+1)])} episodes\n")
                f.write("\n")
                f.write(f"Simulator: {self._config.general.simulator}\n")
                f.write(f"Base Config: {json.dumps(base_config)}\n")
                f.write(80*"=" + "\n")
                f.write("\n")

            self._log_contest()
            self._log_suite()

            self._requires_restart = True
            return self._reincarnate()

        self._runid, contest_index, suite_index, headless, run_timeout = self._resume()
        # a lock file written before the run timeout was kept: this launch's timeout
        self._run_timeout = run_timeout if run_timeout is not None else self._launch_timeout()

        self._contest_index, self._suite_index, self._headless = contest_index, suite_index, headless
        self._episode = -1

    def before_reset(self):
        if self._episode < 0:
            # first reset after a (re)launch: the filewatcher may not have applied the
            # stage's config yet
            with open(self.TASK_GENERATOR_CONFIG) as f:
                self._start_stage(yaml.load(f, yaml.FullLoader))
        # count the episode before the task modes place robot and obstacles, so that a
        # stage change writes the new stage's config in time for this reset (IL fix 51)
        self._episode += 1

    def after_reset(self):
        # a stage change on the same map: the evaluation recorder moves to the new
        # stage's folder once the reset has placed everything, as before IL fix 51
        if self._pending_record_dir is not None:
            robot, record_data_dir = self._pending_record_dir
            self._pending_record_dir = None
            rospy.ServiceProxy(f"/{robot}/change_directory", arena_evaluation_srvs.ChangeDirectory).call(arena_evaluation_srvs.ChangeDirectoryRequest(record_data_dir))


    _logger_object: logging.Logger

    @property
    def _logger(self) -> logging.Logger:
        if not hasattr(self, "_logger_object"):
            handler = logging.FileHandler(self.LOG_DIR(f"{self._runid}.log"))
            handler.setFormatter(logging.Formatter('%(created)f: %(message)s'))
            
            logger = logging.getLogger("benchmark")
            logger.setLevel(logging.DEBUG)
            logger.addHandler(handler)

            self._logger_object = logger
        
        return self._logger_object
    
    def _log_contest(self):
        self._logger.info(f"\tC [{1+self.contest_index:0>{len(str(1+self._contest.max_index))}}/{1+self._contest.max_index}] {self._contest.config(self._contest_index).name}")

    def _log_suite(self):
        self._logger.info(f"\t\tS [{1+self.suite_index:0>{len(str(1+self._suite.max_index))}}/{1+self._suite.max_index}] {self._suite.config(self._suite_index).name}")

    def _log_episode(self):
        if self._episode < 0: return #pre-init
        episode_param = rosparam_get(int, "benchmark_episodes", -1)
        episode_limit = episode_param
        if (episode_param <= 0):
            episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        self._logger.info(f"\t\t\tE [{1+self._episode:0>{len(str(episode_limit))}}/{episode_limit}]")

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index
    
    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)

        if self._contest_index > self._contest.max_index:
            self._logger.info("BENCHMARK COMPLETED SUCCESSFULLY")
            os.remove(self.DIR(self.LOCK_FILE))
            self._taskgen_restore(cleanup=True)
            self._suicide()
        else:
            self._log_contest()

            self._logger.debug(f"contestant change requires restart")
            self._requires_restart = True

            self._reincarnate()

    @property
    def suite_index(self) -> Suite.Index:
        return self._suite_index
    
    @suite_index.setter
    def suite_index(self, index: int):

        old_config = self._suite.config(self._suite_index)
        self._suite_index = Suite.Index(index)

        if self._suite_index > self._suite.max_index:
            self._suite_index = self._suite.min_index
            self.contest_index += 1
        else:
            self._log_suite()

            new_config = self._suite.config(self._suite_index)

            if new_config.map != old_config.map:
                self._logger.debug(f"map change requires restart")
                self._requires_restart = True
                
            if new_config.robot != old_config.robot:
                self._logger.debug(f"robot change requires restart")
                self._requires_restart = True

            # task modes are launch parameters, applied only by a relaunch
            if (new_config.tm_robots, new_config.tm_obstacles) != (old_config.tm_robots, old_config.tm_obstacles):
                self._logger.debug(f"task mode change requires restart")
                self._requires_restart = True
            
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index
    
    @_episode.setter
    def _episode(self, episode: int):
        episode_param = rosparam_get(int, "benchmark_episodes", -1)
        episode_limit = episode_param
        if (episode_param <= 0):
            episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        if episode >= int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes):
            self._episode_index = 0
            self.suite_index += 1
        else:
            self._episode_index = episode
            self._log_episode()

    def _reincarnate(self):

        with open(self.DIR(self.LOCK_FILE), "w") as f:
            f.write(f"{self._runid} {self._contest_index} {self._suite_index} {self._headless} {self._run_timeout}")

        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)

        # a relaunch also passes it as timeout:=, a stage on the same map only gets it here
        timeout = suite_config.timeout if suite_config.timeout is not None else self._run_timeout
        overlay = {"episodes": -1, "timeout": timeout, "RANDOM": {"seed": self._stage_seed()}}
        # the stage's config on top of the base config, written once: the filewatcher
        # never sees the base config alone in between (IL fix 51)
        task_config = self._taskgen_write(overlay, suite_config.config, base=self.TASK_GENERATOR_CONFIG_BKUP)

        default_record_data_dir = f"{self._runid}/{contest_config.name}/{suite_config.name}"
        top_file_location = rosparam_get(str, "rec_dir", '')
        if (top_file_location != ''):
            top_file_location = top_file_location + '/'
        record_data_dir = top_file_location + default_record_data_dir

        launch_file = rosparam_get(str, "benchmark_launch_file", "start_arena.launch")

        rospy.loginfo(f"REINCARNATING with tm_obstacles: {suite_config.tm_obstacles}")

        if self._requires_restart:
            self._logger.info(f"{_get_rosmaster_pid()}")
            subprocess.Popen(
                [
                    os.path.join(
                        rospkg.RosPack().get_path("task_generator"),
                        "scripts",
                        "delay_restart.py"
                    ),
                    f"{_get_rosmaster_pid()}",
                    "arena_bringup",
                    launch_file,
                    "tm_modules:=benchmark",
                    "benchmark_resume:=true",
                    "record_data:=true",
                    f"record_data_dir:={record_data_dir}",

                    f"simulator:={config.general.simulator}",
                    f"timeout:={timeout}",
                    f"headless:={self._headless}",
                    
                    # contest
                    f"inter_planner:={contest_config.inter_planner}",
                    f"local_planner:={contest_config.local_planner}",

                    # suite
                    f"model:={suite_config.robot}",
                    f"map_file:={suite_config.map}",
                    f"tm_robots:={suite_config.tm_robots}",
                    f"tm_obstacles:={suite_config.tm_obstacles}",
                    f"benchmark_launch_file:={rosparam_get(str, 'benchmark_launch_file', 'start_arena.launch')}",
                    f"benchmark_episodes:={rosparam_get(int, 'benchmark_episodes', -1)}",
                    f"benchmark_suite:={rosparam_get(str, 'benchmark_suite', '')}",
                    f"rec_dir:={rosparam_get(str, 'rec_dir', '')}",
                    f"man_gap:={rosparam_get(str, 'man_gap', False)}",
                    f"force_factor:={rosparam_get(float, 'force_factor', 0.0)}",
                    *_ped_vmax_args(),
                    *_man_gap_selection_args(),
                    *_learned_gap_scorer_args(),
                    *_il_goal_cursor_args(),
                    *_il_record_args(self._runid, suite_config.name),
                ],
                start_new_session=True,
                env=os.environ.copy()
            )
            self._suicide()
        
        else:
            self._start_stage(task_config)
            self._pending_record_dir = (suite_config.robot, record_data_dir)
            self._episode = 0

    def _launch_timeout(self) -> float:
        """
        This launch's timeout:= (start_trials.launch keeps it as benchmark_run_timeout;
        the server's param can already hold a stale task config's), else the server's.
        """
        return rosparam_get(float, "benchmark_run_timeout", rosparam_get(float, self.NODE_CONFIGURATION("timeout"), -1.0))

    def _stage_seed(self) -> int:
        return self._suite.config(self._suite_index).seed ^ Suite.Stage.hash({"":self._runid})

    def _start_stage(self, task_config: typing.Dict):
        """
        Put the stage's config in place for the reset that starts it: the filewatcher
        applies the written file asynchronously. The RNG restarts from the config's
        seed (the stage's, or one its config sets), also when the previous stage had
        the same one. If the config cannot be applied, the run stops: the stage would
        start on the previous config and take the new one at some point during it.
        """
        if not self._apply_task_config(task_config):
            # the filewatcher would apply it at some point during the stage
            rospy.logfatal("benchmark: the stage's task config could not be applied, stopping the run")
            self._logger.info("STAGE CONFIG NOT APPLIED, RUN STOPPED")
            # the whole launch, as at the end of a run (the lock file stays: the run is not complete)
            self._suicide()
            return
        reseed(task_config.get("RANDOM", {}).get("seed", self._stage_seed()))

    def _apply_task_config(self, task_config: typing.Dict) -> bool:
        """
        Set the whole task config on task_generator_server and return once the server
        has applied it (as the filewatcher does, but synchronously). False if it failed.
        """
        try:
            if not hasattr(self, "_task_config_client"):
                self._task_config_client = dynamic_reconfigure.client.Client(
                    name=self.NODE_CONFIGURATION, timeout=10
                )
            self._task_config_client.update_configuration(
                task_config_params(task_config, TaskGeneratorConfig.config_description)
            )
            return True
        except Exception as e:
            rospy.logerr(f"benchmark: task config not applied: {e!r}")
            return False

    def _suicide(self):
        subprocess.run(["kill", f"{_get_rosmaster_pid()}"])
        rospy.signal_shutdown("goodbye cruel world")