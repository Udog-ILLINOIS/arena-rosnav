#! /usr/bin/env python3

import functools
import os
from typing import Callable, Optional

import rospkg
import yaml

import rospy

import watchdog.observers
import watchdog.events

import dynamic_reconfigure.client
from task_generator.cfg import TaskGeneratorConfig
from task_generator.task_config import task_config_params


def observe(file: str, callback: watchdog.events.FileSystemEventHandler):
    observer = watchdog.observers.Observer()
    observer.schedule(
        path=file,
        event_handler=callback
    )
    observer.start()
    return observer


def safe_callback(fn: Callable):
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except KeyboardInterrupt as e:
            raise e
        except:
            pass

    return wrapper


def task_config_name(tm_modules: str, master_uri: str) -> str:
    """
    The benchmark module writes each stage's task config to a file per ROS
    master port (benchmark.py, TASK_GENERATOR_CONFIG); every other launch uses
    the shared task_generator.yaml.
    """
    if "benchmark" in [mod for mod in tm_modules.split(",") if mod != ""]:
        port = master_uri.split(":")[-1].rstrip("/")
        return f"task_generator_{port}.yaml"
    return "task_generator.yaml"

def run(namespace: Optional[str] = None):

    if namespace is None:
        namespace = rospy.get_namespace()

    client = dynamic_reconfigure.client.Client(
        name = os.path.join(namespace, "task_generator_server")
    )

    FILE_TASK_CONFIG = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        task_config_name(
            str(rospy.get_param("tm_modules", "")),
            os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        )
    )
    rospy.loginfo(f"task config file: {FILE_TASK_CONFIG}")

    class TaskConfigHandler(watchdog.events.FileSystemEventHandler):

        def __init__(self) -> None:
            super().__init__()
            self.reconfigure()

        @staticmethod
        def reconfigure():
            with open(FILE_TASK_CONFIG) as f:
                content = dict(yaml.load(f, yaml.FullLoader))

            rospy.logdebug("SENSING CHANGE OF TASK_MODE PARAMS")

            client.update_configuration(
                task_config_params(content, TaskGeneratorConfig.config_description)
            )

        def on_modified(self, event):
            @safe_callback
            def callback():
                self.reconfigure()
            callback()

    observers = [
        observe(path, cb)
        for path, cb
        in [
            (FILE_TASK_CONFIG, TaskConfigHandler())
        ]
    ]

    def cleanup():
        for observer in observers:
            observer.stop()
        for observer in observers:
            observer.join()

    rospy.on_shutdown(cleanup)

    try:
        while len(active_observers := [observer for observer in observers if observer.is_alive()]):
            for observer in active_observers:
                observer.join(1)
    except KeyboardInterrupt as e:
        raise e
    finally:
        cleanup()
        


if __name__ == "__main__":
    rospy.init_node("task_generator_filewatcher")
    run()
