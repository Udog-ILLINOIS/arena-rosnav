"""
The task config file (task_generator*.yaml) as task_generator_server parameters.

Used by the filewatcher, which applies the file whenever it changes, and by the
benchmark, which applies a stage's config itself so that it is in place before the
reset that starts the stage (IL fix 51).
"""

import json
from typing import Any, Dict, List


def recursive_get(obj: Any, property: List[str], fallback: Any = None) -> Any:
    if not len(property):
        return fallback if obj is None else obj
    try:
        return recursive_get(dict(obj).get(property[0]), property[1:])
    except:
        return fallback


def encode(var: Any):
    if isinstance(var, list):
        return ";".join(map(str, var))
    if isinstance(var, dict):
        return json.dumps(var)
    return var


def get_or_ignore(obj: dict, key: str) -> dict:
    return {key: obj.get(key)} if key in obj else {}


def task_config_params(content: Dict, config_description: Dict) -> Dict[str, Any]:
    """
    Parameters of a task config: top-level parameters by name, grouped ones by their
    "_"-joined path ({"SCENARIO": {"file": f}} -> {"SCENARIO_file": f}), lists joined
    by ";" and dicts as JSON. config_description is TaskGeneratorConfig.config_description.
    """
    return {
        **{
            k: v
            for d in [get_or_ignore(content, parameter["name"]) for parameter in config_description.get("parameters", [])]
            for k, v in d.items()
        },
        **{
            k: v
            for (k, v)
            in (
                (name, encode(recursive_get(content, name.split("_"))))
                for name in
                (
                    str(parameter["name"])
                    for group in config_description.get("groups", [])
                    for parameter in group.get("parameters", [])
                )
            )
            if v is not None
        }
    }
