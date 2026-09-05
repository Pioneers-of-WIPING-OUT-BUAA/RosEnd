#!/usr/bin/env python

from pathlib import Path
import re
import subprocess


def map_path(name, directory, suffix=""):
    if not isinstance(name, str) or re.fullmatch(r"[\w-]{1,100}", name) is None:
        raise ValueError("Invalid map name")
    root = Path(directory).resolve()
    target = (root / (name + suffix)).resolve()
    if target.parent != root:
        raise ValueError("Map path must remain inside the map directory")
    return target


def save_map(name, directory):
    target = map_path(name, directory)
    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["rosrun", "map_server", "map_saver", "-f", str(target)],
        check=True, timeout=4,
    )
    return target
