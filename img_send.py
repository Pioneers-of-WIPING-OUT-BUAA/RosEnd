import json
import logging
import math
import os
from pathlib import Path
import threading
import time

import requests


IMG_SAVE_DIR = os.environ.get("ROS_IMAGE_DIR", "/tmp/img")
BACKEND_URL = os.environ.get("ROS_BACKEND_URL", "http://127.0.0.1:8000/api").rstrip("/")
logger = logging.getLogger(__name__)


def image_position(filename):
    parts = Path(filename).stem.split("_")
    if len(parts) != 4:
        raise ValueError("Image filename must contain a timestamp and three coordinates")
    pos = [float(value) for value in parts[1:]]
    if not all(math.isfinite(value) for value in pos):
        raise ValueError("Image coordinates must be finite")
    return pos


def upload_loop(stop_event=None, interval=5):
    stop_event = stop_event or threading.Event()
    last_uploaded = None
    with requests.Session() as session:
        while not stop_event.wait(interval):
            files = sorted(Path(IMG_SAVE_DIR).glob("[0-9]*_*.jpg"), reverse=True)
            if not files or files[0] == last_uploaded:
                continue
            latest = files[0]
            try:
                result = img_send(latest, image_position(latest), session)
                if result is not None:
                    last_uploaded = latest
            except (ValueError, OSError):
                logger.exception("Could not read the latest image")


def img_send(filename, pos=None, session=None):
    pos = [0, 0, 0] if pos is None else pos
    if not isinstance(pos, list) or len(pos) != 3 or not all(type(value) in (int, float) and math.isfinite(value) for value in pos):
        raise ValueError("Position must contain three finite numbers")
    path = Path(filename)
    try:
        with path.open("rb") as image:
            response = (session or requests).post(
                BACKEND_URL + "/detect/upload",
                files={"file": (path.name, image, "image/jpeg")},
                data={"pos": json.dumps(pos)},
                timeout=(5, 45),
            )
        response.raise_for_status()
        data = response.json()
        if not isinstance(data, dict) or not all(key in data for key in ("fire", "smoke", "stranger", "rubbish")):
            raise ValueError("Invalid detection response")
        return {
            "pos": pos, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "exist_fire": data["fire"], "exist_smoke": data["smoke"],
            "exist_stranger": data["stranger"], "exist_rubbish": data["rubbish"],
            "filename": str(path),
        }
    except (requests.RequestException, ValueError, OSError) as exc:
        logger.warning("Image upload failed: %s", type(exc).__name__)
        return None


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    try:
        upload_loop()
    except KeyboardInterrupt:
        pass
