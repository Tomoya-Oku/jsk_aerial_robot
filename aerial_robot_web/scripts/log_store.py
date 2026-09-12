#!/usr/bin/env python3
"""Local ROS bag conversion and playback storage for aerial_robot_web."""

import datetime
import hashlib
import json
import math
import os
import re
import secrets
import shutil
import threading


LOG_ID_RE = re.compile(r"^[A-Za-z0-9_-]{8,64}$")
SAFE_FILENAME_RE = re.compile(r"[^A-Za-z0-9_.-]+")
SCHEMA_VERSION = 1


class LogError(Exception):
    """HTTP-friendly log import error."""

    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc)


def iso_time(value):
    return value.astimezone(datetime.timezone.utc).isoformat().replace("+00:00", "Z")


def safe_filename(filename):
    name = os.path.basename(filename or "log.bag")
    name = SAFE_FILENAME_RE.sub("_", name).strip("._") or "log.bag"
    if not name.lower().endswith(".bag"):
        raise LogError("only .bag files are supported")
    return name[:180]


def message_type(message):
    return getattr(message, "_type", "") or ""


def stamp_seconds(stamp):
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    secs = float(getattr(stamp, "secs", 0))
    nsecs = float(getattr(stamp, "nsecs", 0))
    return secs + nsecs * 1e-9


def finite_number(value):
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def pose_payload(message):
    """Return a compact pose for supported ROS pose-bearing messages."""
    kind = message_type(message)
    if kind == "nav_msgs/Odometry":
        pose = getattr(getattr(message, "pose", None), "pose", None)
    elif kind in ("geometry_msgs/PoseStamped", "geometry_msgs/PoseWithCovarianceStamped"):
        pose = getattr(message, "pose", None)
        if kind.endswith("PoseWithCovarianceStamped"):
            pose = getattr(pose, "pose", None)
    elif kind == "geometry_msgs/TransformStamped":
        transform = getattr(message, "transform", None)
        if transform is None:
            return None
        position = getattr(transform, "translation", None)
        orientation = getattr(transform, "rotation", None)
        pose = type("Pose", (), {"position": position, "orientation": orientation})()
    else:
        return None
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return None
    values = [
        finite_number(getattr(position, axis, None)) for axis in ("x", "y", "z")
    ] + [finite_number(getattr(orientation, axis, None)) for axis in ("x", "y", "z", "w")]
    if any(value is None for value in values):
        return None
    return {"p": values[:3], "q": values[3:]}


def joint_payload(message):
    if message_type(message) != "sensor_msgs/JointState":
        return None
    names = list(getattr(message, "name", []) or [])
    positions = list(getattr(message, "position", []) or [])
    count = min(len(names), len(positions))
    clean_positions = [finite_number(value) for value in positions[:count]]
    if not count or any(value is None for value in clean_positions):
        return None
    return {"n": [str(name) for name in names[:count]], "p": clean_positions}


def flatten_numeric(message, prefix="", depth=0, limit=24):
    """Extract small numeric fields without serializing unbounded ROS arrays."""
    result = {}
    if depth > 3 or limit <= 0 or message is None:
        return result
    slots = list(getattr(message, "__slots__", []) or [])
    for slot in slots:
        if len(result) >= limit or slot in ("header", "name", "position", "velocity", "effort"):
            continue
        value = getattr(message, slot, None)
        key = "{}.{}".format(prefix, slot) if prefix else slot
        number = finite_number(value)
        if number is not None:
            result[key] = number
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value[:12]):
                item_number = finite_number(item)
                if item_number is not None:
                    result["{}[{}]".format(key, index)] = item_number
                if len(result) >= limit:
                    break
        elif hasattr(value, "__slots__"):
            nested = flatten_numeric(value, key, depth + 1, limit - len(result))
            result.update(nested)
    return result


def numeric_payload(message):
    kind = message_type(message)
    if kind == "sensor_msgs/JointState":
        return {}
    data = getattr(message, "data", None)
    number = finite_number(data)
    if number is not None:
        return {"value": number}
    return flatten_numeric(message)


def event_payload(topic, message, previous):
    """Extract state transitions and explicit event/failsafe strings."""
    lower = topic.lower()
    data = getattr(message, "data", None)
    watched = any(token in lower for token in ("flight_state", "failsafe", "gate", "reject", "event"))
    if not watched or data is None:
        return None
    comparable = data if isinstance(data, (str, bool, int, float)) else str(data)
    if previous == comparable:
        return None
    if "flight_state" in lower:
        label = "flight state {}".format(data)
        tone = "ok"
    elif "failsafe" in lower:
        label = str(data) if str(data).strip() else "failsafe"
        tone = "bad"
    elif "reject" in lower or "gate" in lower:
        label = "{}: {}".format(topic.rsplit("/", 1)[-1], data)
        tone = "bad" if bool(data) else "ok"
    else:
        label = str(data)
        tone = "warn"
    return {"label": label[:500], "tone": tone, "value": comparable}


def convert_bag(path, max_samples_per_topic):
    """Convert a ROS 1 bag into a bounded, browser-oriented JSON document."""
    try:
        import rosbag
    except ImportError as error:
        raise LogError("python rosbag module is not installed", 500) from error

    data = {
        "schema_version": SCHEMA_VERSION,
        "topics": [],
        "poses": {},
        "joints": {},
        "series": [],
        "events": [],
    }
    series = {}
    seen = {}
    previous_events = {}
    embedded_urdf = ""

    with rosbag.Bag(path, "r") as bag:
        start = float(bag.get_start_time())
        end = float(bag.get_end_time())
        topic_info = bag.get_type_and_topic_info()[1]
        strides = {}
        for topic, info in topic_info.items():
            count = int(getattr(info, "message_count", 0) or 0)
            strides[topic] = max(1, int(math.ceil(float(count) / max_samples_per_topic)))
            data["topics"].append({
                "name": topic,
                "type": getattr(info, "msg_type", ""),
                "message_count": count,
                "sample_stride": strides[topic],
            })

        for topic, message, stamp in bag.read_messages():
            kind = message_type(message)
            seen[topic] = seen.get(topic, 0) + 1
            relative_time = max(0.0, stamp_seconds(stamp) - start)

            event = event_payload(topic, message, previous_events.get(topic))
            if event:
                previous_events[topic] = event.pop("value")
                if len(data["events"]) < 2000:
                    data["events"].append({"t": relative_time, "topic": topic, **event})

            if kind == "std_msgs/String" and topic.rstrip("/").endswith("robot_description"):
                candidate = getattr(message, "data", "")
                if isinstance(candidate, str) and len(candidate) <= 5 * 1024 * 1024:
                    embedded_urdf = candidate

            if (seen[topic] - 1) % strides.get(topic, 1):
                continue

            pose = pose_payload(message)
            if pose:
                data["poses"].setdefault(topic, []).append({"t": relative_time, **pose})

            joints = joint_payload(message)
            if joints:
                data["joints"].setdefault(topic, []).append({"t": relative_time, **joints})

            for field, value in numeric_payload(message).items():
                key = "{}:{}".format(topic, field)
                entry = series.setdefault(key, {
                    "id": key,
                    "topic": topic,
                    "field": field,
                    "points": [],
                })
                entry["points"].append([relative_time, value])

    data["topics"].sort(key=lambda item: item["name"])
    data["series"] = sorted(series.values(), key=lambda item: item["id"])
    return data, {
        "start_time": start,
        "end_time": end,
        "duration": max(0.0, end - start),
        "topic_count": len(data["topics"]),
        "event_count": len(data["events"]),
        "pose_topics": sorted(data["poses"]),
        "joint_topics": sorted(data["joints"]),
        "embedded_urdf": embedded_urdf,
    }


class LogStore:
    """Own locally imported bags and their compact playback data."""

    def __init__(self, root, max_upload_bytes, max_samples_per_topic=2000, keep_source=False):
        self.root = os.path.realpath(os.path.expanduser(root))
        self.max_upload_bytes = int(max_upload_bytes)
        self.max_samples_per_topic = max(100, int(max_samples_per_topic))
        self.keep_source = bool(keep_source)
        self.lock = threading.RLock()
        os.makedirs(self.root, exist_ok=True)

    def _directory(self, log_id):
        if not LOG_ID_RE.match(log_id or ""):
            raise LogError("invalid log id", 404)
        directory = os.path.realpath(os.path.join(self.root, log_id))
        if not directory.startswith(self.root + os.sep):
            raise LogError("invalid log path", 404)
        return directory

    def _json_path(self, log_id, name):
        return os.path.join(self._directory(log_id), name)

    @staticmethod
    def _write_json(path, payload):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, separators=(",", ":"))
        os.replace(temporary, path)

    @staticmethod
    def _read_json(path):
        try:
            with open(path, "r", encoding="utf-8") as stream:
                return json.load(stream)
        except FileNotFoundError as error:
            raise LogError("log not found", 404) from error

    def _metadata(self, log_id):
        return self._read_json(self._json_path(log_id, "metadata.json"))

    def _update_metadata(self, log_id, values):
        with self.lock:
            metadata = self._metadata(log_id)
            metadata.update(values)
            self._write_json(self._json_path(log_id, "metadata.json"), metadata)
            return metadata

    def create(self, input_stream, content_length, filename):
        if content_length <= 0:
            raise LogError("empty upload")
        if content_length > self.max_upload_bytes:
            raise LogError("bag exceeds the configured upload limit", 413)
        filename = safe_filename(filename)
        log_id = secrets.token_urlsafe(12).rstrip("=")
        directory = self._directory(log_id)
        source_path = os.path.join(directory, "source.bag")
        os.makedirs(directory)
        digest = hashlib.sha256()
        remaining = content_length
        try:
            with open(source_path, "wb") as output:
                while remaining:
                    chunk = input_stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise LogError("upload ended before Content-Length bytes were received")
                    output.write(chunk)
                    digest.update(chunk)
                    remaining -= len(chunk)
            now = utc_now()
            metadata = {
                "schema_version": SCHEMA_VERSION,
                "id": log_id,
                "status": "converting",
                "filename": filename,
                "size": content_length,
                "sha256": digest.hexdigest(),
                "created_at": iso_time(now),
                "error": "",
                "urdf": "",
                "robot_ns": "",
                "pose_topic": "",
                "joint_topic": "",
            }
            self._write_json(os.path.join(directory, "metadata.json"), metadata)
        except Exception:
            shutil.rmtree(directory, ignore_errors=True)
            raise

        worker = threading.Thread(target=self._convert, args=(log_id, source_path))
        worker.daemon = True
        worker.start()
        return metadata

    def _convert(self, log_id, source_path):
        try:
            data, summary = convert_bag(source_path, self.max_samples_per_topic)
            self._write_json(self._json_path(log_id, "data.json"), data)
            updates = {
                "status": "ready",
                "error": "",
                **{key: value for key, value in summary.items() if key != "embedded_urdf"},
            }
            if summary.get("embedded_urdf"):
                updates["urdf"] = summary["embedded_urdf"]
            self._update_metadata(log_id, updates)
        except Exception as error:
            self._update_metadata(log_id, {"status": "error", "error": str(error)[:1000]})
        finally:
            if not self.keep_source:
                try:
                    os.remove(source_path)
                except OSError:
                    pass

    def get(self, log_id):
        return self._metadata(log_id)

    def get_data(self, log_id):
        metadata = self._metadata(log_id)
        if metadata.get("status") != "ready":
            raise LogError("log is not ready", 409)
        return self._read_json(self._json_path(log_id, "data.json"))

    def list(self):
        items = []
        for log_id in os.listdir(self.root):
            if not LOG_ID_RE.match(log_id):
                continue
            try:
                metadata = self._metadata(log_id)
            except LogError:
                continue
            summary = {key: value for key, value in metadata.items() if key != "urdf"}
            items.append(summary)
        return sorted(items, key=lambda item: item.get("created_at", ""), reverse=True)

    def configure(self, log_id, config):
        if not isinstance(config, dict):
            raise LogError("configuration must be a JSON object")
        allowed = {}
        for key in ("robot_ns", "pose_topic", "joint_topic"):
            value = config.get(key)
            if isinstance(value, str):
                allowed[key] = value[:500]
        urdf = config.get("urdf")
        if isinstance(urdf, str):
            if len(urdf.encode("utf-8")) > 5 * 1024 * 1024:
                raise LogError("URDF exceeds 5 MiB", 413)
            allowed["urdf"] = urdf
        return self._update_metadata(log_id, allowed)

    def delete(self, log_id):
        directory = self._directory(log_id)
        if not os.path.isdir(directory):
            raise LogError("log not found", 404)
        if self._metadata(log_id).get("status") == "converting":
            raise LogError("wait for log conversion to finish before deleting", 409)
        shutil.rmtree(directory)
