"""Small rosbag2 sqlite3 reader used by the offline analysis pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterator, List, Tuple

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


@dataclass
class BagRecord:
    topic: str
    timestamp_ns: int
    message: Any


def _header_stamp_ns(message: Any) -> int | None:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else None


class RosbagReader:
    """Read selected topics from a ROS 2 sqlite3 bag in timestamp order."""

    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        self.reader = SequentialReader()
        self.reader.open(
            StorageOptions(uri=bag_path, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
        )
        self.topic_types: Dict[str, str] = {
            item.name: item.type for item in self.reader.get_all_topics_and_types()
        }
        self._classes: Dict[str, Any] = {}

    def available_topics(self) -> Dict[str, str]:
        return dict(self.topic_types)

    def records(self, topics: List[str]) -> Iterator[BagRecord]:
        wanted = set(topics)
        missing = sorted(wanted.difference(self.topic_types))
        if missing:
            raise ValueError(
                'Topics not found in bag: ' + ', '.join(missing) +
                '\nAvailable topics: ' + ', '.join(sorted(self.topic_types))
            )

        while self.reader.has_next():
            topic, raw, bag_timestamp_ns = self.reader.read_next()
            if topic not in wanted:
                continue
            if topic not in self._classes:
                self._classes[topic] = get_message(self.topic_types[topic])
            message = deserialize_message(raw, self._classes[topic])
            timestamp_ns = _header_stamp_ns(message) or int(bag_timestamp_ns)
            yield BagRecord(topic, timestamp_ns, message)


def collect_records(bag_path: str, topics: List[str]) -> Tuple[Dict[str, str], Dict[str, List[BagRecord]]]:
    reader = RosbagReader(bag_path)
    grouped: Dict[str, List[BagRecord]] = {topic: [] for topic in topics}
    for record in reader.records(topics):
        grouped[record.topic].append(record)
    return reader.available_topics(), grouped
