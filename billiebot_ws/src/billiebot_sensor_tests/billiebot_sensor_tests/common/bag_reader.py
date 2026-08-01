"""Offline rosbag2 reading for analysis CLIs.

No live ROS node or executor is required — this only performs static message
deserialization, which is why analysis code can run entirely offline (a copied bag on
another machine, days later) and still produce identical metrics.
"""

from pathlib import Path
from typing import Iterator, Tuple

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class BagReader:
    def __init__(self, bag_path, storage_id: str = 'sqlite3'):
        self.bag_path = Path(bag_path)
        self.storage_id = storage_id
        self._type_map = self._load_topics_and_types()

    def _load_topics_and_types(self) -> dict:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id=self.storage_id),
            rosbag2_py.ConverterOptions('', ''),
        )
        return {t.name: t.type for t in reader.get_all_topics_and_types()}

    @property
    def topic_names_and_types(self) -> dict:
        return dict(self._type_map)

    @property
    def is_empty(self) -> bool:
        metadata_path = self.bag_path / 'metadata.yaml'
        if not metadata_path.exists():
            return True
        with open(metadata_path, 'r') as f:
            meta = yaml.safe_load(f) or {}
        info = meta.get('rosbag2_bagfile_information', {})
        total = sum(
            t.get('message_count', 0) for t in info.get('topics_with_message_count', [])
        )
        return total == 0

    def messages(self, topic: str) -> Iterator[Tuple[int, object]]:
        """Yields (timestamp_ns, deserialized_message) for every message on `topic`, in
        recorded order. Silently yields nothing if the topic was never recorded."""
        if topic not in self._type_map:
            return
        msg_type = get_message(self._type_map[topic])
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id=self.storage_id),
            rosbag2_py.ConverterOptions('', ''),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next():
            _topic_name, data, timestamp_ns = reader.read_next()
            yield timestamp_ns, deserialize_message(data, msg_type)
