import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys

def test_read_bag(db_file):
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()
    cursor.execute("SELECT topics.name, topics.type FROM topics")
    topics = cursor.fetchall()
    print(topics)

test_read_bag('dog_ekf_tuning_bag1/dog_ekf_tuning_bag1_0.db3')
