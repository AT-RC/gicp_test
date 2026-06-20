import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

conn = sqlite3.connect('dog_ekf_tuning_bag1/dog_ekf_tuning_bag1_0.db3')
c = conn.cursor()
c.execute('SELECT topics.name, topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id')
frames = set()
for row in c.fetchall():
    topic_name, topic_type, data = row
    if topic_name == '/tf':
        msg_type = get_message(topic_type)
        msg = deserialize_message(data, msg_type)
        for t in msg.transforms:
            frames.add(t.header.frame_id)
            frames.add(t.child_frame_id)
            
print(frames)
