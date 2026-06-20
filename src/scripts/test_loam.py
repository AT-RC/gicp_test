import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def print_odom(db_file, topic_name):
    conn = sqlite3.connect(db_file)
    c = conn.cursor()
    c.execute('SELECT topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id WHERE topics.name = ? LIMIT 10', (topic_name,))
    for row in c.fetchall():
        msg_type = get_message(row[0])
        msg = deserialize_message(row[1], msg_type)
        print(f"X: {msg.pose.pose.position.x:.3f}, Y: {msg.pose.pose.position.y:.3f}")

print_odom('dog_ekf_tuning_bag1/dog_ekf_tuning_bag1_0.db3', '/odometry')
