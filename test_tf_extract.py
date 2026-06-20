import rclpy
from rclpy.time import Time
from rclpy.node import Node
import tf2_ros
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import time

class OfflineTfNode(Node):
    def __init__(self):
        super().__init__('offline_tf')
        self.tf_buffer = tf2_ros.Buffer()
        
    def process_bag(self):
        conn = sqlite3.connect('dog_ekf_tuning_bag1/dog_ekf_tuning_bag1_0.db3')
        c = conn.cursor()
        c.execute('SELECT messages.timestamp, topics.name, topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id WHERE topics.name="/tf" OR topics.name="/tf_static"')
        
        for row in c.fetchall():
            timestamp, topic_name, topic_type, data = row
            msg_type = get_message(topic_type)
            msg = deserialize_message(data, msg_type)
            for t in msg.transforms:
                if topic_name == '/tf_static':
                    self.tf_buffer.set_transform_static(t, 'default_authority')
                else:
                    self.tf_buffer.set_transform(t, 'default_authority')
        
        # Now try to lookup
        try:
            # try to get the latest available
            trans = self.tf_buffer.lookup_transform('map_gicp', 'base_link_lio', Time())
            print("map_gicp -> base_link_lio: ", trans.transform.translation)
        except Exception as e:
            print("Lookup failed:", e)

rclpy.init()
n = OfflineTfNode()
n.process_bag()
rclpy.shutdown()
