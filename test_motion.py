import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

conn = sqlite3.connect('dog_ekf_tuning_bag1/dog_ekf_tuning_bag1_0.db3')
c = conn.cursor()
c.execute('SELECT topics.name, topics.type, messages.data FROM messages JOIN topics ON topics.id = messages.topic_id')
xs = []
ys = []
for row in c.fetchall():
    topic_name, topic_type, data = row
    if topic_name == '/gicp_pose' or topic_name == '/odometry/global':
        # /odometry/global not in the original bag probably, but just in case
        try:
            msg_type = get_message(topic_type)
            msg = deserialize_message(data, msg_type)
            if hasattr(msg, 'pose'):
                if hasattr(msg.pose, 'position'): # PoseStamped
                    xs.append(msg.pose.position.x)
                    ys.append(msg.pose.position.y)
                elif hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'): # Odometry
                    xs.append(msg.pose.pose.position.x)
                    ys.append(msg.pose.pose.position.y)
        except Exception as e:
            pass

if xs:
    print('Min X:', min(xs), 'Max X:', max(xs))
    print('Min Y:', min(ys), 'Max Y:', max(ys))
else:
    print("No data found")
