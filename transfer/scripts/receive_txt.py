#! /usr/bin/env python
# 接收txt文件的节点(receive_txt)，订阅的话题为txt
import rospy
from std_msgs.msg import String
from pathlib import Path

# 写入文件的目录
txt_receive = "/home/sun/Desktop/copy.txt"

# 接收到话题的回调函数
def do_msg(msg):
    rospy.loginfo("I heard: %s", msg.data)
    dir = Path.cwd()
    with open(txt_receive, "a") as f:
        f.write(msg.data)

if __name__ == "__main__":
    rospy.init_node("receive_txt")
    sub = rospy.Subscriber("txt", String, do_msg, queue_size=10)
    rospy.spin()
