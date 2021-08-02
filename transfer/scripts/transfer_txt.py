#! /usr/bin/env python
# 发送确定目录下的txt文档
# 检测目标的txt文档话题为txt，消息类型为String类型
# 此节点(transfer_txt)为服务(transmit_txt)和话题(txt)双通信，当服务接收到发送的请求后，才会发送目录中读取的txt文件
# 此服务的变量：请求：req(bool), 回复：res(bool)

import rospy
from std_msgs.msg import String
from transfer.srv import transmit_txt,transmit_txtResponse

# 读取检测结果的txt文档的目录，根据自己的文件目录修改
txt_transmit = "/home/sun/Desktop/detect/labels/photo.txt"

# 读取并发送检测后的结果文件
def print_txt():
    # data=[]
    # 一行行读取
    for line in open(txt_transmit, "r"):
        msg.data = line
        pub.publish(msg)
        rospy.loginfo('The data is:%s', msg.data)
        rate.sleep()

# 回调函数，判断是否检测到发送请求
def doReq(txt):
    # 定义一个响应标志位，目前无作用，但为了程序完整性和后续升级而添加
    response = False
    if txt.req:
        rospy.loginfo("Start transmit the result of detection")
        print_txt()
        response = True
    return transmit_txtResponse(response)


if __name__ == "__main__":
    rospy.init_node("transfer_txt")
    # 话题注册
    pub = rospy.Publisher("txt", String, queue_size=10)
    rospy.loginfo("The Publisher has alreay started")
    rospy.sleep(2)
    msg = String()
    # 定义循环频率
    rate = rospy.Rate(1)
    rospy.loginfo("The Service has alreay started")
    # 服务注册
    ser = rospy.Service("transmit_txt",transmit_txt,doReq)
    rospy.spin()

