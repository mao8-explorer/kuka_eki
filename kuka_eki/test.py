import rospy
from std_msgs.msg import String

# 回调函数
def callback(data):
    rospy.loginfo("I heard %s", data.data)

def main_task():
    # 主任务逻辑
    rospy.loginfo("Executing main task...")
    rospy.sleep(0.1)  # 模拟主任务的处理时间

def main():
    rospy.init_node('my_node', anonymous=True)

    # 创建订阅者
    rospy.Subscriber("chatter", String, callback)

    rate = rospy.Rate(10)  # 设置循环频率为10Hz

    while not rospy.is_shutdown():
        main_task()
        # rospy.spinOnce() 的替代方案
        # rospy.sleep(0.1)  # 确保在循环中给回调函数留有时间处理

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
