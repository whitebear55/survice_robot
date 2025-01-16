#!/usr/bin/env python3
import rospy, time
from geometry_msgs.msg import Twist        
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

old_status = GoalStatusArray()
old_status.status_list[0].status = 1

def cmd_callback(msg):
    print("cmd_callback")
    if msg.data == "init":
        twist = Twist()              # 새로운 Twist 메세지 변수를 생성한다. 
        twist.linear.x = 0.0      # x축 선형 속도를 정의한 속도로 정의한다.
        twist.angular.z = -0.3      # z축 각속도를 정의한 각속도로 정의한다.
        cmd_pub.publish(twist) # 위에서 정의한 twist 메세지를 publish한다.
        time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기
        twist.linear.x = 0.0      # x축 선형 속도를 정의한 속도로 정의한다.
        twist.angular.z = 0.3      # z축 각속도를 정의한 각속도로 정의한다.
        cmd_pub.publish(twist) # 위에서 정의한 twist 메세지를 publish한다.
        time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기
        twist.linear.x = 0.0      # x축 선형 속도를 정의한 속도로 정의한다.
        twist.angular.z = 0.0      # z축 각속도를 정의한 각속도로 정의한다.
        cmd_pub.publish(twist) # 위에서 정의한 twist 메세지를 publish한다.
        time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기
    elif msg.data == "uturn":
        twist = Twist()              # 새로운 Twist 메세지 변수를 생성한다. 
        twist.linear.x = 0.0      # x축 선형 속도를 정의한 속도로 정의한다.
        twist.angular.z = 0.3      # z축 각속도를 정의한 각속도로 정의한다.
        cmd_pub.publish(twist) # 위에서 정의한 twist 메세지를 publish한다.
        time.sleep(10)  # 실행할 명령어를 전송하기 위해 잠시 대기
        twist.linear.x = 0.0      # x축 선형 속도를 정의한 속도로 정의한다.
        twist.angular.z = 0.0      # z축 각속도를 정의한 각속도로 정의한다.
        cmd_pub.publish(twist) # 위에서 정의한 twist 메세지를 publish한다.
        time.sleep(5)  # 실행할 명령어를 전송하기 위해 잠시 대기

def status_callback(msg):
    print("status_callback")
    global old_status
    now_status = msg
    if len(now_status.status_list): 
        if now_status.status_list[0].status != old_status.status_list[0].status:
            status = GoalStatusArray()
            if now_status.status_list[0].status == 3:
                status.data = "goal_reached"
                status_pub.publish(status)
            elif now_status.status_list[0].status == 2:
                status.data = "stop"
                status_pub.publish(status)
            else:
                status.data = "go"
                status_pub.publish(status)
        old_status = now_status

if __name__ == '__main__':
    rospy.init_node('cmd_node')
    cmd_sub = rospy.Subscriber("cmd", String, cmd_callback, queue_size=1)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback, queue_size=10)
    status_pub = rospy.Publisher("status", String, queue_size=10)
    while(1):
        rospy.spin()

