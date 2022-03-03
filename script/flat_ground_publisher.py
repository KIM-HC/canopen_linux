#!/usr/bin/env python3
import rospy
from elmo import HZ
from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node('tq_publisher')
    desired_pub = rospy.Publisher('/dyros_mobile/desired_joint', JointState, queue_size=5)
    js_ = JointState(
        name = ['m1','m2','m3','m4','m5','m6','m7','m8'],
        velocity = [0,0,0,0,0,0,0,0],
        effort = [0,0,0,0,0,0,0,0],
        position = [0,0,0,0,0,0,0,0]
    )
    js_.position[0] = 95.0  ## for finding flat ground
    while not rospy.is_shutdown():
        a = input()
        if not rospy.is_shutdown():
            print('MODE CHANGE CALL SENDED')
            js_.header.stamp = rospy.Time.now()
            desired_pub.publish(js_)
