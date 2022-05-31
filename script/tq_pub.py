#!/usr/bin/env python3
import rospy
from elmo import HZ
from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node('tq_publisher')
    desired_pub = rospy.Publisher('/dyros_mobile/desired_joint', JointState, queue_size=5)

    start_time = rospy.Time.now()
    js_ = JointState(
        name = ['m1','m2','m3','m4','m5','m6','m7','m8'],
        velocity = [0,0,0,0,0,0,0,0],
        effort = [0,0,0,0,0,0,0,0],
        position = [0,0,0,0,0,0,0,0]
    )

    r = rospy.Rate(HZ)
    play_time = 5.0

    tor_steer = 800
    tor_roll  = 0
    vel_steer = 0.1
    vel_roll = 0

    for i in range(4):
        s = (i+1)*2  ## steering node
        idx_s = s-1  ## steering index
        ## STEERING ##
        js_.effort[idx_s] = tor_steer
        js_.velocity[idx_s] = vel_steer
        ## ROLLING ##
        js_.effort[idx_s - 1] = tor_roll
        js_.velocity[idx_s - 1] = vel_roll

    # js_.effort = [0, 0, 0, 0, 0, 0, -100, -100]

    while not rospy.is_shutdown():
        js_.header.stamp = rospy.Time.now()
        desired_pub.publish(js_)
        print(js_)
        if (rospy.Time.now() - start_time).to_sec() > play_time: break
        r.sleep()
