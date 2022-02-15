#!/usr/bin/env python
'''
Kim Hyoung Cheol
changes contents in camera/camera_info
'''
from __future__ import print_function
import rospy
from sensor_msgs.msg import CameraInfo

class CameraInfoConverter():

  def __init__(self):
    self.camera_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.callback)
    self.camera_pub = rospy.Publisher("/camera/camera_info",CameraInfo, queue_size=5)

    ## for realsense KHC#3 1280 X 720
    ## distortion
    self.D = [0.092987, -0.156411, 0.000390, -0.001912, 0.000000]
    ## camera matrix
    self.K = [907.776943, 0.0,        632.923741,
              0.0,        908.541933, 366.009678,
              0.0,        0.0,        1.0]
    ## projection matrix
    self.P = [914.904968, 0.0,        629.780035, 0.0,
              0.0,        922.132568, 366.194701, 0.0,
              0.0,        0.0,        1.0,        0.0]

    ## for realsense KHC#3 1920 X 1080
    ## distortion

    ## camera matrix

    ## projection matrix


  def callback(self,data):
    data.D = self.D
    data.K = self.K
    data.P = self.P
    self.camera_pub.publish(data)

if __name__ == '__main__':
  rospy.init_node('camera_info_converter', anonymous=False)
  cic = CameraInfoConverter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
