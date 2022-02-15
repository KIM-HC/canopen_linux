#!/usr/bin/env python
'''
Kim Hyoung Cheol
saves aruco pose in csv file
'''
from __future__ import print_function
import os.path
import csv
import time
import rospy
import rospkg
from fiducial_msgs.msg import FiducialTransformArray

class ArucoInfoSaver():
  def __init__(self):
    self.camera_sub = rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.callback)
    self.start_time = rospy.Time.now()
    pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen') + '/data/aruco'
    file_number = 0
    while(os.path.isfile(pkg_path + time.strftime('_%Y_%m_%d_', time.localtime()) + str(file_number) + '.csv')):
      file_number += 1
    self.save_path = open(pkg_path + time.strftime('_%Y_%m_%d_', time.localtime()) + str(file_number) + '.csv', 'w')
    self.wr = csv.writer(self.save_path, delimiter='\t')
    self.is_first = True

  def callback(self,data):
    if (self.is_first):
      self.is_first = False
      self.start_time = rospy.Time.now()

    ## only save when two marks are all found
    idx_0 = 0
    idx_1 = 1
    if (len(data.transforms) == 2):
      if (data.transforms[0].fiducial_id == 0 and data.transforms[1].fiducial_id == 1):
        pass
      elif (data.transforms[0].fiducial_id == 1 and data.transforms[1].fiducial_id == 0):
        idx_0 = 1
        idx_1 = 0
      else:
        return
    else:
      return

    ## time (x y z) (x y z w) (x y z) (x y z w)
    self.wr.writerow([
          (rospy.Time.now() - self.start_time).to_sec(),
          data.transforms[idx_0].transform.translation.x,
          data.transforms[idx_0].transform.translation.y,
          data.transforms[idx_0].transform.translation.z,
          data.transforms[idx_0].transform.rotation.x,
          data.transforms[idx_0].transform.rotation.y,
          data.transforms[idx_0].transform.rotation.z,
          data.transforms[idx_0].transform.rotation.w,
          data.transforms[idx_1].transform.translation.x,
          data.transforms[idx_1].transform.translation.y,
          data.transforms[idx_1].transform.translation.z,
          data.transforms[idx_1].transform.rotation.x,
          data.transforms[idx_1].transform.rotation.y,
          data.transforms[idx_1].transform.rotation.z,
          data.transforms[idx_1].transform.rotation.w
    ])

if __name__ == '__main__':
  rospy.init_node('aruco_saver', anonymous=False)
  cic = ArucoInfoSaver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    cic.save_path.close()
