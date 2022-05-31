#!/usr/bin/env python3
"""
for canopen communication with PCV(elmo controller)
Torque Controlled
Kim Hyoung Cheol
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""
import elmo

if __name__ == "__main__":
    node_set = [1,2,3,4,5,6,7,8]
    tt_ = elmo.TestElmo(node_list=node_set, operation_mode=elmo.OPMode.PROFILED_TORQUE)
    tt_.start_joint_publisher()
