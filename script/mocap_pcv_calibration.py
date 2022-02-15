#!/usr/bin/env python3
"""
Kim Hyoung Cheol
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

import elmo

if __name__ == "__main__":
    ## joint Torque value : in_air - 600    on_ground - 1200
    node_set = [1,2,3,4,5,6,7,8]

    tt_ = elmo.TestElmo(node_list=node_set)
    tt_.mocap_calibration(stationary_set=0, target_1=45.0, target_2=45.0 + 100, st_tor_r=1300, jt_tor_r=960)


