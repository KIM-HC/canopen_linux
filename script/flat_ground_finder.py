#!/usr/bin/env python3
"""
for finding flat ground
Kim Hyoung Cheol
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

import elmo

if __name__ == "__main__":
    ## joint Torque value : in_air - 600    on_ground - 1200
    node_set = [1,2,3,4,5,6,7,8]


    sset = 3

    tt_ = elmo.TestElmo(node_list=node_set)
    tt_.find_flat_ground(stationary_set=0, target_1=0.0, jt_tor_r=960)


