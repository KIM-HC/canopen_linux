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
    tt_.is_cali_speed_compensation = True
    tt_.is_cali_auto_second_steer = True
    tt_.is_cali_use_fixer = True

    tt_.cali_lpf_sensitivity = 0.995
    tt_.cali_target_speed = 3.0
    sset = 3

    tt_.cali_gain_kp = 400.0
    st_tor_r=1300
    jt_tor_r=950
    if tt_.is_cali_use_fixer:
        tt_.cali_gain_kp = 500.0
        st_tor_r=1300
        jt_tor_r=1080


    tt_.mocap_calibration(stationary_set=sset, target_1=45.0, target_2=45.0 + 120, st_tor_r=st_tor_r, jt_tor_r=jt_tor_r)

## CHANGING TORQUE MAKES MORE BAD DATA
