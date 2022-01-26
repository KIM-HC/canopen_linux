#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Calibrates Powered Caster Vehicle
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

import rospkg

class CalibratePCV():
    def __init__(self):
        pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')

        ## parameters chosen by me
        mv_start_time_q = 0.0
        mv_start_time_b = 0.0
        self.time_delay = mv_start_time_q - mv_start_time_b
        self.data_use_start_time = 10.0  ## seconds after mv_start
        

        self.set0_q = open(pkg_path + '/debug/set_'+0+'_qvalue.txt', 'a')
        self.set1_q = open(pkg_path + '/debug/set_'+1+'_qvalue.txt', 'a')
        self.set2_q = open(pkg_path + '/debug/set_'+2+'_qvalue.txt', 'a')
        self.set3_q = open(pkg_path + '/debug/set_'+3+'_qvalue.txt', 'a')


