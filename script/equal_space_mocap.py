#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Edit mocap data to be equally spaced
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

#####################################################
from matplotlib.animation import FuncAnimation      #
from mpl_toolkits.mplot3d import Axes3D             #
from sklearn.cluster import KMeans                  #
import matplotlib.pyplot as plt                     #
import numpy as np                                  #
import rospkg                                       #
import math                                         #
import yaml                                         #
import csv
#####################################################

pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')

file_names = ['mocap_2022_02_28_0', 'mocap_2022_02_28_1',
              'mocap_2022_02_28_2', 'mocap_2022_02_28_3',
              'mocap_2022_01_28_0', 'mocap_2022_01_28_1',
              'mocap_2022_01_28_2', 'mocap_2022_01_28_3']

space = 4  # mm
print('space: {0} mm'.format(space))

for file_name in file_names:
    sp_max = 0.0
    with open(pkg_path + '/setting/basic/original_mocap_data.yaml', 'r') as stream:
                yam = yaml.safe_load(stream)
                idx_c = yam[file_name]['center_index']
                idx_f = yam[file_name]['front_index']
                idx_l = yam[file_name]['left_index']

    mv = np.loadtxt(pkg_path + '/data/mocap/' + file_name + '.txt', delimiter='\t')
    save_path = open(pkg_path + '/data/mocap/equal_' + file_name + '.txt', 'w')
    wr = csv.writer(save_path, delimiter='\t')

    prev_point = mv[0, idx_c:idx_c+3]
    equal_spaced = [np.concatenate((np.array([mv[0,0]]),mv[0,idx_c:idx_c+3],mv[0,idx_f:idx_f+3],mv[0,idx_l:idx_l+3]), axis=0)]
    wr.writerow(mv[0])
    for i in range(np.size(mv, 0)):
        curr_point = mv[i, idx_c:idx_c+3]
        if i != 0:
            cp_cur = np.linalg.norm(mv[i-1, idx_c:idx_c+3] - curr_point)
            if (sp_max < cp_cur):
                sp_max = cp_cur
        if np.linalg.norm(prev_point - curr_point) > space:
            prev_point = curr_point
            ## rearrange: time center front left
            equal_spaced.append(np.concatenate((np.array([mv[i,0]]),mv[i,idx_c:idx_c+3],mv[i,idx_f:idx_f+3],mv[i,idx_l:idx_l+3]), axis=0))
            wr.writerow(mv[i])
    save_path.close()
    equal_spaced = np.array(equal_spaced)

    print('file:    {0}'.format(file_name))
    print('maximum space:  {0}'.format(sp_max))
    print('before:  {0}'.format(np.size(mv, 0)))
    print('after:   {0}'.format(np.size(equal_spaced, 0)))


