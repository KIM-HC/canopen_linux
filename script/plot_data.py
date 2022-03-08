#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Calibrates Powered Caster Vehicle
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

#####################################################
from matplotlib.animation import FuncAnimation      # MOVING PLOT
from mpl_toolkits.mplot3d import Axes3D             # 3D PLOT
from matplotlib.patches import Ellipse              # CIRCLE FITTING
from sklearn.cluster import KMeans                  # CLUSTER CENTER POINT
import matplotlib.pyplot as plt                     # PLOT
from ellipse import LsqEllipse                      # CIRCLE FITTING
from scipy import optimize                          # CIRCLE FITTING
import numpy as np                                  #
import rospkg                                       #
import math                                         #
import yaml                                         # READ SETTING DATA
#####################################################
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class CalibratePCV():
    def __init__(self, yaml_path='mocap_21.yaml'):
        ## reads parameters, paths from yaml
        print('=== {0} ==='.format(yaml_path))
        self.pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')

        with open(self.pkg_path + '/setting/' + yaml_path, 'r') as stream:
            self.yam = yaml.safe_load(stream)
        with open(self.pkg_path + '/setting/' + self.yam['basic_file'], 'r') as stream:
            yam_bas = yaml.safe_load(stream)

        ## data given
        self.eqmv_path = []
        self.mv_path = []

        for module in range(4):
            set_name = 'set_' + str(module)
            self.mv_path.append(self.pkg_path + '/data/' + yam_bas[set_name]['mv_file'])
            self.eqmv_path.append(self.pkg_path + '/data/' + yam_bas[set_name]['equal_mv_file'])

        for module in range(4):
            self.plot_robot_animation(module=module, is_mv=True)
            self.plot_robot_animation(module=module, is_mv=False)

    def plot_robot_animation(self, module, is_mv):
        fig = plt.figure(99)
        if is_mv:
            mv = np.loadtxt(self.mv_path[module], delimiter='\t')
        else:
            mv = np.loadtxt(self.eqmv_path[module], delimiter='\t')
        st = 0
        ed = np.size(mv, 0)
        ax = fig.add_subplot(111)

        if is_mv:
            ci = 4
            fi = 1
            li = 7
        else:
            ci = 1
            fi = 4
            li = 7

        line_robot_x, = ax.plot([],[],'r', linewidth=1.0, markersize=5)
        line_robot_y, = ax.plot([],[],'g', linewidth=1.0, markersize=5)
        ax.plot(mv[:,ci],mv[:,ci+1],'k--',linewidth=0.5)

        def update(idx):
            idx = int(idx)
            line_robot_x.set_data(np.array([mv[idx,ci],mv[idx,fi]]),np.array([mv[idx,ci+1],mv[idx,fi+1]]))
            line_robot_y.set_data(np.array([mv[idx,ci],mv[idx,li]]),np.array([mv[idx,ci+1],mv[idx,li+1]]))
            return line_robot_x, line_robot_y

        max_x, min_x = np.max(mv[:,ci]), np.min(mv[:,ci])
        max_y, min_y = np.max(mv[:,ci+1]), np.min(mv[:,ci+1])
        mid_x, mid_y = (max_x + min_x) / 2.0, (max_y + min_y) / 2.0
        len_x, len_y = abs(max_x - min_x), abs(max_y - min_y)
        len_val = max(len_x, len_y)
        ax.set_xlim(mid_x - len_val * 0.75, mid_x + len_val * 0.75)
        ax.set_ylim(mid_y - len_val * 0.75, mid_y + len_val * 0.75)
        ax.set_aspect('equal')
        if is_mv:
            ax.set_title('  mv path')
        else:
            ax.set_title('eqmv path')
        ani = FuncAnimation(fig, update, frames=np.linspace(st, ed, ed - st + 1), interval=1, repeat=True)
        plt.show()

if __name__ == "__main__":
    CalibratePCV()


