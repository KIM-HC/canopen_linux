#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Calibrates Powered Caster Vehicle
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

from os import TMP_MAX
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import rospkg
import yaml

class CalibratePCV():
    def __init__(self, yaml_path):
        ## reads parameters, paths from yaml
        self.pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')
        self.out_path = self.pkg_path + '/setting/output_' + yaml_path

        with open(self.pkg_path + '/setting/' + yaml_path, 'r') as stream:
            self.yam = yaml.safe_load(stream)
        for key in self.yam:
            print(key,':',self.yam[key])
            print()


        self.plot_data()

    ## Time center[X Y Z] x_axis[X Y Z] y_axis[X Y Z]
    def plot_data(self):
        fig = plt.figure(99)
        fig_2 = plt.figure(98)
        fig_3 = plt.figure(97)
        for i in range(4):
            set_name = 'set_' + str(i)
            mv = np.loadtxt(self.pkg_path + '/data/' + self.yam[set_name]['mv_file'], delimiter='\t')
            print('{0} data shape: {1}'.format(set_name,np.shape(mv)))

            ## find init transform
            if (self.yam['is_mocap']):
                bidx = self.yam[set_name]['base_st']
                bc = mv[bidx,1:4]
                bx = mv[bidx,4:7]
                by = mv[bidx,7:10]
                unit_x = (bx - bc) / np.linalg.norm(bx - bc)
                temp_y = (by - bc) / np.linalg.norm(by - bc)
                unit_z = np.cross(unit_x, temp_y)
                unit_y = np.cross(unit_z, unit_x)
                btf_inv = np.zeros((4,4))
                btf_inv[0:3,0:3] = np.transpose(np.array([unit_x,unit_y,unit_z]))
                btf_inv[0:3,3] = -np.dot(btf_inv[0:3,0:3],bc)
                btf_inv[3,3] = 1.0
                moved_mv = np.ones((np.size(mv,0),4))
                tmp_mv = np.ones((np.size(mv,0),4))
                tmp_mv[:,0:3] = mv[:,1:4]
                for id_x in range(np.size(mv,0)):
                    moved_mv[id_x,:] = np.dot(btf_inv, tmp_mv[id_x,:])

                ## testing unit axis
                if False:
                    tmp_fig = plt.figure(i+5)
                    tmp_ax = tmp_fig.add_subplot(1,1,1,projection='3d')
                    tmp_ax.plot([0.0, unit_x[0]],[0.0, unit_x[1]],[0.0, unit_x[2]],'r')
                    tmp_ax.plot([0.0, unit_y[0]],[0.0, unit_y[1]],[0.0, unit_y[2]],'g')
                    tmp_ax.plot([0.0, unit_z[0]],[0.0, unit_z[1]],[0.0, unit_z[2]],'b')
                    self.set_axes_equal(tmp_ax)
                    print('(should be zero) X dot Y={0}'.format(round(np.dot(unit_x, unit_y),9)))
                    print('(should be zero) X dot Z={0}'.format(round(np.dot(unit_x, unit_z),9)))
                    print('(should be zero) Y dot Z={0}'.format(round(np.dot(unit_y, unit_z),9)))
                    print('base center point: {0}'.format(bc))
                    print('base center transform inverse:\n{0}'.format(btf_inv))

            ax_3 = fig_3.add_subplot(2,2,i+1)
            ax_3.plot(mv[:,1],mv[:,3])
            ax_3.axis('equal')
            ax_3.set_title(set_name)

            ax_2 = fig_2.add_subplot(2,2,i+1)
            ax_2.plot(moved_mv[:,0],moved_mv[:,2])
            ax_2.axis('equal')
            ax_2.set_title(set_name)

            ax = fig.add_subplot(2,2,i+1)
            ax.plot(moved_mv[:,0],moved_mv[:,1])
            ax.axis('equal')
            ax.set_title(set_name)

        plt.show()

    ## this function is from https://stackoverflow.com/a/31364297
    ## for making 3d plot axis 'equal'
    def set_axes_equal(self, ax):
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def calibrate(self):
        pass


    def save_data(self):
        pass

if __name__ == "__main__":
    CalibratePCV('mocap_1.yaml')


