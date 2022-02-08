#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Calibrates Powered Caster Vehicle
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""

from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans
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

        ## data made
        ## [module][point]
        self.mv = [[np.array([]),np.array([])],[np.array([]),np.array([])],
                   [np.array([]),np.array([])],[np.array([]),np.array([])]]
        self.robot_rot_point = [[],[],[],[]]
        self.robot_rot_theta = [[],[],[],[]]
        self.robot_steer_point = []
        self.offset_b = []
        self.angle_error = []

        ## data given
        self.circle_start_tick = [[],[],[],[]]
        self.circle_end_tick = [[],[],[],[]]
        self.sweep_start_tick = [[],[],[],[]]
        self.sweep_end_tick = [[],[],[],[]]
        self.base_start_tick = []
        self.base_end_tick = []
        self.cali_tick = []
        self.steer_delta_theta = []
        self.mv_path = []
        self.jt_path = []
        self.is_mocap = self.yam['is_mocap']

        for module in range(4):
            set_name = 'set_' + str(module)
            self.mv_path.append(self.pkg_path + '/data/' + self.yam[set_name]['mv_file'])
            self.jt_path.append(self.pkg_path + '/data/' + self.yam[set_name]['jt_file'])
            self.base_start_tick.append(self.yam[set_name]['base_start'])
            self.base_end_tick.append(self.yam[set_name]['base_end'])
            self.cali_tick.append(self.yam[set_name]['cali_tick'])
            self.steer_delta_theta.append(self.yam[set_name]['steer_delta_theta'])
            for pt in range(2):
                self.circle_start_tick[module].append(self.yam[set_name][pt]['circle_start'])
                self.circle_end_tick[module].append(self.yam[set_name][pt]['circle_end'])
                self.sweep_start_tick[module].append(self.yam[set_name][pt]['sweep_start'])
                self.sweep_end_tick[module].append(self.yam[set_name][pt]['sweep_end'])

        self.make_data()

        ## TODO: find p1, p2 point

        ## TODO: find offset b

        ## TODO: find steer point of stationary caster

        ## TODO: find angle error

        ## TODO: find wheel radius using sweep distance
        ## find distance between steer point of moving caster and robot_rot_point (dist_d)
        ## use offset_b of moving caster and dist_d to find
        ## distance between wheel of moving caster and stationary caster(robot_rot point)



        # self.plot_animation(module=3, pt=0)
        # self.plot_animation(module=0, pt=1)
        # self.plot_data()

    ## Time center[X Y Z] x_axis[X Y Z] y_axis[X Y Z]
    def make_data(self):
        for module in range(4):
            z_tot = np.zeros(3)
            unit_z = np.zeros(3)
            set_name = 'set_' + str(module)
            mv = np.loadtxt(self.mv_path[module], delimiter='\t')
            print('{0} data shape: {1}'.format(set_name,np.shape(mv)))
            bst = self.base_start_tick[module]
            bed = self.base_end_tick[module]

            ## find plane vector
            ## TODO: use points when it is not moving (base_start_tick base_end_tick)
            for pt in range(2):
                cst = self.circle_start_tick[module][pt]
                ced = self.circle_end_tick[module][pt]
                if (self.is_mocap):
                    for index in range(cst, ced + 1):
                        tc = mv[index,1:4]
                        tx = mv[index,4:7]
                        ty = mv[index,7:10]
                        tmp_z = np.cross((tx - tc), (ty - tc))
                        z_tot += tmp_z / np.linalg.norm(tmp_z)
                else:
                    ## TODO: aruco
                    pass
            unit_z = z_tot / np.linalg.norm(z_tot)

            ## move points to reference axis
            for pt in range(2):
                cst = self.circle_start_tick[module][pt]
                ced = self.circle_end_tick[module][pt]
                num_data = ced - cst + 1
                self.mv[module][pt] = np.ones((num_data,4))

                if (self.is_mocap):
                    bc = mv[bst,1:4]
                    bx = mv[bst,4:7]
                    temp_x = (bx - bc) / np.linalg.norm(bx - bc)
                    unit_y = np.cross(unit_z, temp_x) / np.linalg.norm(np.cross(unit_z, temp_x))
                    unit_x = np.cross(unit_y, unit_z) / np.linalg.norm(np.cross(unit_y, unit_z))

                    btf_inv = np.zeros((4,4))
                    btf_inv[0:3,0:3] = np.transpose(np.array([unit_x,unit_y,unit_z]))
                    btf_inv[0:3,3] = -np.dot(btf_inv[0:3,0:3],bc)
                    btf_inv[3,3] = 1.0

                    by = mv[bst,7:10]
                    test_y = (by - bc) / np.linalg.norm(by - bc)
                    test_z = np.cross(temp_x, test_y) / np.linalg.norm(np.cross(temp_x, test_y))
                    print('unit_z: {0}'.format(unit_z))
                    print('test_z: {0}'.format(test_z))


                else:
                    ## TODO: aruco
                    pass

                tmp_mv = np.ones((num_data,4))
                tmp_mv[:,0:3] = mv[cst:ced + 1, 1:4]
                for id_x in range(num_data):
                    self.mv[module][pt][id_x,:] = np.dot(btf_inv, tmp_mv[id_x,:])
                    ## testing unit axis
                    if False:
                        tmp_fig = plt.figure(module+5)
                        tmp_ax = tmp_fig.add_subplot(1,1,1,projection='3d')
                        tmp_ax.plot([0.0, unit_x[0]],[0.0, unit_x[1]],[0.0, unit_x[2]],'r')
                        tmp_ax.plot([0.0, unit_y[0]],[0.0, unit_y[1]],[0.0, unit_y[2]],'g')
                        tmp_ax.plot([0.0, unit_z[0]],[0.0, unit_z[1]],[0.0, unit_z[2]],'b')
                        tmp_ax.set_title('set_' + str(module) + ' axis')
                        self.set_3d_axes_equal(tmp_ax)
                        print('(should be zero) X dot Y={0}'.format(round(np.dot(unit_x, unit_y),9)))
                        print('(should be zero) X dot Z={0}'.format(round(np.dot(unit_x, unit_z),9)))
                        print('(should be zero) Y dot Z={0}'.format(round(np.dot(unit_y, unit_z),9)))
                        print('base center point: {0}'.format(bc))
                        print('base center transform inverse:\n{0}'.format(btf_inv))
                
                ## find robot_rot_point
                one_third = int(num_data/3)
                tmp_center = np.zeros((one_third - 1, 2))
                for tick in range(one_third - 1):
                    p1 = self.mv[module][pt][tick,0:2]
                    p2 = self.mv[module][pt][tick + one_third,0:2]
                    p3 = self.mv[module][pt][tick + one_third*2,0:2]
                    tmp_center[tick,:] = self.compute_center_of_circle(p1, p2, p3)

                tmp_kmeans = KMeans(n_clusters=1)
                tmp_kmeans.fit(tmp_center)
                tmp_centers = tmp_kmeans.cluster_centers_[0]

                if False:
                    tmp_fig = plt.figure(5)
                    tmp_ax = tmp_fig.add_subplot(1,1,1)
                    tmp_ax.plot(self.mv[module][pt][:,0],self.mv[module][pt][:,1],'r')
                    tmp_ax.plot(tmp_center[:,0],tmp_center[:,1],'bo', markersize=1)
                    tmp_ax.set_title('set_' + str(module) + ' point_' + str(pt+1) + ' center')
                    tmp_ax.axis('equal')
                    plt.show()
                    print('center x diff: {0}'.format(abs(np.max(tmp_center[:,0])-np.min(tmp_center[:,0]))))
                    print('center y diff: {0}'.format(abs(np.max(tmp_center[:,1])-np.min(tmp_center[:,1]))))
                    print('set_{0} z len: {1}'.format(module,
                            abs(np.max(self.mv[module][pt][:,2]) - np.min(self.mv[module][pt][:,2]))))
                    print('set_{0} x len: {1}'.format(module,
                            abs(np.max(self.mv[module][pt][:,0]) - np.min(self.mv[module][pt][:,0]))))
                    print('set_{0} y len: {1}'.format(module,
                            abs(np.max(self.mv[module][pt][:,1]) - np.min(self.mv[module][pt][:,1]))))
                    print('set_{2}: max_z: {0}, min_z: {1}'.format(np.max(self.mv[module][pt][:,2]),
                            np.min(self.mv[module][pt][:,2]), module))
                    print('set_{2}: max_x: {0}, min_x: {1}'.format(np.max(self.mv[module][pt][:,0]),
                            np.min(self.mv[module][pt][:,0]), module))
                    print('set_{2}: max_y: {0}, min_y: {1}'.format(np.max(self.mv[module][pt][:,1]),
                            np.min(self.mv[module][pt][:,1]), module))

    ## from https://mathbang.net/455
    def compute_center_of_circle(self, p1, p2, p3):
        x = 0
        y = 1
        numerator_a = (p2[x]**2 + p2[y]**2 - p1[x]**2 - p1[y]**2) * (p1[y] - p3[y]) - (p3[x]**2 + p3[y]**2 - p1[x]**2 - p1[y]**2) * (p1[y] - p2[y])
        denominator_a = (p1[x] - p2[x]) * (p1[y] - p3[y]) - (p1[x] - p3[x]) * (p1[y] - p2[y])
        numerator_b = (p2[x]**2 + p2[y]**2 - p1[x]**2 - p1[y]**2) * (p1[x] - p3[x]) - (p3[x]**2 + p3[y]**2 - p1[x]**2 - p1[y]**2) * (p1[x] - p2[x])
        denominator_b = (p1[y] - p2[y]) * (p1[x] - p3[x]) - (p1[y] - p3[y]) * (p1[x] - p2[x])
        param_a = numerator_a / denominator_a
        param_b = numerator_b / denominator_b
        center_point = np.array([-param_a/2.0, -param_b/2.0])
        return center_point

    def plot_data(self):
        fig = plt.figure(99)
        for module in range(4):
            ax = fig.add_subplot(2,2,module+1)
            ax.plot(self.mv[module][0][:,0],self.mv[module][0][:,1], 'r', linewidth=1.0, markersize=5)
            ax.plot(self.mv[module][1][:,0],self.mv[module][1][:,1], 'b', linewidth=1.0, markersize=5)
            ax.axis('equal')
            ax.set_title('set_' + str(module))
        plt.show()

    def plot_animation(self, module, pt, plot_what='circle', interval=1):
        fig = plt.figure(99)
        set_name = 'set_' + str(module)
        st = self.circle_start_tick[module][pt]
        ed = self.circle_end_tick[module][pt]
        if (plot_what == 'base'):
            st = self.base_start_tick[module]
            ed = self.base_end_tick[module]

        ax = fig.add_subplot(111)
        x, y = [], []
        line, = ax.plot([],[],'b', linewidth=1.0, markersize=5)

        def update(idx):
            idx = int(idx)
            print(idx + st)
            x.append(self.mv[module][pt][idx,0])
            y.append(self.mv[module][pt][idx,1])
            line.set_data(x, y)
            return line,

        max_x, min_x = np.max(self.mv[module][pt][:,0]), np.min(self.mv[module][pt][:,0])
        max_y, min_y = np.max(self.mv[module][pt][:,1]), np.min(self.mv[module][pt][:,1])
        len_x, len_y = abs(max_x - min_x), abs(max_y - min_y)
        len_val = max(len_x, len_y)
        ax.set_xlim(min_x - len_val * 0.05, min_x + len_val * 1.05)
        ax.set_ylim(min_y - len_val * 0.05, min_y + len_val * 1.05)
        ax.set_aspect('equal')
        ax.set_title(set_name)
        ani = FuncAnimation(fig, update, frames=np.linspace(0, ed - st, ed - st + 1), interval=interval, repeat=False)
        plt.show()

    ## this function is from https://stackoverflow.com/a/31364297
    ## for making 3d plot axis 'equal'
    def set_3d_axes_equal(self, ax):
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


