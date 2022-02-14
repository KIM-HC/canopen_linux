#!/usr/bin/env python3
"""
Kim Hyoung Cheol
Calibrates Powered Caster Vehicle
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
#####################################################

class CalibratePCV():
    def __init__(self, yaml_path='mocap_3.yaml'):
        ## reads parameters, paths from yaml
        self.pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')
        self.out_path = self.pkg_path + '/setting/output_' + yaml_path

        with open(self.pkg_path + '/setting/' + yaml_path, 'r') as stream:
            self.yam = yaml.safe_load(stream)

        ## data made
        self.mv = [np.array([]),np.array([]),np.array([]),np.array([])]
        self.btf_inv = [[],[],[],[]]
        self.robot_rot_point = [[],[],[],[]]
        self.robot_rot_theta = [[],[],[],[]]
        self.robot_steer_point = []
        self.wheel_offset = []
        self.angle_error = []
        self.angle_phi = []
        self.wheel_radius = [0.0, 0.0, 0.0, 0.0]

        ## data given
        self.circle_start_tick = [[],[],[],[]]
        self.circle_end_tick = [[],[],[],[]]
        self.sweep_start_tick = [[],[],[],[]]
        self.sweep_end_tick = [[],[],[],[]]
        self.base_start_tick = [[],[],[],[]]
        self.base_end_tick = [[],[],[],[]]
        self.measured_steer_angle = [[],[],[],[]]
        self.jt_wheel_rot = [[],[],[],[]]
        self.steer_delta_theta = []
        self.mv_path = []
        self.jt_path = []
        self.is_mocap = self.yam['is_mocap']
        self.num_half_circle = self.yam['num_half_circle']
        self.cali_time = []
        self.mv_cali = []
        self.jt_cali = []

        for module in range(4):
            set_name = 'set_' + str(module)
            self.mv_path.append(self.pkg_path + '/data/' + self.yam[set_name]['mv_file'])
            self.jt_path.append(self.pkg_path + '/data/' + self.yam[set_name]['jt_file'])
            self.mv_cali.append(self.yam[set_name]['mv_cali'])
            self.jt_cali.append(self.yam[set_name]['jt_cali'])
            for pt in range(2):
                self.base_start_tick[module].append(self.yam[set_name][pt]['base_start'])
                self.base_end_tick[module].append(self.yam[set_name][pt]['base_end'])
                self.circle_start_tick[module].append(self.yam[set_name][pt]['circle_start'])
                self.circle_end_tick[module].append(self.yam[set_name][pt]['circle_end'])
                self.sweep_start_tick[module].append(self.yam[set_name][pt]['sweep_start'])
                self.sweep_end_tick[module].append(self.yam[set_name][pt]['sweep_end'])
                self.measured_steer_angle[module].append(self.yam[set_name][pt]['steer_angle'])
                ## [ 0 < steer_angle <= 2*pi ]
                while(True):
                    if ((2.0 * math.pi) < self.measured_steer_angle[module][pt]):
                        self.measured_steer_angle[module][pt] = self.measured_steer_angle[module][pt] - (2.0 * math.pi)
                    elif (self.measured_steer_angle[module][pt] <= 0):
                        self.measured_steer_angle[module][pt] = self.measured_steer_angle[module][pt] + (2.0 * math.pi)
                    else: break
            ## [ -pi < delta_theta <= pi ]
            delta_theta = self.measured_steer_angle[module][1] - self.measured_steer_angle[module][0]
            if (math.pi < delta_theta):
                delta_theta = delta_theta - (2.0 * math.pi)
            elif (delta_theta <= -math.pi):
                delta_theta = delta_theta + (2.0 * math.pi)
            self.steer_delta_theta.append(delta_theta)
            self.angle_phi.append((math.pi - abs(delta_theta)) * 0.5 * abs(delta_theta) / delta_theta)

        self.make_data()
        self.save_data()

        ## TODO: find wheel radius using sweep distance
        ## find distance between steer point of moving caster and robot_rot_point (dist_d)
        ## use wheel_offset of moving caster and dist_d to find
        ## distance between wheel of moving caster and stationary caster(robot_rot point)

        # self.plot_animation(module=3, pt=1, interval=1, plot_what='circle', plot_original=True)
        # self.plot_animation(module=1, pt=0)
        self.plot_data(plot_in_one=True)

    ## Time center[X Y Z] x_axis[X Y Z] y_axis[X Y Z]
    def make_data(self):
        for module in range(4):
            set_name = 'set_' + str(module)
            mv = np.loadtxt(self.mv_path[module], delimiter='\t')
            jt = np.loadtxt(self.jt_path[module], delimiter='\t')
            self.mv[module] = np.ones((np.size(mv, 0), 4))
            tmp_mv = np.ones((np.size(mv, 0), 4))
            self.mv[module][:,0:3] = mv[:,1:4]
            tmp_mv[:,0:3] = mv[:,1:4]

            ## sync jt time with mv time
            self.cali_time.append(0.0)
            for idx in range(4):
                self.cali_time[module] += (mv[self.mv_cali[module][idx],0] - jt[self.jt_cali[module][idx],0]) / 4.0
            jt[:, 0] += np.full(np.size(jt, 0), self.cali_time[module])

            ## find reference transform for each rotation -> transform each data with each reference transform
            for pt in range(2):
                tmp_rot_point = np.zeros(2)
                cst = self.circle_start_tick[module][pt]
                ced = self.circle_end_tick[module][pt]
                if (self.is_mocap):
                    half_len = int((ced - cst + 1) / self.num_half_circle)
                    for cir in range(self.num_half_circle):
                        cur_st = cst + half_len * cir
                        cur_ed = cst + half_len * (cir + 1) - 1
                        bc = mv[cur_st, 1:4]
                        bx = mv[cur_st, 4:7]
                        by = mv[cur_st, 7:10]
                        temp_x = (bx - bc) / np.linalg.norm(bx - bc)
                        temp_y = (by - bc) / np.linalg.norm(by - bc)
                        unit_z = np.cross(temp_x, temp_y) / np.linalg.norm(np.cross(temp_x, temp_y))
                        unit_y = np.cross(unit_z, temp_x) / np.linalg.norm(np.cross(unit_z, temp_x))
                        unit_x = np.cross(unit_y, unit_z) / np.linalg.norm(np.cross(unit_y, unit_z))
                        btf_inv = np.zeros((4,4))
                        btf_inv[0:3,0:3] = np.array([unit_x,unit_y,unit_z])  ## transpose of original R
                        btf_inv[0:3,3] = -np.dot(btf_inv[0:3,0:3],bc)
                        btf_inv[3,3] = 1.0
                        if (cir == 0): self.btf_inv[module].append(btf_inv)
                        
                        for tick in range(cur_st, cur_ed):
                            self.mv[module][tick,:] = np.dot(btf_inv, tmp_mv[tick,:])

                        ## find robot_rot_point
                        one_third = int((cur_ed - cur_st + 1)/3)
                        tmp_centers = np.zeros((one_third - 1, 2))
                        for tick in range(one_third - 1):
                            p1 = self.mv[module][cur_st + tick,0:2]
                            p2 = self.mv[module][cur_st + tick + one_third,0:2]
                            p3 = self.mv[module][cur_st + tick + one_third*2,0:2]
                            tmp_centers[tick,:] = self.compute_center_of_circle(p1, p2, p3)
                        test_out = KMeans(n_clusters=1).fit(tmp_centers).cluster_centers_[0]
                        tmp_rot_point = tmp_rot_point + test_out
                    ## in robot center point
                    self.robot_rot_point[module].append(tmp_rot_point / self.num_half_circle)

                else:
                    ## TODO: aruco
                    pass

                ## move points to first reference axis for convenience
                for tick in range(cst, ced+1):
                    self.mv[module][tick,:] = np.dot(self.btf_inv[module][pt], tmp_mv[tick,:])

                if False:
                    tmp_fig1 = plt.figure(module+10)
                    tmp_ax1 = tmp_fig.add_subplot(1,1,1,projection='3d')
                    tmp_ax1.plot([0.0, unit_x[0]],[0.0, unit_x[1]],[0.0, unit_x[2]],'r')
                    tmp_ax1.plot([0.0, unit_y[0]],[0.0, unit_y[1]],[0.0, unit_y[2]],'g')
                    tmp_ax1.plot([0.0, unit_z[0]],[0.0, unit_z[1]],[0.0, unit_z[2]],'b')
                    tmp_ax1.set_title('set_' + str(module) + ' axis')
                    self.set_3d_axes_equal(tmp_ax1)
                    print('(should be zero) X dot Y={0}'.format(round(np.dot(unit_x, unit_y),9)))
                    print('(should be zero) X dot Z={0}'.format(round(np.dot(unit_x, unit_z),9)))
                    print('(should be zero) Y dot Z={0}'.format(round(np.dot(unit_y, unit_z),9)))
                    print('base center point: {0}'.format(bc))
                    print('base center transform inverse:\n{0}'.format(btf_inv))

                    tmp_fig = plt.figure(5)
                    tmp_ax = tmp_fig.add_subplot(1,1,1)
                    tmp_ax.plot(self.mv[module][cst:ced+1,0],self.mv[module][cst:ced+1,1],'r')
                    tmp_ax.plot(tmp_centers[:,0],tmp_centers[:,1],'bo', markersize=1)
                    tmp_ax.plot(self.robot_rot_point[module][pt][0],self.robot_rot_point[module][pt][1],'go', markersize=5)
                    tmp_ax.set_title('set_' + str(module) + ' point_' + str(pt+1) + ' center')
                    tmp_ax.axis('equal')
                    plt.show()
                    print('center x diff: {0}'.format(abs(np.max(tmp_centers[:,0])-np.min(tmp_centers[:,0]))))
                    print('center y diff: {0}'.format(abs(np.max(tmp_centers[:,1])-np.min(tmp_centers[:,1]))))
                    print('set_{0} z len: {1}'.format(module,
                            abs(np.max(self.mv[module][cst:ced+1,2]) - np.min(self.mv[module][cst:ced+1,2]))))
                    print('set_{0} x len: {1}'.format(module,
                            abs(np.max(self.mv[module][cst:ced+1,0]) - np.min(self.mv[module][cst:ced+1,0]))))
                    print('set_{0} y len: {1}'.format(module,
                            abs(np.max(self.mv[module][cst:ced+1,1]) - np.min(self.mv[module][cst:ced+1,1]))))
                    print('set_{2}: max_z: {0}, min_z: {1}'.format(np.max(self.mv[module][cst:ced+1,2]),
                            np.min(self.mv[module][cst:ced+1,2]), module))
                    print('set_{2}: max_x: {0}, min_x: {1}'.format(np.max(self.mv[module][cst:ced+1,0]),
                            np.min(self.mv[module][cst:ced+1,0]), module))
                    print('set_{2}: max_y: {0}, min_y: {1}'.format(np.max(self.mv[module][cst:ced+1,1]),
                            np.min(self.mv[module][cst:ced+1,1]), module))

            ## offset b
            len_p1p2 = np.linalg.norm(self.robot_rot_point[module][1] - self.robot_rot_point[module][0])
            self.wheel_offset.append(len_p1p2 / (2.0 * math.sin(self.steer_delta_theta[module] / 2.0)))

            ## steer point
            uvec_p1p2 = (self.robot_rot_point[module][1] - self.robot_rot_point[module][0]) / len_p1p2
            rot_p1p2 = np.array([
                [math.cos(self.angle_phi[module]), -math.sin(self.angle_phi[module])],
                [math.sin(self.angle_phi[module]),  math.cos(self.angle_phi[module])]
            ])
            self.robot_steer_point.append(self.robot_rot_point[module][0] + self.wheel_offset[module] * np.dot(rot_p1p2, uvec_p1p2))

            ## angle_error
            vec_p1ps = self.robot_steer_point[module] - self.robot_rot_point[module][0]
            vec_p2ps = self.robot_steer_point[module] - self.robot_rot_point[module][1]
            angle_error_1 = self.measured_steer_angle[module][0] - math.atan2(vec_p1ps[1], vec_p1ps[0])
            angle_error_2 = self.measured_steer_angle[module][1] - math.atan2(vec_p2ps[1], vec_p2ps[0])
            self.angle_error.append((angle_error_1 + angle_error_2) / 2.0)

            ## sweep data
            for pt in range(2):
                sweep_start_time = mv[self.sweep_start_tick[module][pt],0]
                sweep_end_time = mv[self.sweep_end_tick[module][pt],0]
                jt_st = 0
                jt_ed = 0
                while(jt[jt_st,0] < sweep_start_time):
                    jt_st += 1
                while(jt[jt_ed,0] < sweep_end_time):
                    jt_ed += 1
                wheel_rot = []
                for wheel_mod in range(4):
                    wheel_rot.append(abs(jt[jt_st, 2 * wheel_mod + 1] - jt[jt_ed, 2 * wheel_mod + 1]))
                self.jt_wheel_rot[module].append(wheel_rot)

        ## radius of caster wheel
        for module in range(4):
            for pt in range(2):
                vec_psc1 = self.mv[module][self.sweep_start_tick[module][pt],0:2] - self.robot_steer_point[module]
                vec_psc2 = self.mv[module][self.sweep_end_tick[module][pt],0:2] - self.robot_steer_point[module]
                angle1 = math.atan2(vec_psc1[1], vec_psc1[0])
                angle2 = math.atan2(vec_psc2[1], vec_psc2[0])
                abs_rot = abs(angle1 - angle2)
                if (abs_rot > math.pi):
                    abs_rot = 2.0 * math.pi - abs_rot

                for mv_set in range(4):
                    if mv_set != module:
                        dist_d = np.linalg.norm(self.robot_rot_point[module][pt] - self.robot_steer_point[mv_set])
                        dist_w = math.sqrt(dist_d**2 - self.wheel_offset[mv_set])
                        calc_rad = dist_w * abs_rot / self.jt_wheel_rot[module][pt][mv_set]
                        self.wheel_radius[mv_set] += calc_rad / 6.0

        for module in range(4):
            print('=======\nset_{0}'.format(module))
            print('cali time: {0}'.format(self.cali_time[module]))
            for pt in range(2):
                print('P_{0}: {1}'.format(pt+1, self.robot_rot_point[module][pt]))
            print('offset b: {0}'.format(self.wheel_offset[module]))
            print('steer point: {0}'.format(self.robot_steer_point[module]))
            print('angle error beta: {0} (radian)'.format(self.angle_error[module]))
            print('angle error beta: {0} (degree)'.format(self.angle_error[module] * 180.0 / math.pi))
            print('wheel radius: {0}'.format(self.wheel_radius[module]))


    def save_data(self):
        dumper = {}
        for module in range(4):
            dumper[module] = {}
            dumper[module]['angle_error_rad'] = float(self.angle_error[module])
            dumper[module]['angle_error_deg'] = float(self.angle_error[module] * 180.0 / math.pi)
            dumper[module]['steer_point'] = self.robot_steer_point[module].tolist()
            dumper[module]['wheel_offset'] = float(self.wheel_offset[module])
            dumper[module]['wheel_radius'] = float(self.wheel_radius[module])
        with open(self.out_path, 'w') as f:
            yaml.dump(dumper, f)

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

#########################################################################################################################
#########################################################################################################################
#########################################################################################################################
    def plot_data(self, plot_in_one=False, plot_what='circle'):
        fig = plt.figure(99)
        p1 = 0
        p2 = 1
        x = 1
        y = 0
        if (plot_in_one):
            ax = fig.add_subplot(111)
            ax.plot([125, -125, -125, 125, 125], [215, 215, -215, -215, 215], 'k--', linewidth=0.8)
            ax.plot([0, 0], [0, 100], 'r', linewidth=0.8)
            ax.plot([0, 100], [0, 0], 'g', linewidth=0.8)
            plt.text(125+100, 215+20, 'set_0')
            plt.text(-125-50, 215+20, 'set_1')
            plt.text(-125-50, -215-20, 'set_2')
            plt.text(125+100, -215-20, 'set_3')
            plt.text(-125-50, -215+60, 'RED: P1', fontdict={'color':'red'})
            plt.text(-125-50, -215+30, 'BLUE: P2', fontdict={'color':'blue'})
            for module in range(4):
                cst1 = self.circle_start_tick[module][p1]
                ced1 = self.circle_end_tick[module][p1]
                cst2 = self.circle_start_tick[module][p2]
                ced2 = self.circle_end_tick[module][p2]
                if plot_what == 'sweep':
                    cst1 = self.sweep_start_tick[module][p1]
                    ced1 = self.sweep_end_tick[module][p1]
                    cst2 = self.sweep_start_tick[module][p2]
                    ced2 = self.sweep_end_tick[module][p2]
                ax.plot(self.mv[module][cst1:ced1+1,x],self.mv[module][cst1:ced1+1,y], 'r', linewidth=1.0)
                ax.plot(self.mv[module][cst2:ced2+1,x],self.mv[module][cst2:ced2+1,y], 'b', linewidth=1.0)
                ax.plot(self.robot_rot_point[module][p1][x],self.robot_rot_point[module][p1][y],'ro', markersize=5)
                ax.plot(self.robot_rot_point[module][p2][x],self.robot_rot_point[module][p2][y],'bo', markersize=5)
                ax.plot(self.robot_steer_point[module][x],self.robot_steer_point[module][y],'ko',markersize=6)
            ax.axis('equal')
            ax.invert_xaxis()

        else:
            for module in range(4):
                cst1 = self.circle_start_tick[module][p1]
                ced1 = self.circle_end_tick[module][p1]
                cst2 = self.circle_start_tick[module][p2]
                ced2 = self.circle_end_tick[module][p2]
                if plot_what == 'sweep':
                    cst1 = self.sweep_start_tick[module][p1]
                    ced1 = self.sweep_end_tick[module][p1]
                    cst2 = self.sweep_start_tick[module][p2]
                    ced2 = self.sweep_end_tick[module][p2]
                ax = fig.add_subplot(2,2,module+1)
                ax.plot([125, -125, -125, 125, 125], [215, 215, -215, -215, 215], 'k--', linewidth=0.8)
                ax.plot([0, 0], [0, 100], 'r', linewidth=0.8)
                ax.plot([0, 100], [0, 0], 'g', linewidth=0.8)
                ax.plot(self.mv[module][cst1:ced1+1,x],self.mv[module][cst1:ced1+1,y], 'r', linewidth=1.0, markersize=5)
                ax.plot(self.mv[module][cst2:ced2+1,x],self.mv[module][cst2:ced2+1,y], 'b', linewidth=1.0, markersize=5)
                ax.plot(self.robot_rot_point[module][p1][x],self.robot_rot_point[module][p1][y],'ro', markersize=5)
                ax.plot(self.robot_rot_point[module][p2][x],self.robot_rot_point[module][p2][y],'bo', markersize=5)
                ax.plot(self.robot_steer_point[module][x],self.robot_steer_point[module][y],'ko',markersize=6)
                ax.axis('equal')
                ax.invert_xaxis()
                ax.set_title('set_' + str(module))
        plt.show()

    def plot_animation(self, module, pt, interval=1, plot_what='circle', plot_original=False):
        if (plot_original):
            mv = np.loadtxt(self.mv_path[module], delimiter='\t')
            self.mv[module][:,0:3] = mv[:,1:4]
        fig = plt.figure(99)
        set_name = 'set_' + str(module)
        st = self.circle_start_tick[module][pt]
        ed = self.circle_end_tick[module][pt]
        if (plot_what == 'base'):
            st = self.base_start_tick[module][pt]
            ed = self.base_end_tick[module][pt]
        elif (plot_what == 'sweep'):
            st = self.sweep_start_tick[module][pt]
            ed = self.sweep_end_tick[module][pt]

        ax = fig.add_subplot(111)
        x, y = [], []
        line, = ax.plot([],[],'b', linewidth=1.0, markersize=5)
        line2, = ax.plot(self.mv[module][st,0],self.mv[module][st,1],'ro', linewidth=1.0, markersize=2)
        line3, = ax.plot(self.mv[module][ed,0],self.mv[module][ed,1],'go', linewidth=1.0, markersize=2)

        def update(idx):
            idx = int(idx)
            print(idx, end='')
            if (idx % 7 == 0):
                print('')
            else:
                print('   ', end='')
            if len(x) > 900:
                x.pop(0)
                y.pop(0)
            x.append(self.mv[module][idx,0])
            y.append(self.mv[module][idx,1])
            line.set_data(x, y)
            return line,

        max_x, min_x = np.max(self.mv[module][st:ed+1,0]), np.min(self.mv[module][st:ed+1,0])
        max_y, min_y = np.max(self.mv[module][st:ed+1,1]), np.min(self.mv[module][st:ed+1,1])
        mid_x, mid_y = (max_x + min_x) / 2.0, (max_y + min_y) / 2.0
        len_x, len_y = abs(max_x - min_x), abs(max_y - min_y)
        len_val = max(len_x, len_y)
        ax.set_xlim(mid_x - len_val * 0.55, mid_x + len_val * 0.55)
        ax.set_ylim(mid_y - len_val * 0.55, mid_y + len_val * 0.55)
        if (not plot_original): ax.invert_xaxis()
        ax.set_aspect('equal')
        ax.set_title(set_name)
        ani = FuncAnimation(fig, update, frames=np.linspace(st, ed, ed - st + 1), interval=interval, repeat=False)
        plt.show()
        print()

    def plot_robot_animation(self, module, pt, interval=1, plot_what='circle'):
        fig = plt.figure(99)
        set_name = 'set_' + str(module)
        mv = np.loadtxt(self.mv_path[module], delimiter='\t')
        self.mv[module][:,0:3] = mv[:,1:4]
        st = self.circle_start_tick[module][pt]
        ed = self.circle_end_tick[module][pt]
        if (plot_what == 'base'):
            st = self.base_start_tick[module][pt]
            ed = self.base_end_tick[module][pt]
        elif (plot_what == 'sweep'):
            st = self.sweep_start_tick[module][pt]
            ed = self.sweep_end_tick[module][pt]
        ax = fig.add_subplot(111)

        x, y = [], []
        line, = ax.plot([],[],'b', linewidth=2.0, markersize=5)
        line_robot_x, = ax.plot([],[],'r', linewidth=1.0, markersize=5)
        line_robot_y, = ax.plot([],[],'g', linewidth=1.0, markersize=5)
        ax.plot(self.mv[module][st:ed+1,0],self.mv[module][st:ed+1,1],'k--',linewidth=0.5)

        def update(idx):
            idx = int(idx)
            if len(x) > 200:
                x.pop(0)
                y.pop(0)
            x.append(self.mv[module][idx,0])
            y.append(self.mv[module][idx,1])
            line.set_data(x, y)
            line_robot_x.set_data(np.array([mv[idx,1],mv[idx,4]]),np.array([mv[idx,2],mv[idx,5]]))
            line_robot_y.set_data(np.array([mv[idx,1],mv[idx,7]]),np.array([mv[idx,2],mv[idx,8]]))
            return line, line_robot_x, line_robot_y

        max_x, min_x = np.max(self.mv[module][st:ed+1,0]), np.min(self.mv[module][st:ed+1,0])
        max_y, min_y = np.max(self.mv[module][st:ed+1,1]), np.min(self.mv[module][st:ed+1,1])
        mid_x, mid_y = (max_x + min_x) / 2.0, (max_y + min_y) / 2.0
        len_x, len_y = abs(max_x - min_x), abs(max_y - min_y)
        len_val = max(len_x, len_y)
        ax.set_xlim(mid_x - len_val * 0.55, mid_x + len_val * 0.55)
        ax.set_ylim(mid_y - len_val * 0.55, mid_y + len_val * 0.55)
        ax.set_aspect('equal')
        ax.set_title(set_name)
        ani = FuncAnimation(fig, update, frames=np.linspace(st, ed, ed - st + 1), interval=interval, repeat=True)
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

if __name__ == "__main__":
    CalibratePCV()


