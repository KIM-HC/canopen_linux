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

class CalibratePCV():
    def __init__(self, yaml_path='mocap_33.yaml'):
        ## reads parameters, paths from yaml
        print('=== {0} ==='.format(yaml_path))
        self.pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')
        self.out_path = self.pkg_path + '/setting/output_' + yaml_path

        with open(self.pkg_path + '/setting/' + yaml_path, 'r') as stream:
            self.yam = yaml.safe_load(stream)
        with open(self.pkg_path + '/setting/' + self.yam['basic_file'], 'r') as stream:
            yam_bas = yaml.safe_load(stream)

        ## DEBUGGING
        self.debug_rot_point = True
        self.debug_rot_point_section = False
        self.debug_rot_point_plot = False
        self.debug_axis_generation = False
        self.debug_rot_point_reult = [[[],[]],[[],[]],[[],[]],[[],[]]]
        self.plt_pos_map = ['y','r','g','b','k','m','k']
        self.plt_ori_map = ['s','^','o','*','+','x','2']
        self.test_error = 0             ## for 3 points on circle
        self.error_checker = 0.000001   ## for 3 points on circle
        self.plot_num = 1               ## for plotting ellipse fitting
        self.plot_num_p = 1             ## for plotting center in each point's frame - one section
        self.plot_num_pp = 1            ## for plotting center in each point's frame - all section
        self.debug_wheel_radius = False
        self.debug_wheel_radius_result = [[],[],[],[]]
        self.debug_wheel_radius_list = [20.0, 30.0, 40.0, 60.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0]

        ## METHODS
        ## 0: 3 points on circle
        ## 1: NUMERICALLY STABLE DIRECT LEAST SQUARES FITTING OF ELLIPSES
        ##    http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1.7559&rep=rep1&type=pdf
        ##    https://github.com/bdhammel/least-squares-ellipse-fitting
        self.circle_fitting_method = 0
        print('=== CIRCLE FITTING METHOD: {0}'.format(self.circle_fitting_method))

        ## 0: plot all sections in each sub_plot
        ## 1: plot all in one
        self.section_plot_method = 1

        ## 0: both had no difference in content,
        ## 1: just a different way of coding
        self.three_point_method = 1

        ## data made
        self.mv = [np.array([]),np.array([]),np.array([]),np.array([])]
        self.eqmv = [np.array([]),np.array([]),np.array([]),np.array([])]
        self.jt = [np.array([]),np.array([]),np.array([]),np.array([])]
        self.wheel_radius = [0.0, 0.0, 0.0, 0.0]
        self.circle_start_tick = [[],[],[],[]]
        self.sweep_start_tick = [[],[],[],[]]
        self.circle_end_tick = [[],[],[],[]]
        self.robot_rot_point = [[],[],[],[]]  ## [module][pt]
        self.robot_rot_theta = [[],[],[],[]]  ## [module][pt]
        self.wheel_rot_theta = [[],[],[],[]]  ## [module][pt][module_other]
        self.sweep_end_tick = [[],[],[],[]]
        self.btf_inv = [[],[],[],[]]          ## [module][pt]
        self.robot_steer_point = []  ## [module]
        self.wheel_offset = []       ## [module]
        self.angle_error = []        ## [module]
        self.angle_phi = []          ## [module]

        ## data given
        self.measured_steer_angle = [[],[],[],[]]
        self.circle_start_time = [[],[],[],[]]
        self.circle_end_time = [[],[],[],[]]
        self.steer_delta_theta = []
        self.cali_time = []
        self.eqmv_path = []
        self.mv_path = []
        self.jt_path = []
        self.mv_cali = []
        self.jt_cali = []
        self.num_section = self.yam['num_section']
        self.sweep_angle = self.yam['sweep_angle'] * math.pi / 180.0
        self.is_mocap = yam_bas['is_mocap']
        loop_index = self.yam['loop_index']

        for module in range(4):
            set_name = 'set_' + str(module)
            self.mv_path.append(self.pkg_path + '/data/' + yam_bas[set_name]['mv_file'])
            self.eqmv_path.append(self.pkg_path + '/data/' + yam_bas[set_name]['equal_mv_file'])
            self.jt_path.append(self.pkg_path + '/data/' + yam_bas[set_name]['jt_file'])
            self.mv_cali.append(yam_bas[set_name]['mv_cali'])
            self.jt_cali.append(yam_bas[set_name]['jt_cali'])

            for pt in range(2):
                self.circle_start_time[module].append(yam_bas['loop_index'][loop_index][module][pt][0])
                self.circle_end_time[module].append(yam_bas['loop_index'][loop_index][module][pt][1])
                self.measured_steer_angle[module].append(yam_bas[set_name]['steer_angle'][pt])
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

        # self.plot_animation(module=3, pt=1, interval=1, plot_what='circle', plot_original=True)
        # self.plot_animation(module=1, pt=0)
        # self.plot_data(plot_in_one=True, plot_what='circle')


    def make_data(self):
        for module in range(4):
            self.eqmv[module] = np.loadtxt(self.eqmv_path[module], delimiter='\t')
            mv_for_time_cali = np.loadtxt(self.mv_path[module], delimiter='\t') ## DONT USE MV
            jt = np.loadtxt(self.jt_path[module], delimiter='\t')
            self.mv[module] = np.ones((np.size(self.eqmv[module], 0), 4))
            self.mv[module][:,0:3] = self.eqmv[module][:,1:4]
            ## tmp_mv(1X4) gets only center point with 4th index in ones
            ## for multiplication with transform matrix(4X4)
            tmp_mv = np.ones((np.size(self.eqmv[module], 0), 4))
            tmp_mv[:,0:3] = self.eqmv[module][:,1:4]

            ## sync jt time with mv time
            self.cali_time.append(0.0)
            tot_cali_index = len(self.mv_cali[module])
            for idx in range(tot_cali_index):
                self.cali_time[module] += (mv_for_time_cali[self.mv_cali[module][idx],0] - jt[self.jt_cali[module][idx],0]) / tot_cali_index
            jt[:, 0] += np.full(np.size(jt, 0), self.cali_time[module])
            self.jt[module] = jt

            ## find reference transform for each rotation -> transform each data with each reference transform
            for pt in range(2):
                tmp_rot_point = np.zeros(2)
                tmp_rot_point_list = []
                tmp_centers_list = []
                ## find circle start and end tick(index) from time
                cst = 0
                while(self.eqmv[module][cst,0] < self.circle_start_time[module][pt]):
                    cst += 1
                ced = cst
                while(self.eqmv[module][ced,0] < self.circle_end_time[module][pt]):
                    ced += 1
                self.circle_start_tick[module].append(cst)
                self.sweep_start_tick[module].append(cst)
                self.circle_end_tick[module].append(ced)
                if (self.is_mocap):
                    ## index include 'ced' and 'cst'
                    section_length = int((ced - cst + 1) / self.num_section)
                    for cir in range(self.num_section):
                        ## index include 'cur_st' and 'cur_ed'
                        cur_st = cst + section_length * cir
                        cur_ed = cst + section_length * (cir + 1) - 1
                        btf_inv = self.btf_inv_from_idx(module=module, index=cur_st)
                        ## use first reference
                        if (cir == 0): self.btf_inv[module].append(btf_inv)

                        # ## self.mv: viewed in current section's reference axis
                        # for tick in range(cur_st, cur_ed + 1):
                        #     self.mv[module][tick,:] = np.dot(btf_inv, tmp_mv[tick,:])
                        ## move all points to current reference axis for testing
                        for tick in range(cst, ced+1):
                            self.mv[module][tick,:] = np.dot(btf_inv, tmp_mv[tick,:])

                        ## find robot_rot_point
                        if self.circle_fitting_method == 0:
                            one_third = int((cur_ed - cur_st + 1)/3)
                            tmp_centers = np.zeros((one_third, 2))
                            for tick in range(one_third):
                                p1 = self.mv[module][cur_st + tick + one_third * 0, 0:2]
                                p2 = self.mv[module][cur_st + tick + one_third * 1, 0:2]
                                p3 = self.mv[module][cur_st + tick + one_third * 2, 0:2]
                                tmp_centers[tick,:] = self.compute_center_of_circle(p1, p2, p3)
                            test_out = KMeans(n_clusters=1).fit(tmp_centers).cluster_centers_[0]
                        elif self.circle_fitting_method == 1:
                            tmp_centers = np.zeros((1, 2))
                            lsqe = LsqEllipse()
                            lsqe.fit(self.mv[module][cur_st:cur_ed+1, 0:2])
                            center, width, height, phi = lsqe.as_parameters()
                            axis_x = np.array([ math.cos(phi), math.sin(phi)])  ## width
                            axis_y = np.array([-math.sin(phi), math.cos(phi)])  ## height
                            center = np.array(center)
                            pnts = []
                            pnts.append(center + width  * axis_x)
                            pnts.append(center - width  * axis_x)
                            pnts.append(center + height * axis_y)
                            pnts.append(center - height * axis_y)

                            idxs = [0, 0, 0, 0]
                            mins = [9999, 9999, 9999, 9999]

                            for ii in range(cur_st, cur_ed + 1):
                                cur_p = self.mv[module][ii, 0:2]
                                for pp in range(4):
                                    dist = np.linalg.norm(cur_p - pnts[pp])
                                    if dist < mins[pp]:
                                        idxs[pp] = ii
                                        mins[pp] = dist
                            # print('+width index:  {0}'.format(idxs[0]))
                            # print('-width index:  {0}'.format(idxs[1]))
                            # print('+height index: {0}'.format(idxs[2]))
                            # print('-height index: {0}'.format(idxs[3]))

                            if (self.plot_num > 0):
                                print('center: {0}, {1}'.format(center[0],center[1]))
                                print('width: {0}'.format(width))
                                print('height: {0}'.format(height))
                                print('phi: {0} RAD'.format(phi))
                                print('phi: {0} DEG'.format(phi * 180.0 / math.pi))
                                self.plot_num -= 1
                                fig = plt.figure(figsize=(6,6))
                                ax = fig.add_subplot(111)
                                ax.axis('equal')
                                ax.plot(self.mv[module][cst:ced+1, 0], self.mv[module][cst:ced+1, 1], 'm-',
                                        label='total', zorder=1, markersize=1, linewidth=1)
                                # ax.plot(self.mv[module][cur_st:cur_ed+1, 0], self.mv[module][cur_st:cur_ed+1, 1], 'r-',
                                #         label='test data', zorder=1, markersize=1, linewidth=1)
                                ax.plot([center[0], center[0] + axis_x[0]*200.0], [center[1], center[1] + axis_x[1]*200.0], 'r', linewidth=2, label='Width')
                                ax.plot([center[0], center[0] + axis_y[0]*200.0], [center[1], center[1] + axis_y[1]*200.0], 'g', linewidth=2, label='Height')
                                for pp in range(4):
                                    ax.plot(self.mv[module][idxs[pp],0], self.mv[module][idxs[pp],1], 'yo', markersize=3)
                                    # ax.plot(pnts[pp][0], pnts[pp][1], 'ko', markersize=3)

                                ellipse = Ellipse(xy=center, width=2*width, height=2*height, angle=np.rad2deg(phi),
                                            edgecolor='b', fc='None', lw=1, label='Fit', zorder = 2)
                                ax.add_patch(ellipse)
                                plt.legend()
                                plt.show()
                            test_out = np.array(center)
                        
                        if self.debug_rot_point:
                            print('=== set_' + str(module) + ' pt_' + str(pt+1) + ' section_' + str(cir+1) + ' center test ===')
                            self.debug_rot_point_reult[module][pt].append([])
                            og_p = np.dot(self.btf_from_idx(module=module, index=cur_st), np.array([test_out[0], test_out[1], 0.0, 1.0]))
                            for pp in range(cur_st, cur_ed + 1):
                                btf_inv_new = self.btf_inv_from_idx(module=module, index=pp)
                                new_p = np.dot(btf_inv_new, og_p)
                                self.debug_rot_point_reult[module][pt][cir].append(new_p[0:2])
                            self.debug_rot_point_reult[module][pt][cir] = np.array(self.debug_rot_point_reult[module][pt][cir])
                            print('center x diff: {0}'.format(abs(np.max(self.debug_rot_point_reult[module][pt][cir][:,0])-np.min(self.debug_rot_point_reult[module][pt][cir][:,0]))))
                            print('center y diff: {0}'.format(abs(np.max(self.debug_rot_point_reult[module][pt][cir][:,1])-np.min(self.debug_rot_point_reult[module][pt][cir][:,1]))))

                            if self.plot_num_p > 0:
                                self.plot_num_p -= 1
                                tmp_fig3 = plt.figure(17)
                                tmp_ax = tmp_fig3.add_subplot(111)
                                tmp_ax.plot(self.debug_rot_point_reult[module][pt][cir][:,0], self.debug_rot_point_reult[module][pt][cir][:,0],
                                            'ro', markersize=3)
                                tmp_ax.set_title('set_' + str(module) + ' pt_' + str(pt+1) + ' section_' + str(cir+1) + ' center')
                                tmp_ax.axis('equal')
                                plt.show()

                        tmp_rot_point = tmp_rot_point + test_out
                        if self.debug_rot_point:
                            tmp_centers_list.append(tmp_centers)
                            tmp_rot_point_list.append(test_out)
                            if self.debug_rot_point_section:
                                print('=== set_' + str(module) + ' pt_' + str(pt+1) + ' section_' + str(cir+1) + ' center test ===')
                                print('center x diff: {0}'.format(abs(np.max(tmp_centers[:,0])-np.min(tmp_centers[:,0]))))
                                print('center y diff: {0}'.format(abs(np.max(tmp_centers[:,1])-np.min(tmp_centers[:,1]))))
                    if self.plot_num_pp > 0:
                        self.plot_num_pp -= 1
                        tmp_fig3 = plt.figure(17)
                        tmp_ax = tmp_fig3.add_subplot(111)
                        for cir in range(self.num_section):
                            tmp_ax.plot(self.debug_rot_point_reult[module][pt][cir][:,0], self.debug_rot_point_reult[module][pt][cir][:,0],
                                        self.plt_pos_map[cir%5]+'o', markersize=1)
                        tmp_ax.set_title('set_' + str(module) + ' pt_' + str(pt+1) + ' centers')
                        tmp_ax.axis('equal')
                        plt.show()

                    ## in robot center point
                    self.robot_rot_point[module].append(tmp_rot_point / self.num_section)

                    if self.debug_rot_point:
                        tmp_rot_point_list = np.array(tmp_rot_point_list)
                        print('=== set_' + str(module) + ' pt_' + str(pt+1) + ' center test ===')
                        print('center x diff: {0}'.format(abs(np.max(tmp_rot_point_list[:,0])-np.min(tmp_rot_point_list[:,0]))))
                        print('center y diff: {0}'.format(abs(np.max(tmp_rot_point_list[:,1])-np.min(tmp_rot_point_list[:,1]))))
                        if self.debug_rot_point_plot:
                            if self.section_plot_method == 0:
                                if self.debug_rot_point_section:
                                    tmp_fig1 = plt.figure(13)
                                    mm = 1
                                    nn = 1
                                    while(True):
                                        if (nn * mm < self.num_section):
                                            nn += 1
                                        else: break
                                        if (nn * mm < self.num_section):
                                            mm += 1
                                        else: break
                                    for cir in range(self.num_section):
                                        tmp_ax1 = tmp_fig1.add_subplot(mm, nn, cir+1)
                                        tmp_ax1.plot(tmp_centers_list[cir][:,0], tmp_centers_list[cir][:,1], 'bo', markersize=1)
                                        tmp_ax1.plot(tmp_rot_point_list[cir,0], tmp_rot_point_list[cir,1], 'ro', markersize=5)
                                        tmp_ax1.set_title('set_' + str(module) + ' pt_' + str(pt+1) + ' section_' + str(cir+1) + ' center')
                                        tmp_ax1.axis('equal')

                                tmp_fig = plt.figure(10)
                                tmp_ax = tmp_fig.add_subplot(111)
                                tmp_ax.plot(tmp_rot_point_list[:,0], tmp_rot_point_list[:,1], 'bo', markersize=1)
                                tmp_ax.plot(self.robot_rot_point[module][pt][0], self.robot_rot_point[module][pt][1], 'ro', markersize=5)
                                tmp_ax.set_title('set_' + str(module) + ' pt_' + str(pt+1) + ' center')
                                tmp_ax.axis('equal')
                            elif self.section_plot_method == 1:
                                tmp_rot_point_list = np.array(tmp_rot_point_list)
                                tmp_fig = plt.figure(10)
                                tmp_ax = tmp_fig.add_subplot(111)

                                for cir in range(self.num_section):
                                    if self.debug_rot_point_section:
                                        tmp_ax.plot(tmp_centers_list[cir][:,0], tmp_centers_list[cir][:,1],
                                                    self.plt_pos_map[cir%5+1]+'o', markersize=1)
                                    tmp_ax.plot(tmp_rot_point_list[cir,0], tmp_rot_point_list[cir,1],
                                                self.plt_pos_map[cir%5+1]+'x', markersize=3)

                                tmp_ax.plot(self.robot_rot_point[module][pt][0], self.robot_rot_point[module][pt][1], 'y*', markersize=5)
                                tmp_ax.set_title('set_' + str(module) + ' pt_' + str(pt+1) + ' center')
                                tmp_ax.axis('equal')
                            plt.show()

                else:
                    ## TODO: aruco
                    pass

                ## move points to first reference axis for convenience
                for tick in range(cst, ced+1):
                    self.mv[module][tick,:] = np.dot(self.btf_inv[module][pt], tmp_mv[tick,:])

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

            ## angle_error (measured(ESTIMATED) - calculated(TRUE))
            vec_p1ps = self.robot_steer_point[module] - self.robot_rot_point[module][0]
            vec_p2ps = self.robot_steer_point[module] - self.robot_rot_point[module][1]
            angle_error_1 = self.measured_steer_angle[module][0] - math.atan2(vec_p1ps[1], vec_p1ps[0])
            angle_error_2 = self.measured_steer_angle[module][1] - math.atan2(vec_p2ps[1], vec_p2ps[0])
            self.angle_error.append((angle_error_1 + angle_error_2) / 2.0)

            ## make sweep data: robot_rot & wheel_rot
            for pt in range(2):
                ## find sweep st,ed index with sweep_angle
                mv_st = self.circle_start_tick[module][pt]
                mv_ed = mv_st
                vec_start = self.mv[module][mv_st,0:2] - self.robot_steer_point[module]
                angle_start = math.atan2(vec_start[1], vec_start[0])
                while(True):
                    vec_current = self.mv[module][mv_ed,0:2] - self.robot_steer_point[module]
                    angle_current = math.atan2(vec_current[1], vec_current[0])
                    abs_rot = abs(angle_start - angle_current)
                    if (abs_rot > math.pi):
                        abs_rot = 2.0 * math.pi - abs_rot
                    if (abs_rot >= self.sweep_angle):
                        self.robot_rot_theta[module].append(abs_rot)
                        break
                    mv_ed += 1
                self.sweep_end_tick[module].append(mv_ed)
                jt_st = 0
                while(jt[jt_st,0] < self.eqmv[module][mv_st,0]):
                    jt_st += 1
                jt_ed = jt_st
                while(jt[jt_ed,0] < self.eqmv[module][mv_ed,0]):
                    jt_ed += 1
                wheel_rot = []
                for wheel_mod in range(4):
                    wheel_rot.append(abs(jt[jt_st, 2 * wheel_mod + 1] - jt[jt_ed, 2 * wheel_mod + 1]))
                self.wheel_rot_theta[module].append(wheel_rot)

        ## radius of caster wheel
        for module in range(4):
            for pt in range(2):
                robot_rot = self.robot_rot_theta[module][pt]
                for mv_set in range(4):
                    if mv_set != module:
                        wheel_rot = self.wheel_rot_theta[module][pt][mv_set]
                        dist_d = np.linalg.norm(self.robot_rot_point[module][pt] - self.robot_steer_point[mv_set])
                        dist_w = math.sqrt(dist_d**2 - self.wheel_offset[mv_set])
                        calc_rad = dist_w * robot_rot / wheel_rot
                        self.wheel_radius[mv_set] += calc_rad / 6.0

        if self.debug_wheel_radius:
            for swp in range(len(self.debug_wheel_radius_list)):
                sweep_angle_db = self.debug_wheel_radius_list[swp] * math.pi / 180.0
                robot_rot_db = [[0,0],[0,0],[0,0],[0,0]]
                wheel_rot_db = [[[0,0,0,0],[0,0,0,0]],[[0,0,0,0],[0,0,0,0]],[[0,0,0,0],[0,0,0,0]],[[0,0,0,0],[0,0,0,0]]]
                for module in range(4):
                    self.debug_wheel_radius_result[module].append(0.0)
                    ## make sweep data: robot_rot & wheel_rot
                    for pt in range(2):
                        ## find sweep st,ed index with sweep_angle
                        mv_st = self.circle_start_tick[module][pt]
                        mv_ed = mv_st
                        vec_start = self.mv[module][mv_st,0:2] - self.robot_steer_point[module]
                        angle_start = math.atan2(vec_start[1], vec_start[0])
                        while(True):
                            vec_current = self.mv[module][mv_ed,0:2] - self.robot_steer_point[module]
                            angle_current = math.atan2(vec_current[1], vec_current[0])
                            abs_rot = abs(angle_start - angle_current)
                            if (abs_rot > math.pi):
                                abs_rot = 2.0 * math.pi - abs_rot
                            if (abs_rot >= sweep_angle_db):
                                robot_rot_db[module][pt] = abs_rot
                                break
                            mv_ed += 1
                        jt_st = 0
                        while(self.jt[module][jt_st,0] < self.eqmv[module][mv_st,0]):
                            jt_st += 1
                        jt_ed = jt_st
                        while(self.jt[module][jt_ed,0] < self.eqmv[module][mv_ed,0]):
                            jt_ed += 1
                        for wheel_mod in range(4):
                            wheel_rot_db[module][pt][wheel_mod] = abs(self.jt[module][jt_st, 2 * wheel_mod + 1] - self.jt[module][jt_ed, 2 * wheel_mod + 1])
                for module in range(4):
                    for pt in range(2):
                        for mv_set in range(4):
                            if mv_set != module:
                                dist_d = np.linalg.norm(self.robot_rot_point[module][pt] - self.robot_steer_point[mv_set])
                                dist_w = math.sqrt(dist_d**2 - self.wheel_offset[mv_set])
                                calc_rad = dist_w * robot_rot_db[module][pt] / wheel_rot_db[module][pt][mv_set]
                                self.debug_wheel_radius_result[mv_set][swp] += calc_rad / 6.0


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
            if self.debug_wheel_radius:
                print('==== sweep angle test ====')
                print('== min max diff: {0}'.format(np.max(self.debug_wheel_radius_result[module]) - np.min(self.debug_wheel_radius_result[module])))
                for swp in range(len(self.debug_wheel_radius_list)):
                    print('{0} DEG: {1}'.format(self.debug_wheel_radius_list[swp], self.debug_wheel_radius_result[module][swp]))

    def save_data(self):
        dumper = {}
        for module in range(4):
            dumper[module] = {}
            dumper[module]['angle_error_rad'] = float(self.angle_error[module])
            dumper[module]['angle_error_deg'] = float(self.angle_error[module] * 180.0 / math.pi)
            self.robot_steer_point[module] /= 1000.0
            dumper[module]['steer_point'] = self.robot_steer_point[module].tolist()
            ## negative value for offset
            dumper[module]['wheel_offset'] = float(self.wheel_offset[module]) / -1000.0
            dumper[module]['wheel_radius'] = float(self.wheel_radius[module]) / 1000.0
        with open(self.out_path, 'w') as f:
            yaml.dump(dumper, f)

    def compute_center_of_circle(self, p1, p2, p3):
        if self.three_point_method == 0:
            ## from https://mathbang.net/455
            x = 0
            y = 1
            numerator_a = (p2[x]**2 + p2[y]**2 - p1[x]**2 - p1[y]**2) * (p1[y] - p3[y]) - (p3[x]**2 + p3[y]**2 - p1[x]**2 - p1[y]**2) * (p1[y] - p2[y])
            denominator_a = (p1[x] - p2[x]) * (p1[y] - p3[y]) - (p1[x] - p3[x]) * (p1[y] - p2[y])
            numerator_b = (p2[x]**2 + p2[y]**2 - p1[x]**2 - p1[y]**2) * (p1[x] - p3[x]) - (p3[x]**2 + p3[y]**2 - p1[x]**2 - p1[y]**2) * (p1[x] - p2[x])
            denominator_b = (p1[y] - p2[y]) * (p1[x] - p3[x]) - (p1[y] - p3[y]) * (p1[x] - p2[x])
            param_a = numerator_a / denominator_a
            param_b = numerator_b / denominator_b
            if (denominator_a < self.error_checker or denominator_b < self.error_checker):
                self.test_error += 1
            center_point = np.array([-param_a/2.0, -param_b/2.0])
        elif self.three_point_method == 1:
            """
            Returns the center and radius of the circle passing the given 3 points.
            In case the 3 points form a line, returns (None, infinity).
            """
            ## https://stackoverflow.com/questions/28910718/give-3-points-and-a-plot-circle
            temp = p2[0] * p2[0] + p2[1] * p2[1]
            bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
            cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
            det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

            if abs(det) < 1.0e-6:
                print('is a line')
                return None

            # Center of circle
            cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
            cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

            radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)

            center_point = np.array([cx, cy])

        return center_point

    def btf_inv_from_idx(self, module, index):
        bc = self.eqmv[module][index, 1:4]
        bx = self.eqmv[module][index, 4:7]
        by = self.eqmv[module][index, 7:10]
        temp_x = (bx - bc) / np.linalg.norm(bx - bc)
        temp_y = (by - bc) / np.linalg.norm(by - bc)
        unit_z = np.cross(temp_x, temp_y) / np.linalg.norm(np.cross(temp_x, temp_y))
        unit_y = np.cross(unit_z, temp_x) / np.linalg.norm(np.cross(unit_z, temp_x))
        unit_x = np.cross(unit_y, unit_z) / np.linalg.norm(np.cross(unit_y, unit_z))
        btf_inv = np.zeros((4,4))
        btf_inv[0:3,0:3] = np.array([unit_x,unit_y,unit_z])  ## transpose of original R
        btf_inv[0:3,3] = -np.dot(btf_inv[0:3,0:3],bc)
        btf_inv[3,3] = 1.0
        return btf_inv

    def btf_from_idx(self, module, index):
        bc = self.eqmv[module][index, 1:4]
        bx = self.eqmv[module][index, 4:7]
        by = self.eqmv[module][index, 7:10]
        temp_x = (bx - bc) / np.linalg.norm(bx - bc)
        temp_y = (by - bc) / np.linalg.norm(by - bc)
        unit_z = np.cross(temp_x, temp_y) / np.linalg.norm(np.cross(temp_x, temp_y))
        unit_y = np.cross(unit_z, temp_x) / np.linalg.norm(np.cross(unit_z, temp_x))
        unit_x = np.cross(unit_y, unit_z) / np.linalg.norm(np.cross(unit_y, unit_z))
        btf = np.zeros((4,4))
        btf[0:3,0:3] = np.transpose(np.array([unit_x,unit_y,unit_z]))
        btf[0:3,3] = bc
        btf[3,3] = 1.0
        return btf

    ## angle between two vectors [0 <= return < pi]
    def abs_ang_bt_two_vec(self, vec1, vec2):
        ang1 = math.atan2(vec1[1], vec1[0])
        return self.abs_ang_bt_ang_vec(ang1, vec2)

    ## angle between angle and 'angle of vector' [0 <= return < pi]
    def abs_ang_bt_ang_vec(self, ang1, vec2):
        ang2 = math.atan2(vec2[1], vec2[0])
        return self.abs_ang_bt_two_ang(ang1, ang2)

    ## angle between two angles [0 <= return < pi]
    def abs_ang_bt_two_ang(self, ang1, ang2):
        abs_ang = abs(ang1 - ang2)
        while (abs_ang > math.pi):
            abs_ang = 2.0 * math.pi - abs_ang
        return abs_ang

################################################################################################################################
################################################################################################################################
################################################################################################################################
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
        if (plot_what == 'sweep'):
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
        if (plot_what == 'sweep'):
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


