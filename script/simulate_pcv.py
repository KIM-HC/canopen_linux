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
from copy import deepcopy
import matplotlib.pyplot as plt                     # PLOT
import numpy as np                                  #
import rospkg                                       #
import math                                         #
import time
import yaml                                         # READ SETTING DATA
import csv
import os
#####################################################

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

class PCVSimulator:
    def __init__(self, stationary_set, param_set):
        target_1 = (45.0) * DEG2RAD
        target_2 = (45.0 + 100.0) * DEG2RAD
        self.debug_rot = []
        self.lap = 3.03
        self.tick = 0
        self.rot_speed = 0.4  # rad/sec
        self.sim_dt = 0.01
        self.gif_interval = 1
        self.gif_repeat = True
        ## for saving gif file
        self.make_gif = False
        if self.make_gif:
            self.rot_speed = 1.0  # rad/sec
            self.sim_dt = 0.1
            self.gif_interval = 100
            self.gif_repeat = False

        self.current_time = 0.0
        self.rest_time = 1.0
        self.stationary_set = stationary_set
        self.set_time = (self.lap * math.pi * 2) / self.rot_speed
        self.plt_color = ['y','r','g','b','c','m','k']
        self.plt_shape = ['s','^','o','*','+','x','2']

        ## PARAMETER
        self.steer_point  = param_set['steer_point']
        self.beta         = param_set['beta']
        self.wheel_offset = param_set['wheel_offset']
        self.wheel_radius = param_set['wheel_radius']
        self.robot_center = np.array([0.0, 0.0, 0.0])  ## position of mocap marker (center)
        self.robot_front  = np.array([200, 0.0, 0.0])  ## position of mocap marker (front)
        self.robot_left   = np.array([0.0, 100, 0.0])  ## position of mocap marker (left)
        self.robot_from_robot = np.array([self.robot_center, self.robot_front, self.robot_left])

        ## DIFFERS BY TARGET_ANGLE
        self.targets = [target_1, target_2] ## target steer angle of stationary caster
        self.targets_w_error = [target_1 + self.beta[stationary_set], target_2 + self.beta[stationary_set]] ## target steer angle of stationary caster
        self.wheel_point         = []       ## location of wheel
        self.sweep_tick          = []       ## sweep length of robot in each wheel point per tick
        self.wheel_rot_tick      = []       ## rotate angle of each wheel point per tick
        self.robot_rot_point     = []       ## robot rotate point (= contact point of stationary caster's wheel)
        self.dist_from_rrp       = []       ## distance from robot rotate point
        self.circle_start_tick   = [0, 0]
        self.circle_end_tick     = [0, 0]
        self.circle_start_time   = [0, 0]
        self.circle_end_time     = [0, 0]
        self.current_radian      = 0.0
        self.local_time          = 0.0

        ## time + joints[R S](8): 9
        self.robot_joint = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.robot_real_steer = np.array([0.0, 0.0, 0.0, 0.0])
        ## time + 3d-points[C F L](3): 10
        self.robot_point = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen') + '/data/simulation/'
        self.ymd = time.strftime('_%Y_%m_%d_', time.localtime())
        self.file_number = 0
        while(os.path.isfile(self.pkg_path + 'joint' + self.ymd + str(self.file_number) + '.csv')):
            self.file_number += 1
        self.wrdb       = open(self.pkg_path + 'debug' + self.ymd + str(self.file_number) + '.txt', 'w')
        self.joint_open = open(self.pkg_path + 'joint' + self.ymd + str(self.file_number) + '.csv', 'w')
        self.mocap_open = open(self.pkg_path + 'mocap' + self.ymd + str(self.file_number) + '.csv', 'w')
        self.wrjt = csv.writer(self.joint_open, delimiter='\t')
        self.wrmc = csv.writer(self.mocap_open, delimiter='\t')

    def make_data(self):
        for pt in range(2):
            ## TARGET_1 DATA
            self.init_align(pt=pt)
            self.circle_start_tick[pt] = deepcopy(self.tick)
            self.circle_start_time[pt] = deepcopy(self.current_time)
            while(self.local_time < self.set_time):
                self.perform_iteration(pt=pt)
            self.circle_end_tick[pt] = self.tick - 1
            self.circle_end_time[pt] = self.current_time - self.sim_dt
            ## REST DATA
            while(self.local_time < self.set_time + self.rest_time):
                self.perform_iteration(pt=pt, is_rest=True)

        self.joint_open.close()
        self.mocap_open.close()

        self.write_debug_info()
        # self.debug_steer_angle()

        # self.plot_rot()
        # self.animate_rot()

    def perform_iteration(self, pt, is_rest=False):
        if is_rest == False:
            rrp = self.robot_rot_point[pt]
            for module in range(4):
                if module == self.stationary_set: continue
                self.robot_joint[module*2+1] += self.wheel_rot_tick[pt][module]
            for point in range(3):
                pp = self.robot_from_robot[point] - rrp
                self.robot_point[point*3+1:(point+1)*3+1] = rrp + np.dot(self.rot_z(self.current_radian), pp)
        self.debug_rot.append(self.current_radian)
        self.save_data()
        if is_rest == False:
            self.current_radian += self.rot_speed * self.sim_dt
        self.tick += 1
        self.current_time += self.sim_dt
        self.local_time += self.sim_dt

    def init_align(self, pt):
        self.wheel_point.append([np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])])
        self.wheel_rot_tick.append(np.array([0.0, 0.0, 0.0, 0.0]))
        self.sweep_tick.append(np.array([0.0, 0.0, 0.0, 0.0]))
        self.dist_from_rrp.append(np.array([0.0, 0.0, 0.0, 0.0]))
        self.robot_rot_point.append(np.array([0.0, 0.0, 0.0]))
        self.current_radian = 0.0
        self.local_time = 0.0
        self.robot_rot_point[pt] = deepcopy(self.steer_point[self.stationary_set])
        self.robot_rot_point[pt] += np.dot(self.rot_z(self.targets[pt]),np.array([self.wheel_offset[self.stationary_set],0,0]))

        test_sweep = 120.0
        # print('test wheel_rot when robot rot {0}'.format(test_sweep))

        ## STEER ANGLE, WHEEL POINT, DISTANCE FROM ROT POINT, ROT OF WHEEL PER TICK
        for module in range(4):
            if module == self.stationary_set:
                self.robot_real_steer[self.stationary_set] = deepcopy(self.targets[pt])
            else:
                d_rs  = np.linalg.norm(self.steer_point[module] - self.robot_rot_point[pt])  # from rot point to steer point
                d_sw  = self.wheel_offset[module]                           # from steer point to wheel point
                a_srw = abs(math.asin(d_sw/d_rs))                           # angle between line rs and rw
                v_rs  = self.steer_point[module] - self.robot_rot_point[pt] # vector from rot point to steer point
                a_rs  = math.atan2(v_rs[1], v_rs[0])                        # angle of vector rs wrt base frame
                a_rw  = a_rs - a_srw                                        # angle of vector rw wrt base frame
                self.robot_real_steer[module]  = a_rw + math.pi / 2.0       # steer angle is vertical to a_rw, '+' because CCW turn

            self.robot_joint[(module+1)*2] = self.robot_real_steer[module] + self.beta[module]
            self.wheel_point[pt][module] = deepcopy(self.steer_point[module])
            self.wheel_point[pt][module] += np.dot(self.rot_z(self.robot_real_steer[module]),np.array([self.wheel_offset[module],0,0]))
            self.dist_from_rrp[pt][module] = np.linalg.norm(self.wheel_point[pt][module] - self.robot_rot_point[pt])
            self.sweep_tick[pt][module] = self.dist_from_rrp[pt][module] * self.rot_speed * self.sim_dt
            self.wheel_rot_tick[pt][module] = self.sweep_tick[pt][module] / self.wheel_radius[module]
            test_wheel_rot = self.dist_from_rrp[pt][module] * test_sweep * math.pi / (180.0 * self.wheel_radius[module])
            # print('set{0}: {1}'.format(module, test_wheel_rot))

    def plot_rot(self):
        self.plt_color = ['y','r','g','b','c','m','k']
        self.plt_shape = ['s','^','o','*','+','x','2']
        x = 1
        y = 0
        mv = np.loadtxt(self.pkg_path + 'mocap' + self.ymd + str(self.file_number) + '.csv', delimiter='\t')
        fig = plt.figure(77)

        for pt in range(2):
            ax = fig.add_subplot(1,2,pt+1)
            cst = self.circle_start_tick[pt]
            ced = self.circle_end_tick[pt]
            ax.plot(mv[cst:ced+1,x+1], mv[cst:ced+1,y+1], 'y-', linewidth=2)
            ax.plot(self.robot_from_robot[:,x], self.robot_from_robot[:,y], 'ko', markersize=3)
            rob_square = deepcopy(self.steer_point)
            rob_square.append(rob_square[0])
            rob_square = np.array(rob_square)
            ax.plot(rob_square[:,x],rob_square[:,y],'k-',linewidth=1)
            sp = np.array(self.steer_point)
            wp = np.array(self.wheel_point[pt])
            ax.plot([sp[:,x]], [sp[:,y]], 'ro', markersize=2)
            ax.plot([wp[:,x]], [wp[:,y]], 'bo', markersize=4)
            for module in range(4):
                ax.plot([sp[module,x],wp[module,x]], [sp[module,y],wp[module,y]], 'b-', linewidth=2)

            ax.axis('equal')
            ax.invert_xaxis()

        plt.legend()
        plt.show()

    def animate_rot(self):
        x = 1
        y = 0
        mv = np.loadtxt(self.pkg_path + 'mocap' + self.ymd + str(self.file_number) + '.csv', delimiter='\t')
        rob_square = deepcopy(self.steer_point)
        rob_square.append(rob_square[0])
        pop_tick = int((math.pi * 2.0) / (3.0 * self.rot_speed * self.sim_dt))

        for pt in range(2):
            if self.make_gif: plt.close(88)
            fig = plt.figure(88)
            ax = fig.add_subplot(111)
            st = self.circle_start_tick[pt]
            ed = self.circle_end_tick[pt] - 1
            rrp = self.robot_rot_point[pt]

            px, py = [], []
            ml,  = ax.plot([],[],'ko', markersize=2, zorder=4) ## mocap markers
            sl,  = ax.plot([],[],'k*', markersize=2, zorder=4) ## steer points
            wl,  = ax.plot([],[],'bo', markersize=3, zorder=4) ## wheel points
            lp,  = ax.plot([],[],'r-',  linewidth=2, zorder=3) ## line path
            rl,  = ax.plot([],[],'k-',  linewidth=1, zorder=5) ## robot square
            sw0, = ax.plot([],[],'b-',  linewidth=3, zorder=2) ## steer-wheel offset 0
            sw1, = ax.plot([],[],'b-',  linewidth=3, zorder=2) ## steer-wheel offset 1
            sw2, = ax.plot([],[],'b-',  linewidth=3, zorder=2) ## steer-wheel offset 2
            sw3, = ax.plot([],[],'b-',  linewidth=3, zorder=2) ## steer-wheel offset 3

            ax.plot(mv[st:ed+1,x+1], mv[st:ed+1,y+1], 'y-', linewidth=1, zorder=1)
            ax.plot([self.robot_rot_point[pt][x]],[self.robot_rot_point[pt][y]],'ys',markersize=5, label='rot point', zorder=1)

            def update(idx):
                idx = int(idx)
                rad = self.debug_rot[idx]
                if len(px) > pop_tick:
                    px.pop(0)
                    py.pop(0)
                if idx==st:
                    px.clear()
                    py.clear()
                px.append(mv[idx,x+1])
                py.append(mv[idx,y+1])
                lp.set_data(px, py)

                rs =[]
                for i in range(len(rob_square)):
                    rs.append(self.from_rrp_rot(rob_square[i],rrp,rad))
                rs = np.array(rs)
                rl.set_data(rs[:,x],rs[:,y])

                sp = []
                wp = []
                for i in range(4):
                    sp.append(self.from_rrp_rot(self.steer_point[i],rrp,rad))
                    wp.append(self.from_rrp_rot(self.wheel_point[pt][i],rrp,rad))
                sp = np.array(sp)
                wp = np.array(wp)
                sl.set_data(sp[:,x],sp[:,y])
                wl.set_data(wp[:,x],wp[:,y])

                mm = []
                for i in range(3):
                    mm.append(self.from_rrp_rot(self.robot_from_robot[i],rrp,rad))
                mm = np.array(mm)
                ml.set_data(mm[:,x],mm[:,y])

                sw0.set_data([sp[0,x],wp[0,x]], [sp[0,y],wp[0,y]])
                sw1.set_data([sp[1,x],wp[1,x]], [sp[1,y],wp[1,y]])
                sw2.set_data([sp[2,x],wp[2,x]], [sp[2,y],wp[2,y]])
                sw3.set_data([sp[3,x],wp[3,x]], [sp[3,y],wp[3,y]])
                return lp, rl, ml, sl, wl, sw0, sw1, sw2, sw3

            radius = np.linalg.norm(self.robot_rot_point[pt] - self.steer_point[(self.stationary_set+2)%4]) * 1.05

            ax.set_xlim(self.robot_rot_point[pt][x] - radius, self.robot_rot_point[pt][x] + radius)
            ax.set_ylim(self.robot_rot_point[pt][y] - radius, self.robot_rot_point[pt][y] + radius)
            ax.set_aspect('equal')
            ax.invert_xaxis()
            ax.set_title('animation')
            ani = FuncAnimation(fig, update, frames=np.linspace(st, ed, ed - st + 1),
                                interval=self.gif_interval, repeat=self.gif_repeat)
            if self.make_gif:
                ani.save('/home/khc/Pictures/rot_ani/set{0}_pt{1}.gif'.format(self.stationary_set, pt+1), writer='imagemagick')
            else: plt.show()

    def from_rrp_rot(self, point, rrp, rad):
        from_rrp = point - rrp
        ret = rrp + np.dot(self.rot_z(rad), from_rrp)
        return ret

    def write_debug_info(self):
        self.wrdb.write('joint_file: {0}\n'.format('joint' + self.ymd + str(self.file_number) + '.csv'))
        self.wrdb.write('mocap_file: {0}\n'.format('mocap' + self.ymd + str(self.file_number) + '.csv'))
        self.wrdb.write('one set time: {0}\n'.format(self.set_time))
        self.wrdb.write('rest time: {0}\n'.format(self.rest_time))
        self.wrdb.write('rot_speed: {0}\n'.format(self.rot_speed))
        self.wrdb.write('stationary_set: {0}\n'.format(self.stationary_set))
        self.wrdb.write('steer_angle: {0}\n'.format(self.targets_w_error))
        self.wrdb.write('P_1: {0}\n'.format(self.robot_rot_point[0]))
        self.wrdb.write('P_2: {0}\n'.format(self.robot_rot_point[1]))
        self.wrdb.write('circle: [[{0}, {1}], [{2}, {3}]]\n'.format(
            self.circle_start_time[0],self.circle_end_time[0],
            self.circle_start_time[1],self.circle_end_time[1]
        ))

        self.wrdb.write('\n================== PARAMETERS ==================\n')
        for i in range(4):
            self.wrdb.write('===== SET {0} =====\n'.format(i))
            self.wrdb.write('angle_error_deg: {0}\n'.format(self.beta[i]*RAD2DEG))
            self.wrdb.write('angle_error_rad: {0}\n'.format(self.beta[i]))
            self.wrdb.write('steer_point: [{0}, {1}]\n'.format(self.steer_point[i][0]/1000.0,self.steer_point[i][1]/1000.0))
            self.wrdb.write('wheel_offset: {0}\n'.format(self.wheel_offset[i]/1000.0))
            self.wrdb.write('wheel_radius: {0}\n'.format(self.wheel_radius[i]/1000.0))
        self.wrdb.write('================================================\n\n')

    def save_data(self):
        self.robot_joint[0] = self.current_time
        self.robot_point[0] = self.current_time
        self.wrjt.writerow(self.robot_joint)
        self.wrmc.writerow(self.robot_point)

    def export_data(self):
        exp = {}
        exp['mv_file'] = 'simulation/mocap' + self.ymd + str(self.file_number) + '.csv'
        exp['equal_mv_file'] = 'simulation/mocap' + self.ymd + str(self.file_number) + '.csv'
        exp['jt_file'] = 'simulation/joint' + self.ymd + str(self.file_number) + '.csv'
        exp['mv_cali'] = [0]
        exp['jt_cali'] = [0]
        exp['steer_angle'] = self.targets_w_error

        loop_index = []
        for pt in range(2):
            loop_index.append([self.circle_start_time[pt], self.circle_end_time[pt]])

        return exp, loop_index

    def debug_steer_angle(self):
        print('\n==DEBUG STEER ANGLE==')
        for pt in range(2):
            print('dist_from_rrp: {0}'.format(self.dist_from_rrp[pt].tolist()))
            # print('wheel_point',self.wheel_point[pt])
            # print('sweep_tick',self.sweep_tick[pt])
            # print('wheel_rot_tick',self.wheel_rot_tick[pt])
        # print('\nINNER PRODUCT OF VEC_RW and VEC_WS')
        # for pt in range(2):
        #     print('DEBUG PT {0}'.format(pt+1))
        #     for module in range(4):
        #         if module == self.stationary_set: continue
        #         v_rw = self.wheel_point[pt][module] - self.robot_rot_point[pt]
        #         v_ws = self.steer_point[module] - self.wheel_point[pt][module]
        #         inner_ = np.dot(v_rw, v_ws)
        #         print('module {0}: {1}'.format(module, inner_))
        print('==DEBUG STEER ANGLE==\n')

    def rot_z(self, rad):
        rot = np.array([
            [math.cos(rad), -math.sin(rad), 0.0],
            [math.sin(rad),  math.cos(rad), 0.0],
            [          0.0,            0.0, 1.0]
        ])
        return rot

    def make_tf(self, ang_z, pos):
        ret = np.identity(4)
        ret[0:3, 0:3] = self.rot_z(ang_z)
        ret[3, 0:3] = pos
        return ret

    def make_tf_inv(self, ang_z, pos):
        ret = np.identity(4)
        ret[0:3, 0:3] = np.transpose(self.rot_z(ang_z))
        ret[3, 0:3] = -np.dot(ret[0:3, 0:3], pos)
        return ret

if __name__ == "__main__":
    e_pnt = 5           # mm
    e_bet = 7.0*DEG2RAD # radian
    e_off = 3           # mm
    e_rad = 3           # mm

    no_error = False
    if no_error:
        e_pnt = 0
        e_bet = 0
        e_off = 0
        e_rad = 0

    error_point = []
    for module in range(4):
        rn = e_pnt * np.random.random_sample(3) - e_pnt/2
        rn[2] = 0.0
        error_point.append(rn)
    error_beta = (e_bet * np.random.random_sample(4) - e_bet/2).tolist()
    error_offset = (e_off * np.random.random_sample(4) - e_off/2).tolist()
    error_radius = (e_rad * np.random.random_sample(4) - e_rad/2).tolist()

    param_set = {}
    param_set['steer_point']  = [np.array([215,125,0.0]), np.array([215,-125,0.0]),np.array([-215,-125,0.0]),np.array([-215,125,0.0])]
    param_set['beta']         = [  0,   0,   0,   0]
    param_set['wheel_offset'] = [-20, -20, -20, -20]
    param_set['wheel_radius'] = [ 55,  55,  55,  55]

    for module in range(4):
        param_set['steer_point'][module]  += error_point[module]
        param_set['beta'][module]         += error_beta[module]
        param_set['wheel_offset'][module] += error_offset[module]
        param_set['wheel_radius'][module] += error_radius[module]

    pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')
    file_number = 0
    while(os.path.isfile(pkg_path + '/setting/simulation/simulation_basic_'  + str(file_number) + '.yaml')):
        file_number += 1
    dumper = {}
    dumper['loop_index'] = {}
    dumper['loop_index'][0] = []
    dumper['is_mocap'] = True
    dumper['is_simulation'] = True
    dumper['parameter_file'] = '/setting/output/sim_param_' + str(file_number) + '.yaml'

    for module in range(4):
        pcv_sim = PCVSimulator(stationary_set=module, param_set=param_set)
        pcv_sim.make_data()
        dumper['set_{0}'.format(module)], lp_idx = pcv_sim.export_data()
        dumper['loop_index'][0].append(lp_idx)

    with open(pkg_path + '/setting/simulation/simulation_basic_' + str(file_number) + '.yaml', 'w') as f:
        yaml.dump(dumper, f)

    param_dumper = {}
    for module in range(4):
        ddd = {}
        ddd['angle_error_deg'] = param_set['beta'][module] * RAD2DEG
        ddd['angle_error_rad'] = param_set['beta'][module]
        ddd['steer_point'] = (param_set['steer_point'][module][0:2] / 1000.0).tolist()
        ddd['wheel_offset'] = param_set['wheel_offset'][module] / 1000.0
        ddd['wheel_radius'] = param_set['wheel_radius'][module] / 1000.0

        param_dumper[module] = ddd
        

    with open(pkg_path + '/setting/output/sim_param_' + str(file_number) + '.yaml', 'w') as f:
        yaml.dump(param_dumper, f)

