#!/usr/bin/env python3
"""
for canopen communication with PCV(elmo controller)
Kim Hyoung Cheol
https://github.com/KIM-HC/dyros_pcv_canopen
https://www.notion.so/kimms74/40dcc3a8ff054dc9994e5fc62de9bc30
"""
import os
import csv
import rospy
import rospkg
import time
import canopen
# from canopen.profiles.p402 import BaseNode402, OperationMode
import traceback
from sensor_msgs.msg import JointState
import math
import threading

#### CONTROL INFORMATION ####
HZ = 300

#### ENCODER INFORMATION ####
####  ELMO 1 3 4 5 6   ####
REV2CNT = 2048.0
CNT2REV = 1.0 / REV2CNT
RAD2CNT = REV2CNT / (2.0 * math.pi)
CNT2RAD = (2.0 * math.pi) / REV2CNT

####      ELMO 2 7 8       ####
REV2CNT2 = 8192.0
CNT2REV2 = 1.0 / REV2CNT2
RAD2CNT2 = REV2CNT2 / (2.0 * math.pi)
CNT2RAD2 = (2.0 * math.pi) / REV2CNT2

##        SET0-R   SET0-S   SET1-R   SET1-S   SET2-R   SET2-S   SET3-R    SET3-S
C2R   = [CNT2RAD, CNT2RAD2, CNT2RAD, CNT2RAD, CNT2RAD, CNT2RAD, CNT2RAD2, CNT2RAD2]
R2C   = [RAD2CNT, RAD2CNT2, RAD2CNT, RAD2CNT, RAD2CNT, RAD2CNT, RAD2CNT2, RAD2CNT2]
REV2C = [REV2CNT, REV2CNT2, REV2CNT, REV2CNT, REV2CNT, REV2CNT, REV2CNT2, REV2CNT2]

##### GEAR RATIO ####
NS = 4.0            ## steer , 96.0/24.0
NR = 85.0 / 35.0    ## translate , 2.428571429
NW = 2.0            ## wheel (rolling) , 2.0/ 1.0

N00 = NS
N10 = NR
N11 = -(NR * NW)

## inverse
NI00 = 1.0 / NS
NI10 = 1.0 / (NS * NW)
NI11 = -1.0 / (NR * NW)

OFF1 =  135.0 * REV2C[1]  * N00 / 360.0
OFF2 = -135.0 * REV2C[3]  * N00 / 360.0
OFF3 = -45.0  * REV2C[5]  * N00 / 360.0
OFF4 =  45.0  * REV2C[7] * N00 / 360.0

HOME_OFFSET = [OFF1, OFF2, OFF3, OFF4]

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class OPMode():
    NO_MODE =              -1
    PROFILED_POSITION =     1
    PROFILED_VELOCITY =     3
    PROFILED_TORQUE =       4
    HOMING =                6
    INTERPOLATED_POSITION = 7

    ## for controlling motor values
    MOTOR_POSITION  = 1
    MOTOR_VELOCITY  = 3
    MOTOR_TORQUE    = 4

    ## for controlling joint values
    JOINT_POSITION  = 11
    JOINT_VELOCITY  = 13
    JOINT_TORQUE    = 14

class CtrlWord():
    FAULT_RESET =  0b10000000
    SWITCH_ON =    0b10000001
    SHUTDOWN =          0b110
    SWITCH_ON =         0b111
    DISABLE_OPERATION = 0b111
    ENABLE_OPERATION = 0b1111

class CtrlPDO(dict):
    def __init__(self):
        self['controlword'] =        1
        self['target_torque'] =      1
        self['modes_of_operation'] = 1
        self['target_velocity'] =    2
        self['target_position'] =    2

class TestElmo():
    def __init__(self, node_list, operation_mode=OPMode.PROFILED_TORQUE):
        self.operation_mode = operation_mode
        self.control_target = 'target_torque'
        if (self.operation_mode == OPMode.PROFILED_VELOCITY): self.control_target = 'target_velocity'
        
        self.js_ = JointState(
            name = ['m1','m2','m3','m4','m5','m6','m7','m8'],
            velocity = [0,0,0,0,0,0,0,0],
            effort = [0,0,0,0,0,0,0,0],
            position = [0,0,0,0,0,0,0,0]
        )
        rospy.init_node('canopen')
        self.joint_pub = rospy.Publisher('dyros_mobile/joint_state', JointState, queue_size=5)
        self.joint_sub = rospy.Subscriber('dyros_mobile/desired_joint', JointState, self._joint_callback)
        rospy.on_shutdown(self.finish_work)
        rospy.set_param('is_torque_control',True)

        ## -----------------------------------------------------------------
        ## basic setting
        self.node_list = node_list
        self.sleep_ = 0.005
        self.start_time = rospy.Time.now()
        self.sub_time = rospy.Time.now()
        self.lock_ = threading.Lock()
        self.torque_limit = 600
        self.stop_call_ = False
        self.cannot_test_ = False
        self.homing_call_ = True
        self.ready4control_ = False
        self.print_motor_status_ = False
        self.is_disconnected_ = False
        self.is_cali_move_mode_ = False
        self.is_cali_mode_changed_ = True
        self.is_cali_speed_compensation = False
        self.is_cali_auto_second_steer = False
        self.is_cali_constant_torque = True
        self.is_cali_use_fixer = False
        self.cali_target_speed = 0.0
        self.cali_gain_kp = 0.0
        self.cali_lpf_sensitivity = 0.0

        ## -----------------------------------------------------------------
        ## for debug
        pkg_path = rospkg.RosPack().get_path('dyros_pcv_canopen')
        self.db_ = open(pkg_path + '/debug/debug'+time.strftime('_%Y_%m_%d', time.localtime())+'.txt', 'a')
        ptime_ = time.strftime('==  DEBUG START %Y.%m.%d - %X', time.localtime())
        two_bar = ''
        for _ in range(len(ptime_)+4):
            two_bar = two_bar + '='
        self._dprint('\n\n\n' + two_bar + two_bar)
        self._dprint(ptime_ + '  ==' + two_bar)
        self._dprint(two_bar + two_bar + '\n')
        self.db_position_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.db_velocity_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        pkg_path_joint = rospkg.RosPack().get_path('dyros_pcv_canopen') + '/data/joint_raw/joint'
        ymd = time.strftime('_%Y_%m_%d_', time.localtime())
        file_number = 0
        while(os.path.isfile(pkg_path_joint + ymd + str(file_number) + '.csv')):
            file_number += 1
        self.wrdb = open(pkg_path_joint + '_debug' + ymd + str(file_number) + '.txt', 'w')
        self.qd_open = open(pkg_path_joint + '_vel' + ymd + str(file_number) + '.csv', 'w')
        self.q_open = open(pkg_path_joint + ymd + str(file_number) + '.csv', 'w')
        self.wrqd = csv.writer(self.qd_open, delimiter='\t')
        self.wrq = csv.writer(self.q_open, delimiter='\t')

        ## -----------------------------------------------------------------
        ## path for eds file of elmo
        self.EDS_PATH = pkg_path + '/eds_file/elmo.eds'  ## os.path.join(os.path.dirname(__file__), 'elmo.eds')

        ## -----------------------------------------------------------------
        ## one network per one CAN Bus
        self.network = canopen.Network()

        self._connect_network()

    ## catches errors and writes it in debug file
    def _try_except_decorator(func):
        def decorated(self, *args, **kwargs):
            if self.cannot_test_:
                self._dprint('\n----\nFUNCTION: {0}\nTEST CANNOT BE PERFORMED (decorator)\n----'.format(func.__name__))
            else:
                try:
                    func(self, *args, **kwargs)

                except KeyboardInterrupt:
                    if (self.lock_.locked()): self.lock_.release()
                    self._dprint('KeyboardInterrupt!! stopping device...')
                    self._stop_operation()
                    self._disconnect_device()

                except canopen.SdoCommunicationError as e:
                    if (self.lock_.locked()): self.lock_.release()
                    self.cannot_test_ = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR SdoCommunicationError\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))

                except canopen.SdoAbortedError as e:
                    if (self.lock_.locked()): self.lock_.release()
                    self.cannot_test_ = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR SdoAbortedError\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))

                except Exception as e:
                    if (self.lock_.locked()): self.lock_.release()
                    self.cannot_test_ = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))
        return decorated

    @_try_except_decorator
    def _connect_network(self):
        ## -----------------------------------------------------------------
        ## connect to CAN Bus (PEAK PCAN-USB)
        # self.network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
        ## using socketcan (PEAK PCAN-USB)
        self.network.connect(bustype='socketcan', channel='can0', bitrate=1000000)

        ## -----------------------------------------------------------------
        ## attempt to read SDO from nodes 1 - 127
        ## add found node
        self.network.scanner.search()
        time.sleep(0.1)
        self._change_status(status='INITIALISING')
        self._dprint()
        for node_id in self.network.scanner.nodes:
            self._dprint('Found node {0}'.format(node_id))
            if node_id in self.node_list:
                self._dprint('Adding node {0}'.format(node_id))
                node_made_ = canopen.RemoteNode(node_id=node_id, object_dictionary=self.EDS_PATH)
                # node_made_ = BaseNode402(node_id=node_id, object_dictionary=self.EDS_PATH)
                self.network.add_node(node_made_)

                ## dont know if it works or not
                self.network[node_id].nmt.state = 'INITIALISING'
                all_codes = [emcy.code for emcy in self.network[node_id].emcy.log]
                active_codes = [emcy.code for emcy in self.network[node_id].emcy.active]
                for code_ in all_codes:
                    self._dprint('emcy: {0}'.format(code_))
                for code_ in active_codes:
                    self._dprint('emcy: {0}'.format(code_))
                self.network[node_id].emcy.reset()

        ## -----------------------------------------------------------------
        ## check bus state
        self._dprint('\nstate: {0}'.format(self.network.bus.state))
        self._dprint('bus channel_info: {0}'.format(self.network.bus.channel_info))

        ## -----------------------------------------------------------------
        ## send out time message
        self.network.time.transmit()

        # ## -----------------------------------------------------------------
        # ## receive message
        # msg = self.network.bus.recv(timeout=1.0)  ## TODO: keeps getting blocked here
        # self._dprint('\nmsg: {0}'.format(msg))

        ## -----------------------------------------------------------------
        ## node with node_list id is needed from now on
        for id_ in self.node_list:
            if (id_ not in self.network):
                self._dprint('\n----\ntest node {0} is not in network\n----'.format(id_))
                self.network.disconnect()
                self.cannot_test_ = True
                return

        ## -----------------------------------------------------------------
        ## read
        self.network.sync.transmit()
        for node_id_ in self.network:
            self.network[node_id_].tpdo.read()
            self.network[node_id_].rpdo.read()

        ## -----------------------------------------------------------------
        ## set SDO value
        self._change_status(status='STOPPED')
        self._change_status(status='RESET')
        self._change_status(status='RESET COMMUNICATION')
        self._change_status(status='INITIALISING')
        self._change_status(status='PRE-OPERATIONAL')

        for node_id_ in self.network:
            if (node_id_ % 2 == 0):
                ## Homing on the positive home switch
                self.network[node_id_].sdo['homing_method'].raw = 20
                self.network[node_id_].sdo['home_offset'].raw = HOME_OFFSET[(node_id_ // 2) - 1]
                self.network[node_id_].sdo['homing_speeds'][1].raw = 500
                self.network[node_id_].sdo['homing_speeds'][2].raw = 900
                if (C2R[node_id_ - 1] == CNT2RAD2):
                    self.network[node_id_].sdo['homing_speeds'][1].raw = 500  * 4
                    self.network[node_id_].sdo['homing_speeds'][2].raw = 900 * 4
                self.network[node_id_].sdo['homing_acceleration'].raw = self.network[node_id_].sdo['profile_acceleration'].raw
            else:
                self.network[node_id_].sdo['homing_method'].raw = 35
                self.network[node_id_].sdo['home_offset'].raw = 0
                self.network[node_id_].sdo['homing_speeds'][1].raw = 1
                self.network[node_id_].sdo['homing_speeds'][2].raw = 1
                self.network[node_id_].sdo['homing_acceleration'].raw = 1
            ## The producer heartbeat time indicates the configured cycle time of the heartbeat, in milliseconds.
            ## A value of 0 disables the heartbeat.
            self.network[node_id_].sdo['Producer Heartbeat Time'].raw = 100
            ## This object defines the communication cycle period, in microseconds.
            ## Its value is 0 if it is not used. I still do not get what it is...
            # self.network[node_id_].sdo['Communication Cycle Period'].raw = 0
            self.network[node_id_].sdo['Communication Cycle Period'].raw = 0
            # self.network[node_id_].sdo['target_velocity'].raw = 0
            # self.network[node_id_].sdo['target_torque'].raw = 0
            # self.network[node_id_].sdo['target_position'].raw = 0
        ## -----------------------------------------------------------------
        ## check error
        for node_id_ in self.network:
            error_log = self.network[node_id_].sdo[0x1003]  ## Predefined Error Field
            for error in error_log.values():
                self._dprint("Error {0} was found in the log".format(hex(error.raw)))

        ## -----------------------------------------------------------------
        ## read current PDO & SDO configuration
        for node_id_ in self.network:
            self._dprint("\n===========\nInformation")
            self._dprint("CAN Open Node ID: {0}".format(self.network[node_id_].sdo['CAN Open Node ID'].raw))
            self._dprint("Product Code: {0}".format(hex(self.network[node_id_].sdo['Identity Object'][2].raw)))
            self._dprint("error_code: {0}".format(self.network[node_id_].sdo['error_code'].raw))
            self._dprint("statusword: {0}".format(bin(self.network[node_id_].sdo['statusword'].raw)))
            self._dprint("modes_of_operation: {0}".format(self.network[node_id_].sdo['modes_of_operation'].raw))
            self._dprint("Producer Heartbeat Time: {0}".format(self.network[node_id_].sdo['Producer Heartbeat Time'].raw))
            self._dprint("Communication Cycle Period: {0}".format(self.network[node_id_].sdo['Communication Cycle Period'].raw))

            self._dprint('\n- homing')
            self._dprint("home_offset: {0}".format(self.network[node_id_].sdo['home_offset'].raw))
            self._dprint("homing_method: {0}".format(self.network[node_id_].sdo['homing_method'].raw))
            self._dprint("homing_speeds search switch: {0}".format(self.network[node_id_].sdo['homing_speeds'][1].raw))
            self._dprint("homing_speeds search zero: {0}".format(self.network[node_id_].sdo['homing_speeds'][2].raw))
            self._dprint("homing_acceleration: {0}".format(self.network[node_id_].sdo['homing_acceleration'].raw))

            # self._dprint('\n- control')
            # self._dprint("target_velocity: {0}".format(self.network[node_id_].sdo['target_velocity'].raw))
            # self._dprint("target_torque: {0}".format(self.network[node_id_].sdo['target_torque'].raw))
            # self._dprint("target_position: {0}".format(self.network[node_id_].sdo['target_position'].raw))
            # self._dprint("profile_velocity: {0}".format(self.network[node_id_].sdo['profile_velocity'].raw))
            # self._dprint("position_demand_value: {0}".format(self.network[node_id_].sdo['position_demand_value'].raw))
            # self._dprint("position_actual_internal_value: {0}".format(self.network[node_id_].sdo['position_actual_internal_value'].raw))
            # self._dprint("position_actual_value: {0}".format(self.network[node_id_].sdo['position_actual_value'].raw))
            # self._dprint("profile_acceleration: {0}".format(self.network[node_id_].sdo['profile_acceleration'].raw))
            # self._dprint("profile_deceleration: {0}".format(self.network[node_id_].sdo['profile_deceleration'].raw))

        ## -----------------------------------------------------------------
        ## change TPDO configuration
        ## trans_type: 1=sync    255=254=asynchronous
        for node_id_ in self.network:
            trans_type_ = 1

            id_ = 1
            self.network[node_id_].tpdo[id_].clear()
            self.network[node_id_].tpdo[id_].add_variable('modes_of_operation')     ## 8bit
            self.network[node_id_].tpdo[id_].add_variable('statusword')             ## 16bit
            self.network[node_id_].tpdo[id_].add_variable('digital_inputs')         ## 32bit
            self.network[node_id_].tpdo[id_].trans_type = trans_type_
            self.network[node_id_].tpdo[id_].event_timer = 0
            self.network[node_id_].tpdo[id_].inhibit_time = 0
            self.network[node_id_].tpdo[id_].enabled = True

            id_ = 2
            self.network[node_id_].tpdo[id_].clear()
            self.network[node_id_].tpdo[id_].add_variable('torque_actual_value')    ## 16bit
            self.network[node_id_].tpdo[id_].add_variable('position_actual_value')  ## 32bit
            self.network[node_id_].tpdo[id_].trans_type = trans_type_
            self.network[node_id_].tpdo[id_].event_timer = 0
            self.network[node_id_].tpdo[id_].inhibit_time = 0
            self.network[node_id_].tpdo[id_].enabled = True

            id_ = 3
            self.network[node_id_].tpdo[id_].clear()
            self.network[node_id_].tpdo[id_].add_variable('velocity_actual_value')  ## 32bit
            self.network[node_id_].tpdo[id_].trans_type = trans_type_
            self.network[node_id_].tpdo[id_].event_timer = 0
            self.network[node_id_].tpdo[id_].inhibit_time = 0
            self.network[node_id_].tpdo[id_].enabled = True

            self.network[node_id_].tpdo.save()

        ## -----------------------------------------------------------------
        ## change RPDO configuration
        for node_id_ in self.network:
            id_ = 1
            self.network[node_id_].rpdo[id_].clear()
            self.network[node_id_].rpdo[id_].add_variable('controlword')            ## 16bit
            self.network[node_id_].rpdo[id_].add_variable('target_torque')          ## 16bit
            self.network[node_id_].rpdo[id_].add_variable('modes_of_operation')     ## 8bit
            self.network[node_id_].rpdo[id_].enabled = True

            id_ = 2
            self.network[node_id_].rpdo[id_].clear()
            self.network[node_id_].rpdo[id_].add_variable('target_velocity')        ## 32bit
            self.network[node_id_].rpdo[id_].add_variable('target_position')        ## 32bit
            self.network[node_id_].rpdo[id_].enabled = True

            id_ = 3
            self.network[node_id_].rpdo[id_].clear()
            self.network[node_id_].rpdo[id_].enabled = False

            self.network[node_id_].rpdo.save()

        ## -----------------------------------------------------------------
        ## fault reset
        self._rpdo_controlword(CtrlWord.FAULT_RESET)
        self._rpdo_controlword(CtrlWord.SWITCH_ON)

    ##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
    def _dprint(self, str=''):
        print(str)
        self.db_.write(str + '\n')

    def _print_status(self):
        self._dprint('master state: {0}'.format(self.network.nmt.state))
        for node_id in self.network:
            self._dprint('node {0} state: {1}'.format(node_id, self.network[node_id].nmt.state))
        self._dprint()

    def _change_status(self, test_set=None, status='PRE-OPERATIONAL', sleep_time=0.08):
        self._dprint('state command: {0}'.format(status))
        self.network.nmt.state = status
        if test_set == None:
            for node_id in self.network:
                self.network[node_id].nmt.state = status
        else:
            for node_id in test_set:
                self.network[node_id].nmt.state = status
        time.sleep(sleep_time)
        self._print_status()

    def _control_command(self, node_id, control_method, value, print_this=""):
        if (print_this != ""): self._dprint(print_this)
        if (C2R[node_id - 1] == CNT2RAD2 and control_method is "target_velocity"):
            value = value * 4
        self.network[node_id].rpdo[control_method].raw = value
        self.network[node_id].rpdo[CtrlPDO()[control_method]].transmit()

    def _rpdo_controlword(self, controlword, node_id = None, command='===UNKNOWN COMMAND==='):
        if controlword is 0b10000000:
            self._dprint('FAULT RESET COMMAND: 0b10000000')
        elif controlword is 0b10000001:
            self._dprint('SWITCH ON COMMAND: 0b10000001')
        elif controlword is 0b110:
            self._dprint('SHUTDOWN COMMAND: 0b110')
        elif controlword is 0b111:
            self._dprint('[SWITCH ON or DISABLE OPERATION] COMMAND: 0b111')
        elif controlword is 0b1111:
            self._dprint('[ENABLE OPERATION or OPERATION] COMMAND: 0b1111')
        elif controlword is 0b11111:
            self._dprint('OPERATION COMMAND: 0b11111')
        else:
            self._dprint('{0}: {1}'.format(command, controlword))

        if node_id is None:
            for node_id_ in self.network:
                self.network[node_id_].rpdo['controlword'].raw = controlword
                self.network[node_id_].rpdo[1].transmit()
            time.sleep(self.sleep_)
            self.network.sync.transmit()
            for node_id_ in self.network:
                str_id = "node_id:{0}".format(node_id_)
                str_stat = " stat:{0:<15}".format(bin(self.network[node_id_].tpdo['statusword'].raw))
                str_ctrl = " ctrl:{0:<10}".format(bin(self.network[node_id_].rpdo['controlword'].raw))
                self._dprint(str_id + str_stat + str_ctrl)

        elif type(node_id) is list:
            for node_id_ in node_id:
                self.network[node_id_].rpdo['controlword'].raw = controlword
                self.network[node_id_].rpdo[1].transmit()
            time.sleep(self.sleep_)
            self.network.sync.transmit()
            for node_id_ in node_id:
                str_id = "node_id:{0}".format(node_id_)
                str_stat = " stat:{0:<15}".format(bin(self.network[node_id_].tpdo['statusword'].raw))
                str_ctrl = " ctrl:{0:<10}".format(bin(self.network[node_id_].rpdo['controlword'].raw))
                self._dprint(str_id + str_stat + str_ctrl)

        else:
            self.network[node_id].rpdo['controlword'].raw = controlword
            self.network[node_id].rpdo[1].transmit()
            time.sleep(self.sleep_)
            self.network.sync.transmit()
            str_id = "node_id:{0}".format(node_id)
            str_stat = " stat:{0:<15}".format(bin(self.network[node_id].tpdo['statusword'].raw))
            str_ctrl = " ctrl:{0:<10}".format(bin(self.network[node_id].rpdo['controlword'].raw))
            self._dprint(str_id + str_stat + str_ctrl)
        self._dprint()

    def _check_statusword(self, statusword):
        if type(statusword) is int:
            statusword = bin(statusword)
        if (statusword[:2] != '0b'):
            self._dprint('cannot check statusword: {0}'.format(statusword))
            return
        statusword = statusword[2:]
        try:
            self._dprint('    Ready to switch on [0]: {0}'.format(statusword[-1]))
            self._dprint('           Switched on [1]: {0}'.format(statusword[-2]))
            self._dprint('     Operation enabled [2]: {0}'.format(statusword[-3]))
            self._dprint('                 Fault [3]: {0}'.format(statusword[-4]))
            self._dprint('       Voltage enabled [4]: {0}'.format(statusword[-5]))
            self._dprint('            Quick stop [5]: {0}'.format(statusword[-6]))
            self._dprint('    Switch on disabled [6]: {0}'.format(statusword[-7]))
            self._dprint('               Warning [7]: {0}'.format(statusword[-8]))
            self._dprint(' Manufacturer specific [8]: {0}'.format(statusword[-9]))
            self._dprint('                Remote [9]: {0}'.format(statusword[-10]))
            self._dprint('       Target reached [10]: {0}'.format(statusword[-11]))
            self._dprint('Internal limit active [11]: {0}'.format(statusword[-12]))
            self._dprint('     Op mode specific [12]: {0}'.format(statusword[-13]))
            self._dprint('     Op mode specific [13]: {0}'.format(statusword[-14]))
            self._dprint('Manufacturer specific [14]: {0}'.format(statusword[-15]))
            self._dprint('Manufacturer specific [15]: {0}\n'.format(statusword[-16]))
        except:
            self._dprint('')

    def _check_controlword(self, controlword):
        if type(controlword) is int:
            controlword = bin(controlword)
        if (controlword[:2] != '0b'):
            self._dprint('cannot check controlword: {0}'.format(controlword))
            return
        controlword = controlword[2:]
        try:
            self._dprint('             Switch on [0]: {0}'.format(controlword[-1]))
            self._dprint('        Enable voltage [1]: {0}'.format(controlword[-2]))
            self._dprint('            Quick stop [2]: {0}'.format(controlword[-3]))
            self._dprint('      Enable operation [3]: {0}'.format(controlword[-4]))
            self._dprint('      Op mode specific [4]: {0}'.format(controlword[-5]))
            self._dprint('      Op mode specific [5]: {0}'.format(controlword[-6]))
            self._dprint('      Op mode specific [6]: {0}'.format(controlword[-7]))
            self._dprint('           Fault reset [7]: {0}'.format(controlword[-8]))
            self._dprint('                 Hault [8]: {0}'.format(controlword[-9]))
            self._dprint('              Reserved [9]: {0}'.format(controlword[-10]))
            self._dprint('             Reserved [10]: {0}\n'.format(controlword[-11]))
        except:
            self._dprint('')

    def _from_joint_torque_to_motor_torque(self, jt_tor_s, jt_tor_r):
        mt_tor_s = NI00 * jt_tor_s + NI10 * jt_tor_r
        mt_tor_r =                   NI11 * jt_tor_r
        return mt_tor_s, mt_tor_r

    def _from_joint_velocity_to_motor_velocity(self, jt_vel_s, jt_vel_r, id_s, id_r):
        mt_vel_s = R2C[id_s - 1] * (N00 * jt_vel_s)
        mt_vel_r = R2C[id_r - 1] * (N10 * jt_vel_s + N11 * jt_vel_r)
        return mt_vel_s, mt_vel_r

    ## set_num (0 ~ 3)
    ## from motor to joint
    def _read_set(self, set_num):
        id_s = (set_num+1) * 2  ## steering node
        id_r = id_s - 1         ## rolling node

        mt_pos_s = self.network[id_s].tpdo['position_actual_value'].raw
        mt_vel_s = self.network[id_s].tpdo['velocity_actual_value'].raw
        mt_tor_s = self.network[id_s].tpdo['torque_actual_value'].raw  ## testing

        mt_pos_r = self.network[id_r].tpdo['position_actual_value'].raw
        mt_vel_r = self.network[id_r].tpdo['velocity_actual_value'].raw
        mt_tor_r = self.network[id_r].tpdo['torque_actual_value'].raw  ## testing

        jt_pos_s = C2R[id_s - 1] *  NI00 * mt_pos_s
        jt_pos_r = C2R[id_r - 1] * (NI10 * mt_pos_s + NI11 * mt_pos_r)

        jt_vel_s = C2R[id_s - 1] *  NI00 * mt_vel_s
        jt_vel_r = C2R[id_r - 1] * (NI10 * mt_vel_s + NI11 * mt_vel_r)

        jt_tor_s = N00 * mt_tor_s + N10 * mt_tor_r
        jt_tor_r =                  N11 * mt_tor_r

        if (self.print_motor_status_):
            return [id_s, mt_pos_s, mt_vel_s, mt_tor_s, jt_tor_s], [id_r, mt_pos_r, mt_vel_r, mt_tor_r, jt_tor_r]
        else:
            return [id_s, jt_pos_s, jt_vel_s, jt_tor_s, mt_tor_s], [id_r, jt_pos_r, jt_vel_r, jt_tor_r, mt_tor_r]

    def _read_and_print(self):
        current_time = 'time:%-6.2f'%((rospy.Time.now() - self.start_time).to_sec())
        homing_satus = ' | homing_satus:{0}'.format(self.ready4control_)
        print_status = ' | print_motor_val:{0} | in DEGREE'.format(self.print_motor_status_)
        self._dprint(current_time + homing_satus + print_status)
        if (self.node_list == [1,2,3,4,5,6,7,8]):
            for i in range(4):
                set_s, set_r = self._read_set(i)

                ids_s = 'S-id:{0} '.format(set_s[0])
                jt_pos_s = ' pos:%-9.2f'%(RAD2DEG * set_s[1])
                jt_vel_s = ' v:%-7.2f'%(RAD2DEG * set_s[2])
                jt_tor_s = ' jt:%-7.1f'%(set_s[3])
                mt_tor_s = ' mt:%-4d'%(set_s[4])
                if (self.print_motor_status_):
                    jt_pos_s = ' pos:%-9d'%(set_s[1])
                    jt_vel_s = ' v:%-7d'%(set_s[2])
                    jt_tor_s = ' mt:%-4d'%(set_s[3])
                    mt_tor_s = ' jt:%-7.1f'%(set_s[4])
                statusword_s = ' stat:%-15s'%bin(self.network[set_s[0]].tpdo['statusword'].raw) 
                di_s = ' DI:%s'%bin(self.network[set_s[0]].tpdo['digital_inputs'].raw) 
                print_string_s = ids_s + jt_pos_s + jt_vel_s + jt_tor_s + mt_tor_s + statusword_s + di_s

                ids_r = 'R-id:{0} '.format(set_r[0])
                jt_pos_r = ' pos:%-9.2f'%(RAD2DEG * set_r[1])
                jt_vel_r = ' v:%-7.2f'%(RAD2DEG * set_r[2])
                jt_tor_r = ' jt:%-7.1f'%(set_r[3])
                mt_tor_r = ' mt:%-4d'%(set_r[4])
                if (self.print_motor_status_):
                    jt_pos_r = ' pos:%-9d'%(set_r[1])
                    jt_vel_r = ' v:%-7d'%(set_r[2])
                    jt_tor_r = ' mt:%-4d'%(set_r[3])
                    mt_tor_r = ' jt:%-7.1f'%(set_r[4])
                statusword_r = ' stat:%-15s'%bin(self.network[set_r[0]].tpdo['statusword'].raw) 
                di_r = ' DI:%s'%bin(self.network[set_r[0]].tpdo['digital_inputs'].raw) 
                print_string_r = ids_r + jt_pos_r + jt_vel_r + jt_tor_r + mt_tor_r + statusword_r + di_r

                self._dprint(print_string_r)
                self._dprint(print_string_s)
        else:
            for node_id_ in self.node_list:
                id_ = 'id:{0} '.format(node_id_)
                pos_ = ' pos:%-8d'%self.network[node_id_].tpdo['position_actual_value'].raw
                vel_ = ' v:%-7d'%self.network[node_id_].tpdo['velocity_actual_value'].raw
                tor_ = ' t:%-4d'%self.network[node_id_].tpdo['torque_actual_value'].raw
                statusword_ = ' stat:%-15s'%bin(self.network[node_id_].tpdo['statusword'].raw) 
                di_ = ' DI:%s'%bin(self.network[node_id_].tpdo['digital_inputs'].raw) 
                print_string = id_ + pos_ + vel_ + tor_ + statusword_ + di_
                self._dprint(print_string)
        self._dprint('')

    def _pub_joint(self):
        self.js_.header.stamp = rospy.Time.now()
        self.db_position_[0] = (self.js_.header.stamp - self.start_time).to_sec()
        self.db_velocity_[0] = self.db_position_[0]
        for i in range(4):
            set_s, set_r = self._read_set(i)
            ## STEERING ##
            self.js_.position[set_s[0] - 1] = set_s[1]
            self.js_.velocity[set_s[0] - 1] = set_s[2]
            self.js_.effort[set_s[0] - 1] = set_s[3]
            ## ROLLING ##
            self.js_.position[set_r[0] - 1] = set_r[1]
            self.js_.velocity[set_r[0] - 1] = set_r[2]
            self.js_.effort[set_r[0] - 1] = set_r[3]
            ## DEBUG ##
            self.db_position_[set_r[0]] = set_r[1]
            self.db_position_[set_s[0]] = set_s[1]
            self.db_velocity_[set_r[0]] = set_r[2]
            self.db_velocity_[set_s[0]] = set_s[2]

        self.joint_pub.publish(self.js_)
        self.wrq.writerow(self.db_position_)
        self.wrqd.writerow(self.db_velocity_)

    @_try_except_decorator
    def _joint_callback(self, data):
        if (self.ready4control_ and len(data.name) == 8):
            self.lock_.acquire()
            self.sub_time = rospy.Time.now()
            if ((self.sub_time - data.header.stamp).to_sec() < 0.001):
                for i in range(4):
                    id_s = (i+1)*2   ## node_id
                    id_r = id_s - 1  ## node_id
                    if (self.control_target == 'target_torque'):
                        mt_val_s, mt_val_r = self._from_joint_torque_to_motor_torque(data.effort[id_s - 1], data.effort[id_r - 1])
                    elif (self.control_target == 'target_velocity'):
                        mt_val_s, mt_val_r = self._from_joint_velocity_to_motor_velocity(data.velocity[id_s - 1], data.velocity[id_r - 1], id_s, id_r)
                    self._control_command(id_s, self.control_target, mt_val_s)
                    self._control_command(id_r, self.control_target, mt_val_r)
                receive_call = data.position[0]
                # if (receive_call == 3.0): self.homing_call_ = True
                if (receive_call == 9.0): self.stop_call_ = True
                elif (receive_call == 95.0):
                    if self.is_cali_move_mode_: self.is_cali_move_mode_ = False
                    else: self.is_cali_move_mode_ = True
                    self.is_cali_mode_changed_ = True
            self.lock_.release()

    @_try_except_decorator
    def start_joint_publisher(self, operation_mode=OPMode.PROFILED_TORQUE):
        half_hz_ = int(HZ/2)
        r = rospy.Rate(HZ)
        tick = 0
        _, control_target = self._check_test_availability([1,2,3,4,5,6,7,8], operation_mode)

        while not rospy.is_shutdown():
            if (self.homing_call_):
                self.homing_call_ = False
                self.ready4control_ = False
                self._dprint('homing call received.')
                self._stop_operation()
                self.homing()
                for node_id_ in [1,2,3,4,5,6,7,8]:
                    self._control_command(node_id_, 'target_torque', 0.0)
                self._start_operation_mode(test_set=[1,2,3,4,5,6,7,8], operation_mode=operation_mode)
                self.wrdb.write('homing end time: {0}\n'.format((rospy.Time.now() - self.start_time).to_sec()))

            if (self.stop_call_):
                self._stop_operation()
                self.finish_work()
                return

            self.lock_.acquire() ## I am not sure if this works or helps
            self.network.sync.transmit()
            self.lock_.release()
            self._pub_joint()
            if ((rospy.Time.now() - self.sub_time ).to_sec() > 0.1):
                self.lock_.acquire()
                for node_id_ in [1,2,3,4,5,6,7,8]:
                    self._control_command(node_id_, control_target, 0.0)
                self.lock_.release()
            tick += 1
            if (tick % half_hz_ == 0): self._read_and_print()
            r.sleep()

    @_try_except_decorator
    def homing(self):
        r = rospy.Rate(HZ)
        tick = 0
        self._switch_on_mode(self.node_list)
        for i in range(4):
            h_node = i*2+2
            self._dprint("\n===\nhoming node: {0}\n===".format(h_node))
            for node_id_ in self.node_list:
                self.network[node_id_].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
                self.network[node_id_].rpdo[1].transmit()
                self._dprint('node {0} operation mode: {1}'.format(node_id_,self.network[node_id_].rpdo['modes_of_operation'].raw))
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.HOMING
            self.network[h_node].rpdo[1].transmit()
            self._dprint('node {0} operation mode: {1}'.format(h_node,self.network[h_node].rpdo['modes_of_operation'].raw))

            time.sleep(self.sleep_)
            self._rpdo_controlword(controlword=CtrlWord.ENABLE_OPERATION, node_id=h_node) ## Enable Operation command
            self._rpdo_controlword(controlword=0b11111, node_id=h_node, command='HOMING COMMAND') ## Operation command
            while not rospy.is_shutdown():
                self.network.sync.transmit()
                self._pub_joint()
                statusword = bin(self.network[h_node].tpdo['statusword'].raw)
                try:
                    statusword = statusword[2:]
                    if statusword[-13] == '1':
                        break
                except:
                    pass
                tick += 1
                if (tick % HZ == 0): self._read_and_print()
                r.sleep()

            self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=h_node) ## Disable Operation command
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
            self.network[h_node].rpdo[1].transmit()
            h_node = i*2+1
            self._dprint("\n===\nhoming node: {0}\n===".format(h_node))
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.HOMING
            self.network[h_node].rpdo[1].transmit()
            self._rpdo_controlword(controlword=CtrlWord.ENABLE_OPERATION, node_id=h_node) ## Enable Operation command
            self._rpdo_controlword(controlword=0b11111, node_id=h_node, command='HOMING COMMAND') ## Operation command
            while not rospy.is_shutdown():
                self.network.sync.transmit()
                self._pub_joint()
                statusword = bin(self.network[h_node].tpdo['statusword'].raw)
                try:
                    statusword = statusword[2:]
                    if statusword[-13] == '1':
                        break
                except:
                    pass
                tick += 1
                if (tick % HZ == 0): self._read_and_print()
                r.sleep()
            self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=h_node) ## Disable Operation command
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
            self.network[h_node].rpdo[1].transmit()

        self._dprint('homing performing finished!')
        self.network.sync.transmit()
        time.sleep(0.001)
        self.ready4control_ = True
        self._read_and_print()
        self._stop_operation()

    def _print_value(self, sec=5.0, iter=20):
        t_sleep_ = sec / iter
        for _ in range(int(iter)):
            self.network.sync.transmit()
            self._read_and_print()
            time.sleep(t_sleep_)

    def _disconnect_device(self):
        ## -----------------------------------------------------------------
        ## disconnect after use
        if self.is_disconnected_: return
        for node_id_ in self.node_list:
            self.network[node_id_].rpdo['modes_of_operation'].raw = -1
            self.network[node_id_].rpdo[1].transmit()

        self._rpdo_controlword(controlword=CtrlWord.SHUTDOWN)

        self._change_status(status='RESET COMMUNICATION')
        self._change_status(status='RESET')

        self.network.disconnect()
        self._dprint('==========================')
        self._dprint('Successfully disconnected!')
        self._dprint('==========================')
        self.is_disconnected_ = True

    def _stop_operation(self):
        ## -----------------------------------------------------------------
        ## stop operation
        if self.is_disconnected_: return
        self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION)

        for node_id_ in self.node_list:
            self.network[node_id_].rpdo['modes_of_operation'].raw = -1
            self.network[node_id_].rpdo[1].transmit()

        self._change_status(status='PRE-OPERATIONAL')

    def _check_test_availability(self, test_set, operation_mode):
        if self.cannot_test_:
            self._dprint('\n----\nTEST CANNOT BE PERFORMED\n----')
            return [False, None]
        for id_ in test_set:
            if id_ not in self.network:
                self._dprint('\n----\ntest node {0} is not in network\n----'.format(id_))
                self._disconnect_device()
                return [False, None]
        if operation_mode == OPMode.INTERPOLATED_POSITION:
            control_target = 'target_position'
        elif operation_mode == OPMode.PROFILED_POSITION:
            control_target = 'target_position'
        elif operation_mode == OPMode.PROFILED_VELOCITY:
            control_target = 'target_velocity'
        elif operation_mode == OPMode.PROFILED_TORQUE:
            control_target = 'target_torque'
        elif operation_mode == OPMode.HOMING:
            control_target = None
        elif operation_mode == OPMode.NO_MODE:
            control_target = None
        else:
            self._dprint('\n----\nOPERATION MODE ERROR\n----')
            self._disconnect_device()
            return [False, None]
        return [True, control_target]

    def _switch_on_mode(self, test_set):
        ## -----------------------------------------------------------------
        ## change network nmt state
        self._change_status(test_set=test_set, status='PRE-OPERATIONAL')
        self._change_status(test_set=test_set, status='OPERATIONAL')

        ## -----------------------------------------------------------------
        ## change drive mode
        self._rpdo_controlword(controlword=CtrlWord.SHUTDOWN, node_id=test_set) ## Shutdown command
        self._rpdo_controlword(controlword=CtrlWord.SWITCH_ON, node_id=test_set) ## Switch On command

    def _start_operation_mode(self, test_set, operation_mode):
        ## -----------------------------------------------------------------
        ## change drive mode to switch on mode
        self._switch_on_mode(test_set)

        ## -----------------------------------------------------------------
        ## change operation mode
        for node_id_ in test_set:
            self.network[node_id_].rpdo['modes_of_operation'].raw = operation_mode
            self.network[node_id_].rpdo[1].transmit()
            self._dprint('node {0} operation mode: {1}'.format(node_id_,self.network[node_id_].rpdo['modes_of_operation'].raw))
        self._dprint()
        time.sleep(self.sleep_)
        ## -----------------------------------------------------------------
        ## change drive mode
        self._rpdo_controlword(controlword=CtrlWord.ENABLE_OPERATION, node_id=test_set) ## Enable Operation command

    @_try_except_decorator
    def finish_work(self):
        self.wrdb.close()
        self.qd_open.close()
        self.q_open.close()
        self._disconnect_device()
        self.cannot_test_ = True

    ##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##

    @_try_except_decorator
    def _cali_value_changer(self, mt_tor_s, mt_tor_r, stationary_set):
        for moving_set in range(4):
            if (moving_set == stationary_set):
                pass
            else:
                self._control_command((moving_set + 1) * 2, 'target_torque', mt_tor_s)
                self._control_command(moving_set * 2 + 1, 'target_torque', mt_tor_r)

    @_try_except_decorator
    def _cali_recorder(self, time):
        half_hz_ = int(HZ/2)
        r = rospy.Rate(HZ)
        tick = 0
        while tick < time * HZ:
            self.network.sync.transmit()
            self._pub_joint()
            tick += 1
            if (tick % half_hz_ == 0):
                self._read_and_print()
            r.sleep()

    @_try_except_decorator
    def _cali_set_steer_angle(self, stationary_set, target):
        half_hz_ = int(HZ/2)
        r = rospy.Rate(HZ)
        stationary_steer = (stationary_set + 1) * 2

        set_s, _ = self._read_set(stationary_set)
        jt_pos_s = RAD2DEG * set_s[1]
        search_vel = 500
        if (target < jt_pos_s):
            search_vel = -500
        self._control_command(stationary_steer, 'target_velocity', search_vel)

        tick = 0
        changing_angle = True
        while changing_angle:
            self.network.sync.transmit()
            self._pub_joint()
            tick += 1
            if (tick % half_hz_ == 0):
                self._dprint('changing angle')
                self._read_and_print()
            set_s, _ = self._read_set(stationary_set)
            jt_pos_s = RAD2DEG * set_s[1]
            if ((jt_pos_s < target and search_vel < 0) or (jt_pos_s > target and search_vel > 0)):
                changing_angle = False
                self._control_command(stationary_steer, 'target_velocity', 0.0)
            r.sleep()

    @_try_except_decorator
    def _cali_mocap_set(self, stationary_set, target, st_tor_r, jt_tor_r, align_time, speed_up_time, play_time, is_second=False):
        stationary_steer = (stationary_set + 1) * 2
        stationary_roll = stationary_set * 2 + 1
        ## set steer angle
        if is_second:
            if self.is_cali_use_fixer == False:
                self._cali_recorder(2.0)
                mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, jt_tor_r)
                self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)
            else:
                os.system('spd-say "press enter after fixing"')
                a = input()

        self._cali_set_steer_angle(stationary_set=stationary_set,target=target)
        ## freeze stationary_set
        self._start_operation_mode(test_set=[stationary_roll], operation_mode=OPMode.PROFILED_VELOCITY)
        self._control_command(stationary_roll, 'target_velocity', 0.0)
        ## align other sets with hand by rotating it for now
        if is_second and self.is_cali_use_fixer == False:
            self._cali_recorder(1.0)
            self._cali_value_changer(0.0, 0.0, stationary_set)
            self._cali_recorder(speed_up_time)
        else:
            self._cali_value_changer(0.0, 0.0, stationary_set)
            if self.is_cali_use_fixer:
                os.system('spd-say "press enter after fixing"')
                a = input()
            os.system('spd-say "rotate by hand"')
            self._cali_recorder(align_time)

        ## speed up
        self.wrdb.write('speed up for time sync start time: {0}\n'.format((rospy.Time.now() - self.start_time).to_sec()))
        mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, st_tor_r)
        self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)
        self._cali_recorder(speed_up_time)
        ## stop for time sync
        self.wrdb.write('stop for time sync start time: {0}\n'.format((rospy.Time.now() - self.start_time).to_sec()))
        self._cali_value_changer(0.0, 0.0, stationary_set)
        self._cali_recorder(speed_up_time)
        ## main rotation
        self.wrdb.write('main rotation start time: {0}\n'.format((rospy.Time.now() - self.start_time).to_sec()))
        if self.is_cali_constant_torque:
            mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, st_tor_r)
            self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)
            self._cali_recorder(speed_up_time)
            mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, jt_tor_r)
            self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)
            self._cali_recorder(play_time)
        else:
            ## P controller
            half_hz_ = int(HZ/2)
            r = rospy.Rate(HZ)
            tick = 0
            refer_set = int((stationary_set + 2) % 4)
            lpf = self.db_velocity_[(refer_set+1) * 2]
            a = self.cali_lpf_sensitivity
            while tick < play_time * HZ:
                self.network.sync.transmit()
                self._pub_joint()
                lpf = a*lpf + (1-a)*self.db_velocity_[(refer_set+1) * 2]
                target_torque = jt_tor_r + self.cali_gain_kp * (self.cali_target_speed - lpf)
                mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, target_torque)
                self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)
                tick += 1
                if (tick % half_hz_ == 0):
                    self._read_and_print()
                r.sleep()

        self._cali_value_changer(0.0, 0.0, stationary_set)
        self.wrdb.write('main rotation end time: {0}\n'.format((rospy.Time.now() - self.start_time).to_sec()))
        self.wrdb.write('\n')

    ## set: (0 ~ 3)
    @_try_except_decorator
    def mocap_calibration(self, stationary_set, target_1=45.0, target_2=225.0, st_tor_r=1300, jt_tor_r=960):
        align_time = 4
        speed_up_time = 3
        play_time = 45
        stationary_steer = (stationary_set + 1) * 2
        stationary_roll = stationary_set * 2 + 1

        self.wrdb.write('stationary_set: {0}\n'.format(stationary_set))
        self.wrdb.write('target_1: {0}\n'.format(target_1))
        self.wrdb.write('target_2: {0}\n'.format(target_2))

        os.system('spd-say "starting calibration"')

        ## perform homing
        self.homing()
        ## torque mode only for rolling
        for node_id in range(1,9):
            if (node_id == stationary_roll or node_id == stationary_steer):
                pass
            else:
                self._start_operation_mode(test_set=[node_id], operation_mode=OPMode.PROFILED_TORQUE)
        self._cali_value_changer(0.0, 0.0, stationary_set)
        ## move to first position
        self._start_operation_mode(test_set=[stationary_steer], operation_mode=OPMode.PROFILED_VELOCITY)
        self.network.sync.transmit()
        self.wrdb.write('================= TARGET 1 =================\n')
        self._cali_mocap_set(stationary_set=stationary_set, target=target_1,
                              align_time=align_time, speed_up_time=speed_up_time, play_time=play_time,
                              st_tor_r=st_tor_r, jt_tor_r=jt_tor_r)
        ## move to second position
        self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=stationary_roll)
        self.network[stationary_roll].rpdo['modes_of_operation'].raw = -1
        self.network[stationary_roll].rpdo[1].transmit()
        self.wrdb.write('================= TARGET 2 =================\n')
        self._cali_mocap_set(stationary_set=stationary_set, target=target_2, is_second=self.is_cali_auto_second_steer,
                              align_time=align_time, speed_up_time=speed_up_time, play_time=play_time,
                              st_tor_r=st_tor_r, jt_tor_r=jt_tor_r)
        ## stop calibration
        self._cali_value_changer(0.0, 0.0, stationary_set)
        self._stop_operation()
        self._disconnect_device()

    ## set: (0 ~ 3)
    @_try_except_decorator
    def find_flat_ground(self, stationary_set, target_1=45.0, jt_tor_r=960):
        speed_up_time = 3
        r = rospy.Rate(HZ)
        half_hz_ = int(HZ/2)
        stationary_steer = (stationary_set + 1) * 2
        stationary_roll = stationary_set * 2 + 1

        ## perform homing
        self.homing()
        ## torque mode only for rolling
        for node_id in range(1,9):
            if (node_id == stationary_roll or node_id == stationary_steer): pass
            else: self._start_operation_mode(test_set=[node_id], operation_mode=OPMode.PROFILED_TORQUE)
        self._cali_value_changer(0.0, 0.0, stationary_set)
        ## move stationary steer
        self._start_operation_mode(test_set=[stationary_steer], operation_mode=OPMode.PROFILED_VELOCITY)
        self._cali_set_steer_angle(stationary_set=stationary_set,target=target_1)

        tick = 0
        os.system('spd-say "find flat ground"')
        while not rospy.is_shutdown():
            ## IF CHANGE MODE CALL RECEIVED
            if self.is_cali_mode_changed_:
                self.is_cali_mode_changed_ = False

                ## MOVE MODE: TURN OFF STATIONARY ROLL, SET 0 TORQUE
                if self.is_cali_move_mode_:
                    os.system('spd-say "move mode"')
                    self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=stationary_roll)
                    self.network[stationary_roll].rpdo['modes_of_operation'].raw = -1
                    self.network[stationary_roll].rpdo[1].transmit()
                    self._cali_value_changer(0.0, 0.0, stationary_set)

                ## ROTATE MODE: TURN ON STATIONARY ROLL AND SET 0 VELOCITY, SET TARGET_1 TORQUE
                else:
                    self._start_operation_mode(test_set=[stationary_roll], operation_mode=OPMode.PROFILED_VELOCITY)
                    self._control_command(stationary_roll, 'target_velocity', 0.0)
                    os.system('spd-say "rotate by hand"')
                    self._cali_recorder(speed_up_time)
                    mt_tor_s, mt_tor_r = self._from_desired_torque_to_motor_torque(0, jt_tor_r)
                    self._cali_value_changer(mt_tor_s, mt_tor_r, stationary_set)

            self.network.sync.transmit()
            self._pub_joint()
            tick += 1
            if (tick % half_hz_ == 0):
                self._read_and_print()
            r.sleep()

        ## STOP
        self._cali_value_changer(0.0, 0.0, stationary_set)
        self._stop_operation()
        self._disconnect_device()

    @_try_except_decorator
    def test_position_control_fail(self, test_id, goal_angle):
        self.homing()

        half_hz_ = int(HZ/2)
        r = rospy.Rate(HZ)

        _,control_target = self._check_test_availability([test_id], OPMode.INTERPOLATED_POSITION)

        self._start_operation_mode(test_set=[test_id], operation_mode=OPMode.INTERPOLATED_POSITION)
        self.network.sync.transmit()

        ## SETTING FOR INTERPOLATED_POSITION
        self._rpdo_controlword(controlword=0b000011111, node_id=test_id)
        ##

        goal_angle *= DEG2RAD
        rot_per_sec = 5 # DEGREE
        init_motor = self.network[test_id].tpdo['position_actual_value'].raw
        init_angle = C2R[test_id - 1] *  NI00 * init_motor
        tick_move = rot_per_sec * DEG2RAD * R2C[test_id - 1] / HZ
        if (goal_angle < init_angle): tick_move *= -1

        # self._control_command(test_id, control_target, 0.0)

        test_integrator = 0.0
        tick = 0
        while not rospy.is_shutdown():
            self.network.sync.transmit()
            self._pub_joint()
            tick += 1

            curr_motor = self.network[test_id].tpdo['position_actual_value'].raw
            curr_angle = C2R[test_id - 1] *  NI00 * curr_motor
            test_integrator += tick_move
            self._control_command(test_id, control_target, init_motor + test_integrator)

            if (tick % half_hz_ == 0):
                print('rotation per second: {0} DEG'.format(rot_per_sec))
                print('rotation per tick: {0} encoder count'.format(tick_move))
                print('diff curr - goal: {0} encoder count'.format(curr_motor - init_motor - test_integrator))
                self._dprint('changing angle')
                self._read_and_print()
            if ((curr_angle < goal_angle and tick_move < 0) or (curr_angle > goal_angle and tick_move > 0)):
                break
            r.sleep()

        print('FINISHED!!!')
        self._cali_recorder(30.0)

        self._stop_operation()

    @_try_except_decorator
    def test_single_elmo(self, test_id, operation_mode, value, play_time=10.0):
        _,control_target = self._check_test_availability([test_id], operation_mode)

        self._start_operation_mode(test_set=[test_id], operation_mode=operation_mode)
        # self._rpdo_controlword(controlword=0b000011111, node_id=test_id, command='OPERATION COMMAND') ## Operation command

        self._control_command(test_id, control_target, value)
        self._print_value(sec=play_time*0.5, iter=play_time)

        self._control_command(test_id, control_target, -value, "\nchanging command")
        self._print_value(sec=play_time*0.5, iter=play_time)

        self._stop_operation()

    @_try_except_decorator
    def test_dual_elmo(self, t1, t2, operation_mode, value):
        test_set = [t1, t2]
        _,control_target = self._check_test_availability(test_set, operation_mode)

        self._start_operation_mode(test_set=test_set, operation_mode=operation_mode)
        sec_=5.0

        self._control_command(t1, control_target, 0)
        self._control_command(t2, control_target, value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._control_command(t1, control_target, 0, "\nchanging command")
        self._control_command(t2, control_target, -value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._control_command(t1, control_target, value, "\nchanging command")
        self._control_command(t2, control_target, 0)
        self._print_value(sec=sec_, iter=sec_*2)

        self._control_command(t1, control_target, -value, "\nchanging command")
        self._control_command(t2, control_target, 0)
        self._print_value(sec=sec_, iter=sec_*2)

        self._control_command(t1, control_target, value, "\nchanging command")
        self._control_command(t2, control_target, -value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._control_command(t1, control_target, -value, "\nchanging command")
        self._control_command(t2, control_target, value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._stop_operation()

    @_try_except_decorator
    def test_multiple_elmo(self, test_set, operation_mode, value, play_time=5.0):
        _,control_target = self._check_test_availability(test_set, operation_mode)
        self._start_operation_mode(test_set=test_set, operation_mode=operation_mode)

        for node_id_ in test_set:
            self._control_command(node_id_, control_target, value)
        self._print_value(sec=play_time, iter=play_time*2)

        self._stop_operation()

    @_try_except_decorator
    def test_odd_and_even_elmo(self, test_set, operation_mode, value):
        _,control_target = self._check_test_availability(test_set, operation_mode)

        self._start_operation_mode(test_set=test_set, operation_mode=operation_mode)
        value_ = [value, -value]
        sec_ = 5.0

        for node_id_ in test_set:
            self._control_command(node_id_, control_target, value_[node_id_%2])
        self._print_value(sec=sec_, iter=sec_*2)
        self._stop_operation()

    @_try_except_decorator
    def test_homing_version_1(self, test_set, play_time=15.0):
        self._start_operation_mode(test_set=test_set, operation_mode=OPMode.HOMING)
        for node_id_ in test_set:
            self._rpdo_controlword(controlword=0b11111, node_id=node_id_, command='HOMING COMMAND') ## Operation command

        self._print_value(sec=play_time, iter=play_time*2)
        self._stop_operation()

    @_try_except_decorator
    def test_homing_version_2(self, test_set):
        r = rospy.Rate(HZ)
        tick = 0
        work_done = False
        work_each = []
        for _ in range(len(test_set)):
            work_each.append(False)

        self._start_operation_mode(test_set=test_set, operation_mode=OPMode.HOMING)
        for node_id_ in test_set:
            self._rpdo_controlword(controlword=0b11111, node_id=node_id_, command='HOMING COMMAND') ## Operation command

        while not rospy.is_shutdown():
            self.network.sync.transmit()
            self._pub_joint()
            for i in range(len(test_set)):
                statusword = bin(self.network[test_set[i]].tpdo['statusword'].raw)
                try:
                    statusword = statusword[2:]
                    if statusword[-13] == '1':
                        work_each[i] = True
                except:
                    pass

            work_done = True
            for work in work_each:
                if work == False: work_done = False

            tick += 1
            if (tick % HZ == 0): self._read_and_print()
            if work_done: break
            r.sleep()

        self._dprint('homing performing finished!')
        self.network.sync.transmit()
        self._read_and_print()
        self._stop_operation()

    @_try_except_decorator
    def set_free_wheel(self, test_set, play_time=30.0):
        _, _ = self._check_test_availability(test_set, OPMode.NO_MODE)
        self._switch_on_mode(test_set)
        self._print_value(sec=play_time, iter=play_time*2)
        self._stop_operation()

if __name__ == "__main__":
    ## operation mode for elmo(402.pdf)
    ## NO_MODE, PROFILED_POSITION, PROFILED_VELOCITY, PROFILED_TORQUE, HOMING, INTERPOLATED_POSITION
    ## motor Torque value : in_air - 130    on_ground - 
    ## joint Torque value : in_air - 600    on_ground - 1200
    ##     Velocity value : in_air - 600   on_ground - 600

    # node_set = [1]
    # node_set = [1,2]
    node_set = [1,2,3,4,5,6,7,8]

    tt_ = TestElmo(node_list=node_set)

    # tt_.test_single_elmo(test_id=node_set[0], operation_mode=OPMode.PROFILED_TORQUE, value=0, play_time=60.0)
    # tt_.test_single_elmo(test_id=node_set[1], operation_mode=OPMode.PROFILED_VELOCITY, value=0, play_time=60.0)

    # tt_.test_dual_elmo(t1=1, t2=2, operation_mode=OPMode.PROFILED_TORQUE, value=80)

    # tt_.test_multiple_elmo(test_set=[7,8], operation_mode=OPMode.PROFILED_TORQUE, value=120)
    # tt_.test_multiple_elmo(test_set=[4,6,8], operation_mode=OPMode.PROFILED_VELOCITY, value=-1000, play_time=60.0)
    # tt_.test_multiple_elmo(test_set=[1,3,5,7], operation_mode=OPMode.PROFILED_VELOCITY, value=0, play_time=60.0)

    # tt_.test_odd_and_even_elmo(test_set=[7,8], operation_mode=OPMode.PROFILED_TORQUE, value=120)
    # tt_.test_odd_and_even_elmo(test_set=node_set, operation_mode=OPMode.PROFILED_VELOCITY, value=1000)

    # tt_.test_homing_version_1(test_set=[2,4,6,8], play_time=10)
    # tt_.test_homing_version_1(test_set=[1,3,5,7], play_time=10)

    # tt_.test_homing_version_2(test_set=[2,4,6,8])
    # tt_.test_homing_version_2(test_set=[1,3,5,7])

    # tt_.set_free_wheel(test_set=node_set, play_time=5)

    tt_.start_joint_publisher()
