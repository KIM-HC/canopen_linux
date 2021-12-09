#!/usr/bin/env python3
import os.path
import rospy
from platform import node
from re import S, T
import time
import canopen
# from canopen.profiles.p402 import BaseNode402, OperationMode
import traceback
from sensor_msgs.msg import JointState
import math
import threading

PKG_PATH = '/home/khc/catkin_ws/src/canopen_linux/'

#### CONTROL INFORMATION ####
HZ = 300

#### ENCODER INFORMATION ####
####  ELMO 1 2 3 4 5 6   ####
REV2CNT = 2048.0
CNT2REV = 1.0 / REV2CNT
RAD2CNT = REV2CNT / (2.0 * math.pi)
CNT2RAD = (2.0 * math.pi) / REV2CNT

####      ELMO 7 8       ####
REV2CNT2 = 8192.0
CNT2REV2 = 1.0 / REV2CNT2
RAD2CNT2 = REV2CNT2 / (2.0 * math.pi)
CNT2RAD2 = (2.0 * math.pi) / REV2CNT2

C2R = [CNT2RAD, CNT2RAD, CNT2RAD, CNT2RAD2]

##### GEAR RATIO ####
NS = 4.0            ## steer
NT = 85.0 / 35.0    ## translate , 2.428571429
NW = 2.0            ## wheel (rolling)

N00 = 1.0 / NS
N01 = 1.0 / (NS * NW)
N11 = 1.0 / (NT * NW)

RAD2DEG = 180.0 / math.pi

class OPMode():
    NO_MODE =              -1
    PROFILED_POSITION =     1
    PROFILED_VELOCITY =     3
    PROFILED_TORQUE =       4
    HOMING =                6
    INTERPOLATED_POSITION = 7

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
    def __init__(self, node_list):
        self.ready4control_ = False
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

        ## -----------------------------------------------------------------
        ## basic setting
        self.node_list = node_list
        self.sleep_ = 0.005
        self.start_time = rospy.Time.now()
        self.sub_time = rospy.Time.now()
        self.lock_ = threading.Lock()
        self.cannot_test = False

        ## -----------------------------------------------------------------
        ## for debug
        self.db_ = open(PKG_PATH + 'debug/debug'+time.strftime('_%Y_%m_%d', time.localtime())+'.txt', 'a')
        ptime_ = time.strftime('==  DEBUG START %Y.%m.%d - %X', time.localtime())
        two_bar = ''
        for _ in range(len(ptime_)+4):
            two_bar = two_bar + '='
        self._dprint('\n\n\n' + two_bar + two_bar)
        self._dprint(ptime_ + '  ==' + two_bar)
        self._dprint(two_bar + two_bar + '\n')

        ## -----------------------------------------------------------------
        ## path for eds file of elmo
        self.EDS_PATH = PKG_PATH + 'eds_file/elmo.eds'  ## os.path.join(os.path.dirname(__file__), 'elmo.eds')

        ## -----------------------------------------------------------------
        ## one network per one CAN Bus
        self.network = canopen.Network()

        self._connect_network()

    ## catches errors and writes it in debug file
    def _try_except_decorator(func):
        def decorated(self, *args, **kwargs):
            if self.cannot_test:
                self._dprint('\n----\nFUNCTION: {0}\nTEST CANNOT BE PERFORMED (decorator)\n----'.format(func.__name__))
            else:
                try:
                    func(self, *args, **kwargs)

                except KeyboardInterrupt:
                    self._dprint('KeyboardInterrupt!! stopping device...')
                    self._stop_operation()
                    self._disconnect_device()

                except canopen.SdoCommunicationError as e:
                    self.cannot_test = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR SdoCommunicationError\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))

                except canopen.SdoAbortedError as e:
                    self.cannot_test = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR SdoAbortedError\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))

                except Exception as e:
                    self.cannot_test = True
                    self._dprint('\n----\nFUNCTION: {2}\nERROR\n----\n{0}\n\n{1}'.format(e, traceback.format_exc(),func.__name__))
        return decorated

    @_try_except_decorator
    def _connect_network(self):
        ## -----------------------------------------------------------------
        ## connect to CAN Bus (PEAK PCAN-USB(125kbps)

        ## using peak driver command pcaninfo will show channel(bus channel) name
        # self.network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=125000)
        ## using socketcan (same PEAK PCAN-USB)
        # self.network.connect(bustype='socketcan', channel='can0', bitrate=125000)
        # self.network.connect(bustype='socketcan', channel='can0', bitrate=500000)
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
                self.cannot_test = True
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
                self.network[node_id_].sdo['homing_speeds'][1].raw = 1200
                self.network[node_id_].sdo['homing_speeds'][2].raw = 1000
                if (node_id_ == 8):
                    self.network[node_id_].sdo['homing_speeds'][1].raw = 1200 * 4
                    self.network[node_id_].sdo['homing_speeds'][2].raw = 1000 * 4
                self.network[node_id_].sdo['homing_acceleration'].raw = self.network[node_id_].sdo['profile_acceleration'].raw
            else:
                self.network[node_id_].sdo['homing_method'].raw = 35
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
            self._dprint("controlword: {0}".format(bin(self.network[node_id_].sdo['controlword'].raw)))
            self._dprint("modes_of_operation: {0}".format(self.network[node_id_].sdo['modes_of_operation'].raw))
            self._dprint("Producer Heartbeat Time: {0}".format(self.network[node_id_].sdo['Producer Heartbeat Time'].raw))
            self._dprint("Communication Cycle Period: {0}".format(self.network[node_id_].sdo['Communication Cycle Period'].raw))

            self._dprint('\n- homing')
            self._dprint("home_offset: {0}".format(self.network[node_id_].sdo['home_offset'].raw))
            self._dprint("homing_method: {0}".format(self.network[node_id_].sdo['homing_method'].raw))
            self._dprint("homing_speeds search switch: {0}".format(self.network[node_id_].sdo['homing_speeds'][1].raw))
            self._dprint("homing_speeds search zero: {0}".format(self.network[node_id_].sdo['homing_speeds'][2].raw))
            self._dprint("homing_acceleration: {0}".format(self.network[node_id_].sdo['homing_acceleration'].raw))

            self._dprint('\n- control')
            self._dprint("target_velocity: {0}".format(self.network[node_id_].sdo['target_velocity'].raw))
            self._dprint("target_torque: {0}".format(self.network[node_id_].sdo['target_torque'].raw))
            self._dprint("target_position: {0}".format(self.network[node_id_].sdo['target_position'].raw))
            self._dprint("profile_velocity: {0}".format(self.network[node_id_].sdo['profile_velocity'].raw))
            self._dprint("position_demand_value: {0}".format(self.network[node_id_].sdo['position_demand_value'].raw))
            self._dprint("position_actual_internal_value: {0}".format(self.network[node_id_].sdo['position_actual_internal_value'].raw))
            self._dprint("position_actual_value: {0}".format(self.network[node_id_].sdo['position_actual_value'].raw))
            self._dprint("profile_acceleration: {0}".format(self.network[node_id_].sdo['profile_acceleration'].raw))
            self._dprint("profile_deceleration: {0}".format(self.network[node_id_].sdo['profile_deceleration'].raw))

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

    def _change_status(self, test_set=None, status='PRE-OPERATIONAL', sleep_time=0.08):
        self._dprint('\nstate command: {0}'.format(status))
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
        if (node_id in [7,8] and control_method is "target_velocity"):
            value = value * 4
        self.network[node_id].rpdo[control_method].raw = value
        self.network[node_id].rpdo[CtrlPDO()[control_method]].transmit()

    def _rpdo_controlword(self, controlword, node_id = None, command='===UNKNOWN COMMAND==='):
        if controlword is 0b10000000:
            self._dprint('\nFAULT RESET COMMAND: 0b10000000')
        elif controlword is 0b10000001:
            self._dprint('\nSWITCH ON COMMAND: 0b10000001')
        elif controlword is 0b110:
            self._dprint('\nSHUTDOWN COMMAND: 0b110')
        elif controlword is 0b111:
            self._dprint('\n[SWITCH ON or DISABLE OPERATION] COMMAND: 0b111')
        elif controlword is 0b1111:
            self._dprint('\n[ENABLE OPERATION or OPERATION] COMMAND: 0b1111')
        elif controlword is 0b11111:
            self._dprint('\nOPERATION COMMAND: 0b11111')
        else:
            self._dprint('{0}: {1}'.format(command, controlword))

        if node_id is None:
            for node_id_ in self.network:
                self.network[node_id_].rpdo['controlword'].raw = controlword
                self.network[node_id_].rpdo[1].transmit()
            time.sleep(self.sleep_)
            self.network.sync.transmit()
            for node_id_ in self.network:
                self._dprint('node_id: {0}'.format(node_id_))
                self._dprint("statusword: {0}".format(bin(self.network[node_id_].tpdo['statusword'].raw)))
                self._dprint("controlword: {0}".format(bin(self.network[node_id_].rpdo['controlword'].raw)))
                # self._check_statusword(bin(self.network[node_id].tpdo['statusword'].raw))
        else:
            self.network[node_id].rpdo['controlword'].raw = controlword
            self.network[node_id].rpdo[1].transmit()
            time.sleep(self.sleep_)
            self.network.sync.transmit()
            self._dprint('node_id: {0}'.format(node_id))
            self._dprint("statusword: {0}".format(bin(self.network[node_id].tpdo['statusword'].raw)))
            self._dprint("controlword: {0}".format(bin(self.network[node_id].rpdo['controlword'].raw)))
            # self._check_statusword(bin(self.network[node_id].tpdo['statusword'].raw))

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

    def _read_and_print(self):
        self._dprint('time: %-6.2f'%((rospy.Time.now() - self.start_time).to_sec()))
        if (self.node_list == [1,2,3,4,5,6,7,8]):
            for i in range(4):
                id_s = (i+1)*2  ## steering node
                ids_s = 'S-id:{0} '.format(id_s)
                pos_s = ' pos:%-8.2f'%(RAD2DEG * C2R[i] * N00 * self.network[id_s].tpdo['position_actual_value'].raw)
                vel_s = ' v:%-7.2f'%(RAD2DEG * C2R[i] * N00 * self.network[id_s].tpdo['velocity_actual_value'].raw)
                tor_s = ' t:%-4d'%self.network[id_s].tpdo['torque_actual_value'].raw
                statusword_s = ' stat:%-15s'%bin(self.network[id_s].tpdo['statusword'].raw) 
                di_s = ' DI:%s'%bin(self.network[id_s].tpdo['digital_inputs'].raw) 
                print_string_s = ids_s + pos_s + vel_s + tor_s + statusword_s + di_s

                id_r = id_s - 1 ## rolling node
                ids_r = 'R-id:{0} '.format(id_r)
                pos_r = ' pos:%-8.2f'%(RAD2DEG * C2R[i] * (N01 * self.network[id_s].tpdo['position_actual_value'].raw
                                             + N11 * self.network[id_r].tpdo['position_actual_value'].raw))
                vel_r = ' v:%-7.2f'%(RAD2DEG * C2R[i] * (N01 * self.network[id_s].tpdo['velocity_actual_value'].raw
                                             + N11 * self.network[id_r].tpdo['velocity_actual_value'].raw))
                tor_r = ' t:%-4d'%self.network[id_r].tpdo['torque_actual_value'].raw
                statusword_r = ' stat:%-15s'%bin(self.network[id_r].tpdo['statusword'].raw) 
                di_r = ' DI:%s'%bin(self.network[id_r].tpdo['digital_inputs'].raw) 
                print_string_r = ids_r + pos_r + vel_r + tor_r + statusword_r + di_r

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
        for i in range(4):
            s = (i+1)*2  ## steering node
            idx_s = s-1  ## steering index
            ## STEERING ##
            self.js_.velocity[idx_s] = C2R[i] * N00 * self.network[s].tpdo['velocity_actual_value'].raw
            self.js_.effort[idx_s] = self.network[s].tpdo['torque_actual_value'].raw
            self.js_.position[idx_s] = C2R[i] * N00 * self.network[s].tpdo['position_actual_value'].raw
            ## ROLLING ##
            self.js_.velocity[idx_s - 1] = C2R[i] * (N01 * self.network[s].tpdo['velocity_actual_value'].raw
                                                   + N11 * self.network[s-1].tpdo['velocity_actual_value'].raw)
            self.js_.effort[idx_s - 1] = self.network[s-1].tpdo['torque_actual_value'].raw
            self.js_.position[idx_s - 1] = C2R[i] * (N01 * self.network[s].tpdo['position_actual_value'].raw
                                                   + N11 * self.network[s-1].tpdo['position_actual_value'].raw)
        self.joint_pub.publish(self.js_)

    def _print_value(self, sec=5.0, iter=20):
        t_sleep_ = sec / iter
        for _ in range(int(iter)):
            self.network.sync.transmit()
            self._read_and_print()
            time.sleep(t_sleep_)

    def _disconnect_device(self):
        ## -----------------------------------------------------------------
        ## disconnect after use
        for node_id_ in self.node_list:
            self.network[node_id_].rpdo['modes_of_operation'].raw = -1
            self.network[node_id_].rpdo[1].transmit()
            self._rpdo_controlword(controlword=CtrlWord.SHUTDOWN, node_id=node_id_)

        self._change_status(status='RESET COMMUNICATION')
        self._change_status(status='RESET')

        self.network.disconnect()
        self._dprint('==========================')
        self._dprint('Successfully disconnected!')
        self._dprint('==========================')

    def _stop_operation(self):
        ## -----------------------------------------------------------------
        ## stop operation
        for node_id_ in self.node_list:
            self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=node_id_)

        for node_id_ in self.node_list:
            self.network[node_id_].rpdo['modes_of_operation'].raw = -1
            self.network[node_id_].rpdo[1].transmit()

        self._change_status(status='PRE-OPERATIONAL')

    def _check_test_availability(self, test_set, operation_mode):
        if self.cannot_test:
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
        for node_id_ in test_set:
            self._rpdo_controlword(controlword=CtrlWord.SHUTDOWN, node_id=node_id_) ## Shutdown command
            self._rpdo_controlword(controlword=CtrlWord.SWITCH_ON, node_id=node_id_) ## Switch On command

    def _start_operation_mode(self, test_set, operation_mode):
        ## -----------------------------------------------------------------
        ## change drive mode to switch on mode
        self._switch_on_mode(test_set)

        ## -----------------------------------------------------------------
        ## change operation mode
        for node_id_ in test_set:
            self.network[node_id_].rpdo['modes_of_operation'].raw = operation_mode
            self.network[node_id_].rpdo[1].transmit()
            self._dprint('operation mode: {0}'.format(self.network[node_id_].rpdo['modes_of_operation'].raw))
        time.sleep(self.sleep_)
        ## -----------------------------------------------------------------
        ## change drive mode
        for node_id_ in test_set:
            self._rpdo_controlword(controlword=CtrlWord.ENABLE_OPERATION, node_id=node_id_) ## Enable Operation command


    ##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##

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
    def test_multiple_elmo(self, test_set, operation_mode, value):
        _,control_target = self._check_test_availability(test_set, operation_mode)

        self._start_operation_mode(test_set=test_set, operation_mode=operation_mode)
        sec_=5.0

        for node_id_ in test_set:
            self._control_command(node_id_, control_target, value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._dprint('\nchanging command')
        for node_id_ in test_set:
            self._control_command(node_id_, control_target, -value)
        self._print_value(sec=sec_, iter=sec_*2)

        self._stop_operation()

    @_try_except_decorator
    def test_odd_and_even_elmo(self, test_set, operation_mode, value):
        _,control_target = self._check_test_availability(test_set, operation_mode)

        self._start_operation_mode(test_set=test_set, operation_mode=operation_mode)
        value_ = [value, 0]
        sec_ = 5.0

        for node_id_ in test_set:
            self._control_command(node_id_, control_target, value_[node_id_%2])
        self._print_value(sec=sec_, iter=sec_*2)

        self._dprint('\nchanging command')
        for node_id_ in test_set:
            self._control_command(node_id_, control_target, value_[(node_id_+1)%2])
        self._print_value(sec=sec_, iter=sec_*2)

        self._stop_operation()

    @_try_except_decorator
    def test_homing(self, test_set, play_time=15.0):
        self._start_operation_mode(test_set=test_set, operation_mode=OPMode.HOMING)
        for node_id_ in test_set:
            self._rpdo_controlword(controlword=0b11111, node_id=node_id_, command='HOMING COMMAND') ## Operation command

        self._print_value(sec=play_time, iter=play_time*2)
        self._stop_operation()

    @_try_except_decorator
    def set_free_wheel(self, test_set, play_time=30.0):
        _, _ = self._check_test_availability(test_set, OPMode.NO_MODE)
        self._switch_on_mode(test_set)
        self._print_value(sec=play_time, iter=play_time*2)
        self._stop_operation()

    @_try_except_decorator
    def perform_homing(self, test_set):
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
    def homing(self):
        r = rospy.Rate(HZ)
        tick = 0
        self._switch_on_mode(self.node_list)
        for i in range(4):
            h_node = i*2+2
            for node_id_ in self.node_list:
                self.network[node_id_].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
                self.network[node_id_].rpdo[1].transmit()
                self._dprint('operation mode: {0}'.format(self.network[node_id_].rpdo['modes_of_operation'].raw))
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.HOMING
            self.network[h_node].rpdo[1].transmit()
            self._dprint('operation mode: {0}'.format(self.network[h_node].rpdo['modes_of_operation'].raw))
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

            self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=h_node) ## Disavle Operation command
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
            self.network[h_node].rpdo[1].transmit()
            h_node = i*2+1
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
            self._rpdo_controlword(controlword=CtrlWord.DISABLE_OPERATION, node_id=h_node) ## Disavle Operation command
            self.network[h_node].rpdo['modes_of_operation'].raw = OPMode.NO_MODE
            self.network[h_node].rpdo[1].transmit()

        self._dprint('homing performing finished!')
        self.network.sync.transmit()
        time.sleep(0.001)
        self.ready4control_ = True
        self._read_and_print()
        self._stop_operation()

    @_try_except_decorator
    def start_joint_publisher(self, operation_mode=OPMode.PROFILED_TORQUE):
        half_hz_ = int(HZ/2)
        r = rospy.Rate(HZ)
        tick = 0

        _,control_target = self._check_test_availability([1,2,3,4,5,6,7,8], operation_mode)
        for node_id_ in [1,2,3,4,5,6,7,8]:
            self._control_command(node_id_, control_target, 0.0)
        self._start_operation_mode(test_set=[1,2,3,4,5,6,7,8], operation_mode=operation_mode)

        while not rospy.is_shutdown():
            self.lock_.acquire()
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
    def _joint_callback(self, data):
        if (self.ready4control_ and len(data.name) == 8):
            self.lock_.acquire()
            self.sub_time = rospy.Time.now()
            if ((self.sub_time - data.header.stamp).to_sec() < 0.001):
                for i in range(8):
                    self._control_command(i+1, 'target_torque', data.effort[i])
            self.lock_.release()

    @_try_except_decorator
    def finish_work(self):
        self._disconnect_device()
        self.cannot_test = True

if __name__ == "__main__":
    ## operation mode for elmo(402.pdf)
    ## NO_MODE, PROFILED_POSITION, PROFILED_VELOCITY, PROFILED_TORQUE, HOMING, INTERPOLATED_POSITION
    ## Torque value : 150
    ## Velocity value : 3000

    # node_set = [1]
    # node_set = [1,2]
    node_set = [1,2,3,4,5,6,7,8]

    tt_ = TestElmo(node_list=node_set)

    # tt_.test_single_elmo(test_id=node_set[0], operation_mode=OPMode.PROFILED_TORQUE, value=150)
    # tt_.test_single_elmo(test_id=node_set[1], operation_mode=OPMode.PROFILED_VELOCITY, value=0, play_time=60.0)

    # tt_.test_dual_elmo(t1=1, t2=2, operation_mode=OPMode.PROFILED_TORQUE, value=80)

    # tt_.test_multiple_elmo(test_set=node_set, operation_mode=OPMode.PROFILED_TORQUE, value=120)
    # tt_.test_multiple_elmo(test_set=node_set, operation_mode=OPMode.PROFILED_VELOCITY, value=1000)

    # tt_.test_odd_and_even_elmo(test_set=node_set, operation_mode=OPMode.PROFILED_TORQUE, value=150)
    # tt_.test_odd_and_even_elmo(test_set=node_set, operation_mode=OPMode.PROFILED_VELOCITY, value=1000)

    # tt_.test_homing(test_set=[2,4,6,8], play_time=10)
    # tt_.test_homing(test_set=[1,3,5,7], play_time=10)

    # tt_.perform_homing(test_set=[2,4,6,8])
    # tt_.perform_homing(test_set=[1,3,5,7])

    # tt_.set_free_wheel(test_set=node_set, play_time=60)

    tt_.homing()
    tt_.start_joint_publisher()
