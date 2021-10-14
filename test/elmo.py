import os.path
from platform import node
from re import T
import time
import canopen


class TestElmo():
    def __init__(self):
        ## -----------------------------------------------------------------
        ## node id for testing
        test_id = 4

        ## -----------------------------------------------------------------
        ## for debug
        self.db_ = open('debug.txt', 'a')
        ptime_ = time.strftime('==  DEBUG START %Y.%m.%d - %X', time.localtime())
        two_bar = ''
        for _ in range(len(ptime_)+4):
            two_bar = two_bar + '='
        self.dprint('\n\n\n' + two_bar + two_bar)
        self.dprint(ptime_ + '  ==' + two_bar)
        self.dprint(two_bar + two_bar + '\n')

        ## -----------------------------------------------------------------
        ## path for eds file of elmo
        EDS_PATH = os.path.join(os.path.dirname(__file__), 'elmo.eds')

        ## -----------------------------------------------------------------
        ## one network per one CAN Bus
        self.network = canopen.Network()

        ## -----------------------------------------------------------------
        ## connect to CAN Bus (PEAK PCAN-USB(125kbps) or IXXAT)
        ## command pcaninfo will show channel(bus channel) name
        self.network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=125000)

        # ## -----------------------------------------------------------------
        # ## clean and reconnect
        # self.network.bus.flush_tx_buffer()
        # self.network.bus.stop_all_periodic_tasks()
        # # self.network.bus.shutdown()
        # self.network.clear()
        # self.network.update()
        # self.network.disconnect()
        # self.network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=125000)

        ## -----------------------------------------------------------------
        ## attempt to read SDO from nodes 1 - 127
        ## add found node
        self.network.scanner.search()
        time.sleep(0.05)
        for node_id in self.network.scanner.nodes:
            self.dprint("Found node %d!" % node_id)
            # local_node_ = canopen.LocalNode(node_id=node_id, object_dictionary=EDS_PATH)
            # self.network.add_node(local_node_)
            self.network.add_node(node=node_id, object_dictionary=EDS_PATH)

        ## -----------------------------------------------------------------
        ## check bus state
        self.dprint('state: {0}'.format(self.network.bus.state))
        self.dprint('bus channel_info: {0}'.format(self.network.bus.channel_info))

        ## -----------------------------------------------------------------
        ## send out time message
        self.network.time.transmit()

        ## -----------------------------------------------------------------
        ## receive message
        msg = self.network.bus.recv(timeout=1.0)
        self.dprint('msg: {0}'.format(msg))

        # ## -----------------------------------------------------------------
        # ## send nmt start to all nodes
        # self.network.send_message(0x0, [0x1, 0])

        ## -----------------------------------------------------------------
        ## node with test_id id is needed from now on
        if (test_id not in self.network):
            self.dprint("testing node id is not in network. EXITING...")
            self.network.disconnect()
            return

        ## -----------------------------------------------------------------
        ## set SDO value
        self.dprint('master state: ' + self.network.nmt.state)
        self.dprint('node state: ' + self.network[test_id].nmt.state)
        self.network.nmt.state = 'INITIALISING'
        self.network[test_id].nmt.state = 'INITIALISING'
        time.sleep(0.5)
        self.network.nmt.state = 'PRE-OPERATIONAL'
        self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.5)

        self.network[test_id].sdo['Producer Heartbeat Time'].raw = 1000
        ## Communication Cycle Period: what is it?
        self.network[test_id].sdo['Communication Cycle Period'].raw = 0


        # ## -----------------------------------------------------------------
        # ## transmit sync every 100 ms
        # self.network.sync.start(0.1)

        # ## -----------------------------------------------------------------
        # ## configure state machine by searching for TPDO that has statusword
        # self.network[test_id].setup_402_state_machine() # not tested

        ## -----------------------------------------------------------------
        ## read current PDO & SDO configuration
        self.network[test_id].tpdo.read()
        self.network[test_id].rpdo.read()
        self.dprint("\n===========\nInformation")
        self.dprint("error_code: {0}".format(self.network[test_id].sdo['error_code'].raw))
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))
        self.dprint("CAN Open Node ID: {0}".format(self.network[test_id].sdo['CAN Open Node ID'].raw))
        self.dprint("modes_of_operation: {0}".format(self.network[test_id].sdo['modes_of_operation'].raw))
        self.dprint("Producer Heartbeat Time: {0}".format(self.network[test_id].sdo['Producer Heartbeat Time'].raw))
        self.dprint("Communication Cycle Period: {0}".format(self.network[test_id].sdo['Communication Cycle Period'].raw))
        self.dprint("target_position: {0}".format(self.network[test_id].sdo['target_position'].raw))
        self.dprint("profile_acceleration: {0}".format(self.network[test_id].sdo['profile_acceleration'].raw))
        self.dprint("profile_deceleration: {0}".format(self.network[test_id].sdo['profile_deceleration'].raw))

        self.dprint('\n- option setting')
        self.dprint("quick_stop_option_code: {0}".format(self.network[test_id].sdo['quick_stop_option_code'].raw))
        self.dprint("shutdown_option_code: {0}".format(self.network[test_id].sdo['shutdown_option_code'].raw))
        self.dprint("disable_operation_option_code: {0}".format(self.network[test_id].sdo['disable_operation_option_code'].raw))
        self.dprint("halt_option_code: {0}".format(self.network[test_id].sdo['halt_option_code'].raw))
        self.dprint("fault_reaction_option_code: {0}".format(self.network[test_id].sdo['fault_reaction_option_code'].raw))

        self.dprint('\n- position control')
        self.dprint("profile_velocity: {0}".format(self.network[test_id].sdo['profile_velocity'].raw))
        self.dprint("position_demand_value: {0}".format(self.network[test_id].sdo['position_demand_value'].raw))
        self.dprint("position_actual_internal_value: {0}".format(self.network[test_id].sdo['position_actual_internal_value'].raw))
        self.dprint("position_actual_value: {0}".format(self.network[test_id].sdo['position_actual_value'].raw))
        self.dprint("following_error_window: {0}".format(self.network[test_id].sdo['following_error_window'].raw))
        self.dprint("following_error_time_out: {0}".format(self.network[test_id].sdo['following_error_time_out'].raw))
        self.dprint("position_window: {0}".format(self.network[test_id].sdo['position_window'].raw))
        self.dprint("position_window_time: {0}".format(self.network[test_id].sdo['position_window_time'].raw))
        self.dprint("Position Demand Internal Value: {0}".format(self.network[test_id].sdo['Position Demand Internal Value'].raw))

        self.dprint('\n- torque control')
        self.dprint("target_torque: {0}".format(self.network[test_id].sdo['target_torque'].raw))
        self.dprint("max_torque: {0}".format(self.network[test_id].sdo['max_torque'].raw))
        self.dprint("max_current: {0}".format(self.network[test_id].sdo['max_current'].raw))
        self.dprint("torque_actual_value: {0}".format(self.network[test_id].sdo['torque_actual_value'].raw))


        error_log = self.network[test_id].sdo[0x1003]
        for error in error_log.values():
            self.dprint("Error {0} was found in the log".format(hex(error.raw)))

        self.dprint("\n=======\nTPDO[1]")
        self.dprint("Num Entry: {0}".format(self.network[test_id].sdo['Transmit PDO Mapping Parameter 0'][0].raw))
        self.dprint("Status Word: {0}".format(bin(self.network[test_id].sdo['Transmit PDO Mapping Parameter 0'][1].raw)))
        self.dprint("Mapping Entry 2: {0}".format(self.network[test_id].sdo['Transmit PDO Mapping Parameter 0'][2].raw))
        self.dprint("COB ID: {0}".format(self.network[test_id].sdo['Transmit PDO Communication Parameter 0'][1].raw))
        self.dprint("Transmission Type: {0}".format(self.network[test_id].sdo['Transmit PDO Communication Parameter 0'][2].raw))
        self.dprint("Inhibit Time: {0}".format(self.network[test_id].sdo['Transmit PDO Communication Parameter 0'][3].raw))
        self.dprint("Event Timer: {0}".format(self.network[test_id].sdo['Transmit PDO Communication Parameter 0'][5].raw))

        self.dprint("\n=======\nRPDO[1]")
        self.dprint("Num Entry: {0}".format(self.network[test_id].sdo['Receive PDO Mapping Parameter 0'][0].raw))
        self.dprint("Mapping Entry 1: {0}".format(self.network[test_id].sdo['Receive PDO Mapping Parameter 0'][1].raw))
        self.dprint("COB ID: {0}".format(self.network[test_id].sdo['Receive PDO Communication Parameter 0'][0].raw))
        self.dprint("Transmission Type: {0}".format(self.network[test_id].sdo['Receive PDO Communication Parameter 0'][0].raw))

        # ## -----------------------------------------------------------------
        # ## read position data
        # self.dprint("\nchecking position reading . . .")
        # for _ in range(100):
        #     self.dprint("position_actual_value: {0}".format(self.network[test_id].sdo['position_actual_value'].raw))
        #     self.dprint("velocity_actual_value: {0}".format(self.network[test_id].sdo['velocity_actual_value'].raw))
        #     time.sleep(0.1)

        ## -----------------------------------------------------------------
        ## change TPDO configuration
        self.network[test_id].tpdo[1].clear()
        self.network[test_id].tpdo[1].add_variable('position_actual_value')
        self.network[test_id].tpdo[1].add_variable('velocity_actual_value')
        self.network[test_id].tpdo[1].trans_type = 254
        self.network[test_id].tpdo[1].event_timer = 100
        self.network[test_id].tpdo[1].inhibit_time = 5
        self.network[test_id].tpdo[1].enabled = True

        self.network[test_id].tpdo[2].clear()
        self.network[test_id].tpdo[2].add_variable('statusword')
        self.network[test_id].tpdo[2].add_variable('torque_actual_value')
        self.network[test_id].tpdo[2].trans_type = 254
        self.network[test_id].tpdo[2].event_timer = 100
        self.network[test_id].tpdo[2].inhibit_time = 5
        self.network[test_id].tpdo[2].enabled = True

        self.network[test_id].tpdo.save()

        ## -----------------------------------------------------------------
        ## change RPDO configuration
        self.network[test_id].rpdo[1].clear()
        self.network[test_id].rpdo[1].add_variable('modes_of_operation')
        self.network[test_id].rpdo[1].trans_type = 254 ## seems it doesnt work
        self.network[test_id].rpdo[1].enabled = True

        self.network[test_id].rpdo[2].clear()
        control_target = 'target_position'
        # control_target = 'target_velocity'
        # control_target = 'target_torque'
        self.network[test_id].rpdo[2].add_variable(control_target)
        self.network[test_id].rpdo[2].trans_type = 254 ## seems it doesnt work
        self.network[test_id].rpdo[2].enabled = True

        self.network[test_id].rpdo.save()

        ## -----------------------------------------------------------------
        ## change network nmt state
        self.network.nmt.state = 'PRE-OPERATIONAL'
        self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.1)
        self.network.nmt.state = 'OPERATIONAL'
        self.network[test_id].nmt.state = 'OPERATIONAL'
        time.sleep(0.1)
        self.print_status()

        ## -----------------------------------------------------------------
        ## control test
        test_ctrl = 0b110 ## Shutdown command
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('SHUTDOWN COMMAND')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))

        test_ctrl = 0b111 ## Switch On command
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('SWITCH ON COMMAND')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))

        test_ctrl = 0b1111 ## Enable Operation command
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('ENABLE OPERATION COMMAND')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))


        ## -----------------------------------------------------------------
        ## operation mode (402.pdf)
        ## -1:no mode / 1:profile position / 3:profiled velocity
        ## 4:torque profiled / 6:homing / 7:interpolated position
        self.network[test_id].sdo['modes_of_operation'].raw = 1

        ## -----------------------------------------------------------------
        ## Profile Position Mode (402.pdf p.34)
        ## bit0-3: 1 (Enable Operation command)
        ## bit4: New set-point
        ## bit5: Change set immediately
        ## bit6: abs/rel
        ## bit8: Halt
        test_ctrl = 0b000001111 ## Profile Position Mode
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('PROFILE POSITION MODE')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))
        self.dprint("modes_of_operation: {0}".format(self.network[test_id].sdo['modes_of_operation'].raw))

        self.dprint('========================')

        ## -----------------------------------------------------------------
        ## start periodic transmission of message in a background thread.
        self.network[test_id].rpdo[1].start(0.1)
        self.network[test_id].rpdo[2].start(0.1)

        ## -----------------------------------------------------------------
        ## transmit message once
        # self.network[test_id].rpdo[control_target].raw = 1
        self.network[test_id].tpdo[1].transmit()
        self.network[test_id].tpdo[2].transmit()
        self.network[test_id].rpdo[1].transmit()
        self.network[test_id].rpdo[2].transmit()

        ## -----------------------------------------------------------------
        ## update periodic message with new data
        self.network[test_id].rpdo[1].update()
        self.network[test_id].rpdo[2].update()

        ## -----------------------------------------------------------------
        ## Read values and save to a file
        tt_ = time.time()
        for _ in range(5):
            self.network[test_id].tpdo[1].wait_for_reception(timeout=1) ## seems like it is not receiveing?
            # self.network[test_id].tpdo[2].wait_for_reception(timeout=3)
            pos_ = self.network[test_id].tpdo['position_actual_value'].raw
            vel_ = self.network[test_id].tpdo['velocity_actual_value'].raw
            tor_ = self.network[test_id].tpdo['torque_actual_value'].raw
            statusword_ = self.network[test_id].tpdo['statusword'].raw
            test = 'time: %-6.3f'%(time.time() - tt_) + 'pos: %-11d'%pos_ + 'vel: %-8d'%vel_ + 'tor: %-8d'%tor_  + 'statusword: %s'%bin(statusword_) 
            self.dprint(test)
            # pos_ = self.network[test_id].sdo['position_actual_value'].raw
            # vel_ = self.network[test_id].sdo['velocity_actual_value'].raw
            # tor_ = self.network[test_id].sdo['torque_actual_value'].raw
            # statusword_ = self.network[test_id].sdo['statusword'].raw
            # test = 'read sdo... pos: %-11d'%pos_ + 'vel: %-8d'%vel_ + 'tor: %-8d'%tor_  + 'statusword: %s'%bin(statusword_) 
            # self.dprint(test)

            tt_ = time.time()

        ## -----------------------------------------------------------------
        ## Using a callback to asynchronously receive values
        ## Do not do any blocking operations here!
        def print_speed(message):
            str_ = '%s received' % message.name + ' '
            for var in message:
                str_ = str_ + ' %s = %d ' % (var.name, var.raw)
            self.dprint(str_)

        self.network[test_id].tpdo[1].add_callback(print_speed)
        self.network[test_id].tpdo[2].add_callback(print_speed)
        self.dprint("callback added")
        time.sleep(3)

        ## -----------------------------------------------------------------
        ## stop transmission of RPDO[1]
        self.network[test_id].rpdo[1].stop()
        self.network[test_id].rpdo[2].stop()
        self.network[test_id].sdo['modes_of_operation'].raw = -1

        test_ctrl = 0b111 ## Disable Operation command
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('DISABLE OPERATION COMMAND')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))

        test_ctrl = 0b110 ## Shutdown command
        self.network[test_id].sdo['controlword'].raw = test_ctrl
        time.sleep(1)
        self.dprint('========================')
        self.dprint('SHUTDOWN COMMAND')
        self.dprint("statusword: {0}".format(bin(self.network[test_id].sdo['statusword'].raw)))
        self.dprint("controlword: {0}".format(bin(self.network[test_id].sdo['controlword'].raw)))

        self.dprint('========================')


        ## -----------------------------------------------------------------
        ## disconnect after use
        self.network.nmt.state = 'PRE-OPERATIONAL'
        self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.1)
        self.print_status()

        self.network.disconnect()
        self.dprint('Successfully disconnected!')

    def dprint(self, str):
        print(str)
        self.db_.write(str + '\n')

    def print_status(self):
        self.dprint('master state: {0}'.format(self.network.nmt.state))
        for node_id in self.network:
            self.dprint('node {0} state: {1}'.format(node_id, self.network[node_id].nmt.state))

if __name__ == "__main__":
    TestElmo()
