import os.path
import time
import canopen


class TestElmo():
    def __init__(self):
        ## -----------------------------------------------------------------
        ## node id for testing
        test_id = 4

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
            print("Found node %d!" % node_id)
            # local_node_ = canopen.LocalNode(node_id=node_id, object_dictionary=EDS_PATH)
            # self.network.add_node(local_node_)
            self.network.add_node(node=node_id, object_dictionary=EDS_PATH)
        for node_id in self.network:
            print(self.network[node_id])
        ## -----------------------------------------------------------------
        ## check bus state
        print('state:',self.network.bus.state)
        print('bus channel_info:',self.network.bus.channel_info)

        ## -----------------------------------------------------------------
        ## send out time message
        self.network.time.transmit()

        ## -----------------------------------------------------------------
        ## receive message
        msg = self.network.bus.recv(timeout=1.0)
        print('msg:',msg)

        ## -----------------------------------------------------------------
        ## change network nmt state
        self.network.nmt.state = 'PRE-OPERATIONAL'
        # self.network.nmt.state = 'OPERATIONAL'

        ## -----------------------------------------------------------------
        ## senf nmt start to all nodes
        self.network.send_message(0x0, [0x1, 0])

        ## -----------------------------------------------------------------
        ## send nmt start to all nodes
        if (test_id not in self.network):
            print("testing node id is not in network. EXITING...")
            self.network.disconnect()
            return
        print("node state:",self.network[test_id].nmt.state)
        self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        # self.network[test_id].nmt.state = 'OPERATIONAL'
        print("node state:",self.network[test_id].nmt.state)

        ## -----------------------------------------------------------------
        ## download SDO data
        # Open the Store EDS variable as a file like object
        infile = self.network[test_id].sdo[0x1021].open('r', encoding='ascii')
        # Open a file for writing to
        outfile = open('out.eds', 'w', encoding='ascii')
        # Iteratively read lines from node and write to file
        outfile.writelines(infile)
        # Clean-up
        infile.close()
        outfile.close()

        ## -----------------------------------------------------------------
        ## get SDO from node
        test_sdo = self.network[test_id].sdo['Producer Heartbeat Time'].raw
        print('test sdo:',test_sdo)

        ## -----------------------------------------------------------------
        ## set SDO value
        self.network[test_id].sdo['Producer Heartbeat Time'].raw = 1000
        self.network[test_id].sdo[1800][0].raw = 1000 # TPDO parameter number of entries

        ## -----------------------------------------------------------------
        ## transmit sync every 100 ms
        self.network.sync.start(0.1)

        # ## -----------------------------------------------------------------
        # ## configure state machine by searching for TPDO that has statusword
        # self.network[test_id].setup_402_state_machine()

        ## -----------------------------------------------------------------
        ## read current PDO configuration
        self.network[test_id].tpdo.read()
        self.network[test_id].rpdo.read()

        # ## -----------------------------------------------------------------
        # ## change TPDO configuration
        # self.network[test_id].tpdo[1].clear()
        # # self.network[test_id].tpdo[2].clear()
        # # self.network[test_id].tpdo[3].clear()
        # # self.network[test_id].tpdo[4].clear()
        # self.network[test_id].tpdo[1].add_variable('velocity_sensor_actual_value')
        # self.network[test_id].tpdo[1].enabled = True
        # self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        # self.network[test_id].tpdo.save()

        ## -----------------------------------------------------------------
        ## change RPDO configuration
        self.network[test_id].rpdo[1].clear()
        # self.network[test_id].rpdo[2].clear()
        # self.network[test_id].rpdo[3].clear()
        # self.network[test_id].rpdo[4].clear()
        # stri = 'target_position'
        # stri = 'target_velocity'
        stri = 'target_torque'
        self.network[test_id].rpdo[1].add_variable(stri)
        self.network[test_id].rpdo[1].enabled = True
        self.network[test_id].nmt.state = 'PRE-OPERATIONAL'
        self.network[test_id].rpdo.save()

        ## -----------------------------------------------------------------
        ## change target velocity
        self.network[test_id].rpdo[1][stri].raw = 0
        self.network[test_id].rpdo[1].start(0.1)
        self.network[test_id].nmt.state = 'OPERATIONAL'
        print('changing velocity')
        time.sleep(10)
        self.network[test_id].rpdo[1].stop()
        print("done!")

        ## -----------------------------------------------------------------
        ## disconnect after use
        self.network.disconnect()

if __name__ == "__main__":
    TestElmo()
