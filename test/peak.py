import os.path
import time
import canopen


class TestElmo():
    def __init__(self):
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

        ## -----------------------------------------------------------------
        ## clean and reconnect
        self.network.bus.flush_tx_buffer()
        self.network.bus.stop_all_periodic_tasks()
        # self.network.bus.shutdown()
        self.network.clear()
        self.network.update()
        self.network.disconnect()
        self.network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=125000)


        ## -----------------------------------------------------------------
        ## change network nmt state
        self.network.nmt.state = 'PRE-OPERATIONAL'

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
        ## disconnect after use
        self.network.disconnect()

if __name__ == "__main__":
    TestElmo()
