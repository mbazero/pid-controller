import ok
import threading

'''
Manages Opal Kelly fpga communication
'''
class DeviceManager:
    def __init__(self, bit_file, serial, clk1_freq, clk2_freq, params):
        self.bit_file = bit_file
        self.serial = serial
        self.clk1_freq = clk1_freq
        self.clk2_freq = clk2_freq
        self.params = params

        self.pll_freq = clk1_freq * clk2_freq # MHz

        self.xem_lock = threading.Lock()

    def init_device(self):
        # connect to opal kelly
        self.xem = ok.okCFrontPanel()
        if (self.xem.NoError != self.xem.OpenBySerial(self.serial)):
            print ("A device could not be opened.  Is one connected?")
            return(False)

        # get device information
        self.devInfo = ok.okTDeviceInfo()
        if (self.xem.NoError != self.xem.GetDeviceInfo(self.devInfo)):
            print ("Unable to retrieve device information.")
            return(False)

        print("         Product: " + self.devInfo.productName)
        print("Firmware version: %d.%d" % (self.devInfo.deviceMajorVersion, self.devInfo.deviceMinorVersion))
        print("   Serial Number: %s" % self.devInfo.serialNumber)
        print("       Device ID: %s" % self.devInfo.deviceID)

        # configure system clocks
        self.pll = ok.PLL22393() # create PLL object
        self.pll.SetReference(48.0) # set reference clock to 48MHz
        self.pll.SetPLLParameters(0, self.pll_freq, 48, True) # set PLL[0] frequency
        self.pll.SetOutputSource(0, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK1 to PLL[0]
        self.pll.SetOutputSource(1, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK2 to PLL[0]
        self.pll.SetOutputDivider(0, self.pll_freq / self.clk1_freq) # set SYSCLK1 frequency
        self.pll.SetOutputDivider(1, self.pll_freq / self.clk2_freq) # set SYSCLK2 frequency
        self.pll.SetOutputEnable(0, True) # enable SYSCLK1
        self.pll.SetOutputEnable(1, True) # enable SYSCLK2

        # save new PLL configuration to EEPROM
        if(self.xem.NoError != self.xem.SetEepromPLL22393Configuration(self.pll)):
            print("PLL configuration error.")
            return(False)

        # load EEPROM PLL configuration
        self.xem.LoadDefaultPLLConfiguration()

        # print new PLL configuration
        print("SYSCLK1: " + format(self.pll.GetOutputFrequency(0), '.2f') + " MHz")
        print("SYSCLK2: " + format(self.pll.GetOutputFrequency(1), '.2f') + " MHz")

        # download PID controller configuration file
        if (self.xem.NoError != self.xem.ConfigureFPGA(self.bit_file)):
            print ("FPGA configuration failed.")
            return(False)

        # check for frontpanel support in fpga configuration
        if (False == self.xem.IsFrontPanelEnabled()):
            print ("FrontPanel support is not available.")
            return(False)

        print ("FrontPanel support is available.")
        return(True)

    '''
    Write data to fpga at address and channel specified
    '''
    def write_data(self, addr, chan, data):
        # split data
        data3 = (int(data) >> 48) & 0xffff
        data2 = (int(data) >> 32) & 0xffff
        data1 = (int(data) >> 16) & 0xffff
        data0 = int(data) & 0xffff

        # send data to opal kelly
        self.set_wire_in(self.params.data3_iwep, data3)
        self.set_wire_in(self.params.data2_iwep, data2)
        self.set_wire_in(self.params.data1_iwep, data1)
        self.set_wire_in(self.params.data0_iwep, data0)
        self.set_wire_in(self.params.addr_iwep, addr)
        self.set_wire_in(self.params.chan_iwep, chan)
        self.xem.UpdateWireIns()
        self.activate_sys_trigger(self.params.wr_en_offset)

    '''
    Send request to fpga
    '''
    def send_request(self, rqst, chan):
        self.write_data(rqst, chan, 0);

    '''
    Activate system trigger at specified offset
    '''
    def activate_sys_trigger(self, offset):
        self.activate_trigger_in(self.params.sys_gp_itep, offset)

    '''
    Activate OPP injection trigger for specified channel
    Injection triggers should only be used when a channel is deactivated
    to cause OPP to write its initial value.
    '''
    def activate_opp_injection(self, chan):
        if chan < self.params.w_ep:
            self.activate_trigger_in(self.params.opp_inject0_itep, chan)
        elif chan < 2 * self.params.w_ep:
            self.activate_trigger_in(self.params.opp_inject1_itep, chan)

    '''
    Return wire out value at specified endpoint
    '''
    def get_wire_out_value(self, ep):
        return self.xem.GetWireOutValue(ep)

    '''
    Read pipe out data to buffer from specified endpoint
    '''
    def read_from_pipe_out(self, ep, buf):
        error = self.xem.ReadFromPipeOut(ep, buf)
        if (error < 0):
            print 'Pipe Read Error: ' + str(error)

    def set_wire_in(self, ep, val):
        error = self.xem.SetWireInValue(ep, val, 0xffffffff)
        if(error != 0):
            print 'Set Wire In Error: ' + str(error)

    def activate_trigger_in(self, ep, offset):
        error = self.xem.ActivateTriggerIn(ep, offset)
        if (error != 0):
            print 'Activate Trigger Error: ' + str(error)

    def lock_xem(self):
        xem_lock.acquire()

    def unlock_xem(self):
        xem_lock.release()
