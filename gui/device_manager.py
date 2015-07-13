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

        self.clk_div = 3
        self.clk_ref = 48

        self.xem_lock = threading.Lock()

    def init_device(self):
        # Connect to opal kelly
        self.xem = ok.okCFrontPanel()
        if (self.xem.NoError != self.xem.OpenBySerial(self.serial)):
            print ("A device could not be opened.  Is one connected?")
            return(False)

        # Get device information
        self.devInfo = ok.okTDeviceInfo()
        if (self.xem.NoError != self.xem.GetDeviceInfo(self.devInfo)):
            print ("Unable to retrieve device information.")
            return(False)

        print("         Product: " + self.devInfo.productName)
        print("Firmware version: %d.%d" % (self.devInfo.deviceMajorVersion, self.devInfo.deviceMinorVersion))
        print("   Serial Number: %s" % self.devInfo.serialNumber)
        print("       Device ID: %s" % self.devInfo.deviceID)

        # Create PLL object and set reference clock
        self.pll = ok.PLL22393()
        self.pll.SetReference(self.clk_ref)

        # Configure clock 1
        self.pll.SetPLLParameters(0, self.clk1_freq * self.clk_div, self.clk_ref, True)
        self.pll.SetOutputSource(0, ok.PLL22393.ClkSrc_PLL0_0)
        self.pll.SetOutputDivider(0, self.clk_div)
        self.pll.SetOutputEnable(0, True)

        # Configure clock 2
        self.pll.SetPLLParameters(1, self.clk2_freq * self.clk_div, self.clk_ref, True)
        self.pll.SetOutputSource(1, ok.PLL22393.ClkSrc_PLL1_0)
        self.pll.SetOutputDivider(1, self.clk_div)
        self.pll.SetOutputEnable(1, True)

        # Download PLL configuration to FPGA
        if(self.xem.NoError != self.xem.SetEepromPLL22393Configuration(self.pll)):
            print("PLL configuration error.")
            return(False)

        # Load EEPROM PLL configuration
        self.xem.LoadDefaultPLLConfiguration()

        # Print new PLL configuration
        print("CLK1: " + format(self.pll.GetOutputFrequency(0), '.2f') + " MHz")
        print("CLK2: " + format(self.pll.GetOutputFrequency(1), '.2f') + " MHz")

        # Download PID controller configuration file
        if (self.xem.NoError != self.xem.ConfigureFPGA(self.bit_file)):
            print ("FPGA configuration failed.")
            return(False)

        # Check for frontpanel support in fpga configuration
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
    Update wire outs
    '''
    def update_wire_outs(self):
        self.xem.UpdateWireOuts();

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
