import ok
import threading

# TODO
# - remove mod update function and implement in PLA

# controls low-level communication with opal kelly board
class OpalKellyController:
    def __init__(self, epm, clk1_freq, clk2_freq, debug=0):
        # endpoint map
        self.epm = epm

        # clock frequencies (MHz)
        self.clk1_freq = clk1_freq
        self.clk2_freq = clk2_freq

        # set debug state
        self.debug = debug

        # pll frequency (MHz)
        self.pll_freq = 400

        # configuration file name
        self.bit_file = "pid_controller.bit"

        # initialize device access lock
        self.xem_lock = threading.Lock()

    def init_device(self):
        # open first opal kelly device found
        self.xem = ok.okCFrontPanel()
        if (self.xem.NoError != self.xem.OpenBySerial("")):
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

        # create system clocks
        self.pll = ok.PLL22393() # create PLL object
        self.pll.SetReference(48.0) # set reference clock to 48MHz
        self.pll.SetPLLParameters(0, self.pll_freq, 48, True) # set PLL[0] frequency
        self.pll.SetOutputSource(0, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK1 to PLL[0]
        self.pll.SetOutputSource(1, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK2 to PLL[0]
        self.pll.SetOutputDivider(0, 400/self.clk1_freq) # set SYSCLK1 frequency
        self.pll.SetOutputDivider(1, 400/self.clk2_freq) # set SYSCLK2 frequency
        self.pll.SetOutputEnable(0, True) # enable SYSCLK1
        self.pll.SetOutputEnable(1, True) # enable SYSCLK2

        # save new PLL configuration to EEPROM
        if(self.xem.NoError != self.xem.SetEepromPLL22393Configuration(self.pll)):
            print("PLL configuration error.")
            return(False)

        # load EEPROM PLL configuration
        self.xem.LoadDefaultPLLConfiguration()

        # print new PLL configuration
        print("SYSCLK1: " + str(self.pll.GetOutputFrequency(0)) + "MHz")
        print("SYSCLK2: " + str(self.pll.GetOutputFrequency(1)) + "MHz")

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

    # write data to FPGA in thread safe manner
    def write_data(self, addr, chan, data):
        # split data
        data2 = (data >> 32) & 0xffff
        data1 = (data >> 16) & 0xffff
        data0 = data & 0xffff

        # send data
        self.set_wire_in(epm.data2_iwep, data2)
        self.set_wire_in(epm.data1_iwep, data1)
        self.set_wire_in(epm.data0_iwep, data0)
        self.set_wire_in(epm.addr_iwep, addr)
        self.set_wire_in(epm.chan_iwep, chan)
        self.xem.UpdateWireIns()
        self.activate_sys_trigger(epm.reg_update_offset)

    # activate system trigger
    def activate_sys_trigger(self, offset):
        self.xem.ActivateTriggerIn(epm.sys_gp_itep, offset)

    # trigger dac write
    def inject_dac_dv(self, chan):
        self.xem.ActivateTriggerIn(epm.opp_dac_inj_itep, 1 << chan)

    # trigger dds frequency write
    def inject_freq_dv(self, chan):
        self.xem.ActivateTriggerIn(epm.opp_freq_inj_itep, 1 << chan)

    # trigger dds phase write
    def inject_phase_dv(self, chan):
        self.xem.ActivateTriggerIn(epm.opp_phase_inj_itep, 1 << chan)

    # trigger dds amplitude write
    def inject_amp_dv(self, chan):
        self.xem.ActivateTriggerIn(epm.opp_amp_inj_itep, 1 << chan)

    def GetWireOutValue(self, ep):
        self.xemLock.acquire()
        value = self.xem.GetWireOutValue(ep)
        self.xemLock.release()
        return value

    def set_wire_in(self, ep, val):
        set_error = self.xem.SetWireInValue(ep, val, 0xffffffff)
        if(set_error != 0):
            print 'Set Wire In Error: ' + str(set_error)
