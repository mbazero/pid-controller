import ok
import threading

# TODO
# - remove mod update function and implement in PLA

# controls low-level communication with opal kelly board
class OpalKellyController:
    def __init__(self, epm, debug=0):
        # endpoint map
        self.epm = epm

        # set debug state
        self.debug = debug

        # configuration file name
        self.bitFile = "pid_controller.bit"

        # initialize device access lock
        self.xemLock = threading.Lock()
        return

    def InitializeDevice(self):
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

        # create new PLL with 50MHz and 17MHz clocks
        self.pll = ok.PLL22393() # create PLL object
        self.pll.SetReference(48.0) # set reference clock to 48MHz
        self.pll.SetPLLParameters(0, 400, 48, True) # set PLL[0] to 400MHz
        self.pll.SetOutputSource(0, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK1 to PLL[0]
        self.pll.SetOutputSource(1, ok.PLL22393.ClkSrc_PLL0_0) # map SYSCLK2 to PLL[0]
        self.pll.SetOutputDivider(0, 14) # set SYSCLK1 divider to 14
        self.pll.SetOutputDivider(1, 24) # set SYSCLK2 divider to 24
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
        if (self.xem.NoError != self.xem.ConfigureFPGA(self.bitFile)):
            print ("FPGA configuration failed.")
            return(False)

        # check for frontpanel support in fpga configuration
        if (False == self.xem.IsFrontPanelEnabled()):
            print ("FrontPanel support is not available.")
            return(False)

        print ("FrontPanel support is available.")
        return(True)

    # thread safe wrappers for opal kelly functions
    def ActivateTriggerIn(self, ep, bit):
        self.xemLock.acquire()
        self.xem.ActivateTriggerIn(ep, bit)
        self.xemLock.release()

    def ModUpdate(self) :
        self.ActivateTriggerIn(self.epm.module_update_tep , 0)

    def GetWireOutValue(self, ep):
        self.xemLock.acquire()
        value = self.xem.GetWireOutValue(ep)
        self.xemLock.release()
        return value

    def SetWireInValue(self, ep, val):
        self.xemLock.acquire()
        set_error = self.xem.SetWireInValue(ep, val, 0xffffffff)
        if(set_error != 0): print 'Set Wire In Error: ' + str(set_error)
        self.xemLock.release()

    def SetAndUpdateWireIn(self, ep, val):
        self.xemLock.acquire()
        set_error = self.xem.SetWireInValue(ep, val, 0xffffffff)
        if(set_error != 0): print 'Set Wire In Error: ' + str(set_error)
        self.xem.UpdateWireIns()
        self.xemLock.release()

    def UpdateWireIns(self):
        self.xemLock.acquire()
        self.xem.UpdateWireIns()
        self.xemLock.release()

    def UpdateWireOuts(self):
        self.xemLock.acquire()
        self.xem.UpdateWireOuts()
        self.xemLock.release()
