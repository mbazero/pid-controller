import sys
import time
import threading
import sip
import OpalKellyController
import PIDLockArray
import pyqtgraph as pg
import numpy as np
import parser

# debug flag
debug = 1

# clock frequencies (MHz)
sys_clk_freq = 30
adc_clk_freq = 17

# parse hdl params
PARAMS_PATH = '../parameters.vh'
params = parser.Parse(PARAMS_PATH)

# parse endpoint map
EP_MAP_PATH = '../ep_map.vh'
epm = parser.Parse(EP_MAP_PATH)

# instantiate opal kelly controller
#okc = OpalKellyController.OpalKellyController(sys_clk_freq, adc_clk_freq, epm, debug)
#if (False == okc.init_device()):
    #exit

# instantiate PID lock array
#pla = PIDLockArray.PIDLockArray(epm, okc, pm.n_dac, pm.n_dds)

# instantiate GUI and run it
#gui = MainWindow(pla, pm.n_dac, pm.n_dds)
#gui.run()
