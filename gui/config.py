'''
Modify this file to change GUI configuration
'''

# list of all hdl header files to parse
header_list = [ '../init.vh', '../ep_map.vh', '../parameters.vh']

# default parameter config file
pconfig = 'default_config.pid'

# fpga config
pid_bit_file = './pid_controller.bit' # path to hdl bit file
serial = '' # opal kelly serial number; leave blank to connect to first device found
sys_clk_freq = 39 # system clock frequency in MHz
adc_clk_freq = 17 # adc serial frequency in MHz
