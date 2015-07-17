'''
Modify this file to change GUI configuration
'''

# default parameter config file
config_path = ''

# fpga config
pid_bit_file = './pid_controller.bit' # path to hdl bit file
serial = '' # opal kelly serial number; leave blank to connect to first device found
adc_clk_freq = 17 # adc serial frequency (max 17MHz)
sys_clk_freq = 50 # system clock frequency (max 50MHz)

# list of all hdl header files to parse
header_list = [ '../init.vh', '../ep_map.vh', '../parameters.vh']

