import threading
import random
import struct
import binascii

class PIDController:

    def __init__(self, params):
        # hdl params
        self.params = params

        # adc params
        self.adc_os = 1
        self.adc_update_rate = 200e3

        # channel params
        self.n_adc = params.n_adc
        self.n_dac = params.n_dac
        self.n_dds = params.n_dds
        self.n_out = n_dac + n_dds

        # initialize channels
        dac_chans = [PIDChannel(self, params, 'DAC', dac_count) for dac_count in range(n_dac)]
        freq_chans = [PIDChannel(self, params, 'FREQ', dds_count) for dds_count in range(n_dds)]
        phase_chans = [PIDChannel(self, params, 'PHASE', dds_count) for dds_count in range(n_dds)]
        amp_chans = [PIDChannel(self, params, 'AMP', dds_count) for dds_count in range(n_dds)]
        self.chans = dac_array + freq_array + phase_array + amp_array

        # focused channel
        self.focused_chan = self.dac_array[0]

        # active channels
        self.active_chans = []

        # activated event barrier
        # set when channel is activated
        # worker thread is awakened when set
        self.activated = threading.Event()
        self.activate_event.clear()

        # error data polling period in seconds
        self.polling_period = 0.1
        self.polling_period_lock = threading.Lock();

        # global block update setting
        # By default the GUI block updates the focused channel and
        # singly updates all other hidden channels.  If the global block
        # update option is toggled off, all channels are updated
        # singly.
        self.block_update = False

    def activate_channel(self, index):
        chan = self.chans[index]

        if chan.rtr_src_sel == self.params.NULL_CHAN:
            return False
        if chan.activated == True:
            return True

        chan.activated = True
        self.activate_chans.append(chan)
        self.activate_event.set()
        return True

    def deactivate_channel(self, index):
        chan = self.chans[index]

        if chan.activated == False:
            return True

        # set channel activation state
        chan.activated = False
        self.active_chans.remove(chan)
        if len(self.active_chans) == 0:
            self.activate_event.clear()

        return True

    def set_polling_period(self, value):
        self.polling_period = value

    def update_error_data(self):
        # update wire outs
        self.okc.UpdateWireOuts()

        # loop through active channels and update error data
        for chan in self.active_chans :
            chan.update_error_data()
            if chan.focused and self.block_update :
                chan.block_update_error_data()

    def set_focused_chan(self, index):
        self.focused_chan = self.chans[index]

    def src_valid(self, index):
        src = self.chans[index].rtr_src_sel

        return 0 <= src < self.n_adc


    def index_to_chan(self, index):
        amp_base = 
        if index >= self.n_dac + 2*n_dds:
            return self.
        if index >= self.n_dac :
            return self.dds_array[index - self.n_dac]
        else :
            return self.dac_array[index]

# PID lock
class PIDChannel:
    def __init__(self, pla, params, channel_type, channel_no):
        # pla
        self.pla = pla

        # endpoint map
        self.epm = pla.epm

        # opal kelly controller reference
        self.okc = okc

        # channel type and number
        self.channel_type = channel_type
        self.channel_no = channel_no

        # channel name
        self.cname = self.channel_type + ' Channel ' + str(self.channel_no)

        # activation status
        self.activated = False

        # tab focus status
        self.focused = True if self.channel_no == 0 else False

        # router params
        self.rtr_src_sel        = -1
        self.rtr_dest_sel       = channel_no

        # osf params
        self.osf_log_ovr = 0
        self.osf_cycle_delay = 0

        # pid params
        self.pid_setpoint = 0
        self.pid_p_coef = 1
        self.pid_i_coef = 0
        self.pid_d_coef = 0

        # opp params
        self.opp_lock_en = 0
        self.opp_init = 0
        self.opp_min = 0
        self.opp_max = 0
        self.opp_mult = 1
        self.opp_right_shift = 0

        # single word error data
        self.error_data = [0 for count in range(1024)]
        intra_block_period = (2**self.osf_log_ovr)*(1/pla.adc_update_rate)
        self.error_data_x = [float(count)*intra_block_period for count in range(1024)]
        # TODO make sure this computation is actually valid

        # block error data
        self.block_error_data = [0 for count in range(1024)]
        self.block_error_data_x = [count for count in range(1024)]
        # TODO compute x values

        # error data lock
        self.data_lock = threading.Lock()

        # initialize fp params
        self.init_params()

    def set_focus(self, state):
        self.focused = state

    def clear_error_data(self):
        self.error_data = [0 for i in self.error_data]
        self.block_error_data = [0 for i in self.block_error_data]

    #################### update error data #######################
    # get new error data from wire outs (continuous transfer)
    def update_error_data(self):
        if self.rtr_src_sel >= 0 :
            osf_data_owep = self.epm.osf_data0_owep + self.rtr_src_sel

            # get unsigned adc error data from wireout
            data_raw_us = self.okc.GetWireOutValue(osf_data_owep)

            # convert data to signed
            data_raw_s = self.uint16_to_int32(data_raw_us)

            # map raw adc data to voltage
            data_volts = self.map_val(data_raw_s, [-2**15, 2**15 - 1], [-5, 5])
            # data_volts = random.randrange(-1, 1)/1000
            self.error_data = self.error_data[1:len(self.error_data)] + [data_volts]

    def block_update_error_data(self):
        if self.rtr_src_sel >= 0 :
            block_size = 1024
            buf = bytearray(block_size*2)
            self.okc.xem.ReadFromPipeOut(self.epm.osf_block_data_pep, buf)

            # unpack byte array as array of signed shorts
            fmt_str = '<' + str(block_size) + 'h'
            data_raw = struct.unpack(fmt_str, buf)

            # map raw adc data to voltage
            # TODO: extract map ranges to stored values
            self.block_error_data = [self.map_val(d, [-2**15, 2**15-1], [-5, 5]) for d in data_raw] #DEBUG change this back to adc range
            # self.block_error_data = [self.map_val(d, [0, 2**16-1], [0, 5]) for d in data_raw]

    def uint16_to_int32(self, uint):
        if uint >= 2**15 :
            uint -= 2**16
        return uint


    #################### helper functions #######################
    def map_val(self, val, vrange, mrange):
        norm_val = val - vrange[0]
        range_ratio = (mrange[1]-mrange[0])/float(vrange[1]-vrange[0])
        return norm_val*range_ratio + mrange[0]
