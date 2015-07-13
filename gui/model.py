'''
Store and manage PID lock array configuration parameters
'''
class Model:
    def __init__(self, params):
        self.n_in = params.n_pid_src
        self.n_out = params.n_pid_chan

        self.params = params

        self.init_param_map();
        self.init_io_params();
        self.init_data_logs();

    '''
    Map model parameters to HDL addresses. The mapping exactly mirrors the
    parameter mapping used in the HDL. The map indexes are identical and are
    parsed from the parameters.vh header file.
    '''
    def init_param_map(self):
        zeros = [0] * self.n_out;
        params = self.params
        self.pmap = {
                # ADC controller
                params.adc_os_addr : params.adc_os_init,
                # Instruction dispatch
                params.pid_lock_en_addr : [params.pid_lock_en_init] * self.n_out,
                params.chan_src_sel_addr : [params.chan_src_sel_init] * self.n_out,
                # Oversample filter
                params.ovr_os_addr : [params.ovr_os_init] * self.n_out,
                # PID filter
                params.pid_setpoint_addr : [params.pid_setpoint_init] * self.n_out,
                params.pid_p_coef_addr : [params.pid_p_coef_init] * self.n_out,
                params.pid_i_coef_addr : [params.pid_i_coef_init] * self.n_out,
                params.pid_d_coef_addr : [params.pid_d_coef_init] * self.n_out,
                params.pid_inv_error_addr : [params.pid_inv_error_init] * self.n_out,
                # Output filter
                params.opt_min_addr : [params.opt_min_init] * self.n_out,
                params.opt_max_addr : [params.opt_max_init] * self.n_out,
                params.opt_init_addr : [params.opt_init_init] * self.n_out,
                params.opt_mult_addr : [params.opt_mult_init] * self.n_out,
                params.opt_rs_addr : [params.opt_rs_init] * self.n_out,
                params.opt_add_chan_addr : [params.opt_add_chan_init] * self.n_out
                }

    '''
    Initialize input and output parameters
    '''
    def init_io_params(self):
        self.adc_units = 'V'
        self.adc_range_units = [-5, 5]
        self.adc_range_norm = self.range_from_bitwidth(self.params.w_ep, 'signed')
        self.adc_cycle_t = 85; # TODO needs dynamic assignment

        self.dac_units = 'V'
        self.dac_range_units = [0, 5]
        self.dac_range_norm = self.range_from_bitwidth(self.params.w_ep, 'unsigned')

        self.freq_units = 'MHz'
        self.freq_range_units = [0, 1000]
        self.freq_range_norm = self.range_from_bitwidth(self.params.w_ep, 'unsigned')

        self.phase_units = '?'
        self.phase_range_units = [0, 100]
        self.phase_range_norm = self.range_from_bitwidth(self.params.w_ep, 'unsigned')

        self.amp_units = '?'
        self.amp_range_units = [0, 100]
        self.amp_range_norm = self.range_from_bitwidth(self.params.w_ep, 'unsigned')

    '''
    Initialize data logging arrays
    '''
    def init_data_logs(self):
        adc_os = self.get_param(self.params.adc_os_addr, 0)
        pipe_delta_t = (adc_os ** self.adc_cycle_t) * (10 ** -6) # seconds

        self.data_log_single_x = [0] * self.n_out
        self.data_log_single_y = [0] * self.n_out
        self.data_log_block_x = [0] * self.n_out
        self.data_log_block_y = [0] * self.n_out

        for chan in range(self.n_out):
            self.data_log_single_x[chan] = []
            self.data_log_single_y[chan] = []
            self.data_log_block_x[chan] = [x * pipe_delta_t for x in range(self.params.pipe_depth)]
            self.data_log_block_y[chan] = [0 for x in range(self.params.pipe_depth)]

    '''
    Clear data logs for specified channel
    '''
    def clear_data_logs(self, chan):
        self.data_log_single_x[chan] = []
        self.data_log_single_y[chan] = []
        self.data_log_block_y[chan] = [0 for x in range(self.params.pipe_depth)]

    '''
    Set parameter specified by address and channel. The function has a twin
    function called write_data() in the Opal Kelly controller class which
    writes data into the parameter mapping on the FPGA. Boths functions have
    the same name and input parameters.
    '''
    def set_param(self, addr, chan, data):
        if isinstance(self.pmap[addr], list):
            self.pmap[addr][chan] = data
        else:
            self.pmap[addr] = data

    '''
    Set parameter map i.e. set all parameters at once
    '''
    def set_param_map(self, pmap):
        self.pmap = pmap

    '''
    Return value of parameter specified by address and channel
    '''
    def get_param(self, addr, chan):
        if isinstance(self.pmap[addr], list):
            return self.pmap[addr][chan]
        else:
            return self.pmap[addr]

    '''
    Return parameter map
    '''
    def get_param_map(self):
        return self.pmap

    '''
    Update single word data log for specified channel. Takes as input a single
    data word and a timestamp
    '''
    def update_data_log_single(self, chan, data_x, data_y):
        self.data_log_single_x[chan].append(data_x)
        self.data_log_single_y[chan].append(data_y)

    '''
    Update block data log for specified channel. Takes as input an array of
    data words. Time data is not required because the time interval between
    block data words is fixed and specified by the ADC update rate.
    '''
    def update_data_log_block(self, chan, data_y):
        self.data_log_block_y[chan] = data_y

    '''
    Return single word data log for specified channel
    '''
    def get_data_log_single(self, chan):
        return [self.data_log_single_x[chan], self.data_log_single_y[chan]]
    '''
    Return block data log for specified channel
    '''
    def get_data_log_block(self, chan):
        return [self.data_log_block_x[chan], self.data_log_block_y[chan]]

    '''
    Return true if PID lock is enabled for the specified channel
    '''
    def is_lock_enabled(self, chan):
        return self.get_param(self.params.pid_lock_en_addr, chan)

    '''
    Return true if channel has a valid input mapping
    '''
    def has_valid_input(self, chan):
        inpt = self.get_param(self.params.chan_src_sel_addr, chan)
        return self.is_valid_input(inpt)

    '''
    Return list of channels with valid input mappings
    '''
    def get_routed_chans(self):
        clist = []
        for chan in range(self.n_out):
            if self.has_valid_input(chan):
                clist.append(chan)

        return clist

    '''
    Return string representation for input channel number
    '''
    def input_to_string(self, inpt):
        if not self.is_valid_input(inpt):
            return 'Invalid input'
        else:
            return 'ADC ' + str(inpt)

    '''
    Return string representation for output channel number
    '''
    def output_to_string(self, output):
        n_dac = self.params.n_dac
        n_dds = self.params.n_dds

        if not self.is_valid_output(output):
            return 'Invalid output'
        elif output < n_dac:
            return 'DAC ' + str(output)
        elif output < n_dac + n_dds:
            return 'DDS FREQ ' + str(output - n_dac)
        elif output < n_dac + 2 * n_dds:
            return 'DDS PHASE ' + str(output - n_dac - n_dds)
        else:
            return 'DDS AMP ' + str(output - n_dac - 2*n_dds)

    '''
    Return string representation for channel number. Channels are defined by
    their outputs, so the output string representation is returned.
    '''
    def chan_to_string(self, chan):
        return self.output_to_string(chan)

    '''
    Return list of all input names
    '''
    def get_input_list(self):
        ilist = []
        for inpt in range(self.n_in):
            ilist.append(self.input_to_string(inpt))

        return ilist

    '''
    Return a list of all channel names
    '''
    def get_chan_list(self):
        clist = []
        for chan in range(self.n_out):
            clist.append(self.chan_to_string(chan))

        return clist

    '''
    Return true if the index specifies a valid input
    '''
    def is_valid_input(self, inpt):
        return 0 <= inpt < self.n_in

    '''
    Return true if the index specifies a valid output
    '''
    def is_valid_output(self, output):
        return 0 <= output < self.n_out

    '''
    Return input range with units and normalized input range for
    specified channel
    '''
    def get_input_ranges(self, chan):
        input_string = self.input_to_string(chan)

        if 'ADC' in input_string:
            range_units = self.adc_range_units
            range_norm = self.adc_range_norm

        return [range_units, range_norm]

    '''
    Return output range with units and normalized output range for
    specified channel
    '''
    def get_output_ranges(self, chan):
        output_string = self.output_to_string(chan)

        if 'DAC' in output_string:
            range_units = self.dac_range_units
            range_norm = self.dac_range_norm
        elif 'FREQ' in output_string:
            range_units = self.freq_range_units
            range_norm = self.freq_range_norm
        elif 'PHASE' in output_string:
            range_units = self.phase_range_units
            range_norm = self.phase_range_norm
        elif 'AMP' in output_string:
            range_units = self.amp_range_units
            range_norm = self.amp_range_norm

        return [range_units, range_norm]

    '''
    Normalize input value from actual input range with units to unitless
    integer range
    '''
    def normalize_input(self, chan, value):
        [range_units, range_norm] = self.get_input_ranges(chan)
        return int(self.map_value(value, range_units, range_norm))

    '''
    Denormalize input value from unitless integer range to actual range
    with units
    '''
    def denormalize_input(self, chan, value):
        [range_units, range_norm] = self.get_input_ranges(chan)
        return self.map_value(value, range_norm, range_units)

    '''
    Normalize output value from actual output range with units to unitless
    integer range
    '''
    def normalize_output(self, chan, value):
        [range_units, range_norm] = self.get_output_ranges(chan)
        return self.map_value(value, range_units, range_norm)

    '''
    Denormalize output value from unitless integer range to actual range
    with units
    '''
    def denormalize_output(self, chan, value):
        [range_units, range_norm] = self.get_output_ranges(chan)
        return self.map_value(value, range_norm, range_units)

    '''
    Helper function to map value from one range to another
    '''
    def map_value(self, value, cur_range, new_range):
        rel_value = value - cur_range[0]
        slope = float(new_range[1] - new_range[0]) / float(cur_range[1] - cur_range[0])
        return slope * rel_value + new_range[0]

    '''
    Helper function to return the integer range for a bit precision. The
    function assumes two's complement for signed inputs.
    '''
    def range_from_bitwidth(self, precision, sign_type):
        if sign_type == 'signed':
            high = (2 ** (precision - 1)) - 1
            low = -high
        elif sign_type == 'unsigned':
            high = (2 ** precision) - 1
            low = 0

        return [low, high]







