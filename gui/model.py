'''
Store and manage PID lock array configuration parameters
'''
class Model:
    def __init__(self, params):
        self.n_in = params.n_pid_src
        self.n_out = params.n_pid_chan

        self.params = params

        self.init_model_params(params);
        self.init_param_map(params);
        self.init_io_params(params);
        self.init_data_logs(params);

    '''
    Initialize PID configuration parameters
    '''
    def init_model_params(self, params):
        self.adc_os = 1;

        self.chan_en = [0] * self.n_out
        self.chan_src_sel = [params.null_src] * self.n_out

        self.ovr_os = [0] * self.n_out

        self.pid_setpoint = [0] * self.n_out
        self.pid_p_coef = [0] * self.n_out
        self.pid_i_coef = [0] * self.n_out
        self.pid_d_coef = [0] * self.n_out
        self.pid_inv_error = [0] * self.n_out

        self.opt_min = [0] * self.n_out
        self.opt_max = [0] * self.n_out
        self.opt_init = [0] * self.n_out
        self.opt_mult = [0] * self.n_out
        self.opt_rs = [0] * self.n_out
        self.opt_add_chan = [0] * self.n_out

        self.pipe_chan = 0

    '''
    Map model parameters to HDL addresses. The mapping exactly mirrors the
    parameter mapping used in the HDL. The map indexes are identical and are
    parsed from the parameters.vh header file.
    '''
    def init_param_map(self, params):
        self.pmap = {
                # ADC controller
                params.adc_os_addr : self.adc_os,
                # Instruction dispatch
                params.chan_en_addr : self.chan_en,
                params.chan_src_sel_addr : self.chan_src_sel,
                # Oversample filter
                params.ovr_os_addr : self.ovr_os,
                # PID filter
                params.pid_setpoint_addr : self.pid_setpoint,
                params.pid_p_coef_addr : self.pid_p_coef,
                params.pid_i_coef_addr : self.pid_i_coef,
                params.pid_d_coef_addr : self.pid_d_coef,
                params.pid_inv_error_addr : self.pid_inv_error,
                # Output filter
                params.opt_min_addr : self.opt_min,
                params.opt_max_addr : self.opt_max,
                params.opt_init_addr : self.opt_init,
                params.opt_mult_addr : self.opt_mult,
                params.opt_rs_addr : self.opt_rs,
                params.opt_add_chan_addr : self.opt_add_chan,
                # Data logging
                params.pipe_chan_addr : self.pipe_chan,
                }

    '''
    Initialize input and output parameters
    '''
    def init_io_params(self, params):
        # Initialize input params
        self.adc_units = 'V'
        self.adc_range_units = [-5, 5]
        self.adc_range_norm = self.range_from_bitwidth(params.w_adc_data, 'signed')
        self.adc_cycle_t = 85; # TODO needs dynamic assignment

        # Initialize output params
        self.dac_units = 'V'
        self.dac_range_units = [0, 5]
        self.dac_range_norm = self.range_from_bitwidth(params.w_dac_data, 'unsigned')

        self.freq_units = 'MHz'
        self.freq_range_units = [0, 1000]
        self.freq_range_norm = self.range_from_bitwidth(params.w_freq_data, 'unsigned')

        self.phase_units = '?'
        self.phase_range_units = [0, 100]
        self.phase_range_norm = self.range_from_bitwidth(params.w_phase_data, 'unsigned')

        self.amp_units = '?'
        self.amp_range_units = [0, 100]
        self.amp_range_norm = self.range_from_bitwidth(params.w_amp_data, 'unsigned')

    '''
    Initialize data logging arrays
    '''
    def init_data_logs(self, params):
        pipe_delta_t = (self.adc_os ** self.adc_cycle_t) * (10 ** -6) # seconds

        self.data_log_single_x = [0] * self.n_out
        self.data_log_single_y = [0] * self.n_out
        self.data_log_block_x = [0] * self.n_out
        self.data_log_block_y = [0] * self.n_out

        for chan in range(self.n_out):
            self.data_log_single_x[chan] = []
            self.data_log_single_y[chan] = []
            self.data_log_block_x[chan] = [x * pipe_delta_t for x in range(params.pipe_depth)]
            self.data_log_block_y[chan] = [0 for x in range(params.pipe_depth)]

    '''
    Set parameter specified by address and channel The function has a twin
    write_data() in the Opal Kelly controller class which writes data into the
    parameter mapping on the FPGA. Boths functions have the same name and input
    parameters.
    '''
    def set_param(self, addr, chan, data):
        if isinstance(self.pmap[addr], list):
            self.pmap[addr][chan] = data
        else:
            self.pmap[addr] = data

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
    Return number of active channels
    '''
    def num_active_chans(self):
        return sum(self.chan_en)

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
    def input_ranges(self, chan):
        input_string = self.input_to_string(chan)

        if 'ADC' in input_string:
            range_units = self.adc_range_units
            range_norm = self.adc_range_norm

        return [range_units, range_norm]

    '''
    Return output range with units and normalized output range for
    specified channel
    '''
    def output_ranges(self, chan):
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
        [range_units, range_norm] = self.input_ranges(chan)
        return self.map_value(value, range_units, range_norm)

    '''
    Denormalize input value from unitless integer range to actual range
    with units
    '''
    def denormalize_input(self, chan, value):
        [range_units, range_norm] = self.input_ranges(chan)
        return self.map_value(value, range_norm, range_units)

    '''
    Normalize output value from actual output range with units to unitless
    integer range
    '''
    def normalize_output(self, chan, value):
        [range_units, range_norm] = self.output_ranges(chan)
        return self.map_value(value, range_units, range_norm)

    '''
    Denormalize output value from unitless integer range to actual range
    with units
    '''
    def denormalize_output(self, chan, value):
        [range_units, range_norm] = self.output_ranges(chan)
        return self.map_value(value, range_norm, range_units)

    '''
    Helper function to map value from one range to another
    '''
    def map_value(self, value, cur_range, norm_range):
        rel_value = value - cur_range[0]
        slope = float(norm_range[1] - norm_range[0]) / float(cur_range[1] - cur_range[0])
        return slope * rel_value + norm_range[0]

    '''
    Helper function to return the integer range for a bit precision. The
    function assumes two's complement for signed inputs.
    '''
    def range_from_bitwidth(self, precision, sign_type):
        if sign_type == 'signed':
            low = -(2 ** (precision - 1))
            high = (2 ** (precision - 1)) - 1
        elif sign_type == 'unsigned':
            low = 0
            high = (2 ** precision) - 1

        return [low, high]







