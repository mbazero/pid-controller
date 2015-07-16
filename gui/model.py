import numpy as np

'''
Store and manage PID lock array configuration parameters
'''
class Model:
    def __init__(self, io_config, params, pmap=0):
        self.n_in = params.n_pid_src
        self.n_out = params.n_pid_chan

        self.io_config = io_config
        self.params = params
        self.os_changed = 1
        self.slog_window = 1000
        self.slog_start_t = [0] * self.n_out

        self.lock_status_window = 10
        self.lock_status = [0] * self.n_out

        if pmap:
            self.pmap = pmap
        else:
            self.init_param_map();

        self.init_data_logs();

    '''
    Map model parameters to HDL addresses. The mapping exactly mirrors the
    parameter mapping used in the HDL. The map indexes are identical and are
    parsed from the parameters.vh header file.
    '''
    def init_param_map(self):
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
                params.opt_add_chan_addr : [params.opt_add_chan_init] * self.n_out,
                # GUI specific
                params.chan_name_addr : self.get_chan_list('logical', 'can'),
                params.qv_visible_addr : [0] * self.n_out,
                params.lock_threshold_addr : [0] * self.n_out
                }

    '''
    Initialize data logging arrays
    '''
    def init_data_logs(self):
        self.data_log_single_x = [[]] * self.n_out
        self.data_log_single_y = [[]] * self.n_out
        self.data_log_block_x = [[]] * self.n_out
        self.data_log_block_y = [[]] * self.n_out

    '''
    Set parameter specified by address and channel. The function has a twin
    function called write_data() in the Opal Kelly controller class which
    writes data into the parameter mapping on the FPGA. Boths functions have
    the same name and input parameters.
    '''
    def set_param(self, addr, chan, data):
        # Set flags if oversample mode is changed
        if addr == self.params.adc_os_addr:
            self.os_changed = 1
        elif addr == self.params.ovr_os_addr:
            self.os_changed = 1

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
    Update single word data log and lock status for specified channel. Takes
    as input a single data word and a timestamp.
    '''
    def update_data_log_single(self, chan, dword_x, dword_y):
        data_x = self.data_log_single_x[chan]
        data_y = self.data_log_single_y[chan]

        # Set start time if this is the first data entry
        if len(data_x) == 0:
            self.slog_start_t[chan] = dword_x

        # Update logs
        dword_x -= self.slog_start_t[chan]
        if len(data_x) < self.slog_window:
            data_x.append(dword_x)
            data_y.append(dword_y)
        else:
            data_x = data_x[1:] + [dword_x]
            data_y = data_y[1:] + [dword_y]

        # Update lock statistics
        if len(data_x) > self.lock_status_window:
            thresh = self.get_param(self.params.lock_threshold_addr, chan)
            setpoint = self.denormalize_input(chan, self.get_param(self.params.pid_setpoint_addr, chan))
            mean = np.mean(data_y[-self.lock_status_window:])
            self.lock_status[chan] = abs(mean - setpoint) < thresh
        else:
            self.lock_status[chan] = 0

    '''
    Update block data log for specified channel. Takes as input an array of
    data words. Time data is not required because the time interval between
    block data words is fixed and specified by the ADC update rate.
    '''
    def update_data_log_block(self, chan, data_y):
        self.data_log_block_y[chan] = data_y

        # Generate block time axis if it is empty or if an oversample
        # ratio has changed
        if self.os_changed or not self.data_log_block_x[chan]:
            adc_os = self.get_param(self.params.adc_os_addr, 0)
            ovr_os = self.get_param(self.params.ovr_os_addr, chan)
            adc_base_t = self.io_config.adc_base_t
            adc_cycle_t = 2**adc_os * adc_base_t
            pipe_delta_t = 2**ovr_os * adc_cycle_t
            self.data_log_block_x[chan] = \
                [x * pipe_delta_t for x in range(self.params.pipe_depth)]
            self.os_changed = 0

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
    Clear all data logs for specified channel
    '''
    def clear_data_logs(self, chan):
        self.clear_data_log_single(chan)
        self.clear_data_log_block(chan)

    '''
    Clear single word data log
    '''
    def clear_data_log_single(self, chan):
        self.data_log_single_x[chan] = []
        self.data_log_single_y[chan] = []

    '''
    Clear block data log
    '''
    def clear_data_log_block(self, chan):
        self.data_log_block_x[chan] = []
        self.data_log_block_y[chan] = []

    '''
    Returns channel lock stataus. '1' indicates the channel is locked
    '''
    def get_lock_status(self, chan):
        return self.lock_status[chan]

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
            return 'ADC ' + str(inpt + 1)

    '''
    Return string representation for output channel number
    '''
    def output_to_string(self, output):
        freq0 = self.params.freq0_addr
        phase0 = self.params.phase0_addr
        amp0 = self.params.amp0_addr

        if not self.is_valid_output(output):
            return 'Invalid output'
        elif output < self.params.freq0_addr:
            # DAC channels must be remapped because there is not a
            # linear mapping between the dac chip output channels
            # and the breakout board analog output ports.
            aout = self.io_config.dac_to_aout[output]
            return 'DAC ' + str(aout + 1)
        elif output < self.params.phase0_addr:
            return 'FREQ ' + str(output - freq0 + 1)
        elif output < self.params.amp0_addr:
            return 'PHASE ' + str(output - phase0 + 1)
        else:
            return 'AMP ' + str(output - amp0 + 1)

    '''
    Return string representation for channel number. The ntype flag
    specifies whether to return the canonical channel name or the
    user-set channel name. Channels are defined by their outputs,
    so the output string representation is returned if the canonical
    name is requested.
    '''
    def chan_to_string(self, chan, ntype):
        if ntype == 'usr':
            return self.get_param(self.params.chan_name_addr, chan)

        elif ntype == 'can':
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
    Return list of channel names. The order flag specifies if the list is
    returned in logical order, or output port order. These ordering are
    different only for DAC channels. The ntype flag specifies whether
    to return the canonical or user-set name.
    '''
    def get_chan_list(self, order, ntype):
        clist = []

        if order == 'port':
            # For port ordering, DAC channels must be added out of order
            for dac in self.io_config.aout_to_dac:
                if self.is_valid_input(dac):
                    clist.append(self.chan_to_string(dac, ntype))
        elif order == 'logical':
            for x in range(self.params.n_dac):
                clist.append(self.chan_to_string(x, ntype))

        # The rest of the channels can be added in order
        for chan in range(self.params.n_dac, self.n_out):
            clist.append(self.chan_to_string(chan, ntype))

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
        range_units = self.io_config.adc_range_units
        range_norm = self.io_config.adc_range_norm

        return [range_units, range_norm]

    '''
    Return input denormalization units
    '''
    def get_input_units(self, chan):
        return self.io_config.adc_units

    '''
    Return input decimal precision
    '''
    def get_input_decimals(self, chan):
        return self.io_config.adc_decimals

    '''
    Return output range with units and normalized output range for
    specified channel
    '''
    def get_output_ranges(self, chan):
        output_string = self.output_to_string(chan)
        io_config = self.io_config

        if 'DAC' in output_string:
            range_units = io_config.dac_range_units
            range_norm = io_config.dac_range_norm
        elif 'FREQ' in output_string:
            range_units = io_config.freq_range_units
            range_norm = io_config.freq_range_norm
        elif 'PHASE' in output_string:
            range_units = io_config.phase_range_units
            range_norm = io_config.phase_range_norm
        elif 'AMP' in output_string:
            range_units = io_config.amp_range_units
            range_norm = io_config.amp_range_norm

        return [range_units, range_norm]

    '''
    Return output denormalized units
    '''
    def get_output_units(self, chan):
        output_string = self.output_to_string(chan)
        io_config = self.io_config

        if 'DAC' in output_string:
            return io_config.dac_units
        elif 'FREQ' in output_string:
            return io_config.freq_units
        elif 'PHASE' in output_string:
            return io_config.phase_units
        elif 'AMP' in output_string:
            return io_config.amp_units
        else:
            return '?'

    '''
    Return output decimal precision
    '''
    def get_output_decimals(self, chan):
        output_string = self.output_to_string(chan)
        io_config = self.io_config

        if 'DAC' in output_string:
            return io_config.dac_decimals
        elif 'FREQ' in output_string:
            return io_config.freq_decimals
        elif 'PHASE' in output_string:
            return io_config.phase_decimals
        elif 'AMP' in output_string:
            return io_config.amp_decimals
        else:
            return 3

    '''
    Normalize input value from actual input range with units to unitless
    integer range
    '''
    def normalize_input(self, chan, value):
        inpt = self.get_param(self.params.chan_src_sel_addr, chan)
        [range_units, range_norm] = self.get_input_ranges(inpt)
        if range_units and range_norm:
            return int(self.map_value(value, range_units, range_norm))
        else:
            return value

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
