'''
Store and manage PID lock array configuration parameters
'''
class Model:
    def __init__(self, params):
        self.n_in = params.n_in
        self.n_out = params.n_out

        self.init_model_params(params);
        self.init_param_map(params);
        self.init_io_params(params);
        self.init_log_arrays(params);

    '''
    Initialize PID configuration parameters
    '''
    def init_model_params(self, params):
        self.adc_os = 1;

        self.chan_activate = [0] * self.n_out
        self.chan_focus = 0
        self.chan_input_sel = [params.null_input] * self.n_out

        self.osf_cycle_delay = [0] * self.n_out
        self.osf_os = [0] * self.n_out

        self.pid_setpoint = [0] * self.n_out
        self.pid_p_coef = [0] * self.n_out
        self.pid_i_coef = [0] * self.n_out
        self.pid_d_coef = [0] * self.n_out
        self.pid_lock_en = [0] * self.n_out

        self.opp_min = [0] * self.n_out
        self.opp_max = [0] * self.n_out
        self.opp_init = [0] * self.n_out
        self.opp_mult = [0] * self.n_out
        self.opp_rs = [0] * self.n_out

    '''
    Map model parameters to HDL addresses. The mapping exactly mirrors the
    parameter mapping used in the HDL. The map indexes are identical and are
    parsed from the parameters.vh header file.
    '''
    def init_param_map(self, params):
        self.pmap = {
                # adc mappings
                params.adc_os_addr : self.adc_os,
                # channel mappings
                params.chan_activate_addr : self.chan_activate,
                params.chan_focus_addr : self.chan_focus,
                params.chan_input_sel_addr : self.chan_input_sel,
                # osf mappings
                params.osf_cycle_delay_addr : self.osf_cycle_delay,
                params.osf_os_addr : self.osf_os,
                # pid mappings
                params.pid_setpoint_addr : self.pid_setpoint,
                params.pid_p_coef_addr : self.pid_p_coef,
                params.pid_i_coef_addr : self.pid_i_coef,
                params.pid_d_coef_addr : self.pid_d_coef,
                params.pid_lock_en_addr : self.pid_lock_en,
                # opp mappings
                params.opp_min_addr : self.opp_min,
                params.opp_max_addr : self.opp_max,
                params.opp_init_addr : self.opp_init,
                params.opp_mult_addr : self.opp_mult,
                params.opp_rs_addr : self.opp_rs,
                }

    '''
    Initialize input and output parameters
    '''
    def init_io_params(self, params):
        # Initialize input params
        self.adc_units = 'V'
        self.adc_range_units = [-5, 5]
        self.adc_range_norm = self.range_from_precision(params.w_adc_data, 'signed')

        # Initialize output params
        self.dac_units = 'V'
        self.dac_range_units = [0, 5]
        self.dac_range_norm = self.range_from_precision(params.w_dac_data, 'unsigned')

        self.freq_units = 'MHz'
        self.freq_range_units = [0, 1000]
        self.freq_range_norm = self.range_from_precision(params.w_freq_data, 'unsigned')

        self.phase_units = '?'
        self.phase_range_units = [0, 100]
        self.phase_range_norm = self.range_from_precision(params.w_phase_data, 'unsigned')

        self.amp_units = '?'
        self.amp_range_units = [0, 100]
        self.amp_range_norm = self.range_from_precision(params.w_amp_data, 'unsigned')

    '''
    Initialize data logging arrays
    '''
    def init_log_arrays(self, params):
        pipe_delta_t = (self.adc_os ** params.adc_cycle_t) * (10 ** -6) # seconds

        self.wire_out_data_x = [0] * params.n_out
        self.wire_out_data_y = [0] * params.n_out
        self.pipe_out_data_x = [0] * params.n_out
        self.pipe_out_data_y = [0] * params.n_out

        for chan in range(params.n_out):
            self.wire_out_data_x[chan] = []
            self.wire_out_data_y[chan] = []
            self.pipe_out_data_x[chan] = [x * pipe_delta_t for x in range(params.pipe_depth)]
            self.pipe_out_data_y[chan] = [0 for x in range(params.pipe_depth)]

    '''
    Write data into parameter mapping
    The function has a twin in the Opal Kelly controller class which
    write data into the parameter mapping on the FPGA. Boths functions
    have the same name and input parameters.
    '''
    def write_data(self, addr, chan, data):
        if isinstance(self.pmap[addr], list):
            self.pmap[addr][chan] = data
        else:
            self.pmap[addr] = data

    '''
    Update wire out data for specified channel
    '''
    def update_wire_out_data(self, chan, data_x, data_y):
        self.wire_out_data_x[chan].append(data_x)
        self.wire_out_data_y[chan].append(data_y)

    '''
    Update pipe out data for specified channel. Time data is not
    required because the time interval between block data words is fixed
    and specified by the ADC update rate.
    '''
    def update_pipe_out_data(self, chan, data_y):
        self.pipe_out_data_y[chan] = data_y

    '''
    Return wire out x and y data for specified channel
    '''
    def get_wire_out_data(self, chan):
        return [wire_out_data_x[chan], wire_out_data_y[chan]]
    '''
    Return pipe-out x and y data for specified channel
    '''
    def get_pipe_out_data(self, chan):
        return [pipe_out_data_x[chan], pipe_out_data_y[chan]]

    '''
    Return the number of active channels
    '''
    def num_active_chans(self):
        return sum(chan_activate)

    '''
    Return string representation input
    '''
    def input_to_string(self, inpt):
        if not is_valid_intput(inpt):
            return 'Invalid input'
        else:
            return 'ADC ' + str(inpt)

    '''
    Return string representation of channel output
    '''
    def output_to_string(self, output):
        n_dac = self.params.n_dac
        n_dds = self.params.n_dds

        if not is_valid_output(output):
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
    Return string representation of channel. Channels are defined by
    their outputs, so the output string is returned.
    '''
    def chan_to_string(self, chan):
        return output_to_string(chan_input_sel(chan))

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
        return map_value(value, range_units, range_norm)

    '''
    Denormalize input value from unitless integer range to actual range
    with units
    '''
    def denormalize_input(self, chan, value):
        [range_units, range_norm] = self.input_ranges(chan)
        return map_value(value, range_norm, range_units)

    '''
    Normalize output value from actual output range with units to unitless
    integer range
    '''
    def normalize_output(self, chan, value):
        [range_units, range_norm] = self.output_ranges(chan)
        return map_value(value, range_units, range_norm)

    '''
    Denormalize output value from unitless integer range to actual range
    with units
    '''
    def denormalize_output(self, chan, value):
        [range_units, range_norm] = self.output_ranges(chan)
        return map_value(value, range_norm, range_units)

    '''
    Helper function to map value from one range to another
    '''
    def map_value(self, value, cur_range, norm_range):
        rel_value = value - cur_range[0]
        slope = float(norm_range[1] - norm_range[0]) / float(cur_range[1] - old_range[0])
        return slope * rel_value + norm_range[0]

    '''
    Helper function to return the integer range for a bit precision. The
    function assumes two's complement for signed inputs.
    '''
    def range_from_precision(self, precision, sign_type):
        if sign_type == 'signed':
            low = -(2 ** (precision - 1))
            high = (2 ** (precision - 1)) - 1
        elif sign_type == 'unsigned':
            low = 0
            high = (2 ** precision) - 1

        return [low, high]







