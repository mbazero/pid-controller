'''
Store input and output configuraitons
'''
class IOConfig:
    def __init__(self, params):
        self.params = params

        self.adc_units = 'V'
        self.adc_range_units = [-5.0, 5.0]
        self.adc_range_norm = self.get_int_range(params.w_adc_data, 'signed')
        self.adc_decimals = 3
        self.adc_base_t = 5e-6; # Base ADC cycle time (s) with adc_os = 0
        self.adc_os_modes = [2**x for x in range(1, 7)]

        self.dac_units = 'V'
        self.dac_range_units = [0.0, 5.0]
        self.dac_range_norm = self.get_int_range(params.w_dac_data, 'unsigned')
        self.dac_decimals = 3

        self.freq_units = 'MHz'
        self.freq_range_units = [0.0, 1000.0]
        self.freq_range_norm = self.get_int_range(params.w_freq_data, 'unsigned')
        self.freq_decimals = 3

        self.phase_units = 'deg'
        self.phase_range_units = [0.0, 360.0]
        self.phase_range_norm = self.get_int_range(params.w_phase_data, 'unsigned')
        self.phase_decimals = 3

        self.amp_units = 'FTW'
        self.amp_range_norm = self.get_int_range(params.w_amp_data, 'unsigned')
        self.amp_range_units = self.amp_range_norm
        self.amp_decimals = 0

        # Map from breakout board analog ports to dac channels. Note that
        # this mapping uses a zero reference while the labels on the breakout
        # board use a one reference.
        self.aout_to_dac = [0, 2, 4, 6, 7, 5, 3, 1]

        # Inverse map from dac channels to breakout board analog ports
        self.dac_to_aout = [self.aout_to_dac.index(x) for x in range(len(self.aout_to_dac))]

    '''
    Function to map logical PID channels to physical output ports. The
    mapping is linear for DDS channels and linear for all DDS channels.
    '''
    def map_chan_to_port(self, chan):
        if 0 < chan < self.params.n_dac:
            return self.dac_to_aout[chan]
        else:
            return chan

    '''
    Inverse of the function above
    '''
    def map_port_to_chan(self, port):
        if 0 < port < self.params.n_dac:
            return self.aout_to_dac[port]
        else:
            return port

    '''
    Helper function to return the integer range for a bit precision. The
    function assumes two's complement for signed inputs.
    '''
    def get_int_range(self, precision, sign_type):
        if sign_type == 'signed':
            high = (2 ** (precision - 1)) - 1
            low = -high
        elif sign_type == 'unsigned':
            high = (2 ** precision) - 1
            low = 0

        return [low, high]
