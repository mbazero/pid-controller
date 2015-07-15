'''
Store input and output configuraitons
'''
class IOConfig:
    def __init__(self, params):
        self.adc_units = 'V'
        self.adc_range_units = [-5.0, 5.0]
        self.adc_range_norm = self.range_from_bitwidth(params.w_adc_data, 'signed')
        self.adc_base_t = 5e-6; # Base ADC cycle time (s) with adc_os = 0

        self.dac_units = 'V'
        self.dac_range_units = [0.0, 5.0]
        self.dac_range_norm = self.range_from_bitwidth(params.w_dac_data, 'unsigned')

        self.freq_units = 'MHz'
        self.freq_range_units = [0.0, 1000.0]
        self.freq_range_norm = self.range_from_bitwidth(params.w_freq_data, 'unsigned')
        print self.freq_range_norm

        self.phase_units = 'rad'
        self.phase_range_norm = self.range_from_bitwidth(params.w_phase_data, 'unsigned')
        self.phase_range_units = self.phase_range_norm
        print self.phase_range_norm

        self.amp_units = 'FTW'
        self.amp_range_norm = self.range_from_bitwidth(params.w_amp_data, 'unsigned')
        self.amp_range_units = self.amp_range_norm
        print self.amp_range_norm

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







