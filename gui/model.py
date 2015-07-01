class PIDModel:

    def __init__(self, params):
        n_in = params.n_adc
        n_out = params.n_dds + params.n_dac

        '''
        Initialize parameter mapping
        This mapping exactly mirrors the parameter
        mapping used in the HDL. The map indexes
        are identical and are parsed from the
        parameters.vh header file.
        '''
        self.pmap = {
                # adc mappings
                params.adc_os_addr : 1,
                # osf mappings
                params.osf_activate_addr : [0] * n_in,
                params.osf_cycle_delay_addr : [0] * n_in,
                params.osf_osm_addr : [0] * n_in,
                # pid mappings
                params.pid_lock_en_addr : [0] * n_in,
                params.pid_setpoint_addr : [0] * n_in,
                params.pid_p_coef_addr : [0] * n_in,
                params.pid_i_coef_addr : [0] * n_in,
                params.pid_d_coef_addr : [0] * n_in,
                # router mappings
                params.ochan_src_sel_addr : [0] * n_out,
                # opp mappings
                params.opp_min_addr : [0] * n_out,
                params.opp_max_addr : [0] * n_out,
                params.opp_init_addr : [0] * n_out,
                params.opp_mult_addr : [0] * n_out,
                params.opp_rs_addr : [0] * n_out,
                # general mappings
                params.focused_chan_addr : 0
                }

    def write_data(self, addr, chan, data):

        if isinstance(tmap[addr], list):
            tmap[addr][chan] = data
        else:
            tmap[addr] = data

    def read_data(self, addr, chan):

        if isinstance(tmap[addr], list):
            return tmap[addr][chan]
        else:
            return tmap[addr]





