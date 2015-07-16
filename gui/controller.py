import struct
import binascii
import threading
import time
import cPickle
import model as md
from PyQt4.Qt import *
from functools import partial

'''
Handle state synchronization between model and view
'''
class Controller():
    def __init__(self, view, init_model, fpga, io_config, params, config_path=0):
        self.view = view
        self.model = init_model # model representing the initial FPGA state
        self.fpga = fpga
        self.params = params
        self.io_config = io_config

        self.sample_rate = 10 # Hz
        self.block_transfer = False
        self.graph_freeze = [0] * params.n_pid_chan
        self.focused_chan = 0
        view.gp_view.chan_sel_arr[self.focused_chan].setChecked(True)

        self.trigger_adc_cstart()
        self.trigger_dac_rset()

        self.init_worker_thread()
        self.register_view_handlers()
        self.set_view_ranges()

        if config_path:
            self.load_config(config_path)
        else:
            self.initialize()

    '''
    Register all input view handlers
    '''
    def register_view_handlers(self):

        # Outer tab handler
        opt_tabs = self.view.opt_tabs;
        get_tab_chan_no = lambda: opt_tabs.currentWidget().currentWidget().chan_no
        opt_tabs.currentChanged.connect(
                lambda: self.update_focused_chan(get_tab_chan_no()))

        # Inner tab handlers
        for x in range(opt_tabs.count()):
            opt_tabs.widget(x).currentChanged.connect(
                lambda: self.update_focused_chan(get_tab_chan_no()))

        # Global params view handlers
        gp_view = self.view.gp_view
        gp_view.adc_os.activated.connect(
                lambda: self.update_adc_os(gp_view.adc_os.currentIndex() + 1))
        gp_view.sample_rate.valueChanged.connect(
                lambda: self.update_sample_rate(gp_view.sample_rate.value()))
        gp_view.block_transfer.clicked.connect(
                lambda: self.update_block_transfer(gp_view.block_transfer.isChecked()))
        gp_view.save_config.clicked.connect(
                lambda: self.save_config())
        gp_view.load_config.clicked.connect(
                lambda: self.load_config())
        gp_view.sys_reset.clicked.connect(
                lambda: self.trigger_sys_reset())

        # Channel view handlers
        for chan in range(self.params.n_pid_chan):
            self.register_chan_view_handlers(chan)

    '''
    Register channel view handlers
    '''
    def register_chan_view_handlers(self, chan):
        gp_view = self.view.gp_view
        chan_view = self.view.chan_views[chan]
        mode_view = chan_view.mode_view
        error_view = chan_view.error_view
        pid_view = chan_view.pid_view
        proc_view = chan_view.proc_view
        output_view = chan_view.output_view

        # Tab handler
        gp_view.chan_sel_arr[chan].clicked.connect(
                lambda: self.update_focused_chan(chan))

        # Graph handlers
        chan_view.graph_freeze.clicked.connect(
                lambda: self.update_graph_freeze(chan, chan_view.graph_freeze.isChecked()))
        chan_view.graph_clear.clicked.connect(
                lambda: self.trigger_graph_clear(chan))

        # Mode view handlers
        mode_view.chan_src_sel.activated.connect(
                lambda: self.update_chan_src_sel(chan, mode_view.chan_src_sel.currentIndex() - 1))
        mode_view.chan_reset.clicked.connect(
                lambda: self.request_chan_reset(chan));
        mode_view.pid_lock_en.clicked.connect(
                lambda: self.update_pid_lock_en(chan, mode_view.pid_lock_en.isChecked()))

        # Error view handlers
        error_view.ovr_os.activated.connect(
                lambda: self.update_ovr_os(chan, error_view.ovr_os.currentIndex()))
        error_view.pid_setpoint.valueChanged.connect(
                lambda: self.update_pid_setpoint(chan, error_view.pid_setpoint.value()))
        error_view.pid_inv_error.stateChanged.connect(
                lambda: self.update_pid_inv_error(chan, error_view.pid_inv_error.isChecked()))

        # PID view handlers
        pid_view.pid_p_coef.valueChanged.connect(
                lambda: self.update_pid_p_coef(chan, pid_view.pid_p_coef.text()))
        pid_view.pid_i_coef.valueChanged.connect(
                lambda: self.update_pid_i_coef(chan, pid_view.pid_i_coef.text()))
        pid_view.pid_d_coef.valueChanged.connect(
                lambda: self.update_pid_d_coef(chan, pid_view.pid_d_coef.text()))
        pid_view.pid_clear.clicked.connect(
                lambda: self.request_pid_clear(chan))

        # Processing view handlers
        proc_view.opt_mult.valueChanged.connect(
                lambda: self.update_opt_mult(chan, proc_view.opt_mult.text()))
        proc_view.opt_rs.valueChanged.connect(
                lambda: self.update_opt_rs(chan, proc_view.opt_rs.text()))
        proc_view.opt_add_chan.activated.connect(
                lambda: self.update_opt_add_chan(chan, proc_view.opt_add_chan.currentIndex() - 1))


        # Output view handlers
        output_view.opt_init.valueChanged.connect(
                lambda: self.update_opt_init(chan, output_view.opt_init.value()))
        output_view.opt_max.valueChanged.connect(
                lambda: self.update_opt_max(chan, output_view.opt_max.text()))
        output_view.opt_min.valueChanged.connect(
                lambda: self.update_opt_min(chan, output_view.opt_min.text()))
        output_view.opt_inject.clicked.connect(
                lambda: self.request_opt_inject(chan))

    '''
    Set allowable ranges for view spin boxes
    '''
    def set_view_ranges(self):
        # Sampling rate
        self.view.gp_view.sample_rate.setValue(self.sample_rate)
        self.view.gp_view.sample_rate.setRange(1, 100)
        self.view.gp_view.sample_rate.setDecimals(1)

        # ADC oversample mode
        self.view.gp_view.adc_os.addItems(
                [str(mode) for mode in self.io_config.adc_os_modes]);

        for chan in range(self.params.n_pid_chan):
            chan_view = self.view.chan_views[chan]
            mode_view = chan_view.mode_view
            error_view = chan_view.error_view
            pid_view = chan_view.pid_view
            proc_view = chan_view.proc_view
            output_view = chan_view.output_view

            # Channel source select
            mode_view.chan_src_sel.clear()
            mode_view.chan_src_sel.addItems(['None'] + self.model.get_input_list())

            # Oversample mode
            error_view.ovr_os.clear()
            error_view.ovr_os.addItems(
                    [str(2**x) for x in range(2**self.params.w_pid_os)])

            # Setpoint
            error_view.pid_setpoint.setRange(*self.model.get_input_ranges(chan)[0])
            error_view.pid_setpoint.setDecimals(self.model.get_input_decimals(chan))

            # PID coefficients
            oprnd_high = 2**self.params.w_pid_oprnds - 1
            oprnd_range = [-oprnd_high, oprnd_high]
            pid_view.pid_p_coef.setRange(*oprnd_range)
            pid_view.pid_i_coef.setRange(*oprnd_range)
            pid_view.pid_d_coef.setRange(*oprnd_range)

            # Multiplier
            proc_view.opt_mult.setRange(*oprnd_range)

            # Right shift
            proc_view.opt_rs.setRange(*oprnd_range)

            # Add channel
            proc_view.opt_add_chan.clear()
            proc_view.opt_add_chan.addItems(['None'] + self.model.get_chan_list())

            # Output bounds
            opt_range = self.model.get_output_ranges(chan)[0]
            output_view.opt_init.setRange(*opt_range)
            output_view.opt_max.setRange(*opt_range)
            output_view.opt_min.setRange(*opt_range)


    '''
    Initialize view, model, and fpga. If a parameter map is supplied, all
    three components are initialized with the map values. If no parameter
    map is supplied, only the view is updated with the prexisting parameters
    in the active model.
    '''
    def initialize(self, pmap=0):
        if pmap:
            model = md.Model(self.io_config, self.params, pmap)
        if not model:
            model = self.model

        # Sample rate
        self.view.gp_view.sample_rate.setValue(self.sample_rate)

        # ADC oversample mode
        adc_os = model.get_param(self.params.adc_os_addr, 0)
        self.view.gp_view.adc_os.setCurrentIndex(adc_os - 1)
        self.update_adc_os(adc_os)

        # Channel params
        for chan in range(self.params.n_pid_chan):
            self.initialize_chan(chan, model)

    '''
    Update channel view with model data
    '''
    def initialize_chan(self, chan, model):
        chan_view = self.view.chan_views[chan]
        mode_view = chan_view.mode_view
        error_view = chan_view.error_view
        pid_view = chan_view.pid_view
        proc_view = chan_view.proc_view
        output_view = chan_view.output_view
        params = self.params

        # Channel source select
        chan_src_sel = model.get_param(params.chan_src_sel_addr, chan)
        mode_view.chan_src_sel.setCurrentIndex(chan_src_sel + 1)
        self.update_chan_src_sel(chan, chan_src_sel)

        # Lock enable
        pid_lock_en = model.get_param(params.pid_lock_en_addr, chan)
        mode_view.pid_lock_en.setChecked(pid_lock_en)
        self.update_pid_lock_en(chan, pid_lock_en)

        # Oversample mode
        ovr_os = model.get_param(params.ovr_os_addr, chan)
        error_view.ovr_os.setCurrentIndex(ovr_os)
        self.update_ovr_os(chan, ovr_os)

        # Setpoint
        pid_setpoint = model.get_param(params.pid_setpoint_addr, chan)
        pid_setpoint_denorm = model.denormalize_input(chan, pid_setpoint)
        error_view.pid_setpoint.setValue(pid_setpoint_denorm)
        self.update_pid_setpoint(chan, pid_setpoint_denorm)

        # Invert error
        pid_inv_error = model.get_param(params.pid_inv_error_addr, chan)
        error_view.pid_inv_error.setChecked(pid_inv_error)
        self.update_pid_inv_error(chan, pid_inv_error)

        # P coefficient
        pid_p_coef = model.get_param(params.pid_p_coef_addr, chan)
        pid_view.pid_p_coef.setValue(pid_p_coef)
        self.update_pid_p_coef(chan, pid_p_coef)

        # I coefficient
        pid_i_coef = model.get_param(params.pid_i_coef_addr, chan)
        pid_view.pid_i_coef.setValue(pid_i_coef)
        self.update_pid_i_coef(chan, pid_i_coef)

        # D coefficient
        pid_d_coef = model.get_param(params.pid_d_coef_addr, chan)
        pid_view.pid_d_coef.setValue(pid_d_coef)
        self.update_pid_d_coef(chan, pid_d_coef)

        # Add channel
        opt_add_chan = model.get_param(params.opt_add_chan_addr, chan)
        proc_view.opt_add_chan.setCurrentIndex(opt_add_chan + 1)
        self.update_opt_add_chan(chan, opt_add_chan)

        # Multiplier
        opt_mult = model.get_param(params.opt_mult_addr, chan)
        proc_view.opt_mult.setValue(opt_mult)
        self.update_opt_mult(chan, opt_mult)

        # Right shift
        opt_rs = model.get_param(params.opt_rs_addr, chan)
        proc_view.opt_rs.setValue(opt_rs)
        self.update_opt_rs(chan, opt_rs)

        # Output initial
        opt_init = model.get_param(params.opt_init_addr, chan)
        opt_init_denorm = model.denormalize_output(chan, opt_init)
        output_view.opt_init.setValue(opt_init_denorm)
        self.update_opt_init(chan, opt_init_denorm)

        # Output max
        opt_max = model.get_param(params.opt_max_addr, chan)
        opt_max_denorm = model.denormalize_output(chan, opt_max)
        output_view.opt_max.setValue(opt_max_denorm)
        self.update_opt_max(chan, opt_max_denorm)

        # Output min
        opt_min = model.get_param(params.opt_min_addr, chan)
        opt_min_denorm = model.denormalize_output(chan, opt_min)
        output_view.opt_min.setValue(opt_min_denorm)
        self.update_opt_min(chan, opt_min_denorm)

    '''
    Save locking configuration to file
    '''
    def save_config(self):
        fname = self.view.get_save_file('Save Config')
        f = open(fname, 'w')
        cPickle.dump(self.model.get_param_map(), f)

    '''
    Load configuration from path
    '''
    def load_config(self, fname=''):
        if not fname:
            fname = self.view.get_open_file('Load Config')
        f = open(fname, 'r')
        pmap = cPickle.load(f)
        self.initialize(pmap)

    '''
    Initialize worker thread for graph updating
    '''
    def init_worker_thread(self):
        self.worker = WorkerThread(self)
        self.view.connect(self.worker, SIGNAL("new_plot_data()"), self.update_graph)
        self.worker.start()

    '''
    Update view for specified channel with model data
    '''
    def update_channel_view(self, chan):
        chan_view = self.view.chans[chan]
        mode_view = chan_view.mode_view
        pid_view = chan_view.pid_view
        output_view = chan_view.output_view

    '''
    Update view graph
    '''
    def update_graph(self):
        if self.block_transfer:
            [data_x, data_y] = self.model.get_data_log_block(self.focused_chan)
        else:
            [data_x, data_y] = self.model.get_data_log_single(self.focused_chan)

        if not self.graph_freeze[self.focused_chan]:
            self.view.update_graph(self.focused_chan, data_x, data_y)

    def update_graph_freeze(self, chan, freeze):
        self.graph_freeze[chan] = freeze

    def trigger_graph_clear(self, chan):
        if self.block_transfer:
            self.model.clear_data_log_block(chan)
        else:
            self.model.clear_data_log_single(chan)

    '''
    Update fpga data sampling period
    '''
    def update_sample_rate(self, value):
        self.sample_rate = value
        print "Sample rate set to " + str(value)

    '''
    Update block transfer state
    '''
    def update_block_transfer(self, enabled):
        if enabled == True:
            self.block_transfer = True
            print 'Block transfer enabled'
        else:
            self.block_transfer = False
            print 'Block transfer disabled'

    '''
    Update model and fpga data according to the channel and address
    specified
    '''
    def update_model_and_fpga(self, addr, chan, value):
        self.model.set_param(addr, chan, value)
        self.fpga.write_data(addr, chan, value)

    def send_fpga_request(self, rqst, chan):
        self.fpga.send_request(rqst, chan)

    '''
    Read single word data logs for all active channels and block data log for
    tab focused channel
    '''
    def read_fpga_data(self):
        self.fpga.update_wire_outs()

        # Read log wire-outs for each channel
        for chan in range(self.params.n_pid_chan):
            self.read_data_log_single(chan)

        # Read log pipe-out for focused channel if it is ready
        if self.block_transfer and \
                self.fpga.get_wire_out_value(self.params.data_log_status_owep):
            self.read_data_log_block()

    '''
    Read single word logged data for specified channel
    '''
    def read_data_log_single(self, chan):
        if self.model.has_valid_input(chan):
            data_log_owep = self.params.data_log0_owep + chan
            data = self.fpga.get_wire_out_value(data_log_owep)
            data = self.uint16_to_int32(data)
            data = self.adjust_logged_data(data, 'ADC')
            data = self.model.denormalize_input(chan, data)
            self.model.update_data_log_single(chan, time.time(), data)
        else:
            self.model.clear_data_log_single(chan)

    '''
    Read block logged data for tab focused channel and store in model
    '''
    def read_data_log_block(self):
        if self.model.has_valid_input(self.focused_chan):
            buf = bytearray(self.params.pipe_depth * 2)
            self.fpga.read_from_pipe_out(self.params.data_log_opep, buf)

            # Unpack byte array as array of signed shorts
            fmt_str = '<' + str(self.params.pipe_depth) + 'h'
            data = struct.unpack(fmt_str, buf)

            # Adjust and denormalize data
            dout = []
            for dword in data:
                adj_dword = self.adjust_logged_data(dword, 'ADC')
                dout.append(self.model.denormalize_input(self.focused_chan, adj_dword))
            self.model.update_data_log_block(self.focused_chan, dout)

        else:
            self.model.clear_data_log_block(self.focused_chan)

    '''
    Adjust logged data to scale of internal signal
    '''
    def adjust_logged_data(self, data, source):
        if source == 'ADC':
            w_delta = self.params.w_adc_data - self.params.w_ep
            if w_delta > 0:
                return data << w_delta
            else:
                return data

    '''
    ADC param update handling
    '''
    def update_adc_os(self, value):
        self.update_model_and_fpga(self.params.adc_os_addr, 0, value)
        print "ADC oversample mode set to " + str(value)

    '''
    Channel param update handling
    '''
    def update_pid_lock_en(self, chan, enable, reset=True):
        self.update_model_and_fpga(self.params.pid_lock_en_addr, chan, enable)

        # Reset channel on disable
        if reset and not enable:
            self.request_chan_reset(chan)

        print self.model.chan_to_string(chan) + (" activated" if enable else " deactivated")

    def update_focused_chan(self, chan):
        print "NEW FOCUS = " + str(chan)
        if chan == self.focused_chan:
            self.view.gp_view.chan_sel_arr[self.focused_chan].setChecked(True)
        else:
            self.view.gp_view.chan_sel_arr[self.focused_chan].setChecked(False)
            self.focused_chan = chan
            self.send_fpga_request(self.params.pipe_cset_rqst, chan);

    def update_chan_src_sel(self, chan, src_sel):
        old_src_sel = self.model.get_param(self.params.chan_src_sel_addr, chan)

        if src_sel != old_src_sel:
            self.model.clear_data_logs(chan)
            if self.model.is_valid_input(src_sel):
                self.update_model_and_fpga(self.params.chan_src_sel_addr, chan, src_sel)
                print self.model.chan_to_string(chan) + " input set to " + self.model.input_to_string(src_sel)

            else:
                self.update_pid_lock_en(chan, False) # disable pid lock for invalid route
                self.update_model_and_fpga(self.params.pid_lock_en_addr, chan, 0)
                self.update_model_and_fpga(self.params.chan_src_sel_addr, chan, self.params.null_src)
                print self.model.chan_to_string(chan) + " input deactivated"

    '''
    Oversample filter param update handling
    '''
    def update_ovr_os(self, chan, value):
        self.request_ovr_clear(chan)
        self.update_model_and_fpga(self.params.ovr_os_addr, chan, value)
        print self.model.chan_to_string(chan) + " oversample ratio set to " + str(2**value)

    '''
    PID filter param update handling
    '''
    def update_pid_setpoint(self, chan, value):
        norm_value = self.model.normalize_input(chan, value);
        self.update_model_and_fpga(self.params.pid_setpoint_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " setpoint set to " + str(value)

    def update_pid_p_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_p_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " P coefficient set to " + str(value)

    def update_pid_i_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_i_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " I coefficient set to " + str(value)

    def update_pid_d_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_d_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " D coefficient set to " + str(value)

    def update_pid_inv_error(self, chan, invert):
        self.update_model_and_fpga(self.params.pid_inv_error_addr, chan, invert)
        print self.model.chan_to_string(chan) + " error inversion " + ("enabled" if invert else "disabled")

    '''
    Output filter param update handling
    '''
    def update_opt_init(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_init_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " init set to " + str(value)

    def update_opt_max(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_max_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " max set to " + str(value)

    def update_opt_min(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_min_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " min set to " + str(value)

    def update_opt_mult(self, chan, value):
        self.update_model_and_fpga(self.params.opt_mult_addr, chan, value)
        self.update_scale_factor(chan)
        print self.model.chan_to_string(chan) + " multiplier set to " + str(value)

    def update_opt_rs(self, chan, value):
        self.update_model_and_fpga(self.params.opt_rs_addr, chan, value)
        self.update_scale_factor(chan)
        print self.model.chan_to_string(chan) + " right shift set to " + str(value)

    def update_scale_factor(self, chan):
        proc_view = self.view.chan_views[chan].proc_view
        try:
            rs = float(proc_view.opt_rs.text())
            mult = float(proc_view.opt_mult.text())
            scale_factor = mult / 2**rs
            proc_view.scale_factor.setText(str(scale_factor))
        except:
            proc_view.scale_factor.setText("NaN")

    def update_opt_add_chan(self, chan, value):
        self.update_model_and_fpga(self.params.opt_add_chan_addr, chan, value)
        print self.model.chan_to_string(chan) + " add channel set to " + self.model.chan_to_string(value)

    '''
    Request handling
    '''
    def request_chan_reset(self, chan):
        toggle_reset = False
        if self.model.is_lock_enabled(chan):
            toggle_reset = True
            self.update_pid_lock_en(chan, False, False)

        self.request_ovr_clear(chan)
        self.request_pid_clear(chan)
        self.request_opt_clear(chan)

        if toggle_reset:
            self.update_pid_lock_en(chan, True, False);

    def request_ovr_clear(self, chan):
        self.send_fpga_request(self.params.ovr_clr_rqst, chan);
        print self.model.chan_to_string(chan) + " oversample memory cleared"

    def request_pid_clear(self, chan):
        self.send_fpga_request(self.params.pid_clr_rqst, chan);
        print self.model.chan_to_string(chan) + " PID memory cleared"

    def request_opt_clear(self, chan):
        self.send_fpga_request(self.params.opt_clr_rqst, chan);
        print self.model.chan_to_string(chan) + " output memory cleared"

    def request_opt_inject(self, chan):
        self.send_fpga_request(self.params.opt_inj_rqst, chan);
        print self.model.chan_to_string(chan) + " write injection sent"

    '''
    Trigger handling
    '''
    def trigger_adc_cstart(self):
        self.fpga.activate_sys_trigger(self.params.adc_cstart_offset)
        print "ADC started"

    def trigger_dac_rset(self):
        self.fpga.activate_sys_trigger(self.params.dac_rset_offset)

    def trigger_sys_reset(self):
        self.fpga.activate_sys_trigger(self.params.sys_rst_offset)
        self.trigger_adc_cstart()
        self.trigger_dac_rset()

    '''
    Helper function to convert from unsigned 16-bit number to signed
    32-bit number
    '''
    def uint16_to_int32(self, uint):
        if uint >= 2**15 :
            uint -= 2**16
        return uint

'''
Worker thread to pull data from fpga and update plots without stalling GUI
'''
class WorkerThread(QThread):
    def __init__(self, controller):
        QThread.__init__(self)
        self.controller = controller
        self.exiting = False

    def run(self):
        while not self.exiting:
            self.controller.read_fpga_data()
            self.emit(SIGNAL('new_plot_data()')) # signal gui that new data is available to be plotted
            time.sleep(1/self.controller.sample_rate) # sleep for a period of time

    def shutDown(self):
        self.exiting = True
        self.wait()
