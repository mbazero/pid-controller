import struct
import binascii
import threading
import time
import cPickle
from PyQt4.Qt import *

'''
Handle state synchronization between model and view
'''
class Controller():
    def __init__(self, view, model, fpga, params, config):
        self.view = view
        self.model = model
        self.fpga = fpga
        self.params = params

        self.sample_period = 0.1
        self.block_transfer = False
        self.graph_freeze = [0] * params.n_pid_chan
        self.init_time = time.time()
        self.focused_chan = 0
        view.gp_view.chan_sel_arr[self.focused_chan].setChecked(True)

        self.trigger_adc_cstart()
        self.trigger_dac_rset()

        self.init_worker_thread()
        self.register_view_handlers()
        self.set_view_validators()
        if config:
            self.load_model(config)
        else:
            self.update_view()

    '''
    Register all input view handlers
    '''
    def register_view_handlers(self):

        # Global params view handlers
        gp_view = self.view.gp_view
        gp_view.adc_os.activated.connect(
                lambda: self.update_adc_os(gp_view.adc_os.currentIndex() + 1))
        gp_view.sample_period.editingFinished.connect(
                lambda: self.update_sampling_period(float(gp_view.sample_period.text())))
        gp_view.block_transfer.clicked.connect(
                lambda: self.update_block_transfer(gp_view.block_transfer.isChecked()))
        gp_view.save_config.clicked.connect(
                lambda: self.save_config())
        gp_view.load_model.clicked.connect(
                lambda: self.load_model())
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
        error_view.pid_setpoint.textEdited.connect(
                lambda: self.update_pid_setpoint(chan, float(error_view.pid_setpoint.text())))
        error_view.pid_inv_error.stateChanged.connect(
                lambda: self.update_pid_inv_error(chan, error_view.pid_inv_error.isChecked()))

        # PID view handlers
        pid_view.pid_p_coef.valueChanged.connect(
                lambda: self.update_pid_p_coef(chan, int(pid_view.pid_p_coef.text())))
        pid_view.pid_i_coef.valueChanged.connect(
                lambda: self.update_pid_i_coef(chan, int(pid_view.pid_i_coef.text())))
        pid_view.pid_d_coef.valueChanged.connect(
                lambda: self.update_pid_d_coef(chan, int(pid_view.pid_d_coef.text())))
        pid_view.pid_clear.clicked.connect(
                lambda: self.request_pid_clear(chan))

        # Processing view handlers
        proc_view.opt_mult.valueChanged.connect(
                lambda: self.update_opt_mult(chan, int(proc_view.opt_mult.text())))
        proc_view.opt_rs.valueChanged.connect(
                lambda: self.update_opt_rs(chan, int(proc_view.opt_rs.text())))
        proc_view.opt_add_chan.activated.connect(
                lambda: self.update_opt_add_chan(chan, proc_view.opt_add_chan.currentIndex() - 1))


        # Output view handlers
        output_view.opt_init.textEdited.connect(
                lambda: self.update_opt_init(chan, float(output_view.opt_init.text())))
        output_view.opt_max.textEdited.connect(
                lambda: self.update_opt_max(chan, float(output_view.opt_max.text())))
        output_view.opt_min.textEdited.connect(
                lambda: self.update_opt_min(chan, float(output_view.opt_min.text())))
        output_view.opt_inject.clicked.connect(
                lambda: self.request_opt_inject(chan))

    '''
    Set view validators
    '''
    def set_view_validators(self):
        for chan in range(self.params.n_pid_chan):
            chan_view = self.view.chan_views[chan]
            error_view = chan_view.error_view
            pid_view = chan_view.pid_view
            proc_view = chan_view.proc_view
            output_view = chan_view.output_view

            # Error view validators
            error_view.ovr_os.addItems(
                    [str(2**x) for x in range(2**self.params.w_pid_os)])
            inpt_validator = QDoubleValidator(
                    *(self.model.get_input_ranges(chan)[0] + [3]))
            error_view.pid_setpoint.setValidator(inpt_validator)

            # PID view handlers
            pid_view.pid_p_coef.setMaximum(2**self.params.w_pid_oprnds)
            pid_view.pid_i_coef.setMaximum(2**self.params.w_pid_oprnds)
            pid_view.pid_d_coef.setMaximum(2**self.params.w_pid_oprnds)

            # Processing view handlers
            proc_view.opt_mult.setMaximum(2**self.params.w_pid_oprnds)
            proc_view.opt_rs.setMaximum(2**self.params.w_pid_oprnds)

            # Output view handlers
            opt_validator = QDoubleValidator(
                    *(self.model.get_output_ranges(chan)[0] + [3]))
            print "output range: " + str(self.model.get_output_ranges(chan)[0])
            output_view.opt_init.setValidator(QDoubleValidator(0.0, 5.0, 2))
            output_view.opt_max.setValidator(opt_validator)
            output_view.opt_min.setValidator(opt_validator)


    '''
    Update view with model data
    '''
    def update_view(self, model=0):
        if not model:
            model = self.model

        # Sample period
        self.view.gp_view.sample_period.setText(str(self.sample_period))

        # ADC oversample mode
        adc_os = model.get_param(self.params.adc_os_addr, 0)
        self.view.gp_view.adc_os.setCurrentIndex(adc_os - 1)
        self.update_adc_os(adc_os)

        # Channel params
        for chan in range(self.params.n_pid_chan):
            self.update_chan_view(chan, model)

    '''
    Update channel view with model data
    '''
    def update_chan_view(self, chan, model):
        chan_view = self.view.chan_views[chan]
        mode_view = chan_view.mode_view
        error_view = chan_view.error_view
        pid_view = chan_view.pid_view
        proc_view = chan_view.proc_view
        output_view = chan_view.output_view
        params = self.params

        # Channel source select
        mode_view.chan_src_sel.clear()
        mode_view.chan_src_sel.addItems(['None'] + model.get_input_list())
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
        error_view.pid_setpoint.setText(str(pid_setpoint_denorm))
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
        proc_view.opt_add_chan.clear()
        proc_view.opt_add_chan.addItems(['None'] + self.model.get_chan_list())
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
        output_view.opt_init.setText(str(opt_init_denorm))
        self.update_opt_init(chan, opt_init_denorm)

        # Output max
        opt_max = model.get_param(params.opt_max_addr, chan)
        opt_max_denorm = model.denormalize_output(chan, opt_max)
        output_view.opt_max.setText(str(opt_max_denorm))
        self.update_opt_max(chan, opt_max_denorm)

        # Output min
        opt_min = model.get_param(params.opt_min_addr, chan)
        opt_min_denorm = model.denormalize_output(chan, opt_min)
        output_view.opt_max.setText(str(opt_min_denorm))
        self.update_opt_min(chan, opt_min_denorm)

    '''
    Save locking configuration to file
    '''
    def save_config(self):
        fname = self.view.get_save_file('Save Config')
        f = open(fname, 'w')
        cPickle.dump(self.model, f)

    '''
    Load configuration from path
    '''
    def load_model(self, fname=''):
        if not fname:
            fname = self.view.get_open_file('Load Config')
        f = open(fname, 'r')
        model = cPickle.load(f)
        self.update_view(model)

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
    def update_sampling_period(self, value):
        self.sample_period = value
        print "Sample period set to " + str(value)

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
            data = self.model.denormalize_input(chan, data*4)
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

            # unpack byte array as array of signed shorts
            fmt_str = '<' + str(self.params.pipe_depth) + 'h'
            data = struct.unpack(fmt_str, buf)

            data = [self.model.denormalize_input(self.focused_chan, word*4) for word in data]
            self.model.update_data_log_block(self.focused_chan, data)
        else:
            self.model.clear_data_log_block(self.focused_chan)

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
        if chan == self.focused_chan:
            self.view.gp_view.chan_sel_arr[self.focused_chan].setChecked(True)
        else:
            self.view.gp_view.chan_sel_arr[self.focused_chan].setChecked(False)
            self.focused_chan = chan
            self.view.chan_stack.setCurrentIndex(chan)
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
            time.sleep(self.controller.sample_period) # sleep for a period of time

    def shutDown(self):
        self.exiting = True
        self.wait()
