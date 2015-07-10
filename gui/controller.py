import struct
import binascii
import threading
import time
from PyQt4.Qt import *

'''
Handles state synchronization between model and view
'''
class Controller():
    def __init__(self, view, model, fpga, params):
        self.view = view
        self.model = model
        self.fpga = fpga
        self.params = params

        self.sample_period = 0.1
        self.adc_started = False
        self.dac_ref_set = False
        self.block_transfer = False
        self.init_time = time.time()

        self.register_view_handlers()

        self.init_view()

        self.init_worker_thread()

    '''
    Register all input view handlers
    '''
    def register_view_handlers(self):
        # Tab handlers
        self.view.tab_widget.currentChanged.connect(
                lambda: self.update_pipe_chan(self.view.tab_widget.currentIndex()))

        # Global params view handlers
        gp_view = self.view.gp_view
        gp_view.adc_os.currentIndexChanged.connect(
                lambda: self.update_adc_os(gp_view.adc_os.currentIndex()))
        gp_view.sample_period.editingFinished.connect(
                lambda: self.update_sampling_period(float(gp_view.sample_period.text())))
        gp_view.block_transfer.clicked.connect(
                lambda: self.update_block_transfer(gp_view.block_transfer.isChecked()))
        gp_view.adc_cstart.clicked.connect(
                lambda: self.trigger_adc_cstart())
        gp_view.dac_ref_set.clicked.connect(
                lambda: self.trigger_dac_ref_set())
        gp_view.sys_reset.clicked.connect(
                lambda: self.trigger_sys_reset())

        # Channel view handlers
        for chan in range(self.params.n_pid_chan):
            self.register_chan_view_handlers(chan)

    '''
    Register view handlers for the specified channel
    '''
    def register_chan_view_handlers(self, chan):
        chan_view = self.view.chan_views[chan]
        input_view = chan_view.input_view
        pid_view = chan_view.pid_view
        proc_view = chan_view.proc_view
        output_view = chan_view.output_view

        # Input view handlers
        input_view.chan_src_sel.currentIndexChanged.connect(
                lambda: self.update_chan_src_sel(chan, input_view.chan_src_sel.currentIndex() - 1))
        input_view.ovr_os.currentIndexChanged.connect(
                lambda: self.update_ovr_os(chan, input_view.ovr_os.currentIndex() + 1))
        input_view.chan_en.toggled.connect(
                lambda: self.update_chan_enable(chan, input_view.chan_en.isChecked()))
        input_view.chan_reset.toggled.connect(
                lambda: self.request_chan_reset(chan));

        # PID view handlers
        pid_view.pid_setpoint.textChanged.connect(
                lambda: self.update_pid_setpoint(chan, float(pid_view.pid_setpoint.text())))
        pid_view.pid_p_coef.textChanged.connect(
                lambda: self.update_pid_p_coef(chan, int(pid_view.pid_p_coef.text())))
        pid_view.pid_i_coef.textChanged.connect(
                lambda: self.update_pid_i_coef(chan, int(pid_view.pid_i_coef.text())))
        pid_view.pid_d_coef.textChanged.connect(
                lambda: self.update_pid_d_coef(chan, int(pid_view.pid_d_coef.text())))
        pid_view.pid_clear.clicked.connect(
                lambda: self.request_pid_clear(chan))

        # Processing view handlers
        proc_view.opt_mult.textChanged.connect(
                lambda: self.update_opt_mult(chan, int(proc_view.opt_mult.text())))
        proc_view.opt_rs_wgt.textChanged.connect(
                lambda: self.update_opt_rs(chan, int(proc_view.opt_rs_wgt.text())))
        proc_view.opt_add_chan.currentIndexChanged.connect(
                lambda: self.update_opt_add_chan(chan, proc_view.opt_add_chan.currentIndex() - 1))


        # Output view handlers
        output_view.opt_init.textChanged.connect(
                lambda: self.update_opt_init(chan, float(output_view.opt_init.text())))
        output_view.opt_max.textChanged.connect(
                lambda: self.update_opt_max(chan, float(output_view.opt_max.text())))
        output_view.opt_min.textChanged.connect(
                lambda: self.update_opt_min(chan, float(output_view.opt_min.text())))
        output_view.opt_clear.clicked.connect(
                lambda: self.request_opt_clear(chan))
        output_view.opt_inject.clicked.connect(
                lambda: self.request_opt_inject(chan))


    '''
    Initialize view
    '''
    def init_view(self):
        for chan in range(self.params.n_pid_chan):
            chan_view = self.view.chan_views[chan]
            input_view = chan_view.input_view
            pid_view = chan_view.pid_view
            proc_view = chan_view.proc_view
            output_view = chan_view.output_view

            # populate input select combo box
            input_view.chan_src_sel.addItems(['None'] + self.model.get_input_list())

            # populate add channel options
            proc_view.opt_add_chan.addItems(['None'] + self.model.get_chan_list())

    '''
    Update view with model data
    '''
    def update_view(self):
        print "do something"

    '''
    Initialize worker thread for graph updating
    '''
    def init_worker_thread(self):
        self.worker = WorkerThread(self)
        self.view.connect(self.worker, SIGNAL("new_plot_data()"), self.update_view_graph)
        self.worker_wake = threading.Event()
        self.worker_wake.clear()
        self.worker.start()

    '''
    Update view for specified channel with model data
    '''
    def update_channel_view(self, chan):
        chan_view = self.view.chans[chan]
        input_view = chan_view.input_view
        pid_view = chan_view.pid_view
        output_view = chan_view.output_view

    '''
    Update view graph
    '''
    def update_view_graph(self):
        pipe_chan = self.model.pipe_chan
        if self.block_transfer:
            [data_x, data_y] = self.model.get_data_log_block(pipe_chan)
        else:
            [data_x, data_y] = self.model.get_data_log_single(pipe_chan)

        self.view.update_graph(pipe_chan, data_x, data_y)

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
        for chan, chan_en in enumerate(self.model.chan_en):
            if chan_en:
                print "Reading from active chan: " + str(chan)
                self.read_data_log_single(chan)
        self.read_data_log_block()

    '''
    Read single word logged data for specified channel
    '''
    def read_data_log_single(self, chan):
        data_log_owep = self.params.data_log0_owep + chan
        print "Wire-out ep: " + str(data_log_owep)
        data = self.fpga.get_wire_out_value(data_log_owep)
        print "Wire-out data: " + str(data)
        data = self.uint16_to_int32(data)
        data = self.model.denormalize_input(chan, data)
        self.model.update_data_log_single(chan, time.time(), data)

    '''
    Read block logged data for tab focused channel and store in model
    '''
    def read_data_log_block(self):
        buf = bytearray(self.params.pipe_depth * 2)
        self.fpga.read_from_pipe_out(self.params.data_log_opep, buf)

        # unpack byte array as array of signed shorts
        fmt_str = '<' + str(self.params.pipe_depth) + 'h'
        data = struct.unpack(fmt_str, buf)

        chan = self.model.pipe_chan
        data = [self.model.denormalize_input(chan, word) for word in data]
        self.model.update_data_log_block(chan, data)

    '''
    ADC param update handling
    '''
    def update_adc_os(self, value):
        self.update_model_and_fpga(self.params.adc_os_addr, 0, value)
        print "ADC oversample mode set to " + str(value)

    '''
    Channel param update handling
    '''
    def update_chan_enable(self, chan, enable):
        self.update_model_and_fpga(self.params.chan_en_addr, chan, enable)

        # Wake worker thread if channel is activated. Sleep worker thread
        # if channel is deactivated and no other active channels remain.
        if enable == True:
            self.worker_wake.set()
        elif self.model.num_active_chans() == 0:
            self.worker_wake.clear()

        print self.model.chan_to_string(chan) + " activated" if enable else " deactivated"

    def update_pipe_chan(self, chan):
        self.update_model_and_fpga(self.params.pipe_chan_addr, chan, 1)

    def update_chan_src_sel(self, chan, inpt):
        if self.model.is_valid_input(inpt):
            self.update_model_and_fpga(self.params.chan_src_sel_addr, chan, inpt)
            print self.model.chan_to_string(chan) + " input set to " + self.model.input_to_string(inpt)
        else:
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
        print self.model.chan_to_string(chan) + " PID setpoint set to " + str(value)

    def update_pid_p_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_p_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " P coefficient set to " + str(value)

    def update_pid_i_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_i_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " I coefficient set to " + str(value)

    def update_pid_d_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_d_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " D coefficient set to " + str(value)

    def update_pid_lock_en(self, chan, checked):
        self.update_model_and_fpga(self.params.pid_lock_en_addr, chan, checked)
        print self.model.chan_to_string(chan) + " PID lock " + " enabled" if checked else "disabled"

    '''
    Output filter param update handling
    '''
    def update_opt_init(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_init_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output init value set to " + str(value)

    def update_opt_init(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_init_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output init set to " + str(value)

    def update_opt_max(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_max_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output max set to " + str(value)

    def update_opt_min(self, chan, value):
        norm_value = self.model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opt_min_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output min set to " + str(value)

    def update_opt_mult(self, chan, value):
        self.update_model_and_fpga(self.params.opt_mult_addr, chan, value)
        print self.model.chan_to_string(chan) + " output multiplier set to " + str(value)

    def update_opt_rs(self, chan, value):
        self.update_model_and_fpga(self.params.opt_rs_addr, chan, value)
        print self.model.chan_to_string(chan) + " output right shift set to " + str(value)

    def update_opt_add_chan(self, chan, value):
        self.update_model_and_fpga(self.params.opt_add_chan_addr, chan, value)
        print self.model.chan_to_string(chan) + " add channel set to " + self.model.chan_to_string(value)

    '''
    Request handling
    '''
    def request_chan_reset(self, chan):
        if model.chan_en[chan]:
            self.update_chan_enable(chan, False);

        self.request_ovr_clear(chan)
        self.request_pid_clear(chan)
        self.request_opt_clear(chan)

        if model.chan_en[chan]:
            self.update_chan_enable(chan, True);

        print self.model.chan_to_string(chan) + " reset"

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
        self.adc_started = True
        self.fpga.activate_sys_trigger(self.params.adc_cstart_offset)
        print "ADC started"

    def trigger_dac_ref_set(self):
        self.dac_ref_set = True
        self.fpga.activate_sys_trigger(self.params.dac_rset_offset)

    def trigger_sys_reset(self):
        self.fpga.activate_sys_trigger(self.params.sys_rst_offset)

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

            # wait for pid lock array to become activate
            while not self.controller.worker_wake.isSet():
                self.controller.worker_wake.wait()

            self.controller.read_fpga_data()
            self.emit(SIGNAL('new_plot_data()')) # signal gui that new data is available to be plotted
            time.sleep(self.controller.sample_period) # sleep for a period of time

    def shutDown(self):
        self.exiting = True
        self.wait()
