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

        self.update_view()

        self.init_worker_thread()

    '''
    Register all input view handlers
    '''
    def register_view_handlers(self):
        # Tab handlers
        self.view.tab_widget.currentChanged.connect(
                lambda: self.update_chan_focus(self.view.tab_widget.currentIndex()))

        # Global params view handlers
        gp_view = self.view.gp_view
        gp_view.adc_os.currentIndexChanged.connect(
                lambda: self.update_adc_os(gp_view.adc_os.currentIndex() + 1))
        gp_view.sample_period.editingFinished.connect(
                lambda: self.update_sampling_period(float(gp_view.sample_period.text())))
        gp_view.block_transfer.clicked.connect(
                lambda: self.update_block_transfer(gp_view.block_transfer.checked()))
        gp_view.adc_cstart.clicked.connect(
                lambda: self.trigger_adc_cstart())
        gp_view.dac_ref_set.clicked.connect(
                lambda: self.trigger_dac_ref_set())
        gp_view.sys_reset.clicked.connect(
                lambda: self.trigger_sys_reset())

        # Channel view handlers
        for chan in range(self.params.n_out):
            self.register_chan_view_handlers(chan)

    '''
    Register view handlers for the specified channel
    '''
    def register_chan_view_handlers(self, chan):
        chan_view = self.view.chan_views[chan]
        input_view = chan_view.input_view
        pid_view = chan_view.pid_view
        output_view = chan_view.output_view

        # Input view handlers
        input_view.chan_input_sel.currentIndexChanged.connect(
                lambda: self.update_chan_input_sel(chan, input_view.chan_input_sel.currentIndex() - 1))
        input_view.osf_ovr_wgt.currentIndexChanged.connect(
                lambda: self.update_osf_ovr(chan, input_view.osf_over.wgt.currentIndex()))
        input_view.osf_cycle_delay.textChanged.connect(
                lambda: self.update_osf_cycle_delay(chan, int(self.osf_cycle_delay.text())))
        input_view.chan_activate.toggled.connect(
                lambda: self.update_chan_activate(chan, input_view.chan_activate.isChecked()))

        # PID view handlers
        pid_view.pid_setpoint.textChanged.connect(
                lambda: self.update_pid_setpoint(chan, float(pid_view.pid_setpoint.text())))
        pid_view.pid_p_coef.textChanged.connect(
                lambda: self.update_pid_p_coef(chan, int(view.pid_p_coef.text())))
        pid_view.pid_i_coef.textChanged.connect(
                lambda: self.update_pid_i_coef(chan, int(view.pid_i_coef.text())))
        pid_view.pid_d_coef.textChanged.connect(
                lambda: self.update_pid_d_coef(chan, int(view.pid_d_coef.text())))
        pid_view.pid_lock_en.stateChanged.connect(
                lambda: self.update_pid_lock_en(chan, view.pid_lock_en.checked()))
        pid_view.pid_clear.clicked.connect(
                lambda: self.handle_pid_clear(chan))

        # Output view handlers
        output_view.opp_init.textChanged.connect(
                lambda: self.update_opp_init(chan, float(output_view.opp_init.text())))
        output_view.opp_max.textChanged.connect(
                lambda: self.update_opp_max(chan, float(output_view.opp_max.text())))
        output_view.opp_min.textChanged.connect(
                lambda: self.update_opp_min(chan, float(output_view.opp_min.text())))
        output_view.opp_mult.textChanged.connect(
                lambda: self.update_opp_mult(chan, int(output_view.opp_mult.text())))
        output_view.opp_right_shift_wgt.textChanged.connect(
                lambda: self.update_opp_right_shift(chan, int(output_view.opp_right_shift_wgt.text())))

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
        chan_focus = self.model.chan_focus
        if self.block_transfer:
            [data_x, data_y] = self.model.get_pipe_out_data(chan_focus)
        else:
            [data_x, data_y] = self.model.get_wire_out_data(chan_focus)

        self.view.update_graph(chan_focus, data_x, data_y)

    '''
    Update fpga data sampling period
    '''
    def update_sampling_period(self, value):
        self.sample_period = value
        print "Sample period set to " + str(value)

    '''
    Update block transfer state
    '''
    def update_block_transfer(self, checked):
        if checked == True:
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
        self.model.write_data(addr, chan, value)
        self.fpga.write_data(addr, chan, value)

    '''
    Read data from fpga and update model
    '''
    def read_fpga_data(self):
        read_wire_out_data()
        read_pipe_out_data()

    '''
    Read wire-out data from fpga and store in model
    '''
    def read_wire_out_data(self):
        for chan in range(self.params.n_out):
            osf_data_owep = self.params.osf_data0_owep + chan
            data = self.fpga.get_wire_out_value(osf_data_owep)
            data = self.uint16_to_int32(data)
            data = self.model.denormalize_input(chan, data)
            self.model.update_wire_out_data(chan, time.time(), data)

    '''
    Read pipe-out data from fpga and store in model
    '''
    def read_pipe_out_data(self):
        buf = bytearray(params.pipe_depth * 2)
        self.fpga.read_from_pipe_out(self.params.osf_data_opep, buf)

        # unpack byte array as array of signed shorts
        fmt_str = '<' + str(block_size) + 'h'
        data = struct.unpack(fmt_str, buf)

        data = [self.model.denormalize_input(chan, word) for word in data]
        self.model.update_pipe_out_data(chan, data)

    '''
    ADC param update functions
    '''
    def update_adc_os(self, value):
        self.update_model_and_fpga(self.params.adc_os_addr, 0, value)
        print "ADC oversample mode set to " + str(value)

    '''
    Channel param update functions
    '''
    def update_chan_activate(self, chan, checked):
        self.update_model_and_fpga(self.params.chan_activate_addr, chan, checked)

        # Wake worker thread if channel is activated. Sleep worker thread
        # if channel is deactivated and no other active channels remain.
        if checked == True:
            self.worker_wake.set()
        elif model.num_active_chans() == 0:
            self.worker_wake.clear()

        print self.model.chan_to_string(chan) + " activated" if checked else " deactivated"

    def update_chan_focus(self, chan):
        self.update_model_and_fpga(self.params.chan_focus_addr, chan, 1)

    def update_chan_input_sel(self, chan, inpt):
        if self.model.is_valid_input(inpt):
            self.update_model_and_fpga(self.params.chan_input_sel_addr, chan, inpt)
            print self.model.chan_to_string(chan) + " input set to " + self.model.inpt_to_string(inpt)
        else:
            self.update_model_and_fpga(self.params.chan_input_sel_addr, chan, self.params.null_input)
            print self.model.chan_to_string(chan) + " input deactivated"

    '''
    OSF param update functions
    '''
    def update_osf_ovr(self, chan, value):
        self.update_model_and_fpga(self.params.osf_ovr_addr, chan, value)
        print self.model.chan_to_string(chan) + " oversample ratio set to " + str(2**value)

    def update_osf_cycle_delay(self, chan, value):
        self.update_model_and_fpga(self.params.osf_cycle_delay_addr, chan, value)
        print self.model.chan_to_string(chan) + " cycle delay set to " + str(value)

    '''
    PID param update functions
    '''
    def update_pid_setpoint(self, chan, value):
        norm_value = model.normalize_input(chan, value);
        self.update_model_and_fpga(self.params.pid_setpoint_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " PID setpoint set to " + str(value)

    def update_p_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_p_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " P coefficient set to " + str(value)

    def update_i_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_i_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " I coefficient set to " + str(value)

    def update_d_coef(self, chan, value):
        self.update_model_and_fpga(self.params.pid_d_coef_addr, chan, value)
        print self.model.chan_to_string(chan) + " D coefficient set to " + str(value)

    def update_pid_lock_en(self, chan, checked):
        self.update_model_and_fpga(self.params.pid_lock_en_addr, chan, checked)
        print self.model.chan_to_string(chan) + " PID lock " + " enabled" if checked else "disabled"

    def handle_pid_clear(self, chan):
        # There is no PID clear option implemented in HDL. The effect is
        # accomplished by disabling and enabling the PID lock.
        if self.model.pid_lock_en(chan):
            update_pid_lock_en(chan, 0)
            update_pid_lock_en(chan, 1)
            print self.model.chan_to_string(chan) + " PID lock cleared"

    '''
    OPP param update functions
    '''
    def update_opp_init(self, value):
        norm_value = model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opp_init_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output init value set to " + str(value)

    def update_opp_init(self, value):
        norm_value = model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opp_init_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output init set to " + str(value)

    def update_opp_max(self, value):
        norm_value = model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opp_max_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output max set to " + str(value)

    def update_opp_min(self, value):
        norm_value = model.normalize_output(chan, value)
        self.update_model_and_fpga(self.params.opp_min_addr, chan, norm_value)
        print self.model.chan_to_string(chan) + " output min set to " + str(value)

    def update_opp_mult(self, value):
        self.update_model_and_fpga(self.params.opp_mult_addr, chan, value)
        print self.model.chan_to_string(chan) + " output multiplier set to " + str(value)

    def update_opp_rs(self, value):
        self.update_model_and_fpga(self.params.opp_rs_addr, chan, value)
        print self.model.chan_to_string(chan) + " output right shift set to " + str(value)

    '''
    Trigger activation functions
    '''
    def trigger_adc_cstart(self):
        self.adc_started = True
        self.fpga.activate_sys_trigger(self.params.adc_cstart_offset)
        print "ADC started"

    def trigger_dac_ref_set(self):
        self.dac_ref_set = True
        self.fpga.activate_sys_trigger(self.params.dac_ref_set_offset)

    def trigger_sys_reset(self):
        self.fpga.activate_sys_trigger(self.params.sys_reset_offset)

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
            time.sleep(self.controller.polling_period) # sleep for a period of time

    def shutDown(self):
        self.exiting = True
        self.wait()
