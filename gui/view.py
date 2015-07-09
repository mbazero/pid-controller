import sys
import time
import threading
import sip
import pyqtgraph as pg
import numpy as np
from PyQt4.Qt import *


'''
TODO
- automatic interal reference setting
- change plot x-axis so current time 0 is on right
- implement algorithm to determine the focused channel in the tab layout
- add lock that restricts access to FPGA to one thread
    >> NO...you only want one thread accessing the FPGA b/c all wires are updated at once
- still need to figure out threading issue (sometimes you still get segmentation fault, other times you get bus error)
- create new lock channel class which stores all variables and defines all handler methods
    > an instance of this class is passed (instead of okc) to all GUI vals
- overhaul update enable system
    > ideally, replace it altogether witch the signle endpoint system with a bunch of triggers to all modules
    > however, if you keep the current system, you must implement a function which converts a decimal channel number to one-hot encoded update enable signal
- enable float inputs for setpoint and OPP min/max/init
- implement conversion of voltage values to digital for sending to FPGA
    > PID Setpoint
    > OPP init/min/max
        = for this one you must also add algorithmic support for writing two all three endpoints if need be. right now you only write to one for simplicity
- figure out way to allow channels to not have a source
- conversely, figure out a way to have one source drive multiple outputs
- change signals to new style
- resolve workerthread segmentation fault issue
'''
qt_app = QApplication(sys.argv)

class View(QWidget):
    def __init__(self, params):
        QWidget.__init__(self)
        self.setWindowTitle('PID Lock Array')

        # create channel views
        n_chan = params.n_dac + params.n_dds
        self.dac_views = [ChannelView(chan_no) for chan_no in range(0, params.n_dac)]
        self.dds_views = [ChannelView(chan_no) for chan_no in range(0, params.n_dds)]
        self.chan_views = self.dac_views + self.dds_views

        # initalize global params view
        self.gp_view = GlobalParamsView(params.n_dac, params.n_dds)
        self.layout = QHBoxLayout()
        self.layout.addWidget(self.gp_view)

        # create tabs and add channel views
        self.tab_widget = QTabWidget()
        for j in range(params.n_dac):
            self.tab_widget.addTab(self.dac_views[j], "DAC " + str(j))
        for k in range(params.n_dds):
            self.tab_widget.addTab(self.dds_views[k], "DDS " + str(k))
        self.layout.addWidget(self.tab_widget)

        self.setLayout(self.layout)

    def update_graph(self, chan, data_x, data_y):
        self.chan_views[chan].update_graph(data_x, data_y)

    def closeEvent(self, event):
        QWidget.closeEvent(self, event)

    def run(self):
        self.show()
        sys.exit(qt_app.exec_())

class GlobalParamsView(QGroupBox):
    def __init__(self, n_dac, n_dds):
        QWidget.__init__(self)
        self.setTitle('Global Parameters')

        # create VBox layout for window
        self.layout = QVBoxLayout()

        # create form layout to store global params
        self.form_layout = QFormLayout()

        #################### adc_os #######################

        # create combo box for ADC oversampling ratios
        self.adc_os_ops     = [ '2',
                                '4',
                                '8',
                                '16',
                                '32',
                                '64']
        self.adc_os = QComboBox(self)
        self.adc_os.addItems(self.adc_os_ops)
        self.form_layout.addRow('ADC Oversample Ratio:', self.adc_os)

        #################### sample period #######################
        self.sample_period = QLineEdit(self)

        # create and add input validator
        spv = QDoubleValidator(0.0, 10.0, 3)
        self.sample_period.setValidator(spv)

        self.form_layout.addRow('Sampling period (s): ', self.sample_period)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        #################### block_transfer #######################
        self.block_transfer = QPushButton('Block Transfer Mode', self)
        self.block_transfer.setCheckable(True)
        self.layout.addWidget(self.block_transfer)

        #################### lock status array #######################
        self.dac_status_layout = QVBoxLayout()
        #self.dac_status_layout.setTitle('DAC Status')
        self.dds_status_layout = QVBoxLayout()
        #self.dds_status_layout.setTitle('DDS Status')
        self.dac_status_arr = [QPushButton('DAC Channel ' + str(count), self) for count in range(n_dac)]
        self.dds_status_arr = [QPushButton('DDS Channel ' + str(count), self) for count in range(n_dds)]

        for i in range(n_dac) :
            self.dac_status_arr[i].setStyleSheet('background-color: grey')
            self.dac_status_layout.addWidget(self.dac_status_arr[i])

        for j in range(n_dds) :
            self.dds_status_arr[j].setStyleSheet('background-color: grey')
            self.dds_status_layout.addWidget(self.dds_status_arr[j])

        self.layout.addLayout(self.dac_status_layout)
        self.layout.addLayout(self.dds_status_layout)
        self.layout.addStretch(1)

        #################### adc_cstart #######################
        self.adc_cstart = QPushButton('Start ADC', self)
        self.adc_cstart.setCheckable(True)
        self.layout.addWidget(self.adc_cstart)

        #################### dac_ref_set #######################
        self.dac_ref_set = QPushButton('Set DAC Reference', self)
        # self.dac_ref_set.setStyleSheet('background-color: red')
        self.layout.addWidget(self.dac_ref_set)

        #################### sys_reset #######################
        self.sys_reset = QPushButton('System Reset', self)
        self.layout.addWidget(self.sys_reset)

        # set widget layout as form layout
        self.setLayout(self.layout)

class ChannelView(QWidget):
    def __init__(self, chan_no):
        # initialize widget
        QWidget.__init__(self)
        self.chan_no = chan_no

        # initialize channel control components
        self.input_view = InputView()
        self.pid_view = PIDView()
        self.proc_view = ProcessingView()
        self.output_view = OutputView()

        # initialize plot widget
        self.init_graph()

        # initialize plot statistics
        self.initStats()

        # create HBox layout; add channel control components
        self.cc_layout = QHBoxLayout()
        self.cc_layout.addWidget(self.input_view)
        self.cc_layout.addWidget(self.pid_view)
        self.cc_layout.addWidget(self.proc_view)
        self.cc_layout.addWidget(self.output_view)

        # create VBox layout; add channel control layout and error plot
        self.layout = QVBoxLayout()
        self.layout.addLayout(self.cc_layout)
        self.layout.addWidget(self.plotWidget)
        self.layout.addLayout(self.stats_layout)

        # set the VBox layout as the tab's primary layout
        self.setLayout(self.layout)

    def init_graph(self):
        self.plotWidget = pg.PlotWidget()
        self.plotItem = self.plotWidget.getPlotItem()
        self.plotItem.setTitle('ADC Data Stream')
        self.plotItem.setLabel('bottom', 'Time', 's')
        self.plotItem.setLabel('left', 'ADC Data', 'V')

    def initStats(self):
        self.stats_layout = QHBoxLayout()

        self.std_dev_fl = QFormLayout()
        self.std_dev = QLabel('0')
        self.std_dev_fl.addRow('std dev: ', self.std_dev)

        self.mean_fl = QFormLayout()
        self.mean = QLabel('0')
        self.mean_fl.addRow('mean: ', self.mean)

        self.stats_layout.addLayout(self.std_dev_fl)
        self.stats_layout.addLayout(self.mean_fl)

    def update_graph(self, data_x, data_y):
        self.plotItem.clear()
        self.plotItem.plot(data_x, data_y)
        self.std_dev.setText(str(np.std(data_y)))
        self.mean.setText(str(np.mean(data_y)))

# Source params widget
class InputView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('Input')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layout
        self.form_layout = QFormLayout()

        #################### chan_src_sel #######################
        # create and fill source channel combo box
        self.chan_src_sel = QComboBox(self)

        # add source combo box to form with label
        self.form_layout.addRow('Input Channel: ', self.chan_src_sel)

        #################### ovr_os #######################
        self.ovr_os_ops = ['Off'] + [str(2**x) for x in range(1, 10)]

        # create and fill oversampling combo box
        self.ovr_os = QComboBox(self)
        self.ovr_os.addItems(self.ovr_os_ops)

        # add oversampling combo box to form with label
        self.form_layout.addRow('Oversample Ratio: ', self.ovr_os)
        self.layout.addLayout(self.form_layout)

        #################### chan_en #######################
        self.chan_en = QPushButton('Enable', self)
        self.chan_en.setCheckable(True)
        self.layout.addWidget(self.chan_en)

        #################### chan_reset #######################
        self.chan_reset = QPushButton('Reset', self)
        self.layout.addWidget(self.chan_reset)

        # set vbox as widget's main layout
        self.setLayout(self.layout)


class PIDView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('PID Filter')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layout
        self.form_layout = QFormLayout()

        #################### pid_setpoint #######################
        self.pid_setpoint = QLineEdit(self)

        # create and add apply input validator
        ps_validator = QDoubleValidator(-5.0, 5.0, 3)
        self.pid_setpoint.setValidator(ps_validator)

        self.pid_setpoint.setPlaceholderText('-5 to 5V')
        self.form_layout.addRow('Setpoint:', self.pid_setpoint)

        #################### pid_p_coef #######################
        self.pid_p_coef = QLineEdit(self)

        # create and apply input validator
        pcoef_validator = QIntValidator(0, 32768)
        self.pid_p_coef.setValidator(pcoef_validator)

        self.pid_p_coef.setPlaceholderText('0 to 32768')
        self.form_layout.addRow('P Coef:', self.pid_p_coef)

        #################### pid_i_coef #######################
        self.pid_i_coef = QLineEdit(self)

        # apply input validator
        self.pid_i_coef.setValidator(pcoef_validator)

        self.pid_i_coef.setPlaceholderText('0 to 32768')
        self.form_layout.addRow('I Coef:', self.pid_i_coef)


        #################### pid_d_coef #######################
        self.pid_d_coef = QLineEdit(self)

        # apply input validator
        self.pid_d_coef.setValidator(pcoef_validator)

        self.pid_d_coef.setPlaceholderText('0 to 32768')
        self.form_layout.addRow('D Coef:', self.pid_d_coef)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        #################### pid_clear #######################
        self.pid_clear = QPushButton('Clear', self)
        self.layout.addWidget(self.pid_clear)

        # set the VBox as the main layout
        self.setLayout(self.layout)

class ProcessingView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('Post Processor')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layout
        self.form_layout = QFormLayout()

        #################### opt_multiplier #######################
        self.opt_mult = QLineEdit(self)

        # create and apply input validator
        mult_validator = QIntValidator(-2**9, 2**9 - 1)
        self.opt_mult.setValidator(mult_validator)

        self.opt_mult.setPlaceholderText(str(-2**9) + ' to ' + str(2**9 - 1))
        self.form_layout.addRow('Multiplier:', self.opt_mult)

        #################### opt_rs #######################
        self.opt_rs_wgt = QLineEdit(self)

        # create and apply input validator
        rs_validator = QIntValidator(0, 2**16 - 1)
        self.opt_rs_wgt.setValidator(rs_validator)

        self.opt_rs_wgt.setPlaceholderText(str(0) + ' to ' + str(2**16 - 1))
        self.form_layout.addRow('Right shift:', self.opt_rs_wgt)

        #################### opt_add_chan #######################
        # create and fill source channel combo box
        self.opt_add_chan = QComboBox(self)

        # add source combo box to form with label
        self.form_layout.addRow('Add channel: ', self.opt_add_chan)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        # set the VBox as the main layout
        self.setLayout(self.layout)

class OutputView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('Output')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layout
        self.form_layout = QFormLayout()

        #################### opt_init #######################
        self.opt_init = QLineEdit(self)

        # create and apply input validator
        init_validator = QDoubleValidator(0.0, 5.0, 2)
        self.opt_init.setValidator(init_validator)

        self.opt_init.setPlaceholderText('0 to 5V')
        self.form_layout.addRow('Initial:', self.opt_init)

        #################### opt_max #######################
        self.opt_max = QLineEdit(self)

        # create and apply input validator
        max_validator = QDoubleValidator(0.0, 5.0, 2)
        self.opt_max.setValidator(max_validator)

        self.opt_max.setPlaceholderText('0 to 5V')
        self.form_layout.addRow('Max:', self.opt_max)

        #################### opt_min #######################
        self.opt_min = QLineEdit(self)

        # create and apply input validator
        min_validator = QDoubleValidator(0.0, 5.0, 2)
        self.opt_min.setValidator(min_validator)

        self.opt_min.setPlaceholderText('0 to 5V')
        self.form_layout.addRow('Min:', self.opt_min)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        #################### opt_inject #######################
        self.opt_inject = QPushButton('Inject', self)
        self.layout.addWidget(self.opt_inject)

        #################### opt_clear #######################
        self.opt_clear = QPushButton('Clear', self)
        self.layout.addWidget(self.opt_clear)

        # set the VBox as the main layout
        self.setLayout(self.layout)


