import sys
import time
import threading
import sip
import pyqtgraph as pg
import numpy as np
from PyQt4.Qt import *

qt_app = QApplication(sys.argv)

class View(QWidget):
    def __init__(self, params):
        QWidget.__init__(self)
        self.setWindowTitle('PID Lock Array')

        # create channel views
        n_chan = params.n_dac + params.n_dds
        self.chan_views = [ChannelView(chan_no) for chan_no in range(params.n_dac)]
        for x in range(params.n_dds):
            self.chan_views += [ChannelView(chan_no) for chan_no in range(3)]

        # initalize global params view
        self.gp_view = GlobalParamsView(params)
        self.layout = QHBoxLayout()
        self.layout.addWidget(self.gp_view)

        # create output tabs
        opt_tabs = QTabWidget()

        # create dac tab layout
        dac_tabs = QTabWidget()
        for x in range(params.n_dac):
            dac_tabs.addTab(self.chan_views[x], 'OUT' + str(x+1))
        opt_tabs.addTab(dac_tabs, 'DAC')

        # create dds tab layout
        dds_tabs = QTabWidget()
        for x in range(params.n_dds):
            dds_tabs = QTabWidget()
            fidx = params.freq0_addr + x
            pidx = params.phase0_addr + x
            aidx = params.amp0_addr + x
            dds_tabs.addTab(self.chan_views[fidx], 'FREQ')
            dds_tabs.addTab(self.chan_views[pidx], 'PHASE')
            dds_tabs.addTab(self.chan_views[aidx], 'AMP')
            opt_tabs.addTab(dds_tabs, 'DDS' + str(x))

        self.layout.addWidget(opt_tabs)

        ## create channel stack
        #self.chan_stack = QStackedWidget()
        #for x in range(params.n_pid_chan):
            #self.chan_stack.addWidget(self.chan_views[x])
        #self.layout.addWidget(self.chan_stack)

        self.setLayout(self.layout)

    def update_graph(self, chan, data_x, data_y):
        self.chan_views[chan].update_graph(data_x, data_y)

    def get_open_file(self, title):
        return QFileDialog.getOpenFileName(self, title, '', '*.pid')

    def get_save_file(self, title):
        return QFileDialog.getSaveFileName(self, title, '', '*.pid')

    def closeEvent(self, event):
        QWidget.closeEvent(self, event)

    def run(self):
        self.show()
        sys.exit(qt_app.exec_())

class GlobalParamsView(QGroupBox):
    def __init__(self, params):
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
        self.form_layout.addRow('ADC oversample', self.adc_os)

        #################### sample period #######################
        self.sample_period = QLineEdit(self)

        # create and add input validator
        spv = QDoubleValidator(0.0, 10.0, 3)
        self.sample_period.setValidator(spv)

        self.form_layout.addRow('Sampling period (s)', self.sample_period)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        #################### block_transfer #######################
        self.block_transfer = QPushButton('Block logging', self)
        self.block_transfer.setCheckable(True)
        self.layout.addWidget(self.block_transfer)

        #################### lock status array #######################
        self.chan_sel_arr = []
        for x in range(params.n_pid_chan):
            if x < params.freq0_addr:
                title = 'OUT' + str(x)
            elif x < params.phase0_addr:
                title = 'FREQ'
            elif x < params.amp0_addr:
                title = 'PHASE'
            else:
                title = 'AMP'
            cs_button = QPushButton(title, self)
            cs_button.setFlat(True)
            cs_button.setCheckable(True)
            self.chan_sel_arr.append(cs_button)

        chan_sel_tb = QToolBox()
        active_group = QGroupBox()
        chan_sel_tb.addItem(active_group, "Active")

        dac_group = QGroupBox()
        dac_layout = QVBoxLayout()
        for i in range(params.n_dac):
            dac_layout.addWidget(self.chan_sel_arr[params.dac0_addr + i])
        dac_layout.addStretch(1)
        dac_group.setLayout(dac_layout)
        chan_sel_tb.addItem(dac_group, 'DAC')

        for j in range(params.n_dds):
            dds_group = QGroupBox()
            dds_layout = QVBoxLayout()
            dds_layout.addWidget(self.chan_sel_arr[params.freq0_addr + j])
            dds_layout.addWidget(self.chan_sel_arr[params.phase0_addr + j])
            dds_layout.addWidget(self.chan_sel_arr[params.amp0_addr + j])
            dds_layout.addStretch(1)
            dds_group.setLayout(dds_layout)
            chan_sel_tb.addItem(dds_group, 'DDS' + str(j))

        self.layout.addWidget(chan_sel_tb)

        #################### save_config #######################
        self.save_config = QPushButton('Save Config', self)
        self.layout.addWidget(self.save_config)

        #################### load_model #######################
        self.load_model = QPushButton('Load Config', self)
        # self.load_model.setStyleSheet('background-color: red')
        self.layout.addWidget(self.load_model)

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
        self.mode_view = ModeView()
        self.error_view = ErrorView()
        self.pid_view = PIDView()
        self.proc_view = ProcessingView()
        self.output_view = OutputView()

        # initialize plot widget
        self.init_graph()

        # initialize graph controls and statistics
        self.init_graph_control()

        # create HBox layout; add channel control components
        self.cc_layout = QHBoxLayout()
        self.cc_layout.addWidget(self.mode_view)
        self.cc_layout.addWidget(self.error_view)
        self.cc_layout.addWidget(self.pid_view)
        self.cc_layout.addWidget(self.proc_view)
        self.cc_layout.addWidget(self.output_view)

        # create VBox layout; add channel control layout and error plot
        self.layout = QVBoxLayout()
        self.layout.addLayout(self.cc_layout)
        self.layout.addWidget(self.plotWidget)
        self.layout.addLayout(self.gcontrol_layout)

        # set the VBox layout as the tab's primary layout
        self.setLayout(self.layout)

    def init_graph(self):
        self.plotWidget = pg.PlotWidget()
        self.plotItem = self.plotWidget.getPlotItem()
        self.plotItem.setTitle('ADC Data Stream')
        self.plotItem.setLabel('bottom', 'Time', 's')
        self.plotItem.setLabel('left', 'ADC Data', 'V')

    def init_graph_control(self):
        self.gcontrol_layout = QHBoxLayout()

        self.graph_freeze = QPushButton('Freeze', self)
        self.graph_freeze.setCheckable(True)
        self.graph_clear = QPushButton('Clear', self)

        self.std_dev_fl = QFormLayout()
        self.std_dev = QLabel('0')
        self.std_dev_fl.addRow('std dev: ', self.std_dev)

        self.mean_fl = QFormLayout()
        self.mean = QLabel('0')
        self.mean_fl.addRow('mean: ', self.mean)

        self.gcontrol_layout.addWidget(self.graph_freeze)
        self.gcontrol_layout.addWidget(self.graph_clear)
        self.gcontrol_layout.addLayout(self.std_dev_fl)
        self.gcontrol_layout.addLayout(self.mean_fl)

    def update_graph(self, data_x, data_y):
        self.plotItem.clear()
        self.plotItem.plot(data_x, data_y)

        if data_x and data_y:
            std_dev = np.std(data_y)
            mean = np.mean(data_y)
        else:
            std_dev = 0
            mean = 0

        self.std_dev.setText(str(std_dev))
        self.mean.setText(str(mean))

# Source params widget
class ModeView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('Channel')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layout
        self.form_layout = QFormLayout()

        #################### chan_mode #######################
        self.chan_mode = QComboBox(self)
        self.chan_mode.addItems(['PID lock', 'Constant drive'])
        self.form_layout.addRow('Mode', self.chan_mode)

        #################### chan_src_sel #######################
        # create and fill source channel combo box
        self.chan_src_sel = QComboBox(self)

        # add source combo box to form with label
        self.form_layout.addRow('Input', self.chan_src_sel)
        self.layout.addLayout(self.form_layout)

        #################### pid_lock_en #######################
        self.pid_lock_en = QPushButton('Enable lock', self)
        self.pid_lock_en.setCheckable(True)
        self.layout.addWidget(self.pid_lock_en)

        #################### chan_reset #######################
        self.chan_reset = QPushButton('Channel reset', self)
        self.layout.addWidget(self.chan_reset)

        # set vbox as widget's main layout
        self.setLayout(self.layout)

# Error params
class ErrorView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('Error Signal')

        # create widget layout
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        #################### ovr_os #######################
        self.ovr_os = QComboBox(self)
        self.form_layout.addRow('Oversample', self.ovr_os)

        #################### pid_setpoint #######################
        self.pid_setpoint = QLineEdit(self)
        self.form_layout.addRow('Setpoint', self.pid_setpoint)
        self.layout.addLayout(self.form_layout)

        #################### pid_inv_error #######################
        self.pid_inv_error = QCheckBox('Invert Error', self)
        self.layout.addWidget(self.pid_inv_error)

        # set vbox as widget's main layout
        self.setLayout(self.layout)

class PIDView(QGroupBox):

    def __init__(self):
        # initialize widget
        QGroupBox.__init__(self)
        self.setTitle('PID Filter')

        # create widget layout
        self.layout = QVBoxLayout()

        # create form layouts
        self.form_layout = QFormLayout()

        #################### pid_p_coef #######################
        self.pid_p_coef = QSpinBox(self)
        self.pid_p_coef.setMinimumWidth(100)
        self.form_layout.addRow('P coef', self.pid_p_coef)

        #################### pid_i_coef #######################
        self.pid_i_coef = QSpinBox(self)
        self.pid_i_coef.setMinimumWidth(100)
        self.form_layout.addRow('I coef', self.pid_i_coef)

        #################### pid_d_coef #######################
        self.pid_d_coef = QSpinBox(self)
        self.pid_d_coef.setMinimumWidth(100)
        self.form_layout.addRow('D coef', self.pid_d_coef)

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
        self.opt_mult = QSpinBox(self)
        self.opt_mult.setMinimumWidth(100)
        self.opt_mult.setMaximum(1000)
        self.form_layout.addRow('Multiplier', self.opt_mult)

        #################### opt_rs #######################
        self.opt_rs = QSpinBox(self)
        self.opt_rs.setMinimumWidth(100)
        self.opt_rs.setMaximum(1000)
        self.form_layout.addRow('Right shift', self.opt_rs)

        #################### scale_factor #######################
        self.scale_factor = QLineEdit(self)
        self.scale_factor.setEnabled(False)
        self.scale_factor.setStyleSheet("QLineEdit {background-color: tomato; }")
        self.form_layout.addRow('Scale factor', self.scale_factor)
        self.scale_factor.setEnabled(False)

        #################### opt_add_chan #######################
        # create and fill source channel combo box
        self.opt_add_chan = QComboBox(self)

        # add source combo box to form with label
        self.form_layout.addRow('Add channel ', self.opt_add_chan)

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
        self.opt_init.setMinimumWidth(100)
        self.form_layout.addRow('Initial', self.opt_init)

        #################### opt_max #######################
        self.opt_max = QLineEdit(self)
        self.opt_max.setMinimumWidth(100)
        self.form_layout.addRow('Max', self.opt_max)

        #################### opt_min #######################
        self.opt_min = QLineEdit(self)
        self.opt_min.setMinimumWidth(100)
        self.form_layout.addRow('Min', self.opt_min)

        # add form layout to main layout
        self.layout.addLayout(self.form_layout)

        #################### opt_inject #######################
        self.opt_inject = QPushButton('Write initial', self)
        self.layout.addWidget(self.opt_inject)

        # set the VBox as the main layout
        self.setLayout(self.layout)


