import sys
import time
import threading
import sip
import pyqtgraph as pg
import numpy as np
from PyQt4.Qt import *

qt_app = QApplication(sys.argv)

'''
Display PID controller GUI
'''
class View(QWidget):
    def __init__(self, io_config, params):
        QWidget.__init__(self)
        self.setWindowTitle('PID Lock Array')

        # Create channel views
        n_chan = params.n_dac + params.n_dds
        self.chan_views = [ChannelView(chan_no) for chan_no in range(params.n_dac)]
        for x in range(params.n_dds):
            self.chan_views += [ChannelView(chan_no) for chan_no in range(3)]

        # Initalize global params view
        self.layout = QHBoxLayout()
        self.gp_view = GlobalParamsView(io_config, params)
        self.layout.addWidget(self.gp_view)

        # Create output tabs
        opt_tabs = QTabWidget()

        # Create dac tab layout
        dac_tabs = QTabWidget()
        for x in range(params.n_dac):
            dac_tabs.addTab(self.chan_views[x], 'OUT' + str(x+1))
        opt_tabs.addTab(dac_tabs, 'DAC')

        # Create dds tab layout
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

'''
Display global parameters
'''
class GlobalParamsView(QGroupBox):
    def __init__(self, io_config, params):
        QWidget.__init__(self)
        self.setTitle('Global Parameters')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # ADC oversample combo box
        self.adc_os = QComboBox(self)
        adc_os_ops = [str(mode) for mode in io_config.adc_os_modes]
        self.adc_os.addItems(adc_os_ops)
        self.form_layout.addRow('ADC oversample', self.adc_os)

        # Sample period line edit
        self.sample_period = QLineEdit(self)
        spv = QDoubleValidator(0.0, 10.0, 3)
        self.sample_period.setValidator(spv)
        self.form_layout.addRow('Sampling period (s)', self.sample_period)

        self.layout.addLayout(self.form_layout)

        # Block transfer push button
        self.block_transfer = QPushButton('Block logging', self)
        self.block_transfer.setCheckable(True)
        self.layout.addWidget(self.block_transfer)

        # Lock status quick view tool box
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

        # Save configuration push button
        self.save_config = QPushButton('Save Config', self)
        self.layout.addWidget(self.save_config)

        # Load configuration push button
        self.load_config = QPushButton('Load Config', self)
        # self.load_config.setStyleSheet('background-color: red')
        self.layout.addWidget(self.load_config)

        # System reset push button
        self.sys_reset = QPushButton('System Reset', self)
        self.layout.addWidget(self.sys_reset)

        self.setLayout(self.layout)

'''
Display channel configuration parameters
'''
class ChannelView(QWidget):
    def __init__(self, chan_no):
        QWidget.__init__(self)
        self.chan_no = chan_no

        # Create and place channel control componenets
        self.mode_view = ModeView()
        self.error_view = ErrorView()
        self.pid_view = PIDView()
        self.proc_view = ProcessingView()
        self.output_view = OutputView()

        self.cc_layout = QHBoxLayout()
        self.cc_layout.addWidget(self.mode_view)
        self.cc_layout.addWidget(self.error_view)
        self.cc_layout.addWidget(self.pid_view)
        self.cc_layout.addWidget(self.proc_view)
        self.cc_layout.addWidget(self.output_view)

        # Initialize graphing area
        self.init_graph()
        self.init_graph_control()

        # Place graph and channel controls
        self.layout = QVBoxLayout()
        self.layout.addLayout(self.cc_layout)
        self.layout.addWidget(self.plotWidget)
        self.layout.addLayout(self.gcontrol_layout)

        self.setLayout(self.layout)

    '''
    Initialize graph plot
    '''
    def init_graph(self):
        self.plotWidget = pg.PlotWidget()
        self.plotItem = self.plotWidget.getPlotItem()
        self.plotItem.setTitle('ADC Data Stream')
        self.plotItem.setLabel('bottom', 'Time', 's')
        self.plotItem.setLabel('left', 'ADC Data', 'V')

    '''
    Initialize graph control and statistics
    '''
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

    '''
    Update graph with supplied data
    '''
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

'''
Display channel mode parameters
'''
class ModeView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('Channel')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Channel mode combo box
        self.chan_mode = QComboBox(self)
        self.chan_mode.addItems(['PID lock', 'Constant drive'])
        self.form_layout.addRow('Mode', self.chan_mode)

        # Channel source select combo box
        self.chan_src_sel = QComboBox(self)
        self.form_layout.addRow('Input', self.chan_src_sel)
        self.layout.addLayout(self.form_layout)

        # PID lock enable button
        self.pid_lock_en = QPushButton('Enable lock', self)
        self.pid_lock_en.setCheckable(True)
        self.layout.addWidget(self.pid_lock_en)

        # Channel reset button
        self.chan_reset = QPushButton('Channel reset', self)
        self.layout.addWidget(self.chan_reset)

        self.setLayout(self.layout)

'''
Display error signal parameters
'''
class ErrorView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('Error Signal')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Oversample mode combo box
        self.ovr_os = QComboBox(self)
        self.form_layout.addRow('Oversample', self.ovr_os)

        # PID setpoint line edit
        self.pid_setpoint = QLineEdit(self)
        self.form_layout.addRow('Setpoint', self.pid_setpoint)
        self.layout.addLayout(self.form_layout)

        # PID invert error check box
        self.pid_inv_error = QCheckBox('Invert Error', self)
        self.layout.addWidget(self.pid_inv_error)

        self.setLayout(self.layout)

'''
Display PID parameters
'''
class PIDView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('PID Filter')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # PID P coefficient spin box
        self.pid_p_coef = QSpinBox(self)
        self.pid_p_coef.setMinimumWidth(100)
        self.form_layout.addRow('P coef', self.pid_p_coef)

        # PID I coefficient spin box
        self.pid_i_coef = QSpinBox(self)
        self.pid_i_coef.setMinimumWidth(100)
        self.form_layout.addRow('I coef', self.pid_i_coef)

        # PID D coefficient spin box
        self.pid_d_coef = QSpinBox(self)
        self.pid_d_coef.setMinimumWidth(100)
        self.form_layout.addRow('D coef', self.pid_d_coef)

        self.layout.addLayout(self.form_layout)

        # PID clear button
        self.pid_clear = QPushButton('Clear', self)
        self.layout.addWidget(self.pid_clear)

        self.setLayout(self.layout)

'''
Display channel output processing parameters
'''
class ProcessingView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('Post Processor')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Multiplier spin box
        self.opt_mult = QSpinBox(self)
        self.opt_mult.setMinimumWidth(100)
        self.opt_mult.setMaximum(1000)
        self.form_layout.addRow('Multiplier', self.opt_mult)

        # Right shift spin box
        self.opt_rs = QSpinBox(self)
        self.opt_rs.setMinimumWidth(100)
        self.opt_rs.setMaximum(1000)
        self.form_layout.addRow('Right shift', self.opt_rs)

        # Scale factor display box
        self.scale_factor = QLineEdit(self)
        self.scale_factor.setEnabled(False)
        self.scale_factor.setStyleSheet("QLineEdit {background-color: tomato; }")
        self.form_layout.addRow('Scale factor', self.scale_factor)
        self.scale_factor.setEnabled(False)

        # Add channel combo box
        self.opt_add_chan = QComboBox(self)
        self.form_layout.addRow('Add channel ', self.opt_add_chan)

        self.layout.addLayout(self.form_layout)
        self.setLayout(self.layout)

'''
Display channel output parameters
'''
class OutputView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('Output')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Initial output line edit
        self.opt_init = QLineEdit(self)
        self.opt_init.setMinimumWidth(100)
        self.form_layout.addRow('Initial', self.opt_init)

        # Max output line edit
        self.opt_max = QLineEdit(self)
        self.opt_max.setMinimumWidth(100)
        self.form_layout.addRow('Max', self.opt_max)

        # Min output line edit
        self.opt_min = QLineEdit(self)
        self.opt_min.setMinimumWidth(100)
        self.form_layout.addRow('Min', self.opt_min)

        self.layout.addLayout(self.form_layout)

        # Write injection button
        self.opt_inject = QPushButton('Write initial', self)
        self.layout.addWidget(self.opt_inject)

        self.setLayout(self.layout)
