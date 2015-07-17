import sys
import time
import threading
import sip
import tooltips as tt
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
        self.chan_views = [ChannelView(chan_no) for chan_no in range(params.n_pid_chan)]

        # Initalize global params view
        self.layout = QHBoxLayout()
        self.gp_view = GlobalParamsView(io_config, params)
        self.layout.addWidget(self.gp_view)

        # Create output tabs
        self.opt_tabs = QTabWidget()

        # Create dac tab layout
        dac_tabs = QTabWidget()
        for x in range(params.n_dac):
            # DAC channels are not mapped linearly to the breakout board output
            # channels. The index x represents the output index, so it must be
            # transformed before selecting a channel view for insertion.
            xmod = io_config.aout_to_dac[x]
            dac_tabs.addTab(self.chan_views[xmod], 'DAC ' + str(x+1))
        self.opt_tabs.addTab(dac_tabs, 'DAC')

        # Create dds tab layout
        for x in range(params.n_dds):
            dds_tabs = QTabWidget()
            fidx = params.freq0_addr + x
            pidx = params.phase0_addr + x
            aidx = params.amp0_addr + x
            dds_tabs.addTab(self.chan_views[fidx], 'FREQ')
            dds_tabs.addTab(self.chan_views[pidx], 'PHASE')
            dds_tabs.addTab(self.chan_views[aidx], 'AMP')
            self.opt_tabs.addTab(dds_tabs, 'DDS ' + str(x + 1))

        self.layout.addWidget(self.opt_tabs)

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
        self.adc_os.setToolTip(tt.adc_os)
        self.form_layout.addRow('ADC oversample', self.adc_os)

        # Sample rate spin box
        self.sample_rate = QDoubleSpinBox(self)
        self.sample_rate.setToolTip(tt.sample_rate)
        self.form_layout.addRow('Sampling rate (Hz)', self.sample_rate)

        self.layout.addLayout(self.form_layout)

        # Block transfer push button
        self.block_transfer = QPushButton('Block logging', self)
        self.block_transfer.setToolTip(tt.block_transfer)
        self.block_transfer.setCheckable(True)
        self.layout.addWidget(self.block_transfer)

        # Lock quick view tool box
        self.chan_labels = []
        for x in range(params.n_pid_chan):
            if x < params.freq0_addr:
                title = 'AOUT' + str(x)
            elif x < params.phase0_addr:
                title = 'FREQ'
            elif x < params.amp0_addr:
                title = 'PHASE'
            else:
                title = 'AMP'
            cl_button = QPushButton(title, self)
            cl_button.setCheckable(True)
            cl_button.setVisible(False)
            cl_button.setStyleSheet('QPushButton { background-color:lightgrey;}')
            cl_button.setMaximumHeight(30)
            self.chan_labels.append(cl_button)

        qv_group = QGroupBox('Quick View')
        self.qv_layout = QVBoxLayout()
        qv_group.setLayout(self.qv_layout)
        self.qv_layout.addStretch(1)

        self.layout.addWidget(qv_group)

        # Save configuration push button
        self.save_config = QPushButton('Save Config', self)
        self.save_config.setToolTip(tt.save_config)
        self.layout.addWidget(self.save_config)

        # Load configuration push button
        self.load_config = QPushButton('Load Config', self)
        self.load_config.setToolTip(tt.load_config)
        self.layout.addWidget(self.load_config)

        # System reset push button
        self.sys_reset = QPushButton('System Reset', self)
        self.sys_reset.setToolTip(tt.sys_reset)
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
        self.config_view = ConfigView()
        self.error_view = ErrorView()
        self.pid_view = PIDView()
        self.proc_view = ProcessingView()
        self.output_view = OutputView()

        cwidth = 200
        self.config_view.setFixedWidth(cwidth)
        self.error_view.setFixedWidth(cwidth)
        self.pid_view.setFixedWidth(cwidth)
        self.proc_view.setFixedWidth(cwidth)
        self.output_view.setFixedWidth(cwidth)

        self.cc_layout = QHBoxLayout()
        self.cc_layout.addWidget(self.config_view)
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
        self.graph_freeze.setToolTip(tt.graph_freeze)
        self.graph_freeze.setCheckable(True)
        self.graph_clear = QPushButton('Clear', self)
        self.graph_clear.setToolTip(tt.graph_clear)

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
Display channel general configuration parameters
'''
class ConfigView(QGroupBox):

    def __init__(self):
        QGroupBox.__init__(self)
        self.setTitle('Channel')
        self.layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Channel source select combo box
        self.chan_src_sel = QComboBox(self)
        self.chan_src_sel.setToolTip(tt.chan_src_sel)
        self.form_layout.addRow('Input', self.chan_src_sel)
        self.layout.addLayout(self.form_layout)

        # Channel name line edit
        self.chan_name = QLineEdit(self)
        self.chan_name.setToolTip(tt.chan_name)
        self.form_layout.addRow('Name', self.chan_name)

        # Add to quick view button
        self.quick_view_toggle = QCheckBox('Show in quick view')
        self.quick_view_toggle.setToolTip(tt.quick_view_toggle)
        self.layout.addWidget(self.quick_view_toggle)

        # PID lock enable button
        self.hb_layout = QHBoxLayout()
        self.pid_lock_en = QPushButton('Enable lock', self)
        self.pid_lock_en.setToolTip(tt.pid_lock_en)
        self.pid_lock_en.setCheckable(True)
        self.hb_layout.addWidget(self.pid_lock_en)

        # Channel reset button
        self.chan_reset = QPushButton('Reset', self)
        self.chan_reset.setToolTip(tt.chan_reset)
        self.hb_layout.addWidget(self.chan_reset)
        self.layout.addLayout(self.hb_layout)

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
        self.ovr_os.setToolTip(tt.ovr_os)
        self.ovr_os.setMaximumWidth(90)
        self.form_layout.addRow('Oversample', self.ovr_os)

        # PID setpoint spin box
        self.pid_setpoint = QDoubleSpinBox(self)
        self.pid_setpoint.setToolTip(tt.pid_setpoint)
        self.pid_setpoint.setMinimumWidth(90)
        self.form_layout.addRow('Setpoint', self.pid_setpoint)

        # Lock threshold spin box
        self.lock_threshold = QDoubleSpinBox(self)
        self.lock_threshold.setToolTip(tt.lock_threshold)
        self.lock_threshold.setMinimumWidth(90)
        self.form_layout.addRow('Threshold', self.lock_threshold)
        self.layout.addLayout(self.form_layout)

        # PID invert error check box
        self.pid_inv_error = QCheckBox('Invert Error', self)
        self.pid_inv_error.setToolTip(tt.pid_inv_error)
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
        self.pid_p_coef.setToolTip(tt.pid_p_coef)
        self.pid_p_coef.setMinimumWidth(90)
        self.form_layout.addRow('P coef', self.pid_p_coef)

        # PID I coefficient spin box
        self.pid_i_coef = QSpinBox(self)
        self.pid_i_coef.setToolTip(tt.pid_i_coef)
        self.pid_i_coef.setMinimumWidth(90)
        self.form_layout.addRow('I coef', self.pid_i_coef)

        # PID D coefficient spin box
        self.pid_d_coef = QSpinBox(self)
        self.pid_d_coef.setToolTip(tt.pid_i_coef)
        self.pid_d_coef.setMinimumWidth(90)
        self.form_layout.addRow('D coef', self.pid_d_coef)

        self.layout.addLayout(self.form_layout)

        # PID clear button
        self.pid_clear = QPushButton('Clear', self)
        self.pid_clear.setToolTip(tt.pid_clear)
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
        self.opt_mult.setToolTip(tt.opt_mult)
        self.opt_mult.setMinimumWidth(90)
        self.form_layout.addRow('Multiplier', self.opt_mult)

        # Right shift spin box
        self.opt_rs = QSpinBox(self)
        self.opt_rs.setToolTip(tt.opt_rs)
        self.opt_rs.setMinimumWidth(90)
        self.form_layout.addRow('Right shift', self.opt_rs)

        # Scale factor display box
        self.scale_factor = QLineEdit(self)
        self.scale_factor.setToolTip(tt.scale_factor)
        self.scale_factor.setEnabled(False)
        self.scale_factor.setStyleSheet('QLineEdit {background-color: lightsteelblue; }')
        self.form_layout.addRow('Scale factor', self.scale_factor)
        self.scale_factor.setEnabled(False)

        # Add channel combo box
        self.opt_add_chan = QComboBox(self)
        self.opt_add_chan.setToolTip(tt.opt_add_chan)
        self.opt_add_chan.setMaximumWidth(90)
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

        # Initial output spin box
        self.opt_init = QDoubleSpinBox(self)
        self.opt_init.setToolTip(tt.opt_init)
        self.opt_init.setMinimumWidth(90)
        self.form_layout.addRow('Initial', self.opt_init)

        # Max output spin box
        self.opt_max = QDoubleSpinBox(self)
        self.opt_max.setToolTip(tt.opt_max)
        self.opt_max.setMinimumWidth(90)
        self.form_layout.addRow('Max', self.opt_max)

        # Min output spin box
        self.opt_min = QDoubleSpinBox(self)
        self.opt_min.setToolTip(tt.opt_min)
        self.opt_min.setMinimumWidth(90)
        self.form_layout.addRow('Min', self.opt_min)

        self.layout.addLayout(self.form_layout)

        # Write injection button
        self.opt_inject = QPushButton('Write initial', self)
        self.opt_inject.setToolTip(tt.opt_inject)
        self.layout.addWidget(self.opt_inject)

        self.setLayout(self.layout)
