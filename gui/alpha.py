import sys
import time
import threading
import sip
import OpalKellyController
import PIDLockArray
import pyqtgraph as pg
import numpy as np


import random
x = range(10)
# y = [random.random() for j in range(1000)]
y = [0 for j in range(100)]
# x = np.random.normal(size=1000)
# y = np.random.normal(size=1000)

yLock = threading.Lock();
#####################################################

# use the more modern PyQt API (not enabled by default in Python 2.x);
# must precede importing any module that provides the API specified
# sip.setapi('QDate', 2)
# sip.setapi('QDateTime', 2)
# sip.setapi('QString', 2)
# sip.setapi('QTextStream', 2)
# sip.setapi('QTime', 2)
# sip.setapi('QUrl', 2)
# sip.setapi('QVariant', 2)

from PyQt4.Qt import *
# from PyQt4 import QtCore, QtGui

qt_app = QApplication(sys.argv)

'''
TODO
- add params (like W_MLT) to special file for parsing
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

# Main app window
class MainWindow(QWidget):
	def __init__(self, pla, num_dac_chan, num_dds_chan):
		# calculate number of output channels
		num_out_chans = num_dac_chan + num_dds_chan

		# initialize widget
		QWidget.__init__(self)
		self.setWindowTitle('PID Lock Array')

		# horizontal widget to hold global variables and tab widget
		self.layout = QHBoxLayout()

		# create tab layout
		self.tab_widget = QTabWidget()
		self.tab_widget.currentChanged.connect(pla.handle_tab_changed)

		# create lock channel GUIs
		dac_array = pla.dac_array
		dds_array = pla.dds_array
		self.dac_chans = [ChannelControl(dac_array[count]) for count in range(num_dac_chan)]
		self.dds_chans = [ChannelControl(dds_array[count]) for count in range(num_dds_chan)]

		# initialize dac tabs
		for j in range(num_dac_chan) :
			self.tab_widget.addTab(self.dac_chans[j], "DAC " + str(j))

		# initialize dds tabs
		for k in range(num_dds_chan) :
			self.tab_widget.addTab(self.dds_chans[k], "DDS " + str(k))

		# add source params widget to HBox
		gp_widget = GlobalParams(pla, num_dac_chan, num_dds_chan)
		self.layout.addWidget(gp_widget)

		# add channel tabs to widget HBox
		self.layout.addWidget(self.tab_widget)

		# set the HBox layout as the window's primary layout
		self.setLayout(self.layout)

		# initialize worker thread to manage graph updating
		self.workThread = WorkerThread(pla)
		self.chan_focus = self.dac_chans[0] # TODO: implement the proper selection algorithm
		self.connect(self.workThread, SIGNAL("newPlotData()"), self.chan_focus.updateGraph)
		self.workThread.start()

	def closeEvent(self, event):
		# might want to added some explict calls here to kill worker threads
		QWidget.closeEvent(self, event)

	def run(self):
		self.show()
		sys.exit(qt_app.exec_())

class GlobalParams(QGroupBox):
	def __init__(self, pla, num_dac_chan, num_dds_chan):
		# total number of output channels
		self.num_out_chans = num_dac_chan + num_dds_chan

		# initialize widget
		QWidget.__init__(self)
		self.setTitle('Global Parameters')

		# create VBox layout for window
		self.layout = QVBoxLayout()

		# create form layout to store global params
		self.form_layout = QFormLayout()

		#################### adc_os #######################

		# create combo box for ADC oversampling ratios
		self.adc_os_ops 	= [	'2',
								'4',
								'8',
								'16',
								'32',
								'64']
		self.adc_os_wgt = QComboBox(self)
		self.adc_os_wgt.addItems(self.adc_os_ops)
		self.form_layout.addRow('ADC Oversample Ratio:', self.adc_os_wgt)

		# connect adc_os signal to handler
		self.adc_os_wgt.currentIndexChanged.connect(lambda: pla.handle_adc_os(self.adc_os_wgt.currentIndex() + 1))

		#################### num_locks #######################
		#self.locks_dec = range(1, self.num_out_chans)
		#self.locks = [str(dec) for dec in self.locks_dec]
		#self.lock = QComboBox(self)
		#self.lock.addItems(self.locks)
		#self.form_layout.addRow('Set Active Locks: ', self.lock)

		#################### poll period #######################
		self.poll_period_wgt = QLineEdit(self)

		# create and add input validator
		ppv = QDoubleValidator(0.0, 10.0, 1)
		self.poll_period_wgt.setValidator(ppv)

		self.poll_period_wgt.setText(str(pla.polling_period))

		self.form_layout.addRow('Polling period: ', self.poll_period_wgt)

		# add form layout to main layout
		self.layout.addLayout(self.form_layout)

		# connect signal to slot
		self.poll_period_wgt.editingFinished.connect(lambda: pla.handle_poll_period(self.poll_period_wgt.text()))

		#################### block_transfer #######################
		self.block_update = QPushButton('Block Update Mode', self)
		self.block_update.setCheckable(True)
		self.layout.addWidget(self.block_update)

		# connect adc_cstart signal to handler
		self.block_update.clicked.connect(pla.handle_block_update)

		#################### lock status array #######################
		self.dac_status_layout = QVBoxLayout()
		#self.dac_status_layout.setTitle('DAC Status')
		self.dds_status_layout = QVBoxLayout()
		#self.dds_status_layout.setTitle('DDS Status')
		self.dac_status_arr = [QPushButton('DAC Channel ' + str(count), self) for count in range(num_dac_chan)]
		self.dds_status_arr = [QPushButton('DDS Channel ' + str(count), self) for count in range(num_dds_chan)]

		for i in range(num_dac_chan) :
			self.dac_status_arr[i].setStyleSheet('background-color: grey')
			self.dac_status_layout.addWidget(self.dac_status_arr[i])

		for j in range(num_dds_chan) :
			self.dds_status_arr[j].setStyleSheet('background-color: grey')
			self.dds_status_layout.addWidget(self.dds_status_arr[j])

		self.layout.addLayout(self.dac_status_layout)
		self.layout.addLayout(self.dds_status_layout)
		self.layout.addStretch(1)

		#################### adc_cstart #######################
		self.adc_cstart = QPushButton('Start ADC', self)
		self.adc_cstart.setCheckable(True)
		self.layout.addWidget(self.adc_cstart)

		# connect adc_cstart signal to handler
		self.adc_cstart.clicked.connect(pla.handle_adc_cstart)

		#################### dac_ref_set #######################
		self.dac_ref_set = QPushButton('Set DAC Reference', self)
		# self.dac_ref_set.setStyleSheet('background-color: red')
		self.layout.addWidget(self.dac_ref_set)

		# connect dac_ref_set signal to handler
		self.dac_ref_set.clicked.connect(pla.handle_dac_ref_set)

		#################### sys_reset #######################
		self.sys_reset = QPushButton('System Reset', self)
		self.layout.addWidget(self.sys_reset)

		# connect sys_reset signal to handler
		self.sys_reset.clicked.connect(pla.handle_sys_reset)

		# set widget layout as form layout
		self.setLayout(self.layout)



class ChannelControl(QWidget):
	def __init__(self, pid):
		# initialize widget
		QWidget.__init__(self)

		# declare pid
		self.pid = pid

		# initialize channel control components
		inputControl = InputWidget(pid)
		pidControl = PIDWidget(pid)
		outputControl = OutputWidget(pid)

		# initialize plot widget
		self.initGraph()

		# create HBox layout; add channel control components
		self.cc_layout = QHBoxLayout()
		self.cc_layout.addWidget(inputControl)
		self.cc_layout.addWidget(pidControl)
		self.cc_layout.addWidget(outputControl)

		# create VBox layout; add channel control layout and error plot
		self.layout = QVBoxLayout()
		self.layout.addLayout(self.cc_layout)
		self.layout.addWidget(self.plotWidget)

		# set the VBox layout as the tab's primary layout
		self.setLayout(self.layout)

	def initGraph(self):
		self.plotWidget = pg.PlotWidget()
		self.plotItem = self.plotWidget.getPlotItem()
		self.plotItem.setTitle("Error Signal")
		self.plotItem.setLabel('bottom', 'Time', 's')
		self.plotItem.setLabel('left', 'ADC Output', 'V')
		self.plotItem.plot(self.pid.error_data_x, self.pid.error_data)

	def updateGraph(self):
		self.plotItem.clear()
		# self.plotItem.plot(y)
		self.plotItem.plot(self.pid.error_data_x, self.pid.error_data)

# Source params widget
class InputWidget(QGroupBox):

	def __init__(self, pid):
		# initialize widget
		QGroupBox.__init__(self)
		self.setTitle('Input')

		# create widget layout
		self.layout = QVBoxLayout()

		# create form layout
		self.form_layout = QFormLayout()

		#################### rtr_src_sel #######################
		self.rtr_src_sel_ops = ['No Input'] + ['ADC Channel ' + str(count) for count in range(8)]

		# create and fill source channel combo box
		self.rtr_src_sel_wgt = QComboBox(self)
		self.rtr_src_sel_wgt.addItems(self.rtr_src_sel_ops)

		# add source combo box to form with label
		self.form_layout.addRow('Input Channel: ', self.rtr_src_sel_wgt)

		# connect rtr_src_sel signal to handler
		self.rtr_src_sel_wgt.currentIndexChanged.connect(pid.handle_rtr_src_sel)

		#DEBUG set initial for channel 0
		if(pid.channel_no == 0): self.rtr_src_sel_wgt.setCurrentIndex(1)

		#################### osf_ovr #######################
		self.osf_ovr_ops = ['Off',
									'2',
									'4',
									'8',
									'16',
									'32',
									'64']

		# create and fill oversampling combo box
		self.osf_ovr_wgt = QComboBox(self)
		self.osf_ovr_wgt.addItems(self.osf_ovr_ops)

		# add oversampling combo box to form with label
		self.form_layout.addRow('Oversample Ratio: ', self.osf_ovr_wgt)

		# connect osf_ovr signal to handler
		self.osf_ovr_wgt.currentIndexChanged.connect(pid.handle_osf_ovr)

		#DEBUG set initial
		self.osf_ovr_wgt.setCurrentIndex(0);

		#################### osf_cycle_delay #######################
		self.osf_cycle_delay_wgt = QLineEdit(self)

		# create and add input validator
		ocd_validator = QIntValidator(0, 65536)
		self.osf_cycle_delay_wgt.setValidator(ocd_validator)


		self.osf_cycle_delay_wgt.setPlaceholderText('0 to 65536')
		self.form_layout.addRow('Cycle Delay:', self.osf_cycle_delay_wgt)

		# add form layout to VBox layout
		self.layout.addLayout(self.form_layout)

		# connect signal to slot
		self.osf_cycle_delay_wgt.textChanged.connect(lambda: pid.handle_osf_cycle_delay(self.osf_cycle_delay_wgt.text()))

		#DEBUG set initial
		self.osf_cycle_delay_wgt.setText('0');

		#################### osf_activate_out #######################
		self.osf_activate_wgt = QPushButton('Activate Channel', self)
		self.osf_activate_wgt.setCheckable(True)
		self.layout.addWidget(self.osf_activate_wgt)

		# connect signal to slot
		self.osf_activate_wgt.toggled.connect(pid.handle_chan_activate)

		# set vbox as widget's main layout
		self.setLayout(self.layout)


class PIDWidget(QGroupBox):

	def __init__(self, pid):
		# initialize widget
		QGroupBox.__init__(self)
		self.setTitle('PID Filter')

		# create widget layout
		self.layout = QVBoxLayout()

		# create form layout
		self.form_layout = QFormLayout()

		#################### pid_setpoint #######################
		self.pid_setpoint_wgt = QLineEdit(self)

		# create and add apply input validator
		ps_validator = QIntValidator(-5, 5)
		self.pid_setpoint_wgt.setValidator(ps_validator)

		self.pid_setpoint_wgt.setPlaceholderText('-5 to 5V')
		self.form_layout.addRow('Setpoint:', self.pid_setpoint_wgt)

		# handler
		self.pid_setpoint_wgt.textChanged.connect(lambda: pid.handle_pid_setpoint(self.pid_setpoint_wgt.text()))

		#DEBUG set initial
		self.pid_setpoint_wgt.setText('0');

		#################### pid_p_coef #######################
		self.pid_p_coef_wgt = QLineEdit(self)

		# create and apply input validator
		pcoef_validator = QIntValidator(0, 32768)
		self.pid_p_coef_wgt.setValidator(pcoef_validator)

		self.pid_p_coef_wgt.setPlaceholderText('0 to 32768')
		self.form_layout.addRow('P Coef:', self.pid_p_coef_wgt)

		# handler
		self.pid_p_coef_wgt.textChanged.connect(lambda: pid.handle_pid_p_coef(self.pid_p_coef_wgt.text()))

		#DEBUG set initial
		self.pid_p_coef_wgt.setText('10');

		#################### pid_i_coef #######################
		self.pid_i_coef_wgt = QLineEdit(self)

		# apply input validator
		self.pid_i_coef_wgt.setValidator(pcoef_validator)

		self.pid_i_coef_wgt.setPlaceholderText('0 to 32768')
		self.form_layout.addRow('I Coef:', self.pid_i_coef_wgt)

		# handler
		self.pid_i_coef_wgt.textChanged.connect(lambda: pid.handle_pid_i_coef(self.pid_i_coef_wgt.text()))

		#DEBUG set initial
		self.pid_i_coef_wgt.setText('3');

		#################### pid_d_coef #######################
		self.pid_d_coef_wgt = QLineEdit(self)

		# apply input validator
		self.pid_d_coef_wgt.setValidator(pcoef_validator)

		self.pid_d_coef_wgt.setPlaceholderText('0 to 32768')
		self.form_layout.addRow('D Coef:', self.pid_d_coef_wgt)

		# handle
		self.pid_d_coef_wgt.textChanged.connect(lambda: pid.handle_pid_d_coef(self.pid_d_coef_wgt.text()))

		# add form layout to main layout
		self.layout.addLayout(self.form_layout)

		# create HBox to hold enable and clear controls
		self.eb_box = QHBoxLayout()

		#DEBUG set initial
		self.pid_d_coef_wgt.setText('0');

		#################### opp_lock_en #######################
		self.opp_lock_en_wgt = QCheckBox('Enable PID Lock', self)
		self.eb_box.addWidget(self.opp_lock_en_wgt)

		# handler
		self.opp_lock_en_wgt.stateChanged.connect(pid.handle_opp_lock_en)

		# add stretch to HBox
		self.eb_box.addStretch(1)

		#################### pid_clear #######################
		self.pid_clear_wgt = QPushButton('Clear', self)
		self.eb_box.addWidget(self.pid_clear_wgt)

		# handler
		self.pid_clear_wgt.clicked.connect(pid.handle_pid_clear)

		# add HBox to bottom of the main VBox layout
		self.layout.addLayout(self.eb_box)

		# set the VBox as the main layout
		self.setLayout(self.layout)


class OutputWidget(QGroupBox):

	def __init__(self, pid):
		# initialize widget
		QGroupBox.__init__(self)
		self.setTitle('Output')

		# create widget layout
		self.layout = QVBoxLayout()

		# create form layout
		self.form_layout = QFormLayout()

		# #################### rtr_dest_sel #######################
		# # create destination channel combo box and add to form layout
		# self.rtr_dest_sel_ops = ['No Output',
		# 							'DAC Channel 1',
		# 							'DAC Channel 2',
		# 							'DAC Channel 3',
		# 							'DAC Channel 4',
		# 							'DAC Channel 5',
		# 							'DAC Channel 6',
		# 							'DAC Channel 7',
		# 							'DAC Channel 8',
		# 							'DDS Frequency 1',
		# 							'DDS Phase 1',
		# 							'DDS Amplitude 1']
		# self.rtr_dest_sel_wgt = QComboBox(self)
		# self.rtr_dest_sel_wgt.addItems(self.rtr_dest_sel_ops)
		# self.form_layout.addRow('Output Channel:', self.rtr_dest_sel_wgt)

		#################### opp_init #######################
		self.opp_init_wgt = QLineEdit(self)

		# create and apply input validator
		init_validator = QDoubleValidator(0.0, 5.0, 2)
		self.opp_init_wgt.setValidator(init_validator)

		self.opp_init_wgt.setPlaceholderText('0 to 5V')
		self.form_layout.addRow('Initial Output:', self.opp_init_wgt)

		# handler
		self.opp_init_wgt.textChanged.connect(lambda: pid.handle_opp_init(self.opp_init_wgt.text()))

		#DEBUG set initial
		self.opp_init_wgt.setText('3.0');

		#################### opp_max #######################
		self.opp_max_wgt = QLineEdit(self)

		# create and apply input validator
		max_validator = QDoubleValidator(0.0, 5.0, 2)
		self.opp_max_wgt.setValidator(max_validator)

		self.opp_max_wgt.setPlaceholderText('0 to 5V')
		self.form_layout.addRow('Max Output:', self.opp_max_wgt)

		# handler
		self.opp_max_wgt.textChanged.connect(lambda: pid.handle_opp_max(self.opp_max_wgt.text()))

		#DEBUG set initial
		self.opp_max_wgt.setText('4.0');

		#################### opp_min #######################
		self.opp_min_wgt = QLineEdit(self)

		# create and apply input validator
		min_validator = QDoubleValidator(0.0, 5.0, 2)
		self.opp_min_wgt.setValidator(min_validator)

		self.opp_min_wgt.setPlaceholderText('0 to 5V')
		self.form_layout.addRow('Min Output:', self.opp_min_wgt)

		# handler
		self.opp_min_wgt.textChanged.connect(lambda: pid.handle_opp_min(self.opp_min_wgt.text()))

		#DEBUG set initial
		self.opp_min_wgt.setText('1.0');

		#################### opp_multiplier #######################
		self.opp_mult_wgt = QLineEdit(self)

		# create and apply input validator
		mult_validator = QIntValidator(-2**9, 2**9 - 1)
		self.opp_mult_wgt.setValidator(mult_validator)

		self.opp_mult_wgt.setPlaceholderText(str(-2**9) + ' to ' + str(2**9 - 1))
		self.form_layout.addRow('Multiplier:', self.opp_mult_wgt)

		# handler
		self.opp_mult_wgt.textChanged.connect(lambda: pid.handle_opp_mult(self.opp_mult_wgt.text()))

		#################### opp_right_shift #######################
		self.opp_right_shift_wgt = QLineEdit(self)

		# create and apply input validator
		right_shift_validator = QIntValidator(0, 2**16 - 1)
		self.opp_right_shift_wgt.setValidator(right_shift_validator)

		self.opp_right_shift_wgt.setPlaceholderText(str(0) + ' to ' + str(2**16 - 1))
		self.form_layout.addRow('Right shift:', self.opp_right_shift_wgt)

		# handler
		self.opp_right_shift_wgt.textChanged.connect(lambda: pid.handle_opp_right_shift(self.opp_right_shift_wgt.text()))

		# add form layout to main layout
		self.layout.addLayout(self.form_layout)

		# set the VBox as the main layout
		self.setLayout(self.layout)


# worker thread to pull data from FPGA and update plots without stalling the GUI
class WorkerThread(QThread):
	def __init__(self, pla):
		QThread.__init__(self)
		self.pla = pla
		self.exiting = False

	def run(self):
		while not self.exiting:

			# wait for pid lock array to become activate
			while not self.pla.activated.isSet():
				self.pla.activated.wait()

			self.pla.update_error_data() # read data from fpga and update in model
			self.emit(SIGNAL('newPlotData()')) # signal gui that new data is available to be plotted
			time.sleep(self.pla.polling_period) # sleep for a period of time

	def shutDown(self):
		self.exiting = True
		self.wait()

# create instance of application window and run it
def main():
	# output channel specifiers
	num_dac_chans = 6
	num_dds_chans = 2

	# debug flag
	debug = 1

	# instantiate opal kelly controller
	okc = OpalKellyController.OpalKellyController(debug)
	if (False == okc.InitializeDevice()):
		exit

	# instantiate PID lock array
	pla = PIDLockArray.PIDLockArray(okc, num_dac_chans, num_dds_chans)

	# instantiate GUI and run it
	gui = MainWindow(pla, num_dac_chans, num_dds_chans)
	gui.run()

if __name__ == '__main__':
	main()
