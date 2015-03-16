import threading
import random
import struct
import binascii

############ ok input address mappings ############
adc_os_owi				= 0x01
adc_cstart_ti			= 0x53

osf_activate_owi		= 0x15
osf_cycle_delay_owi	= 0x02
osf_log_ovr_owi		= 0x03
osf_update_en_owi		= 0x04

pid_clear_ti			= 0x55
pid_lock_en_owi = 0x17
pid_setpoint_owi		= 0x04
pid_p_coef_owi			= 0x05
pid_i_coef_owi			= 0x06
pid_d_coef_owi			= 0x07
pid_update_en_owi		= 0x08

rtr_src_sel_owi		= 0x09
rtr_dest_sel_owi		= 0x0a
rtr_output_active_owi   = 0x16

opp_init_owi			= [0x0b, 0x0c, 0x0d]
opp_min_owi				= [0x0e, 0x0f, 0x10]
opp_max_owi				= [0x11, 0x12, 0x13]
opp_update_en_owi		= 0x14

dac_ref_set_ti			= 0x56

module_update_ti		= 0x57

sys_reset_ti			= 0x58

############ ok output address mappings ############
adc_data_owo			= [32 + count for count in range(6)]
osf_bulk_data_opo		= 0xa3

# trigger id mappings
ADC_CSTART 		= 0
OSF_ACTIVAE 	= 1
PID_CLEAR		= 2
DAC_REF_SET		= 3
MODULE_UPDATE	= 4

'''
Bulk Update Todo
- properly handle x-axis scaling
- add ok address mappings for pipe and focus owi
- test update_error_data_bulk method
- add button to select between continuous or bulk transfer
'''

'''
Todo
- figure out why adc os update isn't working
- proper channel activation wire in value (currently it just activates channel 1)
- implement init_params method
- modify activate channel method to update active_chans struture
- pull all fpga update functionality out into its own class
'''


# PID locking array
class PIDLockArray:
	def __init__(self, okc, num_dac_chan, num_dds_chan):
		# adc params
		self.adc_os = 0
		self.adc_update_rate = 200e3
		self.num_dac_chan = num_dac_chan
		self.num_out_chans = num_dac_chan + num_dds_chan

		# opal kelly controller
		self.okc = okc

		# initialize fp params
		self.init_params()

		# activated event barrier
		# set when channel is activated
		# worker thread is awakened when set
		self.activated = threading.Event();
		self.activated.clear();

		# list of active channels
		self.active_chans = [];

		# dac output array
		self.dac_array = [PIDChannel(self, self.okc, 'DAC', dac_count) for dac_count in range(num_dac_chan)]
		self.dds_array = [PIDChannel(self, self.okc, 'DDS', dds_count) for dds_count in range(num_dds_chan)]

		# focused channel
		self.focused_chan = 0

		# error data polling period in seconds
		self.polling_period = 0.1
		self.polling_period_lock = threading.Lock();

		# global bulk update setting
		'''
		By default the GUI bulk updates the focused channel and continuously updates all other hidden channels.
		If the global bulk update option is toggled off, all channels are updated continuously.
		'''
		self.bulk_update = False

	#################### general #######################
	def handle_poll_period(self, text):
		self.polling_period = float(text)

	# pull data from FPGA and update activate channel error data
	def update_error_data(self):
		# update wire outs
		self.okc.UpdateWireOuts()
		print 'Updated Wire Outs.'

		# loop through active channels and update error data
		for chan in self.active_chans :
			if chan.focused and self.bulk_update :
				chan.bulk_update_error_data()
			else :
				chan.update_error_data()

   # update focused tab based on tab change
	def handle_tab_changed(self, index):
		self.indexToChan(self.focused_chan).set_unfocused() # unfocus old channel
		self.indexToChan(index).set_focused() # focus new channel
		self.focused_chan = index # set focused_chan field

		self.update_focus_on_fpga()

	# set bulk transfer to true
	def handle_bulk_update(self, toggled):
		if toggled == True :
			self.bulk_update = True
			print 'Bulk Update Mode Enabed'
		else :
			self.bulk_update = False
			self.indexToChan(self.focused_chan).clear_error_data()
			print 'Bulk Update Mode Disabled'

	# update focused tab on fpga (set regardless of channel activation status)
	def update_focus_on_fpga(self):
		# construct focus signal
		sig_focus = 1 << self.focused_chan
		# send focus signal to fpga
		'''
		' Focus is unimplemented on hardware as of yet. Uncomment below line when it is implemented.
		'''
		# self.okc.SetAndUpdateWireIn(focused_chan_owi, sig_focus)

	#################### initialization #######################
	def init_params (self):
		# send adc_os default value
		self.handle_adc_os(self.adc_os)

	#################### adc handlers #######################
	def handle_adc_os(self, new_val):
		# store old adc_os value
		adc_os_old = self.adc_os
		self.adc_os = new_val

		# send adc_os to fpga and update modules
		self.okc.SetAndUpdateWireIn(adc_os_owi, self.adc_os)
		self.okc.ModUpdate()

		print 'ADC oversample ratio updated from ' + str(2**adc_os_old) + ' to ' + str(2**self.adc_os)

	def handle_adc_cstart(self, toggled):
		if toggled == True :
			self.okc.ActivateTriggerIn(adc_cstart_ti, 0)
			print 'ADC started'
		else :
			print 'ADC stopped (unimplemented)'

	#################### dac handlers #######################
	def handle_dac_ref_set(self):
		self.okc.ActivateTriggerIn(dac_ref_set_ti, 0)
		print 'DAC reference set'

	def handle_sys_reset(self):
		self.okc.ActivateTriggerIn(sys_reset_ti, 0)
		print 'System reset'

	#################### helpers #######################
	def indexToChan(self, index):
		if index >= self.num_dac_chan :
			return self.dds_array[index - self.num_dac_chan]
		else :
			return self.dac_array[index]

# PID lock
class PIDChannel:
	def __init__(self, pla, okc, channel_type, channel_no):
		# pla
		self.pla = pla

		# opal kelly controller reference
		self.okc = okc

		# channel type and number
		self.channel_type = channel_type
		self.channel_no = channel_no

		# channel name
		self.cname = self.channel_type + ' Channel ' + str(self.channel_no)

		# activation status
		self.activated = False

		# tab focus status
		self.focused = True if self.channel_no == 0 else False

        # router params
		self.rtr_src_sel 		= 8
		self.rtr_dest_sel		= channel_no

		# osf params
		self.osf_log_ovr = 0
		self.osf_cycle_delay = 0

		# pid params
		self.pid_setpoint = 0
		self.pid_p_coef = 1
		self.pid_i_coef = 0
		self.pid_d_coef = 0

		# opp params
		self.opp_lock_en = 0
		self.opp_init = 0
		self.opp_min = 0
		self.opp_max = 0
		self.opp_multiplier = 1

		# error data list
		self.error_data = [0 for count in range(1024)]
		intra_bulk_period = (2**self.osf_log_ovr)*(1/pla.adc_update_rate)
		self.error_data_x = [float(count)*intra_bulk_period for count in range(1024)]
		self.data_lock = threading.Lock()

		# initialize fp params
		self.init_params()

	#################### initialization #######################
	def init_params(self):
		# self.handle_osf_ovr(self.osf_log_ovr)
		# self.handle_osf_cycle_delay('0')
		return

	#################### general handlers #######################
	def set_unfocused(self):
		self.focused = False

	def set_focused(self):
		self.focused = True

	def clear_error_data(self):
		self.error_data = [0 for i in self.error_data]

	#################### update error data #######################
	# get new error data from wire outs (continuous transfer)
	def update_error_data(self):
		if self.rtr_src_sel >= 0 :
			print 'Getting adc channel ' + str(self.rtr_src_sel) + ' data at address: ' + str(adc_data_owo[self.rtr_src_sel])

			# get unsigned adc error data from wireout
			data_raw_us = self.okc.GetWireOutValue(adc_data_owo[self.rtr_src_sel])
			print 'Raw unsigned data: ' + str(data_raw_us)

			# convert data to signed
			data_raw_s = self.uint16_to_int32(data_raw_us)
			print 'Raw signed data: ' + str(data_raw_s)

			# map raw adc data to voltage
			data_volts = self.map_val(data_raw_s, [-2**15, 2**15 - 1], [-5, 5])
			# data_volts = random.randrange(-1, 1)/1000
			self.error_data = self.error_data[1:len(self.error_data)] + [data_volts]

			print 'Voltage data: ' + str(data_volts)

	def bulk_update_error_data(self):
		if self.rtr_src_sel >= 0 :
			block_size = 1024
			buf = bytearray(block_size*2)
			self.okc.xem.ReadFromPipeOut(osf_bulk_data_opo, buf)

			# unpack byte array as array of signed shorts
			fmt_str = '<' + str(block_size) + 'h'
			data_raw = struct.unpack(fmt_str, buf)

			# map raw adc data to voltage
			# TODO: extract map ranges to stored values
			self.error_data = [self.map_val(d, [-2**15, 2**15-1], [-5, 5]) for d in data_raw]

	def uint16_to_int32(self, uint):
		if uint >= 2**15 :
			uint -= 2**16
		return uint

	#################### general handlers #######################
	def handle_chan_activate(self, toggled):
		# TODO: move pla activation functionality to method in PLA class

		if (toggled == True): # channel activated
			self.activated = True # set local activation state
			self.okc.SetAndUpdateWireIn(osf_activate_owi, 1) # TODO: proper update signal creation

			self.pla.active_chans.append(self) # add self to list of active channels
			self.pla.activated.set() # set pla activated event
			print self.cname + ' activated'
		else: # channel deactivated
			self.activated = False # set local activation state
			self.okc.SetAndUpdateWireIn(osf_activate_owi, 0) # TODO: proper update signal

			self.pla.active_chans.remove(self) # remove self from list of active channels
			if not self.pla.active_chans : # clear pla activated event if no activated channels remain
				self.pla.activated.clear()

			print self.cname + ' deactivated'

	#################### osf handlers #######################
	def handle_osf_ovr(self, new_val):
		osf_log_ovr_old = self.osf_log_ovr
		self.osf_log_ovr = new_val

		# send adc_os to fpga and update modules
		self.okc.SetAndUpdateWireIn(adc_os_owi, self.osf_log_ovr)
		self.okc.SetWireInValue(osf_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' oversample ratio updated from ' + str(2**osf_log_ovr_old) + ' to ' + str(2**self.osf_log_ovr)

	def handle_osf_cycle_delay(self, text):
		osf_cycle_delay_old = self.osf_cycle_delay

		try:
			self.osf_cycle_delay = int(text)
		except ValueError:
			return

		if (osf_cycle_delay_old == self.osf_cycle_delay) :
			return

		self.okc.SetAndUpdateWireIn(osf_cycle_delay_owi, self.osf_cycle_delay)
		self.okc.SetWireInValue(osf_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' cycle delay updated from ' + str(osf_cycle_delay_old) + ' to ' + str(self.osf_cycle_delay)

	#################### router handler #######################
	def handle_rtr_src_sel(self, index):
		self.rtr_src_sel = index - 1

		print 'New RTR Source: ' + str(index)

		self.okc.SetWireInValue(rtr_src_sel_owi, self.rtr_src_sel)		# set source channel
		self.okc.SetWireInValue(rtr_dest_sel_owi, self.rtr_dest_sel)	# set destination channel
		self.okc.SetWireInValue(rtr_output_active_owi, 1)	# set destination channel
		self.okc.UpdateWireIns()												# update wire in values
		self.okc.ModUpdate()												# update modules

		print 'Mapped ADC channel ' + str(self.rtr_src_sel) + ' to ' + self.channel_type + ' Channel ' + str(self.rtr_dest_sel)


	#################### pid handlers #######################
	def handle_pid_setpoint(self, text):
		pid_setpoint_old = self.pid_setpoint

		try:
			self.pid_setpoint = int(text)
		except ValueError:
			return

		if (pid_setpoint_old == self.pid_setpoint) :
			return

		pid_setpoint_norm = int(self.map_val(self.pid_setpoint, [-5, 5], [-2**15, 2**15-1]))

		self.okc.SetWireInValue(pid_setpoint_owi, pid_setpoint_norm)
		self.okc.SetWireInValue(pid_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' setpoint updated from ' + str(pid_setpoint_old) + ' to ' + str(self.pid_setpoint)
		print 'Normalized setpoint value: ' + str(pid_setpoint_norm)

	def handle_pid_p_coef(self, text):
		pid_p_coef_old = self.pid_p_coef

		try:
			self.pid_p_coef = int(text)
		except ValueError:
			return

		if (pid_p_coef_old == self.pid_p_coef):
			return

		self.okc.SetWireInValue(pid_p_coef_owi, self.pid_p_coef)
		self.okc.SetWireInValue(pid_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' P coefficient updated from ' + str(pid_p_coef_old) + ' to ' + str(self.pid_p_coef)

	def handle_pid_i_coef(self, text):
		pid_i_coef_old = self.pid_i_coef

		try:
			self.pid_i_coef = int(text)
		except ValueError:
			return

		if (pid_i_coef_old == self.pid_i_coef):
			return

		self.okc.SetWireInValue(pid_i_coef_owi, self.pid_i_coef)
		self.okc.SetWireInValue(pid_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' I coefficient updated from ' + str(pid_i_coef_old) + ' to ' + str(self.pid_i_coef)

	def handle_pid_d_coef(self, text):
		pid_d_coef_old = self.pid_d_coef

		try:
			self.pid_d_coef = int(text)
		except ValueError:
			return

		if (pid_d_coef_old == self.pid_d_coef):
			return

		self.okc.SetWireInValue(pid_d_coef_owi, self.pid_d_coef)
		self.okc.SetWireInValue(pid_update_en_owi, 1 << self.rtr_src_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print self.channel_type + ' Channel ' + str(self.channel_no) + ' D coefficient updated from ' + str(pid_d_coef_old) + ' to ' + str(self.pid_d_coef)

	def handle_pid_clear(self):
		# implement okc clear commands
		print self.cname + ' PID filter cleared'


	#################### opp handlers #######################
	def handle_opp_lock_en(self, state): #TODO change name to handle_pid_lock_en
		if (state == 1) :
			print self.channel_type + ' Channel ' + str(self.channel_no) + ' PID filter activated'
			self.okc.SetAndUpdateWireInValue(pid_lock_en_owi, 1)

			self.okc.Set
		else :
			print self.channel_type + ' Channel ' + str(self.channel_no) + ' PIF filter deactivated'
			self.okc.SetAndUpdateWireInValue(pid_lock_en_owi, 0)

	def handle_opp_init(self, text):
		opp_init_old = self.opp_init

		try:
			self.opp_init = int(text)
		except ValueError:
			return

		if (opp_init_old == self.opp_init):
			return

		opp_init_norm = int(self.map_val(self.opp_init, [0, 5], [0, 2**16-1]))

		self.okc.SetWireInValue(opp_init_owi[0], opp_init_norm)
		self.okc.SetWireInValue(opp_update_en_owi, 1 << self.rtr_dest_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()

		print 'OPP init set to: ' + str(opp_init_norm)

	def handle_opp_min(self, text):
		opp_min_old = self.opp_min

		try:
			self.opp_min = int(text)
		except ValueError:
			return

		if (opp_min_old == self.opp_min):
			return

		opp_min_norm = int(self.map_val(self.opp_min, [0, 5], [0, 2**18-1]))

		self.okc.SetWireInValue(opp_min_owi[0], opp_min_norm)
		self.okc.SetWireInValue(opp_update_en_owi, 1 << self.rtr_dest_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()
		print 'OPP min set to: ' + str(opp_min_norm)

	def handle_opp_max(self, text):
		opp_max_old = self.opp_max

		try:
			self.opp_max = int(text)
		except ValueError:
			return

		if (opp_max_old == self.opp_max):
			return

		opp_max_norm = int(self.map_val(self.opp_max, [0, 5], [0, 2**18-1]))

		self.okc.SetWireInValue(opp_max_owi[0], opp_max_norm)
		self.okc.SetWireInValue(opp_update_en_owi, 1 << self.rtr_dest_sel)
		self.okc.UpdateWireIns()
		self.okc.ModUpdate()
		print 'OPP max set to: ' + str(opp_max_norm)

	#################### helper functions #######################
	def map_val(self, val, vrange, mrange):
		norm_val = val - vrange[0]
		range_ratio = (mrange[1]-mrange[0])/float(vrange[1]-vrange[0])
		return norm_val*range_ratio + mrange[0]