adc_os = 'Set ADC internal oversample ratio. Applies to all ADC channels.'

sample_rate = 'Set single-word datalogging sampling rate. Block mode sampling rate is dictated by channel oversampling ratios.'

block_transfer = 'Enable block datalogging mode. Data will be fetched from the FPGA in contiguous blocks instead of one word at a time.'

save_config = 'Save PID controller configuration.'

load_config = 'Load a PID controller configuration file (.pid).'

sys_reset = 'Reset entire system. All channels and all I/O chips will be reset.'

chan_src_sel = 'Select ADC input source for this channel.'

chan_name = 'Set channel name for display in quick view window.'

quick_view_toggle = 'Show channel in quick view window for quick access and monitoring. Enabled channels wil be automatically added to quick view.'

pid_lock_en = 'Enable PID lock with the chosen input channel supplying error data.'

chan_reset = 'Reset channel. Oversample, PID, and output memories will be cleared for this channel.'

ovr_os = 'Set channel oversampling ratio for moving average filter. Unlike the ADC oversample ratio, this only affects this one channel.'

pid_setpoint = 'Set the PID setpoint used to compute error values.'

lock_threshold = 'Set the locking threshold for the quick view lock status monitor. If the difference between the setpoint and mean of the previous 10 error values exceeds the threshold, the channel is considered unlocked.'

pid_inv_error = 'By default, the error signal is computed as \i setpoint - adc_value \i0. This option inverts that computation.'

pid_p_coef = 'Set PID filter P coefficient.'

pid_i_coef = 'Set PID filter I coefficient.'

pid_d_coef = 'Set PID filter D coefficient.'

pid_clear = 'Clear PID filter memory.'

opt_mult = 'Set multiplication factor to be applied to the PID sum.'

opt_rs = 'Set right shift to be applied to PID sum.'

scale_factor = 'Net PID sum scaling factor after multiplication factor and right shift are applied.'

opt_add_chan = 'Specify another channel whose PID sum will be added to this channel\'s sum before output.'

opt_init = 'Set the initial output of the channel. This will be the initial value to which PID sums will be added.'

opt_max = 'Set the maximum output of the channel. Channel output will not exceed this value even if compelled by the PID filter.'

opt_min = 'Set the minimum output of the channel. Channel output will not go below this value even if compelled by the PID filter.'

opt_inject = 'Inject a one-shot write instruction to the pipeline to write the specified initial value.'

graph_freeze = 'Freeze the plot window. New data will continue to be collected and will be displayed when the plot is unfrozen.'

graph_clear = 'Clear the plot window.'
