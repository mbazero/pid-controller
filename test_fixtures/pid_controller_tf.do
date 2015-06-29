# This is a Modelsim simulation script.
# To use:
#  + Start Modelsim
#  + At the command-line, CD to the directory where this file is.
#  + Type: "do thisfilename.do"
# $Rev$ $Date$

set PATH ./oksim

# Source files and testfixture
vlib work
vlog +incdir+$PATH pid_controller_tf.v

vlog adc_controller.v
vlog clk_sync.v
vlog dac_controller.v
vlog dac_instr_queue.v
vlog dds_controller.v
vlog mux_n_chan.v
vlog output_preprocessor.v
vlog oversample_filter.v
vlog pid_controller.v
vlog pid_core.v
vlog pipe_tx_fifo.v
vlog ./ip_core/fifo_19.v
vlog ./ip_core/fifo_16.v

vlog +incdir+$PATH $PATH/glbl.v
vlog +incdir+$PATH $PATH/okHost.v
vlog +incdir+$PATH $PATH/okWireIn.v
vlog +incdir+$PATH $PATH/okWireOut.v
vlog +incdir+$PATH $PATH/okWireOR.v

vsim -L unisims_ver -t ps pid_controller_tf -novopt +acc

run -all

