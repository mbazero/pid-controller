`timescale 1ns / 1ps

// two_dds_test -- mba 2014

// TODO
// - refactor fuck out of frontpanel_controller (seriously, this is all fucked now)
// - test reset
// - add ability to update phase, freq, and amp simulatenously
// - consider adding full speed 50MHz DAC/DDS serial clock
// - consistency with frontpanel param updating (some modules update on posedge of update signal, some on posedge of clock)
// - consistency with frontpanel param reseting
// - should probably change everything to asynch reset b/c sys reset signal is a trigger (only one clock cycle)
// - make sure output preprocessor multiplier is delivered in signed fashion
// - modify adc reset so it is synchronous with the 17MHz clock
// - check modules instantiation in dds data path in this TLE (dds OPPs and dds controllers)
// - parameterize dds controller
// - add phase and amplitude funcitonality to dds controller

module pid_controller (
	// inputs <- OPAL KELLY PLL
	input wire							clk50_in,				// 50MHz system clock
	input wire							clk17_in,				// 17MHz adc serial clock

	// inputs <- ADC - AD7608
	input wire							adc_busy_in,
	input wire							adc_data_a_in,
	input wire							adc_data_b_in,

	// outputs -> ADC - AD7608
	output wire		[W_ADC_OS-1:0]	adc_os_out,
	output wire							adc_convst_out,
	output wire							adc_reset_out,
	output wire							adc_sclk_out,
	output wire							adc_n_cs_out,

	// outputs -> DAC - DAC8568
	output wire							dac_nldac_out,
	output wire							dac_nsync_out,
	output wire							dac_sclk_out,
	output wire							dac_din_out,
	output wire							dac_nclr_out,

	// outputs -> DDS - AD9912
	output wire		[N_DDS-1:0]		dds_sclk_out,
	output wire		[N_DDS-1:0]		dds_reset_out,
	output wire		[N_DDS-1:0]		dds_csb_out,
	output wire		[N_DDS-1:0]		dds_sdio_out,
	output wire		[N_DDS-1:0]		dds_io_update_out,

	// outputs -> breakout board
	output wire							obuf_en_out,	// breakout board output buffer enable (active low)

	// inouts <-> frontpanel host interface
	input wire		[7:0]				hi_in,
	output wire		[1:0]				hi_out,
	inout wire		[15:0]			hi_inout,
	inout wire							hi_aa,

	output wire							i2c_sda,
	output wire							i2c_scl,
	output wire							hi_muxsel
	);

//////////////////////////////////////////
// includes
//////////////////////////////////////////

`include "parameters.vh"
`include "ep_map.vh"

//////////////////////////////////////////
// derived parameters
//////////////////////////////////////////

localparam N_OUT 					= N_DAC + 3*N_DDS;	// total number of output channels; each dds has three output channels (phase, freq, and amp)
localparam W_RTR_DATA 			= W_COMP + 2;			// width of router data lines

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* all modules */
wire								sys_reset;

/* adc controller */
wire								adc_cstart;
reg	[W_ADC_OS-1:0]			adc_os;
wire	[N_ADC-1:0]				adc_data_valid;
wire	[W_ADC_DATA-1:0]		adc_data_a;
wire	[W_ADC_DATA-1:0]		adc_data_b;

/* clock synchronizer */
wire	[N_ADC-1:0]				cs_data_valid;
wire	[W_ADC_DATA-1:0]		cs_data_a;
wire	[W_ADC_DATA-1:0]		cs_data_b;

/* oversample filter */
reg	[N_ADC-1:0]				osf_activate;
reg	[W_OSF_CD-1:0]			osf_cycle_delay[0:N_ADC-1];
reg	[W_OSF_OSM-1:0]		osf_osm[0:N_ADC-1];
wire	[W_ADC_DATA-1:0]		osf_data[0:N_ADC-1];
wire	[N_ADC-1:0]				osf_data_valid;
reg	[W_RTR_SEL-1:0]		osf_pipe_chan;

/* pid core */
reg	[N_ADC-1:0]				pid_lock_en;
reg	[W_EP-1:0]				pid_setpoint[0:N_ADC-1];
reg	[W_EP-1:0]				pid_p_coef[0:N_ADC-1];
reg	[W_EP-1:0]				pid_i_coef[0:N_ADC-1];
reg	[W_EP-1:0]				pid_d_coef[0:N_ADC-1];
wire	[W_COMP-1:0]			pid_data[0:N_ADC-1];
wire	[N_ADC-1:0]				pid_data_valid;
wire	[W_RTR_DATA*N_ADC-1:0]	pid_data_packed;

/* router */
reg	[W_RTR_SEL-1:0]		rtr_src_sel[0:N_OUT-1];
wire	[W_COMP-1:0]			rtr_data[0:N_OUT-1];
wire	[N_OUT-1:0]				rtr_data_valid;
wire	[N_OUT-1:0]				rtr_lock_en;

/* dac opp */
reg	[W_DAC_DATA:0]			opp_dac_max[0:N_DAC-1];
reg	[W_DAC_DATA:0]			opp_dac_min[0:N_DAC-1];
reg	[W_DAC_DATA:0]			opp_dac_init[0:N_DAC-1];
reg	[W_OPP_MLT-1:0]		opp_dac_mult[0:N_DAC-1];
reg	[W_EP-1:0]				opp_dac_rs[0:N_DAC-1];
wire	[W_DAC_DATA-1:0]		opp_dac_data[0:N_DAC-1];
wire	[N_DAC-1:0]				opp_dac_data_sign;
wire	[N_DAC-1:0]				opp_dac_data_valid;
wire	[W_EP-1:0]				opp_dac_inject;

/* dds frequency opp */
reg	[W_DDS_FREQ-1:0]		opp_freq_max[0:N_DDS-1];
reg	[W_DDS_FREQ-1:0]		opp_freq_min[0:N_DDS-1];
reg	[W_DDS_FREQ-1:0]		opp_freq_init[0:N_DDS-1];
reg	[W_OPP_MLT-1:0]		opp_freq_mult[0:N_DDS-1];
reg	[W_EP-1:0]				opp_freq_rs[0:N_DDS-1];
wire	[W_DDS_FREQ-1:0]		opp_freq_data[0:N_DDS-1];
wire	[N_DDS-1:0]				opp_freq_data_sign;
wire	[N_DDS-1:0]				opp_freq_data_valid;
wire	[W_EP-1:0]				opp_freq_inject;

/* dds phase opp */
reg	[W_DDS_PHASE-1:0]		opp_phase_max[0:N_DDS-1];
reg	[W_DDS_PHASE-1:0]		opp_phase_min[0:N_DDS-1];
reg	[W_DDS_PHASE-1:0]		opp_phase_init[0:N_DDS-1];
reg	[W_OPP_MLT-1:0]		opp_phase_mult[0:N_DDS-1];
reg	[W_EP-1:0]				opp_phase_rs[0:N_DDS-1];
wire	[W_DDS_PHASE-1:0]		opp_phase_data[0:N_DDS-1];
wire	[N_DDS-1:0]				opp_phase_data_sign;
wire	[N_DDS-1:0]				opp_phase_data_valid;
wire	[W_EP-1:0]				opp_phase_inject;

/* dds amplitude opp */
reg	[W_DDS_AMP-1:0]		opp_amp_max[0:N_DDS-1];
reg	[W_DDS_AMP-1:0]		opp_amp_min[0:N_DDS-1];
reg	[W_DDS_AMP-1:0]		opp_amp_init[0:N_DDS-1];
reg	[W_OPP_MLT-1:0]		opp_amp_mult[0:N_DDS-1];
reg	[W_EP-1:0]				opp_amp_rs[0:N_DDS-1];
wire	[W_DDS_AMP-1:0]		opp_amp_data[0:N_DDS-1];
wire	[N_DDS-1:0]				opp_amp_data_sign;
wire	[N_DDS-1:0]				opp_amp_data_valid;
wire	[W_EP-1:0]				opp_amp_inject;

/* dac instruction queue */
wire	[W_DAC_DATA*N_DAC-1:0]	diq_input_packed;
wire	[W_DAC_DATA-1:0]			diq_data;
wire	[W_DAC_CHS-1:0]			diq_chan;
wire									diq_data_valid;

/* dac controller */
wire								dac_ref_set;
wire								dac_done;

/* dds controller */
wire	[N_DDS-1:0]				dds_done;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* output buffer enable */
assign obuf_en_out = 1'b0;

/* pack pid data, data valid, and lock enable signals for router */
genvar i;
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : pid_data_pack
		assign pid_data_packed[ i*W_RTR_DATA +: W_RTR_DATA ] = {pid_lock_en[i], pid_data_valid[i], pid_data[i]};
	end
endgenerate

/* pack dac output channels for dac instruction queue */
generate
	for ( i = 0; i < N_DAC; i = i + 1 ) begin : diq_in_arr
		assign diq_input_packed[ i*W_DAC_DATA +: W_DAC_DATA ] = opp_dac_data[i];
	end
endgenerate

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* initial routing (disable all routes) */
generate
	for ( i = 0; i < N_OUT; i = i+1 ) begin : src_select_init
		initial rtr_src_sel[i] = NULL_CHAN;
	end
endgenerate

//////////////////////////////////////////
// functions
//////////////////////////////////////////

/*
 * Functions to map DAC and DDS relative channel descriptors to absolute
 * descriptors according to the following mapping.
 * ----------------------------------------------------------------------------
 *	[ 0 						: N_DAC - 1					] - DAC Channels
 *	[ N_DAC					: N_DAC + N_DDS - 1		] - DDS Frequency Channels
 *	[ N_DAC + N_DDS		: N_DAC + 2*N_DDS - 1	] - DDS Phase Channels
 *	[ N_DAC + 2*N_DDS		: N_DAC + 3*N_DDS - 1	] - DDS Amplitude Channels
 * ----------------------------------------------------------------------------
*/
function [W_RTR_SEL-1:0] map_dac;
	input [W_EP-1:0] rel_index;
	begin
		map_dac = rel_index;
	end
endfunction

function [W_RTR_SEL-1:0] map_freq;
	input [W_EP-1:0] rel_index;
	begin
		map_freq = N_DAC + rel_index;
	end
endfunction

function [W_RTR_SEL-1:0] map_phase;
	input [W_EP-1:0] rel_index;
	begin
		map_phase = N_DAC + N_DDS + rel_index;
	end
endfunction

function [W_RTR_SEL-1:0] map_amp;
	input [W_EP-1:0] rel_index;
	begin
		map_amp = N_DAC + 2*N_DDS + rel_index;
	end
endfunction

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc controller */
adc_controller #(
	.W_OUT				(W_ADC_DATA),
	.N_CHAN				(N_ADC),
	.W_OS					(W_ADC_OS))
adc_cont (
	.clk_in				(clk17_in),
	.reset_in			(sys_reset),
	.busy_in				(adc_busy_in),
	.data_a_in			(adc_data_a_in),
	.data_b_in			(adc_data_b_in),
	.os_in				(adc_os),
	.cstart_in			(adc_cstart),
	.os_out				(adc_os_out),
	.convst_out			(adc_convst_out),
	.reset_out			(adc_reset_out),
	.sclk_out			(adc_sclk_out),
	.n_cs_out			(adc_n_cs_out),
	.data_valid_out	(adc_data_valid),
	.data_a_out			(adc_data_a),
	.data_b_out			(adc_data_b)
	);

/* clock synchronizer */
clk_sync #(
	.W_DATA				(W_ADC_DATA),
	.N_ADC				(N_ADC))
cs (
	.sys_clk_in			(clk50_in),
	.reset_in			(sys_reset),
	.data_valid_in		(adc_data_valid),
	.data_a_in			(adc_data_a),
	.data_b_in			(adc_data_b),
	.data_valid_out	(cs_data_valid),
	.data_a_out			(cs_data_a),
	.data_b_out			(cs_data_b)
	);

/* oversample filter array */
generate
	for ( i = 0; i < N_ADC/2; i = i + 1 ) begin : osf_array
		/* index parameters */
		localparam A = i;
		localparam B = i + N_ADC/2;

		/* osf bank a: draws from adc channel a */
		oversample_filter #(
			.W_DATA				(W_ADC_DATA),
			.W_EP					(W_EP),
			.W_OSM				(W_OSF_OSM))
		ovr_inst_a (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_a),
			.data_valid_in		(cs_data_valid[A]),
			.cycle_delay_in	(osf_cycle_delay[A]),
			.osm_in				(osf_osm[A]),
			.activate_in		(osf_activate[A]),
			.data_out			(osf_data[A]),
			.data_valid_out	(osf_data_valid[A])
			);

		/* osf bank b: draws from adc channel b */
		oversample_filter #(
			.W_DATA				(W_ADC_DATA),
			.W_EP					(W_EP),
			.W_OSM				(W_OSF_OSM))
		ovr_inst_b (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_b),
			.data_valid_in		(cs_data_valid[B]),
			.cycle_delay_in	(osf_cycle_delay[B]),
			.osm_in				(osf_osm[B]),
			.activate_in		(osf_activate[B]),
			.data_out			(osf_data[B]),
			.data_valid_out	(osf_data_valid[B])
			);
	end
endgenerate

/* pid array */
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : pid_array
		pid_core #(
			.W_IN					(W_ADC_DATA),
			.W_OUT				(W_COMP),
			.W_EP					(W_EP),
			.COMP_LATENCY		(PID_COMP_LATENCY))
		pid_inst (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(osf_data[i]),
			.data_valid_in		(osf_data_valid[i]),
			.setpoint_in		(pid_setpoint[i]),
			.p_coef_in			(pid_p_coef[i]),
			.i_coef_in			(pid_i_coef[i]),
			.d_coef_in			(pid_d_coef[i]),
			.lock_en_in			(pid_lock_en[i]),
			.data_out			(pid_data[i]),
			.data_valid_out	(pid_data_valid[i])
			);
	end
endgenerate

/* routing */
generate
	for ( i = 0; i < N_OUT; i = i+1 ) begin : mux_array
		mux_n_chan #(
			.W_CHAN				(W_RTR_DATA),
			.W_SEL				(W_RTR_SEL),
			.N_IN					(N_ADC))
		mux_inst (
			.data_packed_in	(pid_data_packed),
			.chan_select_in	(rtr_src_sel[i]),
			.enable_in			(~rtr_src_sel[i][W_RTR_SEL-1]), // negative routes disable output
			.data_out			({rtr_lock_en[i], rtr_data_valid[i], rtr_data[i]})
			);
	end
endgenerate

/* dac preprocessor array */
generate
	for ( i = 0; i < N_DAC; i = i + 1 ) begin : dac_opp_array
		localparam D = map_dac(i);

		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_DAC_DATA + 1), // opp output is signed, so dac opp output width must be 1 greater than dac data width
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		dac_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(rtr_data[D]),
			.data_valid_in		(rtr_data_valid[D] | opp_dac_inject[i]),
			.lock_en_in			(rtr_lock_en[D]),
			.output_max_in		(opp_dac_max[i]),
			.output_min_in		(opp_dac_min[i]),
			.output_init_in	(opp_dac_init[i]),
			.multiplier_in		(opp_dac_mult[i]),
			.right_shift_in	(opp_dac_rs[i]),
			.data_out			({opp_dac_data_sign[i], opp_dac_data[i]}),
			.data_valid_out	(opp_dac_data_valid[i])
			);
	end
endgenerate

/* dac instruction queue */
dac_instr_queue #(
	.W_DATA				(W_DAC_DATA),
	.W_CHS				(W_DAC_CHS),
	.N_CHAN				(N_DAC))
dac_iq (
	.clk_in				(clk50_in),
	.reset_in			(sys_reset),
	.data_packed_in	(diq_input_packed),
	.data_valid_in		(opp_dac_data_valid),
	.rd_ack_in			(dac_done),
	.data_out			(diq_data),
	.chan_out			(diq_chan),
	.data_valid_out	(diq_data_valid)
	);

/* dac controller */
dac_controller #(
	.W_DATA				(W_DAC_DATA),
	.W_CHS				(W_DAC_CHS),
	.N_CHAN				(N_DAC))
dac_cntrl (
	.clk_in				(clk50_in),
	.reset_in			(sys_reset),
	.ref_set_in			(dac_ref_set),
	.data_in				(diq_data),
	.channel_in			(diq_chan),
	.data_valid_in		(diq_data_valid),
	.nldac_out			(dac_nldac_out),
	.nsync_out			(dac_nsync_out),
	.sclk_out			(dac_sclk_out),
	.din_out				(dac_din_out),
	.nclr_out			(dac_nclr_out),
	.dac_done_out		(dac_done),
	.data_out			(),
	.channel_out		()
	);

/* dds preprocessor array */
generate
	for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_opp_array
		/* index parameters */
		localparam F = map_freq(i);	// frequency channels index
		localparam P = map_phase(i);	// phase channels index
		localparam A = map_amp(i);		// amplitude channels index

		/* frequency output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_DDS_FREQ),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		freq_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(rtr_data[F]),
			.data_valid_in		(rtr_data_valid[F] | opp_freq_inject[i]),
			.lock_en_in			(rtr_lock_en[F]),
			.output_max_in		(opp_freq_max[i]),
			.output_min_in		(opp_freq_min[i]),
			.output_init_in	(opp_freq_init[i]),
			.multiplier_in		(opp_freq_mult[i]),
			.right_shift_in	(opp_freq_rs[i]),
			.data_out			(opp_freq_data[i]),
			.data_valid_out	(opp_freq_data_valid[i])
			);

		/* phase output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_DDS_PHASE),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		phase_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(rtr_data[P]),
			.data_valid_in		(rtr_data_valid[P] | opp_phase_inject[i]),
			.lock_en_in			(rtr_lock_en[P]),
			.output_max_in		(opp_phase_max[i]),
			.output_min_in		(opp_phase_min[i]),
			.output_init_in	(opp_phase_init[i]),
			.multiplier_in		(opp_phase_mult[i]),
			.right_shift_in	(opp_phase_rs[i]),
			.data_out			(opp_phase_data[i]),
			.data_valid_out	(opp_phase_data_valid[i])
			);

		/* amplitude output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_DDS_AMP),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		amp_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(rtr_data[A]),
			.data_valid_in		(rtr_data_valid[A] | opp_amp_inject[i]),
			.lock_en_in			(rtr_lock_en[A]),
			.output_max_in		(opp_amp_max[i]),
			.output_min_in		(opp_amp_min[i]),
			.output_init_in	(opp_amp_init[i]),
			.multiplier_in		(opp_amp_mult[i]),
			.right_shift_in	(opp_amp_rs[i]),
			.data_out			(opp_amp_data[i]),
			.data_valid_out	(opp_amp_data_valid[i])
			);
	end
endgenerate

/* dds controller array */
generate
	for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_array
		dds_controller dds_cntrl (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.freq_in				(opp_freq_data[i]),
			.phase_in			(opp_phase_data[i]),
			.amp_in				(opp_amp_data[i]),
			.freq_dv_in			(opp_freq_data_valid[i]),
			.phase_dv_in		(opp_phase_data_valid[i]),
			.amp_dv_in			(opp_amp_data_valid[i]),
			.sclk_out			(dds_sclk_out[i]),
			.reset_out			(dds_reset_out[i]),
			.csb_out				(dds_csb_out[i]),
			.sdio_out			(dds_sdio_out[i]),
			.io_update_out		(dds_io_update_out[i]),
			.dds_done_out		(dds_done[i])
			);
	end
endgenerate


/* frontpanel interface */
// -----------------------------------------------------------
// NOTE: Verilog does not allow 2D input ports, so this module
// is included inline rather than in its own file to eliminate
// substantial packing and unpacking complexity.
//
// This module handles all frontpanel input and output. Input
// data is received on three wire-in channels. The data target
// is defined by an address and channel. The address specifies
// the type of data being received. The channel specifies
// the target PID input or output channel.
//
//	Output data is sent in two modes. Single-word modes sends
// single osf data words across wire-outs (one for each
// input channel). Block mode sends blocks of 1024 osf data
// words at a time. Block mode is only active for one
// channel at a time. The active channel is specified by
// the focused_chan register.
// -----------------------------------------------------------

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

wire	[W_EP*3-1:0]			data;
wire	[W_EP-1:0]				chan;
wire	[W_EP-1:0]				addr;
reg	[W_ADC_DATA-1:0]		osf_data_reg[0:N_ADC-1];
wire	[15:0]					osf_pipe_dout;
wire osf_pipe_read;

wire	[W_EP-1:0]				data2, data1, data0;
wire reg_update;
wire 	[W_EP-1:0]				sys_gp_trig;

/* host interface */
wire 								ticlk;
wire	[30:0]					ok1;
wire	[16:0] 					ok2;
wire	[17*(N_ADC+1)-1:0]	ok2x;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* data channel */
assign data				= {data2, data1, data0};

/* frontpanel control */
assign i2c_sda			= 1'bz;
assign i2c_scl			= 1'bz;
assign hi_muxsel		= 1'b0;

/* system general purpose trigger mappings */
assign sys_reset		= sys_gp_trig[sys_reset_offset];
assign adc_cstart		= sys_gp_trig[adc_cstart_offset];
assign reg_update		= sys_gp_trig[reg_update_offset];
assign dac_ref_set	= sys_gp_trig[dac_ref_set_offset];

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* input data multiplexing */
always @( posedge reg_update ) begin
	case ( addr )
		/* adc mappings */
		adc_os_addr				:	adc_os								<= data[W_ADC_OS-1:0];
		/* osf mappings */
		osf_activate_addr		:	osf_activate[chan]				<= data[0];
		osf_cycle_delay_addr	:	osf_cycle_delay[chan]			<= data[W_OSF_CD-1:0];
		osf_osm_addr			:	osf_osm[chan]						<= data[W_OSF_OSM-1:0];
		osf_pipe_chan_addr	:	osf_pipe_chan						<= data[W_RTR_SEL-1:0];
		/* pid mappings */
		pid_lock_en_addr		:	pid_lock_en[chan]					<= data[0];
		pid_setpoint_addr		:	pid_setpoint[chan]				<= data[W_EP-1:0];
		pid_p_coef_addr		:	pid_p_coef[chan]					<= data[W_EP-1:0];
		pid_i_coef_addr		:	pid_i_coef[chan]					<= data[W_EP-1:0];
		pid_d_coef_addr		:	pid_d_coef[chan]					<= data[W_EP-1:0];
		/* router mappings */
		rtr_src_sel_addr		:	rtr_src_sel[chan]					<= data[W_RTR_SEL-1:0];
		rtr_dac_src_addr		:	rtr_src_sel[map_dac(chan)]		<= data[W_RTR_SEL-1:0];
		rtr_freq_src_addr		:	rtr_src_sel[map_freq(chan)]	<= data[W_RTR_SEL-1:0];
		rtr_phase_src_addr	:	rtr_src_sel[map_phase(chan)]	<= data[W_RTR_SEL-1:0];
		rtr_freq_src_addr		:	rtr_src_sel[map_amp(chan)]		<= data[W_RTR_SEL-1:0];
		/* dac opp mappings */
		opp_dac_min_addr		:	opp_dac_min[chan]					<= data[W_DAC_DATA:0];
		opp_dac_max_addr		:	opp_dac_max[chan]					<= data[W_DAC_DATA:0];
		opp_dac_init_addr		:	opp_dac_init[chan]				<= data[W_DAC_DATA:0];
		opp_dac_mult_addr		:	opp_dac_mult[chan]				<= data[W_OPP_MLT-1:0];
		opp_dac_rs_addr		:	opp_dac_rs[chan]					<= data[W_EP-1:0];
		/* dds frequency mappings */
		opp_freq_min_addr		:	opp_freq_min[chan]				<= data[W_DDS_FREQ-1:0];
		opp_freq_max_addr		:	opp_freq_max[chan]				<= data[W_DDS_FREQ-1:0];
		opp_freq_init_addr	:	opp_freq_init[chan]				<= data[W_DDS_FREQ-1:0];
		opp_freq_mult_addr	:	opp_freq_mult[chan]				<= data[W_OPP_MLT-1:0];
		opp_freq_rs_addr		:	opp_freq_rs[chan]					<= data[W_EP-1:0];
		/* dds phase mappings */
		opp_phase_min_addr	:	opp_phase_min[chan]				<= data[W_DDS_PHASE-1:0];
		opp_phase_max_addr	:	opp_phase_max[chan]				<= data[W_DDS_PHASE-1:0];
		opp_phase_init_addr	:	opp_phase_init[chan]				<= data[W_DDS_PHASE-1:0];
		opp_phase_mult_addr	:	opp_phase_mult[chan]				<= data[W_OPP_MLT-1:0];
		opp_phase_rs_addr		:	opp_phase_rs[chan]				<= data[W_EP-1:0];
		/* dds amplitude mappings */
		opp_amp_min_addr		:	opp_amp_min[chan]					<= data[W_DDS_AMP-1:0];
		opp_amp_max_addr		:	opp_amp_max[chan]					<= data[W_DDS_AMP-1:0];
		opp_amp_init_addr		:	opp_amp_init[chan]				<= data[W_DDS_AMP-1:0];
		opp_amp_mult_addr		:	opp_amp_mult[chan]				<= data[W_OPP_MLT-1:0];
		opp_amp_rs_addr		:	opp_amp_rs[chan]					<= data[W_EP-1:0];
	endcase
end

/* osf wire-out data registers */
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : osf_reg_arr
		always @( posedge clk50_in ) begin
			if ( sys_reset == 1 ) begin
				osf_data_reg[i] <= 0;
			end else if ( osf_data_valid[i] == 1 ) begin
				osf_data_reg[i] <= osf_data[i];
			end
		end
	end
endgenerate

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* frontpanel host interface */
okHost hostIf (
	.hi_in			(hi_in),
	.hi_out			(hi_out),
	.hi_inout		(hi_inout),
	.hi_aa			(hi_aa),
	.ti_clk			(ticlk),
	.ok1				(ok1),
	.ok2				(ok2)
	);

/********** Wire-ins ************/

/* data wire */
okWireIn data0_owi (
	.ok1				(ok1),
	.ep_addr			(data0_iwep),
	.ep_dataout		(data0)
	);

okWireIn data1_owi (
	.ok1				(ok1),
	.ep_addr			(data1_iwep),
	.ep_dataout		(data1)
	);

okWireIn data2_owi (
	.ok1				(ok1),
	.ep_addr			(data2_iwep),
	.ep_dataout		(data2)
	);

/* address wire */
okWireIn addr_owi (
	.ok1				(ok1),
	.ep_addr			(addr_iwep),
	.ep_dataout		(addr)
	);

/* channel wire */
okWireIn chan_owi (
	.ok1				(ok1),
	.ep_addr			(chan_iwep),
	.ep_dataout		(chan)
	);

/********** Trigger-ins ************/

/* system general purpose trigger */
okTriggerIn sys_gp_oti (
	.ok1				(ok1),
	.ep_addr			(sys_gp_itep),
	.ep_clk			(clk17_in),
	.ep_trigger		(sys_gp_trig)
	);

/* opp injection triggers */
okTriggerIn dac_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_dac_inj_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_dac_inject)
	);

okTriggerIn freq_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_freq_inj_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_freq_inject)
	);

okTriggerIn phase_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_phase_inj_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_phase_inject)
	);

okTriggerIn amp_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_amp_inj_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_amp_inject)
	);

/********** Outputs ************/

/* output wire or */
okWireOR #(
	.N					(N_ADC+1))
wireOR (
	.ok2				(ok2),
	.ok2s				(ok2x)
	);

/* osf pipe out */
okPipeOut osf_pipe (
	.ok1				(ok1),
	.ok2				(ok2x[N_ADC*17 +: 17]),
	.ep_addr			(osf_data_opep),
	.ep_datain		(osf_pipe_dout),
	.ep_read			(osf_pipe_read)
	);

/* osf pipe fifo */
pipe_tx_fifo osf_pipe_fifo (
	.ti_clk_in		(ticlk),
	.sys_clk_in		(clk50_in),
	.reset_in		(sys_reset),
	.data_valid_in	(osf_data_valid[osf_pipe_chan]),
	.data_in			(osf_data[osf_pipe_chan][W_ADC_DATA-1 -: W_EP]),
	.pipe_read_in	(osf_pipe_read),
	.data_out		(osf_pipe_dout)
	);

/* osf wire-outs */
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : osf_owo_arr
		okWireOut osf_owo (
			.ok1				(ok1),
			.ok2				(ok2x[i*17 +: 17]),
			.ep_addr			(osf_data_owep + i[7:0]),
			.ep_datain		(osf_data_reg[i][W_ADC_DATA-1 -: W_EP])
			);
	end
endgenerate

endmodule
