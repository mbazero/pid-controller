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
// internal structures
//////////////////////////////////////////

/* all modules */
wire								sys_reset;

/* adc controller */
wire								adc_cstart;
reg	[W_ADC_OS-1:0]			adc_os;
wire	[W_ADC_DATA-1:0]		adc_data_a;
wire	[W_ADC_DATA-1:0]		adc_data_b;
wire	[N_ADC-1:0]				adc_data_valid;

/* clock synchronizer */
wire	[W_ADC_DATA-1:0]		cs_data_a;
wire	[W_ADC_DATA-1:0]		cs_data_b;
wire	[N_ADC-1:0]				cs_data_valid;

/* general channel */
reg	[N_CHAN-1:0]			chan_activate;
reg	[W_SRC_SEL-1:0]		chan_focus;
reg	[W_SRC_SEL-1:0]		chan_src_sel[0:N_CHAN-1];

/* router */
wire	[N_CHAN-1:0]			rtr_chan_enable;
wire	[W_ADC_DATA-1:0]		rtr_data[0:N_CHAN-1];
wire	[N_CHAN-1:0]			rtr_data_valid;

/* oversample filter */
reg	[W_OSF_CD-1:0]			osf_cycle_delay[0:N_CHAN-1];
reg	[W_OSF_OS-1:0]			osf_os[0:N_CHAN-1];
wire	[W_ADC_DATA-1:0]		osf_data[0:N_CHAN-1];
wire	[N_CHAN-1:0]			osf_data_valid;

/* pid core */
reg	[N_CHAN-1:0]			pid_lock_en;
reg	[W_EP-1:0]				pid_setpoint[0:N_CHAN-1];
reg	[W_EP-1:0]				pid_p_coef[0:N_CHAN-1];
reg	[W_EP-1:0]				pid_i_coef[0:N_CHAN-1];
reg	[W_EP-1:0]				pid_d_coef[0:N_CHAN-1];
wire	[W_COMP-1:0]			pid_data[0:N_CHAN-1];
wire	[N_CHAN-1:0]			pid_data_valid;

/* output preprocessor */
reg	[W_OPP_MAX-1:0]		opp_max[0:N_CHAN-1];
reg	[W_OPP_MAX-1:0]		opp_min[0:N_CHAN-1];
reg	[W_OPP_MAX-1:0]		opp_init[0:N_CHAN-1];
reg	[W_OPP_MLT-1:0]		opp_mult[0:N_CHAN-1];
reg	[W_EP-1:0]				opp_rs[0:N_CHAN-1];
wire	[W_OPP_MAX-1:0]		opp_data[0:N_CHAN-1];
wire	[N_CHAN-1:0]			opp_data_valid;
wire	[N_CHAN-1:0]			opp_inject;

/* seperate dac opp data */
wire	[W_DAC_DATA*N_DAC-1:0]	opp_dac_data_packed;
wire	[N_DAC-1:0]					opp_dac_data_valid;

/* dac instruction queue */
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

/* active low output buffer */
assign obuf_en_out = 1'b0;

/* generate channel enable signals */
genvar i;
generate
	for (i = 0; i < N_CHAN; i = i + 1) begin : rtr_chan_enable_array
		/* enable channel if it is in the active state and has a valid source */
		assign rtr_chan_enable[i] = chan_activate[i] & (chan_src_sel[i] >= 0) & (chan_src_sel[i] < N_ADC);
	end
endgenerate

/* multiplex rtr data */
generate
	for ( i = 0; i < N_CHAN; i = i + 1 ) begin : rtr_data_array
		/* cs channel a carries data for channels 0-3 and
			cs channel b carries data for channels 4-8 */
		assign rtr_data[i] = (chan_src_sel[i] < 4) ? cs_data_a : cs_data_b;
	end
endgenerate

/* pack dac opp data for dac instruction queue */
generate
	for ( i = 0; i < N_DAC; i = i + 1 ) begin : opp_dac_data_array
		assign opp_dac_data_packed[ i*W_DAC_DATA +: W_DAC_DATA ] = opp_data[map_dac(i)][W_DAC_DATA-1:0];
	end
endgenerate

/* seperate dac opp data valid signals */
assign opp_dac_data_valid = opp_data_valid[map_dac(N_DAC-1):map_dac(0)];

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* initial routing (disable all routes) */
generate
	for ( i = 0; i < N_CHAN; i = i+1 ) begin : src_select_init
		initial chan_src_sel[i] = NULL_SRC;
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
function [W_SRC_SEL-1:0] map_dac;
	input [W_EP-1:0] rel_index;
	begin
		map_dac = rel_index;
	end
endfunction

function [W_SRC_SEL-1:0] map_freq;
	input [W_EP-1:0] rel_index;
	begin
		map_freq = N_DAC + rel_index;
	end
endfunction

function [W_SRC_SEL-1:0] map_phase;
	input [W_EP-1:0] rel_index;
	begin
		map_phase = N_DAC + N_DDS + rel_index;
	end
endfunction

function [W_SRC_SEL-1:0] map_amp;
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


/* routing */
generate
	for ( i = 0; i < N_CHAN; i = i+1 ) begin : mux_array
		mux_n_chan #(
			.W_CHAN				(1),
			.W_SEL				(W_SRC_SEL),
			.N_IN					(N_ADC))
		mux_inst (
			.data_packed_in	(cs_data_valid),
			.chan_select_in	(chan_src_sel[i]),
			.enable_in			(rtr_chan_enable[i]),
			.data_out			(rtr_data_valid[i])
			);
	end
endgenerate

// -----------------------------------------------------------
// start pid pipeline
// -----------------------------------------------------------

/* oversample filter array */
generate
	for ( i = 0; i < N_CHAN; i = i + 1 ) begin : osf_array
		oversample_filter #(
			.W_DATA				(W_ADC_DATA),
			.W_EP					(W_EP),
			.W_OSM				(W_OSF_OS))
		ovr_inst_a (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[i]),
			.data_valid_in		(rtr_data_valid[i]),
			.cycle_delay_in	(osf_cycle_delay[i]),
			.os_in				(osf_os[i]),
			.activate_in		(1'b1),
			.data_out			(osf_data[i]),
			.data_valid_out	(osf_data_valid[i])
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

/* dac preprocessor array */
generate
	for ( i = 0; i < N_DAC; i = i + 1 ) begin : dac_opp_array
		localparam D = map_dac(i); // dac absolute index

		/* The output preprocessor treats its output and
		 * output bounds as signed values. The DAC accepts
		 * unsigned values. Thus modifications to W_OUT,
		 * output_min_in, output_max_in, and output_init_in
		 * are required as below. The sign bit of data_out
		 * is also discarded.
		 */

		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_DAC_DATA + 1),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		dac_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(pid_data[D]),
			.data_valid_in		(pid_data_valid[D] | opp_inject[D]),
			.lock_en_in			(pid_lock_en[D]),
			.output_max_in		({1'b0, opp_max[D][W_DAC_DATA-1:0]}),
			.output_min_in		({1'b0, opp_min[D][W_DAC_DATA-1:0]}),
			.output_init_in	({1'b0, opp_init[D][W_DAC_DATA-1:0]}),
			.multiplier_in		(opp_mult[D]),
			.right_shift_in	(opp_rs[D]),
			.data_out			(opp_data[D][W_DAC_DATA:0]),
			.data_valid_out	(opp_data_valid[D])
			);
	end
endgenerate

/* dds preprocessor array */
generate
	for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_opp_array
		localparam F = map_freq(i);	// frequency absolute index
		localparam P = map_phase(i);	// phase absolute index
		localparam A = map_amp(i);		// amplitude absolute index

		/* The output preprocessor treats its output and
		 * output bounds as signed values. The DDS accepts
		 * unsigned values for frequency, phase, and amp.
		 * Thus modifications to W_OUT, output_min_in,
		 * output_max_in, and output_init_in are required
		 * as below. The sign bits of the data_out ports
		 * are also discarded.
		 */

		/* frequency output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_FREQ_DATA + 1),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		freq_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(pid_data[F]),
			.data_valid_in		(pid_data_valid[F] | opp_inject[F]),
			.lock_en_in			(pid_lock_en[F]),
			.output_max_in		({1'b0, opp_max[F][W_FREQ_DATA-1:0]}),
			.output_min_in		({1'b0, opp_min[F][W_FREQ_DATA-1:0]}),
			.output_init_in	({1'b0, opp_init[F][W_FREQ_DATA-1:0]}),
			.multiplier_in		(opp_mult[F]),
			.right_shift_in	(opp_rs[F]),
			.data_out			(opp_data[F][W_FREQ_DATA:0]),
			.data_valid_out	(opp_data_valid[F])
			);

		/* phase output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_PHASE_DATA + 1),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		phase_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(pid_data[P]),
			.data_valid_in		(pid_data_valid[P] | opp_inject[P]),
			.lock_en_in			(pid_lock_en[P]),
			.output_max_in		({1'b0, opp_max[P][W_PHASE_DATA-1:0]}),
			.output_min_in		({1'b0, opp_min[P][W_PHASE_DATA-1:0]}),
			.output_init_in	({1'b0, opp_init[P][W_PHASE_DATA-1:0]}),
			.multiplier_in		(opp_mult[P]),
			.right_shift_in	(opp_rs[P]),
			.data_out			(opp_data[P][W_PHASE_DATA:0]),
			.data_valid_out	(opp_data_valid[P])
			);

		/* amplitude output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_COMP),
			.W_OUT 				(W_AMP_DATA + 1),
			.W_MLT				(W_OPP_MLT),
			.W_EP					(W_EP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		amp_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.pid_sum_in			(pid_data[A]),
			.data_valid_in		(pid_data_valid[A] | opp_inject[A]),
			.lock_en_in			(pid_lock_en[A]),
			.output_max_in		({1'b0, opp_max[A][W_AMP_DATA-1:0]}),
			.output_min_in		({1'b0, opp_min[A][W_AMP_DATA-1:0]}),
			.output_init_in	({1'b0, opp_init[A][W_AMP_DATA-1:0]}),
			.multiplier_in		(opp_mult[A]),
			.right_shift_in	(opp_rs[A]),
			.data_out			(opp_data[A][W_AMP_DATA:0]),
			.data_valid_out	(opp_data_valid[A])
			);
	end
endgenerate

// -----------------------------------------------------------
// end pid pipeline
// -----------------------------------------------------------


/* dac instruction queue */
dac_instr_queue #(
	.W_DATA				(W_DAC_DATA),
	.W_CHS				(W_DAC_CHS),
	.N_CHAN				(N_DAC))
dac_iq (
	.clk_in				(clk50_in),
	.reset_in			(sys_reset),
	.data_packed_in	(opp_dac_data_packed),
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


/* dds controller array */
generate
	for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_array
		localparam F = map_freq(i);	// frequency absolute index
		localparam P = map_phase(i);	// phase absolute index
		localparam A = map_amp(i);		// amplitude absolute index

		dds_controller dds_cntrl (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.freq_in				(opp_data[F]),
			.phase_in			(opp_data[P]),
			.amp_in				(opp_data[A]),
			.freq_dv_in			(opp_data_valid[F]),
			.phase_dv_in		(opp_data_valid[P]),
			.amp_dv_in			(opp_data_valid[A]),
			.sclk_out			(dds_sclk_out[i]),
			.reset_out			(dds_reset_out[i]),
			.csb_out				(dds_csb_out[i]),
			.sdio_out			(dds_sdio_out[i]),
			.io_update_out		(dds_io_update_out[i]),
			.dds_done_out		(dds_done[i])
			);
	end
endgenerate


// -----------------------------------------------------------
// frontpanel interface
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
// the chan_focus register.
// -----------------------------------------------------------

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* data, channel, and address usb wires */
wire	[W_EP-1:0]				data2_usb, data1_usb, data0_usb;
wire	[W_EP*3-1:0]			data_usb;
wire	[W_EP-1:0]				chan_usb;
wire	[W_EP-1:0]				addr_usb;
wire								write_data;

/* system and opp injection triggers */
wire 	[W_EP-1:0]				sys_gp_trig;
wire	[W_EP-1:0]				opp_inject1_trig, opp_inject0_trig;

/* osf wire-out and pipe structures */
reg	[W_ADC_DATA-1:0]		osf_data_reg[0:N_ADC-1];
wire	[15:0]					osf_pipe_dout;
wire								osf_pipe_read;

/* host interface */
wire 								ticlk;
wire	[30:0]					ok1;
wire	[16:0] 					ok2;
wire	[17*(N_ADC+1)-1:0]	ok2x;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* data bus */
assign data_usb		= {data2_usb, data1_usb, data0_usb};

/* frontpanel control */
assign i2c_sda			= 1'bz;
assign i2c_scl			= 1'bz;
assign hi_muxsel		= 1'b0;

/* system general purpose trigger mappings */
assign sys_reset		= sys_gp_trig[sys_reset_offset];
assign adc_cstart		= sys_gp_trig[adc_cstart_offset];
assign write_data		= sys_gp_trig[write_data_offset];
assign dac_ref_set	= sys_gp_trig[dac_ref_set_offset];

/* opp injection trigger mapping */
assign opp_inject		= {opp_inject1_trig, opp_inject0_trig};

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* input data multiplexing */
always @( posedge write_data ) begin
	case ( addr_usb )
		/* adc mappings */
		adc_os_addr				:	adc_os							<= data_usb[W_ADC_OS-1:0];
		/* channel mappings */
		chan_activate_addr	:	chan_activate[chan_usb]		<= data_usb[0];
		chan_focus_addr		:	chan_focus						<= chan_usb;
		chan_src_sel_addr		:	chan_src_sel[chan_usb]		<= data_usb[W_SRC_SEL-1:0];
		/* osf mappings */
		osf_cycle_delay_addr	:	osf_cycle_delay[chan_usb]	<= data_usb[W_OSF_CD-1:0];
		osf_os_addr				:	osf_os[chan_usb]				<= data_usb[W_OSF_OS-1:0];
		/* pid mappings */
		pid_setpoint_addr		:	pid_setpoint[chan_usb]		<= data_usb[W_EP-1:0];
		pid_p_coef_addr		:	pid_p_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_i_coef_addr		:	pid_i_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_d_coef_addr		:	pid_d_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_lock_en_addr		:	pid_lock_en[chan_usb]		<= data_usb[0];
		/* opp mappings */
		opp_min_addr			:	opp_min[chan_usb]				<= data_usb[W_OPP_MAX-2:0];
		opp_max_addr			:	opp_max[chan_usb]				<= data_usb[W_OPP_MAX-2:0];
		opp_init_addr			:	opp_init[chan_usb]			<= data_usb[W_OPP_MAX-2:0];
		opp_mult_addr			:	opp_mult[chan_usb]			<= data_usb[W_OPP_MLT-2:0];
		opp_rs_addr				:	opp_rs[chan_usb]				<= data_usb[W_EP-1:0];
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
	.ep_dataout		(data0_usb)
	);

okWireIn data1_owi (
	.ok1				(ok1),
	.ep_addr			(data1_iwep),
	.ep_dataout		(data1_usb)
	);

okWireIn data2_owi (
	.ok1				(ok1),
	.ep_addr			(data2_iwep),
	.ep_dataout		(data2_usb)
	);

/* address wire */
okWireIn addr_owi (
	.ok1				(ok1),
	.ep_addr			(addr_iwep),
	.ep_dataout		(addr_usb)
	);

/* input channel wire */
okWireIn chan_owi (
	.ok1				(ok1),
	.ep_addr			(chan_iwep),
	.ep_dataout		(chan_usb)
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
okTriggerIn freq_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_inject1_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_inject1_trig)
	);

okTriggerIn phase_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_inject0_itep),
	.ep_clk			(clk50_in),
	.ep_trigger		(opp_inject0_trig)
	);

/********** Outputs ************/

/* output wire or */
okWireOR #(
	.N					(N_ADC+1))
wireOR (
	.ok2				(ok2),
	.ok2s				(ok2x)
	);

/* osf pipe out for block data transfer */
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
	.data_valid_in	(osf_data_valid[chan_focus]),
	.data_in			(osf_data[chan_focus][W_ADC_DATA-1 -: W_EP]),
	.pipe_read_in	(osf_pipe_read),
	.data_out		(osf_pipe_dout)
	);

/* osf wire-outs for single data transfer */
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
