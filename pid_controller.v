`timescale 1ns / 1ps

// ====================================================================
// PID Controller
// ====================================================================
// Dope pid controller module.
// ====================================================================

// TODO
// * add overflow checking to osf
// * figure out how to handle non-fp pid state reset
// * figure out channel activation shit

module pid_controller (
	// inputs <- OPAL KELLY PLL
	input wire							sys_clk_in,				// system clock; max frequency determined by timing analysis
	input wire							adc_clk_in,				// adc serial clock; max frequency is 17MHz

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
	output wire							obuf_en_out = 1'b0,	// active low output buffer enable

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
reg	[N_OUT-1:0]				chan_activate;
reg	[W_INPUT_SEL-1:0]		chan_focus;
reg	[N_OUT-1:0]				chan_input_sel[0:N_OUT-1];

/* router */
wire	[N_OUT-1:0]				rtr_chan_enable;
wire	[W_ADC_DATA-1:0]		rtr_data[0:N_OUT-1];
wire	[N_OUT-1:0]				rtr_data_valid;

/* oversample filter */
reg	[W_OSF_CD-1:0]			osf_cycle_delay[0:N_OUT-1];
reg	[W_OSF_OS-1:0]			osf_os[0:N_OUT-1];
wire	[W_ADC_DATA-1:0]		osf_data[0:N_OUT-1];
wire	[N_OUT-1:0]				osf_data_valid;

/* pid core */
reg	[N_OUT-1:0]				pid_lock_en;
reg	[W_EP-1:0]				pid_setpoint[0:N_OUT-1];
reg	[W_EP-1:0]				pid_p_coef[0:N_OUT-1];
reg	[W_EP-1:0]				pid_i_coef[0:N_OUT-1];
reg	[W_EP-1:0]				pid_d_coef[0:N_OUT-1];
wire	[W_COMP-1:0]			pid_data[0:N_OUT-1];
wire	[N_OUT-1:0]				pid_data_valid;

/* output preprocessor */
reg	[W_OPP_MAX-1:0]		opp_max[0:N_OUT-1];
reg	[W_OPP_MAX-1:0]		opp_min[0:N_OUT-1];
reg	[W_OPP_MAX-1:0]		opp_init[0:N_OUT-1];
reg	[W_OPP_MLT-1:0]		opp_mult[0:N_OUT-1];
reg	[W_EP-1:0]				opp_rs[0:N_OUT-1];
wire	[W_OPP_MAX-1:0]		opp_data[0:N_OUT-1];
wire	[N_OUT-1:0]				opp_data_valid;
wire	[N_OUT-1:0]				opp_inject;

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
// initialization
//////////////////////////////////////////

/* initial routing (disable all routes) */
generate
	for ( i = 0; i < N_OUT; i = i+1 ) begin : src_select_init
		initial chan_input_sel[i] = NULL_INPUT;
	end
endgenerate

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc controller */
adc_controller #(
	.W_OUT				(W_ADC_DATA),
	.N_CHAN				(N_ADC),
	.W_CHS				(W_ADC_CHS),
	.W_OS					(W_ADC_OS))
adc_cont (
	.clk_in				(adc_clk_in),
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
	.chan_a_out			(adc_chan_a),
	.chan_b_out			(adc_chan_b),
	.data_a_out			(adc_data_a),
	.data_b_out			(adc_data_b)
	);

/* clock synchronizer */
clk_sync #(
	.W_DATA				(W_ADC_DATA),
	.W_CHS				(W_ADC_CHS),
	.N_ADC				(N_ADC))
cs (
	.sys_clk_in			(sys_clk_in),
	.reset_in			(sys_reset),
	.data_valid_in		(adc_data_valid),
	.chan_a_in			(adc_chan_a),
	.chan_b_in			(adc_chan_b),
	.data_a_in			(adc_data_a),
	.data_b_in			(adc_data_b),
	.data_valid_out	(cs_data_valid),
	.chan_out			(cs_chan),
	.data_out			(cs_data)
	);

// ====================================================================
// Start PID pipeline
// ====================================================================


// ====================================================================
// Instruction Dispatch
// ====================================================================
// Dispatch instructions into the pid pipeline. ADC data is buffered in
// a FIFO queue as it is received. Data words are pulled from the queue
// one at a time. The adc channel number for the data is checked against
// output channel mappings to determine which output channel are driven.
// A single input channel may drive multiple outputs. Seperate
// instructions are dispatched for each output channel, one after the
// other. When all instructions have been dispatched for a give data
// word, the next word is fetched from the fifo.
// ====================================================================
wire aiq_rd_en;
wire aiq_data_valid;
reg [W_ADC_CHS-1:0] opt_cnt;

// Buffer adc data. FIFO depth is 64 words. This depth can support twin
// adc channels driving 32 outputs each.
fifo_21 adc_instr_queue (
	.clk		(sys_clk_in),
	.rst		(sys_reset),
	.din		({cs_chan, cs_data}),
	.wr_en	(cs_data_valid),
	.rd_en	(aiq_rd_en),
	.dout		({idp_chan, idp_data}),
	.valid	(aiq_data_valid)
	);

// Decode instruction destination channel giving priority to lower
// numbered channels. If valid destination found, inject instruction
// into pipeline and continue search for more valid destinations. If
// no valid destination found, pull new data from the FIFO. Observe
// that blocking assignment are used. Decoder functionality depends
// on this.
always @( posedge sys_clk_in ) begin
	if ( sys_reset_in | ~aiq_data_valid ) begin
		idp_dest = NULL_OUTPUT;
		output_visited = 0;
		idp_data_valid = 0;
		aiq_rd_en = 0;
	end else begin
		idp_dest = NULL_OUTPUT; // default assignment
		for ( opt_cnt = N_OUT; opt_cnt >= 0; opt_cnt = opt_cnt - 1 ) begin
			if ( chan_input_sel[opt_cnt] == idp_chan
				& output_visited[opt_cnt] == 0 ) begin
					idp_dest = opt_cnt;
			end
		end

		if ( idp_dest != NULL_INPUT ) begin
			output_visited[idp_dest] = 1;
			idp_data_valid = 1;
			aiq_rd_en = 0;
		end else begin
			output_visited = 0;
			idp_data_valid = 0;
			aiq_rd_en = 1;
		end
	end
end

// ====================================================================
// Oversample Filter
// ====================================================================

// Assign inputs
assign osf_dv_ipt = idp_data_valid;
assign osf_dest_ipt = idp_chan;
assign osf_data_ipt = idp_data;

always @( posedge sys_clk_in ) begin

	// ----------------------- Pipe Stage 1 ----------------------------
	// Register inputs
	osf_dv[1] = osf_dv_ipt;
	osf_dest[1] = osf_dest_ipt;
	osf_data[1] = osf_data_ipt;

	// Fetch sum and sample count
	osf_sum[1] = osf_sum[osf_dest_ipt];
	osf_count[1] = osf_count[osf_dest_ipt];

	// ----------------------- Pipe Stage 2 ----------------------------
	// Pass data valid and destination
	osf_dv[2] = osf_dv[1];
	osf_dest[2] = osf_dest[1];

	// Accumlate sum and increment sample count
	osf_sum[2] = osf_sum[1] + osf_data[1];
	osf_count[2] = osf_count[1] + 1'b1;

	// Fetch oversample mode
	osf_os[2] = osf_os[osf_dest[1]];

	// ----------------------- Pipe Stage 3 ----------------------------
	// Check if sample count has been satisifed
	osf_count_sat = ( osf_count[2][osf_os[2]] == 1 ); // assignment must be blocking

	// Pass data valid signal if count satisfied
	osf_dv[3] = ( osf_count_sat ) ? osf_dv[2] : 0;

	// Divide sum by right shifting
	osf_data[3] = osf_sum[2] >>> osf_os[2];

	// Pass destination
	osf_dest[3] = osf_dest[2];

	// Writeback count and sum. Reset both if sample count has been satisfied
	osf_sum[osf_dest[2]] = ( osf_count_sat ) ? 0 : osf_sum[2];
	osf_count[osf_dest[2]] = ( osf_count_sat ) ? 0 : osf_count[2];

end

// Assign outputs
assign osf_dv_opt = osf_dv[3];
assign osf_dest_opt = osf_dest[3];
assign osf_data_opt = osf_data[3];


// ====================================================================
// PID Filter
// ====================================================================

// Assign inputs
assign pid_dv_ipt = osf_dv_opt;
assign pid_dest_ipt = osf_dest_opt;
assign pid_data_ipt = osf_data_opt;

always @( posedge sys_clk_in ) begin

	// pid fetch
	pid_dv[1] <= pid_dv_ipt;
	pid_dest[1] <= pid_dest_ipt;
	pid_data[1] <= pid_data_ipt;

	pid_setpoint[5] <= pid_setpoint[dest_addr[4]];
	pid_p_coef[5] <= pid_p_coef[dest_addr[4]];
	pid_i_coef[5] <= pid_i_coef[dest_addr[4]];
	pid_d_coef[5] <= pid_d_coef[dest_addr[4]];
	pid_lock_en[5] <= pid_locK_en[dest_addr[4]];
	pid_data_prev[5] <= pid_data_prev[dest_addr[4]];
	pid_error_prev0[5] <= pid_error_prev0[dest_addr[4]];
	pid_error_prev1[5] <= pid_error_prev1[dest_addr[4]];

	// pid compute error and z-transform coefficients
	// only pass data if pid lock enabled
	osf_data[6] <= (pid_lock_en[5]) ? osf_data[5] : 0;
	data_valid[6] <= data_valid[6];
	dest_addr[6] <= dest_addr[5];

	pid_error[6] <= pid_setpoint[5] - osf_data[5];
	pid_k1[6] <= pid_p_coef[5] + pid_i_coef[5] + pid_d_coef[5];
	pid_k2[6] <= -pid_p_coef[5] - (pid_d_coef[5] << 1);
	pid_k3[6] <= pid_d_coef[5]

	pid_data_prev[6] <= pid_data_prev[5];
	pid_error_prev0[6] <= pid_error_prev0[5];
	pid_error_prev1[6] <= pid_error_prev1[5];

	// pid compute coefficient error products
	// writeback error data
	data_valid[7] <= data_valid[6];
	dest_addr[7] <= dest_addr[6];

	pid_ce_prod0[7] <= pid_k1[6] * pid_error[6];
	pid_ce_prod1[7] <= pid_k2[6] * pid_error_prev0[6];
	pid_ce_prod2[7] <= pid_k3[6] * pid_error_prev1[6]

	pid_error_prev[dest_addr[6]] <= pid_error[6];
	pid_error_prev1[dest_addr[6]] <= pid_error_prev0[6];

	pid_data_prev[7] <= pid_data_prev[6];

	// pid compute delta
	data_valid[8] <= data_valid[7];
	dest_addr[8] <= dest_addr[7];

	pid_delta[8] <= pid_ce_prod0[7] + pid_ce_prod1[7] + pid_ce_prod2[7];

	pid_data_prev[8] <= pid_data_prev[7];

	// pid compute output data
	// merge this shit with below
	pid_data[9] <= pid_delta[8] + pid_data_prev[8];
	data_valid[9] <= data_valid[8];
	dest_addr[9] <= dest_addr[8];

	pid_data_prev[9] <= pid_data_prev[8];

	// pid check overflow
	pid_data[10] <= pid_data[9];
	data_valid[10] <= data_valid[9];
	dest_addr[10] <= dest_addr[9];

	pid_overflow[10] = (pid_delta[9][W_OUT-1] == pid_data_prev[9][W_OUT-1])
							&& (pid_data_prev[9][W_OUT-1] != pid_data[9][W_OUT-1]);
	pid_data_rail[10] = (pid_data_prev[9][W_OUT-1] == 0) ? PID_MAX_OUTPUT : PID_MIN_OUTPUT;

	// pid correct overflow
	// merge with below
	pid_data[11] <= (pid_overflow[10]) ? pid_data_rail[10] : pid_data[10];
	data_valid[11] <= data_valid[10];
	dest_addr[11] <= dest_addr[10];

	// pid writeback data
	pid_data[12] <= pid_data[11];
	data_valid[12] <= data_valid[11];
	dest_addr[12] <= dest_addr[11];

	pid_data_prev[dest_addr[11]] <= pid_data[11];

end

// ====================================================================
// Output Preprocessor
// ====================================================================

// Assign inputs

always @( posedge sys_clk_in ) begin

	// -----------------------------------------------------------
	// output preprocessor
	// -----------------------------------------------------------

	// fetch multiplier
	pid_data[13] <= pid_data[12];
	data_valid[13] <= data_valid[12];
	dest_addr[13] <= dest_addr[12];

	opp_mult[13] <= opp_mult[dest_addr[12]];

	// multiply and fetch right shift
	opp_data[14] <= pid_data[13] * opp_mult[13];
	data_valid[14] <= data_valid[13];
	dest_addr[14] <= dest_addr[13];

	opp_rs[14] <= opp_rs[dest_addr[13]];

	// right shift and fetch previous data
	opp_data[15] <= opp_data[14] >>> opp_rs[14];
	data_valid[15] <= data_valid[14];
	dest_addr[15] <= dest_addr[14];

	opp_data_prev[15] <= opp_data_prev[dest_addr[14]];

	// accumulate and fetch max
	opp_data[16] <= opp_data[15] + opp_data_prev[15];
	data_valid[16] <= data_valid[15];
	dest_addr[16] <= dest_addr[15];

	opp_max[16] <= opp_max[dest_addr[15]]

	// check upper bound and fetch min
	opp_data[17] <= (opp_data[16] > opp_max[16]) ? opp_max[16] : opp_data[16];
	data_valid[17] <= data_valid[16];
	dest_addr[17] <= dest_addr[16];

	opp_min[17] <= opp_min[dest_addr[16]];

	// check lower bound
	opp_data[18] <= (opp_data[17] < opp_min[17]) ? opp_min[17] : opp_data[17];
	data_valid[18] <= data_valid[17];
	dest_addr[18] <= dest_addr[17];

	// writeback
	opp_data[19] <= opp_data[18];
	data_valid[19] <= data_valid[18];
	dest_addr[19] <= dest_addr[18];

	opp_data_prev[dest_addr[18]] <= opp_data[18];

end

wire [W_COMP-1:0] pipeline_data = opp_data[19];
wire [W_EP-1:0] pipeline_dest = dest_addr[19];
wire pipeline_dv = data_valid[19];
wire pipeline_dac_dv = (pipeline_dest < N_DAC) ? pipeline_dv : 0;

wire [W_DAC_DATA-1:0] dac_data;
wire [W_DAC_CHS-1:0] dac_chan;
wire dac_dv;
wire dac_update_done;

/* dac instruction queue */
fifo_19 dac_instr_queue (
	.clk		(sys_clk_in),
	.rst		(sys_reset),
	.din		({pipeline_dest[W_DAC_CHS:0], pipeline_data[W_DAC_DATA-1:0]),
	.wr_en	(pipeline_dac_dv),
	.rd_en	(dac_update_done),
	.dout		({dac_chan, dac_data}),
	.valid	(dac_dv)
	);

/* dac controller */
dac_controller #(
	.W_DATA				(W_DAC_DATA),
	.W_CHS				(W_DAC_CHS),
	.N_CHAN				(N_DAC))
dac_cntrl (
	.clk_in				(sys_clk_in),
	.reset_in			(sys_reset),
	.ref_set_in			(dac_ref_set),
	.data_in				(dac_data),
	.channel_in			(dac_chan),
	.data_valid_in		(dac_dv),
	.nldac_out			(dac_nldac_out),
	.nsync_out			(dac_nsync_out),
	.sclk_out			(dac_sclk_out),
	.din_out				(dac_din_out),
	.nclr_out			(dac_nclr_out),
	.dac_done_out		(dac_update_done),
	.data_out			(),
	.channel_out		()
	);

/* dds controller array */
generate
	for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_array
		localparam F = FREQ0_ADDR + i;	// frequency absolute index
		localparam P = PHASE0_ADDR + i;	// phase absolute index
		localparam A = AMP0_ADDR + i;		// amplitude absolute index

		pipeline_freq_dv[i] = (pipeline_dest == F) ? pipeline_dv : 0;
		pipeline_phase_dv[i] = (pipeline_dest == P) ? pipeline_dv : 0;
		pipeline_amp_dv[i] = (pipeline_dest == A) ? pipeline_dv : 0;

		dds_controller dds_cntrl (
			.clk_in				(sys_clk_in),
			.reset_in			(sys_reset),
			.freq_in				(pipeline_data[W_FREQ_DATA-1:0]),
			.phase_in			(pipeline_data[W_PHASE_DATA-1:0]),
			.amp_in				(pipeline_data[W_AMP_DATA-1:0]),
			.freq_dv_in			(pipeline_freq_dv),
			.phase_dv_in		(pipeline_phase_dv),
			.amp_dv_in			(pipeline_amp_dv),
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

/* handle state write requests */
always @( posedge write_data ) begin
	case ( addr_usb )
		/* adc state */
		adc_os_addr				:	adc_os							<= data_usb[W_ADC_OS-1:0];
		/* general channel state */
		chan_activate_addr	:	chan_activate[chan_usb]		<= data_usb[0];
		chan_focus_addr		:	chan_focus						<= chan_usb;
		chan_input_sel_addr	:	chan_input_sel[chan_usb]	<= data_usb[W_INPUT_SEL-1:0];
		/* osf state */
		osf_cycle_delay_addr	:	osf_cycle_delay[chan_usb]	<= data_usb[W_OSF_CD-1:0];
		osf_os_addr				:	osf_os[chan_usb]				<= data_usb[W_OSF_OS-1:0];
		/* pid state */
		pid_setpoint_addr		:	pid_setpoint[chan_usb]		<= data_usb[W_EP-1:0];
		pid_p_coef_addr		:	pid_p_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_i_coef_addr		:	pid_i_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_d_coef_addr		:	pid_d_coef[chan_usb]			<= data_usb[W_EP-1:0];
		pid_lock_en_addr		:	pid_lock_en[chan_usb]		<= data_usb[0];
		/* opp state */
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
		always @( posedge sys_clk_in ) begin
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
	.ep_clk			(adc_clk_in),
	.ep_trigger		(sys_gp_trig)
	);

/* opp injection triggers */
okTriggerIn freq_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_inject1_itep),
	.ep_clk			(sys_clk_in),
	.ep_trigger		(opp_inject1_trig)
	);

okTriggerIn phase_inj_oti (
	.ok1				(ok1),
	.ep_addr			(opp_inject0_itep),
	.ep_clk			(sys_clk_in),
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
	.sys_clk_in		(sys_clk_in),
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
			.ep_addr			(osf_data0_owep + i[7:0]),
			.ep_datain		(osf_data_reg[i][W_ADC_DATA-1 -: W_EP])
			);
	end
endgenerate

endmodule
