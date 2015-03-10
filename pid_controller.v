`timescale 1ns / 1ps

// two_dds_test -- mba 2014

// TODO
// - figure out lock en signal (is it really needed, if so, make it a frontpanel param)
// - HUGE: you need to totally revamp the cycle control unit. see notes that module.
// - modify adc reset so it is synchronous with the 17MHz clock
// - check modules instantiation in dds data path in this TLE (dds OPPs and dds controllers)
// - parameterize dds controller
// - add phase and amplitude funcitonality to dds controller
// - implement proper opp min and max endpoints in frontpanel interface
// - need way for multiple sources to drive same channel

// RESOLVED
// - opp min/max/init signal widths are all fucked up
//		> kept as full length in frontpanel controller and did sub-mappings in module instatiations
// - remove ovr cycle start signal and implement the functionality internally to osf
// - figure out and specify what drives adc_cstart
//		> temporarily resolved
// - adc controller clock disparity

module pid_controller #(
	// parameters
	parameter N_ADC			= 6,	// number of adc channels
	parameter N_DAC			= 8,	// number of dac channels
	parameter N_DDS			= 2,	// number of dds channels
	parameter W_ADC			= 18, // width of adc channels
	parameter W_OSF			= 18, // width of oversample filter output
	parameter W_PID			= 16, // width of pid core output
	parameter W_DAC_INST		= 32, // width of dac update instruction
	parameter W_DDS_FREQ		= 48, // width of dds frequency word
	parameter W_DDS_PHASE	= 14,	// width of dds phase word
	parameter W_DDS_AMP 		= 10, // width of dds amplitude instruction
	parameter W_DAC_DATA		= 16,	// width of dac data input
	parameter T_ADC_CYCLE	= 85	// adc conversion cycle time in number of adc clock cycles
	)(
	// inputs <- OPAL KELLY PLL
	input wire							clk50_in,				// 50MHz system clock
	input wire							clk17_in,				// 17MHz adc serial clock

	// inputs <- ADC - AD7608
	input wire							adc_busy_in,
	input wire							adc_data_a_in,
	input wire							adc_data_b_in,

	// outputs -> ADC - AD7608
	output wire		[2:0]				adc_os_out,
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
	output wire							n_out_buf_en,	// breakout board output buffer enable (active low)

	// inouts <-> frontpanel host interface
	input wire		[7:0]				hi_in,
	output wire		[1:0]				hi_out,
	inout wire		[15:0]			hi_inout,
	inout wire							hi_aa,

	output wire							i2c_sda,
	output wire							i2c_scl,
	output wire							hi_muxsel,

	//////////////////////DEBUG//////////////////////
	output wire adc_busy_db,
	output wire adc_data_a_db,
	output wire adc_data_b_db,

	output wire [2:0] adc_os_db,
	output wire adc_convst_db,
	output wire adc_reset_db,
	output wire adc_sclk_db,
	output wire adc_n_cs_db,

	output wire[6:0] adc_data_a_vect_db,
	output wire adc_data_valid_db,

	output wire adc_cstart_db,
	output wire mod_update_db,
	output wire clk17_db
	);

//////////////////////DEBUG//////////////////////
assign adc_busy_db = adc_busy_in;
assign adc_data_a_db = adc_data_a_in;
assign adc_data_b_db = adc_data_b_in;

assign adc_os_db = adc_os_out;
assign adc_convst_db = adc_convst_out;
assign adc_reset_db = adc_reset_out;
assign adc_sclk_db = adc_sclk_out;
assign adc_n_cs_db = adc_n_cs_out;

wire [17:0] adc_data0_db;
assign adc_data_a_vect_db = adc_data0_db[17:11];
assign adc_data_valid_db = | adc_data_valid;

assign adc_cstart_db = adc_cstart;
assign mod_update_db = module_update;
assign clk17_db = clk17_in;

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* parameters */
localparam N_OUT 					= N_DAC + 3*N_DDS;	// total number of output channels; each dds has three output channels (phase, freq, and amp)
localparam W_PIDV 				= W_PID + 1;			// width of pid data bus with valid signal
localparam W_RTR_SEL 			= 4;						// width of router select signal
localparam PIPE_LATENCY 		= 5;						// latency in clock cycles of pipeline
localparam W_OSF_ORT				= 6;						// width of oversample ratio signal
localparam W_OSF_CD				= 16;						// width of osf cycle delay signal
localparam PID_COMP_LATENCY	= 1;						// pid core computation latency
localparam OPP_COMP_LATENCY	= 1;						// output preprocessor compuation latency

/* all modules */
wire								sys_reset;
wire								module_update;

/* adc controller */
wire								adc_cstart;
wire	[2:0]						adc_os;
wire	[N_ADC-1:0]				adc_data_valid;
wire	[W_ADC-1:0]				adc_data_a;
wire	[W_ADC-1:0]				adc_data_b;

/* clock synchronizer */
wire	[N_ADC-1:0]				cs_data_valid;
wire	[W_ADC-1:0]				cs_data_a;
wire	[W_ADC-1:0]				cs_data_b;

/* oversample filter */
wire	[N_ADC-1:0]				osf_activate;
wire	[N_ADC-1:0]				osf_update_en;
wire	[W_OSF_CD-1:0]			osf_cycle_delay;
wire	[W_OSF_ORT-1:0]		osf_log_ovr;
wire	[W_OSF-1:0]				osf_data[0:N_ADC-1];
wire	[N_ADC-1:0]				osf_data_valid;

/* pid core */
wire	[N_ADC-1:0]				pid_update_en;
wire	[N_ADC-1:0]				pid_clear;
wire	[15:0]					pid_setpoint;
wire	[15:0]					pid_p_coef;
wire	[15:0]					pid_i_coef;
wire	[15:0]					pid_d_coef;
wire	[W_PID-1:0]				pid_data[0:N_ADC-1];
wire	[N_ADC-1:0]				pid_data_valid;

/* router */
wire	[W_RTR_SEL-1:0]		rtr_src_sel;
wire	[W_RTR_SEL-1:0]		rtr_dest_sel;
wire	[W_PIDV*N_ADC-1:0]	rtr_input_bus;
wire	[W_PIDV*N_OUT-1:0]	rtr_output_bus;
wire	[W_PID-1:0]				rtr_data[0:N_OUT-1];
wire	[N_OUT-1:0]				rtr_data_valid;

/* output preprocessor */
wire	[N_OUT-1:0]				opp_update_en;
wire	[N_OUT-1:0]				opp_lock_en;
wire	[47:0]					opp_max;
wire	[47:0]					opp_min;
wire	[47:0]					opp_init;
wire	[7:0]						opp_multiplier;
wire	[W_DAC_DATA-1:0]		opp_dac_data[0:N_DAC-1];
wire	[W_DDS_FREQ-1:0]		opp_freq_data[0:N_DDS-1];
wire	[W_DDS_PHASE-1:0]		opp_phase_data[0:N_DDS-1];
wire	[W_DDS_AMP-1:0]		opp_amp_data[0:N_DDS-1];
wire	[N_DAC-1:0]				opp_dac_data_valid;
wire	[N_DDS-1:0]				opp_freq_data_valid;
wire	[N_DDS-1:0]				opp_phase_data_valid;
wire	[N_DDS-1:0]				opp_amp_data_valid;

/* cycle controller */
wire	[W_DAC_DATA*N_DAC-1:0]	ccu_input_bus;
wire	[W_DAC_DATA-1:0]		ccu_data;
wire	[2:0]						ccu_channel;
wire								ccu_data_valid;
wire								ccu_dac_stall;

/* dac controller */
wire								dac_ref_set;
wire								dac_done;
wire	[W_DAC_DATA-1:0]		dac_data;
wire	[2:0]						dac_chan;

/* dds controller */
wire	[N_DDS-1:0]				dds_done;


//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* output buffer enable */
assign n_out_buf_en = 1'b0;

/* concatinate pid data and valid signals to single data bus for presentation to router */
genvar i;
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : rtr_in_arr
		assign rtr_input_bus[ i*W_PIDV +: W_PIDV ] = {pid_data_valid[i], pid_data[i]};
	end
endgenerate

/* split router output data bus to seperate channels */
genvar j;
generate
	for ( j = 0; j < N_OUT; j = j + 1 ) begin : rtr_out_arr
		assign rtr_data[j] 			= rtr_output_bus[ j*W_PIDV +: W_PID ];
		assign rtr_data_valid[j]	= rtr_output_bus[ j*W_PIDV + W_PID ];
	end
endgenerate

/* concatinate dac output channels to single data bus for presentation to cycle controller */
genvar k;
generate
	for ( k = 0; k < N_DAC; k = k + 1 ) begin : ccu_in_arr
		assign ccu_input_bus[ k*W_DAC_DATA +: W_DAC_DATA ] = opp_dac_data[k];
	end
endgenerate

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc controller */
adc_controller #(
	.W_OUT				(W_ADC),
	.N_CHAN				(N_ADC),
	.T_CYCLE				(T_ADC_CYCLE))
adc_cont (
	.clk_in				(clk17_in),
	.reset_in			(sys_reset),
	.busy_in				(adc_busy_in),
	.data_a_in			(adc_data_a_in),
	.data_b_in			(adc_data_b_in),
	.os_in				(adc_os),
	.update_in			(module_update),
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
	.W_DATA				(W_ADC),
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
genvar l;
generate
	for ( l = 0; l < N_ADC/2; l = l + 1 ) begin : osf_array
		/* osf bank a: draws from adc channel a */
		oversample_filter #(
			.W_IN					(W_ADC),
			.W_OUT				(W_OSF),
			.W_ORT				(W_OSF_ORT))
		ovr_inst_a (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_a),
			.data_valid_in		(cs_data_valid[l]),
			.cycle_delay_in	(osf_cycle_delay),
			.log_ovr_in			(osf_log_ovr),
			.activate_in		(osf_activate[l]),
			.update_en_in		(osf_update_en[l]),
			.update_in			(module_update),
			.data_out			(osf_data[l]),
			.data_valid_out	(osf_data_valid[l])
			);

		/* osf bank b: draws from adc channel b */
		oversample_filter #(
			.W_IN					(W_ADC),
			.W_OUT				(W_OSF),
			.W_ORT				(W_OSF_ORT))
		ovr_inst_b (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_b),
			.data_valid_in		(cs_data_valid[l+N_ADC/2]),
			.cycle_delay_in	(osf_cycle_delay),
			.log_ovr_in			(osf_log_ovr),
			.activate_in		(osf_activate[l+N_ADC/2]),
			.update_en_in		(osf_update_en[l+N_ADC/2]),
			.update_in			(module_update),
			.data_out			(osf_data[l+N_ADC/2]),
			.data_valid_out	(osf_data_valid[l+N_ADC/2])
			);
	end
endgenerate

/* pid array */
genvar m;
generate
	for ( m = 0; m < N_ADC; m = m + 1 ) begin : pid_array
		pid_core #(
			.W_IN					(W_OSF),
			.W_OUT				(W_PID),
			.COMP_LATENCY		(PID_COMP_LATENCY))
		pid_inst (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(osf_data[m]),
			.data_valid_in		(osf_data_valid[m]),
			.setpoint_in		(pid_setpoint),
			.p_coef_in			(pid_p_coef),
			.i_coef_in			(pid_i_coef),
			.d_coef_in			(pid_d_coef),
			.clear_in			(pid_clear[m]),
			.update_en_in		(pid_update_en[m]),
			.update_in			(module_update),
			.data_out			(pid_data[m]),
			.data_valid_out	(pid_data_valid[m])
			);
	end
endgenerate

/* router */
router #(
	.W_CHAN				(W_PIDV),
	.W_SEL				(W_RTR_SEL),
	.N_IN					(N_ADC),
	.N_OUT				(N_OUT))
rtr (
	.clk_in				(clk50_in),
	.data_in				(rtr_input_bus),
	.src_select_in		(rtr_src_sel),
	.dest_select_in	(rtr_dest_sel),
	.update_in			(module_update),
	.data_out			(rtr_output_bus)
	);

/* OUTPUT CHANNEL MAPPINGS
*	[ 0 						: N_DAC - 1					] - DAC Channels
*	[ N_DAC					: N_DAC + N_DDDS - 1		] - DDS Frequency Channels
*	[ N_DAC + N_DDS		: N_DAC + 2*N_DDS - 1	] - DDS Phase Channels
*	[ N_DAC + 2*N_DDS		: N_OUT						] - DDS Amplitude Channels */

/* dac preprocessor array */
genvar x;
generate
	for ( x = 0; x < N_DAC; x = x + 1 ) begin : dac_opp_array
		output_preprocessor #(
			.W_IN 				(W_PID),
			.W_OUT 				(W_DAC_DATA),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		dac_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[x]),
			.data_valid_in		(rtr_data_valid[x]),
			// DEBUG .output_max_in		(opp_max[W_DAC_DATA-1:0]),
			// DEBUG .output_min_in		(opp_min[W_DAC_DATA-1:0]),
			.output_max_in		(18'd32767),
			.output_min_in		(18'd0),
			.output_init_in	(opp_init[W_DAC_DATA-1:0]),
			.multiplier_in		(8'b1),							// DEBUG: multiplier fixed at 1 for testing
			.lock_en_in			(1'b1),							// DEBUG: dac locks always enabled for testing
			.update_en_in		(opp_update_en[x]),
			.update_in			(module_update),
			.data_out			(opp_dac_data[x]),
			.data_valid_out	(opp_dac_data_valid[x])
			);
	end
endgenerate

///* dac cycle controller */
//cycle_controller #(
//	.W_DATA				(W_DAC_DATA),
//	.W_DAC_INST			(W_DAC_INST),
//	.N_DAC				(N_DAC),
//	.PIPE_LATENCY		(PIPE_LATENCY))
//ccu (
//	.clk_in				(clk50_in),
//	.reset_in			(sys_reset),
//	.bus_data_in		(ccu_input_bus),
//	.data_valid_in		(opp_dac_data_valid),
//	.adc_data_valid_in(adc_data_valid[0]),
//	.dac_done_in		(dac_done),
//	.data_out			(ccu_data),
//	.channel_out		(ccu_channel),
//	.data_valid_out	(ccu_data_valid),
//	.dac_stall_out		(ccu_dac_stall)
//	);
//
///* dac controller */
//dac_controller #(
//	.W_DATA				(W_DAC_DATA),
//	.N_CHAN				(N_DAC))
//dac_cntrl (
//	.clk_in				(clk50_in),
//	.reset_in			(sys_reset),
//	.ref_set_in			(dac_ref_set),
//	.data_in				(ccu_data),
//	.channel_in			(ccu_channel),
//	.data_valid_in		(ccu_data_valid),
//	.stall_in			(ccu_dac_stall),
//	.nldac_out			(dac_nldac_out),
//	.nsync_out			(dac_nsync_out),
//	.sclk_out			(dac_sclk_out),
//	.din_out				(dac_din_out),
//	.nclr_out			(dac_nclr_out),
//	.dac_done_out		(dac_done),
//	.data_out			(dac_data),
//	.channel_out		(dac_chan)
//	);

/* dac controller */
dac_controller #(
	.W_DATA				(W_DAC_DATA),
	.N_CHAN				(N_DAC))
dac_cntrl (
	.clk_in				(clk50_in),
	.reset_in			(sys_reset),
	.ref_set_in			(dac_ref_set),
	.data_in				(opp_dac_data[0]),
	.channel_in			(0),
	.data_valid_in		(opp_dac_data_valid[0]),
	.stall_in			(0),
	.nldac_out			(dac_nldac_out),
	.nsync_out			(dac_nsync_out),
	.sclk_out			(dac_sclk_out),
	.din_out				(dac_din_out),
	.nclr_out			(dac_nclr_out),
	.dac_done_out		(dac_done),
	.data_out			(dac_data),
	.channel_out		(dac_chan)
	);

/* dds preprocessor array */
genvar y;
generate
	for ( y = 0; y < N_DDS; y = y + 1 ) begin : dds_opp_array
		/* index parameters */
		localparam F = N_DAC + y;					// frequency channels index
		localparam P = N_DAC + N_DDS + y;		// phase channels index
		localparam A = N_DAC + 2*N_DDS + y;		// amplitude channels index

		/* frequency output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_PID),
			.W_OUT 				(W_DDS_FREQ),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		freq_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[F]),
			.data_valid_in		(rtr_data_valid[F]),
			.update_en_in		(opp_update_en[F]),
			.update_in			(module_update),
			.lock_en_in			(1'b1),							// frequency lock always enabled for testing
			.output_max_in		(opp_max[W_DDS_FREQ-1:0]),
			.output_min_in		(opp_min[W_DDS_FREQ-1:0]),
			.output_init_in	(opp_init[W_DDS_FREQ-1:0]),
			.multiplier_in		(8'b1),
			.data_out			(opp_freq_data[y]),
			.data_valid_out	(opp_freq_data_valid[y])
			);

		/* phase output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_PID),
			.W_OUT 				(W_DDS_PHASE),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		phase_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[P]),
			.data_valid_in		(rtr_data_valid[P]),
			.update_en_in		(opp_update_en[P]),
			.update_in			(module_update),
			.lock_en_in			(1'b0),							// phase lock always disabled for testing
			.output_max_in		(opp_max[W_DDS_PHASE-1:0]),
			.output_min_in		(opp_min[W_DDS_PHASE-1:0]),
			.output_init_in	(opp_init[W_DDS_PHASE-1:0]),
			.multiplier_in		(8'b1),
			.data_out			(opp_phase_data[y]),
			.data_valid_out	(opp_phase_data_valid[y])
			);

		/* amplitude output preprocessor */
		output_preprocessor #(
			.W_IN 				(W_PID),
			.W_OUT 				(W_DDS_AMP),
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		amp_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[A]),
			.data_valid_in		(rtr_data_valid[A]),
			.update_en_in		(opp_update_en[A]),
			.update_in			(module_update),
			.lock_en_in			(1'b0),							// amplitude lock always disabled for testing
			.output_max_in		(opp_max[W_DDS_AMP-1:0]),
			.output_min_in		(opp_min[W_DDS_AMP-1:0]),
			.output_init_in	(opp_init[W_DDS_AMP-1:0]),
			.multiplier_in		(8'b1),
			.data_out			(opp_amp_data[y]),
			.data_valid_out	(opp_amp_data_valid[y])
			);
	end
endgenerate

/* dds controller array */
genvar z;
generate
	for ( z = 0; z < N_DDS; z = z + 1 ) begin : dds_array
		dds_controller dds_cntrl (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.freq_in				(opp_freq_data[z]),
			.phase_in			(opp_phase_data[z]),
			.amp_in				(opp_amp_data[z]),
			.freq_valid_in		(opp_freq_data_valid[z]),
			.phase_valid_in	(opp_phase_data_valid[z]),
			.amp_valid_in		(opp_amp_data_valid[z]),
			.sclk_out			(dds_sclk_out[z]),
			.reset_out			(dds_reset_out[z]),
			.csb_out				(dds_csb_out[z]),
			.sdio_out			(dds_sdio_out[z]),
			.io_update_out		(dds_io_update_out[z]),
			.dds_done_out		(dds_done[z])
			);
	end
endgenerate


/* frontpanel interface */
frontpanel_interface #(
	.N_ADC					(N_ADC),
	.N_OUT					(N_OUT),
	.W_ADC					(W_ADC),
	.W_OSF_CD				(W_OSF_CD),
	.W_OSF_ORT				(W_OSF_ORT))
fp_io (
	// debug
	.adc_data0_db			(adc_data0_db),
	.clk50_in				(clk50_in),
	.clk17_in				(clk17_in),
	.adc_data_valid_in	(cs_data_valid),
	.adc_data_a_in			(cs_data_a),
	.adc_data_b_in			(cs_data_b),
	.adc_cstart_out		(adc_cstart),
	.adc_os_out				(adc_os),
	.osf_cycle_delay_out	(osf_cycle_delay),
	.osf_log_ovr_out		(osf_log_ovr),
	.osf_activate_out		(osf_activate),
	.osf_update_en_out	(osf_update_en),
	.pid_clear_out			(pid_clear),
	.pid_setpoint_out		(pid_setpoint),
	.pid_p_coef_out		(pid_p_coef),
	.pid_i_coef_out		(pid_i_coef),
	.pid_d_coef_out		(pid_d_coef),
	.pid_update_en_out	(pid_update_en),
	.rtr_src_sel_out		(rtr_src_sel),
	.rtr_dest_sel_out		(rtr_dest_sel),
	.opp_min_out			(opp_min),
	.opp_max_out			(opp_max),
	.opp_init_out			(opp_init),
	.opp_multiplier_out	(opp_multiplier),
	.opp_update_en_out	(opp_update_en),
	.dac_ref_set_out		(dac_ref_set),
	.module_update_out	(module_update),
	.sys_reset_out			(sys_reset),
	.hi_in					(hi_in),
	.hi_out					(hi_out),
	.hi_inout				(hi_inout),
	.hi_aa					(hi_aa),
	.i2c_sda					(i2c_sda),
	.i2c_scl 				(i2c_scl),
	.hi_muxsel				(hi_muxsel)
	);

endmodule
