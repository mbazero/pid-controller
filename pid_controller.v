`timescale 1ns / 1ps

// two_dds_test -- mba 2014

// TODO
// - consider adding full speed 50MHz DAC/DDS serial clock
// - compartmentalize PID pipeline for easy swapping of ADC/DAC controllers
// - maybe add functionality to support simultenous update of all dac channels (nLDAC pin control)
// - consistency with frontpanel param updating (some modules update on posedge of update signal, some on posedge of clock)
// - consistency with frontpanel param reseting (
// - should probably change everything to asynch reset b/c sys reset signal is a trigger (only one clock cycle)
// - make sure output preprocessor multiplier is delivered in signed fashion
// - figure out lock en signal (is it really needed, if so, make it a frontpanel param)
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
	// config parameters
	parameter N_ADC			= 8,	// number of adc channels
	parameter N_DAC			= 8,	// number of dac channels
	parameter N_DDS			= 0,	// number of dds channels
	parameter W_ADC			= 18, // width of adc channels
	parameter W_OSF			= 18, // width of oversample filter output
	parameter W_PID			= 18, // width of pid core output
	parameter W_DAC_INST		= 32, // width of dac update instruction
	parameter W_DDS_FREQ		= 48, // width of dds frequency word
	parameter W_DDS_PHASE	= 14,	// width of dds phase word
	parameter W_DDS_AMP 		= 10, // width of dds amplitude instruction
	parameter W_DAC_DATA		= 16,	// width of dac data input
	parameter W_DAC_CHS		= 3,	// width of dac channel input
	parameter T_ADC_CYCLE	= 85,	// adc conversion cycle time in number of adc clock cycles

	// initial values
	parameter ADC_OS_INIT	= 1,
	parameter OSF_OSM_INIT	= 0,
	parameter OSF_CDLY_INIT	= 0,
	parameter PID_SETP_INIT = 0,
	parameter PID_PCF_INIT	= 10,
	parameter PID_ICF_INIT	= 3,
	parameter PID_DCF_INIT	= 0,
	parameter RTR_ACTV_INIT	= 1,
	parameter OPP_OMAX_INIT = 9999,
	parameter OPP_OMIN_INIT = 1111,
	parameter OPP_OIN_INIT	= 5000,
	parameter OPP_MLT_INIT	= 1

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

	// inputs <- test fixture
	input wire							adc_cstart_tf_in,

	// DEBUG
	output wire adc_dv_out,
	output wire cs_dv_out,
	output wire osf_dv_out,
	output wire pid_dv_out,
	output wire [W_PID-1:0] pid_data_out,
	output wire opp_dac_dv_out,
	output wire [W_DAC_DATA-1:0] opp_dac_data_out,
	output wire diq_dv_out
	);

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam N_OUT 					= N_DAC + 3*N_DDS;	// total number of output channels; each dds has three output channels (phase, freq, and amp)
localparam W_PIDV 				= W_PID + 2;			// width of pid data bus with data valid and lock enable signals
localparam W_RTR_SEL 			= 4;						// width of router select signal (must be log2(N_DAC) + 1...MSB stores channel activation state)
localparam PIPE_LATENCY 		= 5;						// latency in clock cycles of pipeline
localparam W_OSF_OSM				= 6;						// width of oversample mode signal
localparam W_OSF_CD				= 16;						// width of osf cycle delay signal
localparam PID_COMP_LATENCY	= 1;						// pid core computation latency
localparam OPP_COMP_LATENCY	= 1;						// output preprocessor compuation latency

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

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
wire	[N_ADC-1:0]				osf_activate_dbg = 1; // DEBUG
wire	[N_ADC-1:0]				osf_update_en;
wire	[W_OSF_CD-1:0]			osf_cycle_delay;
wire	[W_OSF_OSM-1:0]		osf_osm;
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
wire	[N_ADC-1:0]				pid_lock_en;

/* router */
wire	[W_RTR_SEL-1:0]		rtr_src_sel;
wire	[W_RTR_SEL-1:0]		rtr_dest_sel;
wire	[N_OUT-1:0]				rtr_output_active;
wire	[W_PIDV*N_ADC-1:0]	rtr_input_packed;
wire	[W_PIDV*N_OUT-1:0]	rtr_output_packed;
wire	[W_PID-1:0]				rtr_data[0:N_OUT-1];
wire	[N_OUT-1:0]				rtr_data_valid;
wire	[N_OUT-1:0]				rtr_lock_en;
wire	[N_OUT-1:0]				rtr_lock_en_dbg = 1; // DEBUG

/* output preprocessor */
wire	[N_OUT-1:0]				opp_update_en;
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

// DEBUG
assign adc_dv_out = adc_data_valid[0];
assign cs_dv_out = cs_data_valid[0];
assign osf_dv_out = osf_data_valid[0];
assign pid_dv_out = pid_data_valid[0];
assign pid_data_out = pid_data[0];
assign opp_dac_dv_out = opp_dac_data_valid[0];
assign opp_dac_data_out = opp_dac_data[0];
assign diq_dv_out = diq_data_valid;

/* output buffer enable */
assign n_out_buf_en = 1'b0;

/* pack pid data and valid signals to single data vector for presentation to router */
genvar i;
generate
	for ( i = 0; i < N_ADC; i = i + 1 ) begin : rtr_in_arr
		assign rtr_input_packed[ i*W_PIDV +: W_PIDV ] = {pid_lock_en[i], pid_data_valid[i], pid_data[i]};
	end
endgenerate

/* split router output data vector to seperate channels */
genvar j;
generate
	for ( j = 0; j < N_OUT; j = j + 1 ) begin : rtr_out_arr
		assign rtr_data[j] 			= rtr_output_packed[ j*W_PIDV +: W_PID ];
		assign rtr_data_valid[j]	= rtr_output_packed[ j*W_PIDV + W_PID ];
		assign rtr_lock_en[j]		= rtr_output_packed[ j*W_PIDV + W_PID + 1];
	end
endgenerate

/* pack dac output channels to single data vector for presentation to dac instruction queue */
genvar k;
generate
	for ( k = 0; k < N_DAC; k = k + 1 ) begin : diq_in_arr
		assign diq_input_packed[ k*W_DAC_DATA +: W_DAC_DATA ] = opp_dac_data[k];
	end
endgenerate

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc controller */
adc_controller #(
	.W_OUT				(W_ADC),
	.N_CHAN				(N_ADC),
	.MIN_T_CYCLE		(T_ADC_CYCLE),
	.OS_INIT				(ADC_OS_INIT))
adc_cont (
	.clk_in				(clk17_in),
	.reset_in			(sys_reset),
	.busy_in				(adc_busy_in),
	.data_a_in			(adc_data_a_in),
	.data_b_in			(adc_data_b_in),
	.os_in				(adc_os),
	.update_in			(module_update),
	.cstart_in			(adc_cstart | adc_cstart_tf_in),
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
			.W_OSM				(W_OSF_OSM),
			.OSM_INIT			(OSF_OSM_INIT),
			.CDLY_INIT			(OSF_CDLY_INIT))
		ovr_inst_a (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_a),
			.data_valid_in		(cs_data_valid[l]),
			.cycle_delay_in	(osf_cycle_delay),
			.osm_in				(osf_osm),
			.activate_in		(osf_activate[l] | osf_activate_dbg[l]),
			.update_en_in		(osf_update_en[l]),
			.update_in			(module_update),
			.data_out			(osf_data[l]),
			.data_valid_out	(osf_data_valid[l])
			);

		/* osf bank b: draws from adc channel b */
		oversample_filter #(
			.W_IN					(W_ADC),
			.W_OUT				(W_OSF),
			.W_OSM				(W_OSF_OSM))
		ovr_inst_b (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(cs_data_b),
			.data_valid_in		(cs_data_valid[l+N_ADC/2]),
			.cycle_delay_in	(osf_cycle_delay),
			.osm_in				(osf_osm),
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
			.W_COMP				(W_COMP),
			.COMP_LATENCY		(PID_COMP_LATENCY),
			.SETPOINT_INIT		(PID_SETP_INIT),
			.P_COEF_INIT		(PID_PCF_INIT),
			.D_COEF_INIT		(PID_DCF_INIT),
			.I_COEF_INIT		(PID_ICF_INIT))
		pid_inst (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(osf_data[m]),
			.data_valid_in		(osf_data_valid[m]),
			.setpoint_in		(pid_setpoint),
			.p_coef_in			(pid_p_coef),
			.i_coef_in			(pid_i_coef),
			.d_coef_in			(pid_d_coef),
			.lock_en_in			(pid_lock_en[m]),
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
	.data_packed_in	(rtr_input_packed),
	.src_select_in		(rtr_src_sel),
	.output_active_in	(rtr_output_active),
	.dest_select_in	(rtr_dest_sel),
	.update_in			(module_update),
	.data_packed_out	(rtr_output_packed)
	);

/* OUTPUT CHANNEL MAPPINGS
*	[ 0 						: N_DAC - 1					] - DAC Channels
*	[ N_DAC					: N_DAC + N_DDS - 1		] - DDS Frequency Channels
*	[ N_DAC + N_DDS		: N_DAC + 2*N_DDS - 1	] - DDS Phase Channels
*	[ N_DAC + 2*N_DDS		: N_OUT						] - DDS Amplitude Channels */

/* dac preprocessor array */
genvar x;
generate
	for ( x = 0; x < N_DAC; x = x + 1 ) begin : dac_opp_array
		output_preprocessor #(
			.W_IN 				(W_PID),
			.W_OUT 				(W_DAC_DATA), // data truncation happens in opp
			.COMP_LATENCY		(OPP_COMP_LATENCY))
		dac_opp (
			.clk_in				(clk50_in),
			.reset_in			(sys_reset),
			.data_in				(rtr_data[x]),
			.data_valid_in		(rtr_data_valid[x]),
			.output_max_in		(opp_max[W_DAC_DATA-1:0]),
			.output_min_in		(opp_min[W_DAC_DATA-1:0]),
			.output_init_in	(opp_init[W_DAC_DATA-1:0]),
			.multiplier_in		(opp_multiplier),
			.lock_en_in			(rtr_lock_en[x] | rtr_lock_en_dbg[x]),
			.update_en_in		(opp_update_en[x]),
			.update_in			(module_update),
			.data_out			(opp_dac_data[x]),
			.data_valid_out	(opp_dac_data_valid[x])
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
			.lock_en_in			(1'b1), //DEBUG
			.output_max_in		(opp_max[W_DDS_FREQ-1:0]),
			.output_min_in		(opp_min[W_DDS_FREQ-1:0]),
			.output_init_in	(opp_init[W_DDS_FREQ-1:0]),
			.multiplier_in		(8'b1), //DEBUG
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
			.lock_en_in			(1'b0), //DEBUG
			.output_max_in		(opp_max[W_DDS_PHASE-1:0]),
			.output_min_in		(opp_min[W_DDS_PHASE-1:0]),
			.output_init_in	(opp_init[W_DDS_PHASE-1:0]),
			.multiplier_in		(8'b1), //DEBUG
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
			.lock_en_in			(1'b0), //DEBUG
			.output_max_in		(opp_max[W_DDS_AMP-1:0]),
			.output_min_in		(opp_min[W_DDS_AMP-1:0]),
			.output_init_in	(opp_init[W_DDS_AMP-1:0]),
			.multiplier_in		(8'b1), //DEBUG
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
	.W_OSF_OSM				(W_OSF_OSM),
	.N_DAC					(N_DAC),
	.W_DAC					(W_DAC_DATA))
fp_io (
	.clk50_in				(clk50_in),
	.clk17_in				(clk17_in),
	.adc_data_valid_in	(cs_data_valid),
	.adc_data_a_in			(cs_data_a),
	.adc_data_b_in			(cs_data_b),
	.adc_cstart_out		(adc_cstart),
	.adc_os_out				(adc_os),
	.opp_dac_data_valid_in (opp_dac_data_valid),
	.opp_dac_data0_in		(opp_dac_data[0]),
	.diq_data_valid_in	(diq_data_valid),
	.diq_data_in			(diq_data),
	.osf_cycle_delay_out	(osf_cycle_delay),
	.osf_osm_out			(osf_osm),
	.osf_activate_out		(osf_activate),
	.osf_update_en_out	(osf_update_en),
	.pid_lock_en_out		(pid_lock_en),
	.pid_clear_out			(pid_clear),
	.pid_setpoint_out		(pid_setpoint),
	.pid_p_coef_out		(pid_p_coef),
	.pid_i_coef_out		(pid_i_coef),
	.pid_d_coef_out		(pid_d_coef),
	.pid_update_en_out	(pid_update_en),
	.rtr_src_sel_out		(rtr_src_sel),
	.rtr_dest_sel_out		(rtr_dest_sel),
	.rtr_output_active_out(rtr_output_active),
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
