`timescale 1ns / 1ps

// pid_controller_tf -- mba 2015
// PID controller test fixture.

module pidc_timing_tf;
	// Endpoint map
	`include "ep_map.vh"
	`include "parameters.vh"
	`include "sim_tasks.v"
	`include "verification_tasks.v"

	// Parameters
	localparam	W_DATA	= 18;
	localparam 	N_CHAN	= 8;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_DATA*N_CHAN/2;

	// Simulation structures
	wire signed [W_DATA-1:0]	chan[0:N_CHAN-1];
	reg [TX_LEN-1:0] 	data_a_tx;
	reg [TX_LEN-1:0] 	data_b_tx;
	reg [15:0] wire_out;

	// Inputs
	reg clk50_in;
	reg clk17_in;
	reg adc_busy_in;
	wire adc_data_a_in;
	wire adc_data_b_in;
	reg adc_cstart_tf_in;

	// Outputs
	wire [2:0] adc_os_out;
	wire adc_convst_out;
	wire adc_reset_out;
	wire adc_sclk_out;
	wire adc_n_cs_out;
	wire dac_nldac_out;
	wire dac_nsync_out;
	wire dac_sclk_out;
	wire dac_din_out;
	wire dac_nclr_out;
	wire [1:0] dds_sclk_out;
	wire [1:0] dds_reset_out;
	wire [1:0] dds_csb_out;
	wire [1:0] dds_sdio_out;
	wire [1:0] dds_io_update_out;
	wire out_buf_en;

	// Frontpanel
	reg [7:0] hi_in;
	wire [1:0] hi_out;
	wire [15:0] hi_inout;
	wire hi_aa;

	// ADC params
	localparam	ADC_OS_INIT		= 1;

	// OSF params
	localparam	OSF_ACTIVATE	= 1;
	localparam	OSF_OSM_INIT	= 0;
	localparam	OSF_CDLY_INIT	= 0;

	// PID params
	localparam	PID_LOCK_EN		= 1;
	localparam	PID_SETP_INIT	= 0;
	localparam	PID_PCF_INIT	= 10;
	localparam	PID_ICF_INIT	= 3;
	localparam	PID_DCF_INIT	= 0;

	// RTR params
	localparam	RTR_ACTV_INIT	= 1;

	// OPP params
	localparam	DAC_MAX_INIT	= 52428;
	localparam	DAC_MIN_INIT	= 13107;
	localparam	DAC_OUT_INIT	= 26214;
	localparam	DAC_MLT_INIT	= 1;
	localparam	DAC_RS_INIT		= 0;

	// Instantiate the Unit Under Test (UUT)
	pid_controller #(
		.ADC_OS_INIT	(ADC_OS_INIT),
		.OSF_ACTIVATE	(OSF_ACTIVATE),
		.OSF_OSM_INIT	(OSF_OSM_INIT),
		.OSF_CDLY_INIT	(OSF_CDLY_INIT),
		.PID_LOCK_EN	(PID_LOCK_EN),
		.PID_SETP_INIT (PID_SETP_INIT),
		.PID_PCF_INIT	(PID_PCF_INIT),
		.PID_ICF_INIT	(PID_ICF_INIT),
		.PID_DCF_INIT	(PID_DCF_INIT),
		.RTR_ACTV_INIT	(RTR_ACTV_INIT),
		.DAC_MAX_INIT	(DAC_MAX_INIT),
		.DAC_MIN_INIT	(DAC_MIN_INIT),
		.DAC_OUT_INIT	(DAC_OUT_INIT),
		.DAC_MLT_INIT	(DAC_MLT_INIT),
		.DAC_RS_INIT	(DAC_RS_INIT))
	uut (
		.clk50_in(clk50_in),
		.clk17_in(clk17_in),
		.adc_busy_in(adc_busy_in),
		.adc_data_a_in(adc_data_a_in),
		.adc_data_b_in(adc_data_b_in),
		.adc_os_out(adc_os_out),
		.adc_convst_out(adc_convst_out),
		.adc_reset_out(adc_reset_out),
		.adc_sclk_out(adc_sclk_out),
		.adc_n_cs_out(adc_n_cs_out),
		.dac_nldac_out(dac_nldac_out),
		.dac_nsync_out(dac_nsync_out),
		.dac_sclk_out(dac_sclk_out),
		.dac_din_out(dac_din_out),
		.dac_nclr_out(dac_nclr_out),
		.dds_sclk_out(dds_sclk_out),
		.dds_reset_out(dds_reset_out),
		.dds_csb_out(dds_csb_out),
		.dds_sdio_out(dds_sdio_out),
		.dds_io_update_out(dds_io_update_out),
		.n_out_buf_en(out_buf_en),
		.hi_in(hi_in),
		.hi_out(hi_out),
		.hi_inout(hi_inout),
		.hi_aa(hi_aa),
		.adc_cstart_tf_in(adc_cstart_tf_in)
	);

	// generate ~17MHz clock
	always #30 clk17_in = ~clk17_in;

	// generate 50MHz clock
	always #10 clk50_in = ~clk50_in;

	// serial data channels
	assign adc_data_a_in = data_a_tx[TX_LEN-1];
	assign adc_data_b_in = data_b_tx[TX_LEN-1];

	// misc params
	localparam REPS = 100;
	parameter pipeOutSize = 2048;
	reg [7:0] pipeOut = 0;

	// pid parameters
	reg signed [15:0] setpoint = PID_SETP_INIT;
	reg signed [15:0]	p_coef = PID_PCF_INIT;
	reg signed [15:0]	i_coef = PID_ICF_INIT;
	reg signed [15:0]	d_coef = PID_DCF_INIT;

	// opp parameters
	reg signed [47:0] output_init = DAC_OUT_INIT;
	reg signed [47:0] output_min = DAC_MIN_INIT;
	reg signed [47:0] output_max = DAC_MAX_INIT;
	reg signed [15:0] multiplier = DAC_MLT_INIT;
	reg [15:0] right_shift = DAC_RS_INIT;

	// pid verification
	reg signed [63:0] pid_data_reg = 0;
	reg signed [63:0] error = 0;
	reg signed [63:0]	error_prev = 0;
	reg signed [63:0]	integral = 0;
	reg signed [63:0]	derivative = 0;
	reg signed [63:0]	pid_expected = 0;
	reg signed [63:0]	e_count = 0;
	reg [15:0] target = 0;
	reg lock_en = | PID_LOCK_EN;
	integer i;

	// output verification
	reg [15:0] dac_data_reg = 0;

	// pipe verification
	reg signed [15:0] pipeOutWord;
	reg signed [15:0] wireOutValue;
	reg signed [15:0] pipe_expected[REPS-1:0];
	integer rep_count = 0;

	// output verification
	reg signed [127:0] proc_stage[5:0];

	// dac received data verification
	reg [31:0] r_instr = 0;
	wire [15:0] r_data;
	wire [3:0] r_prefix, r_control, r_address, r_feature;
	assign {r_prefix, r_control, r_address, r_data, r_feature} = r_instr;

	// adc channel assignments
	reg signed [15:0] chan_1_reg = 0;
	//assign chan[0] = r_data - target;
	//assign chan[0] = 655;
	assign chan[0] = chan_1_reg;
	assign chan[1] = 0;
	assign chan[2] = 0;
	assign chan[3] = 0;
	assign chan[4] = 0;
	assign chan[5] = 0;
	assign chan[6] = 0;
	assign chan[7] = 0;

	initial begin : main
		// Initialize Inputs
		clk50_in = 0;
		clk17_in = 0;
		adc_busy_in = 0;
		data_a_tx = 0;
		data_b_tx = 0;
		wire_out = 0;
		adc_cstart_tf_in = 0;

		// chill for a bit
		#200;

		// manual trigger adc conversion start
		@(posedge clk17_in) adc_cstart_tf_in = 1'b1;
		@(posedge clk17_in) adc_cstart_tf_in = 1'b0;


		fork
			adc_transmit(REPS);
			check_rcv(REPS);
		join

		$stop;

	end

endmodule

