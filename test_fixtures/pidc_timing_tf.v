`timescale 1ns / 1ps

// pid_controller_tf -- mba 2015
// PID controller test fixture.

module pid_controller_tf;
	// Endpoint map
	`include "ep_map.vh"
	`include "parameters.vh"

	// Parameters
	localparam	W_DATA	= 18;
	localparam 	N_CHAN	= 8;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_DATA*N_CHAN/2;

	// Simulation structures
	reg signed [W_DATA-1:0]	chan[0:N_CHAN-1];
	reg [TX_LEN-1:0] 	data_a_tx;
	reg [TX_LEN-1:0] 	data_b_tx;
	reg [15:0] wire_out;

	// Inputs
	reg clk50_in;
	reg clk17_in;
	reg adc_busy_in;
	wire adc_data_a_in;
	wire adc_data_b_in;
	wire adc_cstart_tf_in;

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

	//DEBUG
	wire signed [17:0] pid_data;
	reg signed [17:0] pid_data_reg;
	wire pid_dv;
	wire [15:0] opp_data;
	wire opp_dv;

	// Instantiate the Unit Under Test (UUT)
	pid_controller uut (
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
		.adc_cstart_tf_in(adc_cstart_tf_in),
		//DEBUG
		.pid_data_out(pid_data),
		.pid_dv_out(pid_dv),
		.opp_data_out(opp_data),
		.opp_dv_out(opp_dv)
	);

	// generate ~17MHz clock
	//always #30 clk17_in = ~clk17_in;

	// generate 50MHz clock
	always #10 clk50_in = ~clk50_in;

	// serial data channels
	assign adc_data_a_in = data_a_tx[TX_LEN-1];
	assign adc_data_b_in = data_b_tx[TX_LEN-1];

	// set channel values
	initial begin
		chan[0] = -22222;
		//chan[1] = 2222;
		//chan[2] = 3333;
		//chan[3] = 4444;
		//chan[4] = 5555;
		//chan[5] = 6666;
		//chan[6] = 7777;
		//chan[7] = 8888;
	end

	reg signed [15:0] setpoint = 0, p_coef = 0, i_coef = 0, d_coef = 0;
	integer error = 0, error_prev = 0, integral = 0, derivative = 0, u_expected = 0, e_count = 0;
	integer i;
	reg [15:0] pipeOutWord;
	reg signed [15:0] wireOutValue;

	// dac received data
	reg [31:0] r_instr;
	wire [15:0] r_data;
	wire [3:0] r_prefix, r_control, r_address, r_feature;
	assign {r_prefix, r_control, r_address, r_data, r_feature} = r_instr;

	// simulation params
	localparam REPS = 10;

	initial begin : main
		// Initialize Inputs
		clk50_in = 0;
		clk17_in = 0;
		adc_busy_in = 0;
		data_a_tx = 0;
		data_b_tx = 0;
		wire_out = 0;
		adc_cstart_tf_in = 0;

		// chill out for a bit
		repeat(4) begin
			@(posedge clk17_in) #1;
		end

		// manual trigger adc conversion start
		@(posedge clk17_in) adc_cstart_tf_in = 1'b1;
		@(posedge clk17_in) adc_cstart_tf_in = 1'b0;

		fork : sim

			// adc data transmission simulation
			repeat(REPS) begin
				// wait for convst_out to pulse and then assert busy
				@(posedge adc_convst_out) begin
					@(posedge clk17_in) adc_busy_in = 1;
				end

				// set random chan[0] value
				//chan[0] = $random % 100;

				// initialize data to be transmitted
				@(negedge adc_n_cs_out) begin
					data_a_tx = {chan[0], chan[1], chan[2], chan[3]};
					data_b_tx = {chan[4], chan[5], chan[6], chan[7]};
				end

				// wait one cycle before transmitting
				@(posedge clk17_in);

				// simulate serial data transmission from adc to fpga
				repeat (71) begin
					@(negedge clk17_in)
					data_a_tx = data_a_tx << 1;
					data_b_tx = data_b_tx << 1;
				end

				// simulate conversion end
				#200;
				@(posedge clk17_in) adc_busy_in = 0;

			end

			// check pid value
			repeat(REPS) begin
				@(posedge pid_dv) begin
					pid_data_reg = pid_data;
					e_count = e_count + 1;
					error = setpoint - chan[0];
					#1;
					integral = integral + error;
					derivative = error - error_prev;
					#1;
					u_expected = (p_coef * error) + (i_coef * integral) + (d_coef * derivative);
					error_prev = error;
					#1;
					if(u_expected == pid_data_reg) begin
						$display("PID Success\t(%d, %d)\t--\tExpected: %d\tReceived: %d", error, integral, u_expected, pid_data_reg);
					end else begin
						$display("PID Failure\t(%d, %d)\t--\tExpected: %d\tReceived: %d", error, integral, u_expected, pid_data_reg);
					end
				end
			end

			// simulate received dac data
			repeat(REPS) begin
				@(negedge dac_nsync_out) begin
					repeat(32) begin
						@(negedge dac_sclk_out) begin
							r_instr = {r_instr[30:0], dac_din_out}; // shift data in
						end
					end

				end
			end

		join

		$stop;

	end

endmodule
