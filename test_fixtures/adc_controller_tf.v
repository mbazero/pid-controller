`timescale 1ns / 1ps

// adc_controller -- mba 2014

module adc_controller_tf;
	// Parameters
	localparam	W_DATA	= 18;
	localparam 	N_CHAN	= 6;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_DATA*N_CHAN/2;

	// Simulation structures
	reg [W_DATA-1:0]	chan[0:N_CHAN-1];
	reg [TX_LEN-1:0] 	data_a_tx;
	reg [TX_LEN-1:0] 	data_b_tx;

	// Inputs
	reg clk_in;
	reg reset_in;
	reg busy_in;
	wire data_a_in;
	wire data_b_in;
	reg [2:0] os_in;
	reg update_in;
	reg cstart_in;

	// Outputs
	wire [2:0] os_out;
	wire convst_out;
	wire reset_out;
	wire sclk_out;
	wire n_cs_out;
	wire [5:0] data_valid_out;
	wire [17:0] data_a_out;
	wire [17:0] data_b_out;

	// Instantiate the Unit Under Test (UUT)
	adc_controller uut (
		.clk_in(clk_in),
		.reset_in(reset_in),
		.busy_in(busy_in),
		.data_a_in(data_a_in),
		.data_b_in(data_b_in),
		.os_in(os_in),
		.update_in(update_in),
		.cstart_in(cstart_in),
		.os_out(os_out),
		.convst_out(convst_out),
		.reset_out(reset_out),
		.sclk_out(sclk_out),
		.n_cs_out(n_cs_out),
		.data_valid_out(data_valid_out),
		.data_a_out(data_a_out),
		.data_b_out(data_b_out)
	);

	// generate ~17MHz clock
	always #30 clk_in = !clk_in;

	// serial data channels
	assign data_a_in = data_a_tx[TX_LEN-1];
	assign data_b_in = data_b_tx[TX_LEN-1];

	initial begin
		// Initialize Inputs
		data_a_tx = 0;
		data_b_tx = 0;
		clk_in = 0;
		reset_in = 1;
		busy_in = 0;
		os_in = 0;
		update_in = 0;
		cstart_in = 0;

		chan[0] = 1111;
		chan[1] = 2222;
		chan[2] = 3333;
		chan[3] = 4444;
		chan[4] = 5555;
		chan[5] = 6666;

		// Wait 200 ns for global reset to finish
		#200;
		@(posedge clk_in) reset_in = 0;

		// pulse cstart
		@(posedge clk_in) cstart_in = 1;
		@(posedge clk_in) cstart_in = 0;

		repeat(4) begin
			// wait for convst_out to pulse and then assert busy
			@(posedge convst_out) begin
				@(posedge clk_in) busy_in = 1;
			end

			// simulate serial transmission from adc to fpga
			@(negedge n_cs_out) begin
				data_a_tx = {chan[0], chan[1], chan[2]};
				data_b_tx = {chan[3], chan[4], chan[5]};
			end

			// wait one cycle before transmittin
			@(posedge clk_in);

			// simulate serial data transmission
			repeat (53) begin
				@(negedge clk_in)
					data_a_tx = data_a_tx << 1;
					data_b_tx = data_b_tx << 1;
			end

			// simulate conversion end
			#200;
			@(posedge clk_in) busy_in = 0;
		end

	end

endmodule

