`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:   18:37:23 02/09/2015
// Design Name:   pipe_tx_fifo
// Module Name:   X:/MIST/pid_controller/pipe_tx_fifo_tf.v
// Project Name:  pid_controller_v2
// Target Device:
// Tool versions:
// Description:
//
// Verilog Test Fixture created by ISE for module: pipe_tx_fifo
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module pipe_tx_fifo_tf;

	// Inputs
	reg ti_clk_in;
	reg sys_clk_in;
	reg reset_in;
	reg data_valid_in;
	reg [15:0] data_in;
	reg pipe_read_in;

	// Outputs
	wire [15:0] data_out;

	// Instantiate the Unit Under Test (UUT)
	pipe_tx_fifo uut (
		.ti_clk_in(ti_clk_in),
		.sys_clk_in(sys_clk_in),
		.reset_in(reset_in),
		.data_valid_in(data_valid_in),
		.data_in(data_in),
		.pipe_read_in(pipe_read_in),
		.data_out(data_out)
	);

	// generate 50MHz system clock
	always #20 sys_clk_in = !sys_clk_in;

	// generate ~48MHz ti clock
	always #21 ti_clk_in = !ti_clk_in;

	initial begin
		// Initialize Inputs
		sys_clk_in = 0;
		ti_clk_in = 0;
		reset_in = 0;
		data_valid_in = 0;
		data_in = 0;
		pipe_read_in = 0;

		// Wait 100 ns for global reset to finish
		#100;

		// assign data valid in to counter
		data_in = 0;

		// fill buffer with 1100 sequential values
		repeat(1100) begin
			@(posedge sys_clk_in) begin
				data_valid_in = 1;
			end
			@(posedge sys_clk_in) begin
				data_valid_in = 0;
				data_in = data_in + 1;
			end
		end

		fork
		// fill buffer with 2000 sequential values
		repeat(2000) begin
			@(posedge sys_clk_in) begin
				data_valid_in = 1;
			end
			@(posedge sys_clk_in) begin
				data_valid_in = 0;
				data_in = data_in + 1;
			end
		end

		begin
			// read 48 characters in first burst
			@(posedge ti_clk_in) pipe_read_in = 1;
			repeat(48) @(posedge ti_clk_in);
			@(posedge ti_clk_in) pipe_read_in = 0;

			// wait to simulate OS delay
			repeat(100) @(posedge ti_clk_in);

			// read 975 characters in second burst
			@(posedge ti_clk_in) pipe_read_in = 1;
			repeat(975) @(posedge ti_clk_in);
			@(posedge ti_clk_in) pipe_read_in = 0;
		end
		join

		$stop;

	end

endmodule

