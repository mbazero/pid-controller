`timescale 1ns / 1ps

// dds_controller_tf -- mba 2015

module dds_controller_tf;

	// Inputs
	reg clk_in;
	reg reset_in;

	reg [47:0] freq_in;
	reg [13:0] phase_in;
	reg [9:0] amp_in;
	reg freq_dv_in;
	reg phase_dv_in;
	reg amp_dv_in;

	// Outputs
	wire sclk_out;
	wire reset_out;
	wire csb_out;
	wire sdio_out;
	wire io_update_out;

	wire dds_done_out;

	dds_controller uut (
		.clk_in(clk_in),
		.reset_in(reset_in),
		.freq_in(freq_in),
		.phase_in(phase_in),
		.amp_in(amp_in),
		.freq_dv_in(freq_dv_in),
		.phase_dv_in(phase_dv_in),
		.amp_dv_in(amp_dv_in),
		.sclk_out(sclk_out),
		.reset_out(reset_out),
		.csb_out(csb_out),
		.sdio_out(sdio_out),
		.io_update_out(io_update_out),
		.dds_done_out(dds_done_out)
	);

	// simulation structures
	reg freq_dv_store = 0;
	reg phase_dv_store = 0;
	reg amp_dv_store = 0;
	reg [63:0] r_data = 0;
	reg [47:0] r_freq = 0;
	reg [13:0] r_phase = 0;
	reg [9:0] r_amp = 0;
	integer dv_count = 0;
	integer i = 0;

	// generate clock
	always #10 clk_in = ~clk_in;

	initial begin
		clk_in = 0;
		reset_in = 0;
		freq_in = 0;
		phase_in = 0;
		amp_in = 0;
		freq_dv_in = 0;
		phase_dv_in = 0;
		amp_dv_in = 0;

		#200;

		repeat(100) begin
			// generate freq, phase, and amp values
			@(posedge clk_in) begin
				freq_in = $random;
				phase_in = $random;
				amp_in = $random;
			end

			// assert random data valids
			@(posedge clk_in) begin
				freq_dv_in = $random % 2;
				phase_dv_in = $random % 2;
				amp_dv_in = $random % 2;
				#1;
				freq_dv_store = freq_dv_in;
				phase_dv_store = phase_dv_in;
				amp_dv_store = amp_dv_in;
				dv_count = freq_dv_in + phase_dv_in + amp_dv_in;
			end

			// display test info
			$display("=========================");
			$write("Testing -- ");
			if(freq_dv_in) $write("Freq ");
			if(phase_dv_in) $write("Phase ");
			if(amp_dv_in) $write("Amp ");
			$write("\n");
			$display("=========================");

			// clear data valids
			@(posedge clk_in) begin
				freq_dv_in = 0;
				phase_dv_in = 0;
				amp_dv_in = 0;
			end

			// read and verify dds transmission
			for(i = 0; i < dv_count; i = i+1) begin
				@(negedge csb_out) #1.34;

				// read transmission
				while(csb_out == 0) begin
					@(posedge sclk_out) begin
						r_data = {r_data[62:0], sdio_out};
					end
				end

				@(posedge io_update_out);

				// verify transmission
				if(freq_dv_store == 1) begin
					r_freq = r_data[63:0];
					assert_equals(freq_in, r_freq, "Frequency");
					freq_dv_store = 0;
				end else if (phase_dv_store == 1) begin
					r_phase = r_data[13:0];
					assert_equals(phase_in, r_phase, "Phase");
					phase_dv_store = 0;
				end else if (amp_dv_store == 1) begin
					r_amp = r_data[9:0];
					assert_equals(amp_in, r_amp, "Amp");
					amp_dv_store = 0;
				end
			end

			// clear simulation state
			dv_count = 0;
		end

		$display("Simulation Successful.");
		$stop;

	end

    `include "assert_equals.v"

endmodule
