`timescale 1ns / 1ps

// dac_isntr_queue_tf

module dac_instr_queue_tf;

	// Parameters
	localparam W_DATA = 16;
	localparam W_CHS = 3;
	localparam N_CHAN = 8;

	// Inputs
	reg clk_in;
	reg reset_in;

	reg [W_DATA-1:0] data_in[N_CHAN-1:0];
	wire [W_DATA*N_CHAN-1:0] data_packed_in;
	reg [N_CHAN-1:0] data_valid_in;
	reg rd_ack_in;

	// Outputs
	wire [W_DATA-1:0] data_out;
	wire [W_CHS-1:0] chan_out;
	wire data_valid_out;

	dac_instr_queue #(
		.W_DATA(W_DATA),
		.W_CHS(W_CHS),
		.N_CHAN(N_CHAN))
	diq (
		.clk_in(clk_in),
		.reset_in(reset_in),
		.data_packed_in(data_packed_in),
		.data_valid_in(data_valid_in),
		.rd_ack_in(rd_ack_in),
		.data_out(data_out),
		.chan_out(chan_out),
		.data_valid_out(data_valid_out)
	);

	// simulation structures
	integer i = 0;
	integer valid_count = 0;
	reg [N_CHAN-1:0] data_valid_store = 0;

	// pack input data
	genvar k;
	generate
		for ( k = 0; k < N_CHAN; k = k + 1 ) begin : diq_in_arr
			assign data_packed_in[ k*W_DATA +: W_DATA ] = data_in[k];
		end
	endgenerate

	// generate clock
	always #10 clk_in = ~clk_in;

	initial begin
		clk_in = 0;
		reset_in = 0;
		data_valid_in = 0;
		rd_ack_in = 0;

		#200 @(posedge clk_in);

		repeat(100) begin
			// generate random data
			for ( i = 0; i < N_CHAN; i = i + 1 )  begin
				data_in[i] = $random;
			end

			@(posedge clk_in);

			// generate random data valids
			for ( i = 0; i < N_CHAN; i = i + 1 )  begin
				data_valid_in[i] = $random % 2;
			end

			// store data valid signals
			#1 data_valid_store = data_valid_in;
			for ( i = 0; i < N_CHAN; i = i + 1 )  begin
				valid_count = valid_count + data_valid_in[i];
			end
			@(posedge clk_in);

			// clear data valids
			for ( i = 0; i < N_CHAN; i = i + 1 )  begin
				data_valid_in[i] = 0;
			end

			@(posedge data_valid_out);

			// check data received
			for ( i = 0; i < valid_count; i = i + 1 ) begin
				@(posedge clk_in);
				if (data_valid_out == 0) begin
					$display("Expecting valid data!");
					$stop;
				end else begin
					if (data_valid_store[chan_out] != 1) begin
						$display("Got data from non-active channel!");
						$stop;
					end
					assert_equals(data_in[chan_out], data_out, "Data");

					#320;

					@(posedge clk_in) rd_ack_in = 1;
					@(posedge clk_in) rd_ack_in = 0;
				end
			end

			// clear sim structures
			valid_count = 0;
		end

		$display("Simulation Successful.");
		$stop;
	end

	task assert_equals;
		input [127:0] expected;
		input [127:0] received;
		input [20*8-1:0] test_name;

		begin

			$display("%s Test:", test_name);
			$display("Expected: %d", $signed(expected));
			$display("Received: %d", $signed(received));

			if(expected == received) begin
				$display("Success");
			end else begin
				$display("Failure");
				$stop;
			end
		end
	endtask

endmodule
