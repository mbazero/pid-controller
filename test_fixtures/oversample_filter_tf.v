`timescale 1ns / 1ps

// oversample_filter_tf

module oversample_filter_tf;

	// Parameters
	localparam W_DATA = 18;
	localparam W_EP = 16;
	localparam W_OSM = 6;
	localparam OSM_INIT = 0;
	localparam CDLY_INIT = 0;

	// Inputs
	reg clk_in;
	reg reset_in;

	reg signed [W_DATA-1:0] data_in;
	reg data_valid_in;

	reg [W_EP-1:0] cycle_delay_in;
	reg [W_OSM-1:0] osm_in;
	reg activate_in;
	reg update_en_in;
	reg update_in;

	// Outputs
	wire signed [W_DATA-1:0] data_out;
	wire data_valid_out;

	oversample_filter #(
		.W_DATA(W_DATA),
		.W_EP(W_EP),
		.W_OSM(W_OSM),
		.OSM_INIT(OSM_INIT),
		.CDLY_INIT(CDLY_INIT))
	uut (
		.clk_in(clk_in),
		.reset_in(reset_in),
		.data_in(data_in),
		.data_valid_in(data_valid_in),
		.cycle_delay_in(cycle_delay_in),
		.osm_in(osm_in),
		.activate_in(activate_in),
		.update_en_in(update_en_in),
		.update_in(update_in),
		.data_out(data_out),
		.data_valid_out(data_valid_out)
	);

	// simulation structures
	integer num_samples;
	reg signed [127:0] sum = 0;
	reg signed [W_DATA-1:0] received;
	reg signed [W_DATA-1:0] expected;

	// generate 50MHz clock
	always #10 clk_in = ~clk_in;

	initial begin
		clk_in = 0;
		reset_in = 0;
		data_in = 0;
		data_valid_in = 0;
		cycle_delay_in = 0;
		osm_in = 0;
		activate_in = 1;
		update_en_in = 1;
		update_in = 0;

		#100;

		// set oversample mode
		osm_in = 2;
		num_samples = 2**osm_in;
		@(posedge clk_in) update_in = 1;
		@(posedge clk_in) update_in = 0;

		repeat(10) begin
			fork
				// send data
				repeat(num_samples) begin
					data_in = $random;
					sum = sum + data_in;

					$display("data_in: %d", data_in);

					@(posedge clk_in) data_valid_in = 1;
					@(posedge clk_in) data_valid_in = 0;

					#100;

					$display("sum_rcv: %d", $signed(oversample_filter_tf.uut.sum));
					$display("sum_exp: %d", sum);

					#1700;
				end

				// receive data
				@(posedge data_valid_out) begin
					received = data_out;
					expected = sum / (num_samples);
					sum = 0;

					if(received == expected) begin
						$write("SUCCESS\t--\t");
					end else begin
						$write("FAILURE\t--\t");
					end
					$display("Received: %d\tExpected: %d", received, expected);
				end
			join
		end

		$stop;

	end

endmodule







