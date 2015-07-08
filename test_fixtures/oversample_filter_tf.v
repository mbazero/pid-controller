`timescale 1ns / 1ps

// oversample_filter_tf

module oversample_filter_tf;

	// Parameters
    localparam W_CHAN = 5;
	localparam W_DATA = 18;
    localparam W_SUM = 128;
    localparam W_OS = 5;
    localparam W_WR_ADDR = W_EP;
    localparam W_WR_CHAN = W_EP;
    localparam W_WR_DATA = W_EP*4;

	localparam W_EP = 16;

	// Inputs
	reg clk_in;
	reg rst_in;

    reg dv_in;
    reg chan_in;
	reg signed [W_DATA-1:0] data_in;

    reg wr_en;
    reg [W_WR_ADDR-1:0] wr_addr;
    reg [W_WR_CHAN-1:0] wr_chan;
    reg [W_WR_DATA-1:0] wr_data;

	// Outputs
	wire dv_out;
    wire [W_CHAN-1:0] chan_out;
	wire signed [W_DATA-1:0] data_out;

	oversample_filter #(
        .W_CHAN(W_CHAN),
        .W_DATA(W_DATA),
        .W_SUM(W_SUM),
        .W_OS(W_OS),
        .W_WR_ADDR(W_WR_ADDR),
        .W_WR_CHAN(W_WR_CHAN),
        .W_WR_DATA(W_WR_DATA))
	uut (
        .clk_in(clk_in),
        .rst_in(rst_in),
        .dv_in(dv_in),
        .chan_in(chan_in),
        .data_in(data_in),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_chan(wr_chan),
        .wr_data(wr_data),
        .dv_out(dv_out),
        .chan_out(chan_out),
        .data_out(data_out)
	);

	// simulation structures
	integer num_samples;
	reg signed [127:0] sum = 0;
	reg signed [W_DATA-1:0] received;
	reg signed [W_DATA-1:0] expected;
	integer delta = 0;
    reg [W_OS-1:0] os;

	// generate 50MHz clock
	always #10 clk_in = ~clk_in;

	initial begin
		clk_in = 0;
		rst_in = 0;
		dv_in = 0;
        chan_in = 0;
		data_in = 0;
        wr_en = 0;
        wr_addr = 0;
        wr_chan = 0;
        wr_data = 0;

		#100;

		// initial oversample mode
		os = 0;

		repeat(2**W_OS) begin
            // set oversample mode in module
            num_samples = 2**os;
            wr_addr = ovr_os_addr;
            wr_chan = 0;
            wr_data = os;
            @(posedge clk_in) wr_en = 1;
            @(posedge clk_in) wr_en = 0;

			$display("######################################");
			$display("OS = %d", os);
			$display("######################################");

			repeat(10) begin
				fork
					// send data
					repeat(num_samples) begin
						data_in = $random;
						sum = sum + data_in;

						$display("data_in: %d", data_in);

						@(posedge clk_in) dv_in = 1;
						@(posedge clk_in) dv_in = 0;

						#100;

                        @(posedge oversample_filter_tf.uut.dv_p2) begin
                            assert_equals(sum, oversample_filter_tf.uut.sum_p2, "SUM");
							$stop;
                        end

						#200;
					end

					// receive data
					@(posedge dv_out) begin
						received = data_out;
						expected = sum / (num_samples);
						sum = 0;

						delta = expected - received;

						$display("-------------------------------------------");
						$display("Received: %d\tExpected: %d", received, expected);
						$display("OSM: %d", os);
						$display("num_samples: %d", num_samples);
						if(delta == 0 | delta == -1 | delta == 1) begin // data received might vary by +/- 1
							$display("SUCCESS");
						end else begin
							$display("FAILURE");
							$stop;
						end
						$display("-------------------------------------------");
					end
				join
			end

            // increment os
			os = os + 1;
		end

		$display("*****SIMULATION COMPLETED SUCCESSFULLY*****");

		$stop;

	end

    `include "assert_equals.v"

endmodule
