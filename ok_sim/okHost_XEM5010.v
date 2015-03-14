`default_nettype none
`timescale 1ns / 1ps

module okHost_XEM5010(
	input   wire [7:0]  hi_in,
	output  wire [1:0]  hi_out,
	inout   wire [15:0] hi_inout,
	output  wire        ti_clk,
	output  wire [30:0] ok1,
	input   wire [16:0] ok2
);

okHost h0 (
	.hi_in     (hi_in),
	.hi_out    (hi_out),
	.hi_inout  (hi_inout),
	.ti_clk    (ti_clk),
	.ok1       (ok1),
	.ok2       (ok2)
	);

endmodule
