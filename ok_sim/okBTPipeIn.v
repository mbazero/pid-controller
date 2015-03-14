//------------------------------------------------------------------------
// okBTPipeIn.v
//
// This module simulates the "Block Input Pipe" endpoint.
//
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 591 $ $Date: 2010-10-05 21:31:38 -0700 (Tue, 05 Oct 2010) $
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns / 1ps

module okBTPipeIn(
	input  wire [30:0] ok1,
	output wire [16:0] ok2,
	input  wire [7:0]  ep_addr,
	output reg         ep_write,
	output wire        ep_blockstrobe,
	output reg  [15:0] ep_dataout,
	input  wire        ep_ready
	);

`include "parameters.v" 
`include "mappings.v"

assign ep_blockstrobe                       = ti_blockstrobe & (ti_addr == ep_addr);
assign ok2[OK_DATAOUT_END:OK_DATAOUT_START] = 0;
assign ok2[OK_READY]                        = (ti_addr == ep_addr) ? (ep_ready) : (0);

always @(posedge ti_clock) begin
	#TDOUT_DELAY;
	ep_write = 0;
	if (ti_reset == 1) begin
		ep_dataout  = 0;
	end else if ((ti_write == 1) && (ti_addr == ep_addr)) begin
		ep_dataout = ti_datain;
		ep_write = 1;
	end
end

endmodule
