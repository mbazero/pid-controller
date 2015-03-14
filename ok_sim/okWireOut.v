//------------------------------------------------------------------------
// okWireOut
//
// This module simulates the "Wire Out" endpoint.
//
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 679 $ $Date: 2010-11-17 12:55:11 -0800 (Wed, 17 Nov 2010) $
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns / 1ps

module okWireOut(
	input  wire [30:0] ok1,
	output wire [16:0] ok2,
	input  wire [7:0]  ep_addr,
	input  wire [15:0] ep_datain
	);

`include "parameters.v" 
`include "mappings.v"

reg  [15:0] wirehold;

assign ok2[OK_DATAOUT_END:OK_DATAOUT_START] = (ti_addr == ep_addr) ? (wirehold) : (0);
assign ok2[OK_READY]                        = 0;

always @(posedge ti_clock) begin
	if (ti_reset == 1)
		wirehold <= 0;
	else if (ti_wireupdate == 1)
		wirehold <= ep_datain;
end

endmodule