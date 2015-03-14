//------------------------------------------------------------------------
// okPipeOut.v
//
// This module simulates the "Output Pipe" endpoint.
//
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 591 $ $Date: 2010-10-05 21:31:38 -0700 (Tue, 05 Oct 2010) $
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns / 1ps

module okPipeOut(
	input  wire [30:0] ok1,
	output wire [16:0] ok2,
	input  wire [7:0]  ep_addr,
	output wire        ep_read,
	input  wire [15:0] ep_datain
	);

`include "parameters.v" 
`include "mappings.v"

assign ok2[OK_DATAOUT_END:OK_DATAOUT_START] = (ti_addr == ep_addr) ? (ep_datain) : (0);
assign ok2[OK_READY]                        = (ti_addr == ep_addr) ? (1) : (0);
assign ep_read                              = ((ti_read == 1) && (ti_addr == ep_addr)) ? (1) : (0);

endmodule
