//------------------------------------------------------------------------
// okWireOR
//
// This module implements the okWireOR for simulation usage.
//
//------------------------------------------------------------------------
// Copyright (c) 2004-2009 Opal Kelly Incorporated
// CONFIDENTIAL AND PROPRIETARY
// $Id: okWireOR.v 591 2010-10-06 04:31:38Z jake $
//------------------------------------------------------------------------

`default_nettype none
`timescale 1ns / 1ps

module okWireOR # (parameter N = 1)	(
	output reg  [16:0]     ok2,
	input  wire [N*17-1:0] ok2s
	);

	integer i;
	always @(ok2s)
	begin
		ok2 = 0;
		for (i=0; i<N; i=i+1) begin: wireOR
			ok2 = ok2 | ok2s[ i*17 +: 17 ];
		end
	end
endmodule
