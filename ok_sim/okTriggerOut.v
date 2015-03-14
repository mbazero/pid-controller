//------------------------------------------------------------------------
// okTriggerOut.v
//
// This module simulates the "Trigger Out" endpoint.
//
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 591 $ $Date: 2010-10-05 21:31:38 -0700 (Tue, 05 Oct 2010) $
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns / 1ps

module okTriggerOut(
	input  wire [30:0] ok1,
	output wire [16:0] ok2,
	input  wire [7:0]  ep_addr,
	input  wire        ep_clk,
	input  wire [15:0] ep_trigger
	);

`include "parameters.v" 
`include "mappings.v"

reg  [15:0] eptrig;
reg  [15:0] ep_trigger_p1;
reg  [15:0] trighold;
reg         captrig;

assign ok2[OK_DATAOUT_END:OK_DATAOUT_START] = (ti_addr == ep_addr) ? (trighold) : (0);
assign ok2[OK_READY]                        = 0;

always @(posedge ti_clock) if (ti_trigupdate == 1'b1) captrig = 1;

always @(posedge ep_clk or posedge ti_reset) begin
	if (ti_reset == 1) begin
		ep_trigger_p1 = 0;
		trighold = 0;
		eptrig = 0;
		captrig = 0;
	end 
	else begin
		if (captrig == 1) begin
			trighold = eptrig;
			eptrig = ep_trigger;
			captrig = 0;   
		end
		else eptrig = eptrig | (ep_trigger & ~ep_trigger_p1);
		ep_trigger_p1 = ep_trigger;
  end
end

endmodule
