//------------------------------------------------------------------------
// okHost.v
//
//  Description:
//    This file is a simulation replacement for okCore for
//    FrontPanel. It receives data from okHostCalls.v which is 
//    then restructured and timed to communicate with the endpoint
//    simulation modules.
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 981 $ $Date: 2011-08-19 12:34:36 -0700 (Fri, 19 Aug 2011) $
//------------------------------------------------------------------------
`default_nettype none
`timescale 1ns / 1ps

`define DNOP                  4'h0
`define DReset                4'h1
`define DSetWireIns           4'h2
`define DUpdateWireIns        4'h3
`define DGetWireOutValue      4'h4
`define DUpdateWireOuts       4'h5
`define DActivateTriggerIn    4'h6
`define DUpdateTriggerOuts    4'h7
`define DIsTriggered          4'h8
`define DWriteToPipeIn        4'h9
`define DReadFromPipeOut      4'ha
`define DWriteToBlockPipeIn   4'hb
`define DReadFromBlockPipeOut 4'hc

`define CReset                5'b00001
`define CSetWireIns           5'b00100
`define CUpdateWireIns        5'b01000
`define CGetWireOutValue      5'b00010
`define CUpdateWireOuts       5'b01000
`define CActivateTriggerIn    5'b00100
`define CUpdateTriggerOuts    5'b10000
`define CIsTriggered          5'b00010
`define CWriteToPipeIn        5'b00100
`define CReadFromPipeOut      5'b00010
`define CWriteToBTPipeIn      5'b00100
`define CReadFromBTPipeOut    5'b00010

module okHost(
	input   wire [7:0]  hi_in,
	output  reg  [1:0]  hi_out,
	inout   wire [15:0] hi_inout,
	inout   wire        hi_aa,
	output  wire        ti_clk,
	output  wire [30:0] ok1,
	input   wire [16:0] ok2
);

`include "parameters.v"
`include "mappings.v"

reg [15:0]  hi_dataout;
reg [30:0]  ok1t;
reg [4:0]   ok1_command;
integer     i, j, k;
reg [7:0]   ep;
reg [31:0]  pipeLength;
integer     BlockDelayStates;
integer     blockSize;
integer     blockNum;
integer     ReadyCheckDelay;
integer     PostReadyDelay;

initial begin
	hi_out = 2'b01;
	hi_dataout = 0;
	i = 0;
	j = 0;
	k = 0;
	ep = 8'h00;
	pipeLength = 0;
	BlockDelayStates = 0;
	blockSize = 0;
	ReadyCheckDelay = 0;
	PostReadyDelay = 0;
end

assign ti_clk = hi_in[0];
assign hi_inout = ~hi_in[1] ? hi_dataout : 16'hzzzz;

assign ok1[OK_TI_CLK]           = ti_clk; 
assign ok1[OK_TI_RESET]         = ok1_command[0]; 
assign ok1[OK_TI_READ]          = ok1_command[1]; 
assign ok1[OK_TI_WRITE]         = ok1_command[2]; 
assign ok1[OK_TI_WIREUPDATE]    = ok1_command[3]; 
assign ok1[OK_TI_TRIGUPDATE]    = ok1_command[4]; 
assign ok1[OK_TI_BLOCKSTROBE]                   = ok1t[OK_TI_BLOCKSTROBE]; 
assign ok1[OK_TI_ADDR_END:OK_TI_ADDR_START]     = ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START]; 
assign ok1[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START]; 

always begin

	wait (hi_in[7:4] != `DNOP);
	case (hi_in[7:4])
		`DReset: begin
			hi_out[0] = 1;
			hi_out[1] = 0;
			@(negedge ti_clk) 
			ok1t[OK_TI_BLOCKSTROBE] = 0; 
			ok1_command = `CReset;
			ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk) hi_out[0] = 0;
		end

		`DUpdateWireIns: begin
			@(negedge ti_clk) hi_out[0] = 1;
			for (i=0; i<32; i=i+1) begin
				@(negedge ti_clk) ok1_command = `CSetWireIns;
				ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = i;
				ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = hi_inout;
			end
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk) ok1_command = `CUpdateWireIns;  ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk) hi_out[0] = 0;
		end
		
		`DUpdateWireOuts: begin
			@(negedge ti_clk) hi_out[0] = 1;
			@(negedge ti_clk) ok1_command = `CUpdateWireOuts; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk) ok1_command = 5'h00;
			@(negedge ti_clk);
			for (i=0; i<32; i=i+1) begin
				ok1_command = `CGetWireOutValue;
				ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = (8'h20+i);
				@(negedge ti_clk) hi_dataout = ok2[OK_DATAOUT_END:OK_DATAOUT_START];
			end
			ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk) hi_dataout = 16'h0000;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;		
		end
		
		`DActivateTriggerIn: begin
			@(negedge ti_clk) hi_out[0] = 1;
			ep = hi_inout[7:0];
			@(negedge ti_clk) ok1_command = `CActivateTriggerIn;
			ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = ep;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = hi_inout;
			@(negedge ti_clk) ok1_command = 5'h00;
			ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;
		end
		
		`DUpdateTriggerOuts: begin
			@(negedge ti_clk) hi_out[0] = 1;
			ok1_command = `CUpdateTriggerOuts; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk) ok1_command = 5'h00;
			@(negedge ti_clk);
			@(negedge ti_clk);
			@(negedge ti_clk);
			
			for (i=0; i<UPDATE_TO_READOUT_CLOCKS; i=i+1)@(negedge ti_clk);
		
			for (i=0; i<32; i=i+1) begin
				ok1_command = `CIsTriggered;
				ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = (8'h60+i);
				@(negedge ti_clk) hi_dataout = ok2[OK_DATAOUT_END:OK_DATAOUT_START];
			end
			ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;
		end
		
		`DWriteToPipeIn: begin
			@(negedge ti_clk) hi_out[0] = 1; j = 0;
			ep = hi_inout[7:0];
			BlockDelayStates = hi_inout[15:8];
			@(negedge ti_clk) pipeLength[15:0] = hi_inout;
			@(negedge ti_clk) pipeLength[31:16]= hi_inout;
			for (i=0; i < pipeLength; i=i+1) begin
				@(negedge ti_clk) ok1_command = `CWriteToPipeIn; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = ep;
				ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = hi_inout;
				j=j+2;
				if (j == 1024) begin
					for (k=0; k < BlockDelayStates; k=k+1) begin
						@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
						ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
					end
					j=0;
				end
			end
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;
		end

		`DReadFromPipeOut: begin
			@(negedge ti_clk) hi_out[0] = 1; j = 0;
			ep = hi_inout[7:0];
			BlockDelayStates = hi_inout[15:8];
			@(negedge ti_clk) pipeLength[15:0] = hi_inout;
			@(negedge ti_clk) pipeLength[31:16]= hi_inout;
			for (i=0; i < pipeLength; i=i+1) begin
				ok1_command = `CReadFromPipeOut; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = ep;
				@(negedge ti_clk);
				if (i == (pipeLength-1))
					ok1_command = 13'h0000;
				hi_dataout = ok2[OK_DATAOUT_END:OK_DATAOUT_START];
				j=j+2;
				if (j == 1024) begin
					for (k=0; k < BlockDelayStates; k=k+1) begin
						ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
						ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
						@(negedge ti_clk);
					end
					j=0;
				end
			end
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;
		end

		`DWriteToBlockPipeIn: begin
			@(negedge ti_clk) hi_out[0] = 1;
			ep = hi_inout[7:0];
			BlockDelayStates = hi_inout[15:8];
			@(negedge ti_clk) pipeLength[15:0] = hi_inout;
			@(negedge ti_clk) pipeLength[31:16]= hi_inout;
			@(negedge ti_clk) blockSize = hi_inout;
			@(negedge ti_clk) ReadyCheckDelay = hi_inout[7:0]; PostReadyDelay = hi_inout[15:8];
			ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = ep;
			blockNum = pipeLength/blockSize;
			for (i=0; i<blockNum;i=i+1) begin
				for (j=0; j<ReadyCheckDelay; j=j+1) @(negedge ti_clk);  // Pre ready delay (no checking)
				while (ok2[OK_READY] !== 1) @(negedge ti_clk);                // Loop while waiting for Ready
				hi_out[0] = 0;                                          // Act as signal to okHostCalls
				for (j=0; j<PostReadyDelay-1; j=j+1) @(negedge ti_clk); // Post ready asserted delay
				@(negedge ti_clk); hi_out[0] = 1;
				ok1t[OK_TI_BLOCKSTROBE] = 1;                            // Block strobe signal
				@(negedge ti_clk) ok1t[OK_TI_BLOCKSTROBE] = 0;          // Turn off block strobe
				@(negedge ti_clk);
				for (j=0; j<blockSize;j=j+1) begin
					@(negedge ti_clk) ok1_command = `CWriteToBTPipeIn;
						ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = hi_inout;
				end
				for (j=0; j<BlockDelayStates; j=j+1) begin
					@(negedge ti_clk) ok1_command = 5'h00;
					ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
				end
			end
			
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0; j = 0;
		end

		`DReadFromBlockPipeOut: begin
			@(negedge ti_clk) hi_out[0] = 1;
			ep = hi_inout[7:0];
			BlockDelayStates = hi_inout[15:8];
			@(negedge ti_clk) pipeLength[15:0] = hi_inout;
			@(negedge ti_clk) pipeLength[31:16]= hi_inout;
			@(negedge ti_clk) blockSize = hi_inout;
			@(negedge ti_clk) ReadyCheckDelay = hi_inout[7:0]; PostReadyDelay = hi_inout[15:8];
			ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = ep;
			blockNum = pipeLength/blockSize;
			for (i=0; i < blockNum; i=i+1) begin
				for (j=0; j<ReadyCheckDelay; j=j+1) @(negedge ti_clk);	// Pre ready delay (no checking)
				while (ok2[OK_READY] !== 1) @(negedge ti_clk);                // Loop while waiting for Ready
				hi_out[0] = 0;                                          // Act as signal to okHostCalls
				for (j=0; j<PostReadyDelay-1; j=j+1) @(negedge ti_clk); // Post ready asserted delay
				@(negedge ti_clk); hi_out[0] = 1;
				ok1t[OK_TI_BLOCKSTROBE] = 1;                            // Block strobe signal
				@(negedge ti_clk) ok1t[OK_TI_BLOCKSTROBE] = 0;          // Turn off block strobe
				for (j=0; j<blockSize;j=j+1) begin
					ok1_command = `CReadFromPipeOut;
					@(negedge ti_clk);
					if (i == (pipeLength-1)) ok1_command = 5'h00;
					hi_dataout = ok2[OK_DATAOUT_END:OK_DATAOUT_START];
				end
				for (j=0; j < BlockDelayStates; j=j+1) begin
					ok1_command = 5'h00; 
					@(negedge ti_clk) hi_dataout = 16'h0000;
				end
			end
			@(negedge ti_clk) ok1_command = 5'h00; ok1t[OK_TI_ADDR_END:OK_TI_ADDR_START] = 8'h00;
			ok1t[OK_TI_DATAIN_END:OK_TI_DATAIN_START] = 16'h0000;
			@(negedge ti_clk); @(negedge ti_clk); @(negedge ti_clk);
			@(negedge ti_clk) hi_out[0] = 0;
		end
		default: $display("Unsupport hi_addr sent");
	endcase
end

endmodule
