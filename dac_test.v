`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    22:59:33 04/09/2014
// Design Name:
// Module Name:    dac_test
// Project Name:
// Target Devices:
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module dac_test(
	 // inputs
	 input wire				clk1,
	 input wire[15:0]		FP_DVAL,
	 input wire[15:0]		FP_CHAN,
	 input wire[15:0]		FP_TRIG,
	 input wire[15:0]		FP_REF_SET,

	 // outputs --> DAC controls
    output wire 			nLDAC, // Load DACs
    output wire 			nSYNC, // enables input shift register to get data
    output wire 			SCLK, // serial clock input (max = 50MHz)
    output wire 			DIN, // serial data input
    output wire 			nCLR, // asynchronous clear

	 // outputs --> frontpanel
	 output reg[15:0]		data_out,
	 output reg[3:0]		chan_out,

	 // outputs --> debug
	 output wire[2:0]		cur_state_db
	 );

/* debug */
assign cur_state_db = cur_state;

/* module wires */
wire[3:0] prefix;
wire[3:0] control;
wire[3:0] channel;
wire[15:0] data;
wire[3:0] feature;
wire[31:0] par_instr, up_instr, ref_set_instr; // parallel data instruction

/* DAC control signals */
assign nCLR = 1'b1;
assign nLDAC = 1'b0; // LDAC not used in synchronous update mode

/* DAC instruction */
assign prefix = 4'b0000;
assign control = 4'b0011; // write to selected DAC register and update
assign channel = FP_CHAN[3:0];
assign data = FP_DVAL;
assign feature = 4'b0000;

assign up_instr = {prefix, control, channel, data, feature}; // create dac instruction
assign ref_set_instr = {4'b0000, 4'b1001, 4'b0000, 4'b1010, 16'b0};

assign par_instr = (FP_REF_SET[0]) ? ref_set_instr:up_instr;


/* state machine */
reg[2:0] cur_state, next_state;
reg[31:0] tx_data;
reg[32:0] counter;

localparam	ST_INIT	= 3'd0,
				ST_SYNC	= 3'd1,
				ST_TX		= 3'd2;

/* initial assignments */
initial begin
	cur_state = ST_INIT;
	next_state = ST_INIT;
	tx_data = 0;
	counter = 0;
end

/* state creation */
always @(posedge clk1) begin
	cur_state <= next_state;
end

/* state counter */
always @(posedge clk1) begin
	if (cur_state != next_state) begin
		// reset counter if next state is different
		counter <= 0;
	end else begin
		// increment counter if next state is same
		counter <= counter + 1;
	end
end

/* state transitions */
always @( * ) begin
	next_state <= cur_state;
	case (cur_state)
		ST_INIT: begin
			if(FP_TRIG[0] == 1) next_state <= ST_SYNC;
		end
		ST_SYNC: begin
			if(counter == 0) next_state <= ST_TX;
		end
		ST_TX: begin
			if(counter == 31) next_state <= ST_INIT;
		end
	endcase
end

/* output logic */
assign nSYNC = ~( (cur_state == ST_SYNC) | (cur_state == ST_TX) );
assign SCLK = clk1 | ~(cur_state == ST_TX);
assign DIN = tx_data[31];

/* serial port logic */
always @(posedge clk1) begin
	case (cur_state)
		ST_INIT: begin
			tx_data <= 0;
		end
		ST_SYNC: begin
			tx_data <= par_instr;
		end
		ST_TX: begin
			tx_data <= tx_data << 1;
		end
	endcase
end

/* update data and channel on frontpanel display */
always @(posedge nSYNC) begin
	if (FP_REF_SET[0] == 0) begin
		data_out <= data;
		chan_out <= channel;
	end
end

endmodule
