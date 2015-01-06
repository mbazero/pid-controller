`timescale 1ns / 1ps

// clk_sync -- mba 2014

module clk_sync #(
	// parameters
	parameter W_DATA	= 18,							// width of data bus
	parameter N_ADC	= 6							// number of active adc channels
	)(
	// inputs <- top level entity
	input wire						sys_clk_in,		// 50MHz system clock
	input wire						reset_in,		// system reset

	// inputs <- adc controller
	input wire	[N_ADC-1:0]		data_valid_in,	// data valid input signal synchronous with 17MHz adc clock
	input wire	[W_DATA-1:0]	data_a_in,
	input wire	[W_DATA-1:0]	data_b_in, 	

	// outputs -> oversample filter
	output wire	[N_ADC-1:0]		data_valid_out,
	output reg	[W_DATA-1:0]	data_a_out,
	output reg	[W_DATA-1:0]	data_b_out
	);

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* reduction data valid */
wire 					data_valid_rdc;

/* data valid register */
reg	[N_ADC-1:0]	data_valid;

/* state registers */
reg	[2:0] 				cur_state;	// state machine current state
reg	[2:0] 				next_state;	// state machine next state

/* state parameters */
localparam	ST_WAIT_PE	= 3'd0,	// wait for data_valid_in to go high
				ST_SEND		= 3'd1,	// assert data_valid_out synchronous with 50MHz clock
				ST_WAIT_NE	= 3'd2;	// wait for data_valid_in to go low

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* reduction data valid */
assign data_valid_rdc = | data_valid_in;

/* data valid out */
assign data_valid_out = data_valid & {N_ADC{ cur_state == ST_SEND }};

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data register */ 
always @( posedge data_valid_rdc ) begin
	data_a_out <= data_a_in; 
	data_b_out <= data_b_in;
	data_valid <= data_valid_in; 
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* state register - synchronous with system clock */
always @( posedge sys_clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin 
		cur_state <= ST_WAIT_PE;
	end else begin
		cur_state <= next_state;
	end
end

/* next state transitin logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if no case statement is satisfied
	case ( cur_state )
		ST_WAIT_PE: if ( data_valid_rdc == 1 )	next_state <= ST_SEND;
		ST_SEND: 										next_state <= ST_WAIT_NE;
		ST_WAIT_NE:	if ( data_valid_rdc == 0 )	next_state <= ST_WAIT_PE;
	endcase
end

endmodule