`timescale 1ns / 1ps

// dac_controller -- mba 2014

module dac_controller #(
	// parameters
	parameter W_DATA	= 16,							// width of input data channels
	parameter N_CHAN	= 8							// number of channels
	)(
	// inputs <- top level entity
	input wire						clk_in, 
	input wire						reset_in,

	// inptus <- frontpanel controller
	input wire						ref_set_in,		// dac sets reference voltage when asserted

	// inputs <- cycle controller
	input wire	[W_DATA-1:0]	data_in,
	input wire	[2:0]				channel_in,
	input wire						data_valid_in, 
	input wire						stall_in, 
	 
	// outputs -> top level entity (to dac hardware)
	output wire 					nldac_out, 		// load DACs
	output wire 					nsync_out, 		// enables input shift register to get data
	output wire 					sclk_out, 		// serial clock input (max = 50MHz)
	output wire 					din_out, 		// serial data input
	output wire 					nclr_out, 		// asynchronous clear
	 
	// outputs -> top level entity 
	output wire						dac_done_out,	// pulsed when dac finishes updating a channels
	output wire	[W_DATA-1:0]	data_out,		// output data
	output wire	[2:0]				channel_out		// output data valid
	);

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* dac instruction structures */
wire	[3:0]	 prefix;
wire	[3:0]	 control; 
reg	[3:0]	 channel; 
reg	[15:0] data; 
wire	[3:0]	 feature;
wire	[31:0] data_instr;
wire	[31:0] ref_set_instr; // parallel data instruction

/* data transfer register */
reg	[31:0]	tx_data;

/* state registers */
reg	[7:0]		counter;
reg	[2:0]		cur_state;
reg	[2:0]		next_state;

/* state parameters */
localparam	ST_IDLE			= 3'd0,	// idle state: wait for new data 
				ST_SYNC_DATA	= 3'd1,	// data sync state: prepare dac for data transfer
				ST_SYNC_REF		= 3'd2,	// ref set sync state: prepare dac for reference set 
				ST_STALL			= 3'd3, 	// dac stall state: stall dac for duration of channel write
				ST_TX				= 3'd3,	// transmit state: transmit dac update instruction
				ST_DAC_DONE		= 3'd4;	// dac done state: pusle dac_done signal to indicate operation completion

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* dac control signals */
assign nclr_out = 1'b1; 
assign nldac_out = 1'b0;	// LDAC not used in synchronous update mode  

/* data update instruction components */
assign prefix = 4'b0000; 
assign control = 4'b0011; // write to selected DAC register and update 
assign feature = 4'b0000;
assign data_instr = {prefix, control, channel, data, feature};

/* dac control signals */
assign nsync_out = ~( (cur_state == ST_SYNC_DATA) | (cur_state == ST_SYNC_REF) | (cur_state == ST_TX) ); 
assign sclk_out = clk_in | ~(cur_state == ST_TX); 
assign din_out = tx_data[31]; 

/* reference set instruction */
assign ref_set_instr = {4'b0000, 4'b1001, 4'b0000, 4'b1010, 16'b0};  

/* loop control flow */
assign dac_done_out = ( cur_state == ST_DAC_DONE ); 

/* output data */
assign data_out = data;
assign channel_out = channel; 

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* latch dac write data */
always @( posedge data_valid_in ) begin
	if ( cur_state == ST_IDLE ) begin
		data 		<= data_in; 
		channel 	<= channel_in;
	end
end

/* serial port logic */
always @( posedge clk_in ) begin
	case (cur_state) 
		ST_IDLE: begin
			tx_data <= 0;
		end
		ST_SYNC_DATA: begin
			tx_data <= data_instr; 
		end
		ST_SYNC_REF: begin
			tx_data <= ref_set_instr;
		end
		ST_TX: begin
			tx_data <= tx_data << 1; 
		end
	endcase
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	tx_data		= 0;
	counter		= 0;
	cur_state	= ST_IDLE;
	next_state	= ST_IDLE; 
end

/* state sequential logic */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin 
		cur_state <= ST_IDLE;
	end else begin 
		cur_state <= next_state;
	end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		counter <= 0; 
	end else if ( cur_state != next_state ) begin
		counter <= 0; 
	end else begin 
		counter <= counter + 1'b1;
	end
end

/* next state combinational logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if no case condition is satisified
	case (cur_state) 
		ST_IDLE: begin
			if ( data_valid_in == 1	)			next_state <= ST_SYNC_DATA; 
			else if ( ref_set_in == 1 )		next_state <= ST_SYNC_REF;
			else if ( stall_in == 1 )			next_state <= ST_STALL;
		end 
		ST_SYNC_DATA: begin
			if ( counter == 0 ) 					next_state <= ST_TX;
		end
		ST_SYNC_REF: begin
			if ( counter == 0 ) 					next_state <= ST_TX;
		end
		ST_TX: begin
			if ( counter == 31 ) 				next_state <= ST_DAC_DONE; 
		end
		ST_STALL: begin 
			if ( counter == 32 ) 				next_state <= ST_DAC_DONE;
		end
		ST_DAC_DONE: begin
			if ( counter == 0 ) 					next_state <= ST_IDLE;
		end
	endcase
end

endmodule