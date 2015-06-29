`timescale 1ns / 1ps

// oversample_filter -- mba 2014
// -----------------------------------------------------------
// Computes a moving average of input data with a variable
// number of samples.
// -----------------------------------------------------------

module oversample_filter #(
	// parameters
	parameter W_DATA		= 18,										// width of input data
	parameter W_EP			= 16,										// width of opal kelly endpoint
	parameter W_OSM		= 4										// width of oversample mode signal (max oversample ratio = 2^(2^W_OSM - 1))
	)(
	// inputs <- top level entity
	input wire									clk_in,				// system clock
	input wire									reset_in,			// system reset

	// inputs <- pid core
	input wire signed		[W_DATA-1:0]	data_in,				// input data
	input wire									data_valid_in,		// input data valid signal; asynchronous timing supported

	// inputs <- frontpanel controller
	input wire				[W_EP-1:0]		cycle_delay_in,	// delay period in adc cycles
	input wire				[W_OSM-1:0]		osm_in,				// oversample mode (log base 2 of the oversample ratio)
	input wire									activate_in,		// channel activation signal (1 = activated, 0 = deactivated)

	// outputs -> clk sync
	output wire signed	[W_DATA-1:0]	data_out,			// output data
	output wire									data_valid_out		// output data valid signal
	);

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam	MAX_OS	= 2**W_OSM - 1;							// maximum log2 oversample ratio
localparam	W_SUM		= MAX_OS + W_DATA;						// width of sum register

/* state parameters */
localparam	ST_IDLE			= 3'd0,								// wait for channel activation signal
				ST_DELAY			= 3'd1,								// wait specified number of adc cycles before accepting data (to account for DAC/DDS settling times)
				ST_SAMPLE		= 3'd2,								// collect adc data and maintain accumulating sum
				ST_SEND			= 3'd3;								// divide sum by oversample ratio (right shift) and assert data valid

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* wires */
wire							idle;
wire							osf_reset;								// local reset signal which is activated by system reset or channel deactive

/* registers */
reg			[MAX_OS:0]	sample_counter = 0;
reg signed 	[W_SUM-1:0]	sum = 0;

/* state registers */
reg			[15:0]		counter = 0;
reg			[2:0]			cur_state = ST_IDLE;
reg			[2:0]			next_state = ST_IDLE;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* divide sum by oversample ratio (left shift amount equal to log2 oversample ration) */
assign data_out 			= ( sum >> osm_in );

/* assert data_valid_out during the SEND state */
assign data_valid_out 	= ( cur_state == ST_SEND );

/* osf reset */
assign osf_reset 			= ( reset_in | ~activate_in );

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* sum accumulator */
always @( posedge clk_in ) begin
	if ( osf_reset == 1 ) begin
		sum <= 0;
	end else if (( cur_state == ST_IDLE ) | ( cur_state == ST_DELAY )) begin
		sum <= 0;
	end else if (( data_valid_in == 1 ) & ( cur_state == ST_SAMPLE )) begin
		sum <= sum + data_in;
	end
end

/* count number of adc data words received in the current state */
always @( posedge clk_in ) begin
	if ( osf_reset == 1 ) begin
		sample_counter	<= 0;
	end else if ( cur_state != next_state ) begin
		sample_counter	<= 0;
	end else if ( data_valid_in == 1 ) begin
		sample_counter	<= sample_counter + 1'b1;
	end
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* state register */
always @( posedge clk_in ) begin
	if ( osf_reset == 1 ) begin
		cur_state <= ST_IDLE;
	end else begin
		cur_state <= next_state;
	end
end

/* state counter */
always @( posedge clk_in ) begin
	if ( osf_reset == 1 ) begin
		counter <= 0;
	end else if ( cur_state != next_state ) begin
		counter <= 0;
	end else begin
		counter <= counter + 1'b1;
	end
end

/* next state transition logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if no case statement and condition is satisfied
	case ( cur_state )
		ST_IDLE: begin
			if ( activate_in == 1 )							next_state <= ST_SAMPLE;	// don't have to delay on first iteration
		end
		ST_DELAY: begin
			if ( sample_counter >= cycle_delay_in ) 	next_state <= ST_SAMPLE;
		end
		ST_SAMPLE: begin
			if ( sample_counter[osm_in] == 1 )			next_state <= ST_SEND;
		end
		ST_SEND: 												next_state <= ST_DELAY;
	endcase
end

endmodule

