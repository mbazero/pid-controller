`timescale 1ns / 1ps

// output_preprocessor -- mba 2014

module output_preprocessor #(
	// parameters
	parameter W_IN				= 32,								// width of input data bus
	parameter W_OUT			= 32,								// width of output data bus
	parameter COMP_LATENCY	= 1								// computation latency in clock cycles
	)(
	// inputs <-- top level entity
	input wire								clk_in,				// system clock
	input wire								reset_in, 			// system reset

	// inputs <-- mux
	input wire signed		[W_IN-1:0]	data_in,				// input data bus
	input wire								data_valid_in,		// data valid signal

	// inputs <-- frontpanel controller
	input wire signed		[W_OUT-1:0]	output_max_in,		// output lower bound
	input wire signed		[W_OUT-1:0]	output_min_in,		// output upper bound
	input wire signed		[W_OUT-1:0]	output_init_in,	// initial output value
	input wire signed		[7:0]			multiplier_in,		// output multiplication factor
	input wire								lock_en_in,			// lock enable signal, opp outputs constant value if lock disables
	input wire								update_en_in,		// module becomes sensitive to update signal when asserted
	input wire								update_in,			// pulse triggers update of module frontpanel parameters

	// outputs <-- dds or cycle controller
	output wire	signed	[W_OUT-1:0]	data_out,			// output data
	output wire								data_valid_out		// output data valid signal
    );


//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* data registers */
reg signed	[W_OUT-1:0] data_out_prev;					// previous outputed data
reg signed	[W_OUT-1:0] lock_data_raw;	 				// raw lock data

wire signed	[W_OUT-1:0] lock_data_stage	[0:3];	// lock data signal in various stages of processing

/* pid parameter registers */
reg signed 	[W_OUT-1:0] output_max; 					// active output upper bound
reg signed	[W_OUT-1:0] output_min;						// active output lower bound
reg signed	[W_OUT-1:0] output_init;					// active output initial value
reg signed	[7:0] 		multiplier; 					// active output multiplication factor

/* output range check */
wire							lock_data_in_range_upper;	// boolean indicating if lock data is below upper bound
wire							lock_data_in_range_lower;	// boolean indicating if lock data is above lower bound

/* state registers */
reg			[7:0]			counter; 						// intrastate counter
reg			[2:0]			cur_state;						// current state
reg			[2:0]			next_state; 					// next state

/* state parameters */
localparam 	ST_IDLE 			= 3'd0,						// module idle, wait for valid data
				ST_COMPUTE		= 3'd1,						// compute filter output
				ST_SEND			= 3'd2, 						// send filter data downstream
				ST_DONE			= 3'd3; 						// cycle complete, latch prev data

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

//// lock data processing ////
/* stage 0: multiply lock data by specified factor */
assign lock_data_stage[0] = lock_data_raw * multiplier;

/* stage 1: add lock data to previous outputed data value */
assign lock_data_stage[1] = lock_data_stage[0] + data_out_prev;

/* stage 2: restrict lock data upper bound */
assign lock_data_stage[2] = ( lock_data_in_range_upper ) ? lock_data_state[1] : output_max;

/* stage 3: restrict lock data lower bound */
assign lock_data_stage[3] = ( lock_data_in_range_lower ) ? lock_data_state[2] : output_min;

/* stage 4: assign output data */
if ( W_OUT >= W_IN ) begin
	assign data_out = lock_data_stage[3];
end else begin
	assign data_out = lock_data_stage[3][W_OUT-1 -: W_IN]; // take n MSB bits of lock data where n = W_OUT
end

//////////////////////////////

/* lock data range check */
assign lock_data_in_range_upper = lock_data_stage[1] <= output_max;
assign lock_data_in_range_lower = lock_data_stage[1] >= outout_min;

/* data output valid signal */
assign data_valid_out = ( cur_state == ST_SEND );

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data register */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		lock_data_raw <= 0;
	end else if ( ( data_valid_in == 1 ) & ( cur_state == ST_IDLE ) ) begin
		lock_data_raw <= data_in;
	end
end

/* previous data register */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		data_out_prev <= 0;
	end else if (( update_in == 1 ) & (update_en_in == 1)) begin
		data_out_prev <= output_init_in;
	end else if ( cur_state == ST_DONE ) begin
		data_out_prev <= data_out;
	end
end

/* frontpanel parameter registers */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		output_max	<= 0;
		output_min	<= 0;
		output_init	<= 0;
		multiplier	<= 0;
	end else if (( update_in == 1 ) & ( update_en_in == 1 )) begin
		output_max	<= output_max_in;
		output_min	<= output_min_in;
		output_init	<= output_init_in;
		multiplier	<= multiplier_in;
	end
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	counter		= 0;
	cur_state 	= ST_IDLE;
	next_state 	= ST_IDLE;
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
	next_state <= cur_state; // default assignment if no case and condition is satisfied
	case ( cur_state )
		ST_IDLE: begin
			if ( data_valid_in == 1 )			next_state <= ST_COMPUTE;
		end
		ST_COMPUTE: begin
			if ( counter == COMP_LATENCY-1 )	next_state <= ST_SEND;
		end
		ST_SEND: begin
			if ( counter == 0 )					next_state <= ST_DONE;
		end
		ST_DONE: begin
			if ( counter == 0 )					next_state <= ST_IDLE;
		end
	endcase
end

endmodule
