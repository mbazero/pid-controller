`timescale 1ns / 1ps

// output_preprocessor -- mba 2014
// -----------------------------------------------------------
// Processes data stream before it is sent to DAC/DDS outputs.
// Adds PID sum to previous output value and enforces max and
// min output bounds.
// -----------------------------------------------------------

module output_preprocessor #(
	// parameters
	parameter W_IN				= 64,								// width of input data bus
	parameter W_OUT			= 16,								// width of output data bus
	parameter W_MLT			= 10,								// width of multiplier
	parameter W_EP				= 16,								// width of opal kelly endpoint
	parameter COMP_LATENCY	= 1,								// computation latency in clock cycles
	parameter MAX_INIT		= 52428,							// initial output upper bound
	parameter MIN_INIT		= 13107,							// initial output lower bound
	parameter OUT_INIT		= 39321,							// initial output starting value
	parameter MLT_INIT		= 1,								// initial output multiplier
	parameter RS_INIT			= 1								// initial output right shift
	)(
	// inputs <-- top level entity
	input wire								clk_in,				// system clock
	input wire								reset_in, 			// system reset

	// inputs <-- mux
	input wire signed		[W_IN-1:0]	pid_sum_in,			// pid sum
	input wire								data_valid_in,		// data valid signal
	input wire								lock_en_in,			// lock enable signal, opp outputs constant value if lock disabled

	// inputs <-- frontpanel controller
	input wire signed		[W_OUT-1:0]	output_max_in,		// output lower bound
	input wire signed		[W_OUT-1:0]	output_min_in,		// output upper bound
	input wire signed		[W_OUT-1:0]	output_init_in,	// initial output value
	input wire signed		[W_MLT-1:0]	multiplier_in,		// output multiplication factor
	input wire 				[W_EP-1:0]	right_shift_in,	// output right shift
	input wire								update_en_in,		// module becomes sensitive to update signal when asserted
	input wire								update_in,			// pulse triggers update of module frontpanel parameters

	// outputs --> dds or dac
	output wire	signed	[W_OUT-1:0]	data_out,			// output data
	output wire								data_valid_out		// output data valid signal
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam	MAX_OUTPUT = {1'b0, {W_OUT-1{1'b1}}};
localparam	MIN_OUTPUT = ~MAX_OUTPUT;

/* state parameters */
localparam 	ST_IDLE 			= 3'd0,							// module idle, wait for valid data
				ST_COMPUTE		= 3'd1,							// compute filter output
				ST_SEND			= 3'd2, 							// send filter data downstream
				ST_WRITEBACK	= 3'd3; 							// cycle complete, write back outputted data

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* data registers */
reg signed	[W_OUT-1:0]	data_out_prev = OUT_INIT;		// previous outputed data
reg signed	[W_IN-1:0]	pid_sum = 0;						// current pid sum

/* processing stage */
wire signed [W_IN+W_MLT-1:0]	proc_stage_0;
wire signed [W_IN+W_MLT:0]		proc_stage_1,
										proc_stage_2,
										proc_stage_3,
										proc_stage_4,
										proc_stage_5;

/* pid parameter registers */
reg signed 	[W_OUT-1:0]	output_max = MAX_INIT;			// active output upper bound
reg signed	[W_OUT-1:0]	output_min = MIN_INIT;			// active output lower bound
reg signed	[W_OUT-1:0]	output_init = OUT_INIT;			// active output initial value
reg signed	[W_MLT-1:0]	multiplier = MLT_INIT; 			// active output multiplication factor
reg 			[W_EP-1:0]	right_shift = RS_INIT;			// active ooutput division factor

/* state registers */
reg			[7:0]			counter = 0; 						// intrastate counter
reg			[2:0]			cur_state = ST_IDLE;				// current state
reg			[2:0]			next_state = ST_IDLE;			// next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

//// output data processing ////
/* stage 0: multiply pid sum */
assign proc_stage_0 = pid_sum * multiplier;

/* stage 1: divide pid sum */
assign proc_stage_1 = proc_stage_0 >>> right_shift;

/* stage 2: add lock data to previous outputed data value */
assign proc_stage_2 = proc_stage_1 + data_out_prev;

/* stage 3: select output init value if lock is not enabled */
assign proc_stage_3 = ( lock_en_in == 1 ) ? proc_stage_2 : output_init;

/* stage 4: restrict lock data upper bound */
assign proc_stage_4 = ( proc_stage_3 > output_max ) ? output_max : proc_stage_3;

/* stage 5: restrict lock data lower bound */
assign proc_stage_5 = ( proc_stage_4 < output_min ) ? output_min : proc_stage_4;
////////////////////////////////

/* data out */
assign data_out = proc_stage_5[W_OUT-1:0];
assign data_valid_out = ( cur_state == ST_SEND );

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data register */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		pid_sum <= 0;
	end else if ( ( data_valid_in == 1 ) & ( cur_state == ST_IDLE ) ) begin
		pid_sum <= pid_sum_in;
	end
end

/* previous data register */
always @( posedge clk_in ) begin
	if ( reset_in == 1  ) begin
		data_out_prev <= output_init;
	end else if ( cur_state == ST_WRITEBACK ) begin
		data_out_prev <= data_out;
	end
end

/* frontpanel parameter registers */
always @( posedge update_in ) begin
	if ( update_en_in == 1 ) begin
		output_max	<= output_max_in;
		output_min	<= output_min_in;
		output_init	<= output_init_in;
		multiplier	<= multiplier_in;
		right_shift	<= right_shift_in;
	end
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

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
			if ( counter == 0 )					next_state <= ST_WRITEBACK;
		end
		ST_WRITEBACK: begin
			if ( counter == 0 )					next_state <= ST_IDLE;
		end
	endcase
end

endmodule
