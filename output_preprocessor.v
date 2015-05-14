`timescale 1ns / 1ps

// output_preprocessor -- mba 2014

// TODO
// - change to pipelined version
// - clean up overflow checking

module output_preprocessor #(
	// parameters
	parameter W_IN				= 18,								// width of input data bus
	parameter W_OUT			= 16,								// width of output data bus
	parameter COMP_LATENCY	= 3,								// computation latency in clock cycles
	parameter OMAX_INIT		= 9999,							// initial output upper bound
	parameter OMIN_INIT		= 1111,							// initial output lower bound
	parameter OINIT_INIT		= 5000,							// initial output starting value
	parameter MULT_INIT		= 1								// initial output multiplier
	)(
	// inputs <-- top level entity
	input wire								clk_in,				// system clock
	input wire								reset_in, 			// system reset

	// inputs <-- mux
	input wire signed		[W_IN-1:0]	data_in,				// input data bus
	input wire								data_valid_in,		// data valid signal
	input wire								lock_en_in,			// lock enable signal, opp outputs constant value if lock disables

	// inputs <-- frontpanel controller
	input wire signed		[W_OUT-1:0]	output_max_in,		// output lower bound
	input wire signed		[W_OUT-1:0]	output_min_in,		// output upper bound
	input wire signed		[W_OUT-1:0]	output_init_in,	// initial output value
	input wire 				[7:0]       multiplier_in,		// output multiplication factor
	input wire								update_en_in,		// module becomes sensitive to update signal when asserted
	input wire								update_in,			// pulse triggers update of module frontpanel parameters

	// outputs <-- dds controller or dac instruction queue
	output wire	signed	[W_OUT-1:0]	data_out,			// output data
	output wire								data_valid_out		// output data valid signal
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam MAX_OUTPUT = {1'b0, {W_OUT-1{1'b1}}};
localparam MIN_OUTPUT = ~MAX_OUTPUT;

/* state parameters */
localparam 	ST_IDLE 			= 3'd0,						// module idle, wait for valid data
				ST_COMPUTE		= 3'd1,						// compute filter output
				ST_SEND			= 3'd2, 						// send filter data downstream
				ST_DONE			= 3'd3; 						// cycle complete, latch prev data

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* data registers */
reg signed	[W_OUT-1:0] data_out_prev = 0;			// previous outputed data
reg signed	[W_OUT-1:0] lock_data_raw = 0;			// raw lock data

/* processing stage */
wire signed	[W_OUT-1:0] proc_stage [0:4];				// data processing stages

/* overflow handling */
wire signed [W_OUT-1:0] proc_stage_pre [0:1];		// processing stage pre overflow check
wire signed [W_OUT-1:0] proc_stage_clamped [0:1];	// clamped processing stage
wire							overflow	[0:1];				// overflow indicator

/* pid parameter registers */
reg signed 	[W_OUT-1:0] output_max = OMAX_INIT;		// active output upper bound
reg signed	[W_OUT-1:0] output_min = OMAX_INIT;		// active output lower bound
reg signed	[W_OUT-1:0] output_init = OINIT_INIT;	// active output initial value
reg signed	[W_OUT-1:0]	multiplier = MULT_INIT; 	// active output multiplication factor

/* state registers */
reg			[7:0]			counter = 0; 					// intrastate counter
reg			[2:0]			cur_state = ST_IDLE;			// current state
reg			[2:0]			next_state = ST_IDLE;		// next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

//// lock data processing ////
/* stage 0: multiply lock data by specified factor */
assign proc_stage_pre[0] = lock_data_raw * multiplier;

assign proc_stage_clamped[0] = (lock_data_raw[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign overflow[0] = lock_data_raw[W_OUT-1] != proc_stage_pre[0][W_OUT-1];
assign proc_stage[0] = (overflow[0]) ? proc_stage_clamped[0] : proc_stage_pre[0];

/* stage 1: add lock data to previous outputed data value */
assign proc_stage_pre[1] = proc_stage[0] + data_out_prev;

assign proc_stage_clamped[1] = (data_out_prev[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign overflow[1] = (proc_stage[0][W_OUT-1] == data_out_prev[W_OUT-1])
						&& (proc_stage_pre[1][W_OUT-1] != proc_stage[0]);
assign proc_stage[1] = (overflow[1]) ? proc_stage_clamped[1] : proc_stage_pre[1];

/* stage 2: select output init value if lock is not enabled */
assign proc_stage[2] = ( lock_en_in == 1 ) ? proc_stage[1] : output_init;

/* stage 3: restrict lock data upper bound */
assign proc_stage[3] = ( proc_stage[2] < output_max ) ? proc_stage[2] : output_max;

/* stage 4: restrict lock data lower bound */
assign proc_stage[4] = ( proc_stage[3] > output_min ) ? proc_stage[3] : output_min;

/* data output */
assign data_out = proc_stage[4];

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
		/* convert input data to output width */
		if ( W_OUT < W_IN ) begin
			lock_data_raw <= data_in[W_IN-1 -: W_OUT]; // discard LSB if output width < input width
		end else begin
			lock_data_raw <= data_in; // sign automatically if output width > input width
		end
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
always @( posedge update_in ) begin
	if ( update_en_in == 1 ) begin
		output_max	<= output_max_in;
		output_min	<= output_min_in;
		output_init	<= output_init_in;
		multiplier	<= multiplier_in;
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
			if ( counter == 0 )					next_state <= ST_DONE;
		end
		ST_DONE: begin
			if ( counter == 0 )					next_state <= ST_IDLE;
		end
	endcase
end

endmodule
