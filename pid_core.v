`timescale 1ns / 1ps

// pid_core -- mba 2014

// TODO
// - remove lock enable from here and top level

module pid_core #(
	// parameters
	parameter W_IN				= 18,							// input data width
	parameter W_OUT 			= 18,							// output data width
	parameter COMP_LATENCY	= 1,							// pid computation latency in clock cycles
	parameter SETPOINT_INIT	= 0,							// initial setpoint
	parameter P_COEF_INIT	= 10,							// initial p coefficient
	parameter I_COEF_INIT	= 3,							// initial i coefficient
	parameter D_COEF_INIT	= 0							// initial d coefficient
	)(
	// inputs <- top level entity
	input wire								clk_in,			// system clock
	input wire								reset_in,		// system reset

	// inputs <- oversample filter
	input wire signed 	[W_IN-1:0]	data_in, 		// unsigned input data
	input wire								data_valid_in,	// input data valid signal

	// inputs <- frontpanel controller
	input wire signed		[15:0] 		setpoint_in,	// lock setpoint
	input wire signed 	[15:0] 		p_coef_in,		// proportional coefficient
	input wire signed 	[15:0] 		i_coef_in,		// integral coefficient
	input wire signed 	[15:0] 		d_coef_in,		// derivative coefficient
	input wire								lock_en_in,		// DEBUG not used anymore
	input wire								clear_in,		// clears pid memory
	input wire								update_en_in,	// sensitizes module to update signal
	input wire								update_in, 		// pulse triggers update of frontpanel parameters

	// outputs -> source mux
	output wire signed	[W_OUT-1:0]	data_out,		// pid filter output
	output wire								data_valid_out	// output data valid signal
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam MAX_OUTPUT = {1'b0, {W_OUT-1{1'b1}}};
localparam MIN_OUTPUT = ~MAX_OUTPUT;

/* state parameters */
localparam 	ST_IDLE 			= 3'd0,			// module idle, wait for valid data
				ST_COMPUTE		= 3'd1,			// compute filter output
				ST_SEND			= 3'd2, 			// send filter data downstream
				ST_DONE			= 3'd3; 			// cycle complete, latch prev data

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* input data */
reg signed	[W_OUT-1:0]	data = 0;					// active input data

/* overflow handling */
wire 							overflow;
wire signed [W_OUT-1:0]	u_cur_clamped;

/* pid parameters */
reg signed	[W_OUT-1:0]	setpoint = SETPOINT_INIT;	// active lock setpoint
reg signed 	[W_OUT-1:0]	p_coef = P_COEF_INIT;		// active proportional coefficient
reg signed	[W_OUT-1:0]	i_coef = I_COEF_INIT;		// active integral coefficient
reg signed	[W_OUT-1:0]	d_coef = D_COEF_INIT;		// active derivative coefficient

/* error signals */
wire signed	[W_OUT-1:0]	e_cur;				// current error signal
reg signed	[W_OUT-1:0]	e_prev_0 = 0;		// most recent previous error signal
reg signed	[W_OUT-1:0]	e_prev_1 = 0;		// second most recent previous error signal

/* z-transform coefficients */
wire signed	[W_OUT-1:0]	k1, k2, k3; 		// z-transform coefficients for discrete PID filter

/* control variable (u) cur, prev, and delta vals */
reg signed	[W_OUT-1:0]	u_prev = 0;			// previous pid filter output
reg signed	[W_OUT-1:0] u_next = 0;			// next pid filter output
wire signed	[W_OUT-1:0] u_cur; 				// current pid filter output
wire signed	[W_OUT-1:0] delta_u;				// difference between current and previous pid filter outputs

/* state registers */
reg			[7:0] 		counter = 0; 				// intrastate counter
reg			[2:0] 		cur_state = ST_IDLE;		// current state
reg			[2:0] 		next_state = ST_IDLE; 	// next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* current error */
assign e_cur				= setpoint - data;

/* z-transform coefficients */
assign k1					= p_coef + i_coef + d_coef;
assign k2					= -p_coef - 2*d_coef;
assign k3					= d_coef;

/* delta u */
assign delta_u				= k1*e_cur + k2*e_prev_0 + k3*e_prev_1;
assign u_cur				= delta_u + u_prev;

/* overflow checking */
assign overflow 			= (e_cur[W_OUT-1] == u_prev[W_OUT-1])  && (u_prev[W_OUT-1] != u_cur[W_OUT-1]);
assign u_cur_clamped		= (u_prev[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;

/* data out */
assign data_out			= (overflow) ? u_cur_clamped : u_cur;
assign data_valid_out	= ( cur_state == ST_SEND );

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data register */
always @ ( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		data <= 0;
	end else if (( data_valid_in == 1 ) & ( cur_state == ST_IDLE )) begin
		data <= data_in;
	end
end

/* previous error and output registers */
always @( posedge clk_in ) begin
	if (( reset_in == 1 ) | ( clear_in == 1 )) begin
		u_prev		<= 0;
		e_prev_0 	<= 0;
		e_prev_1		<= 0;
	end else if ( cur_state == ST_DONE ) begin
		u_prev		<= data_out;
		e_prev_0	<= e_cur;
		e_prev_1	<= e_prev_0;
	end
end

/* frontpanel parameter registers */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		setpoint <= 0;
		p_coef	<= 0;
		i_coef	<= 0;
		d_coef	<= 0;
	end else if (( update_in == 1 ) & ( update_en_in == 1 )) begin
		setpoint	<= setpoint_in;
		p_coef	<= p_coef_in;
		i_coef	<= i_coef_in;
		d_coef	<= d_coef_in;
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
		counter <= counter + 1;
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
		ST_SEND: 									next_state <= ST_DONE;
		ST_DONE: 									next_state <= ST_IDLE;
	endcase
end

endmodule
