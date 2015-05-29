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

/* pid products */
wire signed [W_OUT-1:0] prod1_pre, prod2_pre, prod3_pre;
wire signed [W_OUT-1:0] prod1_clamped, prod2_clamped, prod3_clamped;
wire signed [W_OUT-1:0] prod1, prod2, prod3;
wire							prod1_overflow, prod2_overflow, prod3_overflow;

/* pid sums */
wire signed [W_OUT-1:0] sum1_pre, sum2_pre;
wire signed [W_OUT-1:0] sum1_clamped, sum2_clamped;
wire signed [W_OUT-1:0] sum1, sum2;
wire							sum1_overflow, sum2_overflow;

/* current pid output */
wire signed	[W_OUT-1:0] u_cur_pre;
wire signed [W_OUT-1:0]	u_cur_clamped;
wire signed [W_OUT-1:0] u_cur;
wire 							u_cur_overflow;

/* previous pid output */
reg signed	[W_OUT-1:0]	u_prev = 0;			// previous pid filter output

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

/* compute products */
assign prod1_pre			= k1 * e_cur;
assign prod2_pre			= k2 * e_prev_0;
assign prod3_pre			= k3 * e_prev_1;

/* overflow check products */
assign prod1_overflow	= (prod1_pre[W_OUT-1] != e_cur[W_OUT-1]);
assign prod2_overflow	= (prod2_pre[W_OUT-1] != e_prev_0[W_OUT-1]);
assign prod3_overflow	= (prod3_pre[W_OUT-1] != e_prev_1[W_OUT-1]);

assign prod1_clamped		= (e_cur[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign prod2_clamped		= (e_prev_0[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign prod3_clamped		= (e_prev_1[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;

assign prod1 				= (prod1_overflow) ? prod1_clamped : prod1_pre;
assign prod2				= (prod2_overflow) ? prod2_clamped : prod2_pre;
assign prod3				= (prod3_overflow) ? prod3_clamped : prod3_pre;

/* compute first pid sum */
assign sum1_pre			= prod1 + prod2;

/* overflow check first pid sum */
assign sum1_overflow		= (prod1[W_OUT-1] == prod2[W_OUT-1]) && (sum1_pre[W_OUT-1] != prod1[W_OUT-1]);
assign sum1_clamped		= (prod1[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign sum1					= (sum1_overflow) ? sum1_clamped : sum1_pre;

/* compute second pid sum */
assign sum2_pre			= sum1 + prod3;

/* overflow check second pid sum */
assign sum2_overflow		= (sum1[W_OUT-1] == prod3[W_OUT-1]) && (sum2_pre[W_OUT-1] != sum1[W_OUT-1]);
assign sum2_clamped		= (sum1[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign sum2					= (sum2_overflow) ? sum2_clamped : sum2_pre;

/* compute new output */
assign u_cur_pre			= sum2 + u_prev;

/* overflow check new output */
assign u_cur_overflow 	= (sum2[W_OUT-1] == u_prev[W_OUT-1])  && (u_prev[W_OUT-1] != u_cur_pre[W_OUT-1]);
assign u_cur_clamped		= (u_prev[W_OUT-1] == 0) ? MAX_OUTPUT : MIN_OUTPUT;
assign u_cur				= (u_cur_overflow) ? u_cur_clamped : u_cur_pre;

/* data out */
assign data_out			= u_cur;
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
		u_prev	<= 0;
		e_prev_0	<= 0;
		e_prev_1	<= 0;
	end else if ( cur_state == ST_DONE ) begin
		u_prev	<= data_out;
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
