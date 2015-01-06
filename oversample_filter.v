`timescale 1ns / 1ps

// oversample_filter -- mba 2014

// todo
// - pull channel activation and deactivation out to a seperate module 
//		> (actually if you switch to pipeline it is very likely that activation/deactivation will be handled by an entirely different method)

module oversample_filter #(
	// parameters
	parameter W_IN		= 18,										// width of input data
	parameter W_OUT	= 18,										// width of output data
	parameter W_ORT	= 6										// width of log2 oversample ratio signal
	)(
	// inputs <- top level entity
	input wire								clk_in,				// system clock
	input wire								reset_in,			// system reset

	// inputs <- pid core
	input wire signed		[W_IN-1:0]	data_in,				// input data
	input wire								data_valid_in,		// input data valid signal; asynchronous timing supported

	// inputs <- frontpanel controller
	input wire				[15:0]		cycle_delay_in,	// delay period in adc cycles 
	input wire				[W_ORT-1:0]	log_ovr_in,			// log base 2 of the oversample ratio
	input wire								activate_in,		// channel activation signal (1 = activated, 2 = deactivated) 
	input wire								update_en_in,		// sensistizes module to update signal
	input wire								update_in,			// pulse triggers update of frontpanel parameters

	// outputs -> clk sync
	output wire signed	[W_OUT-1:0]	data_out,			// output data	
	output wire								data_valid_out		// output data valid signal
	);

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* local params */
localparam	MAX_ORT	= 2^W_ORT;	// maximum log2 oversample ratio

/* wires */
wire						idle; 
wire						osf_reset;	// local reset signal which is activated by system reset or channel deactive

/* registers */
reg	[15:0]			cycle_delay;
reg	[MAX_ORT-1:0]	sample_counter;
reg	[W_ORT-1:0]		log_ovr_cur;
reg	[31:0]			sum; 

/* state registers */
reg	[15:0]			counter; 
reg	[2:0]				cur_state;
reg	[2:0]				next_state; 

/* state parameters */
localparam	ST_IDLE			= 3'd0,
				ST_DELAY			= 3'd1,
				ST_SAMPLE		= 3'd2,
				ST_SEND			= 3'd3;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* data out */
assign data_out 			= ( sum >> log_ovr_cur ); 

/* data valid out */
assign data_valid_out 	= ( cur_state == ST_SEND ); 

/* osf reset */
assign osf_reset 			= ( reset_in | ~activate_in );

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* sum */
always @( posedge clk_in ) begin
	if (( cur_state == ST_IDLE ) | ( cur_state == ST_DELAY )) begin
		sum <= 0; 
	end else if (( data_valid_in == 1) & ( cur_state == ST_SAMPLE )) begin
		sum <= sum + data_in;
	end
end 

/* sample counter */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		sample_counter	<= 0;
	end else if ( cur_state != next_state ) begin
		sample_counter	<= 0;
	end else if ( data_valid_in == 1 ) begin
		sample_counter	<= sample_counter + 1'b1;
	end
end

/* frontpanel parameter registers */
always @( posedge clk_in ) begin
	if (( update_in == 1 ) & ( update_en_in == 1 )) begin
		log_ovr_cur 	<= log_ovr_in; 
		cycle_delay		<= cycle_delay_in;
	end
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	sample_counter	= 0;
	cur_state		= 0;
	next_state		= 0;
end

/* state register */
always @( posedge clk_in ) begin 
	if ( reset_in == 1 ) begin
		cur_state <= ST_IDLE;
	end else begin
		cur_state <= next_state; 
	end
end

/* state counter */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		counter <= 0; 
	end else if ( cur_state != next_state ) begin
		counter <= 0; 
	end else begin 
		counter <= counter + 1'b1;
	end
end

/* next state transition logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if not case statement and condition is satisfied
	case ( cur_state ) 
		ST_IDLE: begin
			if ( activate_in == 1 )							next_state <= ST_SAMPLE;
		end
		ST_DELAY: begin
			if ( sample_counter == cycle_delay ) 		next_state <= ST_SAMPLE;
		end
		ST_SAMPLE: begin
			if ( sample_counter[log_ovr_cur] == 1 )	next_state <= ST_SEND;
		end
		ST_SEND: 												next_state <= ST_DELAY;
	endcase
end

endmodule

