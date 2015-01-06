`timescale 1ns / 1ps

// cycle_controller -- mba 2014

// TODO 
/* 
This implementation will work for DAC testing, but it is fundamentally flawed and must be changed 
for the final implementation. The fundemental flaw hinges on the assumption that the earliest 
availble ADC channels are mapped to the lower numbered DAC channels. This assumption is simple 
invalid over all usage cases. 

It is suggested that the modified cycle controller use a simple FIFO buffer to recieve write 
requests. Further it is suggested that stall signals be propagated throught the pipeline to 
consistent update periods.  
*/

module cycle_controller #(
	// parameters 
	parameter W_DATA			= 18,								// width of data channels
	parameter W_DAC_INST		= 32,								// width of dac write instruction
	parameter N_DAC			= 6,								// number dac chanels 
	parameter PIPE_LATENCY	= 8								// latency of PID pipeline in clock cycles **NOTE: if this value is too small data will be lost**
	)(
	// inputs <- top level entity
	input wire							clk_in,					// system clock
	input wire							reset_in,				// system reset

	// inputs <- output preprocessor
	input wire	[W_DATA*N_DAC-1:0]bus_data_in,			// input data bus hosting all data channels
	input wire	[N_DAC-1:0]			data_valid_in,			// input data valid signals

	// inputs <- adc controller
	input wire							adc_data_valid_in,	// adc channel 1 data valid signal; used to identify start of a new write cycle

	// inputs <- dac controller
	input wire							dac_done_in,			// dac done signal

	// outputs -> dac controller
	output reg	[W_DATA-1:0]		data_out,				// output data
	output reg	[2:0]					channel_out,			// target channel
	output reg							data_valid_out,		// output data valid
	output reg							dac_stall_out			// stalls dac for one sub cycle for timing purposes
	);

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* wires */
wire	[W_DATA-1:0]	data_in[0:N_DAC-1]; 

/* registers */
reg	[3:0]				send_counter; 
reg	[W_DATA-1:0]	data[0:N_DAC-1];
reg	[N_DAC-1:0]		data_valid; 

/* state registers */
reg	[7:0]				counter;
reg	[2:0]				cur_state; 
reg	[2:0]				next_state;

/* state parameters */
localparam	ST_IDLE			= 3'd0,
				ST_WAIT_PIPE	= 3'd1,
				ST_SEND			= 3'd2,
				ST_WAIT_DAC		= 3'd3;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* send data channels to dac controller at appropriate times */
always @( * ) begin
	if ( cur_state == ST_SEND ) begin
		data_valid_out	= data_valid[send_counter];
		dac_stall_out	= ~data_valid[send_counter];
		data_out			= data[send_counter];
		channel_out		= send_counter;
	end
end

/* split input data bus into seperate channels */
genvar j;
generate
	for ( j = 0; j < N_DAC; j = j + 1 ) begin : data_split
		assign data_in[j] = bus_data_in[ j*W_DATA +: W_DATA ];
	end
endgenerate 


//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data and data valid registers */ 
genvar i;
generate
	for ( i = 0; i < N_DAC; i = i + 1 ) begin : data_reg_arr
		always @( posedge clk_in ) begin
			if ( reset_in == 1 ) begin 
				data[i] 			<= 0;
				data_valid[i]	<= 0; 
			end else if ( data_valid_in[i] == 1 ) begin
				data[i] 			<= data_in[i]; 
				data_valid[i]	<= 1;
			end else if ( cur_state == ST_IDLE ) begin
				data[i] 			<= 0;
				data_valid[i] 	<= 0;
			end
		end
	end
endgenerate 

/* send counter */ 
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		send_counter <= 0;
	end else if ( cur_state == ST_SEND ) begin
		send_counter <= send_counter + 1'b1; 
	end else if ( cur_state == ST_IDLE ) begin 
		send_counter <= 0;
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
			if ( adc_data_valid_in == 1 )		next_state <= ST_WAIT_PIPE;
		end
		ST_WAIT_PIPE: begin
			if ( counter == PIPE_LATENCY ) 	next_state <= ST_SEND;
		end
		ST_SEND: begin
			if ( counter == 0 ) 					next_state <= ST_WAIT_DAC;
		end
		ST_WAIT_DAC: begin
			if ( dac_done_in == 1 ) begin
				if ( send_counter == N_DAC )  begin
					next_state <= ST_IDLE;
				end else begin
					next_state <= ST_SEND;
				end
			end
		end
	endcase
end

endmodule