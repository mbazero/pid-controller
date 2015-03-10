`timescale 1ns / 1ps

// adc_controller -- mba 2014

// TODO
// - CHANGE ALL FP PARAM REGISTERS TO LATCH ON POSEDGE OF UPDATE SIGNAL
// - figure out adc channel mapping

module adc_controller #(
	// parameters
	parameter W_OUT	= 18,								// width of adc data channels
	parameter N_CHAN	= 6,								// number of adc channels to be read (max = 8)
	parameter T_CYCLE	= 85								// conversion cycle time in number of adc clock cycles
	/* adc data sheet suggests a 5us cycle time. max adc clock rate is 17MHz. 5us*17M = 85 */
	)(
	// inputs <- top level entity
	input wire						clk_in,				// ADC serial clock; max frequency 17MHz
	input wire						reset_in, 			// system reset

	// inputs <- top level entity (from adc hardware)
	input wire						busy_in,				// conversion busy signal
	input wire						data_a_in,			// serial data channel a
	input wire						data_b_in,			// serial data channel b

	// inputs <- frontpanel controller
	input wire	[2:0]				os_in,				// sets adc oversampling mode
	input wire						update_in,			// pulse triggers update of frontpanel parameters
	input wire						cstart_in,			// pulse starts continuous adc conversion cycle

	// outputs -> top level entity (to adc hardware)
	output wire	[2:0]				os_out,				// oversampling signal to adc
	output wire						convst_out,			// convert start signal to adc
	output wire						reset_out,			// reset signal to adc
	output wire						sclk_out, 			// serial clock signal to adc
	output wire						n_cs_out,			// chip select signal to adc

	// outputs -> pid core
	output wire				[N_CHAN-1:0]	data_valid_out,	// one-hot encoded output data valid signal
	output reg signed		[W_OUT-1:0]		data_a_out,			// channel a data out
	output reg signed		[W_OUT-1:0]		data_b_out			// channel b data out
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* parameters */
localparam RD_LENGTH		= W_OUT*(N_CHAN/2);		// bits of data to be read per serial port in a single cycle

/* registers */
reg	[2:0]					os_cur;			// active oversampling mode

/* state registers */
reg	[7:0] 				cv_counter; 	// convert state machine counter
reg	[2:0] 				cv_cur_state;	// convert state machine current state
reg	[2:0] 				cv_next_state;	// convert state machine next state

reg	[7:0]					rd_counter;		// read state machine counter
reg	[2:0]					rd_cur_state;	// read state machine current state
reg	[2:0]					rd_next_state;	// read state machine next state

/* state parameters */
localparam	CV_ST_IDLE		= 3'd1,		// wait for module enable signal to begin continuous conversion
				CV_ST_CONVST 	= 3'd2,		// pulse convert start signal to begin conversion
				CV_ST_CONV		= 3'd3;		// wait for ADC to finish conversion

localparam	RD_ST_IDLE		= 3'd1,		// wait for busy signal to begin read
				RD_ST_READ		= 3'd2,		// read adc data off serial lines
				RD_ST_WAIT		= 3'd3;		// wait for busy signal to deassert


//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* adc control */
assign os_out 				= os_cur;
assign reset_out 			= reset_in;
assign sclk_out			= n_cs_out | clk_in;	// only pass serial clock to adc when chip select goes low

/* data valid out */
genvar i;
generate
	for ( i = 0; i < N_CHAN/2; i = i+1 ) begin : data_out_arr
		assign data_valid_out[i] 		= (( rd_cur_state == RD_ST_READ ) & ( rd_counter == W_OUT*(i+1) ));
		assign data_valid_out[i+N_CHAN/2]		= (( rd_cur_state == RD_ST_READ ) & ( rd_counter == W_OUT*(i+1) ));
	end
endgenerate

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* oversampling mode register */
always @( posedge update_in or posedge reset_in ) begin
	if ( reset_in == 1 )
		os_cur <= 0;
	else if ( update_in == 1 )
		os_cur <= os_in;
end

/* serial read shift register */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		data_a_out <= 0;
		data_b_out <= 0;
	end else if ( n_cs_out == 0 ) begin
		data_a_out <= {data_a_out[W_OUT-2:0], data_a_in};
		data_b_out <= {data_b_out[W_OUT-2:0], data_b_in};
	end
end

/*
* Data reading and conversion must happen concurrently in order to acheive
* max throughput of 200ksps. Toward this end, the ADC controller has two
* independent state machines, one to handle data conversion and one to
* handle serial data reading.
*/

//////////////////////////////////////////
// convert state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	cv_counter		= 0;
	cv_cur_state 	= CV_ST_IDLE;
	cv_next_state 	= CV_ST_IDLE;
end

/* state sequential logic */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		cv_cur_state <= CV_ST_IDLE;
	end else begin
		cv_cur_state <= cv_next_state;
	end
end

/* state counter sequential logic */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		cv_counter <= 0;
	end else if ( cv_cur_state != cv_next_state ) begin
		cv_counter <= 0;
	end else begin
		cv_counter <= cv_counter + 1'b1;
	end
end

/* next state combinational logic */
always @( * ) begin
	cv_next_state <= cv_cur_state; // default assignment if no case and condition is satisfied
	case ( cv_cur_state )
		CV_ST_IDLE: begin
			if ( cstart_in == 1 )
				cv_next_state <= CV_ST_CONVST;
		end
		CV_ST_CONVST: begin
				cv_next_state <= CV_ST_CONV;
		end
		CV_ST_CONV: begin
			if (( cv_counter >= T_CYCLE ) & ( busy_in == 0 ))
				cv_next_state <= CV_ST_CONVST;
		end
	endcase
end

/* fsm outputs */
assign convst_out = ~( cv_cur_state == CV_ST_CONVST );

//////////////////////////////////////////
// read state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	rd_counter		= 0;
	rd_cur_state 	= RD_ST_IDLE;
	rd_next_state 	= RD_ST_IDLE;
end

/* state sequential logic */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		rd_cur_state <= RD_ST_IDLE;
	end else begin
		rd_cur_state <= rd_next_state;
	end
end

/* state counter sequential logic */
always @( posedge clk_in or posedge reset_in ) begin
	if ( reset_in == 1 ) begin
		rd_counter <= 0;
	end else if ( rd_cur_state != rd_next_state ) begin
		rd_counter <= 0;
	end else begin
		rd_counter <= rd_counter + 1'b1;
	end
end

/* next state combinational logic */
always @( * ) begin
	rd_next_state <= rd_cur_state; // default assignment if no case and condition is satisfied
	case ( rd_cur_state )
		RD_ST_IDLE: begin
			if ( busy_in == 1 ) 					rd_next_state <= RD_ST_READ;
		end
		RD_ST_READ: begin
			if ( rd_counter == RD_LENGTH )	rd_next_state <= RD_ST_WAIT;
		end
		RD_ST_WAIT: begin
			if ( busy_in == 0 )					rd_next_state <= RD_ST_IDLE;
		end
	endcase
end

/* fsm outputs */
assign n_cs_out = ~(( rd_cur_state == RD_ST_READ ) & ( rd_counter < RD_LENGTH ));

endmodule
