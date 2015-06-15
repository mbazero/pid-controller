`timescale 1ns / 1ps

// adc_controller -- mba 2014
// runs AD7608 in continuous conversion mode

// TODO
// - AD7608 datasheet lists a maximum time from the last CS rising edge to busy falling
// edge. I don't think this time is satisfied for large oversample ratios.
// Investigate why this time restriction exists and perhaps change CS start
// time if it's important.

module adc_controller #(
	// parameters
	parameter W_OUT			= 18,									// width of adc data channels
	parameter N_CHAN			= 8,									// number of adc channels to be read, oversample rate must be >=2 to support 8 channels
	parameter MIN_T_CYCLE	= 85									// minimum cycle time in number of adc clock cycles
	)(
	// inputs <- top level entity
	input wire						clk_in,							// adc serial clock; max frequency 17mhz
	input wire						reset_in, 						// system reset

	// inputs <- AD7608
	input wire						busy_in,							// conversion busy signal
	input wire						data_a_in,						// serial data channel a
	input wire						data_b_in,						// serial data channel b

	// inputs <- frontpanel controller
	input wire	[2:0]				os_in,							// sets adc oversampling mode
	input wire						cstart_in,						// pulse starts continuous adc conversion cycle

	// outputs -> AD7608
	output wire	[2:0]				os_out,							// oversampling signal to adc
	output wire						convst_out,						// convert start signal to adc
	output wire						reset_out,						// reset signal to adc
	output wire						sclk_out, 						// serial clock signal to adc
	output wire						n_cs_out,						// chip select signal to adc

	// outputs -> pid core
	output wire				[N_CHAN-1:0]	data_valid_out,	// one-hot encoded output data valid signal
	output reg signed		[W_OUT-1:0]		data_a_out,			// channel a data out
	output reg signed		[W_OUT-1:0]		data_b_out			// channel b data out
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam RD_LENGTH		= W_OUT*(N_CHAN/2);					// bits of data to be read per serial port in a single cycle
localparam OS_MIN			= 3'b1;									// set minimum oversampling ratio of 2^1 to support 8 channels //DEBUG set back to 1 after testing

/* state parameters */
localparam	CV_ST_IDLE		= 3'd0,								// wait for module enable signal to begin continuous conversion
				CV_ST_CONVST 	= 3'd1,								// pulse convert start signal to begin conversion
				CV_ST_CONV		= 3'd2;								// wait for ADC to finish conversion

localparam	RD_ST_IDLE		= 3'd0,								// wait for busy signal to begin read
				RD_ST_READ		= 3'd1,								// read adc data off serial lines
				RD_ST_WAIT		= 3'd2;								// wait for busy signal to deassert

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* state registers */
reg	[7:0] 				cv_counter = 0; 						// convert state machine counter
reg	[2:0] 				cv_cur_state = CV_ST_IDLE;			// convert state machine current state
reg	[2:0] 				cv_next_state = CV_ST_IDLE;		// convert state machine next state

reg	[7:0]					rd_counter = 0;						// read state machine counter
reg	[2:0]					rd_cur_state = RD_ST_IDLE;			// read state machine current state
reg	[2:0]					rd_next_state = RD_ST_IDLE;		// read state machine next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* adc control */
assign os_out 		= (os_in < OS_MIN) ? OS_MIN : os_in;	// enforce minimum oversample rate
assign reset_out	= reset_in;

/* data valid out */
genvar i;
generate
	for ( i = 0; i < N_CHAN/2; i = i+1 ) begin : data_out_arr
		assign data_valid_out[i] 				= (( rd_cur_state == RD_ST_READ ) & ( rd_counter == W_OUT*(i+1) ));
		assign data_valid_out[i+N_CHAN/2]	= (( rd_cur_state == RD_ST_READ ) & ( rd_counter == W_OUT*(i+1) ));
	end
endgenerate

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* serial read shift register */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		data_a_out <= 0;
		data_b_out <= 0;
	end else if ( n_cs_out == 0 ) begin
		data_a_out <= {data_a_out[W_OUT-2:0], data_a_in};
		data_b_out <= {data_b_out[W_OUT-2:0], data_b_in};
	end
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc sclk forwarding buffer */
ODDR2 #(
	.DDR_ALIGNMENT	("NONE"),
	.INIT				(1'b1),
	.SRTYPE			("SYNC")
) adc_clk_fwd (
	.Q					(sclk_out),
	.C0				(clk_in),
	.C1				(~clk_in),
	.CE				(1'b1),
	.D0				(1'b1), // VCC
	.D1				(1'b0), // GND
	.R					(reset_in),
	.S					(n_cs_out)
);

/*
* Data reading and conversion must happen concurrently in order to acheive
* max throughput. Toward this end, the ADC controller has two independent
* state machines, one to handle data conversion and one to handle serial
* data reading.
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
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		cv_cur_state <= CV_ST_IDLE;
	end else begin
		cv_cur_state <= cv_next_state;
	end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
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
			if (( cv_counter >= MIN_T_CYCLE ) & ( busy_in == 0 ))
				cv_next_state <= CV_ST_CONVST;
		end
	endcase
end

/* fsm outputs */
assign convst_out = ~( cv_cur_state == CV_ST_CONVST );

//////////////////////////////////////////
// read state machine
//////////////////////////////////////////

/* state sequential logic */
always @( posedge clk_in ) begin
	if ( reset_in == 1 ) begin
		rd_cur_state <= RD_ST_IDLE;
	end else begin
		rd_cur_state <= rd_next_state;
	end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
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
