`timescale 1ns / 1ps

// dds_controller -- mba 2014

/*
TODO
- add phase and amp update functionality
*/

module dds_controller(
	// inputs <- top level entity
	input wire				clk_in,				// system clock
	input wire				reset_in, 			// system reset

	// inputs <- output preprocessor
	input wire	[47:0]	freq_in,				// frequency data
	input wire	[13:0]	phase_in,			// phase data
	input wire	[9:0]		amp_in,				// amplitude data
	input wire				freq_valid_in,		// frequency data valid signal
	input wire				phase_valid_in,	// phase data valid signal
	input wire				amp_valid_in, 		// amplitude data valid signal

	// outputs -> top level entity (to dds hardware)
	output wire				sclk_out,			// serial clock signal to dds
	output wire				reset_out,			// reset signal to dds
	output reg				csb_out,				// chip select signal to dds
	output wire				sdio_out,			// serial data line to dds
	output wire				io_update_out,		// io update signal to dds

	// outputs -> top level entity
	output wire				dds_done_out 		// pulsed to indicate dds has finished updating
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

parameter UPDATE_LATENCY	= 5;		// dds freq update latency in clock cycles (update latency = 60ns with a 1GHz sysclk)

/* state parameters */
localparam 	ST_IDLE			= 3'd0,	// idle state: wait for new data
				ST_TX 			= 3'd1,	// transmit state: transmit data instruction
				ST_IO_UPDATE	= 3'd2,	// io update state: pulse io_update signal to initiate dds update
				ST_WAIT			= 3'd3,	// wait state: wait for dds update to complete
				ST_DDS_DONE		= 3'd4;	// dds done state: pulse dds_done signal to indicate operation completion

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* registers */
reg	[47:0]	freq = 0;				// active frequency value
reg	[13:0]	phase = 0;				// active phase value
reg	[9:0]		amp = 0;					// active amplitude value

wire	[63:0]	freq_wr_instr;			// frequency write instruction
wire	[31:0] 	phase_wr_instr;		// phase write instruction
wire	[31:0] 	amp_wr_instr;			// amplitude write instruction

reg	[63:0] 	tx_data = 0;			// active data to be sent to dds

/* state registers */
reg	[31:0] 	counter = 0; 				// intrastate counter
reg	[2:0] 	cur_state = ST_IDLE;		// current state
reg	[2:0] 	next_state = ST_IDLE; 	// next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* dds control signals */
assign sclk_out			= clk_in;
assign reset_out			= reset_in;
assign sdio_out			= tx_data[63];
assign io_update_out		= ( cur_state == ST_IO_UPDATE );

/* loop flow control */
assign dds_done_out		= ( cur_state == ST_DDS_DONE );

/* frequency, phase, and amplitude instruction words */
assign freq_wr_instr 	= {1'b0, 2'b11, 13'h01AB, freq};
assign phase_wr_instr	= {1'b0, 2'b01, 13'h01AD, {2'd0, phase_in}};
assign amp_wr_instr		= {1'b0, 2'b01, 13'h040C, {6'd0, amp_in}};

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* latch freq data */
always @( posedge freq_valid_in ) begin
	if ( cur_state == ST_IDLE ) begin
		freq	<= freq_in;
	end
end

/* latch phase data */
always @( posedge phase_valid_in ) begin
	if ( cur_state == ST_IDLE ) begin
		phase	<= phase_in;
	end
end

/* latch amplitude data */
always @( posedge amp_valid_in ) begin
	if ( cur_state == ST_IDLE ) begin
		amp	<= amp_in;
	end
end

/* serial data transmission */
always @( negedge clk_in ) begin
	case ( cur_state )
		ST_IDLE: begin
			tx_data <= 0;
		end
		ST_TX: begin
			if ( counter == 0 ) begin
				tx_data <= freq_wr_instr;
			end else begin
				tx_data <= tx_data << 1;
			end
		end
	endcase
end

/* dds chip select */
always @( negedge clk_in ) begin
	case ( cur_state )
		ST_TX: begin
			if ( counter == 0 ) begin
				csb_out <= 0;
			end else if ( counter == 64 ) begin
				csb_out <= 1;
			end
		end
		default: begin
			csb_out <= 1;
		end
	endcase
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
	case (cur_state)
		ST_IDLE: begin
			if ( freq_valid_in == 1 )			next_state <= ST_TX;
		end
		ST_TX: begin
			if ( counter == 64 ) 				next_state <= ST_IO_UPDATE;
		end
		ST_IO_UPDATE: begin
			if ( counter == UPDATE_LATENCY ) next_state <= ST_WAIT;
		end
		ST_WAIT: begin
			if ( counter == 0 )					next_state <= ST_DDS_DONE;
		end
		ST_DDS_DONE: begin
			if ( counter == 0 ) 					next_state <= ST_IDLE;
		end
	endcase
end

endmodule
