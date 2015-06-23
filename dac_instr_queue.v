`timescale 1ns / 1ps

// dac_instr_queue -- mba 2014
// -----------------------------------------------------------
// Queues DAC write instructions for the DAC controller which
// can only process one instruction at a time.
// -----------------------------------------------------------

module dac_instr_queue #(
	// parameters
	parameter W_DATA		= 16,									// width of dac data signal
	parameter W_CHS		= 3,									// width of dac channel select signal
	parameter N_CHAN		= 8									// number of DAC channels (must be >1)
	)(
	// inputs <-- top level entity
	input wire								clk_in,				// system clock
	input wire								reset_in,			// system reset

	// inputs <-- DAC output preprocessor
	input wire [W_DATA*N_CHAN-1:0]	data_packed_in,	// input channels packed on a single bus
	input wire [N_CHAN-1:0]				data_valid_in,		// data valid signal

	// inputs <-- dac controller
	input wire								rd_ack_in,			// read acknowledge causes next data word to be presented (if one exists)

	// outputs <-- dac controller
	output wire [W_DATA-1:0]			data_out,			// output data
	output wire	[W_CHS-1:0]				chan_out,			// dac channel associated with output data
	output wire								data_valid_out		// output data valid signal
	);

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam W_DINS = W_DATA + W_CHS;							// width of dac write instruction
localparam N_LOWER = N_CHAN/2;
localparam N_UPPER = N_CHAN - N_LOWER;

/* state parameters */
localparam	ST_IDLE			= 1'd0,							// wait for data valid
				ST_WRITE			= 1'd1;							// write data to FIFO

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

reg	[W_DATA*N_CHAN-1:0]		data_packed_reg = 0;
reg	[N_CHAN-1:0]				data_valid_reg = 0;
wire	[W_DATA-1:0]				mux_dout;
wire									data_valid_rdc;
wire									fifo_wr_en;

/* state registers */
reg	[W_CHS-1:0]					counter = 0;
reg	[2:0]							cur_state = ST_IDLE;
reg	[2:0]							next_state = ST_IDLE;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* reduction or data valid */
assign data_valid_rdc = | data_valid_in;

/* fifo write enable */
assign fifo_wr_en = (cur_state == ST_WRITE) ? data_valid_reg[counter] : 1'b0;

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* input data registers */
always @( posedge clk_in ) begin
	if ( reset_in ) begin
		data_packed_reg	<= 0;
		data_valid_reg		<= 0;
	end else if ( data_valid_rdc ) begin
		data_packed_reg	<= data_packed_in;
		data_valid_reg		<= data_valid_in;
	end
end

/* channel counter */
always @( posedge clk_in ) begin
	if ( reset_in == 1 | cur_state == ST_IDLE ) begin
		counter <= 0;
	end else begin
		counter <= counter + 1'b1;
	end
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* channel multiplexer */
mux_n_chan #(
	.W_CHAN				(W_DATA),
	.W_SEL				(W_CHS),
	.N_IN					(N_CHAN))
mux (
	.data_packed_in	(data_packed_reg),
	.chan_select_in	(counter),
	.enable_in			(cur_state == ST_WRITE),
	.data_out			(mux_dout)
	);

/* fifo */
fifo_19 instr_queue (
	.clk		(clk_in),
	.rst		(reset_in),
	.din		({counter, mux_dout}),
	.wr_en	(fifo_wr_en),
	.rd_en	(rd_ack_in),
	.dout		({chan_out, data_out}),
	.full		(),
	.empty	(),
	.valid	(data_valid_out)
	);

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

/* next state combinational logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if no case and condition is satisfied
	case ( cur_state )
		ST_IDLE: begin
			if ( data_valid_rdc == 1 )			next_state <= ST_WRITE;
		end
		ST_WRITE: begin
			if ( counter == N_CHAN-1 )			next_state <= ST_IDLE;
		end
	endcase
end

endmodule
