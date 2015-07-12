`timescale 1ns / 1ps

// pipe_tx_fifo -- mba13

module pipe_tx_fifo #(
	// parameters
	parameter READ_LEN	= 1024,
	parameter N_ADC		= 6,
	parameter OK2X_LEN	= 17*(N_ADC+1),
	parameter PIPE_ADDR	= 8'ha3
	)(
	// inputs <- top level entity
	input wire										ti_clk_in,
	input wire										pid_clk_in,
	input wire										rst_in,

	// inputs
	input wire						 				data_valid_in,
	input wire 				[15:0] 				data_in,
	input wire										pipe_read_in,

	// outputs
	output wire				[15:0]				data_out
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* wires */
wire fifo_almost_full, fifo_half_full, fifo_rd_en;
wire pipe_read;
wire [15:0] fifo_dout;

/* read counter */
reg	[11:0]	rd_count;

/* state registers */
reg	[2:0] 	cur_state;
reg	[2:0] 	next_state;

/* state parameters */
localparam 	ST_WAIT 	= 3'd0,
				ST_READ	= 3'd1;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

assign fifo_rd_en = (fifo_half_full && (cur_state == ST_WAIT)) || pipe_read_in || fifo_almost_full;

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* count cycles that pipe_read is asserted */
always @( posedge ti_clk_in ) begin
	if ( rst_in == 1 ) begin
		rd_count <= 0;
	end else if ( cur_state == ST_READ ) begin
		if ( pipe_read_in == 1 ) begin
			rd_count <= rd_count + 1'b1;
		end else begin
			rd_count <= rd_count;
		end
	end else begin
		rd_count <= 0;
	end
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

pipe_fifo pipe_buf (
		.rst			(rst_in),
		.wr_clk		(pid_clk_in),
		.rd_clk		(ti_clk_in),
		.din			(data_in),
		.wr_en		(data_valid_in),
		.rd_en		(fifo_rd_en),
		.dout			(data_out),
		.full			(),
		.almost_full(fifo_almost_full),
		.prog_full	(fifo_half_full),
		.empty		()
		);

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
	rd_count		= 0;
	cur_state 	= ST_WAIT;
	next_state 	= ST_WAIT;
end

/* state sequential logic */
always @( posedge ti_clk_in ) begin
	if ( rst_in == 1 ) begin
		cur_state <= ST_WAIT;
	end else begin
		cur_state <= next_state;
	end
end

/* next state combinational logic */
always @( * ) begin
	next_state <= cur_state; // default assignment if no case and condition is satisfied
	case ( cur_state )
		ST_WAIT: begin
			if ( pipe_read_in == 1 )		next_state <= ST_READ;
		end
		ST_READ: begin
			if ( rd_count == READ_LEN-1 )	next_state <= ST_WAIT;
		end
	endcase
end


endmodule
