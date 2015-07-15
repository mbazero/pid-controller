`timescale 1ns / 1ps

// pipe_tx_fifo -- mba13

module pipe_tx_fifo (
    // Inputs
    input wire rd_clk,
    input wire wr_clk,
    input wire rst,

    input wire wr_en,
    input wire [15:0] din,
    input wire rd_en,

    // Outputs
    output wire rd_ready,
    output wire [15:0] dout
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* block read length */
localparam READ_LEN = 1024;

/* fifo fires */
wire fifo_rd_en;
wire fifo_almost_full;
wire fifo_half_full;
wire fifo_half_empty;
wire [15:0] fifo_dout;

/* read counter */
reg [11:0] rd_count;

/* state registers */
reg [2:0] cur_state;
reg [2:0] next_state;

/* state parameters */
localparam  ST_WAIT = 3'd0,
            ST_READ = 3'd1;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

assign rd_ready = !fifo_half_empty;
assign fifo_rd_en = ( fifo_half_full && ( cur_state == ST_WAIT )) || rd_en || fifo_almost_full;

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* count cycles that pipe_read is asserted */
always @( posedge rd_clk ) begin
    if ( rst == 1 ) begin
        rd_count <= 0;
    end else if ( cur_state == ST_READ ) begin
        if ( rd_en == 1 ) begin
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
        .rst        (rst),
        .wr_clk     (wr_clk),
        .rd_clk     (rd_clk),
        .din        (din),
        .wr_en      (wr_en),
        .rd_en      (fifo_rd_en),
        .dout       (dout),
        .full       (),
        .almost_full(fifo_almost_full),
        .prog_full  (fifo_half_full),
        .prog_empty (fifo_half_empty),
        .empty      ()
        );

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
    rd_count = 0;
    cur_state = ST_WAIT;
    next_state = ST_WAIT;
end

/* state sequential logic */
always @( posedge rd_clk ) begin
    if ( rst == 1 ) begin
        cur_state <= ST_WAIT;
    end else begin
        cur_state <= next_state;
    end
end

/* next state combinational logic */
always @( * ) begin
    next_state <= cur_state; // default assignment
    case ( cur_state )
        ST_WAIT: begin
            if ( rd_en == 1 ) next_state <= ST_READ;
        end
        ST_READ: begin
            if ( rd_count == READ_LEN-1 ) next_state <= ST_WAIT;
        end
    endcase
end


endmodule
