`timescale 1ns / 1ps

//--------------------------------------------------------------------
// Dual Fifo -- mba 2015
//--------------------------------------------------------------------
// Dual buffered data logging solution for pipe-out reading. While
// the pipe is reading from one buffer, the other buffer is filling
// up with log data.
//--------------------------------------------------------------------

module dual_fifo (
    // Inputs
    input wire rst,
    input wire rd_clk,
    input wire wr_clk,

    input wire rd_en,
    input wire wr_en,
    input wire [15:0] din,

    // Outputs
    output wire rd_rdy,
    output wire [15:0] dout
    );

//--------------------------------------------------------------------
// Read Counter
//--------------------------------------------------------------------
localparam FIFO_DEPTH = 1023;
reg [15:0] rd_count = 0;
wire rd_in_progess = rd_en || ( rd_count > 0 && rd_count < FIFO_DEPTH );

always @( posedge rd_clk ) begin
    if ( rst ) begin
        rd_count = 0;
    end else if ( rd_en ) begin
        rd_count = rd_count + 1'b1;
    end else if ( rd_count == FIFO_DEPTH-1 ) begin
        rd_count = 0;
    end
end

//--------------------------------------------------------------------
// Dual Buffers
//--------------------------------------------------------------------
wire buf0_wr_en, buf1_wr_en;
wire buf0_rd_en, buf1_rd_en;
wire [15:0] buf0_dout, buf1_dout;
wire buf0_full, buf1_full;

// Write controls
reg wr_target = 0;
always @( posedge wr_clk ) begin
    if ( buf0_full && ( wr_target == 0 )) begin
        wr_target <= 1;
    end else if ( buf1_full ) begin
        wr_target <= 0;
    end
end

assign buf0_wr_en = ( wr_target == 0 ) ? wr_en : 1'b0;
assign buf1_wr_en = ( wr_target == 1 ) ? wr_en : 1'b0;

// Read controls
reg rd_target = 0;
always @( posedge rd_clk ) begin
    if ( !rd_in_progess ) begin
        rd_target = !wr_target;
    end
end

assign buf0_rd_en = ( rd_target == 0 ) ? rd_en : 1'b0;
assign buf1_rd_en = ( rd_target == 1 ) ? rd_en : 1'b0;

pipe_fifo buf0 (
        .rst        (rst),
        .wr_clk     (wr_clk),
        .rd_clk     (rd_clk),
        .din        (din),
        .wr_en      (buf0_wr_en),
        .rd_en      (buf0_rd_en),
        .dout       (buf0_dout),
        .full       (buf0_full)
        );

pipe_fifo buf1 (
        .rst        (rst),
        .wr_clk     (wr_clk),
        .rd_clk     (rd_clk),
        .din        (din),
        .wr_en      (buf1_wr_en),
        .rd_en      (buf1_rd_en),
        .dout       (buf1_dout),
        .full       (buf1_full)
        );

//--------------------------------------------------------------------
// Output Assignment
//--------------------------------------------------------------------
assign rd_rdy = ( rd_target == 0 ) ? buf0_full : buf1_full;
assign dout = ( rd_target == 0 ) ? buf0_dout : buf1_dout;

endmodule
