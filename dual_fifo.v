`timescale 1ns / 1ps

//--------------------------------------------------------------------
// Dual Fifo -- mba 2015
//--------------------------------------------------------------------
// Dual buffered data logging solution for pipe-out reading. While
// the pipe is reading from one buffer, the other buffer is filling
// up with log data.
//--------------------------------------------------------------------

module dual_fifo #(
    parameter READ_LEN = 1024,
    parameter N_ADC = 6,
    parameter OK2X_LEN = 17*(N_ADC+1),
    parameter PIPE_ADDR = 8'ha3
    )(
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
// Dual Buffers
//--------------------------------------------------------------------
reg wr_buf = 0;
wire rd_buf;
wire buf0_wr_en, buf1_wr_en;
wire buf0_rd_en, buf1_rd_en;
wire [15:0] buf0_dout, buf1_dout;
wire buf0_full, buf1_full;

// Write controls
always @( posedge wr_clk ) begin
    if ( buf0_full && ( wr_buf == 0 )) begin
        wr_buf <= 1;
    end else if ( buf1_full ) begin
        wr_buf <= 0;
    end
end

assign buf0_wr_en = ( wr_buf == 0 ) ? wr_en : 1'b0;
assign buf1_wr_en = ( wr_buf == 1 ) ? wr_en : 1'b0;

// Read controls
assign rd_buf = !wr_buf;

assign buf0_rd_en = ( rd_buf == 0 ) ? rd_en : 1'b0;
assign buf1_rd_en = ( rd_buf == 1 ) ? rd_en : 1'b0;

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
assign rd_rdy = ( rd_buf == 0 ) ? buf0_full : buf1_full;
assign dout = ( rd_buf == 0 ) ? buf0_dout : buf1_dout;

endmodule
