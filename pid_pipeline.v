`timescale 1ns / 1ps

//--------------------------------------------------------------------
// PID Pipeline
//--------------------------------------------------------------------
// PID processing pipeline
//--------------------------------------------------------------------

module pid_pipeline #(
    parameter N_SRC = 8,        // Number of source channels
    parameter W_SRC = 5,        // Width of source select signal
    parameter N_CHAN = 5,       // Number of output channels
    parameter W_CHAN = 5,       // Width of output select signal
    parameter W_DIN = 18,       // Width of source input data
    parameter W_DOUT = 64,      // Width of output data
    parameter W_OS = 7,         // Width of oversample mode signal
    parameter W_COMP = 128,     // Width of internal computation registers
    parameter W_OPRNDS = 16,    // Width of operands
    parameter W_WR_ADDR = 16,   // Width of memory write address
    parameter W_WR_CHAN = 16,   // Width of memory write channel
    parameter W_WR_DATA = 48    // Width of memory write data
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_SRC-1:0] src_in,
    input wire signed [W_DIN-1:0] data_in,

    input wire wr_en,
    input wire [W_WR_ADDR-1:0] wr_addr,
    input wire [W_WR_CHAN-1:0] wr_chan,
    input wire [W_WR_DATA-1:0] wr_data,

    // Outputs
    output wire dv_ovr,
    output wire [W_CHAN-1:0] chan_ovr,
    output wire [W_DIN-1:0] data_ovr,

    output wire dv_out,
    output wire [W_CHAN-1:0] chan_out,
    output wire signed [W_DOUT-1:0] data_out
    );

//--------------------------------------------------------------------
// Instruction Dispatch
//--------------------------------------------------------------------
wire dv_idp;
wire [W_CHAN-1:0] chan_idp;
wire [W_DIN-1:0] data_idp;

instr_dispatch #(
    .W_SRC          (W_SRC),
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN),
    .W_DATA         (W_DIN),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
idp (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (dv_in),
    .src_in         (src_in),
    .data_in        (data_in),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (dv_idp),
    .chan_out       (chan_idp),
    .data_out       (data_idp)
);

//--------------------------------------------------------------------
// Oversample Filter
//--------------------------------------------------------------------
oversample_filter #(
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN),
    .W_DATA         (W_DIN),
    .W_SUM          (W_COMP),
    .W_OS           (W_OS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
ovr (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (dv_idp),
    .chan_in        (chan_idp),
    .data_in        (data_idp),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (dv_ovr),
    .chan_out       (chan_ovr),
    .data_out       (data_ovr)
);

//--------------------------------------------------------------------
// PID Filter
//--------------------------------------------------------------------
wire dv_pid;
wire [W_CHAN-1:0] chan_pid;
wire [W_COMP-1:0] data_pid;

pid_filter #(
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN),
    .W_DIN          (W_DIN),
    .W_DOUT         (W_COMP),
    .W_PID_COEFS    (W_OPRNDS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
pid (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (dv_ovr),
    .chan_in        (chan_ovr),
    .data_in        (data_ovr),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (dv_pid),
    .chan_out       (chan_pid),
    .data_out       (data_pid)
);

//--------------------------------------------------------------------
// Output Filtering
//--------------------------------------------------------------------
wire dv_opt;
wire [W_CHAN-1:0] chan_opt;
wire [W_DOUT-1:0] data_opt;

output_filter #(
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN),
    .W_DELTA        (W_COMP),
    .W_DOUT         (W_DOUT),
    .W_MULT         (W_OPRNDS),
    .W_RS           (W_OPRNDS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
opt (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (dv_pid),
    .chan_in        (chan_pid),
    .delta_in       (data_pid),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (dv_opt),
    .chan_out       (chan_opt),
    .data_out       (data_opt)
);

// Output assignment
assign dv_out = dv_opt;
assign chan_out = chan_opt;
assign data_out = data_opt;

endmodule
