`timescale 1ns / 1ps
`include "parameters.vh"

//--------------------------------------------------------------------
// PID Pipeline
//--------------------------------------------------------------------
// PID processing pipeline
//--------------------------------------------------------------------

module pid_pipeline #(
    // Parameters
    parameter W_SRC = 5,        // Width of source select signal
    parameter N_CHAN = 5,       // Number of output channels
    parameter W_CHAN = 5,       // Width of output select signal
    parameter W_DIN = 18,       // Width of source input data
    parameter W_DOUT = 64,      // Width of output data
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
    output wire dv_out,
    output wire [log2(N_CHAN)-1:0] chan_out,
    output wire signed [W_DOUT-1:0] data_out
    );

//--------------------------------------------------------------------
// Instruction Dispatch
//--------------------------------------------------------------------
wire idp_dv;
wire [W_CHAN-1:0] idp_chan;
wire [W_DIN-1:0] idp_data;

instr_dispatch #(
    .W_SRC          (W_SRC),
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN),
    .W_DATA         (W_DIN),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
idp #(
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (dv_in),
    .src_in         (src_in),
    .data_in        (data_in),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (idp_dv),
    .chan_out       (idp_chan),
    .data_out       (idp_data)
);

//--------------------------------------------------------------------
// Oversample Filter
//--------------------------------------------------------------------
wire osf_dv;
wire [W_CHAN-1:0] osf_chan;
wire [W_DIN-1:0] osf_data;

oversample_filter #(
    .W_CHAN         (W_CHAN),
    .W_DATA         (W_DIN),
    .W_SUM          (W_COMP),
    .W_COUNT        (W_COMP),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
osf (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (idp_dv),
    .dest_in        (idp_dest),
    .data_in        (idp_data),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (osf_dv),
    .chan_out       (osf_chan),
    .data_out       (osf_data)
);

//--------------------------------------------------------------------
// PID Filter
//--------------------------------------------------------------------
wire pid_dv;
wire [W_CHAN-1:0] pid_chan;
wire [W_COMP-1:0] pid_data;

pid_filter #(
    .W_CHAN         (W_CHAN),
    .W_DIN          (W_DIN),
    .W_DOUT         (W_COMP),
    .W_PID_COEFS    (W_OPRNDS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
pid (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (osf_dv),
    .chan_in        (osf_chan),
    .data_in        (osf_data),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (pid_dv),
    .chan_out       (pid_chan),
    .data_out       (pid_data)
);

//--------------------------------------------------------------------
// Output Filtering
//--------------------------------------------------------------------
wire opf_dv;
wire [W_CHAN-1:0] opf_chan;
wire [W_COMP-1:0] opf_data;

output_filter #(
    .W_CHAN         (W_CHAN),
    .W_DIN          (W_COMP),
    .W_DOUT         (W_DOUT),
    .W_MULT         (W_OPRNDS),
    .W_RS           (W_OPRNDS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
opf (
    .clk_in         (clk_in),
    .rst_in         (rst_in),
    .dv_in          (pid_dv),
    .chan_in        (pid_chan),
    .data_in        (pid_data),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_out         (opf_dv),
    .chan_out       (opf_chan),
    .data_out       (opf_data)
);

// Output assignment
assign dv_out = opf_dv;
assign chan_out = opf_chan;
assign data_out = opf_data;

endmodule
