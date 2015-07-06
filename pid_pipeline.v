`timescale 1ns / 1ps
`include "parameters.vh"

//--------------------------------------------------------------------
// PID Pipeline
//--------------------------------------------------------------------
// PID processing pipeline
//--------------------------------------------------------------------

module pid_pipeline #(
    // Parameters
    parameter W_SRC = 5,
    parameter W_CHAN = 8,
    parameter N_CHAN = 5,
    parameter W_DATA_IN = 18,
    parameter W_DATA_OUT = 64,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48,
    parameter W_COMP = 128,
    parameter W_OSF_OS = 4,
    parameter W_OPP_MULT = 8,
    parameter W_OPP_RS = 8
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_CHAN-1:0] src_in,
    input wire signed [W_DATA-1:0] data_in,

    input wire wr_en,
    input wire [W_WR_ADDR-1:0] wr_addr,
    input wire [W_WR_CHAN-1:0] wr_chan,
    input wire [W_WR_DATA-1:0] wr_data,

    // Outputs
    output wire dv_out,
    output wire [W_CHAN-1:0] chan_out,
    output wire signed [W_DATA-1:0] data_out
    );

//--------------------------------------------------------------------
// Instruction Dispatch
//--------------------------------------------------------------------
wire idp_dv;
wire [W_CHAN-1:0] idp_chan;
wire [W_DATA_IN-1:0] idp_data;
wire [N_CHAN-1:0] idp_chan_en;

instr_dispatch #(
    .W_SRC          (W_SRC),
    .W_DATA         (W_DATA_IN),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA),
    .W_CHAN         (W_CHAN),
    .N_CHAN         (N_CHAN))
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
    .data_out       (idp_data),
    .chan_en_out    (idp_chan_en)
);

// Channel reset
wire [N_CHAN-1:0] chan_rst = ~idp_chan_en;

//--------------------------------------------------------------------
// Oversample Filter
//--------------------------------------------------------------------
wire osf_dv;
wire [W_CHAN-1:0] osf_chan;
wire [W_DATA_IN-1:0] osf_data;

oversample_filter #(
    .W_CHAN         (W_CHAN),
    .W_DATA         (W_DATA_IN),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA),
    .W_SUM          (W_COMP),
    .W_OS           (W_OSF_OS))
osf (
    .clk_in         (clk_in),
    .sys_rst_in     (rst_in),
    .chan_rst_in    (chan_rst),
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
    .W_DATA_IN      (W_DATA_IN),
    .W_DATA_OUT     (W_COMP),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
pid (
    .clk_in         (clk_in),
    .sys_rst_in     (rst_in),
    .chan_rst_in    (chan_rst),
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
    .W_DATA_IN      (W_COMP),
    .W_DATA_OUT     (W_DATA_OUT),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA),
    .W_MULT         (W_OPF_MULT),
    .W_RS           (W_OPF_RS))
opf (
    .clk_in         (clk_in),
    .sys_rst_in     (rst_in),
    .chan_rst_in    (chan_rst),
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
