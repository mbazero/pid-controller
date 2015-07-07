`timescale 1ns / 1ps
`include "ep_map.vh"

// -----------------------------------------------------------
// frontpanel interface
// -----------------------------------------------------------
// NOTE: Verilog does not allow 2D input ports, so this module
// is included inline rather than in its own file to eliminate
// substantial packing and unpacking complexity.
//
// This module handles all frontpanel input and output. Input
// data is received on three wire-in channels. The data target
// is defined by an address and channel. The address specifies
// the type of data being received. The channel specifies
// the target PID input or output channel.
//
//  Output data is sent in two modes. Single-word modes sends
// single osf data words across wire-outs (one for each
// input channel). Block mode sends blocks of 1024 osf data
// words at a time. Block mode is only active for one
// channel at a time. The active channel is specified by
// the pipe_chan register.
// -----------------------------------------------------------

module frontpanel_interface #(
    parameter N_LOG = 8,
    parameter W_LCHAN = 5,
    parameter W_LDATA = 18,
    parameter W_EP = 16
    )(
    // Inputs
    input wire                  adc_clk,
    input wire                  sys_clk,

    input wire                  log_dv,
    input wire  [W_LCHAN-1:0]   log_chan,
    input wire  [W_LDATA-1:0]   log_data,

    // Outputs
    output wire                 sys_rst,
    output wire                 adc_cstart,
    output wire                 wr_en,
    output wire                 dac_ref_set;

    output wire [W_EP-1:0]      wr_addr,
    output wire [W_EP-1:0]      wr_chan,
    output wire [W_EP*4-1:0]    wr_data,

    // Frontpanel control
    input wire  [7:0]           hi_in,
    output wire [1:0]           hi_out,
    inout wire  [15:0]          hi_inout,
    inout wire                  hi_aa,

    output wire                 i2c_sda,
    output wire                 i2c_scl,
    output wire                 hi_muxsel,
);

//--------------------------------------------------------------------
// Frontpanel Host Interface
//--------------------------------------------------------------------
wire ticlk;
wire [30:0] ok1,
wire [16:0] ok2,
assign i2c_sda = 1'bz;
assign i2c_scl = 1'bz;
assign hi_muxsel = 1'b0;

okHost hostIf (
    .hi_in          (hi_in),
    .hi_out         (hi_out),
    .hi_inout       (hi_inout),
    .hi_aa          (hi_aa),
    .ti_clk         (ticlk),
    .ok1            (ok1),
    .ok2            (ok2)
    );

//--------------------------------------------------------------------
// General Purpose Triggers
//--------------------------------------------------------------------
wire [W_EP-1:0] gp_trig;
assign sys_rst = gp_trig[sys_rst_offset];
assign adc_cstart = gp_trig[adc_cstart_offset];
assign wr_en = gp_trig[wr_en_offset];
assign dac_ref_set = gp_trig[dac_ref_set_offset];

// System trigger
okTriggerIn sys_gp_oti (
    .ok1            (ok1),
    .ep_addr        (sys_gp_itep),
    .ep_clk         (adc_clk),
    .ep_trigger     (gp_trig)
    );

//--------------------------------------------------------------------
// Memory Write Wire-outs
//--------------------------------------------------------------------
wire [W_EP-1:0] wr_data3, wr_data2, wr_data1, wr_data0;
assign wr_data = {wr_data3, wr_data2, wr_data1, wr_data0};

// Address wire-in
okWireIn addr_owi (
    .ok1            (ok1),
    .ep_addr        (addr_iwep),
    .ep_dataout     (wr_addr)
    );

// Channel wire-in
okWireIn chan_owi (
    .ok1            (ok1),
    .ep_addr        (chan_iwep),
    .ep_dataout     (wr_chan)
    );

// Data wire-ins
okWireIn data3_owi (
    .ok1            (ok1),
    .ep_addr        (data3_iwep),
    .ep_dataout     (wr_data3)
    );

okWireIn data2_owi (
    .ok1            (ok1),
    .ep_addr        (data2_iwep),
    .ep_dataout     (wr_data2)
    );

okWireIn data1_owi (
    .ok1            (ok1),
    .ep_addr        (data1_iwep),
    .ep_dataout     (wr_data1)
    );

okWireIn data0_owi (
    .ok1            (ok1),
    .ep_addr        (data0_iwep),
    .ep_dataout     (wr_data0)
    );

//--------------------------------------------------------------------
// Data Logging Wire-or
//--------------------------------------------------------------------
wire [17*(N_LOG+1)-1:0] ok2x;

okWireOR #(
    .N                  (N_LOG+1))
wireOR (
    .ok2                (ok2),
    .ok2s               (ok2x)
    );

//--------------------------------------------------------------------
// Data Logging Wire-outs
//--------------------------------------------------------------------
reg [W_LDATA-1:0] log_data_reg[0:N_LOG-1];
reg [W_LCHAN-1:0] i;

// Wire-out registers
always @( posedge sys_clk ) begin
    if ( sys_reset == 1'b1 ) begin
        for ( i = 0; i < N_LOG; i = i + 1 ) begin
            log_data_reg[i] <= 0;
        end
    end else begin ( log_dv == 1'b1 ) begin
        log_data_reg[chan] <= log_data;
    end
end

// Wire-outs
genvar j;
generate
for ( j = 0; j < N_LOG; j = j + 1 ) begin : log_owo_arr
    okWireOut log_owo (
        .ok1        (ok1),
        .ok2        (ok2x[j*17 +: 17]),
        .ep_addr    (log_data0_owep + j[W_LCHAN-1:0]),
        .ep_datain  (log_data_reg[j][W_LDATA-1 -: W_EP])
    );
end
endgenerate

//--------------------------------------------------------------------
// Data Logging Pipe-out
//--------------------------------------------------------------------
reg [W_LCHAN-1:0] pipe_chan;
wire [W_EP-1:0] log_pipe_data;
wire log_pipe_dv = ( log_chan == pipe_chan ) ? log_dv : 0);
wire log_pipe_rd;

// Pipe channel write handling
always @( posedge wr_en ) begin
    case ( wr_addr ) begin
        pipe_chan_addr : pipe_chan <= wr_chan;
    end
end

// Pipe-out fifo
pipe_tx_fifo log_pipe_fifo (
    .ti_clk_in      (ticlk),
    .sys_clk_in     (sys_clk),
    .reset_in       (sys_rst),
    .data_valid_in  (log_pipe_dv),
    .data_in        (log_data[W_LDATA-1 -: W_EP]),
    .pipe_read_in   (log_pipe_rd),
    .data_out       (log_pipe_data)
    );

// Pipe-out
okPipeOut log_pipe (
    .ok1                (ok1),
    .ok2                (ok2x[N_LOG*17 +: 17]),
    .ep_addr            (log_data_opep),
    .ep_datain      (log_pipe_data),
    .ep_read            (log_pipe_read)
    );

endmodule


