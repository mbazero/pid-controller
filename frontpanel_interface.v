`timescale 1ns / 1ps

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
// Output data is sent in two modes. Single-word modes sends
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
    parameter W_EP = 16,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 5,
    parameter W_WR_DATA = 49
    )(
    // Inputs
    input wire adc_clk_in,
    input wire pid_clk_in,

    input wire log_dv_in,
    input wire [W_LCHAN-1:0] log_chan_in,
    input wire [W_LDATA-1:0] log_data_in,

    // Outputs
    output wire sys_rst_out,
    output wire adc_cstart_out,
    output wire dac_rset_out,

    output wire wr_en_out,
    output wire [W_WR_ADDR-1:0] wr_addr_out,
    output wire [W_WR_CHAN-1:0] wr_chan_out,
    output wire [W_WR_DATA-1:0] wr_data_out,

    // Frontpanel control
    input wire [7:0] hi_in,
    output wire [1:0] hi_out,
    inout wire [15:0] hi_inout,
    inout wire hi_aa,

    output wire i2c_sda,
    output wire i2c_scl,
    output wire hi_muxsel
);

`include "ep_map.vh"

//--------------------------------------------------------------------
// Frontpanel Host Interface
//--------------------------------------------------------------------
wire ticlk;
wire [30:0] ok1;
wire [16:0] ok2;
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
assign sys_rst_out = gp_trig[sys_rst_offset];
assign adc_cstart_out = gp_trig[adc_cstart_offset];
assign wr_en_out = gp_trig[wr_en_offset];
assign dac_rset_out = gp_trig[dac_rset_offset];

// System trigger
okTriggerIn sys_gp_oti (
    .ok1            (ok1),
    .ep_addr        (sys_gp_itep),
    .ep_clk         (pid_clk_in),
    .ep_trigger     (gp_trig)
    );

//--------------------------------------------------------------------
// External Memory Wire-outs
//--------------------------------------------------------------------
wire [W_EP-1:0] wr_addr;
wire [W_EP-1:0] wr_chan;
wire [W_EP*4-1:0] wr_data;
wire [W_EP-1:0] wr_data3, wr_data2, wr_data1, wr_data0;
assign wr_data = {wr_data3, wr_data2, wr_data1, wr_data0};
assign wr_addr_out = wr_addr[W_WR_ADDR-1:0];
assign wr_chan_out = wr_chan[W_WR_CHAN-1:0];
assign wr_data_out = wr_data[W_WR_DATA-1:0];

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
localparam N_WO = N_LOG + 2;
wire [N_WO*17-1:0] ok2x;

okWireOR #(
    .N              (N_WO))
wireOR (
    .ok2            (ok2),
    .ok2s           (ok2x)
    );

//--------------------------------------------------------------------
// Wire-out Datalogging
//--------------------------------------------------------------------
reg [W_LDATA-1:0] data_log_reg[0:N_LOG-1];
reg [W_LCHAN:0] i;

// Wire-out registers
always @( posedge pid_clk_in ) begin
    if ( sys_rst_out ) begin
        for ( i = 0; i < N_LOG; i = i + 1 ) begin
            data_log_reg[i] <= 0;
        end
    end else if ( log_dv_in ) begin
        data_log_reg[log_chan_in] <= log_data_in;
    end
end

// Wire-outs
genvar j;
generate
for ( j = 0; j < N_LOG; j = j + 1 ) begin : log_owo_arr
    okWireOut log_owo (
        .ok1        (ok1),
        .ok2        (ok2x[j*17 +: 17]),
        .ep_addr    (data_log0_owep + j[W_LCHAN-1:0]),
        .ep_datain  (data_log_reg[j][W_LDATA-1 -: W_EP])
        );
end
endgenerate

//--------------------------------------------------------------------
// Pipe Datalogging
//--------------------------------------------------------------------
reg [W_LCHAN-1:0] pipe_chan;
wire [W_EP-1:0] log_pipe_data;
wire log_pipe_dv = ( log_chan_in == pipe_chan ) ? log_dv_in : 1'b0;
wire log_pipe_rd;
wire log_pipe_rdy;

// Pipe channel write handling
always @( posedge pid_clk_in ) begin
    if ( wr_en_out && ( wr_addr_out == pipe_cset_rqst )) begin
        pipe_chan <= wr_chan_out[W_LCHAN-1:0];
    end
end

// Pipe-out fifo
pipe_tx_fifo log_pipe_buf (
    .rd_clk     (ticlk),
    .wr_clk     (pid_clk_in),
    .rst        (sys_rst_out),
    .wr_en      (log_pipe_dv),
    .din        (log_data_in[W_LDATA-1 -: W_EP]),
    .rd_en      (log_pipe_rd),
    .rd_ready   (log_pipe_rdy),
    .dout       (log_pipe_data)
    );

// Pipe-out
okPipeOut log_pipe (
    .ok1        (ok1),
    .ok2        (ok2x[N_LOG*17 +: 17]),
    .ep_addr    (data_log_opep),
    .ep_datain  (log_pipe_data),
    .ep_read    (log_pipe_rd)
    );

// Data log status register
okWireOut log_owo (
    .ok1        (ok1),
    .ok2        (ok2x[(N_LOG+1)*17 +: 17]),
    .ep_addr    (data_log_status_owep),
    .ep_datain  ({15'b0, log_pipe_rdy})
    );

endmodule


