`timescale 1ns / 1ps

// ====================================================================
// PID Controller
// ====================================================================
// Dope pid controller module.
// ====================================================================

// TODO polish
// 9. convert all operators to logical ( double symbol )

// TODO pipe dream
// 1. add overflow signal to send along pipeline

module pid_controller (
    // Inputs <- Opal Kelly PLL
    input wire                  sys_clk_in,
    input wire                  adc_clk_in,

    // Inputs <- ADC (AD7608)
    input wire                  adc_busy_in,
    input wire                  adc_data_a_in,
    input wire                  adc_data_b_in,

    // Outputs -> ADC (AD7608)
    output wire [W_ADC_OS-1:0]  adc_os_out,
    output wire                 adc_convst_out,
    output wire                 adc_reset_out,
    output wire                 adc_sclk_out,
    output wire                 adc_n_cs_out,

    // Outputs -> DAC (DAC8568)
    output wire                 dac_nldac_out,
    output wire                 dac_nsync_out,
    output wire                 dac_sclk_out,
    output wire                 dac_din_out,
    output wire                 dac_nclr_out,

    // Outputs -> DDS (AD9912)
    output wire [N_DDS-1:0]     dds_sclk_out,
    output wire [N_DDS-1:0]     dds_reset_out,
    output wire [N_DDS-1:0]     dds_csb_out,
    output wire [N_DDS-1:0]     dds_sdio_out,
    output wire [N_DDS-1:0]     dds_io_update_out,

    // Outputs -> Breakout Board
    output wire                 obuf_en_out,

    // Inouts <-> Frontpanel Host Interface
    input wire  [7:0]           hi_in,
    output wire [1:0]           hi_out,
    inout wire  [15:0]          hi_inout,
    inout wire                  hi_aa,

    output wire                 i2c_sda,
    output wire                 i2c_scl,
    output wire                 hi_muxsel
    );

`include "ep_map.vh"
`include "parameters.vh"

//--------------------------------------------------------------------
// Frontpanel Interface
//--------------------------------------------------------------------
wire sys_rst;
wire adc_cstart;
wire dv_log;
wire [W_PID_CHAN-1:0] chan_log;
wire [W_ADC_DATA-1:0] data_log;
wire wr_en;
wire dac_rset;
wire [W_WR_ADDR-1:0] wr_addr;
wire [W_WR_CHAN-1:0] wr_chan;
wire [W_WR_DATA-1:0] wr_data;

frontpanel_interface #(
    .N_LOG          (N_ADC),
    .W_LCHAN        (W_PID_CHAN),
    .W_LDATA        (W_ADC_DATA),
    .W_EP           (W_EP),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
fp_intf (
    .adc_clk_in     (adc_clk_in),
    .sys_clk_in     (sys_clk_in),
    .dv_log_in      (dv_log),
    .chan_log_in    (chan_log),
    .data_log_in    (data_log),
    .sys_rst_out    (sys_rst),
    .adc_cstart_out (adc_cstart),
    .wr_en_out      (wr_en),
    .dac_rset_out   (dac_rset),
    .wr_addr_out    (wr_addr),
    .wr_chan_out    (wr_chan),
    .wr_data_out    (wr_data),
    .hi_in          (hi_in),
    .hi_out         (hi_out),
    .hi_inout       (hi_inout),
    .hi_aa          (hi_aa),
    .i2c_sda        (i2c_sda),
    .i2c_scl        (i2c_sel),
    .hi_muxsel      (hi_muxsel)
);

//--------------------------------------------------------------------
// ADC Input
//--------------------------------------------------------------------
wire adc_dv;
reg [W_ADC_OS-1:0] adc_os = 1;
wire [W_ADC_CHAN-1:0] adc_src_a;
wire [W_ADC_CHAN-1:0] adc_src_b;
wire [W_ADC_DATA-1:0] adc_data_a;
wire [W_ADC_DATA-1:0] adc_data_b;

wire adc_sync_dv;
wire [W_ADC_CHAN-1:0] adc_sync_src;
wire [W_ADC_DATA-1:0] adc_sync_data;


// ADC controller
adc_controller #(
    .W_OUT          (W_ADC_DATA),
    .W_CHAN         (W_ADC_CHAN),
    .N_CHAN         (N_ADC),
    .W_OS           (W_ADC_OS))
adc_cntrl (
    .clk_in         (adc_clk_in),
    .reset_in       (sys_rst),
    .busy_in        (adc_busy_in),
    .data_a_in      (adc_data_a_in),
    .data_b_in      (adc_data_b_in),
    .os_in          (adc_os),
    .cstart_in      (adc_cstart),
    .os_out         (adc_os_out),
    .convst_out     (adc_convst_out),
    .reset_out      (adc_reset_out),
    .sclk_out       (adc_sclk_out),
    .n_cs_out       (adc_n_cs_out),
    .dv_out         (adc_dv),
    .chan_a_out     (adc_src_a),
    .chan_b_out     (adc_src_b),
    .data_a_out     (adc_data_a),
    .data_b_out     (adc_data_b)
    );

// Clock synchronizer
clk_sync #(
    .W_DATA         (W_ADC_DATA),
    .W_CHAN         (W_ADC_CHAN),
    .N_ADC          (N_ADC))
csync (
    .sys_clk_in     (sys_clk_in),
    .reset_in       (sys_rst),
    .dv_in          (adc_dv),
    .chan_a_in      (adc_src_a),
    .chan_b_in      (adc_src_b),
    .data_a_in      (adc_data_a),
    .data_b_in      (adc_data_b),
    .dv_out         (adc_sync_dv),
    .chan_out       (adc_sync_src),
    .data_out       (adc_sync_data)
    );

// ADC oversample mode write handling
always @( posedge wr_en ) begin
    if ( wr_en && ( wr_addr == adc_os_addr )) begin
        adc_os <= wr_data[W_ADC_OS-1:0];
    end
end

//--------------------------------------------------------------------
// PID Pipeline
//--------------------------------------------------------------------
wire pid_dv;
wire [W_PID_CHAN-1:0] pid_chan;
wire [W_PID_DOUT-1:0] pid_data;

pid_pipeline #(
    .N_SRC          (N_ADC),
    .W_SRC          (W_PID_SRC),
    .N_CHAN         (N_PID_CHAN),
    .W_CHAN         (W_PID_CHAN),
    .W_DIN          (W_PID_DIN),
    .W_DOUT         (W_PID_DOUT),
    .W_OS           (W_PID_OS),
    .W_COMP         (W_PID_COMP),
    .W_OPRNDS       (W_PID_OPRNDS),
    .W_WR_ADDR      (W_WR_ADDR),
    .W_WR_CHAN      (W_WR_CHAN),
    .W_WR_DATA      (W_WR_DATA))
pid_pipe (
    .clk_in         (sys_clk_in),
    .rst_in         (sys_rst),
    .dv_in          (adc_sync_dv),
    .src_in         (adc_sync_src),
    .data_in        (adc_sync_data),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .dv_ovr         (dv_log),
    .chan_ovr       (chan_log),
    .data_ovr       (data_log),
    .dv_out         (pid_dv),
    .chan_out       (pid_chan),
    .data_out       (pid_data)
);

//--------------------------------------------------------------------
// DAC Output
//--------------------------------------------------------------------
wire pid_dac_dv = (pid_chan < N_DAC) ? pid_dv : 0;
wire [W_DAC_CHAN-1:0] pid_dac_chan = pid_chan[W_DAC_CHAN-1:0];
wire [W_DAC_DATA-1:0] pid_dac_data = pid_data[W_DAC_DATA-1:0];

wire diq_dv;
wire [W_DAC_CHAN-1:0] diq_chan;
wire [W_DAC_DATA-1:0] diq_data;
wire dac_wr_done;

// DAC instruction queue
fifo_19 dac_instr_queue (
    .clk    (sys_clk_in),
    .rst    (sys_rst),
    .din    ({pid_dac_chan, pid_dac_data}),
    .wr_en  (pid_dac_dv),
    .rd_en  (dac_wr_done),
    .dout   ({diq_chan, diq_data}),
    .valid  (diq_dv),
    .full   (),
    .empty  ()
    );

// DAC controller
dac_controller dac_cntrl (
    .clk_in         (sys_clk_in),
    .reset_in       (sys_rst),
    .ref_set_in     (dac_rset),
    .data_in        (diq_data),
    .chan_in        (diq_chan),
    .dv_in          (diq_dv),
    .nldac_out      (dac_nldac_out),
    .nsync_out      (dac_nsync_out),
    .sclk_out       (dac_sclk_out),
    .din_out        (dac_din_out),
    .nclr_out       (dac_nclr_out),
    .wr_done_out    (dac_wr_done),
    .data_out       (),
    .chan_out       ()
    );

//--------------------------------------------------------------------
// DDS Output
//--------------------------------------------------------------------
wire [N_DDS-1:0] pid_freq_dv;
wire [N_DDS-1:0] pid_phase_dv;
wire [N_DDS-1:0] pid_amp_dv;

genvar i;
generate
for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_array
    localparam F = FREQ0_ADDR + i;  // frequency absolute index
    localparam P = PHASE0_ADDR + i; // phase absolute index
    localparam A = AMP0_ADDR + i;       // amplitude absolute index

    assign pid_freq_dv[i] = (pid_chan == F) ? pid_dv : 1'b0;
    assign pid_phase_dv[i] = (pid_chan == P) ? pid_dv : 1'b0;
    assign pid_amp_dv[i] = (pid_chan == A) ? pid_dv : 1'b0;

    dds_controller dds_cntrl (
        .clk_in         (sys_clk_in),
        .reset_in       (sys_rst),
        .freq_in        (pid_data[W_FREQ_DATA-1:0]),
        .phase_in       (pid_data[W_PHASE_DATA-1:0]),
        .amp_in         (pid_data[W_AMP_DATA-1:0]),
        .freq_dv_in     (pid_freq_dv),
        .phase_dv_in    (pid_phase_dv),
        .amp_dv_in      (pid_amp_dv),
        .sclk_out       (dds_sclk_out[i]),
        .reset_out      (dds_reset_out[i]),
        .csb_out        (dds_csb_out[i]),
        .sdio_out       (dds_sdio_out[i]),
        .io_update_out  (dds_io_update_out[i]),
        .wr_done_out    ()
    );
end
endgenerate

// Activate output buffers (active low)
assign obuf_en_out = 1'b0;

endmodule
