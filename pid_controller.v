`timescale 1ns / 1ps

// ====================================================================
// PID Controller
// ====================================================================

// TODO polish
// 9. convert all operators to logical ( double symbol )

module pid_controller (
    // Inputs <- Opal Kelly PLL
    input wire                  clk17_in,
    input wire                  clk50_in,

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
    output wire                 hi_muxsel,

    // Pipe debug
    output wire idp_dv_db,
    output wire pid_dv_db,
    output wire opt_dv_db,
    output wire [1:0] idp_src_db,
    output wire [1:0] idp_chan_db,
    output wire din_db,

    // ADC controller debug
    output wire adc_clk_db,
    output wire adc_dv_db,
    output wire [W_ADC_CHAN-1:0] adc_src_db,
    output wire [W_ADC_DATA-1:0] adc_data_db,

    // ADC buffer debug
    output wire adc_buf_clk_db,
    output wire adc_buf_dv_db,
    output wire [W_ADC_CHAN-1:0] adc_buf_src_db,
    output wire [W_ADC_DATA-1:0] adc_buf_data_db
    );

// Debug
assign din_db = dac_din_out;

`include "ep_map.vh"
`include "parameters.vh"
`include "init.vh"

//--------------------------------------------------------------------
// Clocks and Buffers
//--------------------------------------------------------------------
wire adc_clk = clk17_in;
wire pid_clk = clk50_in;
wire dac_clk = clk50_in;
wire dds_clk = clk50_in;

//--------------------------------------------------------------------
// Frontpanel Interface
//--------------------------------------------------------------------
wire sys_rst;
wire adc_cstart;
wire log_dv;
wire [W_PID_CHAN-1:0] log_chan;
wire [W_ADC_DATA-1:0] log_data;
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
    .adc_clk_in     (adc_clk),
    .pid_clk_in     (pid_clk),
    .log_dv_in      (log_dv),
    .log_chan_in    (log_chan),
    .log_data_in    (log_data),
    .sys_rst_out    (sys_rst),
    .adc_cstart_out (adc_cstart),
    .dac_rset_out   (dac_rset),
    .wr_en_out      (wr_en),
    .wr_addr_out    (wr_addr),
    .wr_chan_out    (wr_chan),
    .wr_data_out    (wr_data),
    .hi_in          (hi_in),
    .hi_out         (hi_out),
    .hi_inout       (hi_inout),
    .hi_aa          (hi_aa),
    .i2c_sda        (i2c_sda),
    .i2c_scl        (i2c_scl),
    .hi_muxsel      (hi_muxsel)
);

//--------------------------------------------------------------------
// ADC Input
//--------------------------------------------------------------------
wire adc_buf_empty;
wire adc_dv, adc_buf_dv;
wire [W_ADC_CHAN-1:0] adc_src, adc_buf_src;
wire [W_ADC_DATA-1:0] adc_data, adc_buf_data;
reg [W_ADC_OS-1:0] adc_os = ADC_OS_INIT;

assign adc_clk_db = adc_clk;
assign adc_dv_db = adc_dv;
assign adc_src_db = adc_src[1:0];
assign adc_data_db = adc_data[3:0];

assign adc_buf_clk_db = pid_clk;
assign adc_buf_dv_db = adc_buf_dv;
assign adc_buf_src_db = adc_buf_src[1:0];
assign adc_buf_data_db = adc_buf_data[3:0];

adc_controller #(
    .W_OUT          (W_ADC_DATA),
    .W_CHAN         (W_ADC_CHAN),
    .N_CHAN         (N_ADC),
    .W_OS           (W_ADC_OS))
adc_cntrl (
    .clk_in         (adc_clk),
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
    .chan_out       (adc_src),
    .data_out       (adc_data)
    );

adc_fifo adc_buf (
    .wr_clk (adc_clk),
    .rd_clk (pid_clk),
    .rst    (sys_rst),
    .din    ({adc_src, adc_data}),
    .wr_en  (adc_dv),
    .rd_en  (!adc_buf_empty),
    .dout   ({adc_buf_src, adc_buf_data}),
    .empty  (adc_buf_empty),
    .valid  (adc_buf_dv)
    );

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
    .N_SRC          (N_PID_SRC),
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
    .clk_in         (pid_clk),
    .rst_in         (sys_rst),
    .dv_in          (adc_buf_dv),
    .src_in         (adc_buf_src),
    .data_in        (adc_buf_data),
    .wr_en          (wr_en),
    .wr_addr        (wr_addr),
    .wr_chan        (wr_chan),
    .wr_data        (wr_data),
    .ovr_dv         (log_dv),
    .ovr_chan       (log_chan),
    .ovr_data       (log_data),
    .dv_out         (pid_dv),
    .chan_out       (pid_chan),
    .data_out       (pid_data),
    // DEBUG
    .idp_dv_db (idp_dv_db),
    .pid_dv_db (pid_dv_db),
    .opt_dv_db (opt_dv_db),
    .idp_src_db (idp_src_db),
    .idp_chan_db (idp_chan_db)
);

//--------------------------------------------------------------------
// DAC Output
//--------------------------------------------------------------------
wire pid_dac_dv, buf_dac_dv;
wire [W_DAC_CHAN-1:0] pid_dac_chan, buf_dac_chan;
wire [W_DAC_DATA-1:0] pid_dac_data, buf_dac_data;
wire dac_wr_done;

assign pid_dac_dv = (pid_chan < N_DAC) ? pid_dv : 1'b0;
assign pid_dac_chan = pid_chan[W_DAC_CHAN-1:0];
assign pid_dac_data = pid_data[W_DAC_DATA-1:0];

dac_fifo dac_buf (
    .wr_clk (pid_clk),
    .rd_clk (dac_clk),
    .rst    (sys_rst),
    .din    ({pid_dac_chan, pid_dac_data}),
    .wr_en  (pid_dac_dv),
    .rd_en  (dac_wr_done),
    .dout   ({buf_dac_chan, buf_dac_data}),
    .valid  (buf_dac_dv)
    );

dac_controller dac_cntrl (
    .clk_in         (dac_clk),
    .rst_in         (sys_rst),
    .ref_set_in     (dac_rset),
    .data_in        (buf_dac_data),
    .chan_in        (buf_dac_chan),
    .dv_in          (buf_dac_dv),
    .nldac_out      (dac_nldac_out),
    .nsync_out      (dac_nsync_out),
    .sclk_out       (dac_sclk_out),
    .din_out        (dac_din_out),
    .nclr_out       (dac_nclr_out),
    .wr_done        (dac_wr_done)
    );

//--------------------------------------------------------------------
// DDS Output
//--------------------------------------------------------------------
wire [N_DDS-1:0] pid_freq_dv, buf_freq_dv;
wire [N_DDS-1:0] pid_phase_dv, buf_phase_dv;
wire [N_DDS-1:0] pid_amp_dv, buf_amp_dv;

wire [W_FREQ_DATA-1:0] pid_freq_data, buf_freq_data;
wire [W_PHASE_DATA-1:0] pid_phase_data, buf_phase_data;
wire [W_AMP_DATA-1:0] pid_amp_data, buf_amp_data;
wire freq_wr_done, phase_wr_done, amp_wr_done;

assign pid_freq_data = pid_data[W_FREQ_DATA-1:0];
assign pid_phase_data = pid_data[W_PHASE_DATA-1:0];
assign pid_amp_data = pid_data[W_AMP_DATA-1:0];
assign obuf_en_out = 1'b0;

genvar i;
generate
for ( i = 0; i < N_DDS; i = i + 1 ) begin : dds_array
    localparam F = FREQ0_ADDR + i; // Frequency absolute index
    localparam P = PHASE0_ADDR + i; // Phase absolute index
    localparam A = AMP0_ADDR + i; // Amplitude absolute index

    assign pid_freq_dv[i] = (pid_chan == F) ? pid_dv : 0;
    assign pid_phase_dv[i] = (pid_chan == P) ? pid_dv : 0;
    assign pid_amp_dv[i] = (pid_chan == A) ? pid_dv : 0;

    freq_fifo freq_buf (
        .wr_clk     (pid_clk),
        .rd_clk     (dds_clk),
        .rst        (sys_rst),
        .din        (pid_freq_data),
        .wr_en      (pid_freq_dv),
        .rd_en      (freq_wr_done),
        .dout       (buf_freq_data),
        .valid      (buf_freq_dv)
        );

    phase_fifo phase_buf (
        .wr_clk     (pid_clk),
        .rd_clk     (dds_clk),
        .rst        (sys_rst),
        .din        (pid_phase_data),
        .wr_en      (pid_phase_dv),
        .rd_en      (phase_wr_done),
        .dout       (buf_phase_data),
        .valid      (buf_phase_dv)
        );

    amp_fifo amp_buf (
        .wr_clk     (pid_clk),
        .rd_clk     (dds_clk),
        .rst        (sys_rst),
        .din        (pid_amp_data),
        .wr_en      (pid_amp_dv),
        .rd_en      (amp_wr_done),
        .dout       (buf_amp_data),
        .valid      (buf_amp_dv)
        );

    dds_controller dds_cntrl (
        .clk_in         (dds_clk),
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
        .freq_wr_done   (freq_wr_done),
        .phase_wr_done  (phase_wr_done),
        .amp_wr_done    (amp_wr_done)
    );
end
endgenerate

endmodule
