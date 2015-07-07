`timescale 1ns / 1ps
`include "ep_map.vh"

//--------------------------------------------------------------------
// PID Filter -- mba 2015
//--------------------------------------------------------------------
// Computes PID sum using discrete form of PID equation.
//--------------------------------------------------------------------

module pid_filter #(
    // Parameters
    parameter W_CHAN = 5,
    parameter W_DIN = 18,
    parameter W_DOUT = 128,
    parameter W_PID_COEFS = 16,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_CHAN-1:0] chan_in,
    input wire signed [W_DIN-1:0] data_in,

    input wire wr_en,
    input wire [W_WR_ADDR-1:0] wr_addr,
    input wire [W_WR_CHAN-1:0] wr_chan,
    input wire [W_WR_DATA-1:0] wr_data,

    // Outputs
    output wire dv_out,
    output wire [W_CHAN-1:0] chan_out,
    output wire signed [W_DIN-1:0] data_out
    );

//--------------------------------------------------------------------
// Parameters
//--------------------------------------------------------------------
localparam W_ERROR = W_DIN + 1;
localparam W_K_COEFS = W_PID_COEFS + 2;
localparam W_CE_PROD = W_K_COEFS + W_ERROR;
localparam W_DELTA = W_CE_PROD + 2;
localparam W_DINT = ((W_DELTA > W_DOUT) ? W_DELTA : W_DOUT) + 1;

//--------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------
// Internal channel memory
reg signed [W_ERROR-1:0] error_prev0_mem[0:N_CHAN-1];
reg signed [W_ERROR-1:0] error_prev1_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] dout_prev_mem[0:N_CHAN-1];

// Writeable channel memory
reg signed [W_DIN-1:0] setpoint_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] p_coef_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] i_coef_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] d_coef_mem[0:N_CHAN-1];

// Channel request registers
reg [N_CHAN-1:0] clr_req;

// Constants
reg signed [W_DOUT-1:0] max_dout = {W_DOUT{1'b1}} >> 1;
reg signed [W_DOUT-1:0] min_dout = ~max_dout;

// Pipe registers
reg dv_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DIN-1:0] din_p1 = 0;
reg signed [W_DIN-1:0] setpoint_p1 = 0;
reg signed [W_PID_COEFS-1:0] p_coef_p1 = 0;
reg signed [W_PID_COEFS-1:0] i_coef_p1 = 0;
reg signed [W_PID_COEFS-1:0] d_coef_p1 = 0;

reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_ERROR-1:0] error_p2 = 0;
reg signed [W_K_COEFS-1:0] k1_p2 = 0;
reg signed [W_K_COEFS-1:0] k2_p2 = 0;
reg signed [W_K_COEFS-1:0] k3_p2 = 0;
reg signed [W_ERROR-1:0] error_prev0_p2 = 0;
reg signed [W_ERROR-1:0] error_prev1_p2 = 0;

reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod0_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod1_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod2_p3 = 0;

reg dv_p4 = 0;
reg [W_CHAN-1:0] chan_p4 = 0;
reg signed [W_DELTA-1:0] delta_p4 = 0;
reg signed [W_DOUT-1:0] dout_prev_p4 = 0;

reg dv_p5 = 0;
reg [W_CHAN-1:0] chan_p5 = 0;
reg signed [W_DINT-1:0] dint_p5 = 0;

reg dv_p6 = 0;
reg [W_CHAN-1:0] chan_p6 = 0;
reg signed [W_DOUT-1:0] dout_p6 = 0;

reg [W_CHAN-1:0] i;

//--------------------------------------------------------------------
// Logic
//--------------------------------------------------------------------
// Computation pipeline
always @( posedge sys_clk_in ) begin
    //------------------------Pipe Stage 1-----------------------------
    // Register inputs
    dv_p1 = dv_in;
    chan_p1 = chan_in;
    din_p1 = data_in;

    // Fetch setpoint, PID coefficients, and lock enable
    setpoint_p1 = setpoint_mem[chan_in];
    p_coef_p1 = p_coef_mem[chan_in];
    i_coef_p1 = i_coef_mem[chan_in];
    d_coef_p1 = d_coef_mem[chan_in];

    //------------------------Pipe Stage 2-----------------------------
    // Pass data valid and channel
    dv_p2 = dv_p1;
    chan_p2 = chan_p1;

    // Compute error
    error_p2 = setpoint_p1 - din_p1;

    // Compute z-transform coefficients
    k1_p2 = p_coef_p1 + i_coef_p1 + d_coef_p1;
    k2_p2 = -p_coef_p1 - (d_coef_p1 <<< 1);
    k3_p2 = d_coef_p1

    // Fetch previous error values
    error_prev0_p2 = error_prev0_mem[chan_p1];
    error_prev1_p2 = error_prev1_mem[chan_p2];

    //------------------------Pipe Stage 3-----------------------------
    // Pass data valid and channel
    dv_p3 = dv_p2;
    chan_p3 = chan_p2;

    // Compute coefficient error products
    ce_prod0_p3 = k1_p2 * error_p2;
    ce_prod1_p3 = k2_p2 * error_prev0_p2;
    ce_prod2_p3 = k3_p2 * error_prev1_p2

    // Writeback error data if data is valid
    if ( dv_p2 == 1'b1 ) begin
        error_prev0_mem[chan_p2] = error_p2;
        error_prev1_mem[chan_p2] = error_prev0_p2;
    end

    //------------------------Pipe Stage 3-----------------------------
    // Pass data valid and channel
    dv_p4 = dv_p3;
    chan_p4 = chan_p3;

    // Compute PID delta
    delta_p4 = ce_prod0_p3 + ce_prod1_p3 + ce_prod2_p3;

    // Fetch previous output
    dout_prev_p4 = dout_prev_mem[chan_p3];

    //------------------------Pipe Stage 4-----------------------------
    // Pass data valid and channel
    dv_p5 = dv_p4;
    chan_p5 = chan_p4;

    // Add delta to previous output
    dint_p5 = dout_prev_p4 + delta_p4;

    //------------------------Pipe Stage 5-----------------------------
    // Pass data valid and channel
    dv_p6 = dv_p5;
    chan_p6 = chan_p5;

    // Handle overflow
    if ( dint_p5 > max_dout ) begin
        dout_p6 = max_dout;
    end else if ( dint_p5 < min_dout ) begin
        dout_p6 = min_dout;
    end else begin
        dout_p6 = dint_p5[W_DOUT-1:0];
    end

    // Writeback data previous if data is valid
    if ( dv_p5 == 1'b1 ) begin
        dout_prev_mem[chan_p5] = dout_p6;
    end

    //-----------------------Pipe Flushing-----------------------------
    if ( rst_in || clr_req[chan_p1] ) dv_p1 = 0;
    if ( rst_in || clr_req[chan_p2] ) dv_p2 = 0;
    if ( rst_in || clr_req[chan_p3] ) dv_p3 = 0;
    if ( rst_in || clr_req[chan_p4] ) dv_p4 = 0;
    if ( rst_in || clr_req[chan_p5] ) dv_p5 = 0;
    if ( rst_in || clr_req[chan_p6] ) dv_p6 = 0;

    //----------------------Channel Memory-----------------------------
    // Clear internal memory and request registers on reset and clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_req[i] ) begin
            error_prev0_mem[i] = 0;
            error_prev1_mem[i] = 0;
            dout_prev_mem[i] = 0;
            clr_req[i] = 0;
        end
    end

    // Handle memory writes
    if ( wr_en ) begin
        case ( wr_addr ) begin
            pid_clr_req_addr : clr_req[wr_chan] <= wr_data[0];
            pid_setpoint_addr : setpoint_mem[wr_chan] <= wr_data[W_DIN-1:0];
            pid_p_coef_addr : p_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
            pid_i_coef_addr : i_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
            pid_d_coef_addr : d_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
        end
    end
    //-----------------------------------------------------------------
end

// Channel memory write handling

// Output assignment
assign dv_out = dv_p6;
assign chan_out = chan_p6;
assign data_out = dout_p6;

endmodule
