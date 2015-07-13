`timescale 1ns / 1ps

//--------------------------------------------------------------------
// PID Filter -- mba 2015
//--------------------------------------------------------------------
// Computes PID sum using discrete form of PID equation.
//--------------------------------------------------------------------

module pid_filter #(
    parameter W_CHAN = 5,
    parameter N_CHAN = 8,
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
    output wire signed [W_DOUT-1:0] data_out
    );

`include "ep_map.vh"
`include "init.vh"

//--------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------
localparam W_ERROR = W_DIN + 1;
localparam W_K_COEFS = W_PID_COEFS + 2;
localparam W_CE_PROD = W_K_COEFS + W_ERROR;
localparam W_DELTA = W_CE_PROD + 2;
localparam W_DOUT_UC = ((W_DOUT > W_DELTA) ? W_DOUT : W_DELTA) + 1;

localparam signed [W_DOUT-1:0] MAX_DOUT = {W_DOUT{1'b1}} >> 1;
localparam signed [W_DOUT-1:0] MIN_DOUT = ~MAX_DOUT;

//--------------------------------------------------------------------
// Request Registers
//--------------------------------------------------------------------
reg [N_CHAN-1:0] clr_rqst;
wire wr_chan_valid = ( wr_chan < N_CHAN );

// Manage clear register
integer i;
always @( posedge clk_in ) begin
    // Handle writes
    if ( wr_en && wr_chan_valid &&
        ( wr_addr == pid_clr_rqst )) begin
        clr_rqst[wr_chan] = wr_data[0];
    end

    // Zero on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            clr_rqst[i] = 0;
        end
    end
end

//--------------------------------------------------------------------
// External Memory
//--------------------------------------------------------------------
reg [N_CHAN-1:0] lock_en_mem;
reg [N_CHAN-1:0] inv_error_mem;
reg signed [W_DIN-1:0] setpoint_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] p_coef_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] i_coef_mem[0:N_CHAN-1];
reg signed [W_PID_COEFS-1:0] d_coef_mem[0:N_CHAN-1];

// Initialize
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        lock_en_mem[i] = PID_LOCK_EN_INIT;
        inv_error_mem[i] = PID_INV_ERROR_INIT;
        setpoint_mem[i] = PID_SETPOINT_INIT;
        p_coef_mem[i] = PID_P_COEF_INIT;
        i_coef_mem[i] = PID_I_COEF_INIT;
        d_coef_mem[i] = PID_D_COEF_INIT;
    end
end

// Handle writes
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid ) begin
        case ( wr_addr )
            pid_lock_en_addr : lock_en_mem[wr_chan] <= wr_data[0];
            pid_inv_error_addr : inv_error_mem[wr_chan] <= wr_data[0];
            pid_setpoint_addr : setpoint_mem[wr_chan] <= wr_data[W_DIN-1:0];
            pid_p_coef_addr : p_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
            pid_i_coef_addr : i_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
            pid_d_coef_addr : d_coef_mem[wr_chan] <= wr_data[W_PID_COEFS-1:0];
        endcase
    end
end

//--------------------------------------------------------------------
// Internal Memory
//--------------------------------------------------------------------
reg signed [W_DOUT-1:0] dout_prev_mem[0:N_CHAN-1];
reg signed [W_ERROR-1:0] error_prev1_mem[0:N_CHAN-1];
reg signed [W_ERROR-1:0] error_prev2_mem[0:N_CHAN-1];

// Initialize
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        dout_prev_mem[i] = 0;
        error_prev1_mem[i] = 0;
        error_prev2_mem[i] = 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 1: Fetch
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p1;

always @( * ) begin
    flush_p1 = ( rst_in || clr_rqst[chan_in] );
end

// Registers
reg dv_p1 = 0;
reg lock_en_p1 = 0;
reg inv_error_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DIN-1:0] din_p1 = 0;
reg signed [W_DIN-1:0] setpoint_p1 = 0;
reg signed [W_PID_COEFS-1:0] p_coef_p1 = 0;
reg signed [W_PID_COEFS-1:0] i_coef_p1 = 0;
reg signed [W_PID_COEFS-1:0] d_coef_p1 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p1 ) begin
        // Register input instruction
        dv_p1 <= dv_in;
        chan_p1 <= chan_in;

        // Register input data
        din_p1 <= data_in;

        // Fetch setpoint, PID coefficients, and invert error flag
        lock_en_p1 <= lock_en_mem[chan_in];
        inv_error_p1 <= inv_error_mem[chan_in];
        setpoint_p1 <= setpoint_mem[chan_in];
        p_coef_p1 <= p_coef_mem[chan_in];
        i_coef_p1 <= i_coef_mem[chan_in];
        d_coef_p1 <= d_coef_mem[chan_in];

    end else begin
        dv_p1 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 2: Compute error and k-coefficients
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p2;
reg signed [W_ERROR-1:0] error_int_p2 = 0;

always @( * ) begin
    // Flush stage if lock is disabled
    flush_p2 = ( rst_in || clr_rqst[chan_p1] || !lock_en_p1 );

    // Compute error
    error_int_p2 = setpoint_p1 - din_p1;
end

// Registers
reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_ERROR-1:0] error_p2 = 0;
reg signed [W_K_COEFS-1:0] k1_p2 = 0;
reg signed [W_K_COEFS-1:0] k2_p2 = 0;
reg signed [W_K_COEFS-1:0] k3_p2 = 0;
reg signed [W_ERROR-1:0] error_prev1_p2 = 0;
reg signed [W_ERROR-1:0] error_prev2_p2 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p2 ) begin
        // Pass instruction
        dv_p2 <= dv_p1;
        chan_p2 <= chan_p1;

        // Optionally invert error
        error_p2 <= ( inv_error_p1 ) ? ~error_int_p2 : error_int_p2;

        // Compute z-transform coefficients
        k1_p2 <= p_coef_p1 + i_coef_p1 + d_coef_p1;
        k2_p2 <= -p_coef_p1 - (d_coef_p1 <<< 1);
        k3_p2 <= d_coef_p1;

        // Fetch previous error values
        error_prev1_p2 <= error_prev1_mem[chan_p1];
        error_prev2_p2 <= error_prev2_mem[chan_p1];

    end else begin
        dv_p2 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 3: Computer error products and writeback
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p3;

always @( * ) begin
    flush_p3 = ( rst_in || clr_rqst[chan_p2] );
end

// Memory
always @( posedge clk_in ) begin
    // Writeback error data
    if ( dv_p2 ) begin
        error_prev1_mem[chan_p2] = error_p2;
        error_prev2_mem[chan_p2] = error_prev1_p2;
    end

    // Zero error data memory on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            error_prev1_mem[i] = 0;
            error_prev2_mem[i] = 0;
        end
    end
end

// Registers
reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod1_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod2_p3 = 0;
reg signed [W_CE_PROD-1:0] ce_prod3_p3 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p3 ) begin
        // Pass instruction
        dv_p3 <= dv_p2;
        chan_p3 <= chan_p2;

        // Compute coefficient error products
        ce_prod1_p3 <= k1_p2 * error_p2;
        ce_prod2_p3 <= k2_p2 * error_prev1_p2;
        ce_prod3_p3 <= k3_p2 * error_prev2_p2;

    end else begin
        dv_p3 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 4: Compute PID delta
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p4;

always @( * ) begin
    flush_p4 = ( rst_in || clr_rqst[chan_p3] );
end

// Registers
reg dv_p4 = 0;
reg [W_CHAN-1:0] chan_p4 = 0;
reg signed [W_DELTA-1:0] delta_p4 = 0;
reg signed [W_DOUT-1:0] dout_prev_p4 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p4 ) begin
        // Pass instruction
        dv_p4 <= dv_p3;
        chan_p4 <= chan_p3;

        // Compute PID delta
        delta_p4 <= ce_prod1_p3 + ce_prod2_p3 + ce_prod3_p3;

        // Fetch previous output
        dout_prev_p4 <= dout_prev_mem[chan_p3];

    end else begin
        dv_p4 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 5: Compute output
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p5;
reg signed [W_DOUT_UC-1:0] dout_int_p5 = 0;

always @ ( * ) begin
    flush_p5 = ( rst_in || clr_rqst[chan_p4] );

    // Compute output
    dout_int_p5 = dout_prev_p4 + delta_p4;
end

// Registers
reg dv_p5 = 0;
reg [W_CHAN-1:0] chan_p5 = 0;
reg signed [W_DOUT-1:0] dout_p5 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p5 ) begin
        // Pass instruction
        dv_p5 <= dv_p4;
        chan_p5 <= chan_p4;

        // Handle output overflow
        if ( dout_int_p5 > MAX_DOUT ) begin
            dout_p5 <= MAX_DOUT;
        end else if ( dout_int_p5 < MIN_DOUT ) begin
            dout_p5 <= MIN_DOUT;
        end else begin
            dout_p5 <= dout_int_p5[W_DOUT-1:0];
        end

    end else begin
        dv_p5 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 6: Writeback output
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p6;

always @( * ) begin
    flush_p6 = ( rst_in || clr_rqst[chan_p5] );
end

// Memory
always @( posedge clk_in ) begin
    // Writeback output
    if ( dv_p5 == 1'b1 ) begin
        dout_prev_mem[chan_p5] = dout_p5;
    end

    // Zero previous output memomry on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            dout_prev_mem[i] = 0;
        end
    end
end

// Registers
reg dv_p6 = 0;
reg [W_CHAN-1:0] chan_p6 = 0;
reg signed [W_DOUT-1:0] dout_p6 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p6 ) begin
        // Pass instruction
        dv_p6 <= dv_p5;
        chan_p6 <= chan_p5;

        // Pass data
        dout_p6 <= dout_p5;

    end else begin
        dv_p6 <= 0;
    end
end

//--------------------------------------------------------------------
// Output Assingment
//--------------------------------------------------------------------
assign dv_out = dv_p6;
assign chan_out = chan_p6;
assign data_out = dout_p6;

endmodule
