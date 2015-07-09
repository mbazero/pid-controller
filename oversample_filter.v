`timescale 1ns / 1ps

//--------------------------------------------------------------------
// Oversample Filter -- mba 2015
//--------------------------------------------------------------------
// Computes a moving average of input data with a variable
// number of samples.
//--------------------------------------------------------------------

module oversample_filter #(
    parameter W_CHAN = 5,
    parameter N_CHAN = 8,
    parameter W_DATA = 18,
    parameter W_SUM = 128,
    parameter W_OS = 5,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_CHAN-1:0] chan_in,
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

`include "ep_map.vh"
`include "functions.vh"

//--------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------
localparam W_COUNT = 2**W_OS;
localparam W_SUM_INT = ((W_SUM > W_DATA) ? W_SUM : W_DATA) + 1;

localparam signed [W_SUM-1:0] MAX_SUM = {W_SUM{1'b1}} >> 1;
localparam signed [W_SUM-1:0] MIN_SUM = ~MAX_SUM;

//--------------------------------------------------------------------
// Request Registers
//--------------------------------------------------------------------
reg [N_CHAN-1:0] clr_rqst = 0;
wire wr_chan_valid = ( wr_chan < N_CHAN );

// Manage clear register
integer i;
always @( posedge clk_in ) begin
    // Handle writes
    if ( wr_en && wr_chan_valid &&
        ( wr_addr == ovr_clr_rqst_addr )) begin
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
reg [W_OS-1:0] os_mem[0:N_CHAN-1];

// Initialization
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        os_mem[i] = 0;
    end
end

// Handle writes
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid ) begin
        case ( wr_addr )
            ovr_os_addr : os_mem[wr_chan] <= wr_data[W_OS-1:0];
        endcase
    end
end

//--------------------------------------------------------------------
// Internal Memory
//--------------------------------------------------------------------
reg signed [W_SUM-1:0] sum_mem[0:N_CHAN-1];
reg [W_COUNT-1:0] count_mem[0:N_CHAN-1];

// Initialization
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        sum_mem[i] = 0;
        count_mem[i] = 0;
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
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DATA-1:0] din_p1 = 0;
reg signed [W_SUM-1:0] sum_p1 = 0;
reg [W_COUNT-1:0] count_p1 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p1 ) begin
        // Register input instruction
        dv_p1 <= dv_in;
        chan_p1 <= chan_in;

        // Register input data
        din_p1 <= data_in;

        // Fetch sum and sample count
        sum_p1 <= sum_mem[chan_in];
        count_p1 <= count_mem[chan_in];

    end else begin
        dv_p1 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 2: Accumulate sum
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p2;
reg signed [W_SUM_INT-1:0] sum_int_p2;

always @( * ) begin
    flush_p2 = ( rst_in || clr_rqst[chan_p1] );

    // Accumulate sum
    sum_int_p2 = sum_p1 + din_p1;
end

// Registers
reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_SUM-1:0] sum_p2 = 0;
reg [W_COUNT-1:0] count_p2 = 0;
reg [W_OS-1:0] os_p2 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p2 ) begin
        // Pass instruction
        dv_p2 <= dv_p1;
        chan_p2 <= chan_p1;

        // Increment sample count
        count_p2 <= count_p1 + 1'b1;

        // Handle sum overflow
        if ( sum_int_p2 > MAX_SUM ) begin
            sum_p2 <= MAX_SUM;
        end else if ( sum_int_p2 < MIN_SUM ) begin
            sum_p2 <= MIN_SUM;
        end else begin
            sum_p2 <= sum_int_p2[W_SUM-1:0];
        end

        // Fetch oversample mode
        os_p2 <= os_mem[chan_p1];

    end else begin
        dv_p2 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 3: Divide sum and writeback
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p3;
reg count_sat_p3;
reg signed [W_SUM-1:0] sum_div_int_p3 = 0;
reg signed [W_SUM-1:0] sum_int_p3 = 0;
reg [W_COUNT-1:0] count_int_p3 = 0;

always @( * ) begin
    flush_p3 = ( rst_in || clr_rqst[chan_p2] );

    // Check whether the oversample count has been satisifed
    count_sat_p3 = ( count_p2[os_p2] == 1'b1 );

    // Divide sum by right shifting
    sum_div_int_p3 = sum_p2 >>> os_p2;

    // Reset sum and sample count if oversample count has been satisified
    sum_int_p3 = ( count_sat_p3 ) ? 0 : sum_p2;
    count_int_p3 = ( count_sat_p3 ) ? 0 : count_p2;
end

// Memory
always @( posedge clk_in ) begin
    // Writeback sum and count
    if ( dv_p2 ) begin
        sum_mem[chan_p2] = sum_int_p3;
        count_mem[chan_p2] = count_int_p3;
    end

    // Zero sum and count memory on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            sum_mem[i] = 0;
            count_mem[i] = 0;
        end
    end
end

// Registers
reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_DATA-1:0] dout_p3 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p3 ) begin
        // Pass instruction if oversample count satisfied
        dv_p3 <= ( count_sat_p3 ) ? dv_p2 : 1'b0;
        chan_p3 <= chan_p2;

        // Truncate divided sum for output
        dout_p3 <= sum_div_int_p3[W_DATA-1:0];

    end else begin
        dv_p3 <= 0;
    end
end

//--------------------------------------------------------------------
// Output Assignment
//--------------------------------------------------------------------
assign dv_out = dv_p3;
assign chan_out = chan_p3;
assign data_out = dout_p3;

endmodule
