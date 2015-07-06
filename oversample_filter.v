`timescale 1ns / 1ps
`include "ep_map.vh"

//--------------------------------------------------------------------
// Oversample Filter -- mba 2015
//--------------------------------------------------------------------
// Computes a moving average of input data with a variable
// number of samples.
//--------------------------------------------------------------------

module oversample_filter #(
    // Parameters
    parameter W_CHAN = 5,
    parameter W_DATA = 18,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48,
    parameter W_SUM = 64,
    parameter W_OS = 4
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

//--------------------------------------------------------------------
// Parameters
//--------------------------------------------------------------------
localparam W_COUNT = 2**W_OS;

//--------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------
// Internal channel memory
reg signed [W_SUM-1:0] sum_mem[0:N_CHAN-1];
reg [W_COUNT-1:0] count_mem[0:N_CHAN-1];

// Writeable channel memory
reg [N_CHAN-1:0] clr_mem;
reg [W_OS-1:0] os_mem[0:N_CHAN-1];

// Pipe registers
reg dv_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DATA-1:0] data_p1 = 0;
reg signed [W_SUM-1:0] sum_p1 = 0;
reg [W_COUNT-1:0] count_p1 = 0;

reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_SUM-1:0] sum_p2 = 0;
reg [W_COUNT-1:0] count_p2 = 0;
reg [W_OS-1:0] os_p2 = 0;

reg count_sat_p3 = 0;
reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_DATA-1:0] data_p3 = 0;

reg [W_CHAN-1:0] i = 0;

//--------------------------------------------------------------------
// Logic
//--------------------------------------------------------------------
// Compuation pipeline
always @( posedge clk_in ) begin
    //------------------------Pipe Stage 1-----------------------------
    // Register inputs
    dv_p1 = dv_in;
    chan_p1 = chan_in;
    data_p1 = data_in;

    // Fetch sum and sample count
    sum_p1 = sum_mem[chan_in];
    count_p1 = count_mem[chan_in];

    //-----------------------Pipe Stage 2------------------------------
    // Pass data valid and channel
    dv_p2 = dv_p1;
    chan_p2 = chan_p1;

    // Accumlate sum and increment sample count
    sum_p2 = sum_p1 + data_p1;
    count_p2 = count_p1 + 1'b1;

    // Fetch oversample mode
    os_p2 = os_mem[chan_pipe[1]];

    //-----------------------Pipe Stage 3------------------------------
    // Check whether the oversample count has been satisfied. This is an
    // intra-state signal so blocking assignments must be used.
    count_sat_p3 = ( count_p2[os_mem[2]] == 1 );

    // Pass data valid signal if count satisfied
    dv_p3 = ( count_sat_p3 ) ? dv_p2 : 0;

    // Pass chanination
    chan_p3 = chan_p2;

    // Divide sum by right shifting
    data_p3 = sum_p2 >>> os_p2;

    // Writeback count and sum if data is valid. Reset both terms if sample
    // count has been satisfied
    if ( dv_p2 == 1'b1 ) begin
       sum_mem[chan_p2] = ( count_sat_p3 ) ? 0 : sum_p2;
       count_mem[chan_p2] = ( count_sat_p3 ) ? 0 : count_p2;
    end

    //-------------------------Pipe Clear------------------------------
    if ( rst_in || clr_mem[chan_p1] ) dv_p1 = 0;
    if ( rst_in || clr_mem[chan_p2] ) dv_p2 = 0;
    if ( rst_in || clr_mem[chan_p3] ) dv_p3 = 0;

    //---------------------Channel Memory Clear------------------------
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_mem[i] ) begin
            sum_mem[i] = 0;
            count_mem[i] = 0;
        end
    end
    //-----------------------------------------------------------------
end

// Channel memory write handling
always @( posedge wr_en ) begin
    case ( wr_addr ) begin
        osf_clr_addr : clr_mem[wr_chan] <= wr_data[0];
        osf_os_addr : os_mem[wr_chan] <= wr_data[W_OS-1:0];
    end
end

// Output assignment
assign dv_out = dv_p3;
assign chan_out = chan_p3;
assign data_out = data_p3;

endmodule
