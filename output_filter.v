`timescale 1ns / 1ps
`include "parameters.vh"

//--------------------------------------------------------------------
// Output Filter
//--------------------------------------------------------------------
// Filters data stream before it is sent to DAC/DDS outputs.
// Adds PID sum to previous output value and enforces max and
// min output bounds.
//--------------------------------------------------------------------

module output_filter #(
    // Parameters
    parameter W_CHAN = 5,
    parameter W_DATA_IN = 18,
    parameter W_DATA_OUT = 64,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48,
    parameter W_MULT = 8,
    parameter W_RS = 8
    )(
    // Inputs
    input wire clk_in,
    input wire sys_rst_in,
    input wire [N_CHAN-1:0] chan_rst_in,

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
localparam W_DATA_MULT = W_DATA_IN + W_MULT;
localparam W_DATA_INT = ((W_DATA_MULT > W_DATA_OUT) ? W_DATA_MULT : W_DATA_OUT) + 1;
localparam MAX_OUT = (2 ** (W_DATA_OUT - 1)) - 1;
localparam MIN_OUT = -(2 ** (W_DATA_OUT - 1));

//--------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------
// Internal channel memory
reg signed [W_DATA_OUT-1:0] dout_prev_mem[0:N_CHAN-1];

// Writeable channel memory
reg signed [W_MULT-1:0] mult_mem[0:N_CHAN-1];
reg [W_RS-1:0] rs_mem[0:N_CHAN-1];
reg signed [W_DATA_OUT-1:0] max_mem[0:N_CHAN-1];
reg signed [W_DATA_OUT-1:0] min_mem[0:N_CHAN-1];
reg signed [W_DATA_OUT-1:0] init_mem[0:N_CHAN-1];

// Pipe registers
reg dv_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DATA_IN-1:0] din_p1 = 0;
reg [W_MULT-1:0] mult_p1 = 0;

reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_DATA_MULT-1:0] dmult_p2 = 0;
reg [W_RS-1:0] rs_p2 = 0;
reg signed [W_DATA_OUT-1:0] dout_prev_p2 = 0;

reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_DATA_INT-1:0] dint_p3 = 0;
reg signed [W_DATA_OUT-1:0] max_p3 = 0;
reg signed [W_DATA_OUT-1:0] min_p3 = 0;

reg dv_p4 = 0;
reg [W_CHAN-1:0] chan_p4 = 0;
reg signed [W_DATA_OUT-1:0] dout_p4 = 0;

reg [W_CHAN-1:0] i = 0;

//--------------------------------------------------------------------
// Logic
//--------------------------------------------------------------------
// Computation pipeline
always @( posedge sys_clk_in ) begin
    //------------------------Pipe Stage 1-----------------------------
	// Register inputs
	dv_p1 = dv_in;
	chan_p1 = chan_in
	din_p1 = data_in;

    // Fetch multiplier
	mult_p1 = mult_mem[chan_in];

    //------------------------Pipe Stage 2-----------------------------
    // Pass data valid and channel
	dv_p2 = dv_p1;
	chan_p2 = chan_p1;

	// Multiply data
	dmult_p2 = din_p1 * mult_p1;

    // Fetch right shift and previous output
	rs_p2 = rs_mem[chan_p1];
	dout_prev_p2 = dout_prev_mem[chan_p1];

    //------------------------Pipe Stage 3-----------------------------
    // Pass data valid and channel
	dv_p3 = dv_p2;
	chan_p3 = chan_p2;

	// Right shift data and add to previous output
	dint_p3 = ( dmult_p2 >>> rs_p2 ) + dout_prev_p2;

    // Fetch output bounds
	max_p3 = max_mem[chan_p2]
    min_p3 = min_mem[chan_p2]

    //------------------------Pipe Stage 4-----------------------------
    // Pass data valid and channel
	dv_p4 = dv_p3;
	chan_p4 = chan_p3;

    // Handle out of bounds
    if ( dint_p3 > max_p3 ) begin
        dout_p4 = max_p3;
    end else if ( dint_p3 < min_p3 ) begin
        dout_p4 = min_p3;
    end else begin
        dout_p4 = dint_p3;
    end

    // Writeback output
    dout_prev_mem[chan_p3] = dout_p4;

    //------------------------Pipe Reset-------------------------------
    if ( sys_rst_in == 1'b1 ) begin
        dv_1 = 0;
        dv_2 = 0;
        dv_3 = 0;
        dv_4 = 0;
    end

    //--------------------Channel Memory Reset-------------------------
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( sys_rst_in || chan_rst_in[i] ) begin
            dout_prev_mem[i] = init_mem[i];
        end
    end
    //-----------------------------------------------------------------
end

// Channel memory write handling
always @( posedge wr_en ) begin
    case ( wr_addr ) begin
        opf_mult_addr : mult_mem[wr_chan] <= wr_data[W_MULT-1:0];
        opf_rs_addr : rs_mem[wr_chan] <= wr_data[W_RS-1:0];
        opf_max_addr : max_mem[wr_chan] <= wr_data[W_DATA_OUT-1:0];
        opf_min_addr : min_mem[wr_chan] <= wr_data[W_DATA_OUT-1:0];
        opf_init_addr : init_mem[wr_chan] <= wr_data[W_DATA_OUT-1:0];
    end
end

// Output assignment
assign dv_out = dv_p4;
assign chan_out = chan_p4;
assign data_out = dout_p4;

endmodule
