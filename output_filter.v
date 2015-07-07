`timescale 1ns / 1ps
`include "ep_map.vh"

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
    parameter W_DIN = 18,
    parameter W_DOUT = 64,
    parameter W_MULT = 8,
    parameter W_RS = 8,
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

//--------------------------------------------------------------------
// Parameters
//--------------------------------------------------------------------
localparam W_DMULT = W_DIN + W_MULT;
localparam W_DSUM_A = W_DMULT + 1;
localparam W_DSUM_B = ((W_DSUM_A > W_DOUT) ? W_DSUM_A : W_DOUT) + 1;

//--------------------------------------------------------------------
// Channel Memory
//--------------------------------------------------------------------
// Internal channel state
reg signed [W_DOUT-1:0] dout_prev_mem[0:N_CHAN-1];
reg signed [W_DMULT-1:0] dmult_prev_mem[0:N_CHAN-1];

// Writeable channel state
reg [W_CHAN:0] add_chan_mem[0:N_CHAN-1];
reg [W_RS-1:0] rs_mem[0:N_CHAN-1];
reg signed [W_MULT-1:0] mult_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] max_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] min_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] init_mem[0:N_CHAN-1];

// Operation request registers
reg [N_CHAN-1:0] clr_req = 0;
reg [N_CHAN-1:0] inj_req = 0;

// Initialization
integer i;
initial begin
    for ( i = 0; i < N_CHAN; i = i+1 ) begin
        add_chan_mem[i] = 1 << W_CHAN; // Invalid route
    end
end

//--------------------------------------------------------------------
// Pipe Registers
//--------------------------------------------------------------------
reg inj_p1 = 0;
reg dv_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DIN-1:0] din_p1 = 0;
reg [W_MULT-1:0] mult_p1 = 0;
reg [W_RS-1:0] rs_p1 = 0;
reg [W_CHAN:0] add_chan_p1 = 0;

reg inj_p2 = 0;
reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_DMULT-1:0] dmult_p2 = 0;
reg signed [W_DMULT-1:0] add_data_p2 = 0;

reg inj_p3 = 0;
reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg [W_DSUM_A-1:0] dsum_a_p3 = 0;
reg signed [W_DOUT-1:0] dout_prev_p3 = 0;

reg inj_p4 = 0;
reg dv_p4 = 0;
reg [W_CHAN-1:0] chan_p4 = 0;
reg signed [W_DSUM_B-1:0] dsum_b_p4 = 0;
reg signed [W_DOUT-1:0] max_p4 = 0;
reg signed [W_DOUT-1:0] min_p4 = 0;
reg signed [W_DOUT-1:0] init_p4 = 0;

reg dv_p5 = 0;
reg [W_CHAN-1:0] chan_p5 = 0;
reg signed [W_DINT-1:0] dinj_p5 = 0;
reg signed [W_DOUT-1:0] dout_p5 = 0;

//--------------------------------------------------------------------
// Logic
//--------------------------------------------------------------------
// Computation pipeline
always @( posedge sys_clk_in ) begin
    //------------------------Pipe Stage 1-----------------------------
    // Register inputs if input data is valid. Otherwise, inject init
    // write instruction if there are pending requests. Injection
    // channel preference is low to high.
    if ( dv_in ) begin
        inj_p1 = 0;
        dv_p1 = dv_in;
        chan_p1 = chan_in;
        din_p1 = data_in;
    end else begin
        // Don't inject by default
        inj_p1 = 0;
        dv_p1 = 0;

        // Inject instruction if request is pending
        for ( i = N_CHAN; i >= 0; i = i - 1 ) begin
            if ( inj_req[i] )
                inj_p1 = 1;
                dv_p1 = 1;
                chan_p1 = i;
                din_p1 = 0;
            end
        end

        // Clear request register if instruction was injected
        if ( inj_p1 ) inj_req[chan_p1] = 0;
    end

    // Fetch multiplier, right shift, and add channel
	mult_p1 = mult_mem[chan_in];
	rs_p1 = rs_mem[chan_in];
    add_chan_p1 = add_chan_mem[chan_in];

    //------------------------Pipe Stage 2-----------------------------
    // Pass injection flag, data valid, and channel
    inj_p2 = inj_p1;
	dv_p2 = dv_p1;
	chan_p2 = chan_p1;

	// Multiply data and right shift
	dmult_p2 = (din_p1 * mult_p1) >>> rs_p1;

    // Fetch add data if add channel is valid
    if ( add_chan_p1 < N_CHAN ) begin
        add_data_p2 = dmult_prev_mem[add_chan_p1];
    end else begin
        add_data_p2 = 0;
    end

    //------------------------Pipe Stage 3-----------------------------
    // Pass injection flag, data valid, and channel
    inj_p3 = inj_p2;
	dv_p3 = dv_p2;
	chan_p3 = chan_p2;

	// Sum data with add channel data
	dsum_a_p3 = dmult_p2 + add_data_p2;

    // Fetch previous output
	dout_prev_p3 = dout_prev_mem[chan_p2];

    // Writeback multiplied and shifted data if it is valid
    if ( dv_p2 ) begin
        dmult_prev_mem[chan_p2] = dmult_p2;
    end

    //------------------------Pipe Stage 4-----------------------------
    // Pass injection flag, data valid, and channel
    inj_p4 = inj_p3;
	dv_p4 = dv_p3;
	chan_p4 = chan_p3;

	// Sum data with previous output
	dsum_b_p4 = dsum_a_p3 + dout_prev_p3;

    // Fetch output bounds
	max_p4 = max_mem[chan_p3];
    min_p4 = min_mem[chan_p3];
    init_p4 = init_mem[chan_p3];

    //------------------------Pipe Stage 4-----------------------------
    // Pass data valid and channel
	dv_p5 = dv_p4;
	chan_p5 = chan_p4;

    // Handle init write injection
    dinj_p5 = ( inj_p4 ) ? init_p4 : dsum_b_p4;

    // Handle output bounds violoations
    if ( dinj_p5 > max_p4 ) begin
        dout_p5 = max_p4;
    end else if ( dinj_p5 < min_p4 ) begin
        dout_p5 = min_p4;
    end else begin
        dout_p5 = dinj_p5;
    end

    // Writeback output if data is valid
    if ( dv_p4 ) begin
        dout_prev_mem[chan_p4] = dout_p5;
    end

    //-----------------------Pipe Flushing-----------------------------
    if ( rst_in || clr_req[chan_p1] ) dv_p1 = 0;
    if ( rst_in || clr_req[chan_p2] ) dv_p2 = 0;
    if ( rst_in || clr_req[chan_p3] ) dv_p3 = 0;

    //----------------------Channel Memory-----------------------------
    // Clear internal memory and request registers on reset and clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_req[i] ) begin
            dout_prev_mem[i] = init_mem[i];
            dmult_prev_mem[i] = 0;
            clr_req[i] = 0;
        end
    end

    // Handle memory writes
    if ( wr_en ) begin
        case ( wr_addr ) begin
            opf_clr_req_addr : clr_req[wr_chan] <= wr_data[0];
            opf_inj_req_addr : inj_req[wr_chan] <= wr_data[0];
            opf_min_addr : min_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opf_max_addr : max_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opf_init_addr : init_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opf_mult_addr : mult_mem[wr_chan] <= wr_data[W_MULT-1:0];
            opf_rs_addr : rs_mem[wr_chan] <= wr_data[W_RS-1:0];
            opf_add_chan : add_chan_mem[wr_chan] <= wr_data[W_CHAN:0];
        end
    end
    //-----------------------------------------------------------------
end

// Output assignment
assign dv_out = dv_p5;
assign chan_out = chan_p4;
assign data_out = dout_p4;

endmodule
