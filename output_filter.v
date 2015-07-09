`timescale 1ns / 1ps

//--------------------------------------------------------------------
// Output Filter
//--------------------------------------------------------------------
// Take delta value as an input. Optionally multiplies and right
// shifts the delta before adding it to the previously outputted
// value. Enforces max and min bounds on the resulting value.
//--------------------------------------------------------------------

module output_filter #(
    parameter W_CHAN = 5,
    parameter N_CHAN = 8,
    parameter W_DELTA = 18,
    parameter W_DOUT = 64,
    parameter W_MULT = 8,
    parameter W_RS = 8,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 5,
    parameter W_WR_DATA = 48
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_CHAN-1:0] chan_in,
    input wire signed [W_DELTA-1:0] delta_in,

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

//--------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------
localparam W_DMTRS = W_DELTA + W_MULT;
localparam W_DSUM = W_DMTRS + 1;
localparam W_DOUT_UC = ((W_DSUM > W_DOUT) ? W_DSUM : W_DOUT) + 1;

localparam [W_CHAN:0] NULL_CHAN = {1'b1, {W_CHAN{1'b0}}};

//--------------------------------------------------------------------
// Request Registers
//--------------------------------------------------------------------
reg inj_in;
reg [W_CHAN-1:0] inj_chan_in;
reg [N_CHAN-1:0] inj_rqst = 0;
reg [N_CHAN-1:0] clr_rqst = 0;
wire wr_chan_valid = ( wr_chan < N_CHAN );

// Manage injection register
always @( posedge clk_in ) begin
    // Handle writes
    if ( wr_en && wr_chan_valid &&
        ( wr_addr == opt_inj_rqst_addr )) begin
        inj_rqst[wr_chan] = wr_data[0];
    end

    // Zero after successful injection
    if ( inj_in ) begin
        inj_rqst[inj_chan_in] = 0;
    end

    // Zero on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            inj_rqst[i] = 0;
        end
    end
end

// Manage clear register
integer i;
always @( posedge clk_in ) begin
    // Handle writes
    if ( wr_en && wr_chan_valid &&
        ( wr_addr == opt_clr_rqst_addr )) begin
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
reg [W_CHAN:0] add_chan_mem[0:N_CHAN-1];
reg [W_RS-1:0] rs_mem[0:N_CHAN-1];
reg signed [W_MULT-1:0] mult_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] max_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] min_mem[0:N_CHAN-1];
reg signed [W_DOUT-1:0] init_mem[0:N_CHAN-1];

// Initialize
initial begin
    for ( i = 0; i < N_CHAN; i = i+1 ) begin
        add_chan_mem[i] = NULL_CHAN;
        rs_mem[i] = 0;
        mult_mem[i] = 0;
        max_mem[i] = 0;
        min_mem[i] = 0;
        init_mem[i] = 0;
    end
end

// Handle writes
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid ) begin
        case ( wr_addr )
            opt_min_addr : min_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opt_max_addr : max_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opt_init_addr : init_mem[wr_chan] <= wr_data[W_DOUT-1:0];
            opt_mult_addr : mult_mem[wr_chan] <= wr_data[W_MULT-1:0];
            opt_rs_addr : rs_mem[wr_chan] <= wr_data[W_RS-1:0];
            opt_add_chan_addr : add_chan_mem[wr_chan] <= wr_data[W_CHAN:0];
        endcase
    end
end

//--------------------------------------------------------------------
// Internal Memory
//--------------------------------------------------------------------
reg signed [W_DOUT-1:0] dout_prev_mem[0:N_CHAN-1];
reg signed [W_DMTRS-1:0] dmtrs_prev_mem[0:N_CHAN-1];

// Initialize
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        dout_prev_mem[i] = 0;
        dmtrs_prev_mem[i] = 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 1: Fetch
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p1;

always @( * ) begin
    flush_p1 = ( rst_in || clr_rqst[chan_in] );

    // Decide whether to inject
    inj_in = ( !dv_in && |inj_rqst );

    // Decode injection channel preferencing low to high
    inj_chan_in = NULL_CHAN;
    for ( i = N_CHAN - 1; i >= 0; i = i - 1 ) begin
        if ( inj_rqst[i] ) begin
            inj_chan_in = i[W_CHAN-1:0];
        end
    end
end

// Registers
reg inj_p1 = 0;
reg dv_p1 = 0;
reg [W_CHAN-1:0] chan_p1 = 0;
reg signed [W_DELTA-1:0] delta_p1 = 0;
reg signed [W_MULT-1:0] mult_p1 = 0;
reg [W_RS-1:0] rs_p1 = 0;
reg [W_CHAN:0] add_chan_p1 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p1 ) begin
        // Inject initial value write instruction if there are
        // injection requests pending and input data is not valid.
        // Otherwise, register input data as usual.
        if ( inj_in ) begin
            inj_p1 <= 1;
            dv_p1 <= 1;
            chan_p1 <= inj_chan_in;
            delta_p1 <= 0;
        end else begin
            inj_p1 <= 0;
            dv_p1 <= dv_in;
            chan_p1 <= chan_in;
            delta_p1 <= delta_in;
        end

        // Fetch multiplier, right shift, and add channel
        mult_p1 <= mult_mem[chan_in];
        rs_p1 <= rs_mem[chan_in];
        add_chan_p1 <= add_chan_mem[chan_in];

    end else begin
        inj_p1 <= 0;
        dv_p1 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 2: Multiply and right shift
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p2;

always @( * ) begin
    flush_p2 = ( rst_in || clr_rqst[chan_p1] );
end

// Registers
reg inj_p2 = 0;
reg dv_p2 = 0;
reg [W_CHAN-1:0] chan_p2 = 0;
reg signed [W_DMTRS-1:0] dmtrs_p2 = 0;
reg signed [W_DMTRS-1:0] add_dmtrs_p2 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p1 ) begin
        // Pass instruction
        inj_p2 <= inj_p1;
        dv_p2 <= dv_p1;
        chan_p2 <= chan_p1;

        // Multiply and right shift delta
        dmtrs_p2 <= (delta_p1 * mult_p1) >>> rs_p1;

        // Fetch add data if add channel is valid. Otherwise set
        // add data value to zero.
        if ( add_chan_p1 < N_CHAN ) begin
            add_dmtrs_p2 <= dmtrs_prev_mem[add_chan_p1];
        end else begin
            add_dmtrs_p2 <= 0;
        end

    end else begin
        inj_p2 <= 0;
        dv_p2 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 3: Sum with add channel data
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p3;

always @( * ) begin
    flush_p3 = ( rst_in || clr_rqst[chan_p2] );
end

// Memory
always @( posedge clk_in ) begin
    // Writeback multiplied and shifted data
    if ( dv_p2 ) begin
        dmtrs_prev_mem[chan_p2] = dmtrs_p2;
    end

    // Zero multiplied and shifted data memory on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            dmtrs_prev_mem[i] = 0;
        end
    end
end

// Registers
reg inj_p3 = 0;
reg dv_p3 = 0;
reg [W_CHAN-1:0] chan_p3 = 0;
reg signed [W_DSUM-1:0] dsum_p3 = 0;
reg signed [W_DOUT-1:0] dout_prev_p3 = 0;
reg signed [W_DOUT-1:0] max_p3 = 0;
reg signed [W_DOUT-1:0] min_p3 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p3 ) begin
        // Pass instruction
        inj_p3 <= inj_p2;
        dv_p3 <= dv_p2;
        chan_p3 <= chan_p2;

        // Sum with add channel data
        dsum_p3 <= dmtrs_p2 + add_dmtrs_p2;

        // Fetch previous output and output bounds
        dout_prev_p3 <= dout_prev_mem[chan_p2];
        max_p3 <= max_mem[chan_p2];
        min_p3 <= min_mem[chan_p2];

    end else begin
        inj_p3 <= 0;
        dv_p3 <= 0;
    end

end

//--------------------------------------------------------------------
// Pipe Stage 4: Compute output
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p4;
reg signed [W_DOUT_UC-1:0] dout_int_p4 = 0;

always @( * ) begin
    flush_p4 = ( rst_in || clr_rqst[chan_p3] );

    // Sum data with previous output
    dout_int_p4 = dsum_p3 + dout_prev_p3;
end

// Registers
reg inj_p4 = 0;
reg dv_p4 = 0;
reg [W_CHAN-1:0] chan_p4 = 0;
reg signed [W_DOUT-1:0] dout_p4 = 0;
reg signed [W_DOUT-1:0] init_p4 = 0;

always @( posedge clk_in ) begin
    if ( !flush_p4 ) begin
        // Pass instruction
        inj_p4 <= inj_p3;
        dv_p4 <= dv_p3;
        chan_p4 <= chan_p3;

        // Handle output bounds violations
        if ( dout_int_p4 > max_p3 ) begin
            dout_p4 <= max_p3;
        end else if ( dout_int_p4 < min_p3 ) begin
            dout_p4 <= min_p3;
        end else begin
            dout_p4 <= dout_int_p4[W_DOUT-1:0];
        end

        // Fetch initial output value
        init_p4 <= init_mem[chan_p3];

    end else begin
        inj_p4 <= 0;
        dv_p4 <= 0;
    end
end

//--------------------------------------------------------------------
// Pipe Stage 5: Inject output and writeback
//--------------------------------------------------------------------
// Intermediate signals
reg flush_p5;

always @( * ) begin
    flush_p5 = ( rst_in || clr_rqst[chan_p4] );
end

// Memory
always @( posedge clk_in ) begin
    // Writeback output
    if ( dv_p4 ) begin
        dout_prev_mem[chan_p4] = dout_p5;
    end

    // Set previous output memory to initial value on reset or clear
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        if ( rst_in || clr_rqst[i] ) begin
            dout_prev_mem[i] = init_mem[i];
        end
    end
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

        // Inject initial value if inject flag set
        dout_p5 <= ( inj_p4 ) ? init_p4 : dout_p4;

    end else begin
        dv_p5 <= 0;
    end
end

//--------------------------------------------------------------------
// Output Assigment
//--------------------------------------------------------------------
assign dv_out = dv_p5;
assign chan_out = chan_p5;
assign data_out = dout_p5;

endmodule
