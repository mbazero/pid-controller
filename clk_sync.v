`timescale 1ns / 1ps

// clk_sync -- mba 2014
// -----------------------------------------------------------
// Synchronizes adc data from the adc clock domain to the
// system clock domain. Also serializes adc data so only one
// word is presented at a time.
// -----------------------------------------------------------

module clk_sync #(
    // parameters
    parameter W_DATA    = 18,
    parameter W_CHAN     = 3,
    parameter N_ADC     = 8
    )(
    // inputs <- top level entity
    input wire                  sync_clk_in,
    input wire                  reset_in,

    // inputs <- adc controller
    input wire                  dv_in,
    input wire  [W_CHAN-1:0]     chan_a_in,
    input wire  [W_CHAN-1:0]     chan_b_in,
    input wire  [W_DATA-1:0]    data_a_in,
    input wire  [W_DATA-1:0]    data_b_in,

    // outputs -> oversample filter
    output wire [N_ADC-1:0]     dv_out,
    output wire [W_CHAN-1:0]     chan_out,
    output wire [W_DATA-1:0]    data_out
    );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

/* state parameters */
localparam  ST_WAIT_DVH = 3'd0,     // wait for data valid to go high
            ST_HOLD     = 3'd1,     // latch and hold data to satisfy hold time
            ST_SEND     = 3'd2,     // assert dv_out synchronous with 50MHz clock
            ST_WAIT_DVL = 3'd3;     // wait for data valid to go low

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* 50MHz data registers */
reg dv = 0;
reg [W_CHAN-1:0] chan_a = 0;
reg [W_CHAN-1:0] chan_b = 0;
reg [W_DATA-1:0] data_a = 0;
reg [W_DATA-1:0] data_b = 0;

/* state registers */
reg [2:0] cur_state = ST_WAIT_DVH;
reg [2:0] next_state = ST_WAIT_DVH;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* data output */
always @( * ) begin
    case ( cur_state ) begin
        ST_SEND_A: begin
            data_out = data_a;
            chan_out = chan_a;
            dv_out = 1;
        end
        ST_SEND_B: begin
            data_out = data_b;
            chan_out = chan_b;
            dv_out = 1;
        end
        default: begin
            data_out = 0;
            chan_out = 0;
            dv_out = 0;
        end
    end
end

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* data registers */
always @( posedge sync_clk_in ) begin
    if ( reset_in == 1 ) begin
        dv <= 0;
        chan_a <= 0;
        chan_b <= 0;
        data_a <= 0;
        data_b <= 0;
    end else if ( cur_state == ST_WAIT_DVH & dv_in == 1 ) begin
        dv <= dv_in;
        chan_a <= chan_a_in;
        chan_b <= chan_b_in;
        data_a <= data_a_in;
        data_b <= data_b_in;
    end
end

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* state register - synchronous with system clock */
always @( posedge sync_clk_in ) begin
    if ( reset_in == 1 ) begin
        cur_state <= ST_WAIT_DVH;
    end else begin
        cur_state <= next_state;
    end
end

/* next state transitin logic */
always @( * ) begin
    next_state <= cur_state; // default assignment if no case statement is satisfied
    case ( cur_state )
        ST_WAIT_DVH: begin
            if ( dv_in == 1 ) begin
                next_state <= ST_HOLD;
            end
        end
        ST_HOLD: begin
            next_state <= ST_SEND_A;
        end
        ST_SEND_A: begin
            next_state <= ST_SEND_B;
        end
        ST_SEND_B: begin
            next_state <= ST_WAIT_DVL;
        end
        ST_WAIT_DVL: begin
            if ( dv_rdc == 0 ) begin
                next_state <= ST_WAIT_DVH;
            end
        end
    endcase
end

endmodule
