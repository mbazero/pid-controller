`timescale 1ns / 1ps
`include "parameters.vh"

//--------------------------------------------------------------------
// Instruction Dispatch -- mba 2015
//--------------------------------------------------------------------
// Decode PID channel from source and dispatch instructions into the PID
// pipeline. ADC data is stored in a FIFO queue as it is received. Data words
// are pulled from the queue one at a time. The adc channel number for the
// data is used to fetch an output bitmap specifying the output channels the
// input drives. A single input channel may drive multiple outputs. Seperate
// instructions are dispatched for each output channel, one after the other.
// When all instructions have been dispatched for a give data word, the next
// word is fetched from the fifo.
//--------------------------------------------------------------------

module instr_dispatch #(
    // parameters
    parameter W_SRC = 5,
    parameter W_DATA = 18,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48,
    parameter W_CHAN = 5,
    parameter N_CHAN = 8
    )(
    // Inputs
    input wire clk_in,
    input wire rst_in,

    input wire dv_in,
    input wire [W_SRC-1:0] src_in,
    input wire [W_DATA-1:0] data_in,

    input wire wr_en,
    input wire [W_WR_ADDR-1:0] wr_addr,
    input wire [W_WR_CHAN-1:0] wr_chan,
    input wire [W_WR_DATA-1:0] wr_data,

    // Outputs
    output wire dv_out,
    output wire [W_CHAN-1:0] chan_out,
    output wire [W_DATA-1:0] data_out
    );

//--------------------------------------------------------------------
// Structures
//--------------------------------------------------------------------
// Internal state
reg [N_CHAN-1:0] instr_sent;
reg [W_CHAN-1:0] chan;

// External state
reg [N_CHAN-1:0] chan_active;
reg [W_SRC-1:0] chan_src_sel[0:N_CHAN-1]; // PID channel to source mappings

// Fifo signals
wire [W_SRC-1:0] src;
wire fifo_rd_en;
wire fifo_dv;

//--------------------------------------------------------------------
// Logic
//--------------------------------------------------------------------
// Input data buffer
fifo_21 adc_instr_queue (
   .clk     (clk_in),
   .rst     (rst_in),
   .din     ({src_in, data_in}),
   .wr_en   (dv_in),
   .rd_en   (fifo_rd_en),
   .dout    ({src, data_out}),
   .valid   (fifo_dv)
   );

// PID channel decoder
// Decode PID channel giving priority to lower numbered channels. If
// valid channel found, inject instruction into pipeline and continue
// search for other valid channels. If no other valid channels found,
// pull new data from the FIFO. Observe that blocking assignment are
// used. Decoder functionality depends on this.
always @( posedge clk_in ) begin
    // Default null channel assignment if source is not routed
    chan_out = NULL_CHAN;

    // Decode PID channel
    for ( chan = N_CHAN; chan >= 0; chan = chan - 1 ) begin
        if ( chan_src_sel[chan] == src
            & instr_sent[chan] == 0 ) begin
            chan_out = chan;
        end
    end

    // Clear registers if reset is high or fifo data is invalid.
    // Otherwise, if no valid PID channel found, clear sent
    // instruction map and pull new data from input buffer.
    // Otherwise, channel is valid, so send instruction
    // if the channel is active and set sent flag.
    if ( rst_in | ~fifo_dv ) begin
        dv_out = 0;
        instr_sent = 0;
        fifo_rd_en = 0;
    end else if ( chan_out == NULL_CHAN ) begin
        dv_out = 0;
        instr_sent = 0;
        fifo_rd_en = 1;
    end else begin
        dv_out = chan_active[chan_out];
        instr_sent[chan_out] = 1;
        fifo_rd_en = 0;
    end
end

// External state write handling
always @( posedge wr_en ) begin
    case ( wr_addr ) begin
        chan_src_sel_addr : chan_src_sel[wr_chan] <= wr_data[W_SRC-1:0];
        chan_active_addr : chan_active[wr_chan] <= wr_data[0];
    end
end

// Memory initialization
genvar i;
generate
	for ( i = 0; i < N_CHAN; i = i+1 ) begin : src_select_init
		initial chan_src_sel[i] = NULL_SRC;
	end
endgenerate;

endmodule
