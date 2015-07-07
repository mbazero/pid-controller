`timescale 1ns / 1ps
`include "ep_map.vh"

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
    parameter W_SRC = 5,
    parameter W_CHAN = 5,
    parameter N_CHAN = 8,
    parameter W_DATA = 18,
    parameter W_WR_ADDR = 16,
    parameter W_WR_CHAN = 16,
    parameter W_WR_DATA = 48
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
    output wire [W_DATA-1:0] data_out,
    );

//--------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------
reg [W_SRC:0] null_src = 1 << W_SRC;
reg [W_CHAN:0] null_chan = 1 << W_CHAN;

//--------------------------------------------------------------------
// Configuration Memory
//--------------------------------------------------------------------
reg [N_CHAN-1:0] chan_en_mem;
reg [W_SRC:0] chan_src_sel_mem[0:N_CHAN-1];

// Initialize memory
integer i;
initial begin
    for ( i = 0; i < N_CHAN; i = i+1 ) begin
        chan_src_sel_mem[i] = null_src;
        chan_en_mem[i] = 0;
    end
end

// Handle write requests
always @( posedge clk_in ) begin
    if ( wr_en ) begin
        case ( wr_addr ) begin
            chan_src_sel_addr : chan_src_sel_mem[wr_chan] <= wr_data[W_SRC:0];
            chan_en_addr : chan_en_mem[wr_chan] <= wr_data[0];
        end
    end
end

//--------------------------------------------------------------------
// Data Buffer
//--------------------------------------------------------------------
wire [W_SRC-1:0] fifo_src;
wire [W_DATA-1:0] fifo_data;
wire fifo_rd_en;
wire fifo_dv;

fifo_21 input_buffer (
   .clk     (clk_in),
   .rst     (rst_in),
   .din     ({src_in, data_in}),
   .wr_en   (dv_in),
   .rd_en   (fifo_rd_en),
   .dout    ({fifo_src, fifo_data}),
   .valid   (fifo_dv)
   );

//--------------------------------------------------------------------
// Channel Decoder
//--------------------------------------------------------------------
wire dec_dv;
reg [W_CHAN:0] dec_chan;
reg [N_CHAN-1:0] instr_sent;

// Decode PID channel giving priority to lower numbered channels. If
// valid channel found, inject instruction into pipeline and continue
// search for other valid channels. If no other valid channels found,
// pull new data from the FIFO. Observe that blocking assignment are
// used. Decoder functionality depends on this.
always @( posedge clk_in ) begin
    // Default null channel assignment if source is not routed
    dec_chan = null_chan;

    // Decode PID channel
    for ( i = N_CHAN; i >= 0; i = i - 1 ) begin
        if ( chan_src_sel_mem[i] == fifo_src
            && !instr_sent[i] ) begin
            dec_chan = i;
        end
    end

    // Clear registers if reset is high or fifo data is invalid.
    // Otherwise, if no valid PID channel found, clear sent
    // instruction map and pull new data from input buffer.
    // Otherwise, channel is valid, so send instruction
    // if the channel is active and set sent flag.
    if ( rst_in | ~fifo_dv ) begin
        dec_dv = 0;
        instr_sent = 0;
        fifo_rd_en = 0;
    end else if ( dec_chan == null_chan ) begin
        dec_dv = 0;
        instr_sent = 0;
        fifo_rd_en = 1;
    end else begin
        dec_dv = chan_en_mem[dec_chan];
        instr_sent[dec_chan] = 1;
        fifo_rd_en = 0;
    end
end

//--------------------------------------------------------------------
// Output Assignment
//--------------------------------------------------------------------
assign dv_out = dec_dv;
assign chan_out = dec_chan;
assign data_out = fifo_data;

endmodule
