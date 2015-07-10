`timescale 1ns / 1ps

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
    parameter N_SRC = 8,
    parameter W_SRC = 5,
    parameter N_CHAN = 8,
    parameter W_CHAN = 5,
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
    output wire [W_DATA-1:0] data_out
    );

`include "ep_map.vh"

//--------------------------------------------------------------------
// Constants
//--------------------------------------------------------------------
localparam [W_SRC:0] NULL_SRC = {W_SRC+1{1'b1}};
localparam [W_CHAN:0] NULL_CHAN = {W_CHAN+1{1'b1}};

//--------------------------------------------------------------------
// External Memory
//--------------------------------------------------------------------
reg [N_CHAN-1:0] chan_en_mem;
reg [W_SRC:0] chan_src_sel_mem[0:N_CHAN-1];
wire wr_chan_valid = ( wr_chan < N_CHAN );

// Initialize
integer i;
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        chan_en_mem[i] = 0;
        chan_src_sel_mem[i] = NULL_SRC;
    end
end

// Handle writes
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid ) begin
        case ( wr_addr )
            chan_en_addr : chan_en_mem[wr_chan] <= wr_data[0];
            chan_src_sel_addr : chan_src_sel_mem[wr_chan] <= wr_data[W_SRC:0];
        endcase
    end
end

//--------------------------------------------------------------------
// Source Channel Map
//--------------------------------------------------------------------
reg [N_CHAN-1:0] src_chan_map[0:N_SRC-1];
wire [W_SRC:0] wr_src_chan = wr_data[W_SRC:0];

// Initialize
initial begin
    for (i = 0; i < N_SRC; i = i + 1 ) begin
        src_chan_map[i] = 0;
    end
end

// Update map when new route is received
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid &&
        ( wr_addr == chan_src_sel_addr )) begin
        // Clear old mapping
        src_chan_map[chan_src_sel_mem[wr_chan]][wr_src_chan] <= 0;

        // Register new mapping if valid
        if ( wr_src_chan < N_SRC ) begin
            src_chan_map[wr_src_chan][wr_chan] <= 1;
        end
    end
end

//--------------------------------------------------------------------
// Input Buffer
//--------------------------------------------------------------------
wire buf_dv;
wire [W_SRC-1:0] buf_src;
wire [W_DATA-1:0] buf_data;
reg buf_rd_en;

fifo_21 input_buffer (
   .clk     (clk_in),
   .rst     (rst_in),
   .din     ({src_in, data_in}),
   .wr_en   (dv_in),
   .rd_en   (buf_rd_en),
   .dout    ({buf_src, buf_data}),
   .valid   (buf_dv),
   .full    (),
   .empty   ()
   );

//--------------------------------------------------------------------
// Dispatcher
//--------------------------------------------------------------------
reg dspch_dv;
reg [W_CHAN:0] dspch_chan;
reg [N_CHAN-1:0] instr_sent;
reg [W_CHAN-1:0] icount;

// Mask source channel map with complement of instruction sent
// register to get a map of remaining instructions to send
wire [N_CHAN-1:0] instr_to_send = src_chan_map[buf_src] & ~instr_sent;

// Dispatch instructions if source has valid channel mappings and if
// the mapped channels are active. A new instruction is dispatched
// every clock cycle for each valid channel mapping. Lower numbered
// channels are given dispatch priority. Buffer read enable is asserted
// when there is one instruction left to send, so new data is available
// on the next clock cycle.
always @( posedge clk_in ) begin
    dspch_chan = NULL_CHAN;
    icount = 0;

    if ( rst_in | ~buf_dv ) begin
        dspch_dv = 0;
        instr_sent = 0;
        buf_rd_en = 0;

    end else if ( |instr_to_send ) begin
        // Decode channel number
        for ( i = N_CHAN - 1; i >= 0; i = i - 1 ) begin
            if ( instr_to_send[i] ) begin
                icount = icount + 1;
                dspch_chan = i;
            end
        end

        // Dispatch instruction
        dspch_dv = chan_en_mem[dspch_chan];
        instr_sent[dspch_chan] = 1;
        buf_rd_en = ( icount == 1 ) ? 1'b1 : 1'b0;

    end else begin
        dspch_dv = 0;
        instr_sent = 0;
        buf_rd_en = 1;
    end
end

//--------------------------------------------------------------------
// Output Assignment
//--------------------------------------------------------------------
assign dv_out = dspch_dv;
assign chan_out = dspch_chan[W_CHAN-1:0];
assign data_out = buf_data;

endmodule
