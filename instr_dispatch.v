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
`include "init.vh"

//--------------------------------------------------------------------
// External Memory
//--------------------------------------------------------------------
reg [W_SRC:0] chan_src_sel_mem[0:N_CHAN-1];
wire wr_chan_valid = ( wr_chan < N_CHAN );

// Initialize
integer i;
initial begin
    for ( i = 0; i < N_CHAN; i = i + 1 ) begin
        chan_src_sel_mem[i] = CHAN_SRC_SEL_INIT;
    end
end

// Handle writes
always @( posedge clk_in ) begin
    if ( wr_en && wr_chan_valid ) begin
        case ( wr_addr )
            chan_src_sel_addr : chan_src_sel_mem[wr_chan] <= wr_data[W_SRC:0];
        endcase
    end
end

//--------------------------------------------------------------------
// Source Channel Map
//--------------------------------------------------------------------
reg [N_CHAN-1:0] src_chan_map[0:N_SRC-1];
wire [W_SRC:0] old_src = chan_src_sel_mem[wr_chan];
wire [W_SRC:0] new_src = wr_data[W_SRC:0];

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
        src_chan_map[old_src][wr_chan] <= 0;

        // Register new mapping if valid
        if ( new_src < N_SRC ) begin
            src_chan_map[new_src][wr_chan] <= 1;
        end
    end
end

//--------------------------------------------------------------------
// Input Buffer
//--------------------------------------------------------------------
wire buf_dv;
wire [W_SRC-1:0] buf_src;
wire [W_DATA-1:0] buf_data;
wire buf_rd_en;

idp_fifo input_buf (
   .clk     (clk_in),
   .rst     (rst_in),
   .din     ({src_in, data_in}),
   .wr_en   (dv_in),
   .rd_en   (buf_rd_en),
   .dout    ({buf_src, buf_data}),
   .valid   (buf_dv)
   );

//--------------------------------------------------------------------
// Channel Decoder
//--------------------------------------------------------------------
reg [W_CHAN:0] dec_chan = 0;
reg [N_CHAN-1:0] instr_sent = 0;
reg [W_CHAN-1:0] icount = 0;

// Mask source channel map with complement of instruction sent
// register to get a map of remaining instructions to send
wire [N_CHAN-1:0] instr_to_send = src_chan_map[buf_src] & ~instr_sent;

// Decode channel from source. Lower numbered channels get priority
// if a source has multiple mappings.
always @( * ) begin
    dec_chan = NULL_CHAN;
    icount = 0;

    for ( i = N_CHAN - 1; i >= 0; i = i - 1 ) begin
        if ( instr_to_send[i] ) begin
            icount = icount + 1;
            dec_chan = i[W_CHAN-1:0];
        end
    end
end

//--------------------------------------------------------------------
// Dispatcher
//--------------------------------------------------------------------
reg dspch_dv = 0;
reg [W_CHAN-1:0] dspch_chan = 0;
reg [W_DATA-1:0] dspch_data = 0;

// Dispatch instructions if source has valid channel mappings and if
// the mapped channels are active. A new instruction is dispatched
// every clock cycle for each valid channel mapping.
always @( posedge clk_in ) begin
    if ( rst_in || !buf_dv || icount == 0 ) begin
        dspch_dv = 0;
        instr_sent = 0;

    end else begin
        dspch_dv = 1'b1;
        dspch_chan = dec_chan[W_CHAN-1:0];
        dspch_data = buf_data;
        instr_sent[dspch_chan] = 1;
    end
end

// Request new data from the buffer when there is one or zero
// instructions left to send for the active source. Read requesting
// when there is one instruction left allows independent instructions
// to be dispatched every clock cycle.
assign buf_rd_en = ( !rst_in && buf_dv && icount <= 1 );

//--------------------------------------------------------------------
// Output Assignment
//--------------------------------------------------------------------
assign dv_out = dspch_dv;
assign chan_out = dspch_chan;
assign data_out = dspch_data;

endmodule
