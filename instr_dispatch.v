`timescale 1ns / 1ps

// instr_dispatch -- mba 2015
// -----------------------------------------------------------
// Dispatch instructions into the pid pipeline. ADC data is
// stored in a FIFO queue as it is received. Data words are
// pulled from the queue one at a time. The adc channel
// number for the data is used to fetch an output bitmap
// specifying the output channels the input drives. A single
// input channel may drive multiple outputs. Seperate
// instructions are dispatched for each output channel, one
// after the other. When all instructions have been dispatched
// for a give data word, the next word is fetched from the fifo.
// -----------------------------------------------------------

module instr_dispatch #(
    // parameters
    parameter W_DATA    = 18,
    parameter W_CHS     = 3,
    )(
    // inputs <- top level entity
    input wire          clk_in,
    input wire          reset_in,

    // inputs <- clock sync
    input wire                  data_valid_in,
    input wire  [W_CHS-1:0]     chan_in,
    input wire  [W_DATA-1:0]    data_in,
