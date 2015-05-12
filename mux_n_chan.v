`timescale 1ns / 1ps

// mux_n_chan -- mba 2014

module mux_n_chan #(
	// parameters
	parameter W_CHAN	= 16,									// width of input channels
	parameter W_SEL	= 4,									// width of select signal
	parameter N_IN 	= 8									// number of input channels
	)(
	// inputs
	input wire[W_CHAN*N_IN-1:0]	data_packed_in,	// input channels packed on a single bus
	input wire[W_SEL-1:0]			chan_select_in,	// channel select

	// outputs
	output wire[W_CHAN-1:0]			data_out				// data out bus
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* input channels seperated into discrete structures */
wire	[W_CHAN-1:0]	channel	[0:N_IN-1];

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* channels */
genvar i;
generate
	for ( i = 0; i < N_IN; i = i+1 ) begin : chan_array
		assign channel[i] = data_packed_in[ i*W_CHAN +: W_CHAN ];
	end
endgenerate

/* data out */
assign data_out = channel[chan_select_in];

endmodule
