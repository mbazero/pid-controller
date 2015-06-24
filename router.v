`timescale 1ns / 1ps

// router -- mba 2014
// -----------------------------------------------------------
// Routes data streams from ADC input channels to DAC/DDS
// output channels. Negative source routes are invalid and
// effectively deactivate the channel.
// -----------------------------------------------------------

module router #(
	// parameters
	parameter W_CHAN	= 16,										// width of input channels
	parameter W_SEL	= 5,										// width of select signals
	parameter N_IN		= 8,										// number of input channels
	parameter N_OUT	= 8,										// number of output channels
	parameter ACTV_INIT = 1										// initial output activation
	)(
	// inputs <- pid core
	input wire	[W_CHAN*N_IN-1:0]		data_packed_in,	// input channels packed on a single bus

	// inputs <- frontpanel controller
	input wire	[W_SEL-1:0]				src_select_in,		// source channel select
	input wire	[W_SEL-1:0]				dest_select_in,	// destination channel select
	input wire								update_in,			// update frontpanel params

	// outputs -> output preprocessor
	output wire	[W_CHAN*N_OUT-1:0]	data_packed_out	// output channels packed on a single bus
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

reg	[W_SEL-1:0]		src_select		[0:N_OUT-1];		// registered source channels
wire	[W_CHAN-1:0]	mux_data_out	[0:N_OUT-1]; 		// mux output channels
wire	[W_CHAN-1:0]	data_out			[0:N_OUT-1]; 		// final output channels

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

genvar i;
generate
	for ( i = 0; i < N_OUT; i = i+1 ) begin : out_array
		assign data_packed_out[ i*W_CHAN +: W_CHAN ] = data_out[i];
	end
endgenerate

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* initial source select assignments */
genvar j;
generate
	for ( j = 0; j < N_OUT; j = j+1 ) begin : src_select_init
		initial src_select[j] = -1;
	end
endgenerate

/* update frontpanel params */
always @( posedge update_in ) begin
	src_select[dest_select_in] <= src_select_in;
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* mux array */
genvar k;
generate
	for ( k = 0; k < N_OUT; k = k+1 ) begin : mux_array
		mux_n_chan #(
			.W_CHAN				(W_CHAN),
			.W_SEL				(W_SEL),
			.N_IN					(N_IN))
		mux_inst (
			.data_packed_in	(data_packed_in),
			.chan_select_in	(src_select[k]),
			.enable_in			(~src_select[k][W_SEL-1]), // negative routes disable output
			.data_out			(mux_data_out[k])
			);
		assign data_out[k] = mux_data_out[k];
	end
endgenerate

endmodule
