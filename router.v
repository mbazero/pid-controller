`timescale 1ns / 1ps

// router -- mba 2014

module router #(
	// parameters
	parameter W_CHAN	= 16,										// width of input channels
	parameter W_SEL	= 4,										// width of select signals
	parameter N_IN		= 8,										// number of input channels
	parameter N_OUT	= 8,										// number of output channels
	parameter ACTV_INIT = 1										// initial output activation
	)(
	// inputs <- pid core
	input wire	[W_CHAN*N_IN-1:0]		data_packed_in,	// input channels packed on a single bus

	// inputs <- frontpanel controller
	input wire	[W_SEL-1:0]				src_select_in,		// source channel select
	input wire	[W_SEL-1:0]				dest_select_in,	// destination channel select
	input wire	[N_OUT-1:0]				output_active_in,	// output channel activation signals
	input wire								update_in,			// update frontpanel params

	// outputs -> output preprocessor
	output wire	[W_CHAN*N_OUT-1:0]	data_packed_out	// output channels packed on a single bus
   );

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

reg	[W_SEL-1:0]		src_select		[0:N_OUT-1];		// registered source channels
reg	[N_OUT-1:0]		output_active = ACTV_INIT;			// registed output channel activations
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
		initial src_select[j] = 0;
	end
endgenerate

/* update frontpanel params */
always @( posedge update_in ) begin
	src_select[dest_select_in] <= src_select_in;
	output_active <= output_active_in;
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
			.data_out			(mux_data_out[k])
			);
		assign data_out[k] = (output_active[k] == 1) ? mux_data_out[k] : {W_CHAN{1'b0}}; // only pass mux output if output channel is activated
	end
endgenerate

endmodule
