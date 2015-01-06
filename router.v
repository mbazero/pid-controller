`timescale 1ns / 1ps

// router -- mba 2014

// todo
// - I think we might a problem here were all output channels source from adc channel zero initially
//		> this is fine while channel 0 is off, but as soon as it is activated it will start pushing multiple channels
//		> for testing I will simply initialize all output channels to source from some channel i know will be deactivated

module router #(
	// parameters
	parameter W_CHAN	= 16,										// width of input channels
	parameter W_SEL	= 4,										// width of select signal
	parameter N_IN		= 8,										// number of input channels
	parameter N_OUT	= 8										// number of output channels
	)(
	// inputs <- top level entity
	input wire								clk_in,				// system clock
	
	// inputs <- pid core 
	input wire	[W_CHAN*N_IN-1:0]		data_in,				// input channels on a single bus
	
	// inputs <- frontpanel controller
	input wire	[W_SEL-1:0]				src_select_in,		// source channel select
	input wire	[W_SEL-1:0]				dest_select_in,	// destination channel select
	input wire								update_in,			// update frontpanel params
	
	// outputs -> output preprocessor
	output wire	[W_CHAN*N_OUT-1:0]	data_out				// output channels on a single bus
   );
	
//////////////////////////////////////////
// internal structures
//////////////////////////////////////////
reg	[W_SEL-1:0]		src_select		[0:N_OUT-1];		// active source channel
wire	[W_CHAN-1:0]	mux_data_out	[0:N_OUT-1]; 		// output channels in discrete structures
			
//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////
genvar i;
generate 
	for ( i = 0; i < N_OUT; i = i+1 ) begin : out_array
		assign data_out[ i*W_CHAN +: W_CHAN ] = mux_data_out[i]; 
	end
endgenerate

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* mux array */ 
genvar j;
generate
	for ( j = 0; j < N_OUT; j = j+1 ) begin : mux_array
		mux_n_chan #(
			.W_CHAN				(W_CHAN),
			.W_SEL				(W_SEL),
			.N_IN					(N_IN))
		mux_inst ( 
			.data_in				(data_in),
			.chan_select_in	(src_select[j]),
			.data_out			(mux_data_out[j])
			); 
	end
endgenerate 

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* update frontpanel params */
always @( posedge update_in ) begin
	src_select[dest_select_in] <= src_select_in; 
end

endmodule
