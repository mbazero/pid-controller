`timescale 1ns / 1ps

// dac_instr_queue -- mba 2014
// -----------------------------------------------------------
// Queues DAC write instructions for the DAC controller which
// can only process one instruction at a time. Specifically
// written for the AD7608 as configured on the MIST breakout
// board, which outputs two data words at a time according to
// the channel pattern 1-5, 2-6, 3-7, 4-8. Thus, this module
// supports simultaneous processing of DAC instructions on
// channels 1-4 and channels 5-8.
// -----------------------------------------------------------

module dac_instr_queue #(
	// parameters
	parameter W_DATA		= 16,		// width of dac data signal
	parameter W_CHS		= 3,		// width of dac channel select signal
	parameter N_CHAN		= 8		// number of DAC channels
	)(
	// inputs <-- top level entity
	input wire								clk_in,			// system clock
	input wire								reset_in,		// system reset

	// inputs <-- DAC output preprocessor
	input wire [W_DATA*N_CHAN-1:0]	data_bus_in,	// input channels on a single bus
	input wire [N_CHAN-1:0]				data_valid_in,	// data valid signal

	// inputs <-- dac controller
	input wire								rd_ack_in,		// read acknowledge causes next data word to be presented (if one exists)

	// outputs <-- dac controller
	output wire [W_DATA-1:0]			data_out,		// output data
	output wire	[W_CHS-1:0]				chan_out,		// dac channel associated with output data
	output wire								data_valid_out	// output data valid signal
	);


//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* local parameters */
localparam W_DINS = W_DATA + W_CHS;							// width of dac write instruction

/* upper split data */
wire [W_DATA*N_CHAN/2-1:0] data_bus_upper;				// upper half of input data bus
wire [N_CHAN/2-1:0]			dv_upper; 						// upper half of data valid bus
wire								dv_rdc_upper;					// reduction OR of upper data valid bus

/* lower split data */
wire [W_DATA*N_CHAN/2-1:0]	data_bus_lower;				// lower half of input data bus
wire [N_CHAN/2-1:0]			dv_lower; 						// lower half of data valid bus
wire								dv_rdc_lower;					// reduction OR of lower data valid bus

/* mux data out */
wire [W_DATA-1:0] mux_dout_upper, mux_dout_lower;		// mux output data for upper and lower channels

/* mux selects */
wire [W_CHS-1:0] mux_sel_upper, mux_sel_lower;					// mux select signals for upper and lower channels

/* channel numbers (generated from data valid) */
wire [W_CHS-1:0] chan_upper, chan_lower;					// upper and lower channel signals

/* dac instruction signals */
wire [W_DINS-1:0] dinstr_upper, dinstr_lower;			// dac instruction signals for upper and lower channels

/* upper channel delay registers */
reg [W_DINS-1:0] dinstr_upper_reg;
reg dv_rdc_upper_reg;

/* fifo wires */
wire [W_DINS-1:0] fifo_din, fifo_dout;
wire fifo_wr_en, fifo_rd_en;
wire fifo_data_valid;

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* split input data bus into upper and lower halfs */
assign data_bus_upper = data_bus_in[W_DATA*N_CHAN-1:W_DATA*N_CHAN/2];
assign data_bus_lower = data_bus_in[W_DATA*N_CHAN/2-1:0];

/* split data valid input into upper and lower half */
assign dv_upper = data_valid_in[N_CHAN-1:N_CHAN/2];
assign dv_lower = data_valid_in[N_CHAN/2-1:0];

/* compute reduction or of data valid signals */
assign dv_rdc_upper = | dv_upper;
assign dv_rdc_lower = | dv_lower;

/* convert one-hot data valid signals to binary select signals */
assign mux_sel_upper = bin_from_oh ( dv_upper );
assign mux_sel_lower = bin_from_oh ( dv_lower );

/* convert select signals to channel signals */
assign chan_upper = mux_sel_upper + 4;
assign chan_lower = mux_sel_lower;

/* combine data and channel signals to form dac instruction */
assign dinstr_upper = { chan_upper, mux_dout_upper };
assign dinstr_lower = { chan_lower, mux_dout_lower };

/* fifo signals */
assign fifo_din 	= ( dv_rdc_lower ) ? dinstr_lower : dinstr_upper_reg;
assign fifo_wr_en = dv_rdc_lower | dv_rdc_upper_reg;

/* output signals */
assign { chan_out, data_out }	= fifo_dout;
assign data_valid_out 			= fifo_data_valid;

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* register upper channel data signals for one cycle before
 * the fifo mux to allow the lower channel signals to be written
 * first in the case of simultaneous arrival */
always @( posedge clk_in ) begin
	dinstr_upper_reg	<= dinstr_upper;
	dv_rdc_upper_reg	<= dv_rdc_upper;
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* upper channels mux */
mux_n_chan #(
	.W_CHAN				(W_DATA),
	.W_SEL				(W_CHS),
	.N_IN					(N_CHAN/2))
mux_upper (
	.data_bus_in		(data_bus_upper),
	.chan_select_in	(mux_sel_upper),
	.data_out			(mux_dout_upper)
	);

/* lower channels mux */
mux_n_chan #(
	.W_CHAN				(W_DATA),
	.W_SEL				(W_CHS),
	.N_IN					(N_CHAN/2))
mux_lower (
	.data_bus_in		(data_bus_lower),
	.chan_select_in	(mux_sel_lower),
	.data_out			(mux_dout_lower)
	);

/* fifo */
fifo_19 instr_queue (
	.clk		(clk_in),
	.rst		(reset_in),
	.din		(fifo_din),
	.wr_en	(fifo_wr_en),
	.rd_en	(rd_ack_in),
	.dout		(fifo_dout),
	.full		(),
	.empty	(),
	.valid	(fifo_data_valid)
	);

//////////////////////////////////////////
// functions
//////////////////////////////////////////

/* four-bit one hot to two-bit binary converter*/
function [W_CHS-1:0] bin_from_oh;
	input [3:0] one_hot;
	begin
		case ( one_hot )
			4'b0001 : bin_from_oh = 0;
			4'b0010 : bin_from_oh = 1;
			4'b0100 : bin_from_oh = 2;
			4'b1000 : bin_from_oh = 3;
			default : bin_from_oh = 0;
		endcase
	end
endfunction

endmodule
