// ----------------------------- i/o params -----------------------------------
// - set number of DDS channels to match hardware configuration
// - don't forget to also set DDS output pin mappings in xem6010.ucf
//   constraints file to match hardward configuration
// - reduce number ADC or DAC channels to save FPGA memory if full number of
//   channels not needed
// ----------------------------------------------------------------------------
parameter N_ADC			= 8;		// number of adc channels to generate
parameter N_DAC			= 8;		// number of dac channels to generate
parameter N_DDS			= 1;		// number of dds channels to generate
parameter W_EP          = 16;    // width of frontpanel endpoints
parameter W_ADC_DATA		= 18; 	// width of adc data word
parameter W_ADC_CHAN    = 3;     // width of adc channel select
parameter W_ADC_OS      = 3;     // width of adc oversample mode signal
parameter W_DAC_DATA		= 16;		// width of dac data word
parameter W_DAC_CHAN    = 3;     // width of dac channel select
parameter W_FREQ_DATA	= 48; 	// width of dds frequency word
parameter W_PHASE_DATA	= 14;		// width of dds phase word
parameter W_AMP_DATA 	= 10; 	// width of dds amplitude instruction

// -------------------------- output map params -------------------------------
// - absolute output channel descriptors are mapped to relative descriptors
//   according to the table below
// ----------------------------------------------------------------------------
//	[ 0 						: N_DAC - 1					] - DAC Channels
//	[ N_DAC					: N_DAC + N_DDS - 1		] - DDS Frequency Channels
//	[ N_DAC + N_DDS		: N_DAC + 2*N_DDS - 1	] - DDS Phase Channels
//	[ N_DAC + 2*N_DDS		: N_DAC + 3*N_DDS - 1	] - DDS Amplitude Channels
// ----------------------------------------------------------------------------
parameter DAC0_ADDR			= 0;						// DAC channel 0 output address
parameter FREQ0_ADDR			= N_DAC;					// Frequency channel 0 output address
parameter PHASE0_ADDR		= N_DAC + N_DDS;		// Phase channel 0 output address
parameter AMP0_ADDR			= N_DAC + 2 * N_DDS;	// Amplitude channel 0 output address

// ----------------------------- misc. params ---------------------------------
// - don't change any of these unless hardware on
//   breakout board changes
// ----------------------------------------------------------------------------
parameter W_COMP			= 128; 	// width of computation registers
parameter W_EP				= 16; 	// width of opal kelly endpoint
parameter W_WR_ADDR     = W_EP;
parameter W_WR_CHAN     = W_EP;
parameter W_WR_DATA     = W_EP*4;
parameter PIPE_DEPTH		= 1024;	// depth of pipe out fifo specified in during core gen

// ---------------------------- derived params --------------------------------
// - don't change these
// ----------------------------------------------------------------------------
parameter W_PID_SRC        = W_ADC_CHAN;        // width of PID source channel select
parameter N_PID_CHAN  		= N_DAC + 3*N_DDS;	// total number of PID channels equal to number of output channels
parameter W_PID_CHAN       = log2(N_PID_CHAN)   // width of PID output channel select
parameter W_PID_DIN        = W_ADC_DATA;        // width of PID input data
parameter W_PID_DOUT       = W_FREQ_DATA + 1;   // width of PID data output; must greater or equal to the max output width plus a sign bit.
parameter W_PID_COMP       = 128;               // width of PID computation registers. A larger value means increased PID precision, but an increased FPGA area.
parameter W_PID_OPRNDS     = W_EP;              // width of PID operands

`include "functions.vh"
