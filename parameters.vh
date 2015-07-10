// ----------------------------- i/o params -----------------------------------
// - set number of DDS channels to match hardware configuration
// - don't forget to also set DDS output pin mappings in xem6010.ucf
//   constraints file to match hardward configuration
// - reduce number ADC or DAC channels to save FPGA memory if full number of
//   channels not needed
// ----------------------------------------------------------------------------
localparam N_ADC			= 8;		// number of adc channels to generate
localparam N_DAC			= 8;		// number of dac channels to generate
localparam N_DDS			= 0;		// number of dds channels to generate
localparam W_ADC_DATA		= 18; 	// width of adc data word
localparam W_ADC_CHAN    = 3;     // width of adc channel select
localparam W_ADC_OS      = 3;     // width of adc oversample mode signal
localparam W_DAC_DATA		= 16;		// width of dac data word
localparam W_DAC_CHAN    = 3;     // width of dac channel select
localparam W_FREQ_DATA	= 48; 	// width of dds frequency word
localparam W_PHASE_DATA	= 14;		// width of dds phase word
localparam W_AMP_DATA 	= 10; 	// width of dds amplitude instruction

// -------------------------- output map params -------------------------------
// - absolute output channel descriptors are mapped to relative descriptors
//   according to the table below
// ----------------------------------------------------------------------------
//	[ 0 						: N_DAC - 1					] - DAC Channels
//	[ N_DAC					: N_DAC + N_DDS - 1		] - DDS Frequency Channels
//	[ N_DAC + N_DDS		: N_DAC + 2*N_DDS - 1	] - DDS Phase Channels
//	[ N_DAC + 2*N_DDS		: N_DAC + 3*N_DDS - 1	] - DDS Amplitude Channels
// ----------------------------------------------------------------------------
localparam DAC0_ADDR			= 0;						// DAC channel 0 output address
localparam FREQ0_ADDR			= N_DAC;					// Frequency channel 0 output address
localparam PHASE0_ADDR		= N_DAC + N_DDS;		// Phase channel 0 output address
localparam AMP0_ADDR			= N_DAC + 2 * N_DDS;	// Amplitude channel 0 output address

// ----------------------------- misc. params ---------------------------------
// - don't change any of these unless hardware on
//   breakout board changes
// ----------------------------------------------------------------------------
localparam W_COMP			= 64; 	// width of computation registers
localparam W_EP				= 16; 	// width of opal kelly endpoint
localparam PIPE_DEPTH		= 1024;	// depth of pipe out fifo specified in during core gen

// ---------------------------- derived params --------------------------------
// - don't change these
// ----------------------------------------------------------------------------
localparam W_PID_OS			= 5;						// width of PID oversample signal
localparam N_PID_SRC        = N_ADC;       // total number of PID source channels
localparam NULL_SRC         = N_ADC + 1;
localparam W_PID_SRC        = W_ADC_CHAN;        // width of PID source channel select
localparam N_PID_CHAN  		= N_DAC + 3*N_DDS;	// total number of PID channels equal to number of output channels
localparam W_PID_CHAN       = 5;   // width of PID output channel select
localparam W_PID_DIN        = W_ADC_DATA;        // width of PID input data
localparam W_PID_DOUT       = W_FREQ_DATA + 1;   // width of PID data output; must greater or equal to the max output width plus a sign bit.
localparam W_PID_COMP       = 64;               // width of PID computation registers. A larger value means increased PID precision, but an increased FPGA area.
localparam W_PID_OPRNDS     = W_EP;              // width of PID operands

localparam W_WR_ADDR     = W_EP;
localparam W_WR_CHAN     = W_PID_CHAN;
localparam W_WR_DATA     = W_FREQ_DATA + 1;
