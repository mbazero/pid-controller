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
parameter W_ADC_DATA		= 18; 	// width of adc data word
parameter W_DAC_DATA		= 16;		// width of dac data word
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
parameter W_COMP			= 64; 	// width of computation registers
parameter W_EP				= 16; 	// width of opal kelly endpoint
parameter W_ADC_CHS		= 3;		// width of adc channel select
parameter W_ADC_OS		= 3;		// width of adc oversample signal
parameter W_OSF_OS		= 4;		// width of oversample mode signal
parameter W_OSF_CD		= 16;		// width of osf cycle delay signal
parameter W_INPUT_SEL	= 5;		// width of router select signal (must be log2(N_DAC) + 1...MSB stores channel activation state)
parameter W_OPP_MLT		= 10;		// width of opp multiplication factor; specifies max allowed multiplier
parameter W_DAC_CHS		= 3;		// width of dac channel select
parameter PIPE_DEPTH		= 1024;	// depth of pipe out fifo specified in during core gen
parameter ADC_CYCLE_T	= 5;		// adc cycle time in microseconds when adc_os = 0

// ---------------------------- derived params --------------------------------
// - don't change these
// ----------------------------------------------------------------------------
parameter N_SRC				= N_ADC;					// total number of input source channels
parameter N_CHAN 				= N_DAC + 3 * N_DDS;	// total number of PID channels equal to number of output channels
parameter NULL_SRC			= N_SRC;					// null source descriptor for deactive routes
parameter NULL_CHAN		   = N_CHAN;				// null PID channel descriptor for deactive routes
parameter W_RTR_DATA 		= W_COMP + 2;			// width of router data lines
parameter W_OPP_MAX			= W_FREQ_DATA + 1;	// maximum opp output data width

