// ----------------- i/o params ---------------------
// - reduce number ADC or DAC channels to save FPGA
//   space if full number of channels not needed
// - set number of DDS channels to reflect hardware
//   configuration
// --------------------------------------------------
parameter N_ADC			= 8;	// number of adc channels to generate
parameter N_DAC			= 8;	// number of dac channels to generate
parameter N_DDS			= 0;	// number of dds channels to generate
parameter W_ADC_DATA		= 18; // width of adc data word
parameter W_DAC_DATA		= 16;	// width of dac data word
parameter W_FREQ_DATA	= 48; // width of dds frequency word
parameter W_PHASE_DATA	= 14;	// width of dds phase word
parameter W_AMP_DATA 	= 10; // width of dds amplitude instruction

// ------------- comp latency params ----------------
// - increase these if clock speed is increased and
//   more time is needed to complete pid or opp
//   computation.
// --------------------------------------------------
parameter PID_COMP_LATENCY	= 1; // pid core computation latency
parameter OPP_COMP_LATENCY	= 1; // output preprocessor compuation latency

// ---------------- derived params ------------------
// - don't change these
// --------------------------------------------------
parameter N_SRC				= N_ADC;					// total number of source channels
parameter N_CHAN 				= N_DAC + 3*N_DDS;	// total number of output channels; each dds has three channels (phase, freq, and amp)
parameter W_RTR_DATA 		= W_COMP + 2;			// width of router data lines
parameter W_OPP_MAX			= W_FREQ_DATA + 1;	// maximum opp output data width

// ---------------- misc. params --------------------
// - don't change any of these unless hardware on
//   breakout board changes
// --------------------------------------------------
parameter W_COMP			= 64; // width of computation registers
parameter W_EP				= 16; // width of opal kelly endpoint
parameter W_ADC_OS		= 3;	// width of adc oversample signal
parameter W_OSF_OS		= 6;	// width of oversample mode signal
parameter W_OSF_CD		= 16;	// width of osf cycle delay signal
parameter W_INPUT_SEL	= 5;	// width of router select signal (must be log2(N_DAC) + 1...MSB stores channel activation state)
parameter W_OPP_MLT		= 10;	// width of opp multiplication factor; specifies max allowed multiplier
parameter W_DAC_CHS		= 3;	// width of dac channel input...only change this if you get a DAC with >8 channels
parameter NULL_SRC		= -1;	// null source for deactive routes
