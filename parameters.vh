// ----------------- i/o params ---------------------
// - reduce number ADC or DAC channels to save FPGA
//   space if full number of channels not needed
// - set number of DDS channels to reflect hardware
//   configuration
// --------------------------------------------------
parameter N_ADC			= 2;	// number of adc channels to generate
parameter N_DAC			= 2;	// number of dac channels to generate
parameter N_DDS			= 1;	// number of dds channels to generate
parameter W_ADC_DATA		= 18; // width of adc data word
parameter W_DAC_DATA		= 16;	// width of dac data word
parameter W_DDS_FREQ		= 48; // width of dds frequency word
parameter W_DDS_PHASE	= 14;	// width of dds phase word
parameter W_DDS_AMP 		= 10; // width of dds amplitude instruction

// ---------------- misc. params --------------------
// - don't change any of these unless hardware on
//   breakout board changes
// --------------------------------------------------
parameter W_COMP			= 64; // width of computation registers
parameter W_EP				= 16; // width of opal kelly endpoint
parameter W_OPP_MLT		= 10;	// width of opp multiplication factor; specifies max allowed multiplier
parameter W_DAC_CHS		= 3;	// width of dac channel input...only change this if you get a DAC with >8 channels

// -------------- simulation params -----------------
// - initial value params used to run timing
//   simulations, which do not support opal kelly
//   parameter setting
// - to run a timing sim, assert TSIM_EN and set
//   initial values as desired
// --------------------------------------------------
parameter OSF_ACTIVATE	= 0;
parameter OSF_OSM_INIT	= 0;
parameter OSF_CDLY_INIT	= 0;
parameter PID_LOCK_EN	= 0;
parameter PID_SETP_INIT = 0;
parameter PID_PCF_INIT	= 0;
parameter PID_ICF_INIT	= 0;
parameter PID_DCF_INIT	= 0;
parameter RTR_ACTV_INIT	= 0;
parameter DAC_MAX_INIT	= 0;
parameter DAC_MIN_INIT	= 0;
parameter DAC_OUT_INIT	= 0;
parameter DAC_MLT_INIT	= 1;
parameter DAC_RS_INIT	= 0;
parameter DDSF_MAX_INIT	= 0;
parameter DDSF_MIN_INIT = 0;
parameter DDSF_OUT_INIT = 0;
parameter DDSF_MLT_INIT = 1;
parameter DDSF_RS_INIT	= 0;
parameter DDSP_MAX_INIT	= 0;
parameter DDSP_MIN_INIT = 0;
parameter DDSP_OUT_INIT = 0;
parameter DDSP_MLT_INIT = 1;
parameter DDSP_RS_INIT	= 0;
parameter DDSA_MAX_INIT = 0;
parameter DDSA_MIN_INIT = 0;
parameter DDSA_OUT_INIT	= 0;
parameter DDSA_MLT_INIT = 1;
parameter DDSA_RS_INIT	= 0;
// --------------------------------------------------

