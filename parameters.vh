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

//////////////////////////////////////////
// wire-in endpoints
//////////////////////////////////////////
parameter addr_iwep					= 8'h01;
parameter chan_iwep					= 8'h02;

parameter data2_iwep					= 8'h03;
parameter data1_iwep					= 8'h04;
parameter data0_iwep					= 8'h05;

//////////////////////////////////////////
// wire-in data addresses
//////////////////////////////////////////
parameter adc_os_addr				= 16'h01;

parameter chan_active_addr  		= 16'h02;
parameter chan_src_sel_addr		= 16'h03;

parameter osf_activate_addr		= 16'h05;
parameter osf_os_addr				= 16'h07;

parameter pid_setpoint_addr		= 16'h08;
parameter pid_p_coef_addr			= 16'h09;
parameter pid_i_coef_addr			= 16'h0a;
parameter pid_d_coef_addr			= 16'h0b;
parameter pid_lock_en_addr			= 16'h0c;

parameter opp_min_addr				= 16'h0d;
parameter opp_max_addr				= 16'h0e;
parameter opp_init_addr				= 16'h0f;
parameter opp_mult_addr				= 16'h10;
parameter opp_rs_addr				= 16'h11;

parameter pipe_chan_addr         = 16'h04;

//////////////////////////////////////////
// trigger-in endpoints
//////////////////////////////////////////
parameter sys_gp_itep				= 8'h40;

parameter opp_inject1_itep			= 8'h41;
parameter opp_inject0_itep			= 8'h42;

//////////////////////////////////////////
// multipurpose trigger offsets
//////////////////////////////////////////
parameter sys_rst_offset			= 0;
parameter adc_cstart_offset		= 1;
parameter write_data_offset		= 2;
parameter dac_ref_set_offset		= 3;

//////////////////////////////////////////
// wire/pipe-out endpoints
//////////////////////////////////////////
parameter osf_data0_owep			= 8'h20;
parameter osf_data_opep				= 8'ha3;

