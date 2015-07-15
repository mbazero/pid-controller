// ----------------------------- I/O Params -----------------------------------
// - Set number of DDS channels to match hardware configuration
// - Son't forget to also set DDS output pin mappings in xem6010.ucf
//   constraints file to match hardward configuration
// - Reduce number ADC or DAC channels to save FPGA memory if full number of
//   channels not needed
// ----------------------------------------------------------------------------
localparam N_ADC            = 8;                    // Number of adc channels to generate
localparam N_DAC            = 1;                    // Number of dac channels to generate
localparam N_DDS            = 0;                    // Number of dds channels to generate
localparam W_ADC_DATA       = 18;                   // Width of adc data word
localparam W_ADC_CHAN       = 3;                    // Width of adc channel select
localparam W_ADC_OS         = 3;                    // Width of adc oversample mode signal
localparam W_DAC_DATA       = 16;                   // Width of dac data word
localparam W_DAC_CHAN       = 3;                    // Width of dac channel select
localparam W_FREQ_DATA      = 48;                   // Width of dds frequency word
localparam W_PHASE_DATA     = 14;                   // Width of dds phase word
localparam W_AMP_DATA       = 10;                   // Width of dds amplitude instruction

// -------------------------- Output Map Params -------------------------------
// - Sbsolute output channel descriptors are mapped to relative descriptors
//   according to the table below
// ----------------------------------------------------------------------------
//  [ 0                         : N_DAC - 1                 ] - DAC Channels
//  [ N_DAC                 : N_DAC + N_DDS - 1     ] - DDS Frequency Channels
//  [ N_DAC + N_DDS     : N_DAC + 2*N_DDS - 1   ] - DDS Phase Channels
//  [ N_DAC + 2*N_DDS       : N_DAC + 3*N_DDS - 1   ] - DDS Amplitude Channels
// ----------------------------------------------------------------------------
localparam DAC0_ADDR        = 0;                    // DAC channel 0 output address
localparam FREQ0_ADDR       = N_DAC;                // Frequency channel 0 output address
localparam PHASE0_ADDR      = N_DAC + N_DDS;        // Phase channel 0 output address
localparam AMP0_ADDR        = N_DAC + 2 * N_DDS;    // Amplitude channel 0 output address

// ----------------------------- Misc. Params ---------------------------------
// - Don't change of these unless hardware on breakout board changes
// ----------------------------------------------------------------------------
localparam W_COMP           = 128;                  // Width of computation registers
localparam W_EP             = 16;                   // Width of opal kelly endpoint
localparam PIPE_DEPTH       = 1024;                 // Depth of pipe out fifo specified in during core gen

// -------------------------- Pid Pipeline Params -----------------------------
// - Reduce W_PID_COMP to save on FPGA area at the expensive of reduced
//   locking precision
// ----------------------------------------------------------------------------
localparam W_PID_OS         = 5;                    // Width of PID oversample signal
localparam N_PID_SRC        = N_ADC;                // Total number of PID source channels
localparam W_PID_SRC        = W_ADC_CHAN;           // Width of PID source channel select
localparam N_PID_CHAN       = N_DAC + 3*N_DDS;      // Total number of PID channels equal to number of output channels
localparam W_PID_CHAN       = 5;                    // Width of PID output channel select
localparam W_PID_DIN        = W_ADC_DATA;           // Width of PID input data
localparam W_PID_DOUT       = W_FREQ_DATA + 1;      // Width of PID data output; must greater or equal to the max output width plus a sign bit.
localparam W_PID_COMP       = 128;                  // Width of PID computation registers. A larger value means increased PID precision, but an increased FPGA area.
localparam W_PID_OPRNDS     = W_EP;                 // Width of PID operands

// ------------------------- Memory Writing Params ----------------------------
// - Don't change these unless the memory interface is changed
// ----------------------------------------------------------------------------
localparam W_WR_ADDR        = W_EP;                 // Width of write address
localparam W_WR_CHAN        = W_PID_CHAN;           // Width of write channel descriptor
localparam W_WR_DATA        = W_FREQ_DATA + 1;      // Width of write data
