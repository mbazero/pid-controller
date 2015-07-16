//--------------------------------------------------------------------
// Opal Kelly Endpoint Map
//--------------------------------------------------------------------
// Wire-in      :   [ 0x00 - 0x1F ]
// Wire-out     :   [ 0x20 - 0x3F ]
// Trigger-in   :   [ 0x40 - 0x5F ]
// Trigger-out  :   [ 0x60 - 0x7F ]
// Pipe-In      :   [ 0x80 - 0x9F ]
// Pipe-Out     :   [ 0xA0 - 0xBf ]
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// Memory Wire-outs Endpoints
//--------------------------------------------------------------------
// Endpoint addresses for the wire-outs involved in writing to PID
// controller memory.
//--------------------------------------------------------------------
localparam addr_iwep            = 8'h01;
localparam chan_iwep            = 8'h02;

localparam data3_iwep           = 8'h03;
localparam data2_iwep           = 8'h04;
localparam data1_iwep           = 8'h05;
localparam data0_iwep           = 8'h06;

//--------------------------------------------------------------------
// Memory Data Addresses
//--------------------------------------------------------------------
// Address specifiers for memory blocks in the PID controller.
//--------------------------------------------------------------------
localparam adc_os_addr          = 16'h01;

localparam chan_src_sel_addr    = 16'h02;

localparam ovr_os_addr          = 16'h03;

localparam pid_lock_en_addr     = 16'h04;
localparam pid_inv_error_addr   = 16'h05;
localparam pid_setpoint_addr    = 16'h06;
localparam pid_p_coef_addr      = 16'h07;
localparam pid_i_coef_addr      = 16'h08;
localparam pid_d_coef_addr      = 16'h09;

localparam opt_min_addr         = 16'h0a;
localparam opt_max_addr         = 16'h0b;
localparam opt_init_addr        = 16'h0c;
localparam opt_mult_addr        = 16'h0d;
localparam opt_rs_addr          = 16'h0e;
localparam opt_add_chan_addr    = 16'h0f;

//--------------------------------------------------------------------
// Request Addresses
//--------------------------------------------------------------------
// Address specifiers for request registers in the PID controller.
// Request registers differ from normal memory in that they will
// automatically clear themselves after the request has been
// completed.
//--------------------------------------------------------------------
localparam ovr_clr_rqst         = 16'h10;
localparam pid_clr_rqst         = 16'h11;
localparam opt_clr_rqst         = 16'h12;
localparam opt_inj_rqst         = 16'h13;
localparam pipe_cset_rqst       = 16'h14;

//--------------------------------------------------------------------
// GUI Specific Memory addresses
//--------------------------------------------------------------------
// These addresses don't reference actual memory on the FPGA, but
// are used by the GUI to access GUI specific channel state.
//--------------------------------------------------------------------
localparam chan_name_addr       = 16'h15;
localparam qv_visible_addr      = 16'h16;
localparam lock_threshold_addr  = 16'h17;

//--------------------------------------------------------------------
// Trigger Endpoints
//--------------------------------------------------------------------
// Endpoint addresses for PID controller trigger arrays. The sys_gp
// trigger array is the only trigger implemented at the moment. It
// supplies general purpose controller triggers specified by the
// offsets in the next section.
//--------------------------------------------------------------------
localparam sys_gp_itep          = 8'h40;

//--------------------------------------------------------------------
// System Trigger Offsets
//--------------------------------------------------------------------
// Offset specifiers for system triggers within the sys_gp trigger
// array.
//--------------------------------------------------------------------
localparam sys_rst_offset       = 0;
localparam adc_cstart_offset    = 1;
localparam wr_en_offset         = 2;
localparam dac_rset_offset      = 3;

//--------------------------------------------------------------------
// Data Logging Endpoints
//--------------------------------------------------------------------
// Endpoint addresses for wire-outs and pipes involved in data
// logging.
//--------------------------------------------------------------------
localparam data_log_status_owep = 8'h20;
localparam data_log0_owep       = 8'h21;
localparam data_log_opep        = 8'ha0;

