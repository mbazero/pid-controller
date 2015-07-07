//////////////////////////////////////////
// wire-in endpoints
//////////////////////////////////////////
parameter addr_iwep             = 8'h01;
parameter chan_iwep             = 8'h02;

parameter data3_iwep            = 8'h03;
parameter data2_iwep            = 8'h04;
parameter data1_iwep            = 8'h05;
parameter data0_iwep            = 8'h06;

//////////////////////////////////////////
// config data addresses
//////////////////////////////////////////
parameter adc_os_addr           = 16'h01;

parameter chan_en_addr          = 16'h02;
parameter chan_src_sel_addr     = 16'h03;

parameter ovr_os_addr           = 16'h05;

parameter pid_clr_addr          = 16'h06;
parameter pid_setpoint_addr     = 16'h07;
parameter pid_p_coef_addr       = 16'h08;
parameter pid_i_coef_addr       = 16'h09;
parameter pid_d_coef_addr       = 16'h0a;

parameter opt_clr_addr          = 16'h0b;
parameter opt_min_addr          = 16'h0c;
parameter opt_max_addr          = 16'h0d;
parameter opt_init_addr         = 16'h0e;
parameter opt_mult_addr         = 16'h0f;
parameter opt_rs_addr           = 16'h10;

parameter pipe_chan_addr        = 16'h11;

//////////////////////////////////////////
// request register addresses
// - request registers are unique in that
//   they will clear themselves after the
//   request has been executed
//////////////////////////////////////////
parameter ovr_clr_req_addr      = 16'h13;
parameter pid_clr_req_addr      = 16'h13;
parameter opt_clr_req_addr      = 16'h13;
parameter opt_inj_req_addr      = 16'h12;

//////////////////////////////////////////
// trigger-in endpoints
//////////////////////////////////////////
parameter sys_gp_itep           = 8'h40;

parameter opt_inject1_itep      = 8'h41;
parameter opt_inject0_itep      = 8'h42;

//////////////////////////////////////////
// multipurpose trigger offsets
//////////////////////////////////////////
parameter sys_rst_offset        = 0;
parameter adc_cstart_offset     = 1;
parameter wr_en_offset          = 2;
parameter dac_ref_set_offset    = 3;

//////////////////////////////////////////
// wire/pipe-out endpoints
//////////////////////////////////////////
parameter log_data0_owep        = 8'h20;
parameter log_data_opep         = 8'ha3;

