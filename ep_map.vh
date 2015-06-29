//////////////////////////////////////////
// wire-in endpoints
//////////////////////////////////////////
parameter data2_iwep					= 8'h01;
parameter data1_iwep					= 8'h02;
parameter data0_iwep					= 8'h03;

parameter addr_iwep					= 8'h04;
parameter chan_iwep					= 8'h05;

//////////////////////////////////////////
// wire-in data addresses
//////////////////////////////////////////
parameter adc_os_addr				= 16'h01;

parameter osf_activate_addr		= 16'h02;
parameter osf_cycle_delay_addr	= 16'h03;
parameter osf_osm_addr				= 16'h04;
parameter osf_pipe_chan_addr		= 16'h29;

parameter pid_lock_en_addr			= 16'h05;
parameter pid_setpoint_addr		= 16'h06;
parameter pid_p_coef_addr			= 16'h07;
parameter pid_i_coef_addr			= 16'h08;
parameter pid_d_coef_addr			= 16'h09;
parameter pid_update_en_addr		= 16'h0a;

parameter rtr_src_sel_addr			= 16'h2a;
parameter rtr_dac_src_addr			= 16'h0b;
parameter rtr_freq_src_addr		= 16'h0c;
parameter rtr_phase_src_addr		= 16'h0d;
parameter rtr_amp_src_addr			= 16'h0e;

parameter opp_dac_min_addr			= 16'h0f;
parameter opp_dac_max_addr			= 16'h10;
parameter opp_dac_init_addr		= 16'h11;
parameter opp_dac_mult_addr		= 16'h12;
parameter opp_dac_rs_addr			= 16'h13;

parameter opp_freq_min_addr		= 16'h14;
parameter opp_freq_max_addr		= 16'h15;
parameter opp_freq_init_addr		= 16'h16;
parameter opp_freq_mult_addr		= 16'h17;
parameter opp_freq_rs_addr			= 16'h18;

parameter opp_phase_min_addr		= 16'h19;
parameter opp_phase_max_addr		= 16'h20;
parameter opp_phase_init_addr		= 16'h21;
parameter opp_phase_mult_addr		= 16'h22;
parameter opp_phase_rs_addr		= 16'h23;

parameter opp_amp_min_addr			= 16'h24;
parameter opp_amp_max_addr			= 16'h25;
parameter opp_amp_init_addr		= 16'h26;
parameter opp_amp_mult_addr		= 16'h27;
parameter opp_amp_rs_addr			= 16'h28;

//////////////////////////////////////////
// trigger-in endpoints
//////////////////////////////////////////
parameter sys_gp_itep				= 8'h40;

parameter opp_dac_inj_itep			= 8'h42;
parameter opp_freq_inj_itep		= 8'h43;
parameter opp_phase_inj_itep		= 8'h44;
parameter opp_amp_inj_itep			= 8'h45;

//////////////////////////////////////////
// multipurpose trigger offsets
//////////////////////////////////////////
parameter sys_reset_offset			= 8'h00;
parameter adc_cstart_offset		= 8'h01;
parameter reg_update_offset		= 8'h02;
parameter dac_ref_set_offset		= 8'h03;

//////////////////////////////////////////
// wire/pipe-out endpoints
//////////////////////////////////////////
parameter osf_data_owep				= 8'h20;
parameter osf_data_opep				= 8'ha3;
