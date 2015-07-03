task configure_chans;
	begin : cchan
		integer c;
		for (c = 0; c < NAC; c = c+1) begin
			// Activate channel
			write_data(chan_activate_addr, dest[c], 1);

			// Route source to destination
			write_data(chan_input_sel_addr, dest[c], src[c]);

			// Set OSF ratio and activate source channel
			write_data(osf_cycle_delay_addr, dest[c], 0);
			write_data(osf_os_addr, dest[c], 0);

			// Set PID params
			write_data(pid_setpoint_addr, dest[c], setpoint[c]);
			write_data(pid_p_coef_addr, dest[c], p_coef[c]);
			write_data(pid_i_coef_addr, dest[c], i_coef[c]);
			write_data(pid_d_coef_addr, dest[c], d_coef[c]);
			write_data(pid_lock_en_addr, dest[c], lock_en[c]);

			// Set OPP params
			write_data(opp_min_addr, dest[c], output_min[c]);
			write_data(opp_max_addr, dest[c], output_max[c]);
			write_data(opp_init_addr, dest[c], output_init[c]);
			write_data(opp_mult_addr, dest[c], multiplier[c]);
			write_data(opp_rs_addr, dest[c], right_shift[c]);

			// Set focus
			if (focus[c] == 1) begin
				write_data(chan_focus_addr, dest[c], 1);
			end

		end
	end
endtask

task write_data;
	input [15:0] addr;
	input [15:0] chan;
	input [47:0] data;
	begin
		SetWireInValue(addr_iwep, addr, MASK);
		SetWireInValue(chan_iwep, chan, MASK);
		SetWireInValue(data2_iwep, data[47:32], MASK);
		SetWireInValue(data1_iwep, data[31:16], MASK);
		SetWireInValue(data0_iwep, data[15:0], MASK);
		UpdateWireIns;
		ActivateTriggerIn(sys_gp_itep, write_data_offset);
	end
endtask

