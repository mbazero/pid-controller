/* Transmit random data words to the ADC controller */
task adc_transmit;
	input [31:0] reps;

	begin
		// adc data transmission simulation
		repeat(reps) begin
			// wait for convst_out to pulse and then assert busy
			@(posedge adc_convst_out) begin
				@(posedge clk17_in) adc_busy_in = 1;
			end

			chan_reg = $random;

			#1 compute_exp_pid();

			#1 compute_exp_output();

			// simulate serial transmission from adc to fpga
			@(negedge adc_n_cs_out) begin
				data_a_tx = {chan[0], chan[1], chan[2], chan[3]};
				data_b_tx = {chan[4], chan[5], chan[6], chan[7]};
			end

			// wait one cycle before transmitting
			@(posedge clk17_in);

			// simulate serial data transmission
			repeat (71) begin
				@(negedge clk17_in)
				data_a_tx = data_a_tx << 1;
				data_b_tx = data_b_tx << 1;
			end

			// simulate conversion end
			#2000;
			@(posedge clk17_in) adc_busy_in = 0;

		end
	end
endtask

/* Compute expected PID value */
task compute_exp_pid;
	begin
		e_count = e_count + 1;
		error = setpoint - chan[src];
		#1 integral = integral + error;
		derivative = error - error_prev;
		#1 pid_expected = (p_coef * error) + (i_coef * integral) + (d_coef * derivative);
		#1 pid_expected = (lock_en) ? pid_expected : 0;
		error_prev = error;
	end
endtask

/* Compute expected output */
task compute_exp_output;
	begin
		proc_stage[0] = pid_expected * multiplier;
		#1 proc_stage[1] = proc_stage[0] >>> right_shift;
		#1 proc_stage[2] = proc_stage[1] + dac_data_reg;
		#1 proc_stage[3] = (lock_en == 1) ? proc_stage[2] : output_init;
		#1 proc_stage[4] = (proc_stage[3] > output_max) ? output_max : proc_stage[3];
		#1 proc_stage[5] = (proc_stage[4] < output_min) ? output_min : proc_stage[4];
	end
endtask

task check_rcv;
	input [31:0] reps;

	repeat(reps) begin
		// simulate dac receiving data
		@(negedge dac_nsync_out) begin
			repeat(32) begin
				@(negedge dac_sclk_out) begin
					r_instr = {r_instr[30:0], dac_din_out}; // shift data in
				end
			end
		end
		#1 dac_data_reg = r_data;
		//#1 print_pipes();
		#1 assert_equals(proc_stage[5], r_data, "Receive");
	end
endtask

task assert_equals;
	input [127:0] expected;
	input [127:0] received;
	input [20*8-1:0] test_name;

	begin

		$display("%s Test:", test_name);
		$display("Expected: %d", $signed(expected));
		$display("Received: %d", $signed(received));

		if(expected == received) begin
			$display("Success");
		end else begin
			$display("Failure");
			$stop;
		end
	end
endtask
