/* Transmit random data words to the ADC controller */
task adc_transmit;
	input [31:0] reps;

	begin : adc_tx
		integer adc_count = 0, j = 0;

		repeat(reps) begin
			// wait for convst_out to pulse and then assert busy
			@(posedge adc_convst_out) begin
				@(posedge clk17_in) adc_busy_in = 1;
			end

			for ( j = 0; j < N_ADC; j = j+1 ) begin
				adc_val[j] = $random % 1000;
			end

			// simulate serial transmission from adc to fpga
			@(negedge adc_n_cs_out) begin
				data_a_tx = {adc_val[0], adc_val[1], adc_val[2], adc_val[3]};
				data_b_tx = {adc_val[4], adc_val[5], adc_val[6], adc_val[7]};
			end

			// wait one cycle before transmitting
			@(posedge clk17_in);

			$display("======================================");
			$display("Iteration #%d", adc_count);
			$display("======================================");

			// simulate serial data transmission
			repeat (71) begin
				@(negedge clk17_in)
				data_a_tx = data_a_tx << 1;
				data_b_tx = data_b_tx << 1;
			end

			// simulate conversion end
			#2000;
			@(posedge clk17_in) adc_busy_in = 0;

			adc_count = adc_count + 1;

		end
	end
endtask
