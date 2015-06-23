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
