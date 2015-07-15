`timescale 1ns / 1ps

// adc_controller -- mba 2014
// -----------------------------------------------------------
// runs AD7608 in continuous conversion mode and reads data
// concurrently during conversion
// -----------------------------------------------------------

module adc_controller #(
    parameter W_OUT = 18,               // width of adc data channels
    parameter N_CHAN = 8,               // number of channels to output
    parameter W_CHAN = 3,               // width of channel select
    parameter W_OS = 2                  // width of oversample signal
    )(
    // inputs <- top level entity
    input wire clk_in,                  // adc serial clock; max frequency 17mhz
    input wire reset_in,                // system reset

    // inputs <- AD7608
    input wire busy_in,                 // conversion busy signal
    input wire data_a_in,               // serial data channel a
    input wire data_b_in,               // serial data channel b

    // inputs <- frontpanel controller
    input wire [W_OS-1:0] os_in,        // sets adc oversampling mode
    input wire cstart_in,               // pulse starts continuous adc conversion cycle

    // outputs -> AD7608
    output wire [W_OS-1:0] os_out,      // oversampling signal to adc
    output wire convst_out,             // convert start signal to adc
    output wire reset_out,              // reset signal to adc
    output wire sclk_out,               // serial clock signal to adc
    output wire n_cs_out,               // chip select signal to adc

    // outputs -> pid core
    output reg dv_out,
    output reg [W_CHAN-1:0] chan_out,
    output reg [W_OUT-1:0] data_out
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam MIN_T_CYCLE  = 85;                // minimum cycle time in number of adc clock cycles (don't change this)
localparam RD_LENGTH = W_OUT*(N_CHAN/2);     // bits of data to be read per serial port in a single cycle

/* state parameters */
localparam  CV_ST_IDLE = 3'd0,               // wait for module enable signal to begin continuous conversion
            CV_ST_CONVST = 3'd1,             // pulse convert start signal to begin conversion
            CV_ST_CONV = 3'd2;               // wait for ADC to finish conversion

localparam  RD_ST_IDLE = 3'd0,               // wait for busy signal to begin read
            RD_ST_READ = 3'd1,               // read adc data off serial lines
            RD_ST_WAIT = 3'd2;               // wait for busy signal to deassert

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* adc cstart register */
reg cstart_reg;

/* transmission structures */
wire [N_CHAN/2-1:0] dv_tx;
reg [W_CHAN-1:0] chan_a_tx, chan_b_tx;
reg [W_OUT-1:0] data_a_tx, data_b_tx;

/* channel b delay registers */
reg dv_reg;
reg [W_CHAN-1:0] chan_b_reg;
reg [W_OUT-1:0] data_b_reg;

/* state registers */
reg [7:0] cv_counter = 0;                   // convert state machine counter
reg [2:0] cv_cur_state = CV_ST_IDLE;        // convert state machine current state
reg [2:0] cv_next_state = CV_ST_IDLE;       // convert state machine next state

reg [7:0] rd_counter = 0;                   // read state machine counter
reg [2:0] rd_cur_state = RD_ST_IDLE;        // read state machine current state
reg [2:0] rd_next_state = RD_ST_IDLE;       // read state machine next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* adc control */
assign reset_out = reset_in;
assign os_out = os_in;

/* transmission data valid */
genvar i;
generate
    for ( i = 0; i < N_CHAN/2; i = i+1 ) begin : data_out_arr
        assign dv_tx[i] = (rd_cur_state == RD_ST_READ && rd_counter == W_OUT*(i+1));
    end
endgenerate

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* adc cstart register */
always @( posedge cstart_in ) begin
    cstart_reg = 1'b1;
end

/* serial read shift register */
always @( posedge clk_in ) begin
    if ( reset_in ) begin
        data_a_tx <= 0;
        data_b_tx <= 0;
    end else if ( !n_cs_out ) begin
        data_a_tx <= {data_a_tx[W_OUT-2:0], data_a_in};
        data_b_tx <= {data_b_tx[W_OUT-2:0], data_b_in};
    end
end

/* transmission channels */
always @( posedge clk_in ) begin
	if ( rd_cur_state != RD_ST_READ ) begin
		chan_a_tx <= 0;
		chan_b_tx <= N_CHAN/2;
	end else if ( |dv_tx ) begin
		chan_a_tx <= chan_a_tx + 1'b1;
		chan_b_tx <= chan_b_tx + 1'b1;
	end
end

/* channel b delay registers */
always @( posedge clk_in ) begin
    dv_reg <= |dv_tx;
    chan_b_reg <= chan_b_tx;
    data_b_reg <= data_b_tx;
end

/* output registers */
always @( posedge clk_in ) begin
    dv_out <= ( |dv_tx || dv_reg );
    chan_out <= ( |dv_tx ) ? chan_a_tx : chan_b_reg;
    data_out <= ( |dv_tx ) ? data_a_tx : data_b_reg;
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* adc sclk forwarding buffer
   helps prevent clock skew issues */
ODDR2 #(
    .DDR_ALIGNMENT  ("NONE"),
    .INIT               (1'b1),
    .SRTYPE         ("SYNC")
) adc_clk_fwd (
    .Q                  (sclk_out),
    .C0             (clk_in),
    .C1             (~clk_in),
    .CE             (1'b1),
    .D0             (1'b1), // VCC
    .D1             (1'b0), // GND
    .R                  (1'b0),
    .S                  (n_cs_out)
);

/*
* Data reading and conversion must happen concurrently in order to acheive
* max throughput. Toward this end, the ADC controller has two independent
* state machines, one to handle data conversion and one to handle serial
* data reading.
*/
//////////////////////////////////////////
// convert state machine
//////////////////////////////////////////

/* initial assignments */
initial begin
    cv_counter      = 0;
    cv_cur_state    = CV_ST_IDLE;
    cv_next_state   = CV_ST_IDLE;
end

/* state sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        cv_cur_state <= CV_ST_IDLE;
    end else begin
        cv_cur_state <= cv_next_state;
    end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        cv_counter <= 0;
    end else if ( cv_cur_state != cv_next_state ) begin
        cv_counter <= 0;
    end else begin
        cv_counter <= cv_counter + 1'b1;
    end
end

/* next state combinational logic */
always @( * ) begin
    cv_next_state <= cv_cur_state; // default assignment
    case ( cv_cur_state )
        CV_ST_IDLE: begin
            if ( cstart_reg == 1 )
                cv_next_state <= CV_ST_CONVST;
        end
        CV_ST_CONVST: begin
                cv_next_state <= CV_ST_CONV;
        end
        CV_ST_CONV: begin
            if (( cv_counter >= MIN_T_CYCLE ) & ( busy_in == 0 ))
                cv_next_state <= CV_ST_CONVST;
        end
    endcase
end

/* fsm outputs */
assign convst_out = ~( cv_cur_state == CV_ST_CONVST );

//////////////////////////////////////////
// read state machine
//////////////////////////////////////////

/* state sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        rd_cur_state <= RD_ST_IDLE;
    end else begin
        rd_cur_state <= rd_next_state;
    end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        rd_counter <= 0;
    end else if ( rd_cur_state != rd_next_state ) begin
        rd_counter <= 0;
    end else begin
        rd_counter <= rd_counter + 1'b1;
    end
end

/* next state combinational logic */
always @( * ) begin
    rd_next_state <= rd_cur_state; // default assignment
    case ( rd_cur_state )
        RD_ST_IDLE: begin
            if ( busy_in == 1 )                     rd_next_state <= RD_ST_READ;
        end
        RD_ST_READ: begin
            if ( rd_counter == RD_LENGTH )  rd_next_state <= RD_ST_WAIT;
        end
        RD_ST_WAIT: begin
            if ( busy_in == 0 )                 rd_next_state <= RD_ST_IDLE;
        end
    endcase
end

/* fsm outputs */
assign n_cs_out = ~(( rd_cur_state == RD_ST_READ ) & ( rd_counter < RD_LENGTH ));

endmodule
