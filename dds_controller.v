`timescale 1ns / 1ps

// dds_controller -- mba 2015
// -----------------------------------------------------------
// Sends update instructions to AD9912 DDS chip.
// -----------------------------------------------------------

module dds_controller(
    // inputs <- top level entity
    input wire clk_in,              // system clock
    input wire reset_in,            // system reset

    // inputs <- output preprocessor
    input wire [47:0] freq_in,      // frequency data
    input wire [13:0] phase_in,     // phase data
    input wire [9:0] amp_in,        // amplitude data
    input wire freq_dv_in,          // frequency data valid signal
    input wire phase_dv_in,         // phase data valid signal
    input wire amp_dv_in,           // amplitude data valid signal

    // outputs -> dds hardware
    output wire sclk_out,           // serial clock signal to dds
    output wire reset_out,          // reset signal to dds
    output wire csb_out,            // chip select signal to dds
    output wire sdio_out,           // serial data line to dds
    output wire io_update_out,      // io update signal to dds

    // outputs -> top level entity
    output wire freq_wr_done,
    output wire phase_wr_done,
    output wire amp_wr_done
   );

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

/* transmission types */
localparam FREQ_TX = 2'd1;
localparam PHASE_TX = 2'd2;
localparam AMP_TX = 2'd3;

/* state parameters */
localparam  ST_IDLE         = 3'd0,     // wait for new data
            ST_TX           = 3'd1,     // transmit update instruction
            ST_WAIT         = 3'd2,     // wait before pulsing io_update to satisfy setup time
            ST_IO_UPDATE    = 3'd3;     // pulse io_update signal to initiate dds update

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* input data registers */
reg [47:0] freq = 0;                // active frequency value
reg [13:0] phase = 0;               // active phase value
reg [9:0] amp = 0;                  // active amplitude value

reg freq_dv = 0;                    // frequency data valid
reg phase_dv = 0;                   // phase data valid
reg amp_dv = 0;                     // amplitude data valid

/* write instructions */
wire [63:0] freq_wr_instr;          // frequency write instruction
wire [31:0] phase_wr_instr;         // phase write instruction
wire [31:0] amp_wr_instr;           // amplitude write instruction

/* transmission registers */
reg [63:0] tx_data = 0;             // active data to be sent to dds
reg [6:0] tx_len = 0;               // length of current write instruction
reg [1:0] tx_type = 0;
reg csb_reg = 1;                    // chip select

/* state registers */
reg [31:0] counter = 0;             // transmit counter
reg [2:0] cur_state = ST_IDLE;      // current state
reg [2:0] next_state = ST_IDLE;     // next state

//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* dds control signals */
assign reset_out = reset_in;
assign csb_out = ~( cur_state == ST_TX );
assign sdio_out = tx_data[63];
assign io_update_out = ( cur_state == ST_IO_UPDATE );

/* loop flow control */
assign freq_wr_done = ( cur_state == ST_IO_UPDATE && tx_type == FREQ_TX );
assign phase_wr_done = ( cur_state == ST_IO_UPDATE && tx_type == PHASE_TX );
assign amp_wr_done = ( cur_state == ST_IO_UPDATE && tx_type == AMP_TX );

/* frequency, phase, and amplitude instruction words */
assign freq_wr_instr = {1'b0, 2'b11, 13'h01AB, freq};
assign phase_wr_instr = {1'b0, 2'b01, 13'h01AD, {2'd0, phase}};
assign amp_wr_instr = {1'b0, 2'b01, 13'h040C, {6'd0, amp}};

//////////////////////////////////////////
// sequential logic
//////////////////////////////////////////

/* freq, phase, and amp data/dv registers */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        freq_dv <= 0;
        phase_dv    <= 0;
        amp_dv  <= 0;
    end else begin
        case ( cur_state )
            ST_IDLE: begin
                if ( freq_dv_in == 1 ) begin
                    freq <= freq_in;
                    freq_dv <= freq_dv_in;
                end
                if ( phase_dv_in == 1 ) begin
                    phase <= phase_in;
                    phase_dv <= phase_dv_in;
                end
                if ( amp_dv_in == 1 ) begin
                    amp <= amp_in;
                    amp_dv <= amp_dv_in;
                end
            end
            ST_TX: begin
                if ( counter == 0 ) begin
                    if ( freq_dv == 1 ) begin
                        freq_dv <= 0;
                    end else if ( phase_dv == 1 ) begin
                        phase_dv <= 0;
                    end else if ( amp_dv == 1 ) begin
                        amp_dv <= 0;
                    end
                end
            end
        endcase
    end
end

/* serial data transmission */
always @( negedge clk_in ) begin
    case ( cur_state )
        ST_IDLE: begin
            tx_data <= 0;
            tx_len  <= 0;
            tx_type <= 0;
        end
        ST_TX: begin
            if ( counter == 0 ) begin
                if ( freq_dv == 1 ) begin
                    tx_data <= freq_wr_instr;
                    tx_len  <= 64;
                    tx_type <= FREQ_TX;
                end else if ( phase_dv == 1 ) begin
                    tx_data <= {phase_wr_instr, 32'b0};
                    tx_len  <= 32;
                    tx_type <= PHASE_TX;
                end else if ( amp_dv == 1 ) begin
                    tx_data <= {amp_wr_instr, 32'b0};
                    tx_len  <= 32;
                    tx_type <= AMP_TX;
                end
            end else begin
                tx_data <= tx_data << 1;
            end
        end
    endcase
end

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* dds sclk forwarding buffer
   helps prevent clock skew issues */
ODDR2 #(
    .DDR_ALIGNMENT  ("NONE"),
    .INIT           (1'b1),
    .SRTYPE         ("SYNC")
) dds_clk_fwd (
    .Q              (sclk_out),
    .C0             (clk_in),
    .C1             (~clk_in),
    .CE             (1'b1),
    .D0             (1'b1), // VCC
    .D1             (1'b0), // GND
    .R              (1'b0),
    .S              (1'b0)
);

//////////////////////////////////////////
// state machine
//////////////////////////////////////////

/* state sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        cur_state <= ST_IDLE;
    end else begin
        cur_state <= next_state;
    end
end

/* state counter sequential logic */
always @( posedge clk_in ) begin
    if ( reset_in == 1 ) begin
        counter <= 0;
    end else if ( cur_state != next_state ) begin
        counter <= 0;
    end else begin
        counter <= counter + 1'b1;
    end
end

/* next state combinational logic */
always @( * ) begin
    next_state <= cur_state; // default assignment
    case (cur_state)
        ST_IDLE: begin
            if ( freq_dv | phase_dv | amp_dv ) begin
                next_state <= ST_TX;
            end
        end
        ST_TX: begin
            if ( counter == tx_len-1 ) begin
                next_state <= ST_WAIT;
            end
        end
        ST_WAIT: begin
            if ( counter == 1 ) begin
                next_state <= ST_IO_UPDATE;
            end
        end
        ST_IO_UPDATE: begin
            next_state <= ST_IDLE;
        end
    endcase
end

endmodule
