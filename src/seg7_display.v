// ============================================================================
// Module      : seg7_display
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3
// Description : 4-digit time-multiplexed 7-segment display driver.
//               Normal mode:  [State Character] [Balance 3 digits]
//               Billing mode: [Area Number 0-9] [Bill 3 digits]
//
//               BCD conversion is REGISTERED using a subtraction-based
//               approach to avoid deep combinational divide paths.
//
//               Basys 3 Hardware Notes:
//                 - Anodes   (an[3:0])  : ACTIVE LOW → 0 = digit ON
//                 - Cathodes (seg[6:0]) : ACTIVE LOW → 0 = segment ON
//                 - Digit mapping: an[0]=rightmost, an[3]=leftmost
//                 - Segment mapping: seg = {CA, CB, CC, CD, CE, CF, CG}
//
// ============================================================================
`timescale 1ns / 1ps

module seg7_display (
    input  wire        clk,
    input  wire        rst,
    input  wire        clk_display,  // ~381 Hz mux clock from clk_divider
    input  wire [9:0]  balance,      // 10-bit value to display (0–999)
    input  wire [2:0]  fsm_state,    // FSM state for leftmost character
    input  wire [3:0]  bill_area,    // Area ID for billing display (0–9)

    output reg  [3:0]  an,           // Anode control (active LOW)
    output reg  [6:0]  seg           // Cathode segments (active LOW)
);

    // =========================================================================
    // REGISTERED BCD Decomposition
    // Uses a simple 2-stage pipeline with subtraction (no division).
    //   Stage 1: Compute hundreds digit and remainder
    //   Stage 2: Compute tens and ones digits from remainder
    // =========================================================================
    reg [3:0] digit_h;  // Hundreds
    reg [3:0] digit_t;  // Tens
    reg [3:0] digit_o;  // Ones

    // Stage 1 intermediates
    reg [3:0] hund_s1;
    reg [6:0] rem_s1;   // Remainder after hundreds (0–99), 7 bits enough

    // Stage 1: Hundreds extraction (registered)
    always @(posedge clk) begin
        if (rst) begin
            hund_s1 <= 4'd0;
            rem_s1  <= 7'd0;
        end else begin
            if      (balance >= 10'd900) begin hund_s1 <= 4'd9; rem_s1 <= balance - 10'd900; end
            else if (balance >= 10'd800) begin hund_s1 <= 4'd8; rem_s1 <= balance - 10'd800; end
            else if (balance >= 10'd700) begin hund_s1 <= 4'd7; rem_s1 <= balance - 10'd700; end
            else if (balance >= 10'd600) begin hund_s1 <= 4'd6; rem_s1 <= balance - 10'd600; end
            else if (balance >= 10'd500) begin hund_s1 <= 4'd5; rem_s1 <= balance - 10'd500; end
            else if (balance >= 10'd400) begin hund_s1 <= 4'd4; rem_s1 <= balance - 10'd400; end
            else if (balance >= 10'd300) begin hund_s1 <= 4'd3; rem_s1 <= balance - 10'd300; end
            else if (balance >= 10'd200) begin hund_s1 <= 4'd2; rem_s1 <= balance - 10'd200; end
            else if (balance >= 10'd100) begin hund_s1 <= 4'd1; rem_s1 <= balance - 10'd100; end
            else                         begin hund_s1 <= 4'd0; rem_s1 <= balance[6:0];       end
        end
    end

    // Stage 2: Tens and ones extraction from remainder (registered)
    always @(posedge clk) begin
        if (rst) begin
            digit_h <= 4'd0;
            digit_t <= 4'd0;
            digit_o <= 4'd0;
        end else begin
            digit_h <= hund_s1;

            if      (rem_s1 >= 7'd90) begin digit_t <= 4'd9; digit_o <= rem_s1 - 7'd90; end
            else if (rem_s1 >= 7'd80) begin digit_t <= 4'd8; digit_o <= rem_s1 - 7'd80; end
            else if (rem_s1 >= 7'd70) begin digit_t <= 4'd7; digit_o <= rem_s1 - 7'd70; end
            else if (rem_s1 >= 7'd60) begin digit_t <= 4'd6; digit_o <= rem_s1 - 7'd60; end
            else if (rem_s1 >= 7'd50) begin digit_t <= 4'd5; digit_o <= rem_s1 - 7'd50; end
            else if (rem_s1 >= 7'd40) begin digit_t <= 4'd4; digit_o <= rem_s1 - 7'd40; end
            else if (rem_s1 >= 7'd30) begin digit_t <= 4'd3; digit_o <= rem_s1 - 7'd30; end
            else if (rem_s1 >= 7'd20) begin digit_t <= 4'd2; digit_o <= rem_s1 - 7'd20; end
            else if (rem_s1 >= 7'd10) begin digit_t <= 4'd1; digit_o <= rem_s1 - 7'd10; end
            else                      begin digit_t <= 4'd0; digit_o <= rem_s1[3:0];     end
        end
    end

    // =========================================================================
    // 7-Segment BCD Decoder (active-low cathodes)
    // =========================================================================
    function [6:0] bcd_to_seg;
        input [3:0] bcd;
        begin
            case (bcd)
                4'd0: bcd_to_seg = 7'b0000001;
                4'd1: bcd_to_seg = 7'b1001111;
                4'd2: bcd_to_seg = 7'b0010010;
                4'd3: bcd_to_seg = 7'b0000110;
                4'd4: bcd_to_seg = 7'b1001100;
                4'd5: bcd_to_seg = 7'b0100100;
                4'd6: bcd_to_seg = 7'b0100000;
                4'd7: bcd_to_seg = 7'b0001111;
                4'd8: bcd_to_seg = 7'b0000000;
                4'd9: bcd_to_seg = 7'b0000100;
                default: bcd_to_seg = 7'b1111111;
            endcase
        end
    endfunction

    // =========================================================================
    // FSM State Character Encoder (leftmost digit, active-low)
    // I=IDLE, n=MONITOR, C=CONSUME, S=SHED, r=RESET, b=BILLING
    // =========================================================================
    reg [6:0] state_seg;
    always @(*) begin
        case (fsm_state)
            3'b000: state_seg = 7'b1111001; // 'I'
            3'b001: state_seg = 7'b0101011; // 'n'
            3'b010: state_seg = 7'b0110001; // 'C'
            3'b011: state_seg = 7'b0100100; // 'S'
            3'b100: state_seg = 7'b0101111; // 'r'
            3'b101: state_seg = 7'b1100000; // 'b' (billing)
            default:state_seg = 7'b1111111;
        endcase
    end

    // =========================================================================
    // Billing mode detect: fsm_state == 3'b101 (BILLING)
    // In billing mode, leftmost digit shows area number (0-9)
    // =========================================================================
    wire billing_mode = (fsm_state == 3'b101);

    // =========================================================================
    // Multiplexer: rotate through 4 digits using clk_display
    // =========================================================================
    reg [1:0] digit_sel;

    always @(posedge clk) begin
        if (rst)
            digit_sel <= 2'b00;
        else if (clk_display)
            digit_sel <= digit_sel + 1'b1;
    end

    always @(*) begin
        case (digit_sel)
            2'b00: begin   // Digit 0: Ones (rightmost)
                an  = 4'b1110;
                seg = bcd_to_seg(digit_o);
            end
            2'b01: begin   // Digit 1: Tens
                an  = 4'b1101;
                seg = bcd_to_seg(digit_t);
            end
            2'b10: begin   // Digit 2: Hundreds
                an  = 4'b1011;
                seg = bcd_to_seg(digit_h);
            end
            2'b11: begin   // Digit 3: State char OR area number
                an  = 4'b0111;
                if (billing_mode)
                    seg = bcd_to_seg(bill_area);  // Show area 0-9
                else
                    seg = state_seg;               // Show state char
            end
            default: begin
                an  = 4'b1111;
                seg = 7'b1111111;
            end
        endcase
    end

endmodule
