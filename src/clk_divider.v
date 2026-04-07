// ============================================================================
// Module      : clk_divider
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3 (100 MHz onboard clock, Pin W5)
// Description : Generates two timing pulses from the 100 MHz system clock:
//               1) pulse_5s    - Single-cycle HIGH pulse every 5 seconds
//               2) clk_display - Single-cycle HIGH display tick for 7-seg mux
//                                (100 MHz / 2^18 = ~381 Hz → each digit refreshes ~95 Hz)
// Coding Style: Synchronous Verilog-2001, no combinational loops
// ============================================================================
`timescale 1ns / 1ps

module clk_divider (
    input  wire clk,          // 100 MHz system clock
    input  wire rst,          // Synchronous active-high reset
    output reg  pulse_1hz,    // Single-cycle HIGH pulse every 5 seconds
    output reg  clk_display   // Single-cycle HIGH display tick
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam TICK_PERIOD  = 500_000_000;   // 5 seconds at 100 MHz
    localparam DISP_DIV     = 18;            // 2^18 = 262144 → 100M/262144 ≈ 381 Hz

    // -------------------------------------------------------------------------
    // 5-Second Pulse Generator
    // Counter counts 0 to 499,999,999 then asserts a one-cycle pulse and resets.
    // -------------------------------------------------------------------------
    reg [28:0] cnt_1hz; // 2^29 = 536M > 500M — sufficient bit width

    always @(posedge clk) begin
        if (rst) begin
            cnt_1hz   <= 29'd0;
            pulse_1hz <= 1'b0;
        end else if (cnt_1hz == TICK_PERIOD - 1) begin
            cnt_1hz   <= 29'd0;
            pulse_1hz <= 1'b1;   // Assert pulse for exactly 1 clock cycle
        end else begin
            cnt_1hz   <= cnt_1hz + 1'b1;
            pulse_1hz <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Display Tick Generator (~381 Hz)
    // Produces a one-cycle pulse instead of a derived clock so downstream
    // logic stays on the main system clock.
    // -------------------------------------------------------------------------
    reg [17:0] cnt_disp; // 18-bit counter

    always @(posedge clk) begin
        if (rst) begin
            cnt_disp    <= 18'd0;
            clk_display <= 1'b0;
        end else begin
            cnt_disp <= cnt_disp + 1'b1;       // Free-running; MSB is the clock
            clk_display <= (cnt_disp == {DISP_DIV{1'b1}});
        end
    end

endmodule
