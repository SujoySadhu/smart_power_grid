// ============================================================================
// Module      : debouncer
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3
// Description : Parameterized button debouncer.
//               Technique: 2-FF synchronizer to eliminate metastability, then
//               a stable-count filter that requires the input to remain steady
//               for STABLE_CYCLES before passing it through.
//               Outputs:
//                 btn_pulse - ONE-CYCLE HIGH pulse on stable rising edge
//                 btn_level - Debounced steady logic level
// Coding Style: Synchronous Verilog-2001; no initial blocks
// ============================================================================
`timescale 1ns / 1ps

module debouncer #(
    parameter STABLE_CYCLES = 500_000   // 5 ms debounce at 100 MHz
)(
    input  wire clk,
    input  wire rst,
    input  wire btn_raw,     // Raw (bouncy) button input from board pin
    output reg  btn_pulse,   // Single-cycle HIGH pulse on confirmed press
    output reg  btn_level    // Steady debounced level output
);

    // -------------------------------------------------------------------------
    // Stage 1: Two-stage synchronizer to prevent metastability
    // -------------------------------------------------------------------------
    reg sync_ff1, sync_ff2;
    always @(posedge clk) begin
        if (rst) begin
            sync_ff1 <= 1'b0;
            sync_ff2 <= 1'b0;
        end else begin
            sync_ff1 <= btn_raw;
            sync_ff2 <= sync_ff1;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Stable-counter filter
    // Counts consecutive cycles during which sync_ff2 differs from btn_level.
    // Only when the raw input has been stable for STABLE_CYCLES cycles do we
    // update btn_level and emit a single-cycle btn_pulse on rising edge.
    // -------------------------------------------------------------------------
    // Bit-width calculation: ceil(log2(500000)) = 19 bits (2^19 = 524288 > 500000)
    reg [18:0] stable_cnt;

    always @(posedge clk) begin
        if (rst) begin
            stable_cnt <= 19'd0;
            btn_level  <= 1'b0;
            btn_pulse  <= 1'b0;
        end else begin
            btn_pulse <= 1'b0;   // Default: pulse is low

            if (sync_ff2 == btn_level) begin
                // Input matches current level — reset counter
                stable_cnt <= 19'd0;
            end else begin
                // Input differs — count how long it has been stable
                if (stable_cnt == STABLE_CYCLES - 1) begin
                    stable_cnt <= 19'd0;
                    btn_level  <= sync_ff2;       // Latch the new stable level
                    btn_pulse  <= sync_ff2;       // Pulse only on rising edge (1)
                end else begin
                    stable_cnt <= stable_cnt + 1'b1;
                end
            end
        end
    end

endmodule
