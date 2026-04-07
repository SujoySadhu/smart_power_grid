// ============================================================================
// Module      : alu
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3
// Description : Resource-Sharing Arithmetic Logic Unit (PIPELINED).
//               Single registered pipeline stage breaks the critical
//               combinational path (multiply → add → mux) that caused
//               timing failure at 100 MHz.
//
//               LATENCY: 1 clock cycle (inputs registered on posedge clk,
//               outputs valid on the NEXT posedge).  The FSM's 3-phase
//               CONSUME protocol already accounts for this.
//
//               Operations (selected by op[1:0]):
//
//               2'b00 – NOP / PASS-THROUGH
//                       Outputs pass inputs unchanged.
//
//               2'b01 – CONSUME
//                       Subtractor : balance_out = balance_in – usage_rate
//                                    (saturates at 0, no underflow)
//                       MAC        : cost_acc_out = cost_acc_in + (usage_rate × price_tier)
//                                    price_tier = priority_in + 1  (maps 0–9 → 1–10)
//                                    cost_acc clamped at 16'hFFFF
//                       Accumulate : total_demand_out = total_demand_in + usage_rate
//                       (All operations only execute if area_active = 1)
//
//               2'b10 – TOPUP
//                       Adder : balance_out = balance_in + topup_amount
//                               (saturates at 999, no overflow)
//
//               2'b11 – COMPARE (Stability Monitor)
//                       Comparator : overload_flag = (total_demand_in > balance_in)
//                       Called by FSM after all 10 areas are accumulated.
//
// ============================================================================
`timescale 1ns / 1ps

module alu (
    // --- Clock & Reset (for pipeline register) ---
    input  wire        clk,
    input  wire        rst,

    // --- Operation Select ---
    input  wire [1:0]  op,

    // --- Per-Area Data Inputs (from BRAM via FSM) ---
    input  wire [3:0]  priority_in,      // Area priority (0–9)
    input  wire [7:0]  usage_rate,       // Area consumption rate (units/sec)
    input  wire [15:0] cost_acc_in,      // Current cost accumulator
    input  wire        area_active,      // 1 = area is currently ON (from status_reg)

    // --- Global Register Inputs ---
    input  wire [9:0]  balance_in,       // Current balance register (10-bit, max 999)
    input  wire [9:0]  total_demand_in,  // Running sum of active usage rates
    input  wire [7:0]  topup_amount,     // Amount to add on TOPUP operation

    // --- Registered Outputs (valid 1 cycle after inputs) ---
    output reg  [9:0]  balance_out,
    output reg  [15:0] cost_acc_out,
    output reg  [9:0]  total_demand_out,
    output reg         overload_flag
);

    // -------------------------------------------------------------------------
    // Pipeline Stage 1: Register all inputs + multiply (posedge clk)
    //   This breaks the long comb path:  BRAM → FSM → multiply → add → mux
    //   into two shorter paths:
    //     Path A: BRAM → FSM → input regs (setup to pipeline FF)
    //     Path B: pipeline FF → add → mux → output regs (FSM latches)
    // -------------------------------------------------------------------------
    reg [1:0]  op_s1;
    reg [7:0]  usage_s1;
    reg [15:0] cost_acc_s1;
    reg        active_s1;
    reg [9:0]  balance_s1;
    reg [9:0]  demand_s1;
    reg [7:0]  topup_s1;
    reg [11:0] cost_delta_s1;  // Registered multiply result

    always @(posedge clk) begin
        if (rst) begin
            op_s1         <= 2'b00;
            usage_s1      <= 8'd0;
            cost_acc_s1   <= 16'd0;
            active_s1     <= 1'b0;
            balance_s1    <= 10'd0;
            demand_s1     <= 10'd0;
            topup_s1      <= 8'd0;
            cost_delta_s1 <= 12'd0;
        end else begin
            op_s1         <= op;
            usage_s1      <= usage_rate;
            cost_acc_s1   <= cost_acc_in;
            active_s1     <= area_active;
            balance_s1    <= balance_in;
            demand_s1     <= total_demand_in;
            topup_s1      <= topup_amount;
            // Bill = usage × 2 (simple shift, no multiplier needed)
            cost_delta_s1 <= {3'b0, usage_rate, 1'b0};
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Stage 2: Add + Mux → Registered Outputs (posedge clk)
    //   Uses registered values from Stage 1. The add/mux/compare is now a
    //   much shorter combinational path before the output register.
    // -------------------------------------------------------------------------
    // Intermediate wires for cost accumulation (stage 2 combinational)
    wire [16:0] cost_new_xt = {1'b0, cost_acc_s1} + {5'b0, cost_delta_s1};

    always @(posedge clk) begin
        if (rst) begin
            balance_out      <= 10'd0;
            cost_acc_out     <= 16'd0;
            total_demand_out <= 10'd0;
            overload_flag    <= 1'b0;
        end else begin
            // Defaults: pass-through (NOP)
            balance_out      <= balance_s1;
            cost_acc_out     <= cost_acc_s1;
            total_demand_out <= demand_s1;
            overload_flag    <= 1'b0;

            case (op_s1)

                // ------------------------------------------------------------------
                2'b01: begin // CONSUME — process one active area
                    if (active_s1) begin
                        // Subtractor: saturating at 0
                        if (balance_s1 >= {2'b0, usage_s1})
                            balance_out <= balance_s1 - {2'b0, usage_s1};
                        else
                            balance_out <= 10'd0;

                        // MAC: clamped at 0xFFFF
                        if (cost_new_xt[16])
                            cost_acc_out <= 16'hFFFF;
                        else
                            cost_acc_out <= cost_new_xt[15:0];

                        // Demand accumulator
                        total_demand_out <= demand_s1 + {2'b0, usage_s1};
                    end
                end

                // ------------------------------------------------------------------
                2'b10: begin // TOPUP — add topup to balance, saturate at 999
                    if (balance_s1 + {3'b0, topup_s1} > 11'd999)
                        balance_out <= 10'd999;
                    else
                        balance_out <= balance_s1 + {2'b0, topup_s1};
                end

                // ------------------------------------------------------------------
                2'b11: begin // COMPARE — stability check
                    overload_flag <= (demand_s1 > balance_s1) ? 1'b1 : 1'b0;
                end

                default: ; // 2'b00 (NOP) handled by defaults above

            endcase
        end
    end

endmodule
