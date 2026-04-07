// ============================================================================
// Module      : top_level
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3
// Description : Top-level integration module. Instantiates and connects all
//               submodules. No logic lives here except I/O wiring.
//
//   I/O Mapping (Basys 3):
//     clk      (W5)          : 100 MHz onboard oscillator
//     sw[3:0]                : Area ID (0–9) for runtime write target
//     sw[11:4]               : Data Entry — new priority value OR top-up amount
//     sw[12]                 : Mode select: 0=Priority Write, 1=Balance Top-Up
//     sw[15:13]              : Unused (tied to ground by design)
//     btnC                   : Global Reset (debounced)
//     btnU                   : Confirm / Execute (debounced)
//     led[9:0]               : Status Register — 1=area ON, 0=area OFF (virtual CB)
//     led[10]                : Overload Flag (OF) — 1=Total Demand > Balance
//     led[13:11]             : FSM State debug output (3-bit binary on LEDs)
//     led[15:14]             : Unused
//     an[3:0]                : 7-seg anodes (active LOW)
//     seg[6:0]               : 7-seg cathodes (active LOW)
//
// ============================================================================
`timescale 1ns / 1ps

module top_level (
    input  wire        clk,         // 100 MHz

    // Switches
    input  wire [15:0] sw,

    // Buttons
    input  wire        btnC,        // Centre button: Global Reset
    input  wire        btnU,        // Up button:    Confirm / Execute

    // LEDs
    output wire [15:0] led,

    // 7-Segment Display
    output wire [6:0]  seg,
    output wire [3:0]  an
);

    // =========================================================================
    // Internal Signal Declarations
    // =========================================================================

    // Clock signals
    wire pulse_1hz;
    wire clk_display;

    // Debounced button outputs
    wire rst_clean;       // Debounced btnC level (used as synchronous reset)
    wire confirm_pulse;   // Debounced btnU single-cycle pulse

    // BRAM interface wires (FSM → BRAM)
    wire        bram_we;
    wire [3:0]  bram_addr;
    wire [37:0] bram_din;
    wire [37:0] bram_dout;

    // ALU interface wires
    wire [1:0]  alu_op;
    wire [7:0]  alu_usage_rate;
    wire [15:0] alu_cost_acc;
    wire        alu_area_active;
    wire [9:0]  alu_balance_in;
    wire [9:0]  alu_total_demand_in;
    wire [7:0]  alu_topup;
    wire [9:0]  alu_balance_out;
    wire [15:0] alu_cost_acc_out;
    wire [9:0]  alu_total_demand_out;
    // alu_overload: FSM computes this inline; ALU output left unconnected.

    // Global state from FSM
    wire [9:0]  balance_reg;
    wire [9:0]  status_reg;
    wire        overload_flag;
    wire [2:0]  fsm_state;
    wire [3:0]  bill_area;

    // =========================================================================
    // Module Instantiations
    // =========================================================================

    // --- Clock Divider ---
    clk_divider u_clk_div (
        .clk         (clk),
        .rst         (rst_clean),
        .pulse_1hz   (pulse_1hz),
        .clk_display (clk_display)
    );

    // --- Button Debouncer: btnC (Global Reset) ---
    // Using btn_level as synchronous reset so it holds HIGH while button held
    debouncer #(.STABLE_CYCLES(500_000)) u_deb_reset (
        .clk       (clk),
        .rst       (1'b0),          // This debouncer needs no external reset
        .btn_raw   (btnC),
        .btn_pulse (),              // Not used for reset; we use level
        .btn_level (rst_clean)
    );

    // --- Button Debouncer: btnU (Confirm / Execute) ---
    debouncer #(.STABLE_CYCLES(500_000)) u_deb_confirm (
        .clk       (clk),
        .rst       (rst_clean),
        .btn_raw   (btnU),
        .btn_pulse (confirm_pulse), // Single-cycle pulse for FSM actions
        .btn_level ()               // Not used
    );

    // --- BRAM Controller ---
    bram_controller u_bram (
        .clk  (clk),
        .we   (bram_we),
        .addr (bram_addr),
        .din  (bram_din),
        .dout (bram_dout)
    );

    // --- Resource-Sharing ALU (Pipelined, 2-cycle latency) ---
    alu u_alu (
        .clk                (clk),
        .rst                (rst_clean),
        .op                 (alu_op),
        .usage_rate         (alu_usage_rate),
        .cost_acc_in        (alu_cost_acc),
        .area_active        (alu_area_active),
        .balance_in         (alu_balance_in),
        .total_demand_in    (alu_total_demand_in),
        .topup_amount       (alu_topup),
        .balance_out        (alu_balance_out),
        .cost_acc_out       (alu_cost_acc_out),
        .total_demand_out   (alu_total_demand_out),
        .overload_flag      ()   // FSM computes overload inline; left unconnected
    );

    // --- FSM Control Unit ---
    fsm_control u_fsm (
        .clk                (clk),
        .rst                (rst_clean),
        .pulse_1hz          (pulse_1hz),
        .btn_confirm        (confirm_pulse),
        .area_id            (sw[3:0]),
        .data_entry         (sw[11:4]),
        .mode_sel           (sw[12]),
        .mode_usage         (sw[13]),
        .freeze             (sw[14]),
        .mode_level         (sw[15]),
        // BRAM
        .bram_we            (bram_we),
        .bram_addr          (bram_addr),
        .bram_din           (bram_din),
        .bram_dout          (bram_dout),
        // ALU
        .alu_op             (alu_op),
        .alu_usage_rate     (alu_usage_rate),
        .alu_cost_acc       (alu_cost_acc),
        .alu_area_active    (alu_area_active),
        .alu_balance_in     (alu_balance_in),
        .alu_total_demand_in(alu_total_demand_in),
        .alu_topup          (alu_topup),
        .alu_balance_out    (alu_balance_out),
        .alu_cost_acc_out   (alu_cost_acc_out),
        .alu_total_demand_out(alu_total_demand_out),
        // alu_overload removed: FSM computes overload inline
        // State outputs
        .balance_reg        (balance_reg),
        .status_reg         (status_reg),
        .overload_flag      (overload_flag),
        .fsm_state          (fsm_state),
        .bill_area          (bill_area)
    );

    // --- 7-Segment Display ---
    seg7_display u_display (
        .clk         (clk),
        .rst         (rst_clean),
        .clk_display (clk_display),
        .balance     (balance_reg),
        .fsm_state   (fsm_state),
        .bill_area   (bill_area),
        .an          (an),
        .seg         (seg)
    );

    // =========================================================================
    // LED Assignments
    // =========================================================================
    assign led[9:0]   = status_reg;        // Virtual Circuit Breakers (1=ON, 0=OFF)
    assign led[10]    = overload_flag;      // Overload Flag
    assign led[11]    = (fsm_state == 3'b101); // Billing indicator (ON during bill display)
    // led[15:12] echo unused switches to suppress no-load / constant-driver warnings
    assign led[12]    = sw[12];
    assign led[13]    = sw[13];
    assign led[14]    = sw[14];
    assign led[15]    = sw[15];

endmodule

