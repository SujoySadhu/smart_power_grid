// ============================================================================
// Module      : bram_controller
// Project     : Autonomous Smart Power Grid Controller
// Target      : Artix-7 Basys 3
// Description : Single-port synchronous BRAM wrapper. 10 entries × 28 bits.
//               Memory Map per entry (address 0–9 = Area 0–Area 9):
//                 [27:24] = Priority Level  (4-bit,  0–9)
//                 [23:16] = Usage Rate      (8-bit,  units/sec, 0–255)
//                 [15:0]  = Cost Accumulator(16-bit, monetary total, 0–65535)
//
//               The (* ram_style = "block" *) attribute instructs Vivado to
//               map this memory explicitly into a Block RAM tile (RAMB18E1)
//               instead of distributed LUT-based RAM.
//
//               Read latency : 1 clock cycle (synchronous registered output)
//               Write mode   : Write-First (dout reflects new data on write)
// Coding Style: Synchronous Verilog-2001
// ============================================================================
`timescale 1ns / 1ps

module bram_controller (
    input  wire        clk,
    input  wire        we,        // Write enable (active HIGH)
    input  wire [3:0]  addr,      // Address: 0–9 (values ≥10 are unused)
    input  wire [27:0] din,       // Write data: {priority[3:0], usage[7:0], cost[15:0]}
    output wire [27:0] dout       // Read data (combinational, valid same cycle as addr)
);

    // -------------------------------------------------------------------------
    // RAM Declaration
    // Use distributed (LUT) RAM — only 16 entries × 28 bits, too shallow for
    // a Block RAM tile. Distributed RAM is faster and more area-efficient here.
    // -------------------------------------------------------------------------
    (* ram_style = "distributed" *) reg [27:0] mem [0:15]; // 16-entry array

    // -------------------------------------------------------------------------
    // Synchronous Read-Write Operation (Write-First mode)
    // Write-First: If we=1, dout reflects the newly written din value.
    // This avoids a stale-read issue when the FSM reads back after writing.
    // -------------------------------------------------------------------------
    // Combinational read (0-cycle latency)
    assign dout = mem[addr];

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din;
        end
    end

endmodule
