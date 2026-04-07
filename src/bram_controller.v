// ============================================================================
// Module      : bram_controller
// Project     : Autonomous Smart Power Grid Controller
// Description : Single-port synchronous BRAM wrapper. 10 entries × 38 bits.
//               Memory Map per entry (address 0–9 = Area 0–Area 9):
//                 [37:28] = Level Threshold (10-bit, 0-1000)
//                 [27:24] = Priority Level  (4-bit,  0–9)
//                 [23:16] = Usage Rate      (8-bit,  units/sec, 0–255)
//                 [15:0]  = Cost Accumulator(16-bit, monetary total, 0–65535)
// ============================================================================
`timescale 1ns / 1ps

module bram_controller (
    input  wire        clk,
    input  wire        we,        // Write enable (active HIGH)
    input  wire [3:0]  addr,      // Address: 0–9
    input  wire [37:0] din,       // Write data
    output wire [37:0] dout       // Read data
);

    (* ram_style = "distributed" *) reg [37:0] mem [0:15];

    initial begin
        // {level[9:0], priority[3:0], usage[7:0], cost[15:0]}
        mem[0]  = {10'd450, 4'd9, 8'd3,  16'h0000};
        mem[1]  = {10'd400, 4'd8, 8'd4,  16'h0000};
        mem[2]  = {10'd350, 4'd7, 8'd4,  16'h0000};
        mem[3]  = {10'd300, 4'd7, 8'd5,  16'h0000};
        mem[4]  = {10'd250, 4'd5, 8'd5,  16'h0000};
        mem[5]  = {10'd200, 4'd5, 8'd6,  16'h0000};
        mem[6]  = {10'd150, 4'd3, 8'd6,  16'h0000};
        mem[7]  = {10'd100, 4'd3, 8'd8,  16'h0000};
        mem[8]  = {10'd50,  4'd1, 8'd10, 16'h0000};
        mem[9]  = {10'd0,   4'd0, 8'd12, 16'h0000};
        
        // Ensure unused entries are zeroed out
        mem[10] = 38'd0; mem[11] = 38'd0;
        mem[12] = 38'd0; mem[13] = 38'd0;
        mem[14] = 38'd0; mem[15] = 38'd0;
    end

    // Combinational read (0-cycle latency)
    assign dout = mem[addr];

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din;
        end
    end

endmodule
