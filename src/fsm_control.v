// ============================================================================
// Module      : fsm_control  [v10 — Synthesis-Safe Direct Balance Control]
//
// KEY LOGIC: Each area has a threshold = (9 - priority) * 50.
//   status_reg[i] is ON when balance >= area's threshold.
//   Thresholds computed as COMBINATIONAL WIRES (no task, no function).
// ============================================================================
`timescale 1ns / 1ps

module fsm_control (
    input  wire        clk, input wire rst, input wire pulse_1hz,
    input  wire        btn_confirm,
    input  wire [3:0]  area_id,
    input  wire [7:0]  data_entry,
    input  wire        mode_sel,
    output reg         bram_we,
    output reg  [3:0]  bram_addr,
    output reg  [27:0] bram_din,
    input  wire [27:0] bram_dout,
    output wire [1:0]  alu_op,
    output wire [3:0]  alu_priority,
    output wire [7:0]  alu_usage_rate,
    output wire [15:0] alu_cost_acc,
    output wire        alu_area_active,
    output wire [9:0]  alu_balance_in,
    output wire [9:0]  alu_total_demand_in,
    output wire [7:0]  alu_topup,
    input  wire [9:0]  alu_balance_out,
    input  wire [15:0] alu_cost_acc_out,
    input  wire [9:0]  alu_total_demand_out,
    output reg  [9:0]  balance_reg,
    output reg  [9:0]  status_reg,
    output reg         overload_flag,
    output reg  [2:0]  fsm_state,
    output reg  [3:0]  bill_area
);

    localparam IDLE=3'b000, MONITOR=3'b001, CONSUME=3'b010,
               RESET=3'b100, BILLING=3'b101;

    localparam [27:0] BILL_DISPLAY_CYCLES = 28'd150_000_000;
    localparam [29:0] ZERO_TIMEOUT        = 30'd999_999_999;

    // Default area config: {priority[3:0], usage[7:0]}
    localparam [11:0] DEF_AREA_0 = {4'd9, 8'd3};
    localparam [11:0] DEF_AREA_1 = {4'd8, 8'd4};
    localparam [11:0] DEF_AREA_2 = {4'd7, 8'd4};
    localparam [11:0] DEF_AREA_3 = {4'd7, 8'd5};
    localparam [11:0] DEF_AREA_4 = {4'd5, 8'd5};
    localparam [11:0] DEF_AREA_5 = {4'd5, 8'd6};
    localparam [11:0] DEF_AREA_6 = {4'd3, 8'd6};
    localparam [11:0] DEF_AREA_7 = {4'd3, 8'd8};
    localparam [11:0] DEF_AREA_8 = {4'd1, 8'd10};
    localparam [11:0] DEF_AREA_9 = {4'd0, 8'd12};

    function [3:0] def_priority; input [3:0] idx; begin
        case(idx)
            4'd0:def_priority=4'd9; 4'd1:def_priority=4'd8;
            4'd2:def_priority=4'd7; 4'd3:def_priority=4'd7;
            4'd4:def_priority=4'd5; 4'd5:def_priority=4'd5;
            4'd6:def_priority=4'd3; 4'd7:def_priority=4'd3;
            4'd8:def_priority=4'd1; 4'd9:def_priority=4'd0;
            default:def_priority=4'd5;
        endcase
    end endfunction

    function [7:0] def_usage; input [3:0] idx; begin
        case(idx)
            4'd0:def_usage=8'd3;  4'd1:def_usage=8'd4;
            4'd2:def_usage=8'd4;  4'd3:def_usage=8'd5;
            4'd4:def_usage=8'd5;  4'd5:def_usage=8'd6;
            4'd6:def_usage=8'd6;  4'd7:def_usage=8'd8;
            4'd8:def_usage=8'd10; 4'd9:def_usage=8'd12;
            default:def_usage=8'd5;
        endcase
    end endfunction

    reg [3:0] cached_priority [0:9];
    reg [7:0] cached_usage    [0:9];
    reg [9:0] area_bill [0:9];

    reg [3:0]  area_ptr;
    reg [2:0]  area_phase;
    reg [9:0]  total_demand;
    reg        eval_pending;
    reg [27:0] bill_timer;
    reg [29:0] zero_timer;

    // Pre-registered timeout flags — breaks wide-compare critical paths
    reg        zero_timeout_hit;
    reg        bill_timeout_hit;
    reg        all_off_reg;

    // =====================================================================
    // COMBINATIONAL threshold computation — NO function calls in sequential
    // Threshold = (9 - priority) * 50. Priority 9 → thresh 0 (always on).
    // This is purely combinational and synthesis-safe.
    // =====================================================================
    wire [9:0] thresh [0:9];
    assign thresh[0] = (4'd9 - cached_priority[0]) * 8'd50;
    assign thresh[1] = (4'd9 - cached_priority[1]) * 8'd50;
    assign thresh[2] = (4'd9 - cached_priority[2]) * 8'd50;
    assign thresh[3] = (4'd9 - cached_priority[3]) * 8'd50;
    assign thresh[4] = (4'd9 - cached_priority[4]) * 8'd50;
    assign thresh[5] = (4'd9 - cached_priority[5]) * 8'd50;
    assign thresh[6] = (4'd9 - cached_priority[6]) * 8'd50;
    assign thresh[7] = (4'd9 - cached_priority[7]) * 8'd50;
    assign thresh[8] = (4'd9 - cached_priority[8]) * 8'd50;
    assign thresh[9] = (4'd9 - cached_priority[9]) * 8'd50;

    // Combinational: which areas should be ON right now
    wire [9:0] computed_status;
    assign computed_status[0] = (balance_reg > thresh[0]);
    assign computed_status[1] = (balance_reg > thresh[1]);
    assign computed_status[2] = (balance_reg > thresh[2]);
    assign computed_status[3] = (balance_reg > thresh[3]);
    assign computed_status[4] = (balance_reg > thresh[4]);
    assign computed_status[5] = (balance_reg > thresh[5]);
    assign computed_status[6] = (balance_reg > thresh[6]);
    assign computed_status[7] = (balance_reg > thresh[7]);
    assign computed_status[8] = (balance_reg > thresh[8]);
    assign computed_status[9] = (balance_reg > thresh[9]);

    wire all_off = (computed_status == 10'd0);

    // Register timeout comparisons to shorten critical paths
    always @(posedge clk) begin
        if (rst) begin
            zero_timeout_hit <= 1'b0;
            bill_timeout_hit <= 1'b0;
            all_off_reg      <= 1'b0;
        end else begin
            zero_timeout_hit <= (zero_timer >= ZERO_TIMEOUT);
            bill_timeout_hit <= (bill_timer >= BILL_DISPLAY_CYCLES - 1);
            all_off_reg      <= all_off;
        end
    end

    // ALU interface registers
    reg [1:0]  r_alu_op;
    reg [3:0]  r_alu_priority;
    reg [7:0]  r_alu_usage_rate;
    reg [15:0] r_alu_cost_acc;
    reg        r_alu_area_active;
    reg [9:0]  r_alu_balance_in;
    reg [9:0]  r_alu_total_demand_in;
    reg [7:0]  r_alu_topup;

    assign alu_op=r_alu_op; assign alu_priority=r_alu_priority;
    assign alu_usage_rate=r_alu_usage_rate; assign alu_cost_acc=r_alu_cost_acc;
    assign alu_area_active=r_alu_area_active; assign alu_balance_in=r_alu_balance_in;
    assign alu_total_demand_in=r_alu_total_demand_in; assign alu_topup=r_alu_topup;

    integer init_i;
    always @(posedge clk) begin
        if (rst) begin
            fsm_state<=IDLE; area_ptr<=4'd0; area_phase<=3'd0;
            balance_reg<=10'd500; status_reg<=10'h3FF; overload_flag<=1'b0;
            total_demand<=10'd0; eval_pending<=1'b0;
            bill_area<=4'd0; bill_timer<=28'd0; zero_timer<=30'd0;
            bram_we<=1'b0; bram_addr<=4'd0; bram_din<=28'd0;
            r_alu_op<=2'b00; r_alu_priority<=4'd0; r_alu_usage_rate<=8'd0;
            r_alu_cost_acc<=16'd0; r_alu_area_active<=1'b0;
            r_alu_balance_in<=10'd500; r_alu_total_demand_in<=10'd0; r_alu_topup<=8'd0;
            for (init_i=0;init_i<10;init_i=init_i+1) begin
                cached_priority[init_i]<=def_priority(init_i[3:0]);
                cached_usage[init_i]<=def_usage(init_i[3:0]);
                area_bill[init_i]<=10'd0;
            end
        end else begin
            bram_we<=1'b0; r_alu_op<=2'b00; eval_pending<=1'b0;

            case (fsm_state)

                IDLE: begin
                    balance_reg<=10'd500;
                    status_reg<=computed_status;
                    overload_flag<=1'b0;
                    total_demand<=10'd0; bill_area<=4'd0; zero_timer<=30'd0;
                    area_phase<=3'd0;
                    bram_we<=1'b1; bram_addr<=area_ptr;
                    bram_din<={def_priority(area_ptr), def_usage(area_ptr), 16'h0000};
                    cached_priority[area_ptr]<=def_priority(area_ptr);
                    cached_usage[area_ptr]<=def_usage(area_ptr);
                    area_bill[area_ptr]<=10'd0;
                    if (area_ptr==4'd9) begin area_ptr<=4'd0; fsm_state<=MONITOR; end
                    else area_ptr<=area_ptr+1'b1;
                end

                MONITOR: begin
                    // Update status from combinational wires (always responsive)
                    status_reg <= computed_status;
                    overload_flag <= ~(&computed_status); // ON if any area is OFF

                    // Auto-reset: all areas off for 10 seconds
                    if (all_off_reg) begin
                        if (zero_timeout_hit) begin
                            area_ptr<=4'd0; area_phase<=3'd0;
                            bill_timer<=28'd0; fsm_state<=BILLING;
                        end else
                            zero_timer<=zero_timer+1'b1;
                    end else
                        zero_timer<=30'd0;

                    // Handle user input
                    if (btn_confirm) begin
                        if (!mode_sel) begin
                            if (area_id<=4'd9) begin
                                bram_we<=1'b1; bram_addr<=area_id;
                                bram_din<={data_entry[3:0], cached_usage[area_id], 16'h0000};
                                cached_priority[area_id]<=data_entry[3:0];
                            end
                        end else begin
                            if (balance_reg+{3'b0,data_entry}>11'd999)
                                balance_reg<=10'd999;
                            else
                                balance_reg<=balance_reg+{2'b0,data_entry};
                        end
                    end

                    if (pulse_1hz && !all_off) begin
                        area_ptr<=4'd0; area_phase<=3'd0;
                        total_demand<=10'd0; fsm_state<=CONSUME;
                    end
                end

                CONSUME: begin
                    if (!eval_pending) begin
                        case (area_phase)
                            3'd0: begin bram_we<=1'b0; bram_addr<=area_ptr; area_phase<=3'd1; end
                            3'd1: begin
                                cached_priority[area_ptr]<=bram_dout[27:24];
                                cached_usage[area_ptr]<=bram_dout[23:16];
                                r_alu_op<=2'b01;
                                r_alu_priority<=bram_dout[27:24];
                                r_alu_usage_rate<=bram_dout[23:16];
                                r_alu_cost_acc<=bram_dout[15:0];
                                r_alu_area_active<=status_reg[area_ptr];
                                r_alu_balance_in<=balance_reg;
                                r_alu_total_demand_in<=total_demand;
                                area_phase<=3'd2;
                            end
                            3'd2: begin r_alu_op<=2'b01; area_phase<=3'd3; end
                            3'd3: begin r_alu_op<=2'b01; area_phase<=3'd4; end
                            3'd4: begin
                                bram_we<=1'b1; bram_addr<=area_ptr;
                                bram_din<={cached_priority[area_ptr],
                                           cached_usage[area_ptr], alu_cost_acc_out};
                                balance_reg<=alu_balance_out;
                                total_demand<=alu_total_demand_out;
                                if (status_reg[area_ptr]) begin
                                    if (area_bill[area_ptr]+{2'b0,cached_usage[area_ptr],1'b0}>11'd999)
                                        area_bill[area_ptr]<=10'd999;
                                    else
                                        area_bill[area_ptr]<=area_bill[area_ptr]+{1'b0,cached_usage[area_ptr],1'b0};
                                end
                                if (area_ptr==4'd9) begin eval_pending<=1'b1; area_phase<=3'd0; end
                                else begin area_ptr<=area_ptr+1'b1; area_phase<=3'd0; end
                            end
                            default: area_phase<=3'd0;
                        endcase
                    end else begin
                        // Update status from new balance
                        status_reg <= computed_status;
                        overload_flag <= ~(&computed_status);
                        fsm_state<=MONITOR;
                    end
                end

                BILLING: begin
                    case (area_phase)
                        3'd0: begin
                            bill_area<=area_ptr;
                            balance_reg<=area_bill[area_ptr];
                            bill_timer<=28'd0; area_phase<=3'd2;
                        end
                        3'd2: begin
                            // 1-cycle settle: bill_timer is now 0, safe to start counting
                            area_phase<=3'd1;
                        end
                        3'd1: begin
                            if (bill_timeout_hit) begin
                                if (area_ptr==4'd9) begin
                                    area_ptr<=4'd0; area_phase<=3'd0; fsm_state<=RESET;
                                end else begin
                                    area_ptr<=area_ptr+1'b1; area_phase<=3'd0;
                                end
                            end else bill_timer<=bill_timer+1'b1;
                        end
                        default: area_phase<=3'd0;
                    endcase
                end

                RESET: begin
                    case (area_phase)
                        3'd0: begin bram_we<=1'b0; bram_addr<=area_ptr; area_phase<=3'd1; end
                        3'd1: begin
                            bram_we<=1'b1; bram_addr<=area_ptr;
                            bram_din<={bram_dout[27:24], bram_dout[23:16], 16'h0000};
                            area_phase<=3'd0;
                            if (area_ptr==4'd9) begin
                                balance_reg<=10'd500; overload_flag<=1'b0;
                                total_demand<=10'd0; bill_area<=4'd0;
                                zero_timer<=30'd0; area_ptr<=4'd0;
                                for (init_i=0;init_i<10;init_i=init_i+1)
                                    area_bill[init_i]<=10'd0;
                                fsm_state<=MONITOR;
                            end else area_ptr<=area_ptr+1'b1;
                        end
                        default: area_phase<=3'd0;
                    endcase
                end

                default: fsm_state<=IDLE;
            endcase
        end
    end
endmodule
