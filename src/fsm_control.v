// ============================================================================
// Module      : fsm_control  [Persistent Configuration Version]
// ============================================================================
`timescale 1ns / 1ps

module fsm_control (
    input  wire        clk, input wire rst, input wire pulse_1hz,
    input  wire        btn_confirm,
    input  wire [3:0]  area_id,
    input  wire [7:0]  data_entry,
    input  wire        mode_sel,
    input  wire        mode_usage,   // sw[13]: 1=Usage Rate Write mode
    input  wire        freeze,       // sw[14]: 1=Freeze consume logic
    input  wire        mode_level,   // sw[15]: 1=Level Threshold Write mode
    output reg         bram_we,
    output reg  [3:0]  bram_addr,
    output reg  [37:0] bram_din,
    input  wire [37:0] bram_dout,
    output wire [1:0]  alu_op,
    output wire [7:0]  alu_usage_rate,
    output wire [15:0] alu_cost_acc,
    output wire        alu_area_active,
    output wire [9:0]  alu_balance_in,
    output wire [10:0]  alu_total_demand_in, // Expanded internally for ALU match if needed (10-bit is mapped in top level)
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

    reg [3:0] cached_priority [0:9];
    reg [7:0] cached_usage    [0:9];
    reg [9:0] area_bill [0:9];
    reg [9:0] level_thresh [0:9];

    reg [3:0]  area_ptr;
    reg [2:0]  area_phase;
    reg [9:0]  total_demand;
    reg [11:0] bill_delta; 
    reg        eval_pending;
    reg [27:0] bill_timer;
    reg [29:0] zero_timer;

    reg        zero_timeout_hit;
    reg        bill_timeout_hit;
    reg        all_off_reg;

    reg [3:0] shed_count;
    always @(posedge clk) begin
        if (rst)
            shed_count <= 4'd0;
        else
            shed_count <= (level_thresh[0] > balance_reg)
                        + (level_thresh[1] > balance_reg)
                        + (level_thresh[2] > balance_reg)
                        + (level_thresh[3] > balance_reg)
                        + (level_thresh[4] > balance_reg)
                        + (level_thresh[5] > balance_reg)
                        + (level_thresh[6] > balance_reg)
                        + (level_thresh[7] > balance_reg)
                        + (level_thresh[8] > balance_reg)
                        + (level_thresh[9] > balance_reg);
    end

    reg [9:0] cmp_bits [0:9];
    integer ri, rj;
    reg [9:0] cmp_tmp;

    always @(posedge clk) begin
        if (rst) begin
            cmp_bits[0] <= 10'b1111111110; cmp_bits[1] <= 10'b1111111100;
            cmp_bits[2] <= 10'b1111111000; cmp_bits[3] <= 10'b1111110000;
            cmp_bits[4] <= 10'b1111100000; cmp_bits[5] <= 10'b1111000000;
            cmp_bits[6] <= 10'b1110000000; cmp_bits[7] <= 10'b1100000000;
            cmp_bits[8] <= 10'b1000000000; cmp_bits[9] <= 10'b0000000000;
        end else begin
            for (ri = 0; ri < 10; ri = ri + 1) begin
                cmp_tmp = 10'd0;
                for (rj = 0; rj < 10; rj = rj + 1) begin
                    if (ri != rj) begin
                        if (cached_priority[rj] < cached_priority[ri])
                            cmp_tmp[rj] = 1'b1;
                        else if (cached_priority[rj] == cached_priority[ri] && rj > ri)
                            cmp_tmp[rj] = 1'b1;
                    end
                end
                cmp_bits[ri] <= cmp_tmp;
            end
        end
    end

    reg [3:0] rank [0:9];
    integer rk;
    always @(posedge clk) begin
        if (rst) begin
            rank[0]<=4'd9; rank[1]<=4'd8; rank[2]<=4'd7; rank[3]<=4'd6;
            rank[4]<=4'd5; rank[5]<=4'd4; rank[6]<=4'd3; rank[7]<=4'd2;
            rank[8]<=4'd1; rank[9]<=4'd0;
        end else begin
            for (rk = 0; rk < 10; rk = rk + 1)
                rank[rk] <= cmp_bits[rk][0] + cmp_bits[rk][1] + cmp_bits[rk][2]
                          + cmp_bits[rk][3] + cmp_bits[rk][4] + cmp_bits[rk][5]
                          + cmp_bits[rk][6] + cmp_bits[rk][7] + cmp_bits[rk][8]
                          + cmp_bits[rk][9];
        end
    end

    wire balance_nonzero = (balance_reg > 10'd0);
    wire [9:0] computed_status;
    assign computed_status[0] = balance_nonzero & (rank[0] >= shed_count);
    assign computed_status[1] = balance_nonzero & (rank[1] >= shed_count);
    assign computed_status[2] = balance_nonzero & (rank[2] >= shed_count);
    assign computed_status[3] = balance_nonzero & (rank[3] >= shed_count);
    assign computed_status[4] = balance_nonzero & (rank[4] >= shed_count);
    assign computed_status[5] = balance_nonzero & (rank[5] >= shed_count);
    assign computed_status[6] = balance_nonzero & (rank[6] >= shed_count);
    assign computed_status[7] = balance_nonzero & (rank[7] >= shed_count);
    assign computed_status[8] = balance_nonzero & (rank[8] >= shed_count);
    assign computed_status[9] = balance_nonzero & (rank[9] >= shed_count);

    wire all_off = (computed_status == 10'd0);

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

    reg [1:0]  r_alu_op;
    reg [7:0]  r_alu_usage_rate;
    reg [15:0] r_alu_cost_acc;
    reg        r_alu_area_active;
    reg [9:0]  r_alu_balance_in;
    reg [9:0]  r_alu_total_demand_in;
    reg [7:0]  r_alu_topup;

    assign alu_op=r_alu_op;
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
            bram_we<=1'b0; bram_addr<=4'd0; bram_din<=38'd0;
            r_alu_op<=2'b00; r_alu_usage_rate<=8'd0;
            r_alu_cost_acc<=16'd0; r_alu_area_active<=1'b0;
            r_alu_balance_in<=10'd500; r_alu_total_demand_in<=10'd0; r_alu_topup<=8'd0;
            
            // Temporary default initialization on reset - IDLE state will overwrite this with persistent BRAM data
            level_thresh[0]<=10'd450; level_thresh[1]<=10'd400;
            level_thresh[2]<=10'd350; level_thresh[3]<=10'd300;
            level_thresh[4]<=10'd250; level_thresh[5]<=10'd200;
            level_thresh[6]<=10'd150; level_thresh[7]<=10'd100;
            level_thresh[8]<=10'd50;  level_thresh[9]<=10'd0;
            for (init_i=0;init_i<10;init_i=init_i+1) begin
                cached_priority[init_i]<=4'd5;
                cached_usage[init_i]<=8'd5;
                area_bill[init_i]<=10'd0;
            end
        end else begin
            bram_we<=1'b0; r_alu_op<=2'b00; eval_pending<=1'b0;

            case (fsm_state)

                IDLE: begin
                    // Load configurations from persistent BRAM setup
                    if (area_phase == 3'd0) begin
                        bram_we<=1'b0; bram_addr<=area_ptr; area_phase<=3'd1;
                        balance_reg<=10'd500;
                        status_reg<=computed_status;
                        overload_flag<=1'b0;
                        total_demand<=10'd0; bill_area<=4'd0; zero_timer<=30'd0;
                    end else begin
                        level_thresh[area_ptr] <= bram_dout[37:28];
                        cached_priority[area_ptr] <= bram_dout[27:24];
                        cached_usage[area_ptr] <= bram_dout[23:16];
                        area_bill[area_ptr] <= 10'd0;
                        
                        if (area_ptr==4'd9) begin 
                            area_ptr<=4'd0; area_phase<=3'd0; fsm_state<=MONITOR; 
                        end else begin 
                            area_ptr<=area_ptr+1'b1; area_phase<=3'd0; 
                        end
                    end
                end

                MONITOR: begin
                    status_reg <= computed_status;
                    overload_flag <= ~(&computed_status); // ON if any area is OFF

                    // Auto-reset: all areas off for 10 seconds (disabled if frozen)
                    if (all_off_reg && !freeze) begin
                        if (zero_timeout_hit) begin
                            area_ptr<=4'd0; area_phase<=3'd0;
                            bill_timer<=28'd0; fsm_state<=BILLING;
                        end else
                            zero_timer<=zero_timer+1'b1;
                    end else
                        zero_timer<=30'd0;

                    if (btn_confirm) begin
                        if (mode_level) begin
                            // sw[15]=1: Level Threshold Write
                            if (area_id <= 4'd9) begin
                                bram_we<=1'b1; bram_addr<=area_id;
                                if (data_entry >= 8'd200) begin
                                    level_thresh[area_id] <= 10'd999;
                                    bram_din <= {10'd999, cached_priority[area_id], cached_usage[area_id], 16'h0000};
                                end else begin
                                    level_thresh[area_id] <= {data_entry, 2'b00} + {2'b0, data_entry};
                                    bram_din <= {{data_entry, 2'b00} + {2'b0, data_entry}, cached_priority[area_id], cached_usage[area_id], 16'h0000};
                                end
                            end
                        end else if (mode_usage) begin
                            // sw[13]=1: Usage Rate Write
                            if (area_id<=4'd9) begin
                                bram_we<=1'b1; bram_addr<=area_id;
                                bram_din<={level_thresh[area_id], cached_priority[area_id], data_entry, 16'h0000};
                                cached_usage[area_id]<=data_entry;
                            end
                        end else if (!mode_sel) begin
                            // sw[13]=0, sw[12]=0: Priority Write
                            if (area_id<=4'd9) begin
                                bram_we<=1'b1; bram_addr<=area_id;
                                bram_din<={level_thresh[area_id], data_entry[3:0], cached_usage[area_id], 16'h0000};
                                cached_priority[area_id]<=data_entry[3:0];
                            end
                        end else begin
                            // sw[13]=0, sw[12]=1: Balance Top-Up
                            if (balance_reg+{3'b0,data_entry}>11'd999)
                                balance_reg<=10'd999;
                            else
                                balance_reg<=balance_reg+{2'b0,data_entry};
                        end
                    end

                    if (pulse_1hz && !all_off && !freeze) begin
                        area_ptr<=4'd0; area_phase<=3'd0;
                        total_demand<=10'd0; fsm_state<=CONSUME;
                    end
                end

                CONSUME: begin
                    if (!eval_pending) begin
                        case (area_phase)
                            3'd0: begin bram_we<=1'b0; bram_addr<=area_ptr; area_phase<=3'd1; end
                            3'd1: begin
                                cached_priority[area_ptr]<=bram_dout[27:24]; // Safe BRAM cache reload
                                cached_usage[area_ptr]<=bram_dout[23:16];
                                r_alu_op<=2'b01;
                                r_alu_usage_rate<=bram_dout[23:16];
                                r_alu_cost_acc<=bram_dout[15:0];
                                r_alu_area_active<=status_reg[area_ptr];
                                r_alu_balance_in<=balance_reg;
                                r_alu_total_demand_in<=total_demand;
                                area_phase<=3'd2;
                            end
                            3'd2: begin r_alu_op<=2'b01; area_phase<=3'd3; end
                            3'd3: begin
                                r_alu_op<=2'b01;
                                bill_delta <= cached_usage[area_ptr] * (cached_priority[area_ptr] + 4'd1);
                                area_phase<=3'd4;
                            end
                            3'd4: begin
                                bram_we<=1'b1; bram_addr<=area_ptr;
                                bram_din<={level_thresh[area_ptr], cached_priority[area_ptr],
                                           cached_usage[area_ptr], alu_cost_acc_out};
                                balance_reg<=alu_balance_out;
                                total_demand<=alu_total_demand_out;
                                if (status_reg[area_ptr]) begin
                                    if (area_bill[area_ptr] + {1'b0, bill_delta} > 13'd999)
                                        area_bill[area_ptr]<=10'd999;
                                    else
                                        area_bill[area_ptr]<=area_bill[area_ptr] + bill_delta[9:0];
                                end
                                if (area_ptr==4'd9) begin eval_pending<=1'b1; area_phase<=3'd0; end
                                else begin area_ptr<=area_ptr+1'b1; area_phase<=3'd0; end
                            end
                            default: area_phase<=3'd0;
                        endcase
                    end else begin
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
                            // Clear cost section but maintain the persistence of the settings
                            bram_din<={bram_dout[37:28], bram_dout[27:24], bram_dout[23:16], 16'h0000};
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
