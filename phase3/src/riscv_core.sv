
module riscv_core(
    inst_addr,
    inst,
    mem_addr,
    mem_data_out,
    mem_data_in,
    mem_write_en,
    halted,
    clk,
    rst_b
);
    output  [31:0] inst_addr;
    input   [31:0] inst;
    output  [31:0] mem_addr;
    input   [7:0]  mem_data_out[0:3];
    output  [7:0]  mem_data_in[0:3];
    output         mem_write_en;
    output reg     halted;
    input          clk;
    input          rst_b;


    //gloabl controls
    wire load, clear;
    assign load = 
        p3_opcode == 7'h03 ? cache_hit :
        p3_opcode == 7'h23 ? cache_hit :
        1
        ;
    assign clear = p2_opcode == 7'h63 ? p2_prediction != taken : 0;
    //pipe0 IF
    reg [31:0] pc;
    reg branch;
    reg first, decision;

    assign inst_addr = pc;

    wire p0_halted;
    wire [6:0] opcode, funct7;
    wire [2:0] funct3;
    wire [4:0] rd, rs1, rs2, rdest;
    wire [31:0] nPC;
    always @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            pc <= 0;
            first <= 1;
        end else if (p2_opcode == 7'h63 && p2_prediction != taken) begin
            pc <= p2_pc;
            first <= 0;
            decision <= ~p2_prediction;
            branch <= 0;
        end else if ((~branch || branch && (p2_opcode == 7'h67)) && load && ~p0_halted) begin
            first <= 1;
            if (opcode == 7'h67) 
                if (branch && p2_opcode == 7'h67) begin
                    branch <= 0;
                    pc <= branch_result;
                end else 
                    branch <= 1;
            else
                pc <= nPC;
        end
    end

    assign nPC = 
        opcode == 7'h6f ? pc + {{11{inst[31]}}, inst[31], inst[19:12], inst[20],inst[30:21], 1'b0} : 
        opcode == 7'h63 ?
            first ?
                prediction ? pc + {{19{inst[31]}}, {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}} : pc + 4
            : decision ?  pc + {{19{inst[31]}}, {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}} : pc + 4
            :
        pc + 4;

    wire prediction, update, taken;
    assign update = p2_opcode == 7'h63 ? 1 : 0;
    assign taken = branch_result == p2_branch_label ? 1 : 0;

    predictor GShare(
        .pc(pc),
        .pc_update(p2_pc),
        .prediction(prediction),
        .update(update),
        .taken(taken),
        .clk(clk),
        .rst_b(rst_b)
    );

    assign rdest = opcode == 7'h23 ? 0 : rd;
    assign p0_halted = inst == 32'h73 ? 1 : 0;;
    assign opcode = inst[6:0] ;
    assign rd = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign funct7 = inst[31:25];

    //pipe1 DR
    reg p1_halted;
    reg [4:0] p1_rd, p1_rs1, p1_rs2;
    reg [2:0] p1_funct3;
    reg [6:0] p1_funct7, p1_opcode;
    reg [11:0] p1_imm1, p1_imm3;
    reg [12:0] p1_imm4;
    reg [19:0] p1_imm2;
    reg [20:0] p1_imm5;
    reg [31:0] p1_pc;
    reg [31:0] p1_branch_label;
    reg p1_prediction;
    
    

    wire [31:0] rf_rs1, rf_rs2, p1_rf_rs1, p1_rf_rs2;
    

    always @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            p1_halted <= 0;
        end else if (~clear && load && ~p1_halted) begin
            p1_halted <= p0_halted;
            p1_rd <= rdest;
            p1_rs1 <= rs1;
            p1_rs2 <= rs2;
            p1_funct3 <= funct3;
            p1_funct7 <= funct7;
            p1_imm1 <= inst[31:20];
            p1_imm2 <= inst[31:12];
            p1_imm3 <= {inst[31:25], inst[11:7]};
            p1_imm4 <= {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
            p1_imm5 <= {inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
            p1_pc <= pc;
            p1_opcode <= opcode;
            p1_branch_label <= pc + {{19{inst[31]}}, {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}};
            if (first)
                p1_prediction <= prediction;
            else 
                p1_prediction <= decision;
        end else if (clear && load) begin
            p1_halted <= 0;
            p1_rd <= 0;
            p1_rs1 <= 0;
            p1_rs2 <= 0;
            p1_funct3 <= 0;
            p1_funct7 <= 0;
            p1_imm1 <= 0;
            p1_imm2 <= 0;
            p1_imm3 <= 0;
            p1_imm4 <= 0;
            p1_imm5 <= 0;
            p1_pc <= 0;
            p1_opcode <= 0;
            p1_branch_label <= 0;
            p1_prediction <= 0;
        end
        
    end

    assign p1_rf_rs1 = 
        p1_rs1 == 0 ? rf_rs1 :
        p1_rs1 == p2_rd ? alu_result :
        p1_rs1 == p3_rd ? p3_data_out :
        p1_rs1 == p4_rd ? p4_rf_rd :
        rf_rs1
        ;    
    
    assign p1_rf_rs2 = 
        p1_rs2 == 0 ? rf_rs2 :
        p1_rs2 == p2_rd ? alu_result :
        p1_rs2 == p3_rd ? p3_data_out :
        p1_rs2 == p4_rd ? p4_rf_rd :
        rf_rs2
        ;    

    regfile RF(
        .rs1_data(rf_rs1),
        .rs2_data(rf_rs2),
        .rs1_num(p1_rs1),
        .rs2_num(p1_rs2),
        .rd_num(p4_rd),
        .rd_data(p4_rf_rd),
        .rd_we(p4_rf_we),
        .clk(clk),
        .rst_b(rst_b),
        .halted(halted)
    );

    //pipe2 ALU
    reg p2_halted;
    reg [31:0] p2_rf_rs1, p2_rf_rs2;
    reg [2:0] p2_funct3;
    reg [6:0] p2_funct7, p2_opcode;
    reg [11:0] p2_imm1;
    reg [19:0] p2_imm2;
    reg [11:0] p2_imm3;
    reg [12:0] p2_imm4;
    reg [20:0] p2_imm5;
    reg [4:0] p2_rd, p2_rs1, p2_rs2;
    reg [31:0] p2_pc;
    reg [31:0] p2_branch_label;
    reg p2_prediction;
    
    wire [31:0] alu_result;
    wire [31:0] branch_result;


    always @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            p2_halted <= 0;
        end else if (~clear && load && ~p2_halted) begin
            p2_halted <= p1_halted;
            p2_funct3 <= p1_funct3;
            p2_funct7 <= p1_funct7;
            p2_imm1 <= p1_imm1;
            p2_imm2 <= p1_imm2;
            p2_imm3 <= p1_imm3;
            p2_imm4 <= p1_imm4;
            p2_imm5 <= p1_imm5;
            p2_pc <= p1_pc;
            p2_opcode <= p1_opcode;
            p2_rd <= p1_rd;
            p2_rs1 <= p1_rs1;
            p2_rs2 <= p1_rs2;
            p2_rf_rs1 <= p1_rf_rs1;
            p2_rf_rs2 <= p1_rf_rs2;
            p2_prediction <= p1_prediction;
            p2_branch_label <= p1_branch_label;
        end else if (clear && load) begin
            p2_halted <= 0;
            p2_funct3 <= 0;
            p2_funct7 <= 0;
            p2_imm1 <= 0;
            p2_imm2 <= 0;
            p2_imm3 <= 0;
            p2_imm4 <= 0;
            p2_imm5 <= 0;
            p2_pc <= 0;
            p2_opcode <= 0;
            p2_rd <= 0;
            p2_rs1 <= 0;
            p2_rs2 <= 0;
            p2_rf_rs1 <= 0;
            p2_rf_rs2 <= 0;
            p2_prediction <= 0;
            p2_branch_label <= 0;
        end
    end
    assign branch_result = 
        (p2_opcode == 7'h63) ? 
            p2_funct3 == 3'd0 && p2_rf_rs1 == p2_rf_rs2 ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_funct3 == 3'd1 && p2_rf_rs1 != p2_rf_rs2 ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_funct3 == 3'd4 && $signed(p2_rf_rs1) < $signed(p2_rf_rs2) ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_funct3 == 3'd5 && $signed(p2_rf_rs1) >= $signed(p2_rf_rs2) ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_funct3 == 3'd6 && p2_rf_rs1 < p2_rf_rs2 ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_funct3 == 3'd7 && p2_rf_rs1 >= p2_rf_rs2 ? p2_pc + {{19{p2_imm4[12]}}, p2_imm4} :
            p2_pc + 4
        : (p2_opcode == 7'h67) ? p2_rf_rs1 + {{20{p2_imm1[11]}}, p2_imm1} & 32'hFFFFFFFE
        : p2_pc + 4;
        
    assign alu_result =  
                (p2_opcode == 7'h33) ? r_type(p2_funct3, p2_funct7, p2_rf_rs1, p2_rf_rs2) :
                (p2_opcode == 7'h13) ? i_type1(p2_funct3, p2_rf_rs1, p2_imm1) :
                (p2_opcode == 7'h37) ? {p2_imm2, 12'd0} :
                (p2_opcode == 7'h17) ? p2_pc + {p2_imm2, 12'd0} :
                (p2_opcode == 7'h03) ? p2_rf_rs1 + {{20{p2_imm1[11]}}, p2_imm1} :
                (p2_opcode == 7'h23) ? p2_rf_rs1 + {{20{p2_imm3[11]}}, p2_imm3} :
                (p2_opcode == 7'h6f) ? p2_pc + 4 :
                (p2_opcode == 7'h67) ? p2_pc + 4 :
                32'd0;

    //pipe3 MEM
    reg p3_halted;
    reg [31:0] p3_data_in, p3_rf_rs2;
    reg [4:0] p3_rd;
    reg [6:0] p3_opcode;
    reg [2:0] p3_funct3;
    reg [11:0] p3_imm1;

    wire [31:0] p3_data_out;
    wire p3_rf_we;
    wire cache_read, cache_write;
    wire [7:0]  cache_data_out[0:3];
    wire [7:0]  cache_data_in[0:3];
    wire cache_hit;
    

    always @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            p3_halted <= 0;        
        end else if (~clear && load && ~p3_halted) begin
            p3_halted <= p2_halted;
            p3_data_in <= alu_result;
            p3_rd <= p2_rd;
            p3_opcode <= p2_opcode;
            p3_funct3 <= p2_funct3;
            p3_rf_rs2 <= p2_rf_rs2;
            p3_imm1 <= p2_imm1;
        end else if (clear && load) begin
            p3_halted <= 0;
            p3_data_in <= 0;
            p3_rd <= 0;
            p3_opcode <= 0;
            p3_funct3 <= 0;
            p3_rf_rs2 <= 0;
            p3_imm1 <= 0;
        end
    end

    assign p3_data_out = 
        p3_opcode == 7'h03 ? i_type2(p3_funct3, p3_imm1, cache_data_out, p3_imm1[1:0]) :
        p3_data_in;
    assign p3_rf_we = 
        p3_opcode == 7'h23 ? 0 :
        p3_opcode == 7'h63 ? 0 :
        1
        ;
    assign cache_read = p3_opcode == 7'h03 ? 1 : 0;
    assign cache_write = p3_opcode == 7'h23 ? 1 : 0;
                            
    assign cache_data_in[0] = (p3_opcode == 7'h23) ? s_type0(p3_funct3, p3_rf_rs2, cache_data_out) : 0 ;
    assign cache_data_in[1] = (p3_opcode == 7'h23) ? s_type1(p3_funct3, p3_rf_rs2, cache_data_out) : 0;
    assign cache_data_in[2] = (p3_opcode == 7'h23) ? s_type2(p3_funct3, p3_rf_rs2, cache_data_out) : 0;
    assign cache_data_in[3] = (p3_opcode == 7'h23) ? s_type3(p3_funct3, p3_rf_rs2, cache_data_out) : 0;

    cache cache(
        .addr(p3_data_in),
        .read(cache_read),
        .write(cache_write),
        .data_out(cache_data_out),
        .data_in(cache_data_in),
        .hit(cache_hit),
        .mem_addr(mem_addr),
        .mem_data_out(mem_data_out),
        .mem_data_in(mem_data_in),
        .mem_write_en(mem_write_en),
        .clk(clk),
        .rst_b(rst_b)
    );


    //pipe4 WB
    reg p4_halted;
    reg [31:0] p4_rf_rd;
    reg p4_rf_we;
    reg [4:0] p4_rd;

    always @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            p4_halted <= 0;
            halted <= 0;
        end else if (load && ~p4_halted) begin
            p4_halted <= p3_halted;
            p4_rf_rd <= p3_data_out;
            p4_rf_we <= p3_rf_we;
            p4_rd <= p3_rd;
        end else if (p4_halted) begin
            halted <= 1;
        end
        
    end

    function [31:0] r_type(input [2:0] funct3, input [6:0] funct7, input [31:0] r1, r2);
        case (funct3)
            3'd0 : begin
                if (funct7 == 7'd0) 
                    r_type = r1 + r2;
                else if (funct7 == 7'd32)
                    r_type = r1 - r2;
            end
            3'd1 : r_type = r1 << r2;
            3'd2 : begin
                if ($signed(r1) < $signed(r2)) 
                    r_type = 32'd1;
                else
                    r_type = 32'd0;
            end
            3'd3 : begin
                if (r1 < r2) 
                    r_type = 32'd1;
                else
                    r_type = 32'd0;
            end
            3'd4 : r_type = r1 ^ r2;
            3'd5 : r_type = r1 >> r2;
            3'd6 : r_type = r1 | r2;
            3'd7 : r_type = r1 & r2;
        endcase
    endfunction

    function [31:0] i_type1(input [2:0] funct3, input [31:0] r1, input [11:0] imm);
        case (funct3)
            3'd0 : i_type1 = r1 + {{20{imm[11]}}, imm};
            3'd1 : begin
                if (imm[11:5] == 7'd0)
                    i_type1 = r1 << imm[4:0];
            end
            3'd2 : begin
                if ($signed(r1) < $signed({{20{imm[11]}}, imm})) 
                    i_type1 = 32'd1;
                else
                    i_type1 = 32'd0;
            end
            3'd3 : begin
                if (r1 < {{20{imm[11]}}, imm}) 
                    i_type1 = 32'd1;
                else
                    i_type1 = 32'd0;
            end
            3'd4 : i_type1 = r1 ^ {{20{imm[11]}}, imm};
            3'd5 : begin
                if (imm[11:5] == 7'd0)
                    i_type1 = r1 >> imm[4:0]; 
                else if (imm[11:5] == 7'd32)
                    i_type1 = $signed(r1) >>> imm[4:0];
            end
            3'd6 : i_type1 = r1 | {{20{imm[11]}}, imm};
            3'd7 : i_type1 = r1 & {{20{imm[11]}}, imm};
        endcase
    endfunction

    function [31:0] i_type2(input [2:0] funct3, input [11:0] imm, input [7:0] mem_data[0:3], input [1:0] idx);
        case (funct3)
            3'd0 : i_type2 = {{24{mem_data[idx][7]}}, mem_data[idx]};
            3'd1 : i_type2 = {{16{mem_data[idx + 1][7]}}, mem_data[idx + 1], mem_data[idx]};
            3'd2 : i_type2 = {mem_data[3], mem_data[2], mem_data[1], mem_data[0]};
            3'd4 : i_type2 = {24'd0, mem_data[idx]};
            3'd5 : i_type2 = {16'd0, mem_data[idx + 1], mem_data[idx]};
            default : ;
        endcase
    endfunction

    function [7:0] s_type0 (input [2:0] funct3, input [31:0] r2, input [7:0] mem_data[0:3]);
        case (funct3)
            3'd0 : begin
                s_type0 = r2[7:0];
            end
            3'd1 : begin
                s_type0 = r2[7:0];
            end
            3'd2 : begin
                s_type0 = r2[7:0];
            end
            default : ;
        endcase
    endfunction

    function [7:0] s_type1 (input [2:0] funct3, input [31:0] r2, input [7:0] mem_data[0:3]);
        case (funct3)
            3'd0 : begin
                s_type1 = mem_data[1];
            end
            3'd1 : begin
                s_type1 = r2[15:8];
            end
            3'd2 : begin
                s_type1 = r2[15:8];
            end
            default : ;
        endcase
    endfunction

    function [7:0] s_type2 (input [2:0] funct3, input [31:0] r2, input [7:0] mem_data[0:3]);
        case (funct3)
            3'd0 : begin
                s_type2 = mem_data[2];
            end
            3'd1 : begin
                s_type2 = mem_data[2];
            end
            3'd2 : begin
                s_type2 = r2[23:16];
            end
            default : ;
        endcase
    endfunction

    function [7:0] s_type3 (input [2:0] funct3, input [31:0] r2, input [7:0] mem_data[0:3]);
        case (funct3)
            3'd0 : begin
                s_type3 = mem_data[3];
            end
            3'd1 : begin
                s_type3 = mem_data[3];
            end
            3'd2 : begin
                s_type3 = r2[31:24];
            end
            default : ;
        endcase
    endfunction


endmodule
