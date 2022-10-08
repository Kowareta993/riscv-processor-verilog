
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

    wire [31:0] rf_rs1, rf_rs2;
    wire [31:0] rf_rd;
    // wire [4:0] rf_rd_n;
    wire rf_we;

    regfile RF(
        .rs1_data(rf_rs1),
        .rs2_data(rf_rs2),
        .rs1_num(rs1),
        .rs2_num(rs2),
        .rd_num(rd),
        .rd_data(rf_rd),
        .rd_we(rf_we),
        .clk(clk),
        .rst_b(rst_b),
        .halted(halted)
    );

    wire cache_hit, cache_read, cache_write;
    wire [31:0] cache_addr;
    wire [7:0]  cache_data_out[0:3];
    wire [7:0]  cache_data_in[0:3];
    
    cache cache(
        .addr(cache_addr),
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

    wire [6:0] opcode, funct7;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    reg [31:0] pc;
    
    assign opcode = inst[6:0];
    assign rd = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign funct7 = inst[31:25];

    always @(posedge clk or negedge rst_b) begin
        if (rst_b == 1'b0) begin
          halted <= 1'b0;  
          pc <= 32'd0;
        end else if (halted == 1'b0) begin
            // $display("%d %h", pc/4 , inst);
            if (inst == 32'h73) begin
                halted <= 1'b1;
            end else if (opcode == 7'h03 || opcode == 7'h23) begin
                if (cache_hit == 1'b1) begin
                    pc <= pc + 4;
                end
            end else if (opcode == 7'h67) begin
                pc <= (rf_rs1 + {{20{inst[31]}}, inst[31:20]}) & 32'hFFFFFFFE;
            end else if (opcode == 7'h63) begin
                if (funct3 == 3'd0 && rf_rs1 == rf_rs2) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else if (funct3 == 3'd1 && rf_rs1 != rf_rs2) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else if (funct3 == 3'd4 && $signed(rf_rs1) < $signed(rf_rs2)) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else if (funct3 == 3'd5 && $signed(rf_rs1) >= $signed(rf_rs2)) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else if (funct3 == 3'd6 && rf_rs1 < rf_rs2) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else if (funct3 == 3'd7 && rf_rs1 >= rf_rs2) begin
                    pc <= pc + {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
                end else begin
                    pc <= pc + 4;
                end
            end else if (opcode == 7'h6F) begin
                pc <= pc + {{11{inst[31]}}, inst[31], inst[19:12], inst[20],inst[30:21], 1'b0};
            end else begin
                pc <= pc + 4;    
            end
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

    assign inst_addr = pc;
    assign cache_addr =   (opcode == 7'h03) ? rf_rs1 + {{20{inst[31]}}, inst[31:20]} :
                        (opcode == 7'h23) ? rf_rs1 + {{20{inst[31]}}, inst[31:25], inst[11:7]} :
                        32'd0;

    
    assign cache_write = (opcode == 7'h23) ? 1'b1 : 1'b0;
    assign cache_read =  (opcode == 7'h03) ? 1'b1 : 1'b0;
                            

    assign cache_data_in[0] = (opcode == 7'h23) ? s_type0(funct3, rf_rs2, cache_data_out) :
                            0 ;
    assign cache_data_in[1] = (opcode == 7'h23) ? s_type1(funct3, rf_rs2, cache_data_out) :
                            0 ;
    assign cache_data_in[2] = (opcode == 7'h23) ? s_type2(funct3, rf_rs2, cache_data_out) :
                            0 ;
    assign cache_data_in[3] = (opcode == 7'h23) ? s_type3(funct3, rf_rs2, cache_data_out) :
                            0 ;

    assign rf_rd =  (opcode == 7'h33) ? r_type(funct3, funct7, rf_rs1, rf_rs2) :
                    (opcode == 7'h13) ? i_type1(funct3, rf_rs1, inst[31:20]) :
                    (opcode == 7'h03) ? i_type2(funct3, inst[31:20], cache_data_out, inst[21:20]) :
                    (opcode == 7'h67) ? pc + 4 :
                    (opcode == 7'h37) ? {inst[31:12], 12'd0} :
                    (opcode == 7'h17) ? pc + {inst[31:12], 12'd0} :
                    (opcode == 7'h6F) ? pc + 4 :
                    32'd0;

    assign rf_we =  (opcode == 7'h13) ? 1'b1 :
                    (opcode == 7'h33) ? 1'b1 :
                    (opcode == 7'h03 && cache_hit == 1'b1) ? 1'b1 :
                    (opcode == 7'h67) ? 1'b1 :
                    (opcode == 7'h6F) ? 1'b1 :
                    (opcode == 7'h17) ? 1'b1 :
                    (opcode == 7'h37) ? 1'b1 :
                    1'b0;
endmodule
