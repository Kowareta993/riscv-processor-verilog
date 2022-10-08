module cache(
    addr,
    read,
    write,
    data_out,
    data_in,
    hit,
    mem_addr,
    mem_data_out,
    mem_data_in,
    mem_write_en,
    clk,
    rst_b
);
    input   [31:0]  addr;
    input           read;
    input           write;
    output  [7:0]   data_out[0:3];
    input  [7:0]    data_in[0:3];
    output          hit;
    output  [31:0]  mem_addr;
    input   [7:0]   mem_data_out[0:3];
    output  [7:0]   mem_data_in[0:3];
    output          mem_write_en;
    input           clk;
    input           rst_b;

    parameter start = 0, top = (1<<10) - 1;

    //entry[63:0] = [unused[9:0], valid, dirty, tag[19:0], data[31:0]]
    //mem_addr[31:0] = [tag[19:0], offset[11:0]]
    reg [63:0] mem[start:top];
    wire [9:0] ea = addr[11:2];

    wire [19:0] tag = mem[ea][51:32];
    wire valid = mem[ea][53];
    wire dirty = mem[ea][52];

    assign data_out[0] = mem[ea][7:0];
    assign data_out[1] = mem[ea][15:8];
    assign data_out[2] = mem[ea][23:16];
    assign data_out[3] = mem[ea][31:24];

    assign hit = (valid == 1'b1 && tag == addr[31:12]) ? 1'b1 : 1'b0;

    assign mem_addr = addr;
    
    assign mem_data_in[0] = mem[ea][7:0];
    assign mem_data_in[1] = mem[ea][15:8];
    assign mem_data_in[2] = mem[ea][23:16];
    assign mem_data_in[3] = mem[ea][31:24];

    assign mem_write_en = ((read == 1'b1 || write == 1'b1) && (hit == 1'b0 && dirty == 1'b1) && count == 4'd0) ? 1'b1 : 1'b0;

    reg [3:0] count;

    localparam  IDLE = 2'd0,
                FETCH = 2'd1,
                WRITE_BACK = 2'd2;
    
    reg [1:0] state;
    always_ff @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            integer i;
            count <= 4'd0;
            state <= IDLE;
            for (i = start; i <= top; i++)
                mem[i] <= 0;
        end else begin
            // $display("%h %h %h %h",  addr, {read, write}, mem[ea], state);
            if (state == IDLE) begin
                if (read == 1'b1 && hit == 1'b0) begin
                    count <= 4'd0;
                    if (dirty == 1'b1) begin
                        state <= WRITE_BACK;
                    end else begin
                        state <= FETCH;
                    end
                end else if (write == 1'b1) begin
                    if (hit == 1'b0) begin
                        count <= 4'd0;
                        if (dirty == 1'b1) begin
                            state <= WRITE_BACK;
                        end else begin
                            state <= FETCH;
                        end
                    end else begin
                        mem[ea] <= {10'd0, 1'b1, 1'd1, addr[31:12], data_in[3], data_in[2], data_in[1], data_in[0]};
                    end
                end
            end else if (state == WRITE_BACK) begin
                if (count == 4'd4) begin
                    count <= 4'd0;
                    state <= FETCH;
                end else begin
                    count <= count + 1;
                end
            end else if (state == FETCH) begin
                if (count == 4'd4) begin
                    count <= 4'd0;
                    state <= IDLE;
                    mem[ea] <= {10'd0, 1'b1, 1'd0, addr[31:12], mem_data_out[3], mem_data_out[2], mem_data_out[1], mem_data_out[0]};
                end else begin
                    count <= count + 1;
                end
            end
        end
    end
endmodule