module regfile(
    rs1_data,
    rs2_data,
    rs1_num,
    rs2_num,
    rd_num,
    rd_data,
    rd_we,
    clk,
    rst_b,
    halted
);
    parameter XLEN=32, size=32;

    output [XLEN-1:0] rs1_data;
    output [XLEN-1:0] rs2_data;
    input       [4:0] rs1_num;
    input       [4:0] rs2_num;
    input       [4:0] rd_num;
    input  [XLEN-1:0] rd_data;
    input             rd_we;
    input             clk;
    input             rst_b;
    input             halted;

    reg [XLEN-1:0] data[0:size-1];

    assign rs1_data = data[rs1_num];
    assign rs2_data = data[rs2_num];

    always_ff @(posedge clk, negedge rst_b) begin
        if (rst_b == 0) begin
            int i;
            for (i = 0; i < size; i++)
                data[i] <= 0;
        end else begin
            if (rd_we && (rd_num != 0))
                data[rd_num] <= rd_data;
        end
    end

	always @(halted) begin
        integer fd = 0;
        integer i = 0;
		if (rst_b && (halted)) begin
			fd = $fopen("output/regdump.reg");

			$display("=== Simulation Cycle %0d ===", $time/2);
			$display("*** RegisterFile dump ***");
			$fdisplay(fd, "*** RegisterFile dump ***");
			
			for(i = 0; i < size; i = i+1) begin
				$display("x%2d = 0x%8x (%0d)", i, data[i], $signed(data[i]));
				$fdisplay(fd, "x%2d = 0x%8h (%0d)", i, data[i], $signed(data[i])); 
			end
			
			$fclose(fd);
		end
	end
    
endmodule
