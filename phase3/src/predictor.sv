module predictor(
    pc,
    pc_update,
    prediction,
    update,
    taken,
    clk,
    rst_b
);

input [31:0] pc;
input [31:0] pc_update;
input update;
input clk;
input rst_b;
input taken;
output prediction;

parameter k = 4;
parameter j = 2;

reg [k-1:0] GHR;
reg [j-1:0] PHT [2**k-1:0];

integer i;

wire [k-1:0] idx, idx2;
assign idx = pc_update[k+1:2] ^ GHR;
assign idx2 = pc[k+1:2] ^ GHR;
assign prediction = PHT[idx2] >= 2**(j-1);
always @(posedge clk, negedge rst_b) begin
    if (rst_b == 0) begin
        GHR <= 2**k-1; 
        for (i = 0; i < 2**k; i = i + 1) begin
            PHT[i] <= 2**(j-1);
        end
    end else begin
        if (update) begin
            if (taken && PHT[idx] != 2**j-1) begin
                PHT[idx] <= PHT[idx] + 1;
            end else if (~taken && PHT[idx] != 0) begin
                PHT[idx] <= PHT[idx] - 1;
            end
            GHR <= {GHR[k-2:0], taken};
        end
    end
end
endmodule