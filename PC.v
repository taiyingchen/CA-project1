module PC
(
    clk_i,
    // rst_i,
    start_i,
    PCWrite_i,
    pc_i,
    pc_o
);

// Ports
input               clk_i;
// input               rst_i;
input               start_i;
input	PCWrite_i;
input   [31:0]      pc_i;
output  [31:0]      pc_o;

// Wires & Registers
reg     [31:0]      pc_o;


always@(posedge clk_i) begin
    // Use start signal to replace reset signal
    if(~start_i)
        pc_o <= 32'b0;
    else if (~PCWrite_i)
	pc_o <= pc_o;
    else begin
        if(start_i)
            pc_o <= pc_i;
        else
            pc_o <= pc_o;
    end
end

endmodule
