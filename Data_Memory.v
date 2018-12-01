module Data_Memory
(
    clk_i,
    addr_i, 
    data_i,
    MemRead_i,
    MemWrite_i,
    data_o
);

// Interface
input   clk_i;
input   [31:0]      addr_i;
input   [31:0]      data_i;
input   MemRead_i;
input   MemWrite_i;
output reg  [31:0]  data_o;

// Data memory
reg     [7:0]  memory  [0:31]; // total size is 32 Bytes = 1 Byte * 32
wire    [31:0]  addr_i_shift;

/*
Notice: hard code due to only lw and sw instruction
*/
// Addressing by Little Endian
always@(posedge clk_i) begin
    if (MemWrite_i) // sw
        memory[addr_i] <= data_i;
    if (MemRead_i) // lw
        data_o <= memory[addr_i];
end

endmodule
