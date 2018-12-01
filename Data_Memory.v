module Data_Memory
(
    addr_i, 
    data_i,
    MemRead_i,
    MemWrite_i,
    data_o
);

// Interface
input   [31:0]      addr_i;
input   [31:0]      data_i;
input   MemRead_i;
input   MemWrite_i;
output reg  [31:0]  data_o;

// Data memory
/*
Notice: hard code due to only lw and sw instruction
*/
reg     [31:0]  memory  [0:7]; // total size is 32 Bytes = 4 Bytes * 8
wire    [31:0]  addr_i_shift;

assign addr_i_shift = addr_i >> 2;

always@(MemWrite_i or MemRead_i or addr_i or data_i) begin
    if (MemWrite_i) // sw
        memory[addr_i_shift] = data_i;
    if (MemRead_i) // lw
        data_o = memory[addr_i_shift];
end

endmodule
