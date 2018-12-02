module Data_Memory
(
    clk_i,
    addr_i, 
    data_i,
    MemRead_i,
    MemWrite_i,
    data_o,
);

// Interface
input   clk_i;
input   [31:0]      addr_i;
input   [31:0]      data_i;
input   MemRead_i;
input   MemWrite_i;
output  [31:0]      data_o;

// Data memory
reg     [7:0]  memory  [0:31]; // total size is 32 Bytes = 1 Byte * 32

/*
Notice: hard code due to only lw and sw instruction
*/
// Addressing by Little Endian
always@(posedge clk_i) begin
    $display("Data_Memmory.addr_i = %d", addr_i);
    $display("Data_Memmory.data_i = %d", data_i);
    $display("Data_Memmory.MemRead_i = %d", MemRead_i);
    $display("Data_Memmory.MemWrite_i = %d", MemWrite_i);
    $display("Data_Memmory.data_o = %d", data_o);
    $display("Data_Memmory.test_o = %d", test_o);

    if (MemWrite_i) begin // sw
        memory[addr_i] <= data_i[7:0];
        memory[addr_i+1] <= data_i[15:8];
        memory[addr_i+2] <= data_i[23:16];
        memory[addr_i+3] <= data_i[31:24];
    end
    if (MemRead_i) begin // lw
        // data_o = {memory[addr_i+3], memory[addr_i+2], memory[addr_i+1], memory[addr_i]};
    end
end

// output reg will delay one clock, use output instead
assign data_o = {memory[addr_i+3], memory[addr_i+2], memory[addr_i+1], memory[addr_i]};

endmodule