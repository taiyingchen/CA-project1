module Hazard_Detection_Unit
(
    ID_EX_MemRead_i,
    IF_ID_RegisterRs1_i,
    IF_ID_RegisterRs2_i,
    ID_EX_RegisterRd_i,
    PCWrite_o,
    IF_ID_Write_o,
    Flush_o
);

// Interface
input   ID_EX_MemRead_i;
input   [4:0]   IF_ID_RegisterRs1_i;
input   [4:0]   IF_ID_RegisterRs2_i;
input   [4:0]   ID_EX_RegisterRd_i;
output reg  PCWrite_o;
output reg  IF_ID_Write_o;
output reg  Flush_o;

// Detection unit
wire    equal_Rs1;
wire    equal_Rs2;
assign equal_Rs1 = (ID_EX_MemRead_i == IF_ID_RegisterRs1_i) ? 1 : 0;
assign equal_Rs2 = (ID_EX_MemRead_i == IF_ID_RegisterRs2_i) ? 1 : 0;

always@(ID_EX_MemRead_i or equal_Rs1 or equal_Rs2) begin
    // Stall the pipeline and flush instruction in ID stage
    if (ID_EX_MemRead_i & (equal_Rs1 | equal_Rs2)) begin
        PCWrite_o <= 0;
        IF_ID_Write_o <= 0;
        Flush_o <= 1;
    end else begin
        PCWrite_o <= 1;
        IF_ID_Write_o <= 1;
        Flush_o <= 0;
    end
end

endmodule