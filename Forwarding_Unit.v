module Forwarding_Unit
(
    ID_EX_Rs1,
    ID_EX_Rs2,
	EX_MEM_Rd,
	EX_MEM_RegWrite,
    MEM_WB_Rd,
	MEM_WB_RegWrite,
    forwardA_o,
	forwardB_o
);


input   [4:0]  ID_EX_Rs1,ID_EX_Rs2,EX_MEM_Rd,MEM_WB_Rd;
input 	EX_MEM_RegWrite,MEM_WB_RegWrite;
output reg  [1:0]  forwardA_o,forwardB_o;

always@(*) begin
	if(EX_MEM_RegWrite==1'b1&&EX_MEM_Rd!=5'b0&&EX_MEM_Rd==ID_EX_Rs1 )  //forwardA_o
		forwardA_o <= 2'b10;
	else if(MEM_WB_RegWrite==1'b1&&MEM_WB_Rd!=5'b0&&MEM_WB_Rd==ID_EX_Rs1)
		forwardA_o <= 2'b01;		
	else 
		forwardA_o <= 2'b00;	
	
	if(EX_MEM_RegWrite==1'b1&&EX_MEM_Rd!=5'b0&&EX_MEM_Rd==ID_EX_Rs2)  //forwardB_o
		forwardB_o <= 2'b10;	
	else if(MEM_WB_RegWrite==1'b1&&MEM_WB_Rd!=5'b0&&MEM_WB_Rd==ID_EX_Rs2)
		forwardB_o <= 2'b01;	
	else 
		forwardB_o <= 2'b00;	
end 

endmodule
