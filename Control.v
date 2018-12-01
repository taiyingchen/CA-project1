module Control
(
    Op_i,
    //RegDst_o,
    ALUOp_o,
    ALUSrc_o,
    RegWrite_o
);

input	[6:0]	Op_i;
output reg	[1:0]	ALUOp_o;
output reg	ALUSrc_o;
output reg	Branch_o, MemRead_o, MemWrite_o;
output reg	RegWrite_o, MemtoReg_o;

always @ (Op_i) begin
  case(Op_i)
	7'b0110011://R-format
	  begin  
	    ALUOp_o = 2'b10;
	    ALUSrc_o = 0;
	    Branch_o = 0;
	    MemRead_o = 0;
	    MemWrite_o = 0;
	    RegWrite_o = 1;
	    MemtoReg_o = 0;
	  end
	7'b0010011: //i type
	  begin 
	    ALUOp_o = 2'b00;
	    ALUSrc_o = 1;
	    RegWrite_o = 1;
	//others filled with 0 temporarily
	    Branch_o = 0;
	    MemRead_o = 0;
	    MemWrite_o = 0;
	    MemtoReg_o = 0;
	  end
	7'b0000011://ld
	  begin  
	    ALUOp_o = 2'b00;
	    ALUSrc_o = 1;
	    Branch_o = 0;
	    MemRead_o = 1;
	    MemWrite_o = 0;
	    RegWrite_o = 1;
	    MemtoReg_o = 1;
	  end
	7'b0100011://sd
	  begin  
	    ALUOp_o = 2'b00;
	    ALUSrc_o = 1;
	    Branch_o = 0;
	    MemRead_o = 0;
	    MemWrite_o = 1;
	    RegWrite_o = 0;
	    MemtoReg_o = 1'bx;
	  end
	7'b1100011://sd
	  begin  
	    ALUOp_o = 2'b01;
	    ALUSrc_o = 0;
	    Branch_o = 1;
	    MemRead_o = 0;
	    MemWrite_o = 0;
	    RegWrite_o = 0;
	    MemtoReg_o = 1'bx;
	  end
  	default : $display("Error in Control."); 
  endcase
end

endmodule
