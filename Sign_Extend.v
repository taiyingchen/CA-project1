module Sign_Extend
(
    data_i,
    data_o
);

input   [31:0]  data_i;
output	reg	[31:0]  data_o;
reg	[6:0]	Op_i;

always @ (data_i) begin
    Op_i = data_i[6:0];
  case(Op_i)
	7'b0110011://R-format  
	    data_o = {31{1'bx}};//don't care
	7'b0010011: //i type
	  begin 
	    data_o = {{20{data_i[11]}}, data_i};
	  end
	7'b0000011://ld
	    data_o = {{20{data_i[31]}}, data_i[31:25], data_i[11:7]};
	7'b0100011://sd
	    data_o = {{20{data_i[31]}}, data_i[31:25], data_i[11:7]};
	7'b1100011://beq
	    data_o = {{20{data_i[31]}}, data_i[31], data_i[7], data_i[30:25], data_i[11:8]};
  	default : $display("Error in Sign_Extend."); 
  endcase
end

endmodule
