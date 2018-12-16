module MUX32_4to1
(
    data1_i,
    data2_i,
	data3_i,
	select_i,
	data4_i,
	ALUSrc,
    data_o
);
input 	ALUSrc;
input   [1:0]	select_i;
input   [31:0]  data1_i,data2_i,data3_i,data4_i;
output reg  [31:0]  data_o;

always@(*) begin
	if(ALUSrc==1)	begin
		data_o <= data4_i;
	end
	else begin
		case(select_i)
			2'b00:data_o<=data1_i;
			2'b01:data_o<=data2_i;
			2'b10:data_o<=data3_i;
			default: data_o<=data1_i;
		endcase
	end
end 

endmodule
