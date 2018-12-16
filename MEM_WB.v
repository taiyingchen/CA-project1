module MEM_WB (RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, D_MEM_read_data_in, D_MEM_read_addr_in, D_MEM_read_data_out, D_MEM_read_addr_out, EX_MEM_RegisterRd_in, MEM_WB_RegisterRd_out, clk/*, reset*/);
	// 1. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 2. data content
	input [31:0] D_MEM_read_data_in, D_MEM_read_addr_in;
	output [31:0] D_MEM_read_data_out, D_MEM_read_addr_out;
	input [4:0] EX_MEM_RegisterRd_in;
	output [4:0] MEM_WB_RegisterRd_out;
	// general signal
	// reset: async; set all register content to 0
	input clk/*, reset*/;
	
	reg RegWrite_out, MemtoReg_out;
	reg [31:0] D_MEM_read_data_out, D_MEM_read_addr_out;
	reg [4:0] MEM_WB_RegisterRd_out;
	
	always @(posedge clk/* or posedge reset*/)
	begin
		/*if (reset == 1'b1)
		begin
			RegWrite_out <= 1'b0;
			MemtoReg_out <= 1'b0;
			D_MEM_read_data_out <= 32'b0;
			D_MEM_read_addr_out <= 32'b0;
			MEM_WB_RegisterRd_out <= 5'b0;
		end
		else */begin
			RegWrite_out <= RegWrite_in;
			MemtoReg_out <= MemtoReg_in;
			D_MEM_read_data_out <= D_MEM_read_data_in;
			D_MEM_read_addr_out <= D_MEM_read_addr_in;
			MEM_WB_RegisterRd_out <= EX_MEM_RegisterRd_in;
		end
		
	end
	
endmodule