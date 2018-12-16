module EX_MEM (EX_Flush, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, MemRead_in, MemWrite_in, Jump_in, MemRead_out, MemWrite_out, Jump_out, jump_addr_in, branch_addr_in, jump_addr_out, branch_addr_out, ALU_zero_in, ALU_zero_out, ALU_result_in, reg_read_data_2_in, ALU_result_out, reg_read_data_2_out, ID_EX_RegisterRd_in, EX_MEM_RegisterRd_out, clk, reset);
	// 1. hazard control signal (sync rising edge)
	// if EX_Flush equals 1,
	// then clear all WB, MEM control signal to 0 on rising edge
	// do not need to clear addr or data content
	input EX_Flush;
	// 2. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 3. MEM control signal
	input MemRead_in, MemWrite_in;
	output MemRead_out, MemWrite_out;
	// 4. data content
	input ALU_zero_in;
	output ALU_zero_out;
	input [31:0] ALU_result_in, reg_read_data_2_in;
	output [31:0] ALU_result_out, reg_read_data_2_out;
	input [4:0] ID_EX_RegisterRd_in;
	output [4:0] EX_MEM_RegisterRd_out;
	// general signal
	// reset: async; set all register content to 0
	input clk, reset;

	reg RegWrite_out, MemtoReg_out;
	reg Branch_out, MemRead_out, MemWrite_out;
	reg [31:0] branch_addr_out;
	reg ALU_zero_out;
	reg [31:0] ALU_result_out, reg_read_data_2_out;
	reg [4:0] EX_MEM_RegisterRd_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  // Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		  ALU_zero_out <= 1'b0;
		  ALU_result_out <= 32'b0;
		  reg_read_data_2_out <= 32'b0;
		  EX_MEM_RegisterRd_out <= 5'b0; 
		end
		else if (EX_Flush == 1'b1)
	    begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  // Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		end
		else begin
		  RegWrite_out <= RegWrite_in;
		  MemtoReg_out <= MemtoReg_in;
		  // Branch_out <= Branch_in;
		  MemRead_out <= MemRead_in;
		  MemWrite_out <= MemWrite_in;
		  ALU_zero_out <= ALU_zero_in;
		  ALU_result_out <= ALU_result_in;
		  reg_read_data_2_out <= reg_read_data_2_in;
		  EX_MEM_RegisterRd_out <= ID_EX_RegisterRd_in;
		end

	end

endmodule