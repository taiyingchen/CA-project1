module IF_ID (instruction_in, instruction_out, IF_ID_Write, IF_Flush, clk, reset);
	// 1. data content
	input [31:0] instruction_in;
	output [31:0] instruction_out;
	// 2. hazard control
	// IF_ID_Write: sync; if (IF_ID_Write==1'b0), do not update content at this rising edge
	// IF_Flush: sync; if (IF_Flush==1), clear ALL content, NOT ONLY control signals
	input IF_ID_Write, IF_Flush;
	// 3. general contorl
	// reset: async; set all register content to 0
	input clk, reset;

	reg [31:0] instruction_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset==1'b1)
		begin
			instruction_out <= 32'b0;
		end
		else if (IF_Flush==1'b1)
		begin
			instruction_out <= 32'b0;
		end
		else if (IF_ID_Write==1'b1)
		begin
			instruction_out <= instruction_in;
		end

	end
	
endmodule
