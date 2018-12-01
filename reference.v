
// pipeline.v
// ver 10.0 (demo)
// VE370 Project #2

// Due: 
// Report & Source code due Nov 27 23:00 through Sakai
// Demonstration is due by 4:00pm, November 27, 2015

// Created by Hua Xing, Junqi Qian, Xinyue Ou on 11/12/15
// Copyright (c) 2015. All rights reserved.

// Version History
// ver 1.0: top level schematics ready
// ver 2.0: added clock-divider and SSD output. SSD outputs PC or reg file content.
// ver 3.0: combine the v2 and the proof reading result.
	// major changes:
	// 1. change the bus width of some wires.
	// 2. add a new wire ID_EX_funct, and hence change the I/O of the stage register of ID_EX.
	// 3. change the connection of the 3to1 mux to make the structure 
	// of the connection parallel (and hence easier to implement forwarding unit).
	// 4. implement the modules other than the stage registers.
// ver 4.0. FPGA implementation complete.
// ver 5.0. Initial simulation code ready. But buggy.
// ver 6.0. Modified clock dividers for simulation. Simulation complete.
// ver 7.0: expanded PC width to 7 bits (5 bit + 2 bit offset). I-MEM now supports 2^5 = 32 lines.
// ver 8.0: I-MEM loads 32 lines of instructions
// ver 9.0: parallel with ver 8.0, only with FPGA implementaion structure
// ver 10.0: update the alu control to accomodate andi; this is a version for demostration

module pipeline (clkFast, reset, SwitchSelector, switchRun, Cathode, AN, LEDIndicator);
	input clkFast, reset;		// clkFast (5m Hz) feeds clock divider
	input switchRun;			// decide run(1) or check reg file(0)
	input [4:0] SwitchSelector;	// select $0 to $31 for output
	output [6:0] Cathode;		// SSD
	output [3:0] AN;
	output LEDIndicator;		// output clkNormal for user reference

	// wires in IF stage
	wire [31:0] PC_in_original;
	wire [6:0] PC_out_short;
	wire [31:0] PC_out_unsign_extended, PC_plus4;
	wire [31:0] instruction;
	wire [31:0] branch_jump_addr;
	// wires in ID stage
	wire [31:0] IF_ID_PC_plus4, IF_ID_instruction;
	wire [4:0] MEM_WB_RegisterRd;
	wire [31:0] reg_read_data_1, reg_read_data_2;

	wire [31:0] immi_sign_extended;
		// jump within ID stage
	wire [31:0] jump_addr;
	wire [27:0] jump_base28;
		// control signal generation within ID stage
	wire RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
	wire [1:0] ALUOp;
	// wires in EX stage
	wire ID_EX_RegDst, ID_EX_Jump, ID_EX_Branch, ID_EX_MemRead, ID_EX_MemtoReg, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_RegWrite;
	wire [1:0] ID_EX_ALUOp;
	wire [31:0] ID_EX_jump_addr;
	wire [31:0] ID_EX_PC_plus4, ID_EX_reg_read_data_1, ID_EX_reg_read_data_2;
	wire [31:0] ID_EX_immi_sign_extended;
	wire [4:0] ID_EX_RegisterRs, ID_EX_RegisterRt, ID_EX_RegisterRd;// Ou modifies: [31:0]
	wire [4:0] EX_RegisterRd;
	wire [5:0] ID_EX_funct;
	wire [3:0] out_to_ALU;
	wire [31:0] muxA_out, muxB_out;
	wire [31:0] after_ALUSrc;
	wire [31:0] ALU_result;
	wire ALU_zero;
	wire [31:0] after_shift, branch_addr;
	// wires in MEM stage
	wire [4:0] EX_MEM_RegisterRd;
	wire EX_MEM_RegWrite, EX_MEM_MemtoReg, EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_Jump;
	wire [31:0] EX_MEM_jump_addr, EX_MEM_branch_addr;
	wire EX_MEM_ALU_zero;
	wire [31:0] EX_MEM_ALU_result, EX_MEM_reg_read_data_2;
	wire [31:0] D_MEM_data;
	wire Branch_taken;
	// wires in WB stage
	wire [31:0] reg_write_data;
	wire MEM_WB_RegWrite, MEM_WB_MemtoReg;
	wire [31:0] MEM_WB_D_MEM_read_data, MEM_WB_D_MEM_read_addr;
	// wires for forwarding
	wire [1:0] ForwardA, ForwardB;
	// wires for lw hazard stall
	wire PCWrite;					// PC stops writing if PCWrite == 0
	wire IF_ID_Write;				// IF/ID reg stops writing if IF_ID_Write == 0
	wire ID_Flush_lwstall;
	// wires for jump/branch control hazard
	wire PCSrc;
	wire IF_Flush, ID_Flush_Branch, EX_Flush;

	// SSD display & clock slow-down
	wire clkSSD, clkNormal, clkRF, clk;
		// clkFast: 5m Hz
		// clkSSD: 500 Hz for ring counter
		// clkNormal: 1 Hz
    wire  [3:0] tho; // Binary-Coded-Decimal 0-15
	wire  [3:0] hun;
	wire  [3:0] ten;
	wire  [3:0] one;
    wire  [6:0] thossd;
	wire  [6:0] hunssd;
	wire  [6:0] tenssd;
	wire  [6:0] onessd;	
	// multi-purpose I-MEM read_addr_1
	wire [4:0] multi_purpose_read_addr;
	wire multi_purpose_RegWrite;

	// reg to resolve always block technicalities
	reg clkRF_reg, clk_reg, multi_purpose_RegWrite_reg;
	reg [4:0] multi_purpose_read_addr_reg;
	reg [3:0] tho_reg, hun_reg, ten_reg, one_reg;		

   //new forward, change id_ex stage register
	wire ForwardC,ForwardD;
	wire [31:0] muxC_out,muxD_out;
	Mux_N_bit #(32) Unit26 (.in0(reg_read_data_1),.in1(reg_write_data),.mux_out(muxC_out),.control(ForwardC));
	Mux_N_bit #(32) Unit27 (.in0(reg_read_data_2),.in1(reg_write_data),.mux_out(muxD_out),.control(ForwardD));


	// output processor clock (1 Hz or freeze) to a LED
	assign LEDIndicator = clk;
	// IF stage
	Program_Counter Unit0 (.clk(clk), .reset(reset), .PC_in(PC_in_original[6:0]), .PC_out(PC_out_short), .PCWrite(PCWrite));
	Instruction_Memory Unit1 (.read_addr(PC_out_short), .instruction(instruction), .reset(reset));
	assign PC_out_unsign_extended = {26'b0000_0000_0000_0000_0000_0000_0, PC_out_short}; // from 8 bits to 32 bits
	ALU_add_only Unit2 (.inA(PC_out_unsign_extended), .inB(32'b0100), .add_out(PC_plus4)); // PC + 4
	Mux_N_bit #(32) Unit3 (.in0(PC_plus4), .in1(branch_jump_addr), .mux_out(PC_in_original), .control(PCSrc));
	IF_ID_Stage_Reg Unit4 (.PC_plus4_in(PC_plus4), .PC_plus4_out(IF_ID_PC_plus4), .instruction_in(instruction), .instruction_out(IF_ID_instruction), .IF_ID_Write(IF_ID_Write), .IF_Flush(IF_Flush), .clk(clk), .reset(reset));
	// ID stage
	Register_File Unit5 (.read_addr_1(multi_purpose_read_addr), .read_addr_2(IF_ID_instruction[20:16]), .write_addr(MEM_WB_RegisterRd), .read_data_1(reg_read_data_1), .read_data_2(reg_read_data_2), .write_data(reg_write_data), .RegWrite(multi_purpose_RegWrite), .clk(clkRF), .reset(reset));
	Sign_Extension Unit6 (.sign_in(IF_ID_instruction[15:0]), .sign_out(immi_sign_extended));
		// jump within ID stage
	Shift_Left_2_Jump Unit7 (.shift_in(IF_ID_instruction[25:0]), .shift_out(jump_base28));
	assign jump_addr = {IF_ID_PC_plus4[31:28], jump_base28}; // jump_addr = (PC+4)[31:28] joined with jump_base28[27:0]
	Control Unit8 (.OpCode(IF_ID_instruction[31:26]), .RegDst(RegDst), .Jump(Jump), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
	ID_EX_Stage_Reg Unit9 (.ID_Flush_lwstall(ID_Flush_lwstall), .ID_Flush_Branch(ID_Flush_Branch), .RegWrite_in(RegWrite), .MemtoReg_in(MemtoReg), .RegWrite_out(ID_EX_RegWrite), .MemtoReg_out(ID_EX_MemtoReg), .Branch_in(Branch), .MemRead_in(MemRead), .MemWrite_in(MemWrite), .Jump_in(Jump), .Branch_out(ID_EX_Branch), .MemRead_out(ID_EX_MemRead), 
	.MemWrite_out(ID_EX_MemWrite), .Jump_out(ID_EX_Jump), .RegDst_in(RegDst), .ALUSrc_in(ALUSrc), .RegDst_out(ID_EX_RegDst), .ALUSrc_out(ID_EX_ALUSrc), .ALUOp_in(ALUOp), .ALUOp_out(ID_EX_ALUOp), .jump_addr_in(jump_addr), .PC_plus4_in(IF_ID_PC_plus4), .jump_addr_out(ID_EX_jump_addr), .PC_plus4_out(ID_EX_PC_plus4),
	.reg_read_data_1_in(muxC_out), .reg_read_data_2_in(muxD_out), .immi_sign_extended_in(immi_sign_extended), .reg_read_data_1_out(ID_EX_reg_read_data_1), .reg_read_data_2_out(ID_EX_reg_read_data_2), .immi_sign_extended_out(ID_EX_immi_sign_extended), .IF_ID_RegisterRs_in(IF_ID_instruction[25:21]), 
	.IF_ID_RegisterRt_in(IF_ID_instruction[20:16]), .IF_ID_RegisterRd_in(IF_ID_instruction[15:11]), .IF_ID_RegisterRs_out(ID_EX_RegisterRs), .IF_ID_RegisterRt_out(ID_EX_RegisterRt), .IF_ID_RegisterRd_out(ID_EX_RegisterRd), .IF_ID_funct_in(IF_ID_instruction[5:0]),.IF_ID_funct_out(ID_EX_funct),.clk(clk), .reset(reset));//Ou adds  IF_ID_funct_in and out
	// EX stage
	Mux_N_bit #(5) Unit10 (.in0(ID_EX_RegisterRt), .in1(ID_EX_RegisterRd), .mux_out(EX_RegisterRd), .control(ID_EX_RegDst));
	ALUControl Unit11 (.ALUOp(ID_EX_ALUOp), .funct(ID_EX_funct), .out_to_ALU(out_to_ALU));// Ou modifies: funct should not be IF_ID_instruction. Rather, it should be ID_EX_funct(a new wire)
	Mux_32bit_3to1 Unit12_muxA (.in00(ID_EX_reg_read_data_1), .in01(reg_write_data), .in10(EX_MEM_ALU_result), .mux_out(muxA_out), .control(ForwardA));
	Mux_32bit_3to1 Unit13_muxB (.in00(ID_EX_reg_read_data_2), .in01(reg_write_data), .in10(EX_MEM_ALU_result), .mux_out(muxB_out), .control(ForwardB));//Ou modifies: keep the structure paralleled with muxA
	Mux_N_bit #(32) Unit14 (.in0(muxB_out), .in1(ID_EX_immi_sign_extended), .mux_out(after_ALUSrc), .control(ID_EX_ALUSrc));
	ALU Unit15 (.inA(muxA_out), .inB(after_ALUSrc), .alu_out(ALU_result), .zero(ALU_zero), .control(out_to_ALU));
	Shift_Left_2_Branch Unit16 (.shift_in(ID_EX_immi_sign_extended), .shift_out(after_shift));
	ALU_add_only Unit17 (.inA(ID_EX_PC_plus4), .inB(after_shift), .add_out(branch_addr)); // (PC+4) + branch_addition*4Z
	// in EX/MEM stage reg, note muxB_out is used in the place of reg_read_data_2 as a result of forwarding;ygb 
	EX_MEM_Stage_Reg Unit18 (.EX_Flush(EX_Flush), .RegWrite_in(ID_EX_RegWrite), .MemtoReg_in(ID_EX_MemtoReg), .RegWrite_out(EX_MEM_RegWrite), .MemtoReg_out(EX_MEM_MemtoReg),
	.Branch_in(ID_EX_Branch), .MemRead_in(ID_EX_MemRead), .MemWrite_in(ID_EX_MemWrite),
	.Jump_in(ID_EX_Jump), .Branch_out(EX_MEM_Branch), .MemRead_out(EX_MEM_MemRead),
	.MemWrite_out(EX_MEM_MemWrite), .Jump_out(EX_MEM_Jump), .jump_addr_in(ID_EX_jump_addr), 
	.branch_addr_in(branch_addr), .jump_addr_out(EX_MEM_jump_addr), 
	.branch_addr_out(EX_MEM_branch_addr), .ALU_zero_in(ALU_zero),
	.ALU_zero_out(EX_MEM_ALU_zero), .ALU_result_in(ALU_result), .reg_read_data_2_in(muxB_out), 
	.ALU_result_out(EX_MEM_ALU_result), .reg_read_data_2_out(EX_MEM_reg_read_data_2), 
	.ID_EX_RegisterRd_in(EX_RegisterRd), .EX_MEM_RegisterRd_out(EX_MEM_RegisterRd), .clk(clk), .reset(reset));
	// MEM stage
	Data_Memory Unit19 (.addr(EX_MEM_ALU_result[7:0]), .write_data(EX_MEM_reg_read_data_2), .read_data(D_MEM_data), .clk(clk), .reset(reset), .MemRead(EX_MEM_MemRead), .MemWrite(EX_MEM_MemWrite));
	and (Branch_taken, EX_MEM_Branch, EX_MEM_ALU_zero);
	jump_OR_branch Unit20 (.Jump(EX_MEM_Jump), .Branch_taken(Branch_taken), .branch_addr(EX_MEM_branch_addr), .jump_addr(EX_MEM_jump_addr), .PCSrc(PCSrc), .addr_out(branch_jump_addr));
	//WB stage
	MEM_WB_Stage_Reg Unit21 (.RegWrite_in(EX_MEM_RegWrite), .MemtoReg_in(EX_MEM_MemtoReg), .RegWrite_out(MEM_WB_RegWrite), .MemtoReg_out(MEM_WB_MemtoReg), .D_MEM_read_data_in(D_MEM_data), .D_MEM_read_addr_in(EX_MEM_ALU_result), .D_MEM_read_data_out(MEM_WB_D_MEM_read_data), 
	.D_MEM_read_addr_out(MEM_WB_D_MEM_read_addr), .EX_MEM_RegisterRd_in(EX_MEM_RegisterRd), .MEM_WB_RegisterRd_out(MEM_WB_RegisterRd), .clk(clk), .reset(reset));
	Mux_N_bit #(32) Unit22 (.in0(MEM_WB_D_MEM_read_addr), .in1(MEM_WB_D_MEM_read_data), .mux_out(reg_write_data), .control(MEM_WB_MemtoReg));
	Forwarding_Control Utni23 (.EX_MEM_RegisterRd(EX_MEM_RegisterRd), .MEM_WB_RegisterRd(MEM_WB_RegisterRd), .ID_EX_RegisterRs(ID_EX_RegisterRs), .ID_EX_RegisterRt(ID_EX_RegisterRt), .EX_MEM_RegWrite(EX_MEM_RegWrite), .MEM_WB_RegWrite(MEM_WB_RegWrite), .IF_ID_RegisterRs(IF_ID_instruction[25:21]),.IF_ID_RegisterRt(IF_ID_instruction[20:16]),.ForwardA(ForwardA), .ForwardB(ForwardB),.ForwardC(ForwardC), .ForwardD(ForwardD));
	stall_for_lw_Control Unit24 (.ID_EX_RegisterRt(ID_EX_RegisterRt), .IF_ID_RegisterRs(IF_ID_instruction[25:21]), .IF_ID_RegisterRt(IF_ID_instruction[20:16]), .ID_EX_MemRead(ID_EX_MemRead), .PCWrite(PCWrite), .IF_ID_Write(IF_ID_Write), .ID_Flush_lwstall(ID_Flush_lwstall));
	branch_and_jump_hazard_control Unit25 (.MEM_PCSrc(PCSrc), .IF_Flush(IF_Flush), .ID_Flush_Branch(ID_Flush_Branch), .EX_Flush(EX_Flush));
	// SSD Display
	divide_by_100k Unit_Clock500HZ (.clock(clkFast), .reset(reset), .clock_out(clkSSD));
	divide_by_500  Unit_Clock1HZ (.clock(clkSSD), .reset(reset), .clock_out(clkNormal));
	Ring_4_counter Unit_Ring_Counter (.clock(clkSSD), .reset(reset), .Q(AN));
	ssd_driver	Unit_SSDTHO (.in_BCD(tho), .out_SSD(thossd));
	ssd_driver	Unit_SSDHUN (.in_BCD(hun), .out_SSD(hunssd));
	ssd_driver	Unit_SSDTEN (.in_BCD(ten), .out_SSD(tenssd));
	ssd_driver	Unit_SSDONE (.in_BCD(one), .out_SSD(onessd));
	choose_chathode Unit_CHOOSE (.tho(thossd), .hun(hunssd), .ten(tenssd), .one(onessd), .AN(AN), .CA(Cathode));

	assign clkRF = clkRF_reg;
	assign clk = clk_reg;
	assign multi_purpose_read_addr = multi_purpose_read_addr_reg;
	assign multi_purpose_RegWrite = multi_purpose_RegWrite_reg;
	assign tho = tho_reg;
	assign hun = hun_reg;
	assign ten = ten_reg;
	assign one = one_reg;

	always @(switchRun or clkSSD) begin
		if (switchRun) begin
			// sys status 1: run pipeline processor
			clkRF_reg <= clkNormal;		// 1 Hz
			clk_reg <= clkNormal;		// 1 Hz
			multi_purpose_read_addr_reg <= IF_ID_instruction[25:21]; // reg-file-port1 reads from instruction
			// reg-file protection measure; explained in "else"
			multi_purpose_RegWrite_reg <= MEM_WB_RegWrite;
			// output PC to SSD, but since PC only has 6 bits
			tho_reg <= PC_out_unsign_extended[15:12];	// always 0
			hun_reg <= PC_out_unsign_extended[11:8];	// always 0
			ten_reg <= PC_out_unsign_extended[7:4];
			one_reg <= PC_out_unsign_extended[3:0];
		end
		else begin
			// sys status 2: pause processor; inspect Reg File content
			clkRF_reg <= clkSSD;	// 500 Hz
			clk_reg <= 1'b0;		// freeze at 0
			multi_purpose_read_addr_reg <= SwitchSelector; // reg-file-port1 reads from SwitchSelector
			// Reg-file is not freezed in time, this protects against RF-data-overwrite
			multi_purpose_RegWrite_reg <= 1'b0;
			// output reg file content to SSD, but only the lower 16 bits (we only have 4 SSD)
			tho_reg <= reg_read_data_1[15:12];
			hun_reg <= reg_read_data_1[11:8];
			ten_reg <= reg_read_data_1[7:4];
			one_reg <= reg_read_data_1[3:0];
		end
	end

endmodule

// IF/ID stage register
// update content & output updated content at rising edge
module IF_ID_Stage_Reg (PC_plus4_in, PC_plus4_out, instruction_in, instruction_out, IF_ID_Write, IF_Flush, clk, reset);
	// 1. data content
	input [31:0] PC_plus4_in, instruction_in;
	output [31:0] PC_plus4_out, instruction_out;
	// 2. hazard control
	// IF_ID_Write: sync; if (IF_ID_Write==1'b0), do not update content at this rising edge
	// IF_Flush: sync; if (IF_Flush==1), clear ALL content, NOT ONLY control signals
	input IF_ID_Write, IF_Flush;
	// 3. general contorl
	// reset: async; set all register content to 0
	input clk, reset;

	reg [31:0] PC_plus4_out, instruction_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset==1'b1)
		begin
			PC_plus4_out <= 32'b0;
			instruction_out <= 32'b0;
		end
		else if (IF_Flush==1'b1)
		begin
			PC_plus4_out <= 32'b0;
			instruction_out <= 32'b0;
		end
		else if (IF_ID_Write==1'b1)
		begin
			PC_plus4_out <= PC_plus4_in;
			instruction_out <= instruction_in;
		end

	end
	
endmodule


// ID/EX stage register
// update content & output updated content at rising edge
module ID_EX_Stage_Reg (ID_Flush_lwstall, ID_Flush_Branch, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, Branch_in, MemRead_in, MemWrite_in, Jump_in, Branch_out, MemRead_out, MemWrite_out, Jump_out, RegDst_in, ALUSrc_in, RegDst_out, ALUSrc_out, ALUOp_in, ALUOp_out, jump_addr_in, PC_plus4_in, jump_addr_out, PC_plus4_out, reg_read_data_1_in, reg_read_data_2_in, immi_sign_extended_in, reg_read_data_1_out, reg_read_data_2_out, immi_sign_extended_out, IF_ID_RegisterRs_in, IF_ID_RegisterRt_in, IF_ID_RegisterRd_in, IF_ID_RegisterRs_out, IF_ID_RegisterRt_out, IF_ID_RegisterRd_out,IF_ID_funct_in, IF_ID_funct_out,clk, reset);
	// 1. hazard control signal (sync rising edge)
	// if either ID_Flush_lwstall or ID_Flush_Branch equals 1,
	// then clear all WB, MEM and EX control signal to 0 on rising edge
	// do not need to clear addr, data or reg content
	input ID_Flush_lwstall, ID_Flush_Branch;
	// 2. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 3. MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// 4. EX control signal
	input RegDst_in, ALUSrc_in;
	input [1:0] ALUOp_in;
	output RegDst_out, ALUSrc_out;
	output [1:0] ALUOp_out;
	// 5. addr content
	input [31:0] jump_addr_in, PC_plus4_in;
	output [31:0] jump_addr_out, PC_plus4_out;
	// 6. data content
	input [31:0] reg_read_data_1_in, reg_read_data_2_in, immi_sign_extended_in;
	output [31:0] reg_read_data_1_out, reg_read_data_2_out, immi_sign_extended_out;
	// 7. reg content
	input [4:0] IF_ID_RegisterRs_in, IF_ID_RegisterRt_in, IF_ID_RegisterRd_in;
	output [4:0] IF_ID_RegisterRs_out, IF_ID_RegisterRt_out, IF_ID_RegisterRd_out;
	input [5:0] IF_ID_funct_in;
	output [5:0] IF_ID_funct_out;
	// general signal
	// reset: async; set all register content to 0
	input clk, reset;
	
	reg RegWrite_out, MemtoReg_out;
	reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	reg RegDst_out, ALUSrc_out;
	reg [1:0] ALUOp_out;
	reg [31:0] jump_addr_out, PC_plus4_out;
	reg [31:0] reg_read_data_1_out, reg_read_data_2_out, immi_sign_extended_out;
	reg [4:0] IF_ID_RegisterRs_out, IF_ID_RegisterRt_out, IF_ID_RegisterRd_out;
	reg [5:0] IF_ID_funct_out;
	
	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
			jump_addr_out = 32'b0;
			PC_plus4_out = 32'b0;
			reg_read_data_1_out = 32'b0;
			reg_read_data_2_out = 32'b0;
			immi_sign_extended_out = 32'b0;
			IF_ID_RegisterRs_out = 5'b0;
			IF_ID_RegisterRt_out = 5'b0;
			IF_ID_RegisterRd_out = 5'b0;
			IF_ID_funct_out = 6'b0;			
		end
		else if (ID_Flush_lwstall == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
		end
		else if (ID_Flush_Branch == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
		end
		else begin
			RegWrite_out = RegWrite_in;
			MemtoReg_out = MemtoReg_in;
			Branch_out = Branch_in;
			MemRead_out = MemRead_in;
			MemWrite_out = MemWrite_in;
			Jump_out = Jump_in;
			RegDst_out = RegDst_in;
			ALUSrc_out = ALUSrc_in;
			ALUOp_out = ALUOp_in;
			jump_addr_out = jump_addr_in;
			PC_plus4_out = PC_plus4_in;
			reg_read_data_1_out = reg_read_data_1_in;
			reg_read_data_2_out = reg_read_data_2_in;
			immi_sign_extended_out = immi_sign_extended_in;
			IF_ID_RegisterRs_out = IF_ID_RegisterRs_in;
			IF_ID_RegisterRt_out = IF_ID_RegisterRt_in;
			IF_ID_RegisterRd_out = IF_ID_RegisterRd_in;
			IF_ID_funct_out = IF_ID_funct_in;
		end	
		
	end	
	
endmodule

// EX/MEM stage register
// update content & output updated content at rising edge
module EX_MEM_Stage_Reg (EX_Flush, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, Branch_in, MemRead_in, MemWrite_in, Jump_in, Branch_out, MemRead_out, MemWrite_out, Jump_out, jump_addr_in, branch_addr_in, jump_addr_out, branch_addr_out, ALU_zero_in, ALU_zero_out, ALU_result_in, reg_read_data_2_in, ALU_result_out, reg_read_data_2_out, ID_EX_RegisterRd_in, EX_MEM_RegisterRd_out, clk, reset);
	// 1. hazard control signal (sync rising edge)
	// if EX_Flush equals 1,
	// then clear all WB, MEM control signal to 0 on rising edge
	// do not need to clear addr or data content
	input EX_Flush;
	// 2. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 3. MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// 4. addr content
	input [31:0] jump_addr_in, branch_addr_in;
	output [31:0] jump_addr_out, branch_addr_out;
	// 5. data content
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
	reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	reg [31:0] jump_addr_out, branch_addr_out;
	reg ALU_zero_out;
	reg [31:0] ALU_result_out, reg_read_data_2_out;
	reg [4:0] EX_MEM_RegisterRd_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		  Jump_out <= 1'b0;
		  jump_addr_out <= 32'b0;
		  branch_addr_out <= 32'b0;
		  ALU_zero_out <= 1'b0;
		  ALU_result_out <= 32'b0;
		  reg_read_data_2_out <= 32'b0;
		  EX_MEM_RegisterRd_out <= 5'b0; 
		end
		else if (EX_Flush == 1'b1)
	    begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		  Jump_out <= 1'b0;
		end
		else begin
		  RegWrite_out <= RegWrite_in;
		  MemtoReg_out <= MemtoReg_in;
		  Branch_out <= Branch_in;
		  MemRead_out <= MemRead_in;
		  MemWrite_out <= MemWrite_in;
		  Jump_out <= Jump_in;
		  jump_addr_out <= jump_addr_in;
		  branch_addr_out <= branch_addr_in;
		  ALU_zero_out <= ALU_zero_in;
		  ALU_result_out <= ALU_result_in;
		  reg_read_data_2_out <= reg_read_data_2_in;
		  EX_MEM_RegisterRd_out <= ID_EX_RegisterRd_in;
		end

	end

endmodule

// MEM/WB stage register
// update content & output updated content at rising edge
module MEM_WB_Stage_Reg (RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, D_MEM_read_data_in, D_MEM_read_addr_in, D_MEM_read_data_out, D_MEM_read_addr_out, EX_MEM_RegisterRd_in, MEM_WB_RegisterRd_out, clk, reset);
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
	input clk, reset;
	
	reg RegWrite_out, MemtoReg_out;
	reg [31:0] D_MEM_read_data_out, D_MEM_read_addr_out;
	reg [4:0] MEM_WB_RegisterRd_out;
	
	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
			RegWrite_out <= 1'b0;
			MemtoReg_out <= 1'b0;
			D_MEM_read_data_out <= 32'b0;
			D_MEM_read_addr_out <= 32'b0;
			MEM_WB_RegisterRd_out <= 5'b0;
		end
		else begin
			RegWrite_out <= RegWrite_in;
			MemtoReg_out <= MemtoReg_in;
			D_MEM_read_data_out <= D_MEM_read_data_in;
			D_MEM_read_addr_out <= D_MEM_read_addr_in;
			MEM_WB_RegisterRd_out <= EX_MEM_RegisterRd_in;
		end
		
	end
	
endmodule

// forwarding unit
// all connecions are aynchronous; no clock signal is provided
// implement forwarding from EX stage as shown in "EX forward" attached in email
// implement forwarding from MEM stage as shown in "MEM forward" attached in email
//		note the MEM forward is modified according to Zheng Gang's lecture
// module I/O was implemented according to Fig 4.56, attached in email as "4.56 forwarding diagram"
module Forwarding_Control (EX_MEM_RegisterRd, MEM_WB_RegisterRd, ID_EX_RegisterRs, ID_EX_RegisterRt, EX_MEM_RegWrite, MEM_WB_RegWrite,IF_ID_RegisterRs,IF_ID_RegisterRt, ForwardA, ForwardB, ForwardC, ForwardD);
	input [4:0] EX_MEM_RegisterRd, MEM_WB_RegisterRd, ID_EX_RegisterRs, ID_EX_RegisterRt,IF_ID_RegisterRs,IF_ID_RegisterRt;
	input EX_MEM_RegWrite, MEM_WB_RegWrite;
	output ForwardC,ForwardD;
	output [1:0] ForwardA, ForwardB;
	reg [1:0] ForwardA, ForwardB;
	reg ForwardC,ForwardD;
	wire equal_EXMEM_rs,equal_EXMEM_rt,equal_MEMWB_rs,equal_MEMWB_rt;
	wire nonzero_EXMEM_rd,nonzero_MEMWB_rd;
	assign nonzero_EXMEM_rd=(EX_MEM_RegisterRd==0)?0:1;
	assign nonzero_MEMWB_rd=(MEM_WB_RegisterRd==0)?0:1;
	assign equal_EXMEM_rs=(EX_MEM_RegisterRd==ID_EX_RegisterRs)?1:0;
	assign equal_EXMEM_rt=(EX_MEM_RegisterRd==ID_EX_RegisterRt)?1:0;
	assign equal_MEMWB_rs=(MEM_WB_RegisterRd==ID_EX_RegisterRs)?1:0;
	assign equal_MEMWB_rt=(MEM_WB_RegisterRd==ID_EX_RegisterRt)?1:0;
	assign equal_WB_ID_rs=(MEM_WB_RegisterRd==IF_ID_RegisterRs)?1:0;
	assign equal_WB_ID_rt=(MEM_WB_RegisterRd==IF_ID_RegisterRt)?1:0;
	always@ (EX_MEM_RegWrite or MEM_WB_RegWrite or nonzero_EXMEM_rd or nonzero_MEMWB_rd or equal_EXMEM_rs
	or equal_EXMEM_rt or equal_MEMWB_rs or equal_MEMWB_rt or equal_WB_ID_rs or equal_WB_ID_rt)
	begin
		if(EX_MEM_RegWrite & nonzero_EXMEM_rd & equal_EXMEM_rs)
			ForwardA<=2'b10;
		else if (MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_MEMWB_rs)
			ForwardA<=2'b01;
		else 
			ForwardA<=2'b00;
			
		if(EX_MEM_RegWrite & nonzero_EXMEM_rd & equal_EXMEM_rt)
			ForwardB<=2'b10;
		else if (MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_MEMWB_rt)
			ForwardB<=2'b01;
		else 
			ForwardB<=2'b00;
			
		if(MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_WB_ID_rs)
			ForwardC<=1;
		else
			ForwardC<=0;
			
      if(MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_WB_ID_rt)
			ForwardD<=1;
		else
			ForwardD<=0;
		
	end
endmodule

// load word causes pipeline with EX/MEM -> EX forwarding scheme to stall for ONE cycle
// all connections acynchronous; no clock signal is provided.
// In this module we are concerned with outputting 3 stall signals (don't care about the ONE cycle).
// at next rising edge, ID_EX_MemRead will be flushed to 0, this module will automatically stop stalling.
//
// stalling condition could be found in "stall_for_lw_Control code" attached in email
// how to stall the pipeline:
//		1. set PCWrite and IF_ID_Write to 0
//		2. set ID_Flush_lwstall to 1 (modified from textbook MUX solution; now we feed this signal to ID/EX stage register)
// IMPORTANT: if stalling condition is not met, set all 3 signals to the opposite value!
module stall_for_lw_Control (ID_EX_RegisterRt, IF_ID_RegisterRs, IF_ID_RegisterRt, ID_EX_MemRead, PCWrite, IF_ID_Write, ID_Flush_lwstall);
	input [4:0] ID_EX_RegisterRt, IF_ID_RegisterRs, IF_ID_RegisterRt;
	input ID_EX_MemRead;
	output PCWrite, IF_ID_Write, ID_Flush_lwstall;
	wire equal_IDEXrt_IFIDrs,equal_IDEXrt_IFIDrt;
	assign equal_IDEXrt_IFIDrs=(ID_EX_RegisterRt==IF_ID_RegisterRs)?1:0;
	assign equal_IDEXrt_IFIDrt=(ID_EX_RegisterRt==IF_ID_RegisterRt)?1:0;
	reg PCWrite, IF_ID_Write, ID_Flush_lwstall;
	
	always@(ID_EX_MemRead or equal_IDEXrt_IFIDrs or equal_IDEXrt_IFIDrt)
	begin
		if(ID_EX_MemRead & (equal_IDEXrt_IFIDrs|equal_IDEXrt_IFIDrt))
		begin PCWrite<=0;IF_ID_Write<=0;ID_Flush_lwstall<=1; end
		else 
		begin PCWrite<=1;IF_ID_Write<=1;ID_Flush_lwstall<=0; end
	end
endmodule


// this module flushes IF/ID, ID/EX and EX/MEM if branch OR jump is determined viable at MEM stage
//		we previously assumed branch NOT-taken, so 3 next instructions need to be flushed
//		we are pushig jump to MEM stage because there might be a jump instruction right below our branch_not_taken assumption
//		so we need to wait for branch result to come out before executing jump
//		and of course becuase of the wait, all jump need to flush the next 3 instrucitons
// all connecions are aynchronous; no clock signal is provided
module branch_and_jump_hazard_control (MEM_PCSrc, IF_Flush, ID_Flush_Branch, EX_Flush);
	input MEM_PCSrc; // the PCSrc generated in MEM stage will be 1 if branch is taken or a jump instruction is detected at MEM stage
	output IF_Flush, ID_Flush_Branch, EX_Flush;
	reg IF_Flush, ID_Flush_Branch, EX_Flush;
	always @(MEM_PCSrc)
	begin
		if(MEM_PCSrc)
		begin IF_Flush<=1; ID_Flush_Branch<=1; EX_Flush<=1; end
		else 
		begin IF_Flush<=0; ID_Flush_Branch<=0; EX_Flush<=0; end
	end
endmodule

// this module is not shown in textbook
// all connections acynchronous; no clock signal is provided.
// this module is designed to merge branch(if taken) and jump instruction
// getting an output of PCSrc and a destintion PC address
module jump_OR_branch (Jump, Branch_taken, branch_addr, jump_addr, PCSrc, addr_out);
	input Jump, Branch_taken; 
	input [31:0] branch_addr, jump_addr;
	output PCSrc;
	output [31:0] addr_out;
	reg [31:0] addr_out;
	reg PCSrc;
	// only one of Jump or Branch_taken can be true in MEM in one cycle
	// so if Jump is true, assign jump_addr to addr_out, and set PCSrc to 1
	// and if Branch is true, assign branch_addr to addr_out, and set PCSrc to 1
	// if none of the two are true, set PCSrc to 0. addr_out could be whatever.
	always @(Jump or Branch_taken or branch_addr or jump_addr)
	begin
		if(Branch_taken)
		begin addr_out<=branch_addr;PCSrc<=1; end
		else if (Jump)
		begin addr_out<=jump_addr;PCSrc<=1;end
		else 
		begin PCSrc<=0; addr_out<=32'b0; end
	end
	
endmodule

// 32-bit 3-to-1 MUX for forwarding
// data input width: 3 32-bit
// data output width: 1 32-bit
// control: 2-bit
module Mux_32bit_3to1 (in00, in01, in10, mux_out, control);
	input [31:0] in00, in01, in10;
	output [31:0] mux_out;
	input [1:0] control;
	reg [31:0] mux_out;
	always @(in00 or in01 or in10 or control)
	begin
		case(control)
		2'b00:mux_out<=in00;
		2'b01:mux_out<=in01;
		2'b10:mux_out<=in10;
		default: mux_out<=in00;
		endcase
	end 
endmodule

//////////////////////////////////////////////////////////////////////////
// modules that require modification from single-cycle implementation ////
//////////////////////////////////////////////////////////////////////////
// this one added a PCWrite signal. 
// Stop writing on this rising edge if PCWrite equals 0
//
// original module description:
// rising-edge synchronous program counter
// output range: decimal 0 to 64 (== I-MEM height)
// data I/O width: 64 = 2^6
// async reset: set program counter to 0 asynchronously
module Program_Counter (clk, reset, PC_in, PC_out, PCWrite);
	input PCWrite; // new input for pipeline
	input clk, reset;
	input [6:0] PC_in;//at most 16 instructions
	output [6:0] PC_out;
	reg [6:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else if(PCWrite)
			PC_out<=PC_in;
	end
endmodule


//////////////////////////////////////////////////////////////////////////
// modules that are direct copies from single cycle implementation    ////
//////////////////////////////////////////////////////////////////////////
// async read I-MEM
// height: 64, width: 32 bits (as required by TA)
// PC input width: 64 = 2^6
// instruction output width: 32 bits (== I-MEM width)
// async reset: as specified in document "Project Two Specification (V3)", 
// 		  		first reset all to 0, then hard-code instructions
module Instruction_Memory (read_addr, instruction, reset);
	input reset;
	input [6:0] read_addr;
	output [31:0] instruction;
	reg [31:0] Imemory [63:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	wire [5:0] shifted_read_addr;
	assign shifted_read_addr=read_addr[6:2];
	assign instruction = Imemory[shifted_read_addr];

	always @(posedge reset)
	begin

		for (k=16; k<32; k=k+1) begin// here Ou changes k=0 to k=16
			Imemory[k] = 32'b0;
		end
					
		Imemory[0] = 32'b00100000000010000000000000100000; //addi $t0, $zero, 32
		Imemory[1] = 32'b00100000000010010000000000110111; //addi $t1, $zero, 55
		Imemory[2] = 32'b00000001000010011000000000100000; //add $s0, $t0, $t1
		Imemory[3] = 32'b00000001000010011000100000100010; //sub $s1, $t0, $t1
		Imemory[4] = 32'b00000001000010011001000000100100; //and $s2, $t0, $t1
		Imemory[5] = 32'b00000001000010011001100000100101; //or $s3, $t0, $t1
		Imemory[6] = 32'b00000001000010011010000000101010; //slt $s4, $t0, $t1 (LOOP) 
		Imemory[7] = 32'b00010010100000000000000000000110; //beq $s4, $zero, EXIT
		Imemory[8] = 32'b00000001000010010101000000100000; //add $t2, $t0, $t1
		Imemory[9] = 32'b00000001010010010101000000100000; //add $t2, $t2, $t1
		Imemory[10] = 32'b00000001010010000101100000100000; //add $t3, $t2, $t0
		Imemory[11] = 32'b10001100000101010000000000000100; //lw $s5, 4($zero)
		Imemory[12] = 32'b00000001010101011011000000100000; //add $s6, $t2, $s5
		Imemory[13] = 32'b00000010101010111011100000100000; //add $s7, $s5, $t3
		Imemory[14] = 32'b10101100000101010000000000001000; //sw $s5, 8($zero) (EXIT)
		Imemory[15] = 32'b00001000000000000000000000000110; //j LOOP


	end
endmodule

// 32-bit ALU for addition only
// data input width: 2 32-bit
// data output width: 1 32-bit, no "zero" output
// control: no control input, only addition operation implemeneted
// as specified in Fig 4.12
// attached in email as "Fig 4_12 ALU Control Input"
module ALU_add_only (inA, inB, add_out);
	input [31:0] inA, inB;
	output [31:0] add_out;
	assign add_out=inA+inB;
endmodule

// N-bit 2-to-1 Mux
// input: 2 N-bit input
// output: 1 N-bit output
// control: 1 bit
// possible value of N in single cycle: 5, 6, 32
module Mux_N_bit (in0, in1, mux_out, control);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input control;
	assign mux_out=control?in1:in0;
endmodule

// sync register file (write/read occupy half cycle each)
// height: 32 (from $0 to $ra), width: 32 bits
// write: on rising edge; data width 32 bit; address width 5 bit
// read: on falling edge; data width 32 bit; address width 5 bit
// control: write on rising edge if (RegWrite == 1)
// async reset: set all register content to 0
module Register_File (read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite, clk, reset);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [31:0] write_data;
	input clk, reset, RegWrite;
	output [31:0] read_data_1, read_data_2;

	reg [31:0] Regfile [31:0];
	reg [31:0] read_data_1, read_data_2;
	integer k;

	always @(posedge clk or posedge reset) // Ou combines the block of reset into the block of posedge clk
	begin
		if (reset==1'b1)
		begin
			for (k=0; k<32; k=k+1) 
			begin
				Regfile[k] = 32'b0;
			end
		end 
		
		else if (RegWrite == 1'b1) Regfile[write_addr] = write_data; 
	end

	always @(negedge clk)
	begin
		read_data_1 = Regfile[read_addr_1];
		read_data_2 = Regfile[read_addr_2];
	end
endmodule

// sign-extend the 16-bit input to the 32_bit output
module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

// shift-left-2 for jump instruction
// input width: 26 bits
// output width: 28 bits
// fill the void with 0 after shifting
// we don't need to shift in this case, becasue the address of the instructions
// are addressed by words
module Shift_Left_2_Jump (shift_in, shift_out);
	input [25:0] shift_in;
	output [27:0] shift_out;
	assign shift_out[27:0]={shift_in[25:0],2'b00};
endmodule

// async control signal generation unit based on OpCode
// as specified in Fig 4.22
// attached in email as file "Fig 4_22 Single Cycle Control"
// input: 6 bits OpCode
// output: all 1 bit except ALUOp which is 2-bits wide
module Control (OpCode, RegDst, Jump, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
	input [5:0] OpCode;
	output RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
	output [1:0] ALUOp;

	assign RegDst=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]);//000000
	assign Jump=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(~OpCode[0]);//000010
	assign Branch=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]);//000100
	assign MemRead=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//100011
	assign MemtoReg=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//100011
	assign MemWrite=(OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//101011
	assign ALUSrc=((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0])) | (((OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]))); //001000,001100,100011,101011
	assign RegWrite=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]));//000000,001000,001100,100011
	assign ALUOp[1]=((~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]))|((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]));//000000, 001100(andi)
	assign ALUOp[0]= ((~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]))|((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]));//000100,001100(andi)
endmodule

// async control to generate ALU input signal
// as specified in Fig 4.12
// attached in email as file "Fig 4_12 ALU Control Input"
// input: 2-bit ALUOp control signal and 6-bit funct field from instruction
// output: 4-bit ALU control input
module ALUControl (ALUOp, funct, out_to_ALU);
	input [1:0] ALUOp;
	input [5:0] funct;
	output [3:0] out_to_ALU;

	assign out_to_ALU[3]=0;
	assign out_to_ALU[2]=((~ALUOp[1])&(ALUOp[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(funct[1])&(~funct[0])) | ((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));
	assign out_to_ALU[1]=((~ALUOp[1])&(~ALUOp[0]))|((~ALUOp[1])&(ALUOp[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(~funct[1])&(~funct[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(funct[1])&(~funct[0]))|((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));
	assign out_to_ALU[0]=((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(funct[2])&(~funct[1])&(funct[0]))|((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));	
endmodule

// 32-bit ALU
// data input width: 2 32-bit
// data output width: 1 32-bit and one "zero" output
// control: 4-bit
// zero: output 1 if all bits of data output is 0
// as specified in Fig 4.12
// attached in email as "Fig 4_12 ALU Control Input"
module ALU (inA, inB, alu_out, zero, control);
	input [31:0] inA, inB;
	output [31:0] alu_out;
	output zero;
	reg zero;
	reg [31:0] alu_out;
	input [3:0] control;
	always @ (control or inA or inB)
	begin
		case (control)
		4'b0000:begin zero<=0; alu_out<=inA&inB; end
		4'b0001:begin zero<=0; alu_out<=inA|inB; end
		4'b0010:begin zero<=0; alu_out<=inA+inB; end
		4'b0110:begin if(inA==inB) zero<=1; else zero<=0; alu_out<=inA-inB; end
		4'b0111:begin zero<=0; if(inA-inB>=32'h8000_0000) alu_out<=32'b1; else alu_out<=32'b0; end// how to implement signed number
		default: begin zero<=0; alu_out<=inA; end
		endcase
	end
endmodule

// shift-left-2 for branch instruction
// input width: 32 bits
// output width: 32 bits
// fill the void with 0 after shifting
module Shift_Left_2_Branch (shift_in, shift_out);
	input [31:0] shift_in;
	output [31:0] shift_out;
	assign shift_out[31:0]={shift_in[29:0],2'b00};
endmodule

// rising edge sync-write, async-read D-MEM
// height: 64, width: 32 bits (from document "Project Two Specification (V3)")
// address input: 6 bits (64 == 2^6)
// data input/output: 32 bits
// write: on rising edge, when (MemWrite == 1)
// read: asynchronous, when (MemRead == 1)
module Data_Memory (addr, write_data, read_data, clk, reset, MemRead, MemWrite);
	input [7:0] addr;
	input [31:0] write_data;
	output [31:0] read_data;
	input clk, reset, MemRead, MemWrite;
	reg [31:0] DMemory [63:0];
	integer k;
	wire [5:0] shifted_addr;
	assign shifted_addr=addr[7:2];
	assign read_data = (MemRead) ? DMemory[shifted_addr] : 32'bx;

	always @(posedge clk or posedge reset)// Ou modifies reset to posedge
	begin
		if (reset == 1'b1) 
			begin
				for (k=0; k<64; k=k+1) begin
					DMemory[k] = 32'b0;
				end

			end
		else
			if (MemWrite) DMemory[shifted_addr] = write_data;
	end
endmodule

//////////////////////////////////////////////////////////////////////////
// modules for slowing the clock and displaying on SSD                ////
//////////////////////////////////////////////////////////////////////////
module Dff_asy (q, d, clk, rst);
	input d, clk, rst;
	output reg q;
	
	always @ (posedge clk or posedge rst)
		if (rst == 1) q <= 0;
		else q <= d;
endmodule

// The following modules implement SSD display & clock slow-down
module divide_by_500 (clock, reset, clock_out);
	parameter N = 9;
	input 	clock, reset;
	wire		load, asyclock_out;
	wire 		[N-1:0] Dat;
	output 	clock_out;
	reg 		[N-1:0] Q;
	assign	Dat = 9'b000000000;
	assign	load = Q[8] & Q[7] & Q[6] & Q[5] & Q[4] & Q[1] & Q[0];
	always @ (posedge reset or posedge clock)
	begin
		if (reset == 1'b1) Q <= 9'b000000000;
		else if (load == 1'b1) Q <= Dat;
		else Q <= Q + 1;
	end
	assign	asyclock_out = load;
	Dff_asy Unit_Dff (.q(clock_out), .d(asyclock_out), .clk(clock), .rst(reset));
endmodule

module divide_by_100k (clock, reset, clock_out);
	parameter N = 17;
	input	clock, reset;
	wire	load, asyclock_out;
	wire 	[N-1:0] Dat;
	output 	clock_out;
	reg 	[N-1:0] Q;
	assign	Dat = 0;
	assign	load = Q[16] & Q[15] & Q[10] & Q[9] & Q[7] & Q[4] & Q[3] & Q[2] & Q[1] & Q[0];
	always @ (posedge reset or posedge clock)
	begin
		if (reset == 1'b1) Q <= 0;
		else if (load == 1'b1) Q <= Dat;
		else Q <= Q + 1;
	end
	assign	asyclock_out = load;
	Dff_asy Unit_Dff (.q(clock_out), .d(asyclock_out), .clk(clock), .rst(reset));
endmodule

module Ring_4_counter(clock, reset, Q);
	input 		clock, reset;
	output reg	[3:0]Q;
	
	always @(posedge clock or posedge reset)
	begin
		if (reset == 1) Q <= 4'b1110;
		else
		begin
			Q[3] <= Q[0];
			Q[2] <= Q[3];
			Q[1] <= Q[2];
			Q[0] <= Q[1];
		end
	end
endmodule

module ssd_driver (in_BCD, out_SSD);
	input [3:0] in_BCD; // input in Binary-Coded Decimal
	output [6:0] out_SSD; // output to Seven-Segment Display
	reg [6:0] out_SSD;
	always @(in_BCD) begin
		case (in_BCD)
		0:out_SSD=7'b0000001;
		1:out_SSD=7'b1001111;
		2:out_SSD=7'b0010010;
		3:out_SSD=7'b0000110;
		4:out_SSD=7'b1001100;
		5:out_SSD=7'b0100100;
		6:out_SSD=7'b0100000;
		7:out_SSD=7'b0001111;
		8:out_SSD=7'b0000000;
		9:out_SSD=7'b0000100;
		10:out_SSD=7'b0001000;
		11:out_SSD=7'b1100000;
		12:out_SSD=7'b0110001;
		13:out_SSD=7'b1000010;
		14:out_SSD=7'b0110000;
		15:out_SSD=7'b0111000;
			default out_SSD = 7'b1111111; // no ssd
		endcase
	end
endmodule

module choose_chathode(tho, hun, ten, one, AN, CA);
	input	[6:0]tho;
	input	[6:0]hun;
	input	[6:0]ten;
	input	[6:0]one;
	input	[3:0]AN;
	output	[6:0]CA;
	assign CA = (AN==4'b1110) ? one : 7'bzzzzzzz,
		   CA =	(AN==4'b1101) ? ten : 7'bzzzzzzz,
		   CA = (AN==4'b1011) ? hun : 7'bzzzzzzz,
		   CA =	(AN==4'b0111) ? tho : 7'bzzzzzzz;
endmodule
© 2018 GitHub, Inc.
Terms
Privacy
Security
Status
Help
Contact GitHub
Pricing
API
Training
Blog
About
Press h to open a hovercard with more details.