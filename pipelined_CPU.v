module pipelined_CPU //project1 new (Lin), not sure about the correctness
(
    clk_i, 
    rst_i,
    start_i
);

// Ports
input               clk_i;
input               rst_i;
input               start_i;

wire    [31:0]  inst_addr, inst,imm_sign_extended_data;
//project1 new
wire    zero, branch;
wire    andGate_o;
assign  andGate_o = branch  && zero;


// IF stage:----------------------------------------------------------------



//project1 new
MUX32 MUX_PCSrc(
    .data1_i    (Add_PC.data_o),
    .data2_i    (Add_Imm.data_o),
    .select_i   (andGate_o),
    .data_o     (PC.pc_i)
);

PC PC(
    .clk_i      (clk_i),
    .rst_i      (rst_i),
    .start_i    (start_i),
    .pc_i       (MUX_PCSrc.data_o),//project1 new
    .pc_o       (inst_addr)
);

//project1 new (Lin)
Adder Add_Imm(//Shouldhould we put this into the ID stage, and pass PC through the IF/ID register? 
    .data1_in   (PC.pc_o),
    .data2_in   (imm_sign_extended_data<<1),
    .data_o     (MUX_PCSrc.data2_i)
);

Adder Add_PC(
    .data1_in   (inst_addr),
    .data2_in   (32'd4),
    .data_o     (PC.pc_i)
);



//project1 new (Lin)
Instruction_Memory Instruction_Memory(
    .addr_i     (inst_addr), 
    .instr_o    (IF_ID_Reg.instruction_in)
);

//project1 new (Lin)
IF_ID IF_ID_Reg(
	// .PC_in			(inst_addr), 
	// .PC_out			(ID_EX_Reg.PC_in), 
	.instruction_in	(Instruction_Memory.instr_o), 
	.instruction_out(inst), 
	.IF_ID_Write	(), //forwarding control unit 
	.IF_Flush		(), //hazard unit
	.clk			(clk), 
	.reset			(reset)
);
// ID stage:----------------------------------------------------------------


//project1 new (Lin)
Control Control(//flush,hazard not yet
    .Op_i       (inst[6:0]),
    //EX
    .ALUOp_o    (ID_EX_Reg.ALUOp_in),
    .ALUSrc_o   (ID_EX_Reg.ALUSrc_in),
    //MEM
    .Branch_o   (ID_EX_Reg.Branch_in),
    .MemRead_o  (ID_EX_Reg.MemRead_in),
    .MemWrite_o (ID_EX_Reg.MemWrite_in),
    //WB
    .RegWrite_o (ID_EX_Reg.RegWrite_in),
    .MemtoReg_o (ID_EX_Reg.MemtoReg_in)
);


//project1 new (Lin)
wire	[31:0]	WriteBack_data;
Registers Registers(
    .clk_i      (clk_i),
    .RS1addr_i   (inst[19:15]),
    .RS2addr_i   (inst[24:20]),
    .RDaddr_i   (MEM_WB_RegisterRd),//inst[11:7]
    .RDdata_i   (WriteBack_data),//from the MUX_RegSrc
    .RegWrite_i (Control.RegWrite_o),
    .RS1data_o   (ALU.data1_i),
    .RS2data_o   (MUX_ALUSrc.data1_i)
);


//project1 new (Lin)
Sign_Extend Sign_Extend(
    .data_i     (inst[31:20]),
    .data_o     (imm_sign_extended_data)
);




//project1 new (Lin)
ID_EX ID_EX_Reg(
	.ID_Flush_lwstall	(), //stall control for load instruction
	.ID_Flush_Branch	(), //hazard unit, control hazard
	.RegWrite_in		(Control.RegWrite_o), 
	.MemtoReg_in		(Control.MemtoReg_o), 
	.RegWrite_out		(EX_MEM_Reg.RegWrite_in), 
	.MemtoReg_out		(EX_MEM_Reg.MemtoReg_in), 
	.Branch_in			(Control.Branch_o), 
	.MemRead_in			(Control.MemRead_o), 
	.MemWrite_in		(Control.MemWrite_o), 
	.Branch_out			(EX_MEM_Reg.Branch_in), 
	.MemRead_out		(EX_MEM_Reg.MemRead_in), 
	.MemWrite_out		(EX_MEM_Reg.MemWrite_in), 
	// .RegDst_in			(), //from Control, needed?
	.ALUSrc_in			(Control.ALUSrc_o), 
	// .RegDst_out			(), //to EX_RegisterRd MUX(what is EX_RegisterRd MUX?)
	.ALUSrc_out			(ALU_input2.ALUSrc), 
	.ALUOp_in			(Control.ALUOp_o),
	.ALUOp_out			(ALU_Control.ALUOp_i), 
	// .PC_in				(IF_ID_Reg.PC_out), 
	// .PC_out				(), //to the adder of the branch address
	.reg_read_data_1_in	(), //from Mux_N_bit(WB data forward to ALU)
	.reg_read_data_2_in	(), //from Mux_N_bit(WB data forward to ALU) 
	.immi_sign_extended_in(imm_sign_extended_data), 
	.reg_read_data_1_out(ALU_input1.data1_i), //to ALU_input1
	.reg_read_data_2_out(ALU_input2.data1_i), //to ALU_input2
	.immi_sign_extended_out(ALU_input2.data4_i), 
	.IF_ID_RegisterRs_in(inst[19:15]),
	.IF_ID_RegisterRt_in(inst[24:20]), 
	.IF_ID_RegisterRd_in(inst[11:7]), 
	.IF_ID_RegisterRs_out(), //to forwarding unit
	.IF_ID_RegisterRt_out(), //to forwarding unit, also to EX_RegisterRd MUX(needed?)
	.IF_ID_RegisterRd_out(EX_MEM_Reg.ID_EX_RegisterRd_in), //to EX_RegisterRd MUX(needed?)
	// .IF_ID_funct_in		(),  //inst[a:b]
	// .IF_ID_funct_out	(),  //to ALU_Control(needed?)
	.clk				(clk), 
	.reset				(reset)
);
// EX stage:----------------------------------------------------------------

//project1 new (Lin)
wire 	[31:0]	EX_MEM_ALU_result;
MUX32_3to1 ALU_input1(//input1 from Registers, input2 from WB MUX, input3 from ALU result(not implemented yet)
    .data1_i    (ID_EX_Reg.reg_read_data_1_out),
    .data2_i    (WriteBack_data),	
    .data3_i    (EX_MEM_ALU_result),	
	.select_i	(), //from forwarding unit
	.data_o		(ALU.data1_i)
);
MUX32_4to1 ALU_input2(//input1 from Registers, input2 from WB MUX, input3 from ALU result, input4 from ID/EX imm(not implemented yet)
    .data1_i    (ID_EX_Reg.reg_read_data_2_out),
    .data2_i    (WriteBack_data),	
    .data3_i    (EX_MEM_ALU_result),	
	.select_i	(), //from forwarding unit(decide from first three input)
	.data4_i    (ID_EX_Reg.immi_sign_extended_out),	//imm, if ALUSrc==1 then choose it
	.ALUSrc		(ID_EX_Reg.ALUSrc_out), //(decide to use imm or first three )
	.data_o		(ALU.data2_i)
);


//project1 new (Lin)
ALU ALU(
    .data1_i    (ALU_input1.data_o),
    .data2_i    (ALU_input2.data_o),
    .ALUCtrl_i  (ALU_Control.ALUCtrl_o),
    .data_o     (EX_MEM_Reg.ALU_result_in),
    .Zero_o     (EX_MEM_Reg.ALU_zero_in)
);

//project1 new (Lin)
ALU_Control ALU_Control(
    .funct3_i    (inst[14:12]),
    .funct7_i    (inst[31:25]), //can we pass this two line through the ID/EX stage?
    .ALUOp_i    (ID_EX_Reg.ALUOp_out),
    .ALUCtrl_o  (ALU.ALUCtrl_i)
);

//project1 new (Lin)
wire [4:0] EX_MEM_RegisterRd; //to forwarding unit
EX_MEM EX_MEM_Reg(
	.EX_Flush		(), //hazard unit, control hazard
	.RegWrite_in	(ID_EX_Reg.RegWrite_out), 
	.MemtoReg_in	(ID_EX_Reg.MemtoReg_out), 
	.RegWrite_out	(MEM_WB_Reg.RegWrite_in), 
	.MemtoReg_out	(MEM_WB_Reg.MemtoReg_in), 
	.Branch_in		(ID_EX_Reg.Branch_out), 
	.MemRead_in		(ID_EX_Reg.MemRead_out), 
	.MemWrite_in	(ID_EX_Reg.MemWrite_out),
	.Branch_out		(branch), // to the branch and-gate(result to the jump_OR_branch unit)
	.MemRead_out	(Data_Memory.MemRead_i), 
	.MemWrite_out	(Data_Memory.MemWrite_i),
	.ALU_zero_in	(ALU.Zero_o), 
	.ALU_zero_out	(zero), // to the branch and-gate
	.ALU_result_in	(ALU.data_o),
	.reg_read_data_2_in(ALU_input2.data_o), //from ALU_input2(mux3to1_B)
	.ALU_result_out	(EX_MEM_ALU_result),
	.reg_read_data_2_out(Data_Memory.data1_i), 
	.ID_EX_RegisterRd_in(ID_EX_Reg.IF_ID_RegisterRd_out), //from EX_RegisterRd MUX(needed?)
	.EX_MEM_RegisterRd_out(EX_MEM_RegisterRd), 
	.clk			(clk), 
	.reset			(reset)
);
// MEM stage:----------------------------------------------------------------

//project1 new (Lin)
Data_Memory Data_Memory(
    .addr_i     (EX_MEM_ALU_result),
    .data_i     (EX_MEM_Reg.reg_read_data_2_out),
    .MemRead_i  (EX_MEM_Reg.MemRead_out),
    .MemWrite_i (EX_MEM_Reg.MemWrite_out),
    .data_o     (MEM_WB_Reg.D_MEM_read_data_in),
	.clk		(clk)
);

//project1 new (Lin)
wire 	[4:0]	MEM_WB_RegisterRd;//determine which register to write back
MEM_WB MEM_WB_Reg(
	.RegWrite_in		(EX_MEM_Reg.RegWrite_out), 
	.MemtoReg_in		(EX_MEM_Reg.MemtoReg_out), 
	.RegWrite_out		(), //to forwarding unit
	.MemtoReg_out		(MUX_RegSrc.select_i), 
	.D_MEM_read_data_in	(Data_Memory.data_o), 
	.D_MEM_read_addr_in	(EX_MEM_ALU_result),
	.D_MEM_read_data_out(MUX_RegSrc.data1_i), 
	.D_MEM_read_addr_out(MUX_RegSrc.data2_i), 
	.EX_MEM_RegisterRd_in(EX_MEM_RegisterRd), 
	.MEM_WB_RegisterRd_out(MEM_WB_RegisterRd), 
	.clk				(clk), 
	.reset				(reset)
);
// WB stage:----------------------------------------------------------------


//project1 new (Lin)
MUX32 MUX_RegSrc(//WB MUX
    .data1_i    (MEM_WB_Reg.D_MEM_read_data_out),
    .data2_i    (MEM_WB_Reg.D_MEM_read_addr_out),
    .select_i   (MEM_WB_Reg.MemtoReg_out),
    .data_o     (WriteBack_data)
);





endmodule

