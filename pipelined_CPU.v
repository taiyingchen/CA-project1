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

wire    [31:0]  inst_addr, inst;
//project1 new
wire    zero, branch;
wire    andGate_o;
assign  andGate_o = branch  && zero;

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
);// pass all message to ID/EX register ?


//project1 new
MUX32 MUX_PCSrc(
    .data1_i    (Add_PC.data_o),
    .data2_i    (Add_Imm.data_o),
    .select_i   (andGate_o),
    .data_o     (PC.pc_i)
);

//project1 new
Adder Add_Imm(
    .data1_in   (PC.pc_o),
    .data2_in   (Sign_Extend.data_o<<1),
    .data_o     (MUX_PCSrc.data2_i)
);

Adder Add_PC(
    .data1_in   (inst_addr),
    .data2_in   (32'd4),
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
Instruction_Memory Instruction_Memory(
    .addr_i     (inst_addr), 
    .instr_o    (IF_ID_Reg.instruction_in)
);

//project1 new (Lin)
IF_ID IF_ID_Reg(
	.PC_plus4_in	(inst_addr), 
	.PC_plus4_out	(ID_EX_Reg.PC_plus4_in), 
	.instruction_in	(Instruction_Memory.instr_o), 
	.instruction_out(inst), 
	.IF_ID_Write	(), //forwarding control unit 
	.IF_Flush		(), //hazard unit
	.clk			(clk), 
	.reset			(reset)
);


Registers Registers(
    .clk_i      (clk_i),
    .RS1addr_i   (inst[19:15]),
    .RS2addr_i   (inst[24:20]),
    .RDaddr_i   (inst[11:7]),
    .RDdata_i   (MUX_RegSrc.data_o),//project1 new
    .RegWrite_i (Control.RegWrite_o),
    .RS1data_o   (ALU.data1_i),
    .RS2data_o   (MUX_ALUSrc.data1_i)
);



Sign_Extend Sign_Extend(
    .data_i     (inst[31:20]),
    .data_o     ()
);

//project1 new (Lin)
ID_EX ID_EX_Reg(
	.ID_Flush_lwstall	(), //stall control for load instruction
	.ID_Flush_Branch	(), //hazard unit
	.RegWrite_in		(Control.RegWrite_o), 
	.MemtoReg_in		(Control.MemtoReg_o), 
	.RegWrite_out		(EX_MEM_Reg.RegWrite_in), 
	.MemtoReg_out		(EX_MEM_Reg.MemtoReg_in), 
	.Branch_in			(Control.Branch_o), 
	.MemRead_in			(Control.MemRead_o), 
	.MemWrite_in		(Control.MemWrite_o), 
	.Jump_in			(), //from Control, needed?
	.Branch_out			(),  //to whom?
	.MemRead_out		(EX_MEM_Reg.MemRead_in), 
	.MemWrite_out		(EX_MEM_Reg.MemWrite_in), 
	.Jump_out			(), 
	.RegDst_in			(), //from Control, needed?
	.ALUSrc_in			(Control.ALUSrc_o), 
	.RegDst_out			(), 
	.ALUSrc_out			(MUX_ALUSrc.select_i), 
	.ALUOp_in			(Control.ALUOp_o),
	.ALUOp_out			(ALU_Control.ALUOp_i), 
	.jump_addr_in		(),
	.jump_addr_out		(),
	.PC_plus4_in		(IF_ID_Reg.PC_plus4_out), 
	.PC_plus4_out		(), //to the adder of the branch address
	.reg_read_data_1_in	(), //from Mux_N_bit(what that?)
	.reg_read_data_2_in	(), //from Mux_N_bit(what that?) 
	.immi_sign_extended_in(Sign_Extend.data_o), 
	.reg_read_data_1_out(), //to ALU_data1
	.reg_read_data_2_out(), //to ALU_data2
	.immi_sign_extended_out(MUX_ALUSrc.data2_i), 
	.IF_ID_RegisterRs_in(), //inst[a:b]
	.IF_ID_RegisterRt_in(), //inst[a:b]
	.IF_ID_RegisterRd_in(), //inst[a:b]
	.IF_ID_RegisterRs_out(), //to forwarding unit
	.IF_ID_RegisterRt_out(), //to forwarding unit 
	.IF_ID_RegisterRd_out(), 
	.IF_ID_funct_in		(),  //inst[a:b]
	.IF_ID_funct_out	(),  //to ALU_Control(needed?)
	.clk				(clk), 
	.reset				(reset)
);

//project1 new (Lin)
MUX32_3to1 ALU_data1(
);
MUX32_3to1 ALU_data2(
);

//project1 new (Lin)
MUX32 MUX_ALUSrc(
    .data1_i    (Registers.RS2data_o),
    .data2_i    (ID_EX_Reg.immi_sign_extended_out),
    .select_i   (ID_EX_Reg.ALUSrc_out),
    .data_o     (ALU.data2_i)
);

//project1 new (Lin)
ALU ALU(
    .data1_i    (Registers.RS1data_o),
    .data2_i    (MUX_ALUSrc.data_o),
    .ALUCtrl_i  (ALU_Control.ALUCtrl_o),
    .data_o     (EX_MEM_Reg.ALU_result_in),
    .Zero_o     (zero)
);

//project1 new (Lin)
ALU_Control ALU_Control(
    .funct3_i    (inst[14:12]),
    .funct7_i    (inst[31:25]),
    .ALUOp_i    (ID_EX_Reg.ALUOp_out),
    .ALUCtrl_o  (ALU.ALUCtrl_i)
);

//project1 new (Lin)
EX_MEM EX_MEM_Reg(
	.EX_Flush		(), 
	.RegWrite_in	(), 
	.MemtoReg_in	(), 
	.RegWrite_out	(), 
	.MemtoReg_out	(), 
	.Branch_in		(), 
	.MemRead_in		(), 
	.MemWrite_in	(),
	.Jump_in		(), 
	.Branch_out		(),
	.MemRead_out	(), 
	.MemWrite_out	(),
	.Jump_out		(), 
	.jump_addr_in	(), 
	.branch_addr_in	(),
	.jump_addr_out	(),
	.branch_addr_out(),
	.ALU_zero_in	(zero), 
	.ALU_zero_out	(),
	.ALU_result_in	(ALU.data_o),
	.reg_read_data_2_in(),
	.ALU_result_out	(Data_Memory.addr_i),
	.reg_read_data_2_out(), 
	.ID_EX_RegisterRd_in(), 
	.EX_MEM_RegisterRd_out(), 
	.clk			(), 
	.reset			()
);

//project1 new (Lin)
Data_Memory Data_Memory(
    .addr_i     (ALU.data_o),
    .data_i     (Registers.RS2data_o),
    .MemRead_i  (),
    .MemWrite_i (),
    .data_o     (MUX_RegSrc.data2_i)
);

//project1 new (Lin)
MEM_WB MEM_WB_Reg(
	.RegWrite_in		(), 
	.MemtoReg_in		(), 
	.RegWrite_out		(), 
	.MemtoReg_out		(), 
	.D_MEM_read_data_in	(), 
	.D_MEM_read_addr_in	(),
	.D_MEM_read_data_out(), 
	.D_MEM_read_addr_out(), 
	.EX_MEM_RegisterRd_in(), 
	.MEM_WB_RegisterRd_out(), 
	.clk				(), 
	.reset				()
);


MUX32 MUX_RegSrc(
    .data1_i    (ALU.data_o),
    .data2_i    (Data_Memory.data_o),
    .select_i   (Control.MemtoReg_o),
    .data_o     (Registers.RDdata_i)
);





endmodule

