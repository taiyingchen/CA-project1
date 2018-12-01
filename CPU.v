module CPU
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

//project1 new
Control Control(
    .Op_i       (inst[6:0]),
    //EX
    .ALUOp_o    (ALU_Control.ALUOp_i),
    .ALUSrc_o   (MUX_ALUSrc.select_i),
    //MEM
    .Branch_o   (branch)
    .MemRead_o  (Data_Memory.MemRead_i)
    .MemWrite_o (Data_Memory.MemWrite_i)
    //WB
    .RegWrite_o (Registers.RegWrite_i)
    .MemtoReg_o (MUX_RegSrc.select_i)
);


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
    .data2_in   (Sign_Extend<<1),
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

Instruction_Memory Instruction_Memory(
    .addr_i     (inst_addr), 
    .instr_o    (inst)
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

MUX32 MUX_ALUSrc(
    .data1_i    (Registers.RS2data_o),
    .data2_i    (Sign_Extend.data_o),
    .select_i   (Control.ALUSrc_o),
    .data_o     (ALU.data2_i)
);

Sign_Extend Sign_Extend(
    .data_i     (inst[31:20]),
    .data_o     (MUX_ALUSrc.data2_i)
);

ALU ALU(
    .data1_i    (Registers.RS1data_o),
    .data2_i    (MUX_ALUSrc.data_o),
    .ALUCtrl_i  (ALU_Control.ALUCtrl_o),
    .data_o     (Data_Memory.addr_i),
    .Zero_o     (zero)
);

Data_Memory Data_Memory(
    .addr_i     (ALU.data_o),
    .data_i     (Registers.RS2data_o),
    .MemRead_i  (Control.MemRead_o),
    .MemWrite_i (Control.MemWrite_o),
    .data_o     (MUX_RegSrc.data2_i)
);

MUX32 MUX_RegSrc(
    .data1_i    (ALU.data_o),
    .data2_i    (Data_Memory.data_o),
    .select_i   (Control.MemtoReg_o),
    .data_o     (Registers.RDdata_i)
);

ALU_Control ALU_Control(
    .funct3_i    (inst[14:12]),
    .funct7_i    (inst[31:25]),
    .ALUOp_i    (Control.ALUOp_o),
    .ALUCtrl_o  (ALU.ALUCtrl_i)
);

endmodule

