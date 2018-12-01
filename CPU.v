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
wire    zero;

Control Control(
    .Op_i       (inst[6:0]),
    // .RegDst_o   (),
    .ALUOp_o    (ALU_Control.ALUOp_i),
    .ALUSrc_o   (MUX_ALUSrc.select_i),
    .RegWrite_o (Registers.RegWrite_i)
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
    .pc_i       (Add_PC.data_o),
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
    .RDdata_i   (ALU.data_o),
    .RegWrite_i (Control.RegWrite_o),
    .RS1data_o   (ALU.data1_i),
    .RS2data_o   (MUX_ALUSrc.data1_i)
);

MUX32 MUX_RegSrc(
    .data1_i    (),
    .data2_i    (),
    .select_i   (),
    .data_o     ()
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
    .data_o     (Registers.RDdata_i),
    .Zero_o     (zero)
);

ALU_Control ALU_Control(
    .funct3_i    (inst[14:12]),
    .funct7_i    (inst[31:25]),
    .ALUOp_i    (Control.ALUOp_o),
    .ALUCtrl_o  (ALU.ALUCtrl_i)
);

endmodule

