module ALU_Control
(
    funct3_i,
    funct7_i,
    ALUOp_i,
    ALUCtrl_o
);

input   [2:0]   funct3_i;
input   [6:0]   funct7_i;
input   [1:0]   ALUOp_i;
output reg  [2:0]   ALUCtrl_o;

always@(*) begin
    if (ALUOp_i == 2'b00)
        ALUCtrl_o = 3'b010; // addi
    else if(ALUOp_i == 2'b10) begin
        case(funct3_i)
            3'b110: ALUCtrl_o = 3'b001; // or
            3'b111: ALUCtrl_o = 3'b000; // and
            3'b000: begin
                if(funct7_i[0] == 1'b1)
                    ALUCtrl_o = 3'b111; // mul
                else if(funct7_i[5] == 1'b1)
                    ALUCtrl_o = 3'b110; // sub
                else if(funct3_i == 0 && funct7_i == 0)
                    ALUCtrl_o = 3'b010; // add
                else
                    ALUCtrl_o = 3'bx; // don't care
            end
            default: ALUCtrl_o = 3'bx; // don't care 
        endcase
    end
    else
        ALUCtrl_o = 3'bx; // don't care
end

endmodule
