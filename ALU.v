module ALU
(
    data1_i,
    data2_i,
    ALUCtrl_i,
    data_o,
    Zero_o
);

input   [31:0]  data1_i;
input   [31:0]  data2_i;
input   [2:0]   ALUCtrl_i;
output reg  [31:0]  data_o;
output reg  Zero_o;

always@(data1_i or data2_i or ALUCtrl_i) begin
    case(ALUCtrl_i)
        3'b000: data_o = data1_i & data2_i; // and
        3'b001: data_o = data1_i | data2_i; // or
        3'b010: data_o = data1_i + data2_i; // add/addi
        3'b110: begin
            data_o = data1_i - data2_i; // sub
            if (data_o == 0)
                Zero_o = 1;
            else
                Zero_o = 0;
        end
        3'b111: data_o = data1_i * data2_i; // mul
        default: data_o = 32'bx;
    endcase
end
   
endmodule 
