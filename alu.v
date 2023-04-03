
//Arithmetic logic unit (ALU) module for use in the data path

module ALU(Ain, Bin, ALUop, out, status_out);

//input declarations

input[15:0] Ain, Bin;
input[1:0] ALUop;
output[15:0] out;
output[2:0] status_out;

reg[15:0] out;
reg[2:0] status_out;

//case statement used to determine which operation should be carried out by the ALU

always @(*) begin
case(ALUop)
2'b00: out = (Ain + Bin);
2'b01: out = (Ain - Bin);
2'b10: out = (Ain & Bin);
2'b11: out = (~Bin);
default: out = {16{1'bx}};
endcase

//check to see if the output is negative - if so, set status flag accordingly

if(out[15] == 1'b1)
status_out[0] = 1'b1;
else
status_out[0] = 1'b0;

//check to see if the output is overflowed - if so, set status flag accordingly

if((Ain[15] == 1'b1 && Bin[15] == 1'b1 && out[15] == 1'b0) || (Ain[15] == 1'b0 && Bin[15] == 1'b0 && out[15] == 1'b1)) //overflow
status_out[1] = 1'b1;
else
status_out[1] = 1'b0;

//MSB1 ^ MSB2 

//checks whether output from ALU is zero; if so, Z is set to 1, otherwise 0

if(out == {16{1'b0}})
status_out[2] = 1'b1;
else
status_out[2] = 1'b0;
end

endmodule
