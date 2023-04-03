////module for the creation of a sequential datapath using a regfile, ALU, 
////shifter, and other combinational building blocks

module datapath(mdata, writenum, write, 
readnum, clk, loada, loadb, loadc, loads, shift, 
asel, bsel, vsel, ALUop, C, status_out, sximm8, sximm5, PC);

input[15:0] mdata; 
input[2:0] writenum, readnum; 
input write, clk, loada, loadb, loadc, loads, asel, bsel; 
input[1:0] shift, ALUop, vsel; 

input[15:0] sximm8, sximm5;
input[8:0] PC;

output[2:0]status_out;
output[15:0] C;

wire[15:0] data_in, data_out, aOut, bOut, sOut, Ain, Bin, outALU;
wire[2:0]status_in;

//initialzing a 4 to 1 mux that will precede the 
//register file and select between mdata, sximm8, a concatenation of PC and 0, and 
//the output of the datapath 

Mux4to1 muxIn(mdata, sximm8, {{7{1'b0}}, PC}, C, data_in, vsel);

//initializing a 16 bit, 8 register register file 

regfile REGFILE(data_in, writenum, write, readnum, clk, data_out);

//initializing two load enable registers that will hold data loaded from memory

vDFFE #(16) RA(clk, loada, data_out, aOut);
vDFFE #(16) RB(clk, loadb, data_out, bOut);

//initializing a shifter unit used to shift an input based on a given operation code

shifter U1(bOut, shift, sOut);

//initializing two muxs that will be used to select between inputs given to the ALU

Mux2to1 muxA({16{1'b0}}, aOut, Ain, asel);
Mux2to1 muxB(sximm5, sOut, Bin, bsel);

//initializing an arithemtic logic unit (ALU) to be used to perform computations 

ALU U2(Ain, Bin, ALUop, outALU, status_in);

//initializing two more load enable registers, one that will hold the output of the ALU (RC)
//and the other that will hold the status of the output (status)

vDFFE #(16) RC(clk, loadc, outALU, C);
vDFFE #(3) status(clk, loads, status_in, status_out);

endmodule

//2 to 1 mux module used in the datapath

module Mux2to1(in1, in0, out, sel);
input[15:0] in1, in0;
input sel;
output[15:0] out;
assign out = sel ? in1: in0;
endmodule

//4 to 1 mux module used at the beginning of the datapath

module Mux4to1(in1, in2, in3, in4, out, sel);
input[15:0] in1, in2, in3, in4;
input[1:0] sel;
output[15:0] out;
reg[15:0] out;
//case statement used to select between inputs to the datapath
always @ (*)
case(sel)
2'b00: out = in1;
2'b01: out = in2;
2'b10: out = in3;
2'b11: out = in4;
default: out = {16{1'bx}};
endcase
endmodule
