//top-level module for the register file in the data path

module regfile(data_in, writenum, write, readnum, clk, data_out);
input [15:0] data_in;
input [2:0] writenum, readnum;
input write, clk;
output [15:0] data_out;

wire [7:0] load;
wire [7:0] decOut1, decOut2;
wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;

dec decWrite(writenum,decOut1);
assign load = {8{write}} & decOut1;

//initialize registers Reg0 through Reg7

vDFFE #(16) Reg0(clk, load[0], data_in, R0);
vDFFE #(16) Reg1(clk, load[1], data_in, R1);
vDFFE #(16) Reg2(clk, load[2], data_in, R2);
vDFFE #(16) Reg3(clk, load[3], data_in, R3);
vDFFE #(16) Reg4(clk, load[4], data_in, R4);
vDFFE #(16) Reg5(clk, load[5], data_in, R5);
vDFFE #(16) Reg6(clk, load[6], data_in, R6);
vDFFE #(16) Reg7(clk, load[7], data_in, R7);

//initialize first decoder used with readnum input

dec decRead(readnum, decOut2);

//initialize output mux

Mux8to1 muxOut(R0, R1, R2, R3, R4, R5, R6, R7, decOut2, data_out);

endmodule

//load enable register module

module vDFFE(clk, en, in, out);
parameter n = 1;
input clk, en;
input [n-1:0] in;
output [n-1:0] out;
reg [n-1:0] out;
wire [n-1:0] next_out;

assign next_out = en ? in : out;

//always block used to change out to next_out during a clock pulse

always @(posedge clk) begin
out = next_out;
end
endmodule

//decoder module

module dec(in,out);

input[2:0] in; 
output[7:0] out;

wire[7:0]out = 1 << in ;

endmodule

//multiplexer module

module Mux8to1(MUXR0, MUXR1, MUXR2, MUXR3, MUXR4, MUXR5, MUXR6, MUXR7, sel, MUXout);

input[15:0] MUXR0, MUXR1, MUXR2, MUXR3, MUXR4, MUXR5, MUXR6, MUXR7;
input[7:0] sel;
output[15:0] MUXout;
reg[15:0] MUXout;

//combinational always block used to select between register outputs based on select signal

always @(*) begin
case(sel)
8'b00000001 : MUXout = MUXR0;
8'b00000010 : MUXout = MUXR1;
8'b00000100 : MUXout = MUXR2;
8'b00001000 : MUXout = MUXR3;
8'b00010000 : MUXout = MUXR4;
8'b00100000 : MUXout = MUXR5;
8'b01000000 : MUXout = MUXR6;
8'b10000000 : MUXout = MUXR7;
default: MUXout = {15{1'b1}};
endcase
end
endmodule
