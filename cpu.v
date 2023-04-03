module cpu(clk, reset, read_data, write_data, N, V, Z, mem_cmd, mem_addr);

input clk, reset;
input[15:0] read_data;
output[15:0] write_data;
output N, V, Z;
output[1:0] mem_cmd;
output[8:0] mem_addr;

//internal signal declarations

wire[8:0] next_pc, PC, DA_out;
wire load_ir, load_addr, load_pc, addr_sel, reset_pc;

wire[15:0] OUTIR;
wire[2:0] opcode;
wire[1:0] op;
wire[1:0] ALUop;
wire[15:0] sximm5; 
wire[15:0] sximm8; 
wire[2:0] readnum; 
wire[2:0] writenum;
wire[2:0] nsel;
wire[1:0] shift;
wire[2:0] stat_out;
wire loada, loadb, asel, bsel, loadc, loads, write;
wire[1:0] vsel;

wire[15:0] mdata, C;

assign mdata = read_data;

assign Z = stat_out[2]; 
assign N = stat_out[0];
assign V = stat_out[1];
assign write_data = C;
assign mem_addr = addr_sel ? PC : DA_out;
assign next_pc = reset_pc ? 9'b0 : PC + 1'h1;

//intruction register declaration

vDFFE #(16) instructionReg(clk, load_ir, read_data, OUTIR);
vDFFE #(9) PC_reg(clk, load_pc, next_pc, PC); //figure out proper signals (esp clock/reset
vDFFE #(9) DA(clk, load_addr, write_data[8:0], DA_out);

FSM statemachine(clk, reset, reset_pc, addr_sel, mem_cmd, opcode, op , nsel, w, 
loada, loadb, loadc, loads, load_addr, load_ir, load_pc, asel, bsel, vsel, write);
instructionDec dec(OUTIR, sximm5, sximm8, readnum, writenum, nsel, ALUop, op, opcode, shift);

//datapath instance declaration

datapath DP(mdata, writenum, write, 
readnum, clk, loada, loadb, loadc, loads, shift, 
asel, bsel, vsel, ALUop, C, stat_out, sximm8, sximm5, PC);

endmodule

//controller module containing FSM that will dictate operation

module FSM(clk, reset, reset_pc, addr_sel, mem_cmd, opcode, op , nsel, w, 
loada, loadb, loadc, loads, load_addr, load_ir, load_pc, asel, bsel, vsel, write);

//signal declarations

output w; 
reg w;
input reset; 
input clk;
input [2:0] opcode;
input[1:0] op;

output[2:0] nsel;
reg [2:0] nsel;
output loada, loadb, asel, bsel, loadc, loads, write, load_ir, addr_sel, reset_pc, load_pc, load_addr;
reg loada, loadb, asel, bsel, loadc, loads, write, load_ir, addr_sel, reset_pc, load_pc, load_addr;
output[1:0] vsel, mem_cmd;
reg[1:0] vsel, mem_cmd;
reg[4:0] present_state;

//state definitions for ease of reference

`define RST 5'b00000
`define IF1 5'b00001
`define IF2 5'b00010
`define updatePC 5'b00011

`define decode 5'b00100

`define writeImm 5'b00101
`define getA 5'b00110
`define getB 5'b00111
`define ADD 5'b01000
`define CMP 5'b01001
`define AND 5'b01010
`define MVN 5'b01011
`define writeReg 5'b01100
`define MOV 5'b01101

`define LDR 5'b01110
`define STR 5'b01111
`define HALT 5'b10000
`define LoadAddr 5'b11111
`define STRNEXT  5'b10101
`define STRNEXT2 5'b11110

`define MNONE 00
`define MREAD 01
`define MWRITE 10

//clock dependent always block responsible for the changes in states of the FSM

always @ (posedge clk) begin

//set state back to waiting if reset is hit

if(reset) begin
present_state = `RST;
load_pc = 1'b1;
reset_pc = 1'b1;
end
else 

//case statement for present_state

case(present_state)

`RST: begin
present_state = `IF1;
vsel = 2'b00;
loada = 1'b0;
loadb = 1'b0;
asel = 1'b0;
bsel = 1'b0;
loadc = 1'b0;
loads = 1'b0;
write = 1'b0;
nsel = 3'b000;
w = 1'b1;
mem_cmd = 2'b00;
load_ir = 1'b0;
load_addr = 1'b0;
addr_sel = 1'b0;
end

`IF1: begin   // IF1 sate and signals
load_pc = 1'b0;
reset_pc = 1'b0;
w = 1'b0;
addr_sel = 1'b1;
vsel = 2'b00;
loada = 1'b0;
loadb = 1'b0;
asel = 1'b0;
bsel = 1'b0;
loadc = 1'b0;
loads = 1'b0;
write = 1'b0;
nsel = 3'b000;
mem_cmd = `MREAD;
present_state = `IF2;
end

`IF2: begin    // IF2 state and signals 
w = 1'b0;
load_ir = 1'b1;
present_state = `updatePC;
end

`updatePC: begin   // updatePC state and signals 
load_pc = 1'b1;
load_ir = 1'b0;
addr_sel = 1'b0;
present_state = `decode;
mem_cmd = `MNONE;
end

//instruction decoder state

`decode : begin
load_pc = 1'b0; 
w = 1'b0; 

if({opcode, op} == 5'b11010)
present_state = `writeImm;

else if (({opcode,op} == 5'b11000) | ({opcode,op} == 5'b10111))
present_state = `getB;

else if ({opcode, op}== 5'b11100)
present_state = `HALT;

else
present_state = `getA;
end

`HALT: begin    // Halt goes back to halt
  present_state = `HALT;
end

//MOV immediate value into register

`writeImm: begin
nsel = 3'b100; 
vsel = 2'b01; 
write = 1'b1;
present_state = `IF1;
end 

//load first register value into enable register A

`getA: begin
nsel = 3'b000; 
loada = 1'b1;
present_state = `getB;
end 

//load second reigster value into enable register B

`getB: begin
nsel = 3'b010; 
loada = 1'b0;
loadb = 1'b1;

if ({opcode,op} == 5'b10100) 
present_state = `ADD;
      
else if ({opcode,op} == 5'b10101)  
present_state = `CMP;
             
else if ({opcode,op} == 5'b10110) 
present_state = `AND;

else if ({opcode,op} == 5'b10111) 
present_state = `MVN;
               
else if ({opcode,op} == 5'b11000)
present_state = `MOV;

else if ({opcode,op} == 5'b01100) 
present_state = `ADD;
                
else if ({opcode,op} == 5'b10000)
present_state = `ADD;
end

`ADD: begin

if ({opcode,op} == 5'b01100) begin   // LDR logic
asel = 1'b0; 
bsel = 1'b1; 
loadb = 1'b0;
loadc = 1'b1;
present_state = `LoadAddr;
end

else if ({opcode,op} == 5'b10000) begin //STR LOGIC
asel = 1'b0; 
bsel = 1'b1; 
loadb = 1'b0;
loadc = 1'b1;
present_state = `LoadAddr;
end

else begin     // else we're going to writereg
asel = 1'b0;
bsel = 1'b0; 
loadb = 1'b0;
loadc = 1'b1;
loads = 1'b1;
present_state = `writeReg;
end
end

`LoadAddr: begin
load_addr = 1'b1;
addr_sel = 1'b0; 
loadc = 1'b0;
if({opcode, op} == 5'b01100) present_state = `LDR;
else if({opcode, op} == 5'b10000) present_state = `STR;
end


//set signals for the comparison of two values

`CMP: begin
asel = 1'b0; 
bsel = 1'b0; 
loadb = 1'b0;
loadc = 1'b1;
loads = 1'b1; 
present_state = `writeReg;
end

//set signals for the bitwise AND of two values

`AND: begin
asel = 1'b0; 
bsel = 1'b0; 
loadb = 1'b0;
loadc = 1'b1;
loads = 1'b1;
present_state = `writeReg;
end

//set signals for the bitwise compliment of a single value 

`MVN: begin
asel = 1'b1; 
bsel = 1'b0; 
loadb = 1'b0;
loadc = 1'b1;
loads = 1'b1;
present_state = `writeReg;
end

//set signals for the movement of a register value into another register

`MOV: begin
asel = 1'b1; 
bsel = 1'b0; 
loadb = 1'b0;
loadc = 1'b1;
present_state = `writeReg; 
end

`LDR: begin       // LDR STATE AND SIGNALS
mem_cmd = `MREAD;
load_addr = 1'b0;
present_state = `writeReg;
end

// STR states 
`STR: begin
load_addr = 1'b0;
nsel = 3'b001; 
loadb = 1'b1;
present_state = `STRNEXT;
end

`STRNEXT:begin
asel = 1'b1;
bsel = 1'b0;
loadb = 1'b0;
loadc = 1'b1;
present_state = `STRNEXT2;
end

`STRNEXT2: begin
mem_cmd = `MWRITE;
present_state = `IF1;
end


//write computed result back into memory of declared output register

`writeReg: begin
loads = 1'b0;

if ({opcode,op} == 5'b01100) begin
nsel = 3'b101; 
vsel = 2'b00; 
write = 1'b1;                   
present_state = `IF1;
end

else begin
nsel = 3'b101; 
vsel = 2'b11; 
write = 1'b1;
present_state = `IF1;
end
end

default: begin
present_state = `IF1;
end
endcase
end
endmodule

//module instantiation of the instruction decoder used in the CPU

module instructionDec(OUTIR, sximm5, sximm8, readnum, 
writenum, nsel, ALUop, op, opcode, shift);

input[15:0] OUTIR; 
output[1:0] shift;
output[1:0] ALUop, op;
output[2:0] opcode;

input[2:0] nsel;

output[15:0] sximm8;
output[15:0] sximm5;

reg[15:0] sximm8;
reg[15:0] sximm5;

output[2:0] writenum;
output[2:0] readnum;

wire[2:0] Rn = OUTIR[10:8];
wire[2:0] Rd = OUTIR[7:5];
wire[2:0] Rm = OUTIR[2:0];
wire[7:0] imm8 = OUTIR[7:0];
wire[4:0] imm5 = OUTIR[4:0];

//instance of a 3 to 1 mux used to select between registers in the instruction

Mux3to1 regMux(Rn, Rd, Rm, writenum, readnum, nsel);

always @ (*) begin

//sign extension of imm8

if((OUTIR[15:13] !== 3'b011) && (OUTIR[15:13] !== 3'b100)) begin 

if(imm8[7] == 1'b1) begin
sximm8 = {{8{1'b1}}, imm8};
sximm5 = {16{1'bx}};
end
else begin
sximm8 = {{8{1'b0}}, imm8};
sximm5 = {16{1'bx}};
end

end
//sign extension of imm5
else begin

if(imm5[4] == 1'b1) begin
sximm5 = {{11{1'b1}}, imm5};
sximm8 = {16{1'bx}};
end
else begin
sximm5 = {{11{1'b0}}, imm5};
sximm8 = {16{1'bx}};
end
end
end

assign op = OUTIR[12:11];
assign ALUop = OUTIR[12:11];
assign opcode = OUTIR[15:13];
assign shift = OUTIR[4:3];
endmodule

//module instantiation of the 3 to 1 mmux used in the instruction decoder 

module Mux3to1(in1, in2, in3, writenum, readnum, sel);

input[2:0] in1, in2, in3;
input[2:0] sel;
output[2:0] writenum;
output[2:0] readnum;

reg[2:0] writenum;
reg[2:0] readnum;

//combinational always block used to select between reading and writing of certain registers as determined by FSM

always @ (*)
case(sel)
3'b100: begin
writenum = in1;
readnum = {3{1'bx}};
end
3'b000: begin 
writenum = {3{1'bx}};
readnum = in1;
end
3'b101: begin
writenum = in2;
readnum = {3{1'bx}};
end
3'b001: begin
writenum = {3{1'bx}};
readnum = in2;
end
3'b110: begin
writenum = in3;
readnum = {3{1'bx}};
end
3'b010: begin
writenum = {3{1'bx}};
readnum = in3;
end
default: begin
writenum = {3{1'bx}};
readnum = {3{1'bx}};
end
endcase
endmodule
