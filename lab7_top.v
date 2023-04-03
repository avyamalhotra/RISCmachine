module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

//signal definitions for memory commands

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

//signal to blank out the hex displays

`define blank 7'b1111111

assign HEX0 = `blank;
assign HEX1 = `blank;
assign HEX2 = `blank;
assign HEX3 = `blank;
assign HEX4 = `blank;

//internal signal declarations

wire clk, reset;
wire msel_read, write, w;
wire[1:0] mem_cmd; 
wire[8:0] mem_addr;
wire[7:0] read_address, write_address;
wire[15:0] din, dout, read_data, write_data;
wire N, V, Z, outputLoad;
assign LEDR[9] = w; 

//clock and reset inputs

assign clk = ~KEY[0];
assign reset = ~KEY[1];

//status flags

assign HEX5[0] = ~Z;
assign HEX5[6] = ~N;
assign HEX5[3] = ~V;

assign read_address = mem_addr[7:0];
assign write_address = mem_addr[7:0];

//memory declaration for RAM

RAM #(16,8) MEM(clk, read_address, write_address, write, din, dout); 
cpu CPU(clk, reset, read_data, write_data, N, V, Z, mem_cmd, mem_addr);

//combinational logic for reading and writing from the RAM

assign msel_read = ((mem_cmd == `MREAD) && (mem_addr[8] == 1'b0));
assign read_data = msel_read ? dout : {16{1'bz}};
assign write = ((mem_cmd == `MWRITE) && (mem_addr[8] == 1'b0));
assign din = write ? write_data : {16{1'bz}};

//interfacing commands with De1-SoC

wire switch_read;

vDFFE #(8) outputReg(clk, outputLoad, write_data[7:0], LEDR[7:0]);
assign outputLoad = ((mem_cmd == `MWRITE) && (mem_addr == 9'h100));

assign switch_read = ((mem_addr == 9'h140) && (mem_cmd == `MREAD));
assign read_data[15:8] = switch_read ? 8'b0 : {8{1'bz}};
assign read_data[7:0] = switch_read ? SW[7:0] : {8{1'bz}};
  
endmodule   

//module declaration for the RAM module (given in slide set 11

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 32; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule
