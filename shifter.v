//shifter module used in data path

module shifter(in, shift, sout);

input[15:0] in;
input[1:0] shift;
output[15:0] sout;
reg[15:0] sout;

//combinational always block used to select between four 
//shifting options for output from B load enable register 

always@(*)
case(shift)
2'b00: sout = in;
2'b01: sout = in << 1;
2'b10: sout = in >> 1;
2'b11: sout = {in[15], in[15:1]};
endcase
endmodule
