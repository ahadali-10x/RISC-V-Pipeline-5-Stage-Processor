`timescale 1ns / 1ps


module main(rst, clk, Anode_Activate, LED_out);
input rst, clk;						// 1 button to reset, clock signal as input
output reg[7:0] Anode_Activate;		// Anodes to control 7-segments
output reg [6:0] LED_out;			// output result to be sent on 7-segments
wire [31:0] pc;
wire [31:0]Result;
wire [31:0]Result1;
 

	// ALL modules will be called in this file. Code will be executed and results will be shown on 7-segment display
// Code segment for BCD to 7-segment Decoder. Keep this code as it is
reg [31:0] counter;		// A 32 bit flashing counter
reg toggle = 0;			// A variable to toggle between two 7-segments 

always @(posedge clk)
    begin
            if(counter>=100000) begin
                 counter <= 0;
				 toggle = ~toggle; end
            else begin
                counter <= counter + 1;
				end
    end 
    // anode activating signals for 8 segments, digit period of 1ms
    // decoder to generate anode signals 
    always @(*)
    begin
        case(toggle)
        1'b0: begin
            Anode_Activate = 8'b01111111; 
            // activate SEGMENT1 and Deactivate all others
              end
        1'b1: begin
            Anode_Activate = 8'b10111111; 
            // activate LED2 and Deactivate all others    
               end
        endcase
    end
    // Cathode patterns of the 7-segment 1 LED display 
    always @(*)
    begin
	if (toggle) begin
        case(Result1[3:0])				// First 4 bits of Result from ALU will be displayed on 1st segment
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1010: LED_out = 7'b0001000; // "A"     
        4'b1011: LED_out = 7'b1100000; // "b"     
        4'b1100: LED_out = 7'b0110001; // "C"     
        4'b1101: LED_out = 7'b1000010; // "d"     
        4'b1110: LED_out = 7'b0110000; // "E"     
        4'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
		end
    

	// Cathode patterns of the 7-segment 2 LED display
if(!toggle) begin	
        case(Result1[7:4])			// Next 4 bits of Result from ALU will be displayed on 2nd segment
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1010: LED_out = 7'b0001000; // "A"     
        4'b1011: LED_out = 7'b1100000; // "b"     
        4'b1100: LED_out = 7'b0110001; // "C"     
        4'b1101: LED_out = 7'b1000010; // "d"     
        4'b1110: LED_out = 7'b0110000; // "E"     
        4'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
    end
end

	
wire [31:0]PC_in;

wire [31:0]pcD;
wire [31:0]pcE;
wire [31:0]pcM;

wire [31:0]instruction;
wire [31:0]inst1;
wire [4:0]rs1;
wire [4:0]rs2;
wire [4:0]rs1E;
wire [4:0]rs2E;
wire [4:0]rd;
wire [31:0] RD1E;
wire [31:0] RD2E;
wire [31:0] RD2M;
wire [4:0]rdE;
wire [4:0]rdM;
wire [4:0]rdW;

wire [31:7]immBitField;
wire [6:0]opCode;
wire [6:0]funct7;
wire [2:0]funct3;


wire [1:0]resultSrcE;
wire memWrE;
wire aluSrcE;
wire pcSrcE;
wire regWrE;
wire memReadE;
wire [3:0]aluControlE;
wire [31:0]ALUResultE;

wire [1:0]resultSrcM;
wire memWrM;
wire regWrM;
wire memReadM;
wire [31:0]aluResultM;
wire readDataM;

wire [1:0]resultSrcW;
wire [31:0]resultW;
wire regWrW;
wire [31:0]aluResultW;
wire [31:0]readDataW;

wire [31:0]aluResult;
wire zeroFlag;
wire [31:0]srcA;
wire [31:0]srcAE;
wire [31:0]srcBE;
wire [31:0]Op_B;
wire [3:0]aluControl;

wire [31:0]srcB;
wire [31:0]result;
wire regWr;

wire [31:0]dataOut;
wire memRead;
wire memWr;

wire [31:0]immext;
wire [31:0]immextE;
wire [1:0]immSrc;

wire [1:0]resultSrc;

wire aluSrc;

wire [31:0]pcPlus4;
wire [31:0]pcPlus4D;
wire [31:0]pcPlus4E;
wire [31:0]pcPlus4M;
wire [31:0]pcPlus4W;

wire [31:0]pcTarget;
wire [31:0]pcNext;
wire pcSrc;
wire negFlag;
wire branch;
wire jump;
wire branchE;
wire jumpE;

wire [1:0]frwA;
wire [1:0]frwB;
wire StallF;
wire StallD;
wire FlushE;
wire FlushD;
wire ENf;
wire EN;
wire CLR;
wire CLRE;
wire and1;

mux02 #(32) x9(.d0(pcPlus4), .d1(pcTarget), .y(pcNext), .s(pcSrcE));
adress_generator  x12(.pc(pc), .pcNext(pcNext), .clk(clk), .rst(rst),.ENf(StallF));
Instruction_Memory  x13(.pc(pc), .instruction(instruction));
clkRegister1 a14(.clk(clk),.instruction(instruction),.pc(pc),.pcPlus4(pcPlus4),.inst1(inst1),.pcD(pcD),.pcPlus4D(pcPlus4D),.StallD(StallD),.CLR(FlushD));
Instruction_fetch  x1(.instruction(inst1),.rs1(rs1), .rs2(rs2), .rd(rd), .immBitField(immBitField), .opCode(opCode), .funct7(funct7), .funct3(funct3));
register_file x4(.srcA(srcA),.srcB(srcB), .result(result), .rs1(rs1), .rs2(rs2), .rdW(rdW), .regWrW(regWrW), .rst(rst),.clk(clk),.GCD_Result(Result1));
clkRegister2 a15(.clk(clk),.CLRE(FlushE),.rs1(rs1), .rs2(rs2),.resultSrc(resultSrc),.branch(branch),.jump(jump),.pcSrc(pcSrc),.memWr(memWr),.aluSrc(aluSrc),.regWr(regWr),.memRead(memRead),.aluControl(aluControl),.srcA(srcA),.srcB(srcB),.pcD(pcD),.rd(rd),.immext(immext),.branchE(branchE), .jumpE(jumpE),.pcPlus4D(pcPlus4D),.rs1E(rs1E), .rs2E(rs2E),.resultSrcE(resultSrcE),.memWrE(memWrE),.aluSrcE(aluSrcE),.regWrE(regWrE),.memReadE(memReadE),.aluControlE(aluControlE),.RD1E(RD1E),.RD2E(RD2E),.pcE(pcE),.rdE(rdE),.immextE(immextE),.pcPlus4E(pcPlus4E));
mux01 #(32) x15(.d0(RD1E), .d1(result),.d2(aluResultM), .y(srcAE), .s(frwA));
mux01 #(32) x16(.d0(RD2E), .d1(result),.d2(aluResultM), .y(srcBE), .s(frwB));
mux02 #(32) x8(.d0(srcBE), .d1(immextE), .y(Op_B), .s(aluSrcE));
ALU x3(.aluResult(aluResult), .zeroFlag(zeroFlag),.negFlag(negFlag), .srcA(srcAE), .Op_B(Op_B), .aluControlE(aluControlE));
clkRegister3 a16(.clk(clk),.resultSrcE(resultSrcE),.memWrE(memWrE),.regWrE(regWrE),.memReadE(memReadE),.aluResult(aluResult),.RD2E(RD2E),.rdE(rdE),.pcPlus4E(pcPlus4E),.resultSrcM(resultSrcM),.memWrM(memWrM),.regWrM(regWrM),.memReadM(memReadM),.aluResultM(aluResultM),.RD2M(RD2M),.rdM(rdM),.pcPlus4M(pcPlus4M));
Data_Memory x5(.dataOut(dataOut),.aluResult(aluResultM), .srcB(RD2E), .memReadM(memReadM), .memWrM(memWrM), .rst(rst),.clk(clk));
clkRegister4 a17(.clk(clk),.resultSrcM(resultSrcM),.regWrM(regWrM),.aluResultM(aluResultM),.readDataM(dataOut),.rdM(rdM),.pcPlus4M(pcPlus4M),.resultSrcW(resultSrcW),.regWrW(regWrW),.aluResultW(aluResultW),.readDataW(readDataW),.rdW(rdW),.pcPlus4W(pcPlus4W));

//branch a11(.branch(branch),.jump(jump),.zeroFlag(zeroFlag),.negFlag(negFlag),.pcSrcE(pcSrcE));
controller  x2(.opCode(opCode),.funct7(funct7),.funct3(funct3),.zeroFlag(zeroFlag),.negFlag(negFlag),.branch(branch), .jump(jump),.pcSrc(pcSrc),.resultSrc(resultSrc),.aluSrc(aluSrc),.memWr(memWr),.aluControl(aluControl),.immSrc(immSrc),.regWr(regWr),.memRead(memRead));
HazardControl a21(.rs1E(rs1E),.rs2E(rs2E),.rs1(rs1),.rs2(rs2),.rdW(rdW),.rdM(rdM),.rdE(rdE),.regWrM(regWrM),.regWrW(regWrW),.resultSrcE(resultSrcE),.pcSrcE(pcSrcE),.frwA(frwA),.frwB(frwB),.StallF(StallF),.StallD(StallD),.FlushE(FlushE),.FlushD(FlushD));
Signextend x6(.immext(immext),.immBitField(immBitField), .immSrc(immSrc));
mux01 #(32) x7(.d0(aluResultW), .d1(readDataW),.d2(pcPlus4W), .y(result), .s(resultSrcW));

adder x10(.a(pc),.b(32'h00000004),.y(pcPlus4));
adder x11(.a(pcE),.b(immextE),.y(pcTarget));

and a18(and1,branchE,zeroFlag);
or a20(pcSrcE,jumpE,and1);


endmodule

module clkRegister1(input clk,input [31:0]instruction,input [31:0]pc,input [31:0]pcPlus4,input CLR,input StallD, output reg [31:0]inst1,output reg [31:0]pcD,output reg [31:0]pcPlus4D);


always @ (posedge clk)begin
if (CLR ==1'b1)begin
inst1[31:0] <= 0;
pcD[31:0] <=0;
pcPlus4D[31:0] <= 0;
end
else if(StallD==1'b1)begin
inst1[31:0] <= inst1[31:0];
pcD[31:0] <= pcD[31:0];
pcPlus4D[31:0] <= pcPlus4D[31:0];
end
else begin
inst1[31:0] <= instruction[31:0];
pcD[31:0] <= pc[31:0];
pcPlus4D[31:0] <= pcPlus4[31:0];
end
end
endmodule

 module clkRegister2(input clk,input CLRE,input branch,input jump,input [1:0]resultSrc,input memWr,input [4:0]rs1,input [4:0]rs2,input aluSrc,input pcSrc,input regWr,input memRead,input [3:0]aluControl,input [31:0]srcA,input [31:0]srcB,input [31:0]pcD,input [4:0]rd,input [31:0]immext,input [31:0]pcPlus4D,output reg branchE,output reg jumpE,output reg [4:0]rs1E,output reg [4:0]rs2E,output reg [1:0]resultSrcE,output reg memWrE,output reg aluSrcE,output reg regWrE,output reg memReadE,output reg [3:0]aluControlE,output reg [31:0]RD1E,output reg [31:0]RD2E,output reg [31:0]pcE,output reg [4:0]rdE,output reg [31:0]immextE,output reg [31:0]pcPlus4E);

always @ (posedge clk)begin
if (CLRE ==1'b1) begin
RD1E <= 0;
RD2E <= 0;
pcE <= 0;
rs1E <= 0;
rs2E <= 0;
rdE <= 0;
immextE[31:0] <= 0;
pcPlus4E[31:0] <= 0;
//pcSrcE <= 0;
resultSrcE <= 0;
memWrE <= 0;
aluSrcE <= 0;
regWrE <= 0;
memReadE <= 0;
aluControlE[3:0] <= 0;
branchE <= 0;
jumpE <= 0;
end
else begin 

RD1E[31:0]<=srcA[31:0];
RD2E[31:0]<=srcB[31:0];
pcE[31:0]<=pcD[31:0];
rs1E <= rs1;
rs2E <= rs2; 
rdE[4:0]<= rd;
immextE[31:0]<=immext[31:0];
pcPlus4E[31:0]<=pcPlus4D[31:0];
//pcSrcE<=pcSrc;
resultSrcE[1:0]<=resultSrc[1:0];
memWrE<=memWr;
aluSrcE<=aluSrc;
regWrE<=regWr;
memReadE<=memRead;
aluControlE[3:0]<=aluControl[3:0];
branchE <= branch;
jumpE <= jump;
end
end
endmodule

module clkRegister3(input clk,input [1:0]resultSrcE,input memWrE,input regWrE,input memReadE,input [31:0]aluResult,input [31:0]RD2E,input [11:7]rdE,input [31:0]pcPlus4E,output reg [1:0]resultSrcM,output reg memWrM,output reg regWrM,output reg memReadM,output reg [31:0]aluResultM,output reg [31:0]RD2M,output reg [11:7]rdM,output reg [31:0]pcPlus4M);
always @ (posedge clk)begin
aluResultM[31:0]<=aluResult[31:0];
RD2M[31:0]<=RD2E[31:0];
rdM<=rdE;
pcPlus4M[31:0]<=pcPlus4E[31:0];
resultSrcM[1:0]<=resultSrcE[1:0];
memReadM<=memReadE;
memWrM<=memWrE;
regWrM<=regWrE;
end
endmodule

module clkRegister4(input clk,input [1:0]resultSrcM,input regWrM,input [31:0]aluResultM,input [31:0]readDataM,input [11:7]rdM,input [31:0]pcPlus4M,output reg [1:0]resultSrcW,output reg regWrW,output reg [31:0]aluResultW,output reg [31:0]readDataW,output reg [11:7]rdW,output reg [31:0]pcPlus4W);

always @ (posedge clk)begin
aluResultW[31:0]<=aluResultM[31:0];
readDataW[31:0]<=readDataM[31:0];
rdW<=rdM;
pcPlus4W[31:0]<=pcPlus4M[31:0];
resultSrcW[1:0]<=resultSrcM[1:0];
regWrW<=regWrM;
end
endmodule

module adress_generator (output reg [31:0] pc,input [31:0] pcNext,input clk, input rst,input ENf);	

	initial begin
	pc <= 0;
	end
	always @ (posedge clk or posedge rst)
	begin
	if(ENf==1'b1)begin
	   pc<=pc;
	   end	
		if(rst==1'b1)
			pc<=0;
		else
			pc<=pcNext;
	end

endmodule


//INSTRUCTION MEMORY MODULE
module Instruction_Memory (input [31:0] pc,output reg [31:0] instruction);

    always@(pc)
        case(pc) 
/*32'h00: instruction = 32'h00000000;
32'h04: instruction = 32'h00700413;
32'h08: instruction = 32'h00500493;
32'h0c: instruction = 32'h00940c63;
32'h10: instruction = 32'h00944663;
32'h14: instruction = 32'h40940433;
32'h18: instruction = 32'hff5ff06f;
32'h1c: instruction = 32'h408484b3;
32'h20: instruction = 32'hfedff06f;
32'h24: instruction = 32'h0000006f;


32'h00: instruction = 32'h00100093;//addi x1 , x0 , 1
32'h04: instruction = 32'h00200113;//addi x2 , x0 , 2
32'h08: instruction = 32'h00300193;//addi x3 , x0 , 3
32'h0c: instruction = 32'h00400213;//addi x4 , x0 , 4
32'h10: instruction = 32'h401102b3;//sub  x5 , x2 , x1
32'h14: instruction = 32'h00508313;//add  x6 , x1 , 5
32'h18: instruction = 32'h402183b3;//sub  x7, x3 , x2
32'h1c: instruction = 32'h00138413;//addi x8,x7,1
32'h20: instruction = 32'h407404b3;//sub x9,x8,x7
32'h24: instruction = 32'h00940533;//sub x10,x8,x9

default: instruction = 32'h00000000;*/
32'h00: instruction = 32'h00100093;//addi x1 , x0 , 1
32'h04: instruction = 32'h00200113;//addi x2 , x0 , 2
32'h08: instruction = 32'h00300193;//addi x3 , x0 , 3
32'h0c: instruction = 32'h00400213;//addi x4 , x0 , 4
32'h10: instruction = 32'h401102b3;//sub  x5 , x2 , x1
32'h14: instruction = 32'h00508313;//add  x6 , x1 , 5
32'h18: instruction = 32'h402183b3;//sub  x7, x3 , x2
32'h1c: instruction = 32'h00138413;//addi x8,x7,1
32'h20: instruction = 32'h407404b3;//sub x9,x8,x7
32'h24: instruction = 32'h40940533;//sub x10,x8,x9
32'h28: instruction = 32'h0030a583;//lw x11,3(x1)
32'h2c: instruction = 32'h00558613;//addi x12 , x11 , 5
32'h30: instruction = 32'h00860693;//addi x13 , x12 , 8
32'h34: instruction = 32'h00368713;//addi x14 , x13 , 3
32'h38: instruction = 32'h00208663;//beq x1,x2,stop
32'h3c: instruction = 32'h40110133;//sub x2,x2,x1
32'h40: instruction = 32'hfc9ff06f;//j label
32'h44: instruction = 32'h0000006f;//j stop
default: instruction = 32'h00000000;
        endcase

endmodule

//INSTRUCTION FETCH MODULE
module Instruction_fetch (input [31:0] instruction, output reg [4:0]rs1, output reg [4:0]rs2, output reg [4:0]rd, 
output reg[31:7]immBitField,output reg [6:0]opCode,output reg [6:0]funct7,output reg [2:0]funct3);

    always@(*)begin
        opCode      <= instruction[6:0];
        funct3      <= instruction[14:12];
        funct7      <= instruction[31:25];
        immBitField <= instruction[31:7];
        rs1         <= instruction[19:15];
        rs2         <= instruction[24:20];
        rd          <= instruction[11:7];
        end

endmodule

//REGISTER FILE
module register_file(output [31:0] srcA, output [31:0] srcB, input [31:0] result,input rst,clk, 
                     input [4:0] rs1, input [4:0] rs2, input [4:0] rdW, input regWrW,output  [31:0] GCD_Result);
			
    integer i;	
	reg [31:0] Reg_File [31:0];				
	assign srcA = (rs1 != 0) ? Reg_File[rs1] : 0;
	assign srcB = (rs2 != 0) ? Reg_File[rs2] : 0;
    assign GCD_Result = Reg_File[8];
     
    always @(negedge clk or posedge rst) begin
        Reg_File[0] <=0;
        if(rst==1'b1)
        for (i = 1; i < 32; i = i + 1)
            Reg_File[i] <= 0;
        else if(regWrW==1'b1)
            Reg_File[rdW] <=result;
        end	
endmodule

//ALU
module ALU(output reg [31:0] aluResult,output zeroFlag,output negFlag, input [31:0] srcA, input [31:0] Op_B, input [3:0] aluControlE);

    wire [31:0] tmp;
    assign tmp = srcA - Op_B;
    assign zeroFlag = (tmp == 0)?1:0;
    assign negFlag = tmp[31];
    always @(*)begin
        case(aluControlE)
            4'b0000 : aluResult = srcA + Op_B;
            4'b0001 : aluResult = srcA - Op_B;
            4'b0010 : aluResult = srcA * Op_B;
            4'b0011 : aluResult = srcA / Op_B;
            4'b0100 : aluResult = srcA << Op_B;
            4'b0110 : aluResult = srcA >> Op_B;
            4'b0111 : aluResult = srcA & Op_B;
            4'b1000 : aluResult = srcA | Op_B;
            4'b1001 : aluResult = srcA ^ Op_B;
            4'b1010 : aluResult = ~(srcA | Op_B);
            default : aluResult = srcA + Op_B;
        endcase
    end
endmodule

//DATA MEMORY
module Data_Memory(output [31:0] dataOut,input memReadM,clk,rst ,input [31:0] srcB, input [31:0] aluResult, input memWrM);
     integer i;
    reg [31:0] Mem [255:0];			
    assign dataOut = memReadM? Mem[aluResult]:32'bx;
    always @(posedge clk) begin
       if (rst == 1'b1) begin
		  for (i=0; i<256; i=i+1) begin
			Mem[i] = 32'b0;
				end
			end
        Mem[4] = 32'h9;
        if (memWrM)
            Mem[aluResult] = srcB;
        end
       	
endmodule

//SIGN EXTENSION
module Signextend(input [31:7] immBitField,input [1:0] immSrc,output reg[31:0] immext);
    always@(immSrc or immBitField)
      case(immSrc)
               
        2'b00:     immext = {{20{immBitField[31]}}, immBitField[31:20]};                     // I-type		 
        2'b01:     immext = {{20{immBitField[31]}}, immBitField[31:25], immBitField[11:7]};  // S-type (stores) 
        2'b10:      immext = {{20{immBitField[31]}}, immBitField[7],  immBitField[30:25], immBitField[11:8], 1'b0}; // B-type (branches)     // J-type (jal)
	    2'b11:      immext = {{12{immBitField[31]}}, immBitField[19:12],  immBitField[20], immBitField[30:21], 1'b0};// J-type (branches)
           
	    default: 	immext = 32'bx; // undefined
       endcase
endmodule

//MUX
module mux02 #(parameter WIDTH = 32)(input [WIDTH-1:0] d0, d1,input s,output [WIDTH-1:0] y);
     assign y = s ? d1 : d0;
endmodule

//MUX
module mux01 #(parameter WIDTH = 32)(input [WIDTH-1:0] d0, d1,d2,input[1:0] s,output reg [31:0] y);
     always @(*)begin
     case(s)
     2'b00: y <= d0;
     2'b01: y <= d1;
     2'b10: y <= d2;
     endcase
     end
endmodule

//ADDER
module adder(input   [31:0] a, b,output  [31:0] y);
     assign y = a + b;
endmodule

/*module branch(input branch,input jump,input zeroFlag,input negFlag,output reg pcSrcE);
        initial begin 
        pcSrcE<=1'b0;
       end
       always@(branch or jump or zeroFlag or negFlag)begin
       
        pcSrcE = jump||(branch&&zeroFlag)||(branch&&negFlag);
      // pcSrcE<=0;
       end*/
       
       /*always@(branchE or jumpE or zeroFlagE or negFlagE)begin
      
       if(branchE||jumpE)
       begin 
       pcSrcE<=1'b1;
       end
       else 
        pcSrcE<=1'b0;
       end*/
       //endmodule

//CONTROLLER
module controller( input zeroFlag,negFlag,input [6:0]opCode,input [2:0]funct3,input [6:0]funct7,output reg pcSrc,output reg branch,output reg jump,output reg [1:0]resultSrc,output reg memWr,output reg aluSrc,output reg regWr,output reg memRead,output reg [1:0] immSrc,output reg [3:0]aluControl);
 
always @(*) begin

    aluSrc <= 1'b0;
    regWr  <= 1'b0;
    resultSrc <=1'b0;
    pcSrc <=1'b0;
    memWr <= 1'b0;
    memRead <=1'b0;
    immSrc <= 2'b00; 
    branch <= 1'b0;
    jump   <=  1'b0;
    
    case(opCode)
    7'b0110011 : begin   // R_Type  
    regWr <=1'b1;
    end  
    7'b0000011 : begin  // I_Type load
    
    regWr <=1'b1;
    aluSrc <= 1'b1;
    immSrc <= 2'b00; 
    resultSrc <= 1'b1;
    memRead <=1'b1;
    end
    7'b0010011 : begin  // I-Type 
   
    regWr <=1'b1;
    aluSrc <= 1'b1; 
        
    end
    7'b1100111 : begin  // jalr
    
    regWr <=1'b1; 
    aluSrc <= 1'b1;
    pcSrc = 1'b1;  
    end
    7'b1100011 : begin  // B_Type 
     
    immSrc <= 2'b10;
    branch <= 1'b1;
    case(funct3)
    3'b000: pcSrc <=  zeroFlag; //BEQ
    3'b001: pcSrc <=  ~zeroFlag; //BNE
    3'b100: pcSrc <= negFlag;  //BLT
    3'b101: pcSrc <= ~negFlag || zeroFlag;  
    endcase
    end
     
    7'b0100011 : begin  //S_Type 
    
    memWr <= 1'b1;
    aluSrc <= 1'b1;
    immSrc =2'b01;
    end   
    7'b1101111 : begin  // J_Type 
   
    regWr <=1'b1;
    aluSrc <= 1'b1;
    immSrc <= 2'b11;
    pcSrc = 1'b1;  
    jump <= 1'b1;
    end
   endcase 
  end  
    
    always @(*)
    begin
        casex({opCode,funct3,funct7})
        17'b0110011_000_0000000  : aluControl = 4'b0000;   //ADD
        17'b0010011_000_xxxxxxx  : aluControl = 4'b0000;   //ADDI
        17'b0000011_010_xxxxxxxx : aluControl = 4'b0000;  // LW
        17'b0010011_001_0000000  : aluControl = 4'b0100;  //slli
        17'b0010011_100_0000000  : aluControl = 4'b1010;  // xor
        17'b0010011_101_0000000  : aluControl = 4'b0101;  // shift right
        17'b0010011_110_0000000  : aluControl = 4'b1001;  // or
        17'b0010011_111_0000000  : aluControl = 4'b1000;  //and
        17'b0110011_000_0100000  : aluControl = 4'b0001;   //sub
        17'b1100011_000_xxxxxxx  : aluControl = 4'b0001;   // beq
        default: aluControl =4'b0000;
        endcase
    
    end
endmodule

module HazardControl(input [4:0]rs1E,input [4:0]rs2E,input [4:0]rs1,input [4:0]rs2,input [11:7]rdW,input [11:7]rdM,input [11:7]rdE,input regWrM,input regWrW,input resultSrcE,input pcSrcE,output reg StallF,output reg StallD,output reg FlushE,output reg FlushD,output reg[1:0]frwA,output reg[1:0]frwB);

reg lwstall;
initial begin
lwstall<=0;
end

always@(*)begin


 lwstall <= (rdE==rs1||rdE==rs2)&&resultSrcE==1'b1;
 StallF<=lwstall;
 StallD<=lwstall;
 FlushE <=lwstall||pcSrcE;
 FlushD <=pcSrcE;
end
always@(*)begin

if(((rs1E==rdM)&&regWrM)&&rs1E!=0)begin
frwA<=2'b10;
end
else if(((rs1E==rdW)&&regWrW)&&rs1E!=0)begin
frwA<=2'b01;
end
else begin
frwA<=2'b00;
end
if(((rs2E==rdM)&&regWrM)&&rs2E!=0)begin
frwB<=2'b10;
end
else if(((rs2E==rdW)&&regWrW)&&rs2E!=0)begin
frwB<=2'b01;
end
else begin
frwB<=2'b00;
end
end
endmodule