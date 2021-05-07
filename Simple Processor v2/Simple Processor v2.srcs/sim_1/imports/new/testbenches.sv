/**
* Title : Simple Processor - Term Project
* Author : Zubeyir Bodur
* ID: 21702382
* Section : 4
* Description : Testbenches for the processor
*/

`timescale 1ns / 1ps

module prioTest();
    logic [1:0] Q;
    logic [3:0] D;
    integer i;
    priorityEncoder uut(.Q(Q),
                        .D(D));
    initial begin
        D = 0;
        for (i = 0; i < 16; i = i + 1) begin
            #1 D = D + 1;
        end
        $stop;
    end
endmodule

module regTest();
    localparam N = 4;
    logic [N-1: 0] Q, D;
    logic clk, clr, ld;
    integer i;
    register#(N) uut(.Q(Q),
                 .clk(clk),
                 .clr(clr),
                 .ld(ld),
                 .D(D));
                 
    initial begin
        clk = 0; clr = 0; ld = 0; D = 4'hf;
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
            if (i == 1 || i == 2) clr = 1;
            else clr = 0;
            if (i == 15 || i == 16) ld = 1;
            else ld = 0;
            if (i == 19 || i == 20) D = 4'ha;
        end
        ld = 0;
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
            if (i == 15 || i == 16) clr = 1;
            else clr = 0;
            if (i == 19 || i == 20) D = 4'hc;
        end
        $stop;
    end
endmodule

module cntTest();
    localparam N = 4;
    logic [N-1: 0] C;
    logic clk, clr, cnt;
    integer i;
    counter#(N) uut(.C(C),
                 .clk(clk),
                 .clr(clr),
                 .cnt(cnt));
                 
    initial begin
        clk = 0; clr = 0; cnt = 0;
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
            if (i == 1 || i == 2) clr = 1;
            else clr = 0;
            if (i == 15 || i == 16) cnt = 1;
            else cnt = 0;
        end
        cnt = 0;
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
            if (i == 15 || i == 16) clr = 1;
            else clr = 0;
        end
        
        for (i = 0; i < 32; i = i + 1) begin
            #1 if (i == 0) cnt = 1;
            clk = ~clk;
        end
        $stop;
    end
endmodule

module rfTest();
    logic [3:0] RD1, RD2;
    logic [2:0] RA1, RA2;
    logic [2:0] WA;
    logic [3:0] WD;
    logic clk, WE;
    integer i;
    registerFile uut(.RD1(RD1),
                     .RD2(RD2),
                     .RA1(RA1),
                     .RA2(RA2),
                     .WA(WA),
                     .WD(WD),
                     .clk(clk),
                     .WE(WE));
                         
    initial begin
        WE = 0; clk = 0; RA1 = 0; RA2 = 3'b111; WA = 0; WD = 0; #10
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
        end
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk; #1 clk = ~clk;
            RA1 = RA1 + 1;
            RA2 = RA2 - 1; 
        end
        #5 WD = 4'b1111; WA = 3'b101; #5 WE = 1;
        for (i = 0; i < 4; i = i + 1) begin
            #1 clk = ~clk;
        end
        #5 WE = 0;
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk; #1 clk = ~clk;
            RA1 = RA1 + 1;
            RA2 = RA2 - 1;
        end
        $stop;
    end
endmodule

module dmTest();
    logic [3:0] RD;
    logic [3:0] WD;
    logic [3:0] Adr;
    logic clk, WE, RE;
    integer i;
    dataMemory uut(.RD(RD),
                   .WD(WD),
                   .Adr(Adr),
                   .clk(clk),
                   .WE(WE),
                   .RE(RE));
                         
    initial begin
        WE = 0; RE = 0; clk = 0; Adr = 0; WD = 0; #10
        for (i = 0; i < 32; i = i + 1) begin
            #1 clk = ~clk;
        end
        RE = 1;
        for (i = 0; i < 16; i = i + 1) begin
            #1 clk = ~clk; #1 clk = ~clk;
            Adr = Adr + 1;
        end
        #5 WD = 4'b1111; Adr = 3'b101; #5 WE = 1; RE = 0;
        for (i = 0; i < 4; i = i + 1) begin
            #1 clk = ~clk;
        end
        #5 WE = 0; Adr = 0; RE = 1;
        for (i = 0; i < 16; i = i + 1) begin
            #1 clk = ~clk; #1 clk = ~clk;
            Adr = Adr + 1;
        end
        $stop;
    end
endmodule

module imTest();
    logic [11:0] RD;
    logic [2:0] Addr;
    integer i;
    instructionMemory uut(.RD(RD),
                        .Addr(Addr));
    initial begin
        Addr = 0;
        for (i = 0; i < 8; i = i + 1) begin
            #1 Addr = Addr + 1;
        end
        $stop;
    end
endmodule

module outLogicTest();
    logic [17:0] control_bits;
    logic [3:0] S;
    integer i;
    outputLogic uut(.control_bits(control_bits),
                    .S(S));
    initial begin
        S = 0;
        for (i = 0; i < 16; i = i + 1) begin
            #1 S = S + 1;
        end
        $stop;
    end
endmodule

module nextLogicTest();
    logic [3:0] RD;
    logic [7:0] Adr;
    integer i;
    nextStateLogic uut(.RD(RD),
                       .Adr(Adr));
                       
    initial begin
        Adr = 0;
        for (i = 0; i < 256; i = i + 1) begin
            #1 Adr = Adr + 1;
        end
        $stop;
    end
endmodule

module controllerTest();
    logic [17:0] control_bits;
    logic LB, RB;
    logic [1:0] func;
    logic clk, reset;
    integer i;
    controller uut(.control_bits(control_bits),
                   .LB(LB),
                   .RB(RB),
                   .func(func),
                   .clk(clk),
                   .reset(reset));
                     
    initial begin
        clk = 0; reset = 0; LB = 1; RB = 0; func = 2'b00;
        for (i = 0; i < 72; i = i + 1) begin
            #1 clk = ~clk;
            if (i >= 54) func = 2'b11;
            else if (i >= 36) func = 2'b10;
            else if (i >= 18) func = 2'b01;
        end
        $stop;
    end
endmodule

module simpleProcessorSimTest();
    logic [3:0] ALUA, ALUB, ALUResult;
    logic [17:0] control_bits;
    logic [1:0] func;
    logic [2:0] PC, RA2, WA;
    logic [11:0] IR;
    logic [3:0] RD1, RD2, WB_D, Res;
    logic clk, reset, LB, RB;
    logic [11:0] switches;
    integer i;
    
    simpleProcessorSim uut(.ALUA(ALUA),
                        .ALUB(ALUB),
                        .ALUResult(ALUResult),
                        .control_bits(control_bits),
                        .func(func),
                        .PC(PC),
                        .RA2(RA2), 
                        .WA(WA),
                        .IR(IR),
                        .RD1(RD1),
                        .RD2(RD2),
                        .WB_D(WB_D),
                        .Res(Res),
                        .clk(clk),
                        .reset(reset),
                        .LB(LB),
                        .RB(RB),
                        .switches(switches));
                        
    initial begin
        clk = 0; reset = 0; LB = 1; RB = 0; switches = 0;
        for (i = 0; i < 80; i = i + 1) begin
            #1 clk = ~clk;
        end
        $stop;
    end
endmodule

module simpleProcessorTest();
    logic [3:0] ALUA, ALUB, ALUResult;
    logic clk, reset, LB, RB;
    logic [11:0] switches;
    integer i;
    
    simpleProcessor uut(.ALUA(ALUA),
                        .ALUB(ALUB),
                        .ALUResult(ALUResult),
                        .clk(clk),
                        .reset(reset),
                        .LB(LB),
                        .RB(RB), 
                        .switches(switches));
                        
    initial begin
        clk = 0; reset = 0; LB = 1; RB = 0; switches = 0;
        for (i = 0; i < 80; i = i + 1) begin
            #1 clk = ~clk;
        end
        $stop;
    end
endmodule
