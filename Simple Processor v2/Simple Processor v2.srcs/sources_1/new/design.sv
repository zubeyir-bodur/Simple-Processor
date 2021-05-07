/**
* Title : Simple Processor - Term Project
* Author : Zubeyir Bodur
* ID: 21702382
* Section : 4
* Description : Design sources for the processor
*/

`timescale 1ns / 1ps

module top(
    output logic dp,
    output logic [3:0] an,
    output logic [6:0] seg,
    input logic clk, btnL, btnR, btnC,
    input logic [11:0] sw
    );
    logic LB, RB, CB;
    logic [3:0] ALUA, ALUB, ALUResult;
    debouncer lbdeb(LB, clk, btnL);
    debouncer rbdeb(RB, clk, btnR);
    debouncer cbdeb(CB, clk, btnC);
    // keyword parameters makes it easier for larger parameter modules
    simpleProcessor processor(.ALUA(ALUA),
                        .ALUB(ALUB),
                        .ALUResult(ALUResult),
                        .clk(clk),
                        .reset(CB),
                        .LB(LB),
                        .RB(RB),
                        .switches(sw));
    // it's always a good idea to make sure bit width is the same
    sevenSegmentDisplay display(seg, dp, an, clk, 
                                {1'b0, ALUA}, {1'b0, ALUB}, 5'd17,{1'b0, ALUResult}); 
                                
endmodule

module simpleProcessor(
    output logic [3:0] ALUA, ALUB, ALUResult,
    input logic clk, reset, LB, RB,
    input logic [11:0] switches 
    );
    logic [17:0] control_bits;
    logic [1:0] func;
    controller control(control_bits, LB, RB, func, clk, reset);
    datapath dpath(ALUA, ALUB, ALUResult, func, clk, switches, control_bits);
endmodule

module debouncer(
    output logic btn_out,
    input logic clk, btn_in
    );
    logic [24:0] timer;
    typedef enum logic [1:0]{S0,S1,S2,S3} states;
    states state, nextState;
    logic gotInput;
    
    always_ff@(posedge clk)
        begin    
            state <= nextState;
            if(gotInput)
                timer <= 25000000;
            else
                timer <= timer - 1;
        end
    always_comb
        case(state)
            S0: if(btn_in) 
                begin //startTimer
                    nextState = S1;    
                    gotInput = 1;
                end
                else begin nextState = S0; gotInput = 0; end
            S1: begin nextState = S2; gotInput = 0; end
            S2: begin nextState = S3; gotInput = 0; end
            S3: begin if(timer == 0) nextState = S0; else nextState = S3; gotInput = 0; end
            default: begin nextState = S0; gotInput = 0; end
            endcase
    
    assign btn_out = ( state == S1 );
endmodule

// LED positions inside 7-segment :
//   A
// F   B
//   G
// E   C
//   D    DP
// digit positions on Basys3 :
// in3(msb), in2, in1, in0(lsb)
module sevenSegmentDisplay(
    output logic [6:0] seg,
    output logic dp,
    output logic [3:0] an, //active low display
    input logic clk,
    input logic [4:0] in3, in2, in1, in0
    );
    // divide clock by 2^N
    localparam N = 18;
    logic [N-1:0] count = {N{1'b0}}; //initial value 
    
    always_ff @(posedge clk) begin
        count <= count + 1;
    end
    
    logic [4:0] digit_val; // 7-bit register to hold the current data on output 
    logic [3:0] digit_en; //register for enable vector 
    
    always_comb begin
        digit_en = 4'b1111; //default
        digit_val = in0; //default
        case(count[N-1:N-2]) //using only the 2 MSB's of the counter
        
            2'b00 : begin //select first 7Seg.
                digit_val = in0;
                digit_en = 4'b1110;
            end
        
            2'b01: begin //select second 7Seg.
                digit_val = in1;
                digit_en = 4'b1101;
            end
        
            2'b10: begin //select third 7Seg.
                digit_val = in2;
                digit_en = 4'b1011;
            end
            
            2'b11: begin //select forth 7Seg.
                digit_val = in3;
                digit_en = 4'b0111;
            end
        endcase
    end
    
    //Convert digit number to LED vector. LEDs are active low.
    logic [6:0] sseg_LEDs;
    always_comb begin
        sseg_LEDs = 7'b1111111; //default
        case(digit_val)
            5'd0 : sseg_LEDs = 7'b1000000; //to display 0
            5'd1 : sseg_LEDs = 7'b1111001; //to display 1
            5'd2 : sseg_LEDs = 7'b0100100; //to display 2
            5'd3 : sseg_LEDs = 7'b0110000; //to display 3
            5'd4 : sseg_LEDs = 7'b0011001; //to display 4
            5'd5 : sseg_LEDs = 7'b0010010; //to display 5
            5'd6 : sseg_LEDs = 7'b0000010; //to display 6
            5'd7 : sseg_LEDs = 7'b1111000; //to display 7
            5'd8 : sseg_LEDs = 7'b0000000; //to display 8
            5'd9 : sseg_LEDs = 7'b0010000; //to display 9
            5'd10 : sseg_LEDs = 7'b0001000; //to display A
            5'd11 : sseg_LEDs = 7'b0000011; //to display B
            5'd12 : sseg_LEDs = 7'b1000110; //to display C
            5'd13 : sseg_LEDs = 7'b0100001; //to display D
            5'd14 : sseg_LEDs = 7'b0000110; //to display E
            5'd15 : sseg_LEDs = 7'b0001110; //to display F
            5'd16 : sseg_LEDs = 7'b0111111; // to display -
            5'd17 : sseg_LEDs = 7'b0110111; // to display =
            5'd18 : sseg_LEDs = 7'b0100111; // to display c
            default :  sseg_LEDs = 7'b0111111; // display - otherwise
        endcase
    end
    assign an = digit_en;
    assign seg = sseg_LEDs;
    assign dp = 1'b1; //turn dp off
endmodule

// Controls only the single bit inputs of the datapath
// works as expected in the diagram
module controller(
    output logic [17:0] control_bits,
    input logic LB, RB,
    input logic [1:0] func,
    input logic clk, reset
    );
    logic [3:0] S_next, S;
    nextStateLogic NSL(S_next, {S, LB, RB, func});
    register#(4) stateRegister(S, clk, reset, 1'b1, S_next);
    outputLogic OL(control_bits, S);
endmodule

// Datapath for the simple processor
// All of the multi-bit operations are inside this datapath
module datapath(
    output logic [3:0] ALUA, ALUB, ALUResult,
    output logic [1:0] func,
    input logic clk,
    input logic [11:0] switches,
    input logic [17:0] control_bits
    );
    logic [2:0] PC;
    counter programCntr(PC, clk, control_bits[17], control_bits[16]);
    
    logic [11:0] IM_PC, IR_I;
    instructionMemory IM(IM_PC, PC);
    
    mux#(12, 1) ins_mux(IR_I, {switches, IM_PC}, control_bits[15]);
    
    logic [11:0] IR;
    register#(12) insReg(IR, clk, 1'b0, control_bits[14], IR_I);
    
    logic load, store, add, sub;
    assign load = (IR[11:9] == 3'b000);
    assign store = (IR[11:9] == 3'b001);
    assign add = (IR[11:9] == 3'b101);
    assign sub = (IR[11:9] == 3'b110);
    priorityEncoder PE(func, {load, store, add, sub});
    
    logic [2:0] RA2, WA;
    mux#(3, 1) rtmux(RA2, {IR[6:4], IR[5:3]}, control_bits[8]);
    mux#(3, 1) rdmux(WA, {IR[8:6], IR[6:4]}, control_bits[13]);
    
    logic [3:0] RD1, RD2, WB_D;
    registerFile RF(RD1, RD2, IR[2:0], RA2, WA, WB_D, clk, control_bits[12]);
    
    register#(4) ALUAreg(ALUA, clk, control_bits[7], control_bits[6], RD1);
    register#(4) ALUBreg(ALUB, clk, control_bits[5], control_bits[4], RD2);
    
    logic [3:0] Res, ALUResult_I;
    alu#(4) ALU(Res, RD1, RD2, control_bits[11:10]);
    
    logic [3:0] RD;
    mux#(4, 2) alurmux(ALUResult_I, {2'bZZ, RD, RD2, Res}, control_bits[1:0]);
    register#(4) ALURreg(ALUResult, clk, control_bits[3], control_bits[2], ALUResult_I);
    
    dataMemory DM(RD, RD2, IR[3:0], clk, control_bits[8], control_bits[9]);
    mux#(4, 1) wbmux(WB_D, {RD, Res}, control_bits[9]);
    
endmodule


// 256x4 ROM next state logic
// utilized don't cares to decrease num of lines
// works as expected
module nextStateLogic(
    output logic [3:0] RD,
    input logic [7:0] Adr
    );
    logic [3:0] S;
    logic [1:0] func;
    logic LB, RB;
    assign S = Adr[7:4];
    assign LB = Adr[3];
    assign RB = Adr[2];
    assign func = Adr[1:0];
    always_comb begin
        case (S)
            4'h0: RD = 4'b0001; 
            4'h1: begin
                case({LB, RB})
                    2'h0: RD = 4'b0001;
                    2'h1: RD = 4'b0011;
                    2'h2: RD = 4'b0010;
                    2'h3: RD = 4'b0001; // stay idle if buttons are pressed at the same time
                endcase
            end
            4'h2: RD = 4'b0100;
            4'h3: RD = 4'b0100;
            4'h4: begin
                case (func)
                    2'h0: RD = 4'b1000; // sub
                    2'h1: RD = 4'b0111; // add
                    2'h2: RD = 4'b0110; // store 
                    2'h3: RD = 4'b0101; // load
                endcase
            end
            4'h5: RD = 4'b0001; 
            4'h6: RD = 4'b0001; 
            4'h7: RD = 4'b0001; 
            4'h8: RD = 4'b0001;
            default : RD = 4'b0000; // reset state if S is unkown
        endcase
    end
endmodule

// works as expected
module outputLogic(
    output logic [17:0] control_bits,
    input logic [3:0] S
    );
    always_comb begin
        case(S)
            4'h0: control_bits = 18'h200a8;
            4'h1: control_bits = 18'h0;
            4'h2: control_bits = 18'h14000;
            4'h3: control_bits = 18'h0c000;
            4'h4: control_bits = 18'h0;
            4'h5: control_bits = 18'h012a6;
            4'h6: control_bits = 18'h001a5;
            4'h7: control_bits = 18'h03454;
            4'h8: control_bits = 18'h03854;
            default: control_bits = 18'h0; // unreachable states should do nothing - if reached for some reason
        endcase  
    end
endmodule

// works as expected
module instructionMemory(
    output logic [11:0] RD,
    input logic [2:0] Addr
    );
    always_comb begin
        case(Addr)
            3'h0: RD = 12'h005; // RF[0] = DM[5]
            3'h1: RD = 12'h017; // RF[1] = DM[7]
            3'h2: RD = 12'ha88; // RF[2] = RF[0] + RF[1]
            3'h3: RD = 12'h229; // DM[9] = RF[2]
            3'h4: RD = 12'h035; // RF[3] = DM[5]
            3'h5: RD = 12'h047; // RF[4] = DM[7]
            3'h6: RD = 12'hd63; // RF[5] = RF[3] - RF[4]
            3'h7: RD = 12'h250; // DM[0] = RF[5]
            default : RD = 12'h000; // RF[0] = DM[0] is default
        endcase
    end
endmodule

// 16x4 Memory
// simulation works fine
// implementation doesn't assign initial values
module dataMemory(
    output logic [3:0] RD,
    input logic [3:0] WD,
    input logic [3:0] Adr,
    input logic clk, WE, RE
    );
    logic [3:0] RAM [15:0] = {4'hf, 4'he, 4'hd, 4'hc,
                              4'hb, 4'ha, 4'h9, 4'h8,
                              4'h7, 4'h6, 4'h5, 4'h4,
                              4'h3, 4'h2, 4'h1, 4'h0}; // initial value
    
    always_ff @(posedge clk)
        if (WE) RAM[Adr] <= WD; // write is always synchronous

    always_comb
        if (RE) RD = RAM[Adr]; // asynchronous read
        
endmodule

// 8x4 RF
// works fine
module registerFile(
    output logic [3:0] RD1, RD2,
    input logic [2:0] RA1, RA2,
    input logic [2:0] WA,
    input logic [3:0] WD,
    input logic clk, WE
    );
    logic [3:0] RAM [7:0] = {4'h0, 4'h0, 4'h0, 4'h0,
                             4'h0, 4'h0, 4'h0, 4'h0}; // initial value
    
    always_ff @(posedge clk)
        if (WE) RAM[WA] <= WD; // write is always synchronous

    always_comb begin
        RD1 = RAM[RA1]; // asynchronous read
        RD2 = RAM[RA2];
    end
endmodule

// up counter with clear and parallel load
module counter#(parameter N = 3) (
    output logic [N-1:0] C,
    input logic clk, clr, cnt
    );
    logic [N-1:0] C_in;
    register#(N) regis(C, clk, clr, cnt, C_in);
    assign C_in = C + 1;
endmodule

// register with clear and parallel load
module register#(parameter N = 1)(
    output logic [N - 1: 0] Q = 0, // initial value
    input logic clk, clr, ld,
    input logic [N - 1: 0] D
    );
    logic [N-1:0] d_, d__;
    logic [1:0] [N - 1 : 0] inputs0, inputs1;
    assign inputs0 = {{N{1'b0}}, d_};
    assign inputs1 = {D, Q};
    mux#(N, 1) clrmux(d__, inputs0, clr); // multiplexer for reset
    mux#(N, 1) ldmux(d_, inputs1, ld); // multiplexer for enable
    always_ff @(posedge clk) begin
        Q <= d__;
    end
endmodule

// works as expected
module alu#(parameter N = 4)(
    output logic [N-1:0] res,
    input logic [N-1:0] A, B,
    input logic [1:0]  sel
    );
    always_comb
        case(sel)
            2'b00: res = A + 1;
            2'b01: res = A + B;
            2'b10: res = A - B;
            2'b11: res = A | B;
            default: res = {N{1'bZ}};
        endcase
endmodule

// 4 bit priority encoder
// works as expected
module priorityEncoder(
    output logic [1:0] Q,
    input logic [3:0] D
    );
    always_comb begin
        case (D)
            4'h0: Q = 2'bXX;
            4'h1: Q = 2'b00;
            4'h2: Q = 2'b01;
            4'h3: Q = 2'b01;
            4'h4: Q = 2'b10;
            4'h5: Q = 2'b10;
            4'h6: Q = 2'b10;
            4'h7: Q = 2'b10;
            4'h8: Q = 2'b11;
            4'h9: Q = 2'b11;
            4'ha: Q = 2'b11;
            4'hb: Q = 2'b11;
            4'hc: Q = 2'b11;
            4'hd: Q = 2'b11;
            4'he: Q = 2'b11;
            4'hf: Q = 2'b11;
        endcase
    end
endmodule

// M bit 2**N : 1 mux
// works as expected
module mux#(parameter M = 1, N = 1)(
    output logic [M - 1 : 0] Y,
    input logic [2**N - 1:0] [M - 1 : 0] D,
    input logic [N - 1:0] S
    );
    always_comb begin
        Y = D[S];
    end
endmodule

// SIMULATION MODULES BELOW
module simpleProcessorSim(
    output logic [3:0] ALUA, ALUB, ALUResult,
    output logic [17:0] control_bits,
    output logic [1:0] func,
    output logic [2:0] PC, RA2, WA,
    output logic [11:0] IR,
    output logic [3:0] RD1, RD2, WB_D, Res,
    input logic clk, reset, LB, RB,
    input logic [11:0] switches 
    );
    controller control(control_bits, LB, RB, func, clk, reset);
    datapathSim dpath(ALUA, ALUB, ALUResult, func, PC, RA2, WA, IR, RD1, RD2, WB_D, Res, clk, switches, control_bits);
endmodule

// Datapath for simulation
module datapathSim(
    output logic [3:0] ALUA, ALUB, ALUResult,
    output logic [1:0] func,
    output logic [2:0] PC, RA2, WA,
    output logic [11:0] IR,
    output logic [3:0] RD1, RD2, WB_D, Res,
    input logic clk,
    input logic [11:0] switches,
    input logic [17:0] control_bits
    );
    //logic [2:0] PC;
    counter programCntr(PC, clk, control_bits[17], control_bits[16]);
    
    logic [11:0] IM_PC, IR_I;
    instructionMemory IM(IM_PC, PC);
    
    mux#(12, 1) ins_mux(IR_I, {switches, IM_PC}, control_bits[15]);
    
    //logic [11:0] IR;
    register#(12) insReg(IR, clk, 1'b0, control_bits[14], IR_I);
    
    logic load, store, add, sub;
    assign load = (IR[11:9] == 3'b000);
    assign store = (IR[11:9] == 3'b001);
    assign add = (IR[11:9] == 3'b101);
    assign sub = (IR[11:9] == 3'b110);
    priorityEncoder PE(func, {load, store, add, sub});
    
    //logic [2:0] RA2, WA;
    mux#(3, 1) rtmux(RA2, {IR[6:4], IR[5:3]}, control_bits[8]);
    mux#(3, 1) rdmux(WA, {IR[8:6], IR[6:4]}, control_bits[13]);
    
    //logic [3:0] RD1, RD2, WB_D;
    registerFile RF(RD1, RD2, IR[2:0], RA2, WA, WB_D, clk, control_bits[12]);
    
    register#(4) ALUAreg(ALUA, clk, control_bits[7], control_bits[6], RD1);
    register#(4) ALUBreg(ALUB, clk, control_bits[5], control_bits[4], RD2);
    
    logic [3:0] /*Res,*/ ALUResult_I;
    alu#(4) ALU(Res, RD1, RD2, control_bits[11:10]);
    
    logic [3:0] RD;
    mux#(4, 2) alurmux(ALUResult_I, {2'bZZ, RD, RD2, Res}, control_bits[1:0]);
    register#(4) ALURreg(ALUResult, clk, control_bits[3], control_bits[2], ALUResult_I);
    
    dataMemory DM(RD, RD2, IR[3:0], clk, control_bits[8], control_bits[9]);
    mux#(4, 1) wbmux(WB_D, {RD, Res}, control_bits[9]);
    
//    always_comb begin
//        $monitor("Test cycle");
//        $monitor("PC = 0x%h - IM[PC] = 0x%h - switches = 0x%h - IR = 0b%b", PC, IM_PC, switches, IR);
//        $monitor("RD1-ALUA = 0x%h - RD2-ALUB-WD = 0x%h - WA = 0x%h - RD = 0x%h - ALURESULT = 0x%h", RD1, RD2, WA, RD, Res);
//    end
endmodule