/*

Group-29
CPU 
Authors : Hariharan (E/18/128) and Thulasiyan (E/18/366)

*/
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    wire READsignal, WRITEsignal, BUSYWAITsignal;
    wire [7:0] READDATA, ADDRESS, WRITEDATA;

    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    

    reg [7:0] instr_mem[0:1023];
  
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)

    always @(PC) begin
        #2
        INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        
        /* {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]}        = 32'b00000000_00000000_00000000_00001001;  //loadi 0     #9
        {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]}        = 32'b00000110_00000001_00000000_00000000;  //j     #1     
        {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]}      = 32'b00000000_00000001_00000000_00000100;  //loadi 1     #8
        {instr_mem[10'd15], instr_mem[10'd14], instr_mem[10'd13], instr_mem[10'd12]}    = 32'b00000000_00000001_00000000_00001010;  //loadi 1     #10  
        {instr_mem[10'd19], instr_mem[10'd18], instr_mem[10'd17], instr_mem[10'd16]}    = 32'b00000111_00000001_00000001_00000000;  //beq   #1  1  0
        {instr_mem[10'd23], instr_mem[10'd22], instr_mem[10'd21], instr_mem[10'd20]}    = 32'b00000000_00000010_00000000_00001001;  //loadi 2     #9
        {instr_mem[10'd27], instr_mem[10'd26], instr_mem[10'd25], instr_mem[10'd24]}    = 32'b00000111_00000001_00000010_00000000;  //beq   #1  2  0
        {instr_mem[10'd31], instr_mem[10'd30], instr_mem[10'd29], instr_mem[10'd28]}    = 32'b00000011_00000011_00000000_00000001;  //sub   3   0  1
        {instr_mem[10'd35], instr_mem[10'd34], instr_mem[10'd33], instr_mem[10'd32]}    = 32'b00000010_00000011_00000000_00000001;  //add   3   0  1 */
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */

    //
    cpu mycpu(PC, INSTRUCTION, CLK, RESET, READsignal,WRITEsignal, ADDRESS, WRITEDATA, READDATA, BUSYWAITsignal);

    data_memory datamem(CLK, RESET, READsignal, WRITEsignal, ADDRESS, WRITEDATA, READDATA, BUSYWAITsignal);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata29.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #2
        RESET = 1'b1;

        #4
        RESET= 1'b0;
        
        // finish simulation after some time
        #500
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule


//CPU module for handling componentns ALU,Registers,Control unit, MUX, 2s'complements as one module
module cpu(PC, INSTRUCTION, CLK, RESET, READsignal, WRITEsignal,RESULT, OPERAND1, READdata, WAITsignal);

    //Declaring inputs,outputs,wires
    input [31:0] INSTRUCTION;
    input CLK, RESET, WAITsignal;
    output reg [31:0] PC;
    wire ZERO,ISB,ISJ;
    reg [2:0] ALU_SELECT;
    wire [7:0]  OPERAND2, MUX1OUT, MUX2OUT, MUX3RESULT;
    reg [7:0] twosComplement ,IMMEDIATE, OPCODE,OFFSET;
    reg [2:0] DESTINATION, SOURCE1, SOURCE2;
    reg WRITE , isSUBSTRACT, isIMMEDIATE, isBRANCH, isJUMP, MEMsignal;
    reg [31:0] NEXT;
    wire[31:0] FINAL1,FINAL2,MUX3OUT, MUX4OUT;
    reg [31:0] targetAddress;
    output reg READsignal, WRITEsignal;
    input [7:0] READdata;
    wire WRITE_FINAL;
    output [7:0] OPERAND1,RESULT;

    
    // WRITE signal for register. which will enable when WRITE singal from control unit is enable and wait signal disable
    assign WRITE_FINAL=WRITE & !WAITsignal;

    //Create instances for all modules

    //This MUX will select whether to choose twosComplement value for sub or  positive value for add operation according the select input provided.
    MUX mux1(twosComplement, OPERAND2, MUX1OUT, isSUBSTRACT); 

    //This MUX will select whether to choose Register value or the immediate value according to the select input from control unit.
    MUX mux2(IMMEDIATE, MUX1OUT, MUX2OUT, isIMMEDIATE);  
   
    //ALU for arithmetic and logic operations 
    alu alu1(OPERAND1, MUX2OUT,ALU_SELECT, RESULT,ZERO);
  
    //Register file for reading from registers and writing to registers.
    reg_file register(MUX3RESULT, OPERAND1, OPERAND2, DESTINATION, SOURCE1, SOURCE2, WRITE_FINAL, CLK, RESET);

    //Extend the 8 bits OFFSET value to 32 bits.
    signextender sign_ex(OFFSET,FINAL1);

    //shift left the FINAL1 value to FINAL2 value inorder to calculate targetAddress
    shift shft(FINAL1,FINAL2);
  

    //Whenever the adder value to PC changes, target address will be calculated.    
    always @(FINAL2) begin
        #2 targetAddress= NEXT + FINAL2;
    end
    
    //AND gate to choose the isbranch signal
    and and_1(ISB,isBRANCH,ZERO);


    //MUX with 32 bits I/O for selecting BRANCH address
    MUX_JB mux_3(NEXT,targetAddress,MUX3OUT,ISB);

    //MUX with 32 bits I/O for selecting JUMP address
    MUX_JB mux_4(MUX3OUT,targetAddress,MUX4OUT,isJUMP);


    //--------------------------------------------------------------------------
    //This MUX will select whether to choose ALU result or datamemory read data value 
    MUX mux3(READdata, RESULT, MUX3RESULT, MEMsignal);
    //-----------------------------------------------------------------------------
   

    //Get the twos complement whenever OPERAND2 is available.
    always @(OPERAND2) begin          
        twosComplement = #1 (~OPERAND2 +1);   //taking the compliment of the REGOUT2
    end


    //Assign the next PC value  to PC whenever there is a positive edge clock.
    always @(posedge CLK ) begin  
         //Reset whenever there is a reset signal during positive edge clock.

        if (WAITsignal == 1'b0) begin
            if(RESET) begin 
               #1  PC <= 32'd0;
            end
            else begin
                #1 PC = MUX4OUT;    //PC is updated by nextAddress(MUX4OUT)
            end
        end
    end

    //Find the (PC+4) NEXT value and store it in OUT reg in this module and assign it to PC whenever there is a positive edge clock.
    always@(PC) begin               //when PC is changed 
        #1 NEXT = PC + 4;         //PC_register is updated by PC + 4
    end



    always @(INSTRUCTION) begin
        //Decode Instructions and assign required values to do the work of control unit. 
        IMMEDIATE = INSTRUCTION[7:0];
        DESTINATION = INSTRUCTION[18:16];
        SOURCE1 = INSTRUCTION[10:8];
        SOURCE2 = INSTRUCTION[2:0];
        OPCODE = INSTRUCTION[31:24];
        OFFSET = INSTRUCTION[23:16];

          #1
        case(OPCODE)            // 31 : 24
            // LOAD
            8'd0 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b1;
                    isBRANCH=1'b0;
                    isJUMP=1'b0;
                    ALU_SELECT = 3'b000;
                    MEMsignal = 1'b0;
                end
            // MOV
            8'd1 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    ALU_SELECT = 3'b000;
                    isBRANCH=1'b0;
                    isJUMP=1'b0; 
                    MEMsignal = 1'b0;
                end
            // ADD
            8'd2 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    ALU_SELECT = 3'b001;
                    isBRANCH=1'b0;
                    isJUMP=1'b0; 
                    MEMsignal = 1'b0;
                end
            // SUB
            8'd3 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b1;
                    isIMMEDIATE = 1'b0;
                    ALU_SELECT = 3'b001;
                    isBRANCH=1'b0;
                    isJUMP=1'b0; 
                    MEMsignal = 1'b0;
                end
            // AND
            8'd4 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    ALU_SELECT = 3'b010;
                    isBRANCH=1'b0;
                    isJUMP=1'b0; 
                    MEMsignal = 1'b0;
                end
            // OR
            8'd5 : 
                begin
                    WRITE = 1;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    ALU_SELECT = 3'b011;
                    isBRANCH=1'b0;
                    isJUMP=1'b0; 
                    MEMsignal = 1'b0;
                end

            8'd6: begin // j
                    WRITE = 0;
                    isSUBSTRACT = 1'b0;
                    isIMMEDIATE = 1'b0;
                    isBRANCH=1'b0;
                    isJUMP=1'b1;
                    MEMsignal = 1'b0;
                    end

            8'd7: begin // beq
                WRITE = 0;
                ALU_SELECT = 3'b001 ;
                isSUBSTRACT = 1'b1;
                isIMMEDIATE=1'b0;
                isBRANCH=1'b1;
                isJUMP=1'b0; 
                MEMsignal = 1'b0; 
            end

            8'd8: begin //lwd
                WRITE = 1'b1;
                ALU_SELECT = 3'b000;
                isSUBSTRACT = 1'b0;
                isIMMEDIATE = 1'b0;
                isJUMP = 1'b0;
                isBRANCH = 1'b0;
                MEMsignal = 1'b1;
                READsignal = 1'b1;
                WRITEsignal = 1'b0;
            end

            8'd9: begin //lwi
                WRITE = 1'b1;
                ALU_SELECT = 3'b000;
                isSUBSTRACT = 1'b0;
                isIMMEDIATE = 1'b1;
                isJUMP = 1'b0;
                isBRANCH = 1'b0;
                MEMsignal = 1'b1;
                READsignal = 1'b1;
                WRITEsignal = 1'b0;
            end

            8'd10: begin //swd
                WRITE = 1'b0;
                ALU_SELECT = 3'b000;
                isSUBSTRACT = 1'b0;
                isIMMEDIATE = 1'b0;
                isJUMP = 1'b0;
                isBRANCH = 1'b0;
                MEMsignal = 1'b0;
                READsignal = 1'b0;
                WRITEsignal = 1'b1;
            end

            8'd11: begin //swi
                WRITE = 1'b0;
                ALU_SELECT = 3'b000;
                isSUBSTRACT = 1'b0;
                isIMMEDIATE = 1'b1;
                isJUMP = 1'b0;
                isBRANCH = 1'b0;
                MEMsignal = 1'b0;
                READsignal = 1'b0;
                WRITEsignal = 1'b1;
            end

        endcase
    end
endmodule


//Declaring MUX module to select outputs according to the SELECT input.
module MUX(IN1, IN2, OUT, SELECT);
    input [7:0] IN1, IN2;
    input SELECT;
    output reg [7:0] OUT;

    //Choose outputs
    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN1;
            1'b0: OUT <= IN2;
            default: OUT <= 8'd0;
        endcase
    end
endmodule


//Signextender for the OFFSET value from the instructions in order to calculate the next PC value.
module signextender(unextended, sign_extended);
    input [7:0] unextended; //the msb bit is the sign bit // 8-bit input
    output reg [31:0] sign_extended; // 32-bit output

    always @(unextended) begin
        if(unextended[7] == 1'b0) begin
            sign_extended[31:8] = 24'd0;
            sign_extended[7:0]  = unextended[7:0];
        end else if (unextended[7] == 1'b1) begin
            sign_extended[31:8] = 24'd1;
            sign_extended[7:0]  = unextended[7:0];
        end
    end 
endmodule

//Left shift by 2.
module shift(IN,OUT);
    input signed [31:0] IN;
    output signed [31:0] OUT;
    assign OUT=IN<<2;
endmodule


//Declaring MUX_JB module to select outputs according to the SELECT input for 32 bit inputs to be used in Beq, and Jump.
module MUX_JB(IN1, IN2, OUT, SELECT);
    input [31:0] IN1, IN2;
    input SELECT;
    output reg [31:0] OUT;

    //Choose outputs
    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN2;
            1'b0: OUT <= IN1;
            default: OUT <= 32'd0;
        endcase
    end
endmodule