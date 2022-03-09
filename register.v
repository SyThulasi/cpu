/*
Lab5
Group-29
Register File
*/
`timescale 1ns/100ps
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
//Defining all the required variables

    input [7:0] IN;             //8 bits data to be written
     //3 bit address to find required registers since no of registers cant exceed 8.
     //i.e Registers[xxx]
    input [2:0] OUT1ADDRESS,OUT2ADDRESS;
    input [2:0] INADDRESS;

    //Since CLK, RESET, and WRITE signals are 1 bit, they are declared as inputs.
    input CLK, RESET, WRITE;

    //Data which was read to be displayed are 8 bits.
    output [7:0] OUT1, OUT2;

    //8 registers which can store 8 bits
    //Values in registers can be accessed by using j given as address in Registers[j] where 0<j<8.
    reg [7:0] Registers[0:7];

    //Find the correct register asynchronously to be read by using Outaddresses and store the read value to out1, and out2.
    assign #2 OUT1 = Registers[OUT1ADDRESS];
    assign #2 OUT2 = Registers[OUT2ADDRESS];

    integer i; //For using in for loop
    //This always block is executed whenever there is a positive edge in the clock cycle.
    always@(posedge CLK)
    begin
        //Reset all the values in 8 registers if RESET=1
        if(RESET) begin
            //Using for loop to iterate through all registers and to reset them.
            //Waiting 1 nanosecond before resetting.
            #1 for(i = 0; i < 8; i += 1) begin
                Registers[i] = 8'd0;
            end
        end

        //If RESET=0, and WRITE=1 we have to write the given data to requested register.
        else begin
            #0.1
            if (WRITE) begin
              #0.9 Registers[INADDRESS] = IN;
            end
        end
    end
/*     initial begin
     #250 $display($time, "%d %d %d %d %d %d %d %d\n", Registers[0], Registers[1], Registers[2], Registers[3], Registers[4], Registers[5], Registers[6], Registers[7]);
     end */

endmodule

