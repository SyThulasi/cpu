/*
Lab5 part 4
Group-29
ALU UNIT 
*/

module alu(DATA1,DATA2,SELECT,RESULT,ZERO);
//Defining all the required variables
    input [7:0] DATA1,DATA2; //8 bit input data
    input [2:0] SELECT;     //3 bit selection data to select required arithmetic functions
    output [7:0] RESULT;    //8 bit output data
    reg RESULT;             //Store the result in register
    output reg ZERO=1'b 0 ;

    //Creating wires for getting the outputs.
    wire [7:0] forward_result, add_result, and_result, or_result;

    //initiate functions, and assign the results to the wires.
    forward forward1(DATA2,forward_result);
    ADD add1(DATA1,DATA2,add_result);
    AND and1(DATA1,DATA2,and_result);
    OR or1(DATA1,DATA2,or_result);

    //This always block is executed whenever values in this sensitivity list change. 
    always@(forward_result, add_result, and_result, or_result, SELECT)
    begin
        //We are using case for switching between required arithmetic functions according to SELECT.
        case (SELECT) 
        3'b000: RESULT=forward_result;       //Forward the DATA2
        3'b001: RESULT= add_result; //Add DATA1,and DATA2
        3'b010: RESULT=and_result; //Bitwise AND
        3'b011: RESULT= or_result; //Bitwise OR
        default:RESULT = 8'bxxxxxxxx; //Default case
        // 3'b11: RESULT=DATA_1|DATA_2; 
        endcase
    end

    //If the result coming from sub/ add is ZERO, then assign 1 to ZERO in order to use in j, and beq.
    always @(add_result ) begin
        if (add_result == 8'b00000000) begin
            ZERO = 1'b1;
        end else begin
            ZERO = 1'b0;
        end
    end

endmodule

//ADD module
module ADD(DATA1,DATA2,RESULT);
    input [7:0] DATA1,DATA2; //8 bit input data
    output [7:0] RESULT;    //8 bit output data
    reg RESULT;             //Store the result in register
    always @(DATA1,DATA2) begin
        #2
        RESULT=DATA1+DATA2;
    end
endmodule

//OR module
module OR(DATA1,DATA2,RESULT);
    input [7:0] DATA1,DATA2; //8 bit input data
    output [7:0] RESULT;    //8 bit output data
    reg RESULT;             //Store the result in register
    always @(DATA1,DATA2) begin
        #1
        RESULT=DATA1|DATA2;
    end
endmodule

//AND module
module AND(DATA1,DATA2,RESULT);
    input [7:0] DATA1,DATA2; //8 bit input data
    output [7:0] RESULT;    //8 bit output data
    reg RESULT;             //Store the result in register
    always @(DATA1,DATA2) begin
        #1
        RESULT=DATA1&DATA2;
    end
endmodule

//Forward module
module forward(DATA2,RESULT);
    input [7:0] DATA2; //8 bit input data
    output [7:0] RESULT;    //8 bit output data
    reg RESULT;             //Store the result in register
    always @(DATA2) begin
        #1
        RESULT=DATA2;
    end
endmodule









