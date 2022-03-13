
`timescale  1ns/100ps

module dcache (clock,reset,C_WRITE,C_READ,C_Address,C_WRITEDATA,C_READDATA,busywait,mem_write,mem_read,mem_address,mem_writedata,mem_readdata,mem_busywait);

    //Declaring input, and output registers.
    input  clock, reset,C_WRITE,C_READ,mem_busywait;
    input [7:0] C_WRITEDATA,C_Address;
    input [31:0] mem_readdata;
    output reg [31:0] mem_writedata;
    output reg [5:0] mem_address;
    output reg [7:0] C_READDATA;
    output reg mem_write,mem_read,busywait;

    reg [2:0] index,tag;
    reg [36:0] Cache_MEM [7:0];
    reg [31:0] BLOCK;
    reg [2:0] address_tag;
    reg [1:0] offset;
    reg readaccess,writeaccess,dirtybit,validbit,hit;
  

//Cache access signals are generated based on the read and write signals given to the cache
always @(C_WRITE,C_READ)
begin
		
        //if C_READ or C_WRITE are either 1,busywait=1
        if ((C_WRITE||C_READ ))
            begin
                busywait=1;
            end
        else
            begin
                busywait=0;
            end 

        //if AND of C_READ and !C_WRITE is 1, then readaccess=1 
         if (C_READ && !C_WRITE)
            begin
               readaccess=1;
            end
        else
            begin
                readaccess=0;
            end 

        //if AND of !C_READ and C_WRITE is 1, then writeaccess=1 
        if (!C_READ && C_WRITE)
            begin
               writeaccess=1;
            end
        else
            begin
                writeaccess=0;
            end 
end



always @(*)begin
		
		//Cache memory array is seperated as block ,tag ,dirtybit ,validbit ,offset and index
		if(readaccess == 1'b1 || writeaccess == 1'b1)
        begin
            #1;
            //Decoding tag, offset, and index
                offset       =  C_Address[1:0];
                index        =  C_Address[4:2]; 
                address_tag  =  C_Address[7:5]; 
                
        
                //finding the correct cache entry and extracting the stored data block,
                //tag,valid and dirty bits 
                //This extraction was done based on the index extracted from address      
                BLOCK    =  Cache_MEM[index][31:0];
                tag      =  Cache_MEM[index][34:32];
                dirtybit =  Cache_MEM[index][35];    //dirty bit
                validbit =  Cache_MEM[index][36];   //valid bit
		end
end


//Compare tag and choose whether it is a hit or a miss.
always @(address_tag[0],address_tag[1],address_tag[2],tag[0],tag[1],tag[2],validbit)
    begin
    
    //Make hit as high when validbit is 1, and tag matches with the address_tag provided.
    if( validbit && (tag[0]==address_tag[0] && tag[1]==address_tag[1] && tag[2]==address_tag[2]))
        begin
            #0.9 hit=1'b1;
        end  

    //Else, hit=0
    else
        begin
            #0.9 hit=1'b0;
        end 

end



//4 small blocks of data are selected from offset.
always@(BLOCK[7:0],BLOCK[15:8],BLOCK[23:16],BLOCK[31:24],offset) 
    begin
    //this delay will overlap with tag comaprison delay of 0.9
    #1;
    case(offset)
            2'b00 :C_READDATA=BLOCK[7:0]; 
            2'b01 :C_READDATA=BLOCK[15:8];
            2'b10 :C_READDATA=BLOCK[23:16];
            2'b11 :C_READDATA=BLOCK[31:24];

    endcase 
    end


//When Readhit is 1, cpu will not stall.
always @(posedge clock)begin 

    if(readaccess == 1'b1 && hit == 1'b1)
        begin 
                
            //to prevent accessing memory for reading 
            readaccess = 1'b0;
            busywait= 1'b0; //Stop cpu stalling

        end



//When write hit is 1, Data can be written to cache, and cpu will not stall.
else if(writeaccess == 1'b1 && hit == 1'b1)begin 
    //Make buswait=0 inorder to write to the cache.
	busywait = 1'b0;
    //Offset determines dataword that has to be written to the cache.
	#1;
	case(offset)
		2'b00	:	Cache_MEM[index][7:0]   = C_WRITEDATA;
		2'b01 	:	Cache_MEM[index][15:8]  = C_WRITEDATA;
		2'b10 	:	Cache_MEM[index][23:16] = C_WRITEDATA;
		2'b11   :	Cache_MEM[index][31:24] = C_WRITEDATA;
	endcase

    //make dirtybit = 1 if the block and the memory  are inconsistent 
	Cache_MEM[index][35] = 1'b1; 

    //make validbit = 1 to make sure that a valid data is available in block rather than a garbage value. 
    Cache_MEM[index][36] = 1'b1;

    //Prevent accessing memory for writing        	
	writeaccess = 1'b0;
	end
end
    
  

//Finite State Machine of Cache controller
    reg [2:0] state, next_state;
    parameter IDLE = 3'b000, MEM_READ = 3'b001,MEM_WRITE=3'b010,C_MEM_WRITE=3'b011;

    // combinational next state logic
    always @(*)
    begin
        case (state)

            //Idle state 
            IDLE:

                //if the existing block is not dirty ,the missing block should be fetched from memory
                if ((C_READ || C_WRITE) && !dirtybit && !hit)               
                    next_state = MEM_READ;        //memory read

                //if the existing block is dirty ,that block must be written back to the memory before fetching the missing block
                else if ((C_READ ||C_WRITE) && dirtybit && !hit)            
                    next_state = MEM_WRITE;       //write-back

                //if C_READ and C_WRITE are 0, IDLE state is maintained.
                else
                    next_state = IDLE;
            


            //next state after writing in cache from data received from memory is IDLE		
	        C_MEM_WRITE:
              
                next_state = IDLE;
            
                
            MEM_WRITE:
                //After memory writing, memory reading is started.
                if (!mem_busywait)
                        next_state = MEM_READ;
                //Else, write back to the memory
                else    
                    next_state = MEM_WRITE;	

            MEM_READ:
                if (!mem_busywait)
                    next_state = C_MEM_WRITE;
                else    
                    next_state = MEM_READ;
               
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)                 
			C_MEM_WRITE: 
                begin
                    mem_read = 1'd0;
                    mem_write = 1'd0;
                    mem_address = 6'dx;
                    mem_writedata = 32'dx;
                    busywait=1;

                //this writing operation happens in the cache block after fetching the  memory
                    #1;
                    //write a data block to the cache
                    Cache_MEM[index][31:0] = mem_readdata;	
                    //tag C_Address[7:5]
                    Cache_MEM[index][34:32] = address_tag ;
                    //dirty bit=0 since we are writing a data in cache which is also in memory	
                    Cache_MEM[index][35] = 1'd0;	
                    //valid bit=1
                    Cache_MEM[index][36] = 1'd1;	
                
                end


            //contol signals of cache control during read and write is 0
            IDLE:
                begin
                    mem_read      = 0;
                    mem_write     = 0;
                    mem_address   = 6'dx;
                    mem_writedata = 32'dx;
                end
         

            //during a read miss we assert the mem_read to read from data memory
            MEM_READ: 
                begin
                    mem_read      = 1;
                    mem_write     = 0;
                    mem_address   = {tag,index};
                    mem_writedata = 32'dx;
                    busywait      = 1;
                end


           	MEM_WRITE: 
                begin
                    mem_read  = 1'd0;
                    mem_write = 1'd1;
                    mem_address ={tag,index};	//data block address from the cache
                    mem_writedata =BLOCK;
                    busywait=1;
                end
           
        endcase
    end

    // sequential logic for state transitioning 
    integer i;
    always @(posedge clock, reset)
    
    begin
        if(reset)
            begin
                state = IDLE;
                busywait=1'b0;

                //reset the cache after 1 time unit delay
                #1;
                for( i=0;i<8;i=i+1)
                    begin
                    Cache_MEM[i] = 37'd0;
                    end
            end
        
        else begin
            state = next_state;
        end
    end
endmodule
