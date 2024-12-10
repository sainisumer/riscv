`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 02:35:20 PM
// Design Name: 
// Module Name: processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module processor(input clk,rst);
//pc pc(.clk(clk),.rst(rst),.pc(pcs));
wire [31:0] instr,data1,data2,wr_data,result,pc;
wire [4:0] addr1,addr2,wr_addr;
wire [31:0] imm_val,data_mem_addr;
wire en,sw;
wire [5:0] alu_control;
data_mem data_mem(.clk(clk),.rst(rst),.mem_read(),.mem_write(sw),.address(),.write_data(data2),.read_data());
alu alu(.src1(data1),.src2(data2),.alu_control(alu_control),.result(result),.imm_val_r(imm_val));
pc fetch(.clk(clk),.reset(rst),.pc(pc));
instruction_memory mem(.clk(clk),.reset(rst),.pc(pc),.instruction_code(instr));
ctrl_unit ctrl(.clk(clk),.rst(rst),.opcode(instr[6:0]),.funct7(instr[31:25]),.funct3(instr[14:12]),.alu_control(alu_control),.sw(sw));
pc_reg_file reg_file(.clk(clk),.reset(rst),.rd_addr1(instr[24:20]),.rd_addr2(instr[19:15]),.rd_data1(data1),.rd_data2(data2),.wr_data(result),.wr_en(en),.wr_addr(instr[11:7]));
assign en = (instr[6:0] == 'b0110011)?1:0;
assign imm_val = {{20{instr[31]}},{instr[30:20]}};
assign data_mem_addr = imm_val + data1;
endmodule






`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/09/2024 09:43:10 PM
// Design Name: 
// Module Name: data_mem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module data_mem (
    input logic clk,                     // Clock signal
    input logic rst,                     // Reset signal
    input logic mem_read,                // Memory read enable
    input logic mem_write,               // Memory write enable
    input logic [31:0] address,          // Address bus (32-bit)
    input logic [31:0] write_data,       // Data to be written to memory
    output logic [31:0] read_data        // Data read from memory
);

    // Declare a memory array (32-bit wide, 1024 locations)
    logic [31:0] memory_array [0:1023]; 

    // Read operation
    always_comb begin
        if (mem_read) begin
            read_data = memory_array[address[11:2]]; // Address decoded for word alignment
        end else begin
            read_data = 32'b0;                      // Default to 0 when not reading
        end
    end

    // Write operation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Initialize memory during reset (optional, can be removed)
            integer i;
            for (i = 0; i < 1024; i++) begin
                memory_array[i] <= 32'b0;
            end
        end else if (mem_write) begin
            memory_array[address[11:2]] <= write_data; // Write data to memory
        end
    end

endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 10:24:02 AM
// Design Name: 
// Module Name: alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module alu(input [31:0] src1,src2,input [5:0] alu_control,output reg [31:0] result,input [31:0] imm_val_r);

always@(*)
begin
    case(alu_control)
     6'b000001 ://addition 
                                begin
                
                                        result = src1 + src2;
                                end
                6'b000010 : //subraction
                
                                        result = src1 - src2;
                6'b000011 : //shift left logical
                
                                        result = src1 << src2;
            
                6'b000100 : // set less than
                        begin
                            
                                        result = (src1 < src2) ? 1 : 0;
                            
                        end
                        
                 6'b000101 : // set less than unsigned
                        begin
                            
                            
                        end
                        
                        
                 6'b000110 : // xor operation
                        begin
                            
                                result = src1 ^ src2;
                        end
                        
                  6'b000111 : //shift right logical
                        begin
                                result = src1 >> src2;
                         end
                   
                   6'b001000 : // shift right arthimetic
                        begin
                                result = src1 >>> src2;
                         end
                         
                    6'b001001 : //or operation
                        begin
                                result = src1 | src2;
                        end
                        
                    6'b001010 : // and operation
                            
                         begin
                                result = (src1) & src2 ;
                          end
                         6'b001011 :
                                result = src1 + imm_val_r; // add immediate

                     6'b001100 :
               
                                result = imm_val_r << 1; // shift left logical immediate

                     6'b001101 :
                                result = (imm_val_r < src1) ? 1 : 0;// SET LESS THAN IMMEDIATE
                                        
                     6'b001110:
                                result = src1 & src2;
              
                     6'b001111 :
               
                               result = src1 ^ imm_val_r; // xor immediate
               
              
                    6'b010000 :              
                               result = src1 >> imm_val_r; // shift right logical immediate
              
                    6'b010001 :
          
                              result = (src1 < src2) ? 1 : 0;
             endcase


end

endmodule




`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:    TADAKAMALLA GOURAV
// 
// Create Date: 07/18/2024 04:07:13 PM
// Design Name: INSTRUTION FETCH UNIT
// Module Name: instruction_fetch_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module pc(
                                input clk,                          //  Clock source
                                input reset,                        //  Reset signal
                              //  input [31:0] imm_address,           //  Immediate address for branch instructins
                              //  input [31:0] imm_address_jump,      //  Immediate address for jump instructions
                              //  input beq,                          //  control signal for enabling beq operation
                              //  input bneq,                         //  control signal for enabling bneq operation
                               // input bge,                          //  control signal for enabling bge operation
                                //input blt,                          //  control signal for enabling blt operation
                                //input jump,                         //  control signal for enabling jump operation
                                output reg [31:0] pc              //  programme counter
                                //output reg [31:0] current_pc        //  register for storing retrun address of programme counter
    );
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// LOGIC FOR INCREMENTING PROGRAMME COUNTER /////////////////////////////////////////////   

    always@(posedge clk)
        begin
            if(reset == 1)
                begin
                    pc <= 0;
                end
            else 
            pc <= pc+4;/*if(beq == 0 && bneq == 0 && bge == 0 && blt == 0 && jump == 0)
                    pc <= pc + 4;
                    
            else if(beq == 1 || bneq == 1 || bge == 1 || blt == 1)
                    pc <= pc + imm_address;
          
            else if(jump)
                    pc <= pc + imm_address_jump;*/
         end

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////  LOGIC FOR STORING RETURN ADDRESS OF PROGRAMME COUNTER  /////////////////////////////////////////////////////
        
      /*   always@(posedge clk)
            begin
            if(reset)
                begin
                    current_pc = 0;
                 end
                 
                else if(reset == 0 && jump == 0)
                    current_pc <= pc + 4;
                else
                    current_pc <= current_pc;
            end*/
    
endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 02:17:18 PM
// Design Name: 
// Module Name: instruction_memory
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module instruction_memory(
    input clk,
    input [31:0] pc,                    // PROGRAMME COUNTER
    input reset,                        // ACTIVE HIGH RESET
    output [31:0] instruction_code      // 32 BIT INSTRUCITON      
    );
    
    
    reg [7:0] memory [108 :0];          // MEMORY FOR STORING THE INSTRUCTION OF WIDTH 32 BITS AND NUMBER OF LOCATIONS : 109
    
    
    //INSTRUCTION FETCHING FROM THE MEMORY
    
    assign instruction_code = {memory[pc+3],memory[pc+2],memory[pc+1],memory[pc]}; 
    
    
    always@(posedge clk)
        begin
                 if(reset == 1)
                   begin
                       // setting 32 - bit instruction : addi : 0x00a08513 
                       memory[3] = 8'h00;
                       memory[2] = 8'h73;
                       memory[1] = 8'h28;
                       memory[0] = 8'h23;
                       // Setting 32-bit instruction: sub : 0x800100b3
                       memory[7] = 8'h80;
                       memory[6] = 8'h01;
                       memory[5] = 8'h00;
                       memory[4] = 8'hb3;
                       // Setting 32-bit instruction: sll : 0x00209133
                       memory[11] = 8'h00;
                       memory[10] = 8'h20;
                       memory[9] = 8'h91;
                       memory[8] = 8'h33;
                       // Setting 32-bit instruction: xor : 0x00c54ab3 
                       memory[15] = 8'h00;
                       memory[14] = 8'hc5;
                       memory[13] = 8'h4A;
                       memory[12] = 8'hb3;
                       // Setting 32-bit instruction: srl : 0x00c55ab3
                       memory[19] = 8'h00;
                       memory[18] = 8'hc5;
                       memory[17] = 8'h5a;
                       memory[16] = 8'hb3;
                       // Setting 32-bit instruction: all : 0z01bd5f33
                       memory[23] = 8'h01;
                       memory[22] = 8'hbd;
                       memory[21] = 8'h5f;
                       memory[20] = 8'h33;
                       // Setting 32-bit instruction: or : 0x00d67fb3
                       memory[27] = 8'h00;
                       memory[26] = 8'hd6;
                       memory[25] = 8'h7f;
                       memory[24] = 8'hb3;
                       // Setting 32-bit instruction: and : 0x00f78b3
                       memory[31] = 8'h00;
                       memory[30] = 8'hf7;
                       memory[29] = 8'h68;
                       memory[28] = 8'hb3;
                      
                       // setting 32 - bit instruction : addi : 0x00a08513 
                       memory[35] = 8'h00;
                       memory[34] = 8'ha0;
                       memory[33] = 8'h85;
                       memory[32] = 8'h13;
                       
                       // setting 32 bit instruction : slli : 
                       memory[39] = 8'h00;
                       memory[38] = 8'h41;
                       memory[37] = 8'h93;
                       memory[36] = 8'h13;
                       
                       
                       
                       
                       // setting 32 bit instruction for xor immediate
                       
                       memory[43] = 8'h03;
                       memory[42] = 8'hf2;
                       memory[41] = 8'hc7;
                       memory[40] = 8'h26 ;
                       
                       
                       // setting 32 bit instruction for set less than immediate
                       
                       memory[47] = 8'h00;
                       memory[46] = 8'ha1;
                       memory[45] = 8'h20;
                       memory[44] = 8'h93;
                       
                       //setting 32 bit instruction for shift right logical immediate
                       
                       memory[51] = 8'h00;
                       memory[50] = 8'h31;
                       memory[49] = 8'h50;
                       memory[48] = 8'h93;
                       
                       // setting 32 bit instructino for or immediate
                       memory[55] = 8'h00;
                       memory[54] = 8'hf1;
                       memory[53] = 8'h60;
                       memory[52] = 8'h93;
                       
                       
                       // setting 32 bit instrcution for and immediate
                       memory[59] = 8'h00;
                       memory[58] = 8'hf1;
                       memory[57] = 8'h70;
                       memory[56] = 8'h93;
                       
                       //LOAD INSTRUCTIONS I TYPE 
                       //LOAD WORD
                       memory[63] = 8'h00;
                       memory[62] = 8'h43;
                       memory[61] = 8'h02;
                       memory[60] = 8'h83;
                       
                       // STORE INSTUCTION IN I TYPE
                       // STORE WORD
                       memory[67] = 8'h00;
                       memory[66] = 8'h73;
                       memory[65] = 8'h28;
                       memory[64] = 8'h23;
                       
                       
                       //BRANCH INSTRUCTIONS
                       //BRANCH EQUAL
                        
                       memory[71] = 8'H00;
                       memory[70] = 8'H41;
                       memory[69] = 8'H00;
                       memory[68] = 8'H63;
                       
                       
                       //BRANCH NOT EQUAL
                       
                       memory[75] = 8'h00;
                       memory[74] = 8'h20;
                       memory[73] = 8'h94;
                       memory[72] = 8'h63;
                       
                       //HERE AFTER GIVING BNE INSTRUCTION THE PC VALUE DECIMAL 80
                       
                       //BRANCH GREATER THAN OR EQUAL TO INSTRUCTION
                       memory[83] = 8'h00;
                       memory[82] = 8'h41;
                       memory[81] = 8'ha4;
                       memory[80] = 8'h63;
                       
//                       //BRANCH LESS THAN INSTRUCTION
//                       memory[87] = 8'h00;
//                       memory[86] = 8'h20;
//                       memory[85] = 8'hc1;
//                       memory[84] = 8'h63;
                       
                       
                       //LOAD UPPER IMMEDIATE INSTRUCTION
                       memory[91] = 8'h12;
                       memory[90] = 8'h34;
                       memory[89] = 8'h52;
                       memory[88] = 8'hb7;
                       
                       // JUMP AND LINK INSTRUCTION
                       memory[95] = 8'h00;
                       memory[94] = 8'h00 ;
                       memory[93] = 8'h80;
                       memory[92] = 8'hef;
                       
                       
                       
            end
            end

                
endmodule




`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 10:05:38 AM
// Design Name: 
// Module Name: ctrl_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ctrl_unit(input clk,rst,input [6:0] opcode,funct7,input [2:0] funct3,output reg [5:0] alu_control,output reg mem_to_reg,          //control signal for enabling data flow from memory to register
                    output reg bneq_control,        //control signal for enabling bneq operation
                    output reg beq_control,         //control signal for enabling beq operation
                    output reg bgeq_control,        //control signal for enabling bgeq operation
                    output reg blt_control,         //control signal for enabling blt operation
                    output reg jump,                //control signal for enabling jump operation
                    output reg sw);
always@(posedge clk)
begin
    if(rst)
    alu_control = 0;
end

always@(opcode,funct7,funct3)
begin
    if(opcode == 7'b0110011)
    begin
    case(funct3)
    3'b000: begin
                        
//////////////////////////////////////////// ADDITION /////////////////////////////////////////////////////////////////////////////////////////                        
                          
                            if(funct7 == 0)
                                alu_control = 6'b000001;
                                                     
//////////////////////////////////////////// SUBRACTION /////////////////////////////////////////////////////////////////////////////////////////                        
                       
                            else if(funct7 == 64)
                                alu_control = 6'b000010; 
                        end
                        
    //////////////////////////////////////////// SHIFT LEFT LOGICAL /////////////////////////////////////////////////////////////////////////////////////////                        

                    3'b001 :
                          begin
                             if(funct7 == 0)
                                alu_control = 6'b000011;                           
                           end
                           
//////////////////////////////////////////// SET LESS THAN /////////////////////////////////////////////////////////////////////////////////////////                        

                      3'b010 :
                         begin
                              if(funct7 == 0)
                                  alu_control = 6'b000100; 
                           end
                            
//////////////////////////////////////////// SET LESS THAN UNSIGNED /////////////////////////////////////////////////////////////////////////////////////////                        
                           
                       3'b011 :
                            begin
                                if(funct7 == 0)
                                    alu_control = 6'b000101; 
                             end
                             
//////////////////////////////////////////// XOR OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        
                             
                       3'b100 :
                            begin
                                if(funct7 == 0)
                                    alu_control = 6'b000110; 
                                end

//////////////////////////////////////////// SHIFT RIGHT LOGICAL AND ARTHIMETIC /////////////////////////////////////////////////////////////////////////////////////////                        
                       
                       3'b101 :
                            begin
                                if(funct7 == 0)
                                    alu_control = 6'b000111; 
                                else
                                    if(funct7 == 64)
                                        alu_control = 6'b001000;   // shift right arthimetic
                             end


////////////////////////////////////////////  OR OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        

                             
                      3'b110 :
                            begin
                                if(funct7 == 0)
                                   alu_control = 6'b001001; // or operation
                             end


//////////////////////////////////////////// AND OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        
                                 
                      3'b111 :
                            begin
                                if(funct7 == 0)
                                    alu_control = 6'b001010; //and operation
                             end
                  endcase
                  end
                 
                  else if(opcode == 7'b0100_011)
                            begin
                            case(funct3)
                                3'b010 :begin
                                        mem_to_reg = 1;        
                                        beq_control = 0;               
                                        bneq_control = 0;              
                                        jump = 0;
                                        sw = 1;
                                        alu_control = 6'b011000; // store byte
                                        end
                                        
                                3'b110 :begin
                                        mem_to_reg = 1;        
                                        beq_control = 0;               
                                        bneq_control = 0;              
                                        jump = 0;
                                        alu_control = 6'b011001; // store half word
                                        end
                                        
                                3'b111 :begin
                                        mem_to_reg = 1;        
                                        beq_control = 0;               
                                        bneq_control = 0;              
                                        jump = 0;
                                        sw = 1;
                                        alu_control = 6'b011010; // store word
                                        end
                           endcase
                          end
                          
                           else if(opcode == 7'b001_0011)// I TYPE
                  begin
                  
                  mem_to_reg = 0;
                  beq_control = 0;
                  bneq_control = 0;
                  jump = 0;
                 // lb = 0;
                  sw = 0;
                        
                       case(funct3)
                       
///////////////////////////////////////////////// ADD IMMEDIATE  ///////////////////////////////////////////////////////////////////////////////////////

                                3'b000 :
                                        begin
                                            alu_control = 6'b001011; // add immediate
                                        end
                                        endcase
                    end
end

endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 02:28:57 PM
// Design Name: 
// Module Name: reg_file
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module pc_reg_file #(
    parameter NUM_REGS = 32, // Number of PC registers
    parameter PC_WIDTH = 32 // Width of each PC register
) (
    input  logic                  clk,           // Clock
    input  logic                  reset,         // Asynchronous Reset
    input  logic [$clog2(NUM_REGS)-1:0] rd_addr1, // Read address 1
    input  logic [$clog2(NUM_REGS)-1:0] rd_addr2, // Read address 2
    input  logic [$clog2(NUM_REGS)-1:0] wr_addr,  // Write address
    input  logic [PC_WIDTH-1:0]   wr_data,       // Write data
    input  logic                  wr_en,         // Write enable
    output logic [PC_WIDTH-1:0]   rd_data1,      // Read data 1
    output logic [PC_WIDTH-1:0]   rd_data2       // Read data 2
);

    // Internal register array
    logic [PC_WIDTH-1:0] pc_regs [NUM_REGS-1:0];

    // Write Logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all PC registers to 0
            for (int i = 0; i < NUM_REGS; i++) begin
                pc_regs[i] <= i;
            end
        end else if (wr_en) begin
            // Write to the selected register
            pc_regs[wr_addr] <= wr_data;
        end
    end

    // Read Logic (Combinational)
    assign rd_data1 = pc_regs[rd_addr1]; // Read data for address 1
    assign rd_data2 = pc_regs[rd_addr2]; // Read data for address 2

endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/30/2024 03:51:40 PM
// Design Name: 
// Module Name: tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb;

    logic clk; // Clock signal
    logic rst; // Reset signal

    // Clock generation: 10 time units clock period
    initial clk = 0;
    always #5 clk = ~clk; // Toggles every 5 time units (10 time unit period)

    // Reset generation
    initial begin
        rst = 1;         // Assert reset
        #20 rst = 0;     // Deassert reset after 20 time units
    end

    // Monitor signals (optional for debug)
    initial begin
        $monitor("Time: %0t | clk: %b | rst: %b", $time, clk, rst);
        #200 $stop; // End simulation after 100 time units
    end
processor proc(.clk(clk),.rst(rst));
endmodule




