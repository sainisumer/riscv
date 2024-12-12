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
                          
if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000001;end
                                                     
//////////////////////////////////////////// SUBRACTION /////////////////////////////////////////////////////////////////////////////////////////                        
                       
else if(funct7 == 64)begin
                                        sw = 0;
					alu_control = 6'b000010; end
                        end
                        
    //////////////////////////////////////////// SHIFT LEFT LOGICAL /////////////////////////////////////////////////////////////////////////////////////////                        

                    3'b001 :
                          begin
				  if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000011;    end                       
                           end
                           
//////////////////////////////////////////// SET LESS THAN /////////////////////////////////////////////////////////////////////////////////////////                        

                      3'b010 :
                         begin
				 if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000100; end
                           end
                            
//////////////////////////////////////////// SET LESS THAN UNSIGNED /////////////////////////////////////////////////////////////////////////////////////////                        
                           
                       3'b011 :
                            begin
				    if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000101; end
                             end
                             
//////////////////////////////////////////// XOR OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        
                             
                       3'b100 :
                            begin
				    if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000110; end
                                end

//////////////////////////////////////////// SHIFT RIGHT LOGICAL AND ARTHIMETIC /////////////////////////////////////////////////////////////////////////////////////////                        
                       
                       3'b101 :
                            begin
				    if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b000111; end
                                else
					if(funct7 == 64) begin
                                        sw = 0;
					alu_control = 6'b001000; end  // shift right arthimetic
                             end


////////////////////////////////////////////  OR OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        

                             
                      3'b110 :
                            begin
				    if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b001001;end // or operation
                             end


//////////////////////////////////////////// AND OPERATION /////////////////////////////////////////////////////////////////////////////////////////                        
                                 
                      3'b111 :
                            begin
				    if(funct7 == 0) begin
                                        sw = 0;
					alu_control = 6'b001010;end //and operation
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
                                        sw = 0;
                                        end
                                        endcase
                    end
end

endmodule

