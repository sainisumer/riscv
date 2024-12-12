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

