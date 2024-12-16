module processor(input clk,rst);
wire [31:0] instr,data1,data2,wr_data,result,pc,mem_to_reg_data,reg_write_data;
wire [4:0] addr1,addr2,wr_addr;
wire [31:0] imm_val,data_mem_addr,imm_val_s;
wire en,sw,lw;
wire [5:0] alu_control;
data_mem data_mem(.clk(clk),.rst(rst),.mem_read(lw),.mem_write(sw),.address(data_mem_addr),.write_data(data2),.read_data(mem_to_reg_data));
alu alu(.src1(data1),.src2(data2),.alu_control(alu_control),.result(result),.imm_val_r(imm_val));
pc fetch(.clk(clk),.reset(rst),.pc(pc));
instruction_memory mem(.clk(clk),.reset(rst),.pc(pc),.instruction_code(instr));
ctrl_unit ctrl(.clk(clk),.rst(rst),.opcode(instr[6:0]),.funct7(instr[31:25]),.funct3(instr[14:12]),.alu_control(alu_control),.sw(sw));
pc_reg_file reg_file(.clk(clk),.reset(rst),.rd_addr2(instr[24:20]),.rd_addr1(instr[19:15]),.rd_data1(data1),.rd_data2(data2),.wr_data(reg_write_data),.wr_en(en),.wr_addr(instr[11:7]));
assign en = ((instr[6:0] == 'b0110011)||(instr[6:0] == 'b0011))?1:0;
assign imm_val = {{20{instr[31]}},{instr[31:20]}};
assign imm_val_s = {{20{instr[31]}},{instr[31:25]},{instr[11:7]}};
assign lw =(instr[6:0] == 3 )?1:0;
assign data_mem_addr =(instr[6:0] == 3)?(imm_val + data1): (imm_val_s + data1);
assign reg_write_data = (lw == 1)?mem_to_reg_data:result;
endmodule

