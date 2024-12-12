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

