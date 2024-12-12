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

