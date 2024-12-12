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
        #200 $finish; // End simulation after 100 time units
    end
processor proc(.clk(clk),.rst(rst));
   initial
   begin
	   $dumpfile("waveforms.vcd");
	   $dumpvars(0,tb);
   end
endmodule





