module main_test_bench();

reg rst, clk;
//reg [15:0] label_address;
wire alu_z;						// An LED turned ON if result is zero
wire [7:0] Anode_Activate;		// Anodes to control 7-segments
wire [6:0] LED_out;


main dut(.rst(rst), .clk(clk),.Anode_Activate(Anode_Activate), .LED_out(LED_out));


initial begin

clk=1'b0;
rst = 1'b1;
#100 rst=1'b0; 


end

always #10 clk=~clk;

endmodule