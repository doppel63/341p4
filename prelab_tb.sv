module test(
  output logic clk, rst_L);
  
  logic [7:0]   flash_addr;
  logic [63:0]  flash_data;
  logic [63:0]  receivedMsg;
  logic         success;

  // NOTE: For prelab you do not have to do timeouts.  It just needs to send
  // packets
  
  initial begin
    rst_L = 1'b1;
    clk   = 1'b1;
    @(posedge clk) rst_L = 1'b0;
    @(posedge clk) rst_L = 1'b1;
    
    $display("\n");
    $display("***************************************************************");
    $display("**********************WRITE AND READ TEST**********************");
    $display("***************************************************************");
  
///////////////////////////////////////////////////////////////////////////////
// Host sends data to the device to be printed
///////////////////////////////////////////////////////////////////////////////
    
    flash_addr = 8'hAB;
    flash_data = 64'hCAFEBABEDEADBEEF;
    
    // Read Data
    @(posedge clk);
    $display("");
    $display("Sending an OUT to endpoint 4 with data=CAFEBABEDEADBEEF");
    host.prelabRequest(flash_data);
    thumbDrive.outputData();

    @(posedge clk);
    $display("\n");
    $display("***************************************************************");
    $display("***************************END TEST****************************");
    $display("***************************************************************");    
    $finish();
  end
  always #1 clk = ~clk;
endmodule
