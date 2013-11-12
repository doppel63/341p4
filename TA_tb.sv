module test
  (output logic clk, rst_L);
  
  logic [15:0]   flash_addr;
  logic [63:0]  flash_data;
  logic [63:0]  receivedMsg;
  logic         success;
  
  initial 
  begin
    rst_L = 1'b1;
    clk   = 1'b1;
    @(posedge clk) rst_L = 1'b0;
    @(posedge clk) rst_L = 1'b1;
    
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************WRITE AND READ TEST********************************");
    $display("********************************************************************************");
    
///////////////////////////////////////////////////////////////////////////////
// Host sends data to the device to be printed
///////////////////////////////////////////////////////////////////////////////
    flash_addr = 16'hAB;
    flash_data = 64'hCAFEBABEDEADBEEF;
    
    // Write Data
    @(posedge clk);
    $display("");
    $display($time,, "TestBench:\tSending addr 0x%x, data 0x%x to Device",
        flash_addr, flash_data);
    host.writeData(flash_addr, flash_data, success);
    if(!success)
      $display($time,, "TestBench:\tSending addr 0x%x, data 0x%x to Device: timed out", 
          flash_addr, flash_data);
    else
    $display($time,, "TestBench:\tDevice received data success! addr 0x%x, data 0x%x", 
          flash_addr, flash_data);
    
    // Read Data
    @(posedge clk);
    $display("");
    $display($time,, "TestBench:\tRequest data from Device addr 0x%x",
        flash_addr);
    host.readData(flash_addr, receivedMsg, success);
    
    if(!success)
      $display($time,, "TestBench:\tReceiving data from Device: timed out");
    else if(receivedMsg !== flash_data)
      $display($time,, "TestBench:\tReceived 0x%x instead of 0x%x", receivedMsg, flash_data);
    else
      $display($time,, "TestBench:\tHost received correct data! 0x%x", receivedMsg);
    
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************END TEST*******************************************");
    $display("********************************************************************************");
    
    $display("\n");
    $display("********************************************************************************");
    $display("**************************UNINITIALIZED MEMORY TEST*****************************");
    $display("********************************************************************************");
    // Read an address with nothing in it
    @(posedge clk);
    $display("");
    $display($time,, "TestBench:\tRequest data from Device addr 0x%x",
        flash_addr+1);
    receivedMsg = 0;
    host.readData(flash_addr+1, receivedMsg, success);
    
    if(!success)
      $display($time,, "TestBench:\tReceiving data from Device: timed out");
    else if(receivedMsg !== 0)
      $display($time,, "TestBench:\tReceived 0x%x instead of 0x%x",
          receivedMsg, 0);
    else
      $display($time,, "TestBench:\tHost received correct data! 0x%x", receivedMsg);
 
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************END TEST*******************************************");
    $display("********************************************************************************");
     $finish();
  end

  always #1 clk = ~clk;

endmodule
