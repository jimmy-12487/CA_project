//=========================================================================
// IO2I 2-Wide RISC-V Reorder Buffer and Rename Table
//=========================================================================

`ifndef RISCV_CORE_REORDERBUFFER_V
`define RISCV_CORE_REORDERBUFFER_V

module riscv_CoreReorderBuffer
(
  input         clk,
  input         reset,

  input         brj_taken_X0hl,
  input         brj_resolved_X0hl,

  output        ROB_alloc_valid,

  input         ROB_req_A,
  input         ROB_req_we_A,
  input         ROB_req_spec_A,
  input   [4:0] ROB_req_preg_A,
  input         ROB_req_B,
  input         ROB_req_we_B,
  input         ROB_req_spec_B,
  input   [4:0] ROB_req_preg_B,

  input         ROB_commit_req_A,
  input         ROB_commit_req_B,
  input   [4:0] ROB_commit_req_slot_A,
  input   [4:0] ROB_commit_req_slot_B,

  output        ROB_commit_wen_A,
  output  [4:0] ROB_commit_slot_A,
  output  [4:0] ROB_commit_rdaddr_A,
  output        ROB_commit_ready_A,
  output        ROB_commit_spec_A,
  output        ROB_commit_wen_B,
  output  [4:0] ROB_commit_slot_B,
  output  [4:0] ROB_commit_rdaddr_B,
  output        ROB_commit_ready_B,
  output        ROB_commit_spec_B,

  output  [4:0] ROB_alloc_slot_A,
  output  [4:0] ROB_alloc_slot_B,

  input         raw_hazardA,
  input         raw_hazardB,

  input   [4:0] rs0_i0_addr,
  input   [4:0] rs1_i0_addr,
  input   [4:0] rs0_i1_addr,
  input   [4:0] rs1_i1_addr,

  output        rs0_i0_renamed,
  output        rs1_i0_renamed,
  output        rs0_i1_renamed,
  output        rs1_i1_renamed,

  output  [4:0] rs0_i0_slot,
  output  [4:0] rs1_i0_slot,
  output  [4:0] rs0_i1_slot,
  output  [4:0] rs1_i1_slot  
);

  reg [31:0] ROB_valid;
  reg [31:0] ROB_state;
  reg [31:0] ROB_wen;
  reg [31:0] ROB_spec;
  reg  [4:0] ROB_preg    [31:0];

  reg [4:0] ROB_head;
  reg [4:0] ROB_tail;
  integer i;

  //----------------------------------------------------------------------  
  // Reset Case
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      ROB_head    <= 5'b0;
      ROB_tail    <= 5'b0;

      for (i = 0; i < 32; i = i + 1) begin
        ROB_valid[i]    <= 1'b0;
        ROB_state[i]  <= 1'b0;  
        ROB_wen[i]      <= 1'b0;
        ROB_spec[i]     <= 1'b0;
        ROB_preg[i]     <= 5'b0;
      end
    end
  end

  //----------------------------------------------------------------------
  // ROB Alloc
  //----------------------------------------------------------------------

  wire ROB_entry_in_use_A;
  wire ROB_entry_in_use_B;

  assign ROB_alloc_valid = ((ROB_head != ROB_tail) || ((ROB_head == ROB_tail) && !ROB_valid[ROB_head])) && (ROB_head != ROB_tail + 1);
  assign ROB_alloc_slot_A = (ROB_req_A && ROB_alloc_valid) ? ROB_tail : 0;
  assign ROB_alloc_slot_B = (ROB_req_A && ROB_req_B && ROB_alloc_valid) ? ROB_tail + 1 : (ROB_req_B && ROB_alloc_valid) ? ROB_tail : 0;
  assign ROB_entry_in_use_A = (ROB_req_A && ROB_alloc_valid) ? 1 : 0;
  assign ROB_entry_in_use_B = (ROB_req_B && ROB_alloc_valid) ? 1 : 0;

  always @(posedge clk) begin
    if (ROB_entry_in_use_A && ROB_entry_in_use_B) begin
      ROB_valid[ROB_tail]   <= 1'b1;
      ROB_state[ROB_tail] <= 1'b1;
      ROB_preg[ROB_tail]    <= ROB_req_preg_A;
      ROB_wen[ROB_tail]     <= ROB_req_we_A;
      ROB_spec[ROB_tail]    <= ROB_req_spec_A;

      ROB_valid[ROB_tail + 1'b1]   <= 1'b1;
      ROB_state[ROB_tail + 1'b1] <= 1'b1;
      ROB_preg[ROB_tail + 1'b1]    <= ROB_req_preg_B;
      ROB_wen[ROB_tail + 1'b1]     <= ROB_req_we_B;
      ROB_spec[ROB_tail + 1'b1]    <= ROB_req_spec_B;

      ROB_tail              <= ROB_tail + 2;
    end
    else if (ROB_entry_in_use_A) begin
      ROB_valid[ROB_tail]   <= 1'b1;
      ROB_state[ROB_tail] <= 1'b1;
      ROB_preg[ROB_tail]    <= ROB_req_preg_A;
      ROB_wen[ROB_tail]     <= ROB_req_we_A;
      ROB_spec[ROB_tail]    <= ROB_req_spec_A;

      ROB_tail              <= ROB_tail + 1;
    end
    else if (ROB_entry_in_use_B) begin
      ROB_valid[ROB_tail]   <= 1'b1;
      ROB_state[ROB_tail] <= 1'b1;
      ROB_preg[ROB_tail]    <= ROB_req_preg_B;
      ROB_wen[ROB_tail]     <= ROB_req_we_B;
      ROB_spec[ROB_tail]    <= ROB_req_spec_B;

      ROB_tail              <= ROB_tail + 1;
    end
  end

  //----------------------------------------------------------------------
  // ROB Pending
  //----------------------------------------------------------------------

  // ROB_state- 0: pending 1: finish
  always @(posedge clk) begin
    if(ROB_commit_req_A) begin
      ROB_state[ROB_commit_req_slot_A] <= 1'b0;
    end
    if(ROB_commit_req_B) begin
      ROB_state[ROB_commit_req_slot_B] <= 1'b0;
    end
  end

  //----------------------------------------------------------------------
  // ROB Commit
  //----------------------------------------------------------------------  

  assign ROB_commit_slot_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_head : 0;
  assign ROB_commit_rdaddr_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_preg[ROB_head] : 0;
  assign ROB_commit_wen_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_wen[ROB_head] : 0;
  assign ROB_commit_slot_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) && 
                              (ROB_valid[ROB_head + 1'b1] && !ROB_state[ROB_head + 1'b1] && !(ROB_wen[ROB_head] && ROB_wen[ROB_head + 1'b1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1'b1]))) ? ROB_head + 1'b1 : 0;
  assign ROB_commit_rdaddr_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) && 
                              (ROB_valid[ROB_head + 1'b1] && !ROB_state[ROB_head + 1'b1] && !(ROB_wen[ROB_head] && ROB_wen[ROB_head + 1'b1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1'b1]))) ? ROB_preg[ROB_head + 1'b1] : 0;
  assign ROB_commit_wen_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) && 
                              (ROB_valid[ROB_head + 1'b1] && !ROB_state[ROB_head + 1'b1] && !(ROB_wen[ROB_head] && ROB_wen[ROB_head + 1'b1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1'b1]))) ? ROB_wen[ROB_head + 1'b1] : 0;
  assign ROB_commit_ready_A = ROB_valid[ROB_head] && !ROB_state[ROB_head] ? 1 : 0;
  assign ROB_commit_ready_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) && 
                              (ROB_valid[ROB_head + 1'b1] && !ROB_state[ROB_head + 1'b1] && !(ROB_wen[ROB_head] && ROB_wen[ROB_head + 1'b1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1'b1]))) ? 1 : 0;
  assign ROB_commit_spec_A = ROB_spec[ROB_head];
  assign ROB_commit_spec_B = ROB_spec[ROB_head + 1'b1];
    

  // Update head pointer and ROB state
  always @(posedge clk) begin
    if(ROB_commit_ready_B) begin
      ROB_valid[ROB_head]       <= 1'b0;
      ROB_valid[ROB_head + 1'b1]  <= 1'b0;

      ROB_head                <= ROB_head + 2;
    end
    else if(ROB_commit_ready_A) begin
      ROB_valid[ROB_head] <= 1'b0;

      ROB_head            <= ROB_head + 1;
    end
  end

  //----------------------------------------------------------------------
  // ROB Speculation
  //----------------------------------------------------------------------  
  always @(posedge clk) begin
    if( brj_resolved_X0hl ) begin
      for( i = 0; i < 32; i = i + 1 ) begin
        ROB_wen[i]      = ROB_wen[i] && !(ROB_spec[i] && brj_taken_X0hl);
        ROB_spec[i]     = ROB_spec[i] && brj_taken_X0hl;
        ROB_state[i]  = ROB_state[i] && !(ROB_spec[i] && brj_taken_X0hl);
      end
    end
  end


  //----------------------------------------------------------------------
  // Rename Table
  //----------------------------------------------------------------------  

  reg [31:0] renamed;
  reg [31:0] spec_renamed;
  reg  [4:0] rename_slot   [31:0];
  reg  [4:0] spec_rename_slot [31:0];

  // Reset
  always @ (posedge clk) begin
    if( reset ) begin
      renamed    <= 32'b0;
      spec_renamed  <= 32'b0;

      for( i = 0; i < 32; i = i + 1 ) begin
        rename_slot[i]   <= 5'b0;
        spec_rename_slot[i] <= 5'b0;
      end
    end
  end

  //----------------------------------------------------------------------
  // Rename Table Allocation
  //----------------------------------------------------------------------

  // Only rename if the instruction writes a register
  wire rename_alloc_A = ROB_entry_in_use_A && ROB_req_we_A && ROB_req_preg_A != 5'b0;
  wire rename_alloc_B = ROB_entry_in_use_B && ROB_req_we_B && ROB_req_preg_B != 5'b0;
  always @ (posedge clk) begin
    if(ROB_entry_in_use_A && ROB_entry_in_use_B) begin
      if(ROB_req_we_A && ROB_req_preg_A != 5'b0) begin

        // Check if the instruction is speculative
        if(ROB_req_spec_A) begin
          spec_renamed[ROB_req_preg_A]      <= 1'b1;
          spec_rename_slot[ROB_req_preg_A]  <= ROB_tail;
        end
        else begin
          renamed[ROB_req_preg_A]      <= 1'b1;
          rename_slot[ROB_req_preg_A]  <= ROB_tail;
        end
      end

      if(ROB_req_we_B && ROB_req_preg_B != 5'b0) begin

        // Check if the instruction is speculative
        if(ROB_req_spec_B) begin
          spec_renamed[ROB_req_preg_B]      <= 1'b1;
          spec_rename_slot[ROB_req_preg_B]  <= ROB_tail + 1'b1;
        end
        else begin
          renamed[ROB_req_preg_B]      <= 1'b1;
          rename_slot[ROB_req_preg_B]  <= ROB_tail + 1'b1;
        end
      end
    end
    else if(rename_alloc_A) begin

      // Check if the instruction is speculative
      if(ROB_req_spec_A) begin
        spec_renamed[ROB_req_preg_A]      <= 1'b1;
        spec_rename_slot[ROB_req_preg_A]  <= ROB_tail;
      end
      else begin
        renamed[ROB_req_preg_A]      <= 1'b1;
        rename_slot[ROB_req_preg_A]  <= ROB_tail;
      end
    end
    else if(rename_alloc_B) begin

      // Check if the instruction is speculative
      if(ROB_req_spec_B) begin
        spec_renamed[ROB_req_preg_B]      <= 1'b1;
        spec_rename_slot[ROB_req_preg_B]  <= ROB_tail;
      end
      else begin
        renamed[ROB_req_preg_B]      <= 1'b1;
        rename_slot[ROB_req_preg_B]  <= ROB_tail;
      end
    end
  end

  //----------------------------------------------------------------------
  // Freeing the Rename Table
  //----------------------------------------------------------------------

  always @ (posedge clk) begin
    if(ROB_commit_ready_A) begin
      if( rename_slot[ROB_commit_rdaddr_A] == ROB_head ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rename_alloc_A && ROB_commit_rdaddr_A == ROB_req_preg_A) &&
            !(rename_alloc_B && ROB_commit_rdaddr_A == ROB_req_preg_B) ) begin
          renamed[ROB_commit_rdaddr_A] <= 1'b0; 
        end
      end
    end
    if(ROB_commit_ready_B) begin
      if( rename_slot[ROB_commit_rdaddr_B] == ROB_head + 1'b1 ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rename_alloc_A && ROB_commit_rdaddr_B == ROB_req_preg_A) &&
            !(rename_alloc_B && ROB_commit_rdaddr_B == ROB_req_preg_B) ) begin
          renamed[ROB_commit_rdaddr_B] <= 1'b0;
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Renamed Source Registers
  //----------------------------------------------------------------------

  assign rs0_i0_renamed = (ROB_req_spec_A && spec_renamed[rs0_i0_addr]) ? 1'b1
                       : renamed[rs0_i0_addr];
  assign rs1_i0_renamed = (ROB_req_spec_A && spec_renamed[rs1_i0_addr]) ? 1'b1
                       : renamed[rs1_i0_addr];
  assign rs0_i0_slot = (ROB_req_spec_A && spec_renamed[rs0_i0_addr]) ? spec_rename_slot[rs0_i0_addr]
                    : rename_slot[rs0_i0_addr];
  assign rs1_i0_slot = (ROB_req_spec_A && spec_renamed[rs1_i0_addr]) ? spec_rename_slot[rs1_i0_addr]
                    : rename_slot[rs1_i0_addr];

  assign rs0_i1_renamed = (raw_hazardA) ? 1'b1
                       : (ROB_req_spec_B && spec_renamed[rs0_i1_addr]) ? 1'b1
                       : renamed[rs0_i1_addr];
  assign rs1_i1_renamed = (raw_hazardB) ? 1'b1
                       : (ROB_req_spec_B && spec_renamed[rs1_i1_addr]) ? 1'b1
                       : renamed[rs1_i1_addr];
  assign rs0_i1_slot = (raw_hazardA) ? ROB_tail
                    : (ROB_req_spec_B && spec_renamed[rs0_i1_addr]) ? spec_rename_slot[rs0_i1_addr]
                    : rename_slot[rs0_i1_addr];
  assign rs1_i1_slot = (raw_hazardB) ? ROB_tail
                    : (ROB_req_spec_B && spec_renamed[rs1_i1_addr]) ? spec_rename_slot[rs1_i1_addr]
                    : rename_slot[rs1_i1_addr];

  //----------------------------------------------------------------------
  // Rename Table Speculation
  //----------------------------------------------------------------------

  always @(negedge clk) begin
    if( brj_resolved_X0hl ) begin
      for( i = 0; i < 32; i = i + 1 ) begin
        if( spec_renamed[i] && !brj_taken_X0hl ) begin
          renamed[i]     = 1'b1;
          rename_slot[i] = spec_rename_slot[i];
        end
      end
      spec_renamed = 32'b0;
    end
  end


endmodule

`endif

