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

  input         ROB_req_A,
  input         ROB_req_we_A,
  input         ROB_req_spec_A,
  input   [4:0] ROB_req_preg_A,
  
  input         ROB_req_B,
  input         ROB_req_we_B,
  input         ROB_req_spec_B,
  input   [4:0] ROB_req_preg_B,

  input         ROB_commit_req_A, 
  input   [4:0] ROB_commit_req_slot_A,
  input         ROB_commit_req_B,
  input   [4:0] ROB_commit_req_slot_B,

  output        ROB_alloc_valid,
  output        ROB_commit_we_A,
  output [4:0]  ROB_commit_slot_A,
  output [4:0]  ROB_commit_rdaddr_A,
  output        ROB_commit_ready_A,
  output        ROB_commit_spec_A,
  output        ROB_commit_we_B,
  output [4:0]  ROB_commit_slot_B,
  output [4:0]  ROB_commit_rdaddr_B,
  output        ROB_commit_ready_B,
  output        ROB_commit_spec_B,

  output [4:0]  ROB_alloc_slot_A,
  output [4:0]  ROB_alloc_slot_B,

  input         raw_hazard0,
  input         raw_hazard1,

  input   [4:0] rs0_i1,
  input   [4:0] rs1_i1,
  input   [4:0] rs0_i2,
  input   [4:0] rs1_i2,

  output        rs0_i1_renamed,
  output        rs1_i1_renamed,
  output        rs0_i2_renamed,
  output        rs1_i2_renamed,

  output  [4:0] rs0_i1_slot,
  output  [4:0] rs1_i1_slot,
  output  [4:0] rs0_i2_slot,
  output  [4:0] rs1_i2_slot
);

  reg [4:0]  ROB_head;
  reg [4:0]  ROB_tail;
  reg [31:0] ROB_valid;
  reg [31:0] ROB_we;
  reg [31:0] ROB_spec;
  reg [31:0] ROB_state;   // Pending:0, Finish:1
  reg  [4:0] ROB_preg     [31:0];

  wire ROB_entry_in_use_A;
  wire ROB_entry_in_use_B;

  integer i;

  //----------------------------------------------------------------------  
  // Reset ROB
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      for (i = 0; i < 32; i = i + 1) begin
        ROB_valid[i]    <= 0;
        ROB_we[i]      <= 0;
        ROB_spec[i]     <= 0;
        ROB_state[i]    <= 0;  
        ROB_preg[i]     <= 0;
      end
    end
  end

  //----------------------------------------------------------------------
  // ROB Alloc
  //----------------------------------------------------------------------

  // On a ROB allocation, the ROB slot must be returned before the end of
  // the clock cycle. The actual changes to the entry may be changed on
  // the next clock edge.
  // The ROB is only not ready for allocation if the head and tail pointers
  // are equal and the entry they point to is valid.


  assign ROB_alloc_valid = (((ROB_head == ROB_tail) && !ROB_valid[ROB_head]) || (ROB_head != ROB_tail)) && (ROB_head != ROB_tail + 1);
  assign ROB_alloc_slot_A = (ROB_req_A && ROB_alloc_valid) ? ROB_tail : 0;
  assign ROB_alloc_slot_B = (ROB_req_B && ROB_alloc_valid) ? (ROB_req_A ? ROB_tail + 1 : ROB_tail) : 0;
  assign ROB_entry_in_use_A = (ROB_req_A && ROB_alloc_valid) ? 1 : 0;
  assign ROB_entry_in_use_B = (ROB_req_B && ROB_alloc_valid) ? 1 : 0;

  // Update ROB state
  always @(posedge clk) begin
    if (ROB_entry_in_use_A) begin
      ROB_valid[ROB_alloc_slot_A]   <= 1;
      ROB_state[ROB_alloc_slot_A]   <= 1;
      ROB_preg[ROB_alloc_slot_A]    <= ROB_req_preg_A;
      ROB_we[ROB_alloc_slot_A]      <= ROB_req_we_A;
      ROB_spec[ROB_alloc_slot_A]    <= ROB_req_spec_A;
    end
    if (ROB_entry_in_use_B) begin
      ROB_valid[ROB_alloc_slot_B]   <= 1;
      ROB_state[ROB_alloc_slot_B]   <= 1;
      ROB_preg[ROB_alloc_slot_B]    <= ROB_req_preg_B;
      ROB_we[ROB_alloc_slot_B]      <= ROB_req_we_B;
      ROB_spec[ROB_alloc_slot_B]    <= ROB_req_spec_B;
    end
  end

  always @(posedge clk) begin
    if(reset) begin
      ROB_head    <= 0;
      ROB_tail    <= 0;
    end
    else begin
      if(ROB_entry_in_use_A && ROB_entry_in_use_B) ROB_tail <= ROB_tail + 2;
      else if(ROB_entry_in_use_A) ROB_tail <= ROB_tail + 1;
      else if(ROB_entry_in_use_B) ROB_tail <= ROB_tail + 1;
    end
  end
  //----------------------------------------------------------------------
  // Set Pending
  //----------------------------------------------------------------------

  // When a result is written to the ROB, flip the pending bit of that slot
  // on the next clock edge.

  always @(posedge clk) begin
    if(ROB_commit_req_A) begin
      ROB_state[ROB_commit_req_slot_A] <= 0;
    end
    if(ROB_commit_req_B) begin
      ROB_state[ROB_commit_req_slot_B] <= 0;
    end
  end

  //----------------------------------------------------------------------
  // ROB Commit
  //----------------------------------------------------------------------  

  // A result in the ROB is ready to be committed when the entry at the
  // head pointer is both valid and not pending. The output information
  // should be set before the end of the clock cycle, but the head pointer
  // can be iterated at the next clock edge.
  
    
  // Since commits occur in order, check the head pointer first, checking
  // the next entry after only if the first entry is committed
  // Check to commit second instruction as well
  // Avoid committing two values to the same register

  assign ROB_commit_slot_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_head : 0;
  assign ROB_commit_slot_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head] && ROB_valid[ROB_head + 1] && !ROB_state[ROB_head + 1] && !(ROB_we[ROB_head] && ROB_we[ROB_head + 1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1]))) ? ROB_head + 1 : 0;
  assign ROB_commit_rdaddr_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_preg[ROB_head] : 0;
  assign ROB_commit_rdaddr_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head] && ROB_valid[ROB_head + 1] && !ROB_state[ROB_head + 1] && !(ROB_we[ROB_head] && ROB_we[ROB_head + 1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1]))) ? ROB_preg[ROB_head + 1] : 0;
  assign ROB_commit_we_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? ROB_we[ROB_head] : 0;
  assign ROB_commit_we_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head] && ROB_valid[ROB_head + 1] && !ROB_state[ROB_head + 1] && !(ROB_we[ROB_head] && ROB_we[ROB_head + 1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1]))) ? ROB_we[ROB_head + 1] : 0;
  assign ROB_commit_ready_A = (ROB_valid[ROB_head] && !ROB_state[ROB_head]) ? 1 : 0;
  assign ROB_commit_ready_B = (ROB_valid[ROB_head] && !ROB_state[ROB_head] && ROB_valid[ROB_head + 1] && !ROB_state[ROB_head + 1] && !(ROB_we[ROB_head] && ROB_we[ROB_head + 1] && (ROB_preg[ROB_head] == ROB_preg[ROB_head + 1]))) ? 1 : 0;
  assign ROB_commit_spec_A = ROB_spec[ROB_head];
  assign ROB_commit_spec_B = ROB_spec[ROB_head + 1];
    


  // Update head pointer and ROB state
  always @(posedge clk) begin
    if(ROB_commit_ready_B) begin
      ROB_valid[ROB_head]       <= 1'b0;
      ROB_valid[ROB_head + 1]  <= 1'b0;
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

  // When the branch resolves, if it is not taken, unset speculative bits,
  // else disable speculative instructions from writing the register file.
  // On a mispredict, set the pending bit of speculative instructions to 0
  // so they can be immediately committed and removed.

  always @(posedge clk) begin
    if( brj_resolved_X0hl ) begin
      for( i = 0; i < 32; i = i + 1 ) begin
        ROB_we[i]       = ROB_we[i] && !(ROB_spec[i] && brj_taken_X0hl);
        ROB_spec[i]     = ROB_spec[i] && brj_taken_X0hl;
      end
    end
  end


  //----------------------------------------------------------------------
  // Rename Table
  //----------------------------------------------------------------------  

  reg [31:0] rt_renamed;
  reg [31:0] spec_renamed;
  reg  [4:0] rt_rename_slot   [31:0];
  reg  [4:0] spec_rename_slot [31:0];

  // Reset
  always @ (posedge clk) begin
    if( reset ) begin
      rt_renamed    <= 0;
      spec_renamed  <= 0;

      for( i = 0; i < 32; i = i + 1 ) begin
        rt_rename_slot[i]   <= 0;
        spec_rename_slot[i] <= 0;
      end
    end
  end

  //----------------------------------------------------------------------
  // Rename Table Allocation
  //----------------------------------------------------------------------

  // Only rename if the instruction writes a register
  wire rt_entry_1 = ROB_entry_in_use_A && ROB_req_we_A
                  && ROB_req_preg_A != 0;
  wire rt_entry_2 = ROB_entry_in_use_B && ROB_req_we_B
                  && ROB_req_preg_B != 0;
  always @ (posedge clk) begin
    if(ROB_entry_in_use_A && ROB_entry_in_use_B) begin
      if(ROB_req_we_A && ROB_req_preg_A != 0) begin

        // Check if the instruction is speculative
        if(ROB_req_spec_A) begin
          spec_renamed[ROB_req_preg_A]      <= 1;
          spec_rename_slot[ROB_req_preg_A]  <= ROB_tail;
        end
        else begin
          rt_renamed[ROB_req_preg_A]      <= 1;
          rt_rename_slot[ROB_req_preg_A]  <= ROB_tail;
        end
      end

      if(ROB_req_we_B && ROB_req_preg_B != 0) begin

        // Check if the instruction is speculative
        if(ROB_req_spec_B) begin
          spec_renamed[ROB_req_preg_B]      <= 1;
          spec_rename_slot[ROB_req_preg_B]  <= ROB_tail + 1;
        end
        else begin
          rt_renamed[ROB_req_preg_B]      <= 1;
          rt_rename_slot[ROB_req_preg_B]  <= ROB_tail + 1;
        end
      end
    end
    else if(rt_entry_1) begin

      // Check if the instruction is speculative
      if(ROB_req_spec_A) begin
        spec_renamed[ROB_req_preg_A]      <= 1'b1;
        spec_rename_slot[ROB_req_preg_A]  <= ROB_tail;
      end
      else begin
        rt_renamed[ROB_req_preg_A]      <= 1'b1;
        rt_rename_slot[ROB_req_preg_A]  <= ROB_tail;
      end
    end
    else if(rt_entry_2) begin

      // Check if the instruction is speculative
      if(ROB_req_spec_B) begin
        spec_renamed[ROB_req_preg_B]      <= 1'b1;
        spec_rename_slot[ROB_req_preg_B]  <= ROB_tail;
      end
      else begin
        rt_renamed[ROB_req_preg_B]      <= 1'b1;
        rt_rename_slot[ROB_req_preg_B]  <= ROB_tail;
      end
    end
  end

  //----------------------------------------------------------------------
  // Freeing the Rename Table
  //----------------------------------------------------------------------

  always @ (posedge clk) begin
    if(ROB_commit_ready_A) begin
      if( rt_rename_slot[ROB_commit_rdaddr_A] == ROB_head ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rt_entry_1 && ROB_commit_rdaddr_A == ROB_req_preg_A) &&
            !(rt_entry_2 && ROB_commit_rdaddr_A == ROB_req_preg_B) ) begin
          rt_renamed[ROB_commit_rdaddr_A] <= 1'b0;
        end
      end
    end
    if(ROB_commit_ready_B) begin
      if( rt_rename_slot[ROB_commit_rdaddr_B] == ROB_head + 1 ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rt_entry_1 && ROB_commit_rdaddr_B == ROB_req_preg_A) &&
            !(rt_entry_2 && ROB_commit_rdaddr_B == ROB_req_preg_B) ) begin
          rt_renamed[ROB_commit_rdaddr_B] <= 1'b0;
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Renamed Source Registers
  //----------------------------------------------------------------------

  // First Allocation
  assign rs0_i1_renamed = (ROB_req_spec_A && spec_renamed[rs0_i1]) ? 1'b1
                       : rt_renamed[rs0_i1];
  assign rs1_i1_renamed = (ROB_req_spec_A && spec_renamed[rs1_i1]) ? 1'b1
                       : rt_renamed[rs1_i1];
  assign rs0_i1_slot = (ROB_req_spec_A && spec_renamed[rs0_i1]) ? spec_rename_slot[rs0_i1]
                    : rt_rename_slot[rs0_i1];
  assign rs1_i1_slot = (ROB_req_spec_A && spec_renamed[rs1_i1]) ? spec_rename_slot[rs1_i1]
                    : rt_rename_slot[rs1_i1];

  assign rs0_i2_renamed = (raw_hazard0) ? 1'b1
                       : (ROB_req_spec_B && spec_renamed[rs0_i2]) ? 1'b1
                       : rt_renamed[rs0_i2];
  assign rs1_i2_renamed = (raw_hazard1) ? 1'b1
                       : (ROB_req_spec_B && spec_renamed[rs1_i2]) ? 1'b1
                       : rt_renamed[rs1_i2];
  assign rs0_i2_slot = (raw_hazard0) ? ROB_tail
                    : (ROB_req_spec_B && spec_renamed[rs0_i2]) ? spec_rename_slot[rs0_i2]
                    : rt_rename_slot[rs0_i2];
  assign rs1_i2_slot = (raw_hazard1) ? ROB_tail
                    : (ROB_req_spec_B && spec_renamed[rs1_i2]) ? spec_rename_slot[rs1_i2]
                    : rt_rename_slot[rs1_i2];

  //----------------------------------------------------------------------
  // Rename Table Speculation
  //----------------------------------------------------------------------

  // If a branch resolves to taken, clear the speculative rename table.
  // If it is not taken, set the non-speculative rename table equal the the
  // speculative entries, then clear the speculative rename table.

  always @(negedge clk) begin
    if( brj_resolved_X0hl ) begin
      for( i = 0; i < 32; i = i + 1 ) begin
        if( spec_renamed[i] && !brj_taken_X0hl ) begin
          rt_renamed[i]     = 1'b1;
          rt_rename_slot[i] = spec_rename_slot[i];
        end
      end
      spec_renamed = 32'b0;
    end
  end

endmodule

`endif

