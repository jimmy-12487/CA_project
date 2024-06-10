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

  output        rob_alloc_req_rdy,

  input         rob_alloc_req_val_1,
  input         rob_alloc_req_wen_1,
  input         rob_alloc_req_spec_1,
  input   [4:0] rob_alloc_req_preg_1,
  input         rob_alloc_req_val_2,
  input         rob_alloc_req_wen_2,
  input         rob_alloc_req_spec_2,
  input   [4:0] rob_alloc_req_preg_2,

  input         rob_fill_val_1,
  input   [4:0] rob_fill_slot_1,
  input         rob_fill_val_2,
  input   [4:0] rob_fill_slot_2,

  output        rob_commit_wen_1,
  output  [4:0] rob_commit_slot_1,
  output  [4:0] rob_commit_rf_waddr_1,
  output        rob_commit_val_1,
  output        rob_commit_spec_1,
  output        rob_commit_wen_2,
  output  [4:0] rob_commit_slot_2,
  output  [4:0] rob_commit_rf_waddr_2,
  output        rob_commit_val_2,
  output        rob_commit_spec_2,

  output  [4:0] rob_alloc_resp_slot_1,
  output  [4:0] rob_alloc_resp_slot_2,

  input         raw_hazard0,
  input         raw_hazard1,

  input   [4:0] src00,
  input   [4:0] src01,
  input   [4:0] src10,
  input   [4:0] src11,

  output        src00_renamed,
  output        src01_renamed,
  output        src10_renamed,
  output        src11_renamed,

  output  [4:0] src00_slot,
  output  [4:0] src01_slot,
  output  [4:0] src10_slot,
  output  [4:0] src11_slot
);

  reg rob_alloc_req_rdy;

  reg [4:0] rob_alloc_resp_slot_1;
  reg       rob_commit_wen_1;
  reg [4:0] rob_commit_rf_waddr_1;
  reg [4:0] rob_commit_slot_1;

  reg [4:0] rob_alloc_resp_slot_2;
  reg       rob_commit_wen_2;
  reg [4:0] rob_commit_rf_waddr_2;
  reg [4:0] rob_commit_slot_2;

  reg [31:0] rob_valid;
  reg [31:0] rob_pending;
  reg [31:0] rob_wen;
  reg [31:0] rob_spec;
  reg  [4:0] rob_preg    [31:0];

  reg [4:0] rob_head;
  reg [4:0] rob_tail;
  wire [4:0] rob_head_next = rob_head + 1'b1;
  wire [4:0] rob_tail_next = rob_tail + 1'b1;
  integer i;

  //----------------------------------------------------------------------  
  // Reset Case
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      rob_alloc_req_rdy     <= 1'b1;

      rob_alloc_resp_slot_1 <= 5'b0;
      rob_commit_wen_1      <= 1'b0;
      rob_commit_slot_1     <= 5'b0;
      rob_commit_rf_waddr_1 <= 5'b0;

      rob_alloc_resp_slot_2 <= 5'b0;
      rob_commit_wen_2      <= 1'b0;
      rob_commit_slot_2     <= 5'b0;
      rob_commit_rf_waddr_2 <= 5'b0;

      rob_head    <= 5'b0;
      rob_tail    <= 5'b0;

      rob_entry_allocated_1 <= 1'b0;
      rob_entry_allocated_2 <= 1'b0;

      rob_commit_val_1 <= 1'b0;
      rob_commit_val_2 <= 1'b0;

      for (i = 0; i < 32; i = i + 1) begin
        rob_valid[i]    <= 1'b0;
        rob_pending[i]  <= 1'b0;  
        rob_wen[i]      <= 1'b0;
        rob_spec[i]     <= 1'b0;
        rob_preg[i]     <= 5'b0;
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

  reg rob_entry_allocated_1;
  reg rob_entry_allocated_2;

  always @(*) begin

    // Only indicate that the ROB is ready if two slots are open
    rob_alloc_req_rdy = ( (rob_head != rob_tail) ||
                        ( (rob_head == rob_tail) && !rob_valid[rob_head] ) )
                        && (rob_head != rob_tail + 1);

    // Case 1: Allocate two entries
    if(rob_alloc_req_val_1 && rob_alloc_req_val_2 && rob_alloc_req_rdy) begin
      rob_alloc_resp_slot_1 = rob_tail;
      rob_alloc_resp_slot_2 = rob_tail + 1;

      rob_entry_allocated_1 = 1'b1;
      rob_entry_allocated_2 = 1'b1;
    end

    // Case 2: Allocate entry 1 only
    else if(rob_alloc_req_val_1 && rob_alloc_req_rdy) begin
      rob_alloc_resp_slot_1 = rob_tail;

      rob_entry_allocated_1 = 1'b1;
      rob_entry_allocated_2 = 1'b0;
    end

    // Case 3: Allocate entry 2 only
    else if(rob_alloc_req_val_2 && rob_alloc_req_rdy) begin
      rob_alloc_resp_slot_2 = rob_tail;

      rob_entry_allocated_1 = 1'b0;
      rob_entry_allocated_2 = 1'b1;
    end

    // Case 4: No allocation
    else begin
      rob_alloc_resp_slot_1 = 5'b0;
      rob_alloc_resp_slot_2 = 5'b0;
      
      rob_entry_allocated_1 = 1'b0;
      rob_entry_allocated_2 = 1'b0;
    end
  end

  // Update tail pointer, as well as ROB state
  always @(posedge clk) begin
    if (rob_entry_allocated_1 && rob_entry_allocated_2) begin
      rob_valid[rob_tail]   <= 1'b1;
      rob_pending[rob_tail] <= 1'b1;
      rob_preg[rob_tail]    <= rob_alloc_req_preg_1;
      rob_wen[rob_tail]     <= rob_alloc_req_wen_1;
      rob_spec[rob_tail]    <= rob_alloc_req_spec_1;

      rob_valid[rob_tail_next]   <= 1'b1;
      rob_pending[rob_tail_next] <= 1'b1;
      rob_preg[rob_tail_next]    <= rob_alloc_req_preg_2;
      rob_wen[rob_tail_next]     <= rob_alloc_req_wen_2;
      rob_spec[rob_tail_next]    <= rob_alloc_req_spec_2;

      rob_tail              <= rob_tail + 2;
    end
    else if (rob_entry_allocated_1) begin
      rob_valid[rob_tail]   <= 1'b1;
      rob_pending[rob_tail] <= 1'b1;
      rob_preg[rob_tail]    <= rob_alloc_req_preg_1;
      rob_wen[rob_tail]     <= rob_alloc_req_wen_1;
      rob_spec[rob_tail]    <= rob_alloc_req_spec_1;

      rob_tail              <= rob_tail + 1;
    end
    else if (rob_entry_allocated_2) begin
      rob_valid[rob_tail]   <= 1'b1;
      rob_pending[rob_tail] <= 1'b1;
      rob_preg[rob_tail]    <= rob_alloc_req_preg_2;
      rob_wen[rob_tail]     <= rob_alloc_req_wen_2;
      rob_spec[rob_tail]    <= rob_alloc_req_spec_2;

      rob_tail              <= rob_tail + 1;
    end
  end

  //----------------------------------------------------------------------
  // ROB Fill
  //----------------------------------------------------------------------

  // When a result is written to the ROB, flip the pending bit of that slot
  // on the next clock edge.

  always @(posedge clk) begin
    if(rob_fill_val_1) begin
      rob_pending[rob_fill_slot_1] <= 1'b0;
    end
    if(rob_fill_val_2) begin
      rob_pending[rob_fill_slot_2] <= 1'b0;
    end
  end

  //----------------------------------------------------------------------
  // ROB Commit
  //----------------------------------------------------------------------  

  // A result in the ROB is ready to be committed when the entry at the
  // head pointer is both valid and not pending. The output information
  // should be set before the end of the clock cycle, but the head pointer
  // can be iterated at the next clock edge.

  reg rob_commit_val_1;
  reg rob_commit_val_2;

  reg rob_commit_spec_1;
  reg rob_commit_spec_2;

  always @(*) begin

    // Since commits occur in order, check the head pointer first, checking
    // the next entry after only if the first entry is committed
    if(rob_valid[rob_head] && !rob_pending[rob_head]) begin
      rob_commit_slot_1     = rob_head;
      rob_commit_rf_waddr_1 = rob_preg[rob_head];
      rob_commit_wen_1      = rob_wen[rob_head];

      // Check to commit second instruction as well
      // Avoid committing two values to the same register
      if(rob_valid[rob_head_next] && !rob_pending[rob_head_next]
          && !(rob_wen[rob_head] && rob_wen[rob_head_next]
            && (rob_preg[rob_head] == rob_preg[rob_head_next]) ) ) begin
        
          rob_commit_slot_2     = rob_head_next;
          rob_commit_rf_waddr_2 = rob_preg[rob_head_next];
          rob_commit_wen_2      = rob_wen[rob_head_next];

          rob_commit_val_1 = 1'b1;
          rob_commit_val_2 = 1'b1;
      end
      else begin
        rob_commit_slot_2     = 5'b0;
        rob_commit_rf_waddr_2 = 5'b0;
        rob_commit_wen_2      = 1'b0;

        rob_commit_val_1 = 1'b1;
        rob_commit_val_2 = 1'b0;
      end
    end
    else begin
      rob_commit_slot_1     = 5'b0;
      rob_commit_rf_waddr_1 = 5'b0;
      rob_commit_wen_1      = 1'b0;

      rob_commit_slot_2     = 5'b0;
      rob_commit_rf_waddr_2 = 5'b0;
      rob_commit_wen_2      = 1'b0;

      rob_commit_val_1 = 1'b0;
      rob_commit_val_2 = 1'b0;
    end

    rob_commit_spec_1 = rob_spec[rob_head];
    rob_commit_spec_2 = rob_spec[rob_head_next];
  end

  // Update head pointer and ROB state
  always @(posedge clk) begin
    if(rob_commit_val_2) begin
      rob_valid[rob_head]       <= 1'b0;
      rob_valid[rob_head_next]  <= 1'b0;

      rob_head                <= rob_head + 2;
    end
    else if(rob_commit_val_1) begin
      rob_valid[rob_head] <= 1'b0;

      rob_head            <= rob_head + 1;
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
        rob_wen[i]      = rob_wen[i] && !(rob_spec[i] && brj_taken_X0hl);
        rob_spec[i]     = rob_spec[i] && brj_taken_X0hl;
        rob_pending[i]  = rob_pending[i] && !(rob_spec[i] && brj_taken_X0hl);
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
      rt_renamed    <= 32'b0;
      spec_renamed  <= 32'b0;

      for( i = 0; i < 32; i = i + 1 ) begin
        rt_rename_slot[i]   <= 5'b0;
        spec_rename_slot[i] <= 5'b0;
      end
    end
  end

  //----------------------------------------------------------------------
  // Rename Table Allocation
  //----------------------------------------------------------------------

  // Only rename if the instruction writes a register
  wire rt_entry_1 = rob_entry_allocated_1 && rob_alloc_req_wen_1
                  && rob_alloc_req_preg_1 != 5'b0;
  wire rt_entry_2 = rob_entry_allocated_2 && rob_alloc_req_wen_2
                  && rob_alloc_req_preg_2 != 5'b0;
  always @ (posedge clk) begin
    if(rob_entry_allocated_1 && rob_entry_allocated_2) begin
      if(rob_alloc_req_wen_1 && rob_alloc_req_preg_1 != 5'b0) begin

        // Check if the instruction is speculative
        if(rob_alloc_req_spec_1) begin
          spec_renamed[rob_alloc_req_preg_1]      <= 1'b1;
          spec_rename_slot[rob_alloc_req_preg_1]  <= rob_tail;
        end
        else begin
          rt_renamed[rob_alloc_req_preg_1]      <= 1'b1;
          rt_rename_slot[rob_alloc_req_preg_1]  <= rob_tail;
        end
      end

      if(rob_alloc_req_wen_2 && rob_alloc_req_preg_2 != 5'b0) begin

        // Check if the instruction is speculative
        if(rob_alloc_req_spec_2) begin
          spec_renamed[rob_alloc_req_preg_2]      <= 1'b1;
          spec_rename_slot[rob_alloc_req_preg_2]  <= rob_tail_next;
        end
        else begin
          rt_renamed[rob_alloc_req_preg_2]      <= 1'b1;
          rt_rename_slot[rob_alloc_req_preg_2]  <= rob_tail_next;
        end
      end
    end
    else if(rt_entry_1) begin

      // Check if the instruction is speculative
      if(rob_alloc_req_spec_1) begin
        spec_renamed[rob_alloc_req_preg_1]      <= 1'b1;
        spec_rename_slot[rob_alloc_req_preg_1]  <= rob_tail;
      end
      else begin
        rt_renamed[rob_alloc_req_preg_1]      <= 1'b1;
        rt_rename_slot[rob_alloc_req_preg_1]  <= rob_tail;
      end
    end
    else if(rt_entry_2) begin

      // Check if the instruction is speculative
      if(rob_alloc_req_spec_2) begin
        spec_renamed[rob_alloc_req_preg_2]      <= 1'b1;
        spec_rename_slot[rob_alloc_req_preg_2]  <= rob_tail;
      end
      else begin
        rt_renamed[rob_alloc_req_preg_2]      <= 1'b1;
        rt_rename_slot[rob_alloc_req_preg_2]  <= rob_tail;
      end
    end
  end

  //----------------------------------------------------------------------
  // Freeing the Rename Table
  //----------------------------------------------------------------------

  always @ (posedge clk) begin
    if(rob_commit_val_1) begin
      if( rt_rename_slot[rob_commit_rf_waddr_1] == rob_head ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rt_entry_1 && rob_commit_rf_waddr_1 == rob_alloc_req_preg_1) &&
            !(rt_entry_2 && rob_commit_rf_waddr_1 == rob_alloc_req_preg_2) ) begin
          rt_renamed[rob_commit_rf_waddr_1] <= 1'b0;
        end
      end
    end
    if(rob_commit_val_2) begin
      if( rt_rename_slot[rob_commit_rf_waddr_2] == rob_head_next ) begin
        // Check that the same architectural register isn't also being renamed
        if( !(rt_entry_1 && rob_commit_rf_waddr_2 == rob_alloc_req_preg_1) &&
            !(rt_entry_2 && rob_commit_rf_waddr_2 == rob_alloc_req_preg_2) ) begin
          rt_renamed[rob_commit_rf_waddr_2] <= 1'b0;
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Renamed Source Registers
  //----------------------------------------------------------------------

  // First Allocation
  assign src00_renamed = (rob_alloc_req_spec_1 && spec_renamed[src00]) ? 1'b1
                       : rt_renamed[src00];
  assign src01_renamed = (rob_alloc_req_spec_1 && spec_renamed[src01]) ? 1'b1
                       : rt_renamed[src01];
  assign src00_slot = (rob_alloc_req_spec_1 && spec_renamed[src00]) ? spec_rename_slot[src00]
                    : rt_rename_slot[src00];
  assign src01_slot = (rob_alloc_req_spec_1 && spec_renamed[src01]) ? spec_rename_slot[src01]
                    : rt_rename_slot[src01];

  assign src10_renamed = (raw_hazard0) ? 1'b1
                       : (rob_alloc_req_spec_2 && spec_renamed[src10]) ? 1'b1
                       : rt_renamed[src10];
  assign src11_renamed = (raw_hazard1) ? 1'b1
                       : (rob_alloc_req_spec_2 && spec_renamed[src11]) ? 1'b1
                       : rt_renamed[src11];
  assign src10_slot = (raw_hazard0) ? rob_tail
                    : (rob_alloc_req_spec_2 && spec_renamed[src10]) ? spec_rename_slot[src10]
                    : rt_rename_slot[src10];
  assign src11_slot = (raw_hazard1) ? rob_tail
                    : (rob_alloc_req_spec_2 && spec_renamed[src11]) ? spec_rename_slot[src11]
                    : rt_rename_slot[src11];

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

  // initial begin
  //   for(i = 0; i < 32; i = i + 1) begin
  //     $dumpvars(0, rt_rename_slot[i]);
  //     $dumpvars(0, spec_rename_slot[i]);
  //   end
  // end

endmodule

`endif

