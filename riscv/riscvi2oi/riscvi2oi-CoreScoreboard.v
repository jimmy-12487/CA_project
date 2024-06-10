//=========================================================================
// IO2I 2-Wide RISC-V Scoreboard
//=========================================================================

`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

module riscv_CoreScoreboard
(
  input         clk,
  input         reset,

  input  [ 4:0] srcA0,            // Source 0 of A
  input         srcA0_en,
  input         srcA0_renamed,
  input  [ 4:0] srcA1,            // Source 1 of A
  input         srcA1_en,
  input         srcA1_renamed,
  input  [ 4:0] dstA,             // Instruction A destination ROB slot
  input         dstA_en,          // Instruction A write enable
  input  [ 1:0] func_irA,         // IrA type
  input         A_issued,

  input  [ 4:0] srcB0,            // Source 0 of B
  input         srcB0_en,
  input         srcB0_renamed,
  input  [ 4:0] srcB1,            // Source 1 of B
  input         srcB1_en,
  input         srcB1_renamed,
  input  [ 4:0] dstB,             // Instruction B destination ROB slot
  input         dstB_en,          // Instruction B write enable
  input         B_issued,

  input         stall_X0hl,
  input         stall_X1hl,
  input         stall_X2hl,
  input         stall_X3hl,
  input         stall_Whl,

  input  [ 4:0] rob_commit_slot_1,  // First committed slot in the ROB
  input         rob_commit_val_1,   // Ensure ROB slot 1 is valid
  input  [ 4:0] rob_commit_slot_2,  // Second committed slot in the ROB
  input         rob_commit_val_2,   // Ensure ROB slot 2 is valid

  output [ 3:0] opA0_byp_mux_sel,   // srcA0 bypass signal
  output [ 3:0] opA1_byp_mux_sel,   // srcA1 bypass signal
  output [ 3:0] opB0_byp_mux_sel,   // srcB0 bypass signal
  output [ 3:0] opB1_byp_mux_sel,   // srcB1 bypass signal

  output [31:0] src_ready
);

  reg [31:0] pending;
  reg [31:0] pipeline;
  reg [31:0] src_ready;
  reg [ 4:0] reg_latency  [31:0];
  reg [ 4:0] rob_slots    [31:0];
  reg [ 1:0] func_unit    [31:0];

  localparam alu_inst     = 2'b00;
  localparam mem_inst     = 2'b01;
  localparam muldiv_inst  = 2'b10;

  wire [4:0] stalls = { stall_X0hl,
                        stall_X1hl,
                        stall_X2hl,
                        stall_X3hl,
                        stall_Whl};

  wire [4:0] alu_st = { 3'b0, stall_X0hl, stall_Whl };

  integer i;
  always @ ( posedge clk ) begin
    if ( reset ) begin
      pending  <= 32'b0;
      pipeline <= 32'b0;

      for(i = 0; i < 32; i = i + 1) begin
        reg_latency[i] = 5'b00000;
        func_unit[i]   = 2'b00;
      end
    end else begin
      //----------------------------------------------------------------------
      // Issue Logic
      //----------------------------------------------------------------------

      // If an instruction is issued, update the scoreboard if the instruction
      // writes to the register file. Index into the scoreboard using the write
      // address of the instruction.
      // For all registers where a write-enabled instruction is not issued, set
      // the X0 scoreboard entry to 0.
      // The scoreboard should only issue if X0 is not stalled.
      // Pipeline 0 is A, pipeline 1 is B

      for ( i = 0; i < 32; i = i + 1 ) begin
        if ( A_issued && dstA_en && ( i == dstA ) ) begin
          reg_latency[i] <= 5'b10000;
          pipeline[i]   <= 1'b0;
          pending[i]    <= 1'b1;
          func_unit[i]  <= func_irA;
          rob_slots[i]  <= i;
        end
        else if ( B_issued && dstB_en && ( i == dstB ) ) begin
          reg_latency[i] <= 5'b00010;
          pipeline[i]   <= 1'b1;
          pending[i]    <= 1'b1;
          func_unit[i]  <= alu_inst;
          rob_slots[i]  <= i;
        end

        //----------------------------------------------------------------------
        // Shift and Pending Logic
        //----------------------------------------------------------------------

        // Shift reg_latency based on the combined stall vector.
        // The pending bit is set to 1 if the instruction is still in the pipeline,
        // not necessarily if it cannot yet be bypassed.
        // Pending bit gets reset to 0 once the instruction is committed in the ROB
        else begin
          if(!pipeline[i]) begin
            reg_latency[i] <= ( reg_latency[i] & stalls ) |
                              ( ( reg_latency[i] & ~stalls ) >> 1 );            
          end
          else begin
            reg_latency[i] <= ( reg_latency[i] & alu_st ) |
                              ( ( reg_latency[i] & ~alu_st ) >> 1 );
          end

          if ( rob_commit_val_1 && rob_commit_slot_1 == i ) begin
            pending[i] <= 1'b0;
          end
          else if ( rob_commit_val_2 && rob_commit_slot_2 == i ) begin
            pending[i] <= 1'b0;
          end
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Bypassing Logic
  //----------------------------------------------------------------------

  // Bypass mux selects
  localparam byp_r0  = 4'd0;  // Use rdata0
  localparam byp_AX0 = 4'd1;  // Bypass from AX0
  localparam byp_AX1 = 4'd2;  // Bypass from AX1
  localparam byp_AX2 = 4'd3;  // Bypass from AX2
  localparam byp_AX3 = 4'd4;  // Bypass from AX3
  localparam byp_AW  = 4'd5;  // Bypass from AW
  localparam byp_BX0 = 4'd6;  // Bypass from BX0
  localparam byp_BX1 = 4'd7;  // Bypass from BX1
  localparam byp_BX2 = 4'd8;  // Bypass from BX2
  localparam byp_BX3 = 4'd9;  // Bypass from BX3
  localparam byp_BW  = 4'd10; // Bypass from BW
  localparam byp_ROB = 4'd11; // Bypass from reorder buffer

  wire rsA0_bypA_cond_Dhl = !pipeline[srcA0] && srcA0_renamed && pending[srcA0];
  wire rsA0_bypB_cond_Dhl = pipeline[srcA0]  && srcA0_renamed && pending[srcA0];

  wire rsA1_bypA_cond_Dhl = !pipeline[srcA1] && srcA1_renamed && pending[srcA1];
  wire rsA1_bypB_cond_Dhl = pipeline[srcA1]  && srcA1_renamed && pending[srcA1];

  wire rsB0_bypA_cond_Dhl = !pipeline[srcB0] && srcB0_renamed && pending[srcB0];
  wire rsB0_bypB_cond_Dhl = pipeline[srcB0]  && srcB0_renamed && pending[srcB0];

  wire rsB1_bypA_cond_Dhl = !pipeline[srcB1] && srcB1_renamed && pending[srcB1];
  wire rsB1_bypB_cond_Dhl = pipeline[srcB1]  && srcB1_renamed && pending[srcB1];

  // Operand Bypass Mux Select

  wire [3:0] opA0_byp_mux_sel
    = (rsA0_bypA_cond_Dhl && ( reg_latency[srcA0] == 5'b10000)) ? byp_AX0
    : (rsA0_bypA_cond_Dhl && ( reg_latency[srcA0] == 5'b01000)) ? byp_AX1
    : (rsA0_bypA_cond_Dhl && ( reg_latency[srcA0] == 5'b00100)) ? byp_AX2
    : (rsA0_bypA_cond_Dhl && ( reg_latency[srcA0] == 5'b00010)) ? byp_AX3
    : (rsA0_bypA_cond_Dhl && ( reg_latency[srcA0] == 5'b00001)) ? byp_AW

    : (rsA0_bypB_cond_Dhl && ( reg_latency[srcA0] == 5'b00010)) ? byp_BX0
    : (rsA0_bypB_cond_Dhl && ( reg_latency[srcA0] == 5'b00001)) ? byp_BW

    : (pending[srcA0] && srcA0_renamed && (reg_latency[srcA0] == 5'd0)) ? byp_ROB
    :                                                byp_r0;  

  wire [3:0] opA1_byp_mux_sel
    = (rsA1_bypA_cond_Dhl && ( reg_latency[srcA1] == 5'b10000)) ? byp_AX0
    : (rsA1_bypA_cond_Dhl && ( reg_latency[srcA1] == 5'b01000)) ? byp_AX1
    : (rsA1_bypA_cond_Dhl && ( reg_latency[srcA1] == 5'b00100)) ? byp_AX2
    : (rsA1_bypA_cond_Dhl && ( reg_latency[srcA1] == 5'b00010)) ? byp_AX3
    : (rsA1_bypA_cond_Dhl && ( reg_latency[srcA1] == 5'b00001)) ? byp_AW

    : (rsA1_bypB_cond_Dhl && ( reg_latency[srcA1] == 5'b00010)) ? byp_BX0
    : (rsA1_bypB_cond_Dhl && ( reg_latency[srcA1] == 5'b00001)) ? byp_BW

    : (pending[srcA1] && srcA1_renamed && (reg_latency[srcA1] == 5'd0)) ? byp_ROB
    :                                                byp_r0;

  wire [3:0] opB0_byp_mux_sel
    = (rsB0_bypA_cond_Dhl && ( reg_latency[srcB0] == 5'b10000)) ? byp_AX0
    : (rsB0_bypA_cond_Dhl && ( reg_latency[srcB0] == 5'b01000)) ? byp_AX1
    : (rsB0_bypA_cond_Dhl && ( reg_latency[srcB0] == 5'b00100)) ? byp_AX2
    : (rsB0_bypA_cond_Dhl && ( reg_latency[srcB0] == 5'b00010)) ? byp_AX3
    : (rsB0_bypA_cond_Dhl && ( reg_latency[srcB0] == 5'b00001)) ? byp_AW

    : (rsB0_bypB_cond_Dhl && ( reg_latency[srcB0] == 5'b00010)) ? byp_BX0
    : (rsB0_bypB_cond_Dhl && ( reg_latency[srcB0] == 5'b00001)) ? byp_BW

    : (pending[srcB0] && srcB0_renamed && (reg_latency[srcB0] == 5'd0)) ? byp_ROB
    :                                                byp_r0;

  wire [3:0] opB1_byp_mux_sel
    = (rsB1_bypA_cond_Dhl && ( reg_latency[srcB1] == 5'b10000)) ? byp_AX0
    : (rsB1_bypA_cond_Dhl && ( reg_latency[srcB1] == 5'b01000)) ? byp_AX1
    : (rsB1_bypA_cond_Dhl && ( reg_latency[srcB1] == 5'b00100)) ? byp_AX2
    : (rsB1_bypA_cond_Dhl && ( reg_latency[srcB1] == 5'b00010)) ? byp_AX3
    : (rsB1_bypA_cond_Dhl && ( reg_latency[srcB1] == 5'b00001)) ? byp_AW

    : (rsB1_bypB_cond_Dhl && ( reg_latency[srcB1] == 5'b00010)) ? byp_BX0
    : (rsB1_bypB_cond_Dhl && ( reg_latency[srcB1] == 5'b00001)) ? byp_BW

    : (pending[srcB1] && srcB1_renamed && (reg_latency[srcB1] == 5'd0)) ? byp_ROB
    :                                                byp_r0;

  //----------------------------------------------------------------------
  // Source Ready Logic
  //----------------------------------------------------------------------

  always @(*) begin
    for( i = 0; i < 32; i = i + 1 ) begin
      src_ready[i] = !pending[i]                  ? 1'b1
                 : (func_unit[i] == alu_inst)     ? 1'b1
                 : (func_unit[i] == mem_inst)     ? reg_latency[i] < 5'b10000
                 : (func_unit[i] == muldiv_inst)  ? reg_latency[i] < 5'b00100
                 : (reg_latency[i] == 5'b00000)   ? 1'b1
                 : 1'b0;
    end
  end

  // initial begin
  //   for(i = 0; i < 32; i = i + 1) begin
  //     $dumpvars(0, rob_slots[i]);
  //     $dumpvars(0, reg_latency[i]);
  //   end
  // end

endmodule

`endif

