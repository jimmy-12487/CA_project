//=========================================================================
// IO2I 2-Wide RISC-V Scoreboard
//=========================================================================

`ifndef RISCV_CORE_SCOREBOARD_V
`define RISCV_CORE_SCOREBOARD_V

module riscv_CoreScoreboard
(
  input         clk,
  input         reset,

// wide 0
  input  [ 4:0] rs0_A,            // Source 0 of A
  input         rs0_A_en,
  input         rs0_A_renamed,
  input  [ 4:0] rs1_A,            // Source 1 of A
  input         rs1_A_en,
  input         rs1_A_renamed,
  input  [ 4:0] rd_A,             // Instruction A destination ROB slot
  input         rd_A_en,          // Instruction A write enable
  input  [ 1:0] func_irA,         // IrA type
  input         A_issued,

// wide 1
  input  [ 4:0] rs0_B,            // Source 0 of B
  input         rs0_B_en,
  input         rs0_B_renamed,
  input  [ 4:0] rs1_B,            // Source 1 of B
  input         rs1_B_en,
  input         rs1_B_renamed,
  input  [ 4:0] rd_B,             // Instruction B destination ROB slot
  input         rd_B_en,          // Instruction B write enable
  input         B_issued,

// stall signal of X0~X3
  input         stall_X0hl,
  input         stall_X1hl,
  input         stall_X2hl,
  input         stall_X3hl,
  input         stall_Whl,

  input  [ 4:0] ROB_commit_slot_A,  // First committed slot in the ROB
  input         ROB_commit_ready_A,   // Ensure ROB slot 1 is valid
  input  [ 4:0] ROB_commit_slot_B,  // Second committed slot in the ROB
  input         ROB_commit_ready_B,   // Ensure ROB slot 2 is valid

  output [ 3:0] op0_A_byp_sel,   // rs0_A bypass signal
  output [ 3:0] op1_A_byp_sel,   // rs1_A bypass signal
  output [ 3:0] op0_B_byp_sel,   // rs0_B bypass signal
  output [ 3:0] op1_B_byp_sel,   // rs1_B bypass signal

  output reg [31:0] src_ready
);


  localparam ALU        = 2'b00;
  localparam LS         = 2'b01;
  localparam MULDIV     = 2'b10;
  localparam FINISHED   = 1'b0;
  localparam PENDING    = 1'b1;

  // Use 32 bit represent 32 registers
  reg [31:0] status;
  // status bit , 0 means finished , 1 means pending
  
  reg [31:0] wide_index;
  // represent which pipeline wide this register used
  
  reg [ 4:0] instr_pos  [31:0];
  // exec step , X0~X3 in functional unit + WB stage

  reg [ 1:0] functional_unit_used    [31:0];
  // record which functional unit this register in



  wire [4:0] stalls     = { stall_X0hl,
                            stall_X1hl,
                            stall_X2hl,
                            stall_X3hl,
                            stall_Whl };

  wire [4:0] stalls_alu = { 3'b0, 
                            stall_X0hl, 
                            stall_Whl };

  integer i;

  always @ ( posedge clk ) begin
    if ( reset ) begin
    
      status      <= 32'b0;
      wide_index  <= 32'b0;

      for( i = 0; i < 32; i = i + 1 ) begin
        instr_pos[i]            = 5'b0;
        functional_unit_used[i]   = 2'b0;
      end
    
    end 
    else begin
      //-------------//
      // Issue Logic //
      //-------------//

      // If an instruction is issued, update the scoreboard if the instruction
      // writes to the register file. 
      // Index in the scoreboard use rd
      // Pipeline 0 is A, pipeline 1 is B

      for ( i = 0; i < 32; i = i + 1 ) begin
      
        if ( A_issued && rd_A_en && ( i == rd_A ) ) begin
          
          instr_pos[i]            <= 5'b10000;
          wide_index[i]             <= 1'b0;
          status[i]                 <= PENDING;
          functional_unit_used[i]   <= func_irA;
        
        end
        else if ( B_issued && rd_B_en && ( i == rd_B ) ) begin
          // Pipeline B handles ALU
          instr_pos[i]            <= 5'b00010;
          wide_index[i]             <= 1'b1;
          status[i]                 <= PENDING;
          functional_unit_used[i]   <= ALU;
        
        end

        //-------------------------//
        // Shift and Pending Logic //
        //-------------------------//

        // reset to FINISHED if commit to ROB
        else begin
        
          if( wide_index[i] == 1'b0 ) begin // in pipeline A
            instr_pos[i] <= (   instr_pos[i] & stalls ) | // stall bits don't move
                            ( ( instr_pos[i] & ~stalls ) >> 1 ); // only shift no tstaled bits
          end
          else begin // in pipeline B
            instr_pos[i] <= (   instr_pos[i] & stalls_alu ) |
                            ( ( instr_pos[i] & ~stalls_alu ) >> 1 );
          end

          if ( ( ROB_commit_ready_A && ( ROB_commit_slot_A == i ) ) ||
               ( ROB_commit_ready_B && ( ROB_commit_slot_B == i ) ) ) begin
            status[i] <= FINISHED;
          end
        
        end
      end
    end
  end

  //-----------------//
  // Bypassing Logic //
  //-----------------//

  // Bypass mux selects
  localparam byp_r0   = 4'd0;  // Use rdata0
  localparam byp_X0_A = 4'd1;  // Bypass from X0_A
  localparam byp_X1_A = 4'd2;  // Bypass from X1_A
  localparam byp_X2_A = 4'd3;  // Bypass from X2_A
  localparam byp_X3_A = 4'd4;  // Bypass from X3_A
  localparam byp_W_A  = 4'd5;  // Bypass from W_A


  localparam byp_X0_B = 4'd6;  // Bypass from X0_B
  localparam byp_X1_B = 4'd7;  // Bypass from X1_B
  localparam byp_X2_B = 4'd8;  // Bypass from X2_B
  localparam byp_X3_B = 4'd9;  // Bypass from X3_B
  localparam byp_W_B  = 4'd10; // Bypass from W_B
  localparam byp_ROB  = 4'd11; // Bypass from ROB

  wire rs0_A_byp_A_Dhl = ~wide_index[rs0_A] && rs0_A_renamed && ( status[rs0_A] == PENDING );
  wire rs0_A_byp_B_Dhl =  wide_index[rs0_A] && rs0_A_renamed && ( status[rs0_A] == PENDING );

  wire rs1_A_byp_A_Dhl = ~wide_index[rs1_A] && rs1_A_renamed && ( status[rs1_A] == PENDING );
  wire rs1_A_byp_B_Dhl =  wide_index[rs1_A] && rs1_A_renamed && ( status[rs1_A] == PENDING );

  wire rs0_B_byp_A_Dhl = ~wide_index[rs0_B] && rs0_B_renamed && ( status[rs0_B] == PENDING );
  wire rs0_B_byp_B_Dhl =  wide_index[rs0_B] && rs0_B_renamed && ( status[rs0_B] == PENDING );

  wire rs1_B_byp_A_Dhl = ~wide_index[rs1_B] && rs1_B_renamed && ( status[rs1_B] == PENDING );
  wire rs1_B_byp_B_Dhl =  wide_index[rs1_B] && rs1_B_renamed && ( status[rs1_B] == PENDING );


  // Operand Bypass Mux Select
  wire [3:0] op0_A_byp_sel  = ( rs0_A_byp_A_Dhl             && ( instr_pos[rs0_A] == 5'b10000 ) ) ? byp_X0_A
                            : ( rs0_A_byp_A_Dhl             && ( instr_pos[rs0_A] == 5'b01000 ) ) ? byp_X1_A
                            : ( rs0_A_byp_A_Dhl             && ( instr_pos[rs0_A] == 5'b00100 ) ) ? byp_X2_A
                            : ( rs0_A_byp_A_Dhl             && ( instr_pos[rs0_A] == 5'b00010 ) ) ? byp_X3_A
                            : ( rs0_A_byp_A_Dhl             && ( instr_pos[rs0_A] == 5'b00001 ) ) ? byp_W_A
                            : ( rs0_A_byp_B_Dhl             && ( instr_pos[rs0_A] == 5'b00010 ) ) ? byp_X0_B
                            : ( rs0_A_byp_B_Dhl             && ( instr_pos[rs0_A] == 5'b00001 ) ) ? byp_W_B
                            : ( status[rs0_A] && rs0_A_renamed && ( instr_pos[rs0_A] == 5'd0  ) ) ? byp_ROB
                            :   byp_r0;  

  wire [3:0] op1_A_byp_sel  = ( rs1_A_byp_A_Dhl             && ( instr_pos[rs1_A] == 5'b10000 ) ) ? byp_X0_A
                            : ( rs1_A_byp_A_Dhl             && ( instr_pos[rs1_A] == 5'b01000 ) ) ? byp_X1_A
                            : ( rs1_A_byp_A_Dhl             && ( instr_pos[rs1_A] == 5'b00100 ) ) ? byp_X2_A
                            : ( rs1_A_byp_A_Dhl             && ( instr_pos[rs1_A] == 5'b00010 ) ) ? byp_X3_A
                            : ( rs1_A_byp_A_Dhl             && ( instr_pos[rs1_A] == 5'b00001 ) ) ? byp_W_A
                            : ( rs1_A_byp_B_Dhl             && ( instr_pos[rs1_A] == 5'b00010 ) ) ? byp_X0_B
                            : ( rs1_A_byp_B_Dhl             && ( instr_pos[rs1_A] == 5'b00001 ) ) ? byp_W_B
                            : ( status[rs1_A] && rs1_A_renamed && ( instr_pos[rs1_A] == 5'd0  ) ) ? byp_ROB
                            :   byp_r0;

  wire [3:0] op0_B_byp_sel  = ( rs0_B_byp_A_Dhl             && ( instr_pos[rs0_B] == 5'b10000 ) ) ? byp_X0_A
                            : ( rs0_B_byp_A_Dhl             && ( instr_pos[rs0_B] == 5'b01000 ) ) ? byp_X1_A
                            : ( rs0_B_byp_A_Dhl             && ( instr_pos[rs0_B] == 5'b00100 ) ) ? byp_X2_A
                            : ( rs0_B_byp_A_Dhl             && ( instr_pos[rs0_B] == 5'b00010 ) ) ? byp_X3_A
                            : ( rs0_B_byp_A_Dhl             && ( instr_pos[rs0_B] == 5'b00001 ) ) ? byp_W_A
                            : ( rs0_B_byp_B_Dhl             && ( instr_pos[rs0_B] == 5'b00010 ) ) ? byp_X0_B
                            : ( rs0_B_byp_B_Dhl             && ( instr_pos[rs0_B] == 5'b00001 ) ) ? byp_W_B
                            : ( status[rs0_B] && rs0_B_renamed && ( instr_pos[rs0_B] == 5'd0  ) ) ? byp_ROB
                            :   byp_r0;

  wire [3:0] op1_B_byp_sel  = ( rs1_B_byp_A_Dhl             && ( instr_pos[rs1_B] == 5'b10000 ) ) ? byp_X0_A
                            : ( rs1_B_byp_A_Dhl             && ( instr_pos[rs1_B] == 5'b01000 ) ) ? byp_X1_A
                            : ( rs1_B_byp_A_Dhl             && ( instr_pos[rs1_B] == 5'b00100 ) ) ? byp_X2_A
                            : ( rs1_B_byp_A_Dhl             && ( instr_pos[rs1_B] == 5'b00010 ) ) ? byp_X3_A
                            : ( rs1_B_byp_A_Dhl             && ( instr_pos[rs1_B] == 5'b00001 ) ) ? byp_W_A
                            : ( rs1_B_byp_B_Dhl             && ( instr_pos[rs1_B] == 5'b00010 ) ) ? byp_X0_B
                            : ( rs1_B_byp_B_Dhl             && ( instr_pos[rs1_B] == 5'b00001 ) ) ? byp_W_B
                            : ( status[rs1_B] && rs1_B_renamed && ( instr_pos[rs1_B] == 5'd0  ) ) ? byp_ROB
                            :   byp_r0;

  //----------------------------------------------------------------------
  // Source Ready Logic
  //----------------------------------------------------------------------

  always @(*) begin
    for( i = 0; i < 32; i = i + 1 ) begin
      src_ready[i] = ( status[i] == FINISHED )/*finished*/    ? 1'b1
                   : ( functional_unit_used[i] == ALU )       ? 1'b1
                   : ( functional_unit_used[i] == LS )        ? ( instr_pos[i] < 5'b10000 ) /*first cycle has taken data*/
                   : ( functional_unit_used[i] == MULDIV )    ? ( instr_pos[i] < 5'b00100 ) /* so in later stages,  src can be used again*/
                   : ( instr_pos[i] == 5'b00000 )             ? 1'b1
                   : 1'b0;
    end
  end

endmodule

`endif

