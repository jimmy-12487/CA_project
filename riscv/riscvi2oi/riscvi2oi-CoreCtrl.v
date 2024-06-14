//=========================================================================
// 7-Stage RISCV Control Unit
//=========================================================================

`ifndef RISCV_CORE_CTRL_V
`define RISCV_CORE_CTRL_V

`include "riscvi2oi-CoreScoreboard.v"
`include "riscvi2oi-CoreReorderBuffer.v"
`include "riscvi2oi-InstMsg.v"


module riscv_CoreCtrl
(
  input clk,
  input reset,

  // Instruction Memory Port
  output        imemreq0_val,
  input         imemreq0_rdy,
  input  [31:0] imemresp0_msg_data,
  input         imemresp0_val,

  // Instruction Memory Port
  output        imemreq1_val,
  input         imemreq1_rdy,
  input  [31:0] imemresp1_msg_data,
  input         imemresp1_val,

  // Data Memory Port

  output        dmemreq_msg_rw,
  output  [1:0] dmemreq_msg_len,
  output        dmemreq_val,
  input         dmemreq_rdy,
  input         dmemresp_val,

  // Controls Signals (ctrl->dpath)

  output  [1:0] pc_mux_sel_Phl,
  output        steering_mux_sel_Ihl,
  output  [3:0] opA0_byp_mux_sel_Ihl,
  output  [1:0] opA0_mux_sel_Ihl,
  output  [3:0] opA1_byp_mux_sel_Ihl,
  output  [2:0] opA1_mux_sel_Ihl,
  output  [3:0] opB0_byp_mux_sel_Ihl,
  output  [1:0] opB0_mux_sel_Ihl,
  output  [3:0] opB1_byp_mux_sel_Ihl,
  output  [2:0] opB1_mux_sel_Ihl,
  output [31:0] instA_Ihl,
  output [31:0] instB_Ihl,
  output  [3:0] aluA_fn_X0hl,
  output  [3:0] aluB_fn_X0hl,
  output  [2:0] muldivreq_msg_fn_Ihl,
  output        muldivreq_val,
  input         muldivreq_rdy,
  input         muldivresp_val,
  output        muldivresp_rdy,
  output  [2:0] dmemresp_mux_sel_X1hl,
  output        dmemresp_queue_en_X1hl,
  output        dmemresp_queue_val_X1hl,
  output        muldiv_mux_sel_X3hl,
  output        execute_mux_sel_X3hl,
  output        memex_mux_sel_X1hl,
  output        stall_Fhl,
  output        stall_Dhl,
  output        stall_Ihl,
  output        stall_X0hl,
  output        stall_X1hl,
  output        stall_X2hl,
  output        stall_X3hl,
  output        stall_Whl,

  // Control Signals (dpath->ctrl)

  input         branch_cond_eq_X0hl,
  input         branch_cond_ne_X0hl,
  input         branch_cond_lt_X0hl,
  input         branch_cond_ltu_X0hl,
  input         branch_cond_ge_X0hl,
  input         branch_cond_geu_X0hl,
  input  [31:0] proc2csr_data_Whl,

  // Reorder Buffer Signals (ctrl->dpath)

  output [ 4:0] opA0_byp_ROB_slot_Ihl,
  output [ 4:0] opA1_byp_ROB_slot_Ihl,
  output [ 4:0] opB0_byp_ROB_slot_Ihl,
  output [ 4:0] opB1_byp_ROB_slot_Ihl,

  output        ROB_fill_wen_A_Whl,
  output [ 4:0] ROB_fill_slot_A_Whl,
  output        ROB_fill_wen_B_Whl,
  output [ 4:0] ROB_fill_slot_B_Whl,

  output        ROB_commit_wen_A_Chl,
  output [ 4:0] ROB_commit_slot_A_Chl,
  output [ 4:0] ROB_commit_waddr_1_Chl,
  output        ROB_commit_wen_B_Chl,
  output [ 4:0] ROB_commit_slot_B_Chl,
  output [ 4:0] ROB_commit_waddr_B_Chl,

  // CSR Status

  output [31:0] csr_status
);

  //----------------------------------------------------------------------
  // PC Stage: Instruction Memory Request
  //----------------------------------------------------------------------

  // PC Mux Select
  assign pc_mux_sel_Phl = brj_taken_X0hl ? pm_b
                        : brj_taken_Ihl  ? pc_mux_sel_Ihl
                        : pm_p;

  // Only send a valid imem request if not stalled
  wire   imemreq_val_Phl = reset || !stall_Phl;
  assign imemreq0_val    = imemreq_val_Phl;
  assign imemreq1_val    = imemreq_val_Phl;

  // Dummy Squash Signal
  wire squash_Phl = 1'b0;

  // Stall in PC if F is stalled
  wire stall_Phl = stall_Fhl;

  // Next bubble bit
  wire bubble_next_Phl = ( squash_Phl || stall_Phl );

  //----------------------------------------------------------------------
  // F <- P
  //----------------------------------------------------------------------

  reg imemreq_val_Fhl;
  reg bubble_Fhl;

  always @ ( posedge clk ) begin
    // Only pipeline the bubble bit if the next stage is not stalled
    if ( reset ) begin
      imemreq_val_Fhl <= 1'b0;
      bubble_Fhl <= 1'b0;
    end
    else if( !stall_Fhl ) begin 
      imemreq_val_Fhl <= imemreq_val_Phl;
      bubble_Fhl <= bubble_next_Phl;
    end
    else begin 
      imemreq_val_Fhl <= imemreq_val_Phl;
    end
  end

  //----------------------------------------------------------------------
  // Fetch Stage: Instruction Memory Response
  //----------------------------------------------------------------------

  // Is the current stage valid?
  wire inst_val_Fhl = ( !bubble_Fhl && !squash_Fhl );

  // Squash instruction in F stage if branch taken for a valid
  // instruction or if there was an exception in X stage
  wire squash_Fhl = ( inst_val_Ihl && brj_taken_Ihl ) || ( inst_val_X0hl && brj_taken_X0hl );

  // Stall in F if D is stalled
  assign stall_Fhl = stall_Dhl;

  // Next bubble bit 
  wire bubble_sel_Fhl  = ( squash_Fhl || stall_Fhl );
  wire bubble_next_Fhl = ( !bubble_sel_Fhl ) ? bubble_Fhl
                       : ( bubble_sel_Fhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // Queue for instruction memory response
  //----------------------------------------------------------------------

  wire imemresp0_queue_en_Fhl = ( stall_Dhl && imemresp0_val );
  wire imemresp0_queue_val_next_Fhl
    = stall_Dhl && ( imemresp0_val || imemresp0_queue_val_Fhl );

  wire imemresp1_queue_en_Fhl = ( stall_Dhl && imemresp1_val );
  wire imemresp1_queue_val_next_Fhl
    = stall_Dhl && ( imemresp1_val || imemresp1_queue_val_Fhl );

  reg [31:0] imemresp0_queue_reg_Fhl;
  reg        imemresp0_queue_val_Fhl;

  reg [31:0] imemresp1_queue_reg_Fhl;
  reg        imemresp1_queue_val_Fhl;

  always @ ( posedge clk ) begin
    if ( imemresp0_queue_en_Fhl ) begin
      imemresp0_queue_reg_Fhl <= imemresp0_msg_data;
    end
    if ( imemresp1_queue_en_Fhl ) begin
      imemresp1_queue_reg_Fhl <= imemresp1_msg_data;
    end
    imemresp0_queue_val_Fhl <= imemresp0_queue_val_next_Fhl;
    imemresp1_queue_val_Fhl <= imemresp1_queue_val_next_Fhl;
  end

  //----------------------------------------------------------------------
  // Instruction memory queue mux
  //----------------------------------------------------------------------

  wire [31:0] imemresp0_queue_mux_out_Fhl
    = ( !imemresp0_queue_val_Fhl ) ? imemresp0_msg_data
    : ( imemresp0_queue_val_Fhl )  ? imemresp0_queue_reg_Fhl
    :                               32'bx;

  wire [31:0] imemresp1_queue_mux_out_Fhl
    = ( !imemresp1_queue_val_Fhl ) ? imemresp1_msg_data
    : ( imemresp1_queue_val_Fhl )  ? imemresp1_queue_reg_Fhl
    :                               32'bx;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] ir0_Dhl;
  reg [31:0] ir1_Dhl;
  reg        bubble_Dhl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Dhl         <= 1'b1;
    end
    else if( !stall_Dhl ) begin
      ir0_Dhl    <= imemresp0_queue_mux_out_Fhl;
      ir1_Dhl    <= imemresp1_queue_mux_out_Fhl;
      bubble_Dhl <= bubble_next_Fhl;
    end 
  end

  //----------------------------------------------------------------------
  // Decode Stage: Constants
  //----------------------------------------------------------------------

  // Generic Parameters

  localparam n = 1'd0;
  localparam y = 1'd1;

  // Register specifiers

  localparam rx = 5'bx;
  localparam r0 = 5'd0;

  // Branch Type

  localparam br_x    = 3'bx;
  localparam br_none = 3'd0;
  localparam br_beq  = 3'd1;
  localparam br_bne  = 3'd2;
  localparam br_blt  = 3'd3;
  localparam br_bltu = 3'd4;
  localparam br_bge  = 3'd5;
  localparam br_bgeu = 3'd6;

  // PC Mux Select

  localparam pm_x   = 2'bx;  // Don't care
  localparam pm_p   = 2'd0;  // Use pc+4
  localparam pm_b   = 2'd1;  // Use branch address
  localparam pm_j   = 2'd2;  // Use jump address
  localparam pm_r   = 2'd3;  // Use jump register

  // Operand Bypass Mux Select  

  localparam byp_r0   = 4'd0;   // Use rdata0
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

  // Operand 0 Mux Select

  localparam am_x     = 2'bx;
  localparam am_rdat  = 2'd0; // Use output of bypass mux for rs1
  localparam am_pc    = 2'd1; // Use current PC
  localparam am_pc4   = 2'd2; // Use PC + 4
  localparam am_0     = 2'd3; // Use constant 0

  // Operand 1 Mux Select

  localparam bm_x      = 3'bx; // Don't care
  localparam bm_rdat   = 3'd0; // Use output of bypass mux for rs2
  localparam bm_shamt  = 3'd1; // Use shift amount
  localparam bm_imm_u  = 3'd2; // Use U-type immediate
  localparam bm_imm_sb = 3'd3; // Use SB-type immediate
  localparam bm_imm_i  = 3'd4; // Use I-type immediate
  localparam bm_imm_s  = 3'd5; // Use S-type immediate
  localparam bm_0      = 3'd6; // Use constant 0

  // ALU Function

  localparam alu_x    = 4'bx;
  localparam alu_add  = 4'd0;
  localparam alu_sub  = 4'd1;
  localparam alu_sll  = 4'd2;
  localparam alu_or   = 4'd3;
  localparam alu_lt   = 4'd4;
  localparam alu_ltu  = 4'd5;
  localparam alu_and  = 4'd6;
  localparam alu_xor  = 4'd7;
  localparam alu_nor  = 4'd8;
  localparam alu_srl  = 4'd9;
  localparam alu_sra  = 4'd10;

  // Muldiv Function

  localparam md_x    = 3'bx;
  localparam md_mul  = 3'd0;
  localparam md_div  = 3'd1;
  localparam md_divu = 3'd2;
  localparam md_rem  = 3'd3;
  localparam md_remu = 3'd4;

  // MulDiv Mux Select

  localparam mdm_x = 1'bx; // Don't Care
  localparam mdm_l = 1'd0; // Take lower half of 64-bit result, mul/div/divu
  localparam mdm_u = 1'd1; // Take upper half of 64-bit result, rem/remu

  // Execute Mux Select

  localparam em_x   = 1'bx; // Don't Care
  localparam em_alu = 1'd0; // Use ALU output
  localparam em_md  = 1'd1; // Use muldiv output

  // Memory Request Type

  localparam nr = 2'b0; // No request
  localparam ld = 2'd1; // Load
  localparam st = 2'd2; // Store

  // Subword Memop Length

  localparam ml_x  = 2'bx;
  localparam ml_w  = 2'd0;
  localparam ml_b  = 2'd1;
  localparam ml_h  = 2'd2;

  // Memory Response Mux Select

  localparam dmm_x  = 3'bx;
  localparam dmm_w  = 3'd0;
  localparam dmm_b  = 3'd1;
  localparam dmm_bu = 3'd2;
  localparam dmm_h  = 3'd3;
  localparam dmm_hu = 3'd4;

  // Writeback Mux 1

  localparam wm_x   = 1'bx; // Don't care
  localparam wm_alu = 1'd0; // Use ALU output
  localparam wm_mem = 1'd1; // Use data memory response

  //----------------------------------------------------------------------
  // Decode Stage: Logic
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Dhl = ( !bubble_Dhl && !squash_Dhl );

  // Parse instruction fields

  wire   [4:0] inst0_rs1_Dhl;
  wire   [4:0] inst0_rs2_Dhl;
  wire   [4:0] inst0_rd_Dhl;

  riscv_InstMsgFromBits inst0_msg_from_bits
  (
    .msg      (ir0_Dhl),
    .opcode   (),
    .rs1      (inst0_rs1_Dhl),
    .rs2      (inst0_rs2_Dhl),
    .rd       (inst0_rd_Dhl),
    .funct3   (),
    .funct7   (),
    .shamt    (),
    .imm_i    (),
    .imm_s    (),
    .imm_sb   (),
    .imm_u    (),
    .imm_uj   ()
  );

  wire   [4:0] inst1_rs1_Dhl;
  wire   [4:0] inst1_rs2_Dhl;
  wire   [4:0] inst1_rd_Dhl;

  riscv_InstMsgFromBits inst1_msg_from_bits
  (
    .msg      (ir1_Dhl),
    .opcode   (),
    .rs1      (inst1_rs1_Dhl),
    .rs2      (inst1_rs2_Dhl),
    .rd       (inst1_rd_Dhl),
    .funct3   (),
    .funct7   (),
    .shamt    (),
    .imm_i    (),
    .imm_s    (),
    .imm_sb   (),
    .imm_u    (),
    .imm_uj   ()
  );

  // Shorten register specifier name for table


  wire [4:0] rd0 = inst0_rd_Dhl;
  wire [4:0] rd1 = inst1_rd_Dhl;

  // Instruction Decode

  localparam cs_sz =  `RISCV_INST_MSG_CS_SZ;
  reg [cs_sz-1:0] cs0;
  reg [cs_sz-1:0] cs1;

  always @ (*) begin

    cs0 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir0_Dhl )

      //                                j     br       pc      op0      rs1 op1       rs2 alu       md       md md     ex      mem  mem   memresp wb      rf      csr
      //                            val taken type     muxsel  muxsel   en  muxsel    en  fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `RISCV_INST_MSG_LUI     :cs0={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AUIPC   :cs0={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_ADDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ORI     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      
      `RISCV_INST_MSG_ADD     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_LW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_SW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  r0, n   };

      `RISCV_INST_MSG_JAL     :cs0={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
     
      `RISCV_INST_MSG_BNE     :cs0={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BLT     :cs0={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };

      `RISCV_INST_MSG_CSRW    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  r0, y   };
    
      `RISCV_INST_MSG_JALR    :cs0={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_BEQ     :cs0={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BGE     :cs0={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BLTU    :cs0={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BGEU    :cs0={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };

      `RISCV_INST_MSG_LB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LBU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd0, n   };
      `RISCV_INST_MSG_LHU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd0, n   };


      `RISCV_INST_MSG_SB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  r0, n   };
      `RISCV_INST_MSG_SH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  r0, n   };


      `RISCV_INST_MSG_SLTI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTIU   :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };


      `RISCV_INST_MSG_XORI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_ANDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `RISCV_INST_MSG_SLLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRLI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRAI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };


      `RISCV_INST_MSG_SUB     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLT     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SLTU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      
      `RISCV_INST_MSG_XOR     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_SRA     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_OR      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_AND     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      
      `RISCV_INST_MSG_MUL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIV     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REM     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_DIVU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `RISCV_INST_MSG_REMU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };


    endcase

  end

  always @ (*) begin

    cs1 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir1_Dhl )

      //                                j     br       pc      op0      rs1 op1       rs2 alu       md       md md     ex      mem  mem   memresp wb      rf      csr
      //                            val taken type     muxsel  muxsel   en  muxsel    en  fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `RISCV_INST_MSG_LUI     :cs1={ y,  n,    br_none, pm_p,   am_0,    n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AUIPC   :cs1={ y,  n,    br_none, pm_p,   am_pc,   n,  bm_imm_u, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_ADDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ORI     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      
      `RISCV_INST_MSG_ADD     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_LW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_SW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  r0, n   };
    
      `RISCV_INST_MSG_JAL     :cs1={ y,  y,    br_none, pm_j,   am_pc4,  n,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_BNE     :cs1={ y,  n,    br_bne,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BLT     :cs1={ y,  n,    br_blt,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };

      `RISCV_INST_MSG_CSRW    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_0,     y,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  r0, y   };

      `RISCV_INST_MSG_JALR    :cs1={ y,  y,    br_none, pm_r,   am_pc4,  y,  bm_0,     n,  alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_BEQ     :cs1={ y,  n,    br_beq,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BGE     :cs1={ y,  n,    br_bge,  pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BLTU    :cs1={ y,  n,    br_bltu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };
      `RISCV_INST_MSG_BGEU    :cs1={ y,  n,    br_bgeu, pm_b,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  r0, n   };

      `RISCV_INST_MSG_LB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LBU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rd1, n   };
      `RISCV_INST_MSG_LHU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rd1, n   };

      `RISCV_INST_MSG_SB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_b,  wm_mem, n,  r0, n   };
      `RISCV_INST_MSG_SH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_s, y,  alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_h,  wm_mem, n,  r0, n   };

      `RISCV_INST_MSG_SLTI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTIU   :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
 
      `RISCV_INST_MSG_SUB     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLT     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLTU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      
      `RISCV_INST_MSG_XORI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_ANDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SLLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRLI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRAI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_imm_i, n,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
   
      `RISCV_INST_MSG_XOR     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_SRA     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_OR      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_AND     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `RISCV_INST_MSG_MUL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIV     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REM     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_DIVU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `RISCV_INST_MSG_REMU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y,  bm_rdat,  y,  alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };


    endcase

  end

  //----------------------------------------------------------------------
  // Instruction Decode
  //----------------------------------------------------------------------

  // Jump and Branch Controls
 
  wire j0_en_Dhl = cs0[`RISCV_INST_MSG_J_EN];
  wire j1_en_Dhl = cs1[`RISCV_INST_MSG_J_EN];

// get branch type
  wire [2:0] br0_sel_Dhl    = cs0[`RISCV_INST_MSG_BR_SEL];
  wire [2:0] br1_sel_Dhl    = cs1[`RISCV_INST_MSG_BR_SEL];

  // PC Mux Select

  wire [1:0] pc0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_PC_SEL];
  wire [1:0] pc1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_PC_SEL];


  // Operand RF Read Addresses and Enables (using rs or rt?)

  wire [4:0] rs0_i0_addr_Dhl  = inst0_rs1_Dhl;
  wire [4:0] rs1_i0_addr_Dhl  = inst0_rs2_Dhl;

  wire [4:0] rs0_i1_addr_Dhl  = inst1_rs1_Dhl;
  wire [4:0] rs1_i1_addr_Dhl  = inst1_rs2_Dhl;

  wire       rs0_i0_en_Dhl    = cs0[`RISCV_INST_MSG_RS1_EN];
  wire       rs1_i0_en_Dhl    = cs0[`RISCV_INST_MSG_RS2_EN];

  wire       rs0_i1_en_Dhl    = cs1[`RISCV_INST_MSG_RS1_EN];
  wire       rs1_i1_en_Dhl    = cs1[`RISCV_INST_MSG_RS2_EN];  

  // Operand Mux Select

  wire [1:0] op0_i0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_OP0_SEL];
  wire [2:0] op1_i0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_OP1_SEL];

  wire [1:0] op0_i1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_OP0_SEL];
  wire [2:0] op1_i1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_OP1_SEL];

  // ALU Function

  wire [3:0] alu0_fn_Dhl = cs0[`RISCV_INST_MSG_ALU_FN];
  wire [3:0] alu1_fn_Dhl = cs1[`RISCV_INST_MSG_ALU_FN];

  // Muldiv Function

  wire [2:0] muldivreq0_msg_fn_Dhl = cs0[`RISCV_INST_MSG_MULDIV_FN];
  wire [2:0] muldivreq1_msg_fn_Dhl = cs1[`RISCV_INST_MSG_MULDIV_FN];

  // Muldiv Controls

  wire muldivreq0_val_Dhl = cs0[`RISCV_INST_MSG_MULDIV_EN];
  wire muldivreq1_val_Dhl = cs1[`RISCV_INST_MSG_MULDIV_EN];

  // Muldiv Mux Select

  wire muldiv0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_MULDIV_SEL];
  wire muldiv1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_MULDIV_SEL];

  // Execute Mux Select

  wire execute0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_MULDIV_EN];
  wire execute1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_MULDIV_EN];

  // Memory Controls

  wire       dmemreq0_msg_rw_Dhl  = ( cs0[`RISCV_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq0_msg_len_Dhl = cs0[`RISCV_INST_MSG_MEM_LEN];
  wire       dmemreq0_val_Dhl     = ( cs0[`RISCV_INST_MSG_MEM_REQ] != nr );

  wire       dmemreq1_msg_rw_Dhl  = ( cs1[`RISCV_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq1_msg_len_Dhl = cs1[`RISCV_INST_MSG_MEM_LEN];
  wire       dmemreq1_val_Dhl     = ( cs1[`RISCV_INST_MSG_MEM_REQ] != nr );

  // Memory response mux select

  wire [2:0] dmemresp0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_MEM_SEL];
  wire [2:0] dmemresp1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_MEM_SEL];

  // Writeback Mux Select

  wire memex0_mux_sel_Dhl = cs0[`RISCV_INST_MSG_WB_SEL];
  wire memex1_mux_sel_Dhl = cs1[`RISCV_INST_MSG_WB_SEL];

  // Register Writeback Controls

  wire rf0_wen_Dhl         = cs0[`RISCV_INST_MSG_RF_WEN];
  wire [4:0] rf0_waddr_Dhl = cs0[`RISCV_INST_MSG_RF_WADDR];

  wire rf1_wen_Dhl         = cs1[`RISCV_INST_MSG_RF_WEN];
  wire [4:0] rf1_waddr_Dhl = cs1[`RISCV_INST_MSG_RF_WADDR];

  // CSR register write enable

  wire csr0_wen_Dhl = cs0[`RISCV_INST_MSG_CSR_WEN];
  wire csr1_wen_Dhl = cs1[`RISCV_INST_MSG_CSR_WEN];

  // CSR register address

  wire [11:0] csr0_addr_Dhl  = ir0_Dhl[31:20];
  wire [11:0] csr1_addr_Dhl  = ir1_Dhl[31:20];

  //----------------------------------------------------------------------
  // Decode Stage: Reorder Buffer Logic
  //----------------------------------------------------------------------

  wire ROB_req_rdy_Dhl;

  wire rs0_i0_renamed;
  wire rs1_i0_renamed;
  wire rs0_i1_renamed;
  wire rs1_i1_renamed;

  wire [4:0] rs0_i0_slot;
  wire [4:0] rs1_i0_slot;
  wire [4:0] rs0_i1_slot;
  wire [4:0] rs1_i1_slot;

  wire [4:0] ROB_resp_slot_1;
  wire [4:0] ROB_resp_slot_2;

  wire ROB_commit_ready_A;
  wire ROB_commit_ready_B;

  wire raw_hazardA_Dhl = (    inst_val_Dhl && rf0_wen_Dhl )
                       && ( ( rf0_waddr_Dhl == rs0_i1_addr_Dhl ) && rs0_i1_en_Dhl );

  wire raw_hazardB_Dhl = (    inst_val_Dhl && rf0_wen_Dhl )
                       && ( ( rf0_waddr_Dhl == rs1_i1_addr_Dhl ) && rs1_i1_en_Dhl );

  // speculative instruction
  reg spec_Dhl;
  always @(posedge clk) begin
    if( reset ) begin
      spec_Dhl <= 1'b0;
    end
    else begin
      // if decode instr is branch and not stall then set speculative
      if(   inst_val_Dhl &&
        ( ( br0_sel_Dhl && !squash_ir0_Dhl && !stall_agg_Dhl ) ||
          ( br1_sel_Dhl && !stall_Dhl ) ) ) begin
        spec_Dhl <= 1'b1;
      end
    end
  end
  always @(*) begin
    // set speculative to 0 when branch resolved
    if( inst_val_X0hl && brj_resolved_X0hl ) begin
      spec_Dhl <= 1'b0;
    end
  end






  // set double branch signal
  reg squash_ir0_Dhl;
  wire double_br_Dhl =  inst_val_Dhl    && 
                        br0_sel_Dhl     && 
                        br1_sel_Dhl     &&
                        !squash_ir0_Dhl && 
                        !stall_agg_Dhl;

  // branch0 has higher priority
  wire ROB_req_val_1_Dhl =  inst_val_Dhl    && 
                            !stall_agg_Dhl  && 
                            !squash_ir0_Dhl;

  wire ROB_req_val_2_Dhl =  inst_val_Dhl    && 
                            !stall_Dhl      && 
                            !j0_en_Dhl      && 
                            !double_br_Dhl;

  wire ROB_req_spec_1_Dhl = spec_Dhl;
  wire ROB_req_spec_2_Dhl = spec_Dhl || (br0_sel_Dhl != br_none && !squash_ir0_Dhl);

  reg rs0_i0_renamed_Dhl;
  reg rs1_i0_renamed_Dhl;
  reg rs0_i1_renamed_Dhl;
  reg rs1_i1_renamed_Dhl;

  reg [4:0] rs0_i0_rt_slot_Dhl;
  reg [4:0] rs1_i0_rt_slot_Dhl;
  reg [4:0] rs0_i1_rt_slot_Dhl;
  reg [4:0] rs1_i1_rt_slot_Dhl;

  reg [4:0] ROB_fill_slot_0_Dhl;
  reg [4:0] ROB_commit_req_slot_A_Dhl;

  always @(*) begin
    rs0_i0_renamed_Dhl = rs0_i0_renamed;
    rs1_i0_renamed_Dhl = rs1_i0_renamed;
    rs0_i1_renamed_Dhl = rs0_i1_renamed;
    rs1_i1_renamed_Dhl = rs1_i1_renamed;

    rs0_i0_rt_slot_Dhl = rs0_i0_slot;
    rs1_i0_rt_slot_Dhl = rs1_i0_slot;
    rs0_i1_rt_slot_Dhl = rs0_i1_slot;
    rs1_i1_rt_slot_Dhl = rs1_i1_slot;

    ROB_fill_slot_0_Dhl = ROB_resp_slot_1;
    ROB_commit_req_slot_A_Dhl = ROB_resp_slot_2;
  end

  riscv_CoreReorderBuffer ROB
  (
    .clk                         (clk),
    .reset                       (reset),

    .brj_taken_X0hl              (brj_taken_X0hl),
    .brj_resolved_X0hl           (brj_resolved_X0hl),

    .ROB_alloc_valid           (ROB_req_rdy_Dhl),

    .ROB_req_A         (ROB_req_val_1_Dhl),
    .ROB_req_we_A         (rf0_wen_Dhl),
    .ROB_req_spec_A        (ROB_req_spec_1_Dhl),
    .ROB_req_preg_A        (rf0_waddr_Dhl),
    .ROB_commit_req_A              (ROB_fill_val_A_Whl),
    .ROB_commit_req_slot_A             (ROB_fill_slot_A_Whl),
    .ROB_commit_slot_A           (ROB_commit_slot_A_Chl),
    .ROB_commit_wen_A            (ROB_commit_wen_A_Chl),
    .ROB_commit_rdaddr_A       (ROB_commit_waddr_1_Chl),
    .ROB_commit_ready_A            (ROB_commit_ready_A),
    .ROB_commit_spec_A           (ROB_commit_spec_A),

    .ROB_req_B         (ROB_req_val_2_Dhl),
    .ROB_req_we_B         (rf1_wen_Dhl),
    .ROB_req_spec_B        (ROB_req_spec_2_Dhl),
    .ROB_req_preg_B        (rf1_waddr_Dhl),
    .ROB_commit_req_B              (ROB_fill_val_B_Whl),
    .ROB_commit_req_slot_B             (ROB_fill_slot_B_Whl),
    .ROB_commit_slot_B           (ROB_commit_slot_B_Chl),
    .ROB_commit_wen_B            (ROB_commit_wen_B_Chl),
    .ROB_commit_rdaddr_B       (ROB_commit_waddr_B_Chl),
    .ROB_commit_ready_B            (ROB_commit_ready_B),
    .ROB_commit_spec_B           (ROB_commit_spec_B),

    .ROB_alloc_slot_A       (ROB_resp_slot_1),
    .ROB_alloc_slot_B       (ROB_resp_slot_2),

    .raw_hazardA                 (raw_hazardA_Dhl),
    .raw_hazardB                 (raw_hazardB_Dhl),

    .rs0_i0_addr                       (rs0_i0_addr_Dhl),
    .rs1_i0_addr                       (rs1_i0_addr_Dhl),
    .rs0_i1_addr                       (rs0_i1_addr_Dhl),
    .rs1_i1_addr                       (rs1_i1_addr_Dhl),

    .rs0_i0_renamed               (rs0_i0_renamed),
    .rs1_i0_renamed               (rs1_i0_renamed),
    .rs0_i1_renamed               (rs0_i1_renamed),
    .rs1_i1_renamed               (rs1_i1_renamed),

    .rs0_i0_slot                  (rs0_i0_slot),
    .rs1_i0_slot                  (rs1_i0_slot),
    .rs0_i1_slot                  (rs0_i1_slot),
    .rs1_i1_slot                  (rs1_i1_slot)
  );

  //---------------------------//
  // Decode Stage: Stall Logic //
  //---------------------------//

  // squash instruction if br/j taken in I or X0
  wire squash_Dhl = ( inst_val_Ihl && brj_taken_Ihl ) || ( inst_val_X0hl && brj_taken_X0hl );

  // Stall in D if the ROB is full, if a second branch is seen before the first
  // resolved, or if I is stalled

  // stall ROB if full
  wire stall_ROB_Dhl = !ROB_req_rdy_Dhl;
  
  wire stall_br_Dhl = spec_Dhl && ( br0_sel_Dhl || br1_sel_Dhl );

  // Aggregate Stall Signal

  wire stall_agg_Dhl =  ( stall_Ihl || 
                        ( ( stall_ROB_Dhl || stall_br_Dhl ) && inst_val_Dhl ) 
                        );

  wire stall_Dhl = stall_agg_Dhl || double_br_Dhl;

  // Next bubble bit
  // Send a bubble bit if no ROB entries are allocated
  wire bubble_sel_Dhl  = ( !ROB_req_val_1_Dhl && !ROB_req_val_2_Dhl );
  wire bubble_next_Dhl = ( !bubble_sel_Dhl ) ? bubble_Dhl
                       : ( bubble_sel_Dhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // I <- D
  //----------------------------------------------------------------------

  reg         bubble_Ihl;

  reg [31:0]  ir0_Ihl;
  reg         ir0_squashed_Ihl;
  reg         rf0_wen_Ihl;
  reg  [4:0]  rf0_waddr_Ihl;
  reg  [3:0]  alu0_fn_Ihl;
  reg  [2:0]  br0_sel_Ihl;
  reg         j0_en_Ihl;
  reg  [1:0]  pc0_mux_sel_Ihl;
  reg         muldivreq0_val_Ihl;
  reg  [2:0]  muldivreq0_msg_fn_Ihl;
  reg         muldiv0_mux_sel_Ihl;
  reg         execute0_mux_sel_Ihl;
  reg         dmemreq0_msg_rw_Ihl;
  reg  [1:0]  dmemreq0_msg_len_Ihl;
  reg         dmemreq0_val_Ihl;
  reg  [2:0]  dmemresp0_mux_sel_Ihl;
  reg         memex0_mux_sel_Ihl;
  reg         csr0_wen_Ihl;
  reg [11:0]  csr0_addr_Ihl;

  reg [31:0]  ir1_Ihl;
  reg         ir1_squashed_Ihl;
  reg         rf1_wen_Ihl;
  reg  [4:0]  rf1_waddr_Ihl;
  reg  [3:0]  alu1_fn_Ihl;
  reg  [2:0]  br1_sel_Ihl;
  reg         j1_en_Ihl;
  reg  [1:0]  pc1_mux_sel_Ihl;
  reg         muldivreq1_val_Ihl;
  reg  [2:0]  muldivreq1_msg_fn_Ihl;
  reg         muldiv1_mux_sel_Ihl;
  reg         execute1_mux_sel_Ihl;
  reg         dmemreq1_msg_rw_Ihl;
  reg  [1:0]  dmemreq1_msg_len_Ihl;
  reg         dmemreq1_val_Ihl;
  reg  [2:0]  dmemresp1_mux_sel_Ihl;
  reg         memex1_mux_sel_Ihl;
  reg         csr1_wen_Ihl;
  reg [11:0]  csr1_addr_Ihl;


  reg  [4:0]  rs0_i0_addr_Ihl;
  reg  [4:0]  rs1_i0_addr_Ihl;
  reg  [4:0]  rs0_i1_addr_Ihl;
  reg  [4:0]  rs1_i1_addr_Ihl;

  reg         rs0_i0_en_Ihl;
  reg         rs1_i0_en_Ihl;
  reg         rs0_i1_en_Ihl;
  reg         rs1_i1_en_Ihl;

  reg  [4:0]  ROB_fill_slot_0_Ihl;
  reg  [4:0]  ROB_commit_req_slot_A_Ihl;
  reg         ROB_fill_val_0_Ihl;
  reg         ROB_commit_req_A_Ihl;

  reg         rs0_i0_renamed_Ihl;
  reg         rs1_i0_renamed_Ihl;
  reg         rs0_i1_renamed_Ihl;
  reg         rs1_i1_renamed_Ihl;

  reg  [4:0]  rs0_i0_rt_slot_Ihl;
  reg  [4:0]  rs1_i0_rt_slot_Ihl;
  reg  [4:0]  rs0_i1_rt_slot_Ihl;
  reg  [4:0]  rs1_i1_rt_slot_Ihl;

  reg  [1:0]  op0_i0_mux_sel_Ihl;
  reg  [2:0]  op1_i0_mux_sel_Ihl;
  reg  [1:0]  op0_i1_mux_sel_Ihl;
  reg  [2:0]  op1_i1_mux_sel_Ihl;

  always @ (posedge clk) begin
    if( reset ) begin
      bubble_Ihl      <= 1'b1;
      squash_ir0_Dhl  <= 1'b0;
    end
    else if( !stall_Ihl ) begin
      ir0_Ihl               <= ir0_Dhl;
      ir0_squashed_Ihl      <= !ROB_req_val_1_Dhl;
      rf0_wen_Ihl           <= rf0_wen_Dhl;
      rf0_waddr_Ihl         <= rf0_waddr_Dhl;
      alu0_fn_Ihl           <= alu0_fn_Dhl;
      br0_sel_Ihl           <= br0_sel_Dhl;
      j0_en_Ihl             <= j0_en_Dhl;
      pc0_mux_sel_Ihl       <= pc0_mux_sel_Dhl;
      muldivreq0_val_Ihl    <= muldivreq0_val_Dhl;
      muldivreq0_msg_fn_Ihl <= muldivreq0_msg_fn_Dhl;
      muldiv0_mux_sel_Ihl   <= muldiv0_mux_sel_Dhl;
      execute0_mux_sel_Ihl  <= execute0_mux_sel_Dhl;
      dmemreq0_msg_rw_Ihl   <= dmemreq0_msg_rw_Dhl;
      dmemreq0_msg_len_Ihl  <= dmemreq0_msg_len_Dhl;
      dmemreq0_val_Ihl      <= dmemreq0_val_Dhl;
      dmemresp0_mux_sel_Ihl <= dmemresp0_mux_sel_Dhl;
      memex0_mux_sel_Ihl    <= memex0_mux_sel_Dhl;
      csr0_wen_Ihl          <= csr0_wen_Dhl;
      csr0_addr_Ihl         <= csr0_addr_Dhl;

      ir1_Ihl               <= ir1_Dhl;
      ir1_squashed_Ihl      <= !ROB_req_val_2_Dhl;
      rf1_wen_Ihl           <= rf1_wen_Dhl;
      rf1_waddr_Ihl         <= rf1_waddr_Dhl;
      alu1_fn_Ihl           <= alu1_fn_Dhl;
      br1_sel_Ihl           <= br1_sel_Dhl;
      j1_en_Ihl             <= j1_en_Dhl;
      pc1_mux_sel_Ihl       <= pc1_mux_sel_Dhl;
      muldivreq1_val_Ihl    <= muldivreq1_val_Dhl;
      muldivreq1_msg_fn_Ihl <= muldivreq1_msg_fn_Dhl;
      muldiv1_mux_sel_Ihl   <= muldiv1_mux_sel_Dhl;
      execute1_mux_sel_Ihl  <= execute1_mux_sel_Dhl;
      dmemreq1_msg_rw_Ihl   <= dmemreq1_msg_rw_Dhl;
      dmemreq1_msg_len_Ihl  <= dmemreq1_msg_len_Dhl;
      dmemreq1_val_Ihl      <= dmemreq1_val_Dhl;
      dmemresp1_mux_sel_Ihl <= dmemresp1_mux_sel_Dhl;
      memex1_mux_sel_Ihl    <= memex1_mux_sel_Dhl;
      csr1_wen_Ihl          <= csr1_wen_Dhl;
      csr1_addr_Ihl         <= csr1_addr_Dhl;

      rs0_i0_addr_Ihl         <= rs0_i0_addr_Dhl;
      rs1_i0_addr_Ihl         <= rs1_i0_addr_Dhl;
      rs0_i1_addr_Ihl         <= rs0_i1_addr_Dhl;
      rs1_i1_addr_Ihl         <= rs1_i1_addr_Dhl;
      rs0_i0_en_Ihl           <= rs0_i0_en_Dhl;
      rs1_i0_en_Ihl           <= rs1_i0_en_Dhl;
      rs0_i1_en_Ihl           <= rs0_i1_en_Dhl;
      rs1_i1_en_Ihl           <= rs1_i1_en_Dhl;
      ROB_fill_slot_0_Ihl   <= ROB_fill_slot_0_Dhl;
      ROB_commit_req_slot_A_Ihl   <= ROB_commit_req_slot_A_Dhl;

      rs0_i0_renamed_Ihl      <= rs0_i0_renamed_Dhl;
      rs1_i0_renamed_Ihl      <= rs1_i0_renamed_Dhl;
      rs0_i1_renamed_Ihl      <= rs0_i1_renamed_Dhl;
      rs1_i1_renamed_Ihl      <= rs1_i1_renamed_Dhl;
      rs0_i0_rt_slot_Ihl      <= rs0_i0_rt_slot_Dhl;
      rs1_i0_rt_slot_Ihl      <= rs1_i0_rt_slot_Dhl;
      rs0_i1_rt_slot_Ihl      <= rs0_i1_rt_slot_Dhl;
      rs1_i1_rt_slot_Ihl      <= rs1_i1_rt_slot_Dhl;

      op0_i0_mux_sel_Ihl      <= op0_i0_mux_sel_Dhl;
      op1_i0_mux_sel_Ihl      <= op1_i0_mux_sel_Dhl;
      op0_i1_mux_sel_Ihl      <= op0_i1_mux_sel_Dhl;
      op1_i1_mux_sel_Ihl      <= op1_i1_mux_sel_Dhl;

      bubble_Ihl <= bubble_next_Dhl;

      if( double_br_Dhl ) begin
        squash_ir0_Dhl  <= 1'b1;
      end
      else if( !stall_Dhl ) begin
        squash_ir0_Dhl  <= 1'b0;
      end
    end
  end

  // Is the current stage valid?

  wire inst_val_Ihl = ( !bubble_Ihl && !squash_Ihl );

  // PC Mux Select

  wire [1:0] pc_mux_sel_Ihl  = ( ~steering_mux_sel_Ihl ) ? pc0_mux_sel_Ihl
                             : (  steering_mux_sel_Ihl ) ? pc1_mux_sel_Ihl
                             : 2'bx;

  //----------------------------------------------------------------------
  // Issue Stage: Jump and Branch Controls
  //----------------------------------------------------------------------

  wire       brj0_taken_Ihl = ( inst_val_Ihl && j0_en_Ihl );
  wire       brj1_taken_Ihl = ( inst_val_Ihl && j1_en_Ihl );

  wire       brj_taken_Ihl  = ( ~steering_mux_sel_Ihl ) ? brj0_taken_Ihl && ir0_issued_Ihl
                            : (  steering_mux_sel_Ihl ) ? brj1_taken_Ihl && ir1_issued_Ihl
                            :  1'bx;

  //-----------------------------//
  // Issue Stage: Steering Logic //
  //-----------------------------//

  reg [31:0] irA_Ihl;
  reg        rfA_wen_Ihl;
  reg  [4:0] rfA_waddr_Ihl;
  reg  [3:0] aluA_fn_Ihl;
  reg  [4:0] ROB_fill_slot_A_Ihl;
  reg        ROB_fill_val_A_Ihl;
  

  reg [31:0] irB_Ihl;
  reg        rfB_wen_Ihl;
  reg  [4:0] rfB_waddr_Ihl;
  reg  [3:0] aluB_fn_Ihl;
  reg  [4:0] ROB_fill_slot_B_Ihl;
  reg        ROB_fill_val_B_Ihl;

  reg  [2:0] br_sel_Ihl;
  reg        muldivreq_val_Ihl;
  reg  [2:0] muldivreq_msg_fn_Ihl;
  reg        muldiv_mux_sel_Ihl;
  reg        execute_mux_sel_Ihl;
  reg        dmemreq_msg_rw_Ihl;
  reg  [1:0] dmemreq_msg_len_Ihl;
  reg        dmemreq_val_Ihl;
  reg  [2:0] dmemresp_mux_sel_Ihl;
  reg        memex_mux_sel_Ihl;
  reg        csr_wen_Ihl;
  reg [11:0] csr_addr_Ihl;

  reg        rs0_A_renamed_Ihl;
  reg        rs1_A_renamed_Ihl;
  reg  [4:0] rs0_A_rt_slot_Ihl;
  reg  [4:0] rs1_A_rt_slot_Ihl;
  reg        rs0_A_en_Ihl;
  reg        rs1_A_en_Ihl;

  reg        rs0_B_renamed_Ihl;
  reg        rs1_B_renamed_Ihl;
  reg  [4:0] rs0_B_rt_slot_Ihl;
  reg  [4:0] rs1_B_rt_slot_Ihl;
  reg        rs0_B_en_Ihl;
  reg        rs1_B_en_Ihl;


  // For disassembly in dpath

  assign instA_Ihl            = irA_Ihl;
  assign instB_Ihl            = irB_Ihl;

  
  // stall instr1 if RAW hazard to instr0

  wire ir0_squash_total_Ihl = squash_first_I_inst_Ihl || ir0_squashed_Ihl;

  wire stall_i1_RAW_rs0_Ihl = ( inst_val_Ihl && rf0_wen_Ihl )
                        && ( ( rf0_waddr_Ihl == rs0_i1_addr_Ihl ) && rs0_i1_en_Ihl )
                        && ( !ir0_squash_total_Ihl )
                        && ( !ir1_squashed_Ihl );

  wire stall_i1_RAW_rs1_Ihl = ( inst_val_Ihl && rf0_wen_Ihl )
                        && ( ( rf0_waddr_Ihl == rs1_i1_addr_Ihl ) && rs1_i1_en_Ihl )
                        && ( !ir0_squash_total_Ihl )
                        && ( !ir1_squashed_Ihl );

  // structural hazard if both instructions are 
  // loads, stores, muldivs, jumps, branches, or CSRWs.

  wire reqA_0_Ihl       = (   inst_val_Ihl && !squash_first_I_inst_Ihl )
                        && (    muldivreq0_val_Ihl
                            ||  dmemreq0_val_Ihl
                            ||  brj0_taken_Ihl
                            ||  ( br0_sel_Ihl != br_none )
                            ||  csr0_wen_Ihl
                           );

  wire reqA_1_Ihl       = (   inst_val_Ihl )
                        && (    muldivreq1_val_Ihl
                            ||  dmemreq1_val_Ihl
                            ||  brj1_taken_Ihl
                            ||  ( br1_sel_Ihl != br_none )
                            ||  csr1_wen_Ihl
                           );

  wire stall_i1_structural_Ihl = reqA_0_Ihl && reqA_1_Ihl;

  // Aggregate hazards for steering instruction 1

  wire stall_i1_steer_Ihl = ( stall_i1_RAW_rs0_Ihl || stall_i1_RAW_rs1_Ihl || stall_i1_structural_Ihl );

  // Signals to indicate when both instructions are issued, or if only
  // instruction 0 was issued
  reg squash_first_I_inst_Ihl;
  reg ir0_issued_Ihl = 1'b0;
  reg ir1_issued_Ihl = 1'b0;

  // Steering signal calculation
  // A value of 0 indicates instruction 0 being steered to A
  // A value of 1 indicates instruction 1 being steered to A
  reg steer_signal_Ihl;
  assign steering_mux_sel_Ihl = steer_signal_Ihl;

  always @(*) begin

    // If the first instruction was already issued or squashed, steer the second
    // into pipeline A.
    if( ir0_squash_total_Ihl ) begin
      steer_signal_Ihl = 1'b1;
    end

    // If the second instruction requires A, and the first does not,
    // steer the second to A.
    else if ( !reqA_0_Ihl && reqA_1_Ihl && !stall_i1_Ihl && !ir1_squashed_Ihl ) begin
      steer_signal_Ihl = 1'b1;
    end

    // Steer the first instruction to A in all other cases
    else begin
      steer_signal_Ihl = 1'b0;
    end
  end

  // Steer operand and mux select signals
  assign opA0_mux_sel_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? op0_i0_mux_sel_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? op0_i1_mux_sel_Ihl
    :  2'bx;
  assign opA1_mux_sel_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? op1_i0_mux_sel_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? op1_i1_mux_sel_Ihl
    :  2'bx;
  assign opB0_mux_sel_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? op0_i1_mux_sel_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? op0_i0_mux_sel_Ihl
    :  2'bx;
  assign opB1_mux_sel_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? op1_i1_mux_sel_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? op1_i0_mux_sel_Ihl
    :  2'bx;


  assign opA0_byp_ROB_slot_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? rs0_i0_rt_slot_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? rs0_i1_rt_slot_Ihl
    :  5'bx;
  assign opA1_byp_ROB_slot_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? rs1_i0_rt_slot_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? rs1_i1_rt_slot_Ihl
    :  5'bx;
  assign opB0_byp_ROB_slot_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? rs0_i1_rt_slot_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? rs0_i0_rt_slot_Ihl
    :  5'bx;
  assign opB1_byp_ROB_slot_Ihl
    = ( steering_mux_sel_Ihl == 1'b0 ) ? rs1_i1_rt_slot_Ihl
    : ( steering_mux_sel_Ihl == 1'b1 ) ? rs1_i0_rt_slot_Ihl
    :  5'bx;



  always @(*) begin
    if ( steering_mux_sel_Ihl == 1'b0 ) begin

      // Issue the instruction0 (ir0) to A
      irA_Ihl              = ir0_Ihl;
      rfA_wen_Ihl          = rf0_wen_Ihl;
      rfA_waddr_Ihl        = rf0_waddr_Ihl;
      aluA_fn_Ihl          = alu0_fn_Ihl;
      ROB_fill_slot_A_Ihl  = ROB_fill_slot_0_Ihl;

      br_sel_Ihl           = br0_sel_Ihl;
      muldivreq_val_Ihl    = muldivreq0_val_Ihl;
      muldivreq_msg_fn_Ihl = muldivreq0_msg_fn_Ihl;
      muldiv_mux_sel_Ihl   = muldiv0_mux_sel_Ihl;
      execute_mux_sel_Ihl  = execute0_mux_sel_Ihl;
      dmemreq_msg_rw_Ihl   = dmemreq0_msg_rw_Ihl;
      dmemreq_msg_len_Ihl  = dmemreq0_msg_len_Ihl;
      dmemreq_val_Ihl      = dmemreq0_val_Ihl;
      dmemresp_mux_sel_Ihl = dmemresp0_mux_sel_Ihl;
      memex_mux_sel_Ihl    = memex0_mux_sel_Ihl;
      csr_wen_Ihl          = csr0_wen_Ihl;
      csr_addr_Ihl         = csr0_addr_Ihl;

      rs0_A_renamed_Ihl     = rs0_i0_renamed_Ihl;
      rs1_A_renamed_Ihl     = rs1_i0_renamed_Ihl;
      rs0_A_rt_slot_Ihl     = rs0_i0_rt_slot_Ihl;
      rs1_A_rt_slot_Ihl     = rs1_i0_rt_slot_Ihl;
      rs0_A_en_Ihl          = rs0_i0_en_Ihl;
      rs1_A_en_Ihl          = rs1_i0_en_Ihl;

      if( !stall_i0_Ihl ) begin
        ir0_issued_Ihl     = 1'b1;
        ROB_fill_val_A_Ihl = 1'b1;
      end
      else begin
        ir0_issued_Ihl     = 1'b0;
        ROB_fill_val_A_Ihl = 1'b0;
      end

      // Issue ir1 to B if ir1, ir0, and X0 are not stalled and 
      // if ir0 is not a jump instruction
      if( !stall_i1_Ihl && !stall_i0_Ihl && !brj_taken_Ihl && !ir1_squashed_Ihl ) begin
        irB_Ihl              = ir1_Ihl;
        rfB_wen_Ihl          = rf1_wen_Ihl;
        rfB_waddr_Ihl        = rf1_waddr_Ihl;
        aluB_fn_Ihl          = alu1_fn_Ihl;
        ROB_fill_slot_B_Ihl  = ROB_commit_req_slot_A_Ihl;
        ROB_fill_val_B_Ihl   = 1'b1;

        rs0_B_renamed_Ihl     = rs0_i1_renamed_Ihl;
        rs1_B_renamed_Ihl     = rs1_i1_renamed_Ihl;
        rs0_B_rt_slot_Ihl     = rs0_i1_rt_slot_Ihl;
        rs1_B_rt_slot_Ihl     = rs1_i1_rt_slot_Ihl;
        rs0_B_en_Ihl          = rs0_i1_en_Ihl;
        rs1_B_en_Ihl          = rs1_i1_en_Ihl;

        ir1_issued_Ihl       = 1'b1;
      end

      // Send an invalid instruction down B if ir1 is stalled
      // Set ir1_issued to 0 to indicate that only ir0 has been issued
      else begin

        irB_Ihl              = 32'b0;
        rfB_wen_Ihl          = 1'b0;
        ir1_issued_Ihl       = 1'b0;
        ROB_fill_val_B_Ihl   = 1'b0;
      
      end
    end
    else if ( steering_mux_sel_Ihl == 1'b1 ) begin

      // Issue the second instruction (ir1) to A
      irA_Ihl              = ir1_Ihl;
      rfA_wen_Ihl          = rf1_wen_Ihl;
      rfA_waddr_Ihl        = rf1_waddr_Ihl;
      aluA_fn_Ihl          = alu1_fn_Ihl;
      ROB_fill_slot_A_Ihl  = ROB_commit_req_slot_A_Ihl;

      br_sel_Ihl           = br1_sel_Ihl;
      aluA_fn_Ihl          = alu1_fn_Ihl;
      muldivreq_val_Ihl    = muldivreq1_val_Ihl;
      muldivreq_msg_fn_Ihl = muldivreq1_msg_fn_Ihl;
      muldiv_mux_sel_Ihl   = muldiv1_mux_sel_Ihl;
      execute_mux_sel_Ihl  = execute1_mux_sel_Ihl;
      dmemreq_msg_rw_Ihl   = dmemreq1_msg_rw_Ihl;
      dmemreq_msg_len_Ihl  = dmemreq1_msg_len_Ihl;
      dmemreq_val_Ihl      = dmemreq1_val_Ihl;
      dmemresp_mux_sel_Ihl = dmemresp1_mux_sel_Ihl;
      memex_mux_sel_Ihl    = memex1_mux_sel_Ihl;
      csr_wen_Ihl          = csr1_wen_Ihl;
      csr_addr_Ihl         = csr1_addr_Ihl;

      rs0_A_renamed_Ihl     = rs0_i1_renamed_Ihl;
      rs1_A_renamed_Ihl     = rs1_i1_renamed_Ihl;
      rs0_A_rt_slot_Ihl     = rs0_i1_rt_slot_Ihl;
      rs1_A_rt_slot_Ihl     = rs1_i1_rt_slot_Ihl;
      rs0_A_en_Ihl          = rs0_i1_en_Ihl;
      rs1_A_en_Ihl          = rs1_i1_en_Ihl;

      // If i1 and X0 are not stalled, both instructions will issued,
      // set ir0_issued_only to 0, and ir1_issued to 1
      if( !stall_i1_Ihl ) begin
        ir1_issued_Ihl       = 1'b1;
        ROB_fill_val_A_Ihl   = 1'b1;
      end
      else begin
        ir1_issued_Ihl       = 1'b0;
        ROB_fill_val_A_Ihl   = 1'b0;
      end

      // If ir0 has not been issued, send it to path B
      if( !ir0_squash_total_Ihl ) begin
        irB_Ihl              = ir0_Ihl;
        rfB_wen_Ihl          = rf0_wen_Ihl;
        rfB_waddr_Ihl        = rf0_waddr_Ihl;
        aluB_fn_Ihl          = alu0_fn_Ihl;
        ROB_fill_slot_B_Ihl  = ROB_fill_slot_0_Ihl;
        ROB_fill_val_B_Ihl   = 1'b1;

        rs0_B_renamed_Ihl     = rs0_i0_renamed_Ihl;
        rs1_B_renamed_Ihl     = rs1_i0_renamed_Ihl;
        rs0_B_rt_slot_Ihl     = rs0_i0_rt_slot_Ihl;
        rs1_B_rt_slot_Ihl     = rs1_i0_rt_slot_Ihl;
        rs0_B_en_Ihl          = rs0_i0_en_Ihl;
        rs1_B_en_Ihl          = rs1_i0_en_Ihl;

        ir0_issued_Ihl       = 1'b1;
      end

      // Otherwise send an invalid instruction down path B
      else begin
        irB_Ihl              = 32'b0;
        rfB_wen_Ihl          = 1'b0;
        ir0_issued_Ihl       = 1'b0;
        ROB_fill_val_B_Ihl   = 1'b0;
      end
    end

    // Do not issue if X0 is stalled or instructions are invalid
    if ( !inst_val_Ihl || stall_X0hl ) begin
      ir0_issued_Ihl         = 1'b0;
      ir1_issued_Ihl         = 1'b0;
      ROB_fill_val_A_Ihl     = 1'b0;
      ROB_fill_val_B_Ihl     = 1'b0;
    end
  end

  //----------------------------------------------------------------------
  // Scoreboard Logic
  //----------------------------------------------------------------------

  wire [1:0] func_irA_Ihl = { muldivreq_val_Ihl, dmemreq_val_Ihl };

  wire A_issued_Ihl = ( !steering_mux_sel_Ihl && ir0_issued_Ihl ) ||
                      (  steering_mux_sel_Ihl && ir1_issued_Ihl );
  wire B_issued_Ihl = ( !steering_mux_sel_Ihl && ir1_issued_Ihl ) ||
                      (  steering_mux_sel_Ihl && ir0_issued_Ihl );

  wire [31:0] src_ready_sb;


  wire stall_i0_sb_Ihl = (    ( !src_ready_sb[rs0_i0_rt_slot_Ihl] && rs0_i0_en_Ihl && rs0_i0_renamed_Ihl )
                          ||  ( !src_ready_sb[rs1_i0_rt_slot_Ihl] && rs1_i0_en_Ihl && rs1_i0_renamed_Ihl ) 
                          ) && inst_val_Ihl;

  wire stall_i1_sb_Ihl = (    ( !src_ready_sb[rs0_i1_rt_slot_Ihl] && rs0_i1_en_Ihl && rs0_i1_renamed_Ihl )
                          ||  ( !src_ready_sb[rs1_i1_rt_slot_Ihl] && rs1_i1_en_Ihl && rs1_i1_renamed_Ihl ) 
                          ) && inst_val_Ihl;


  riscv_CoreScoreboard scoreboard
  (
    .clk                (clk),
    .reset              (reset),

    .rs0_A              (rs0_A_rt_slot_Ihl),
    .rs0_A_en           (rs0_A_en_Ihl),
    .rs0_A_renamed      (rs0_A_renamed_Ihl),
    .rs1_A              (rs1_A_rt_slot_Ihl),
    .rs1_A_en           (rs1_A_en_Ihl),
    .rs1_A_renamed      (rs1_A_renamed_Ihl),
    .rd_A               (ROB_fill_slot_A_Ihl),
    .rd_A_en            (ROB_fill_val_A_Ihl),
    .func_irA           (func_irA_Ihl),
    .A_issued           (A_issued_Ihl),

    .rs0_B              (rs0_B_rt_slot_Ihl),
    .rs0_B_en           (rs0_B_en_Ihl),
    .rs0_B_renamed      (rs0_B_renamed_Ihl),
    .rs1_B              (rs1_B_rt_slot_Ihl),
    .rs1_B_en           (rs1_B_en_Ihl),
    .rs1_B_renamed      (rs1_B_renamed_Ihl),
    .rd_B               (ROB_fill_slot_B_Ihl),
    .rd_B_en            (ROB_fill_val_B_Ihl),
    .B_issued           (B_issued_Ihl),

    .stall_X0hl         (stall_X0hl),
    .stall_X1hl         (stall_X1hl),
    .stall_X2hl         (stall_X2hl),
    .stall_X3hl         (stall_X3hl),
    .stall_Whl          (stall_Whl),

    .ROB_commit_slot_A  (ROB_commit_slot_A_Chl),
    .ROB_commit_ready_A   (ROB_commit_ready_A),
    .ROB_commit_slot_B  (ROB_commit_slot_B_Chl),
    .ROB_commit_ready_B   (ROB_commit_ready_B),

    .op0_A_byp_sel   (opA0_byp_mux_sel_Ihl),
    .op1_A_byp_sel   (opA1_byp_mux_sel_Ihl),
    .op0_B_byp_sel   (opB0_byp_mux_sel_Ihl),
    .op1_B_byp_sel   (opB1_byp_mux_sel_Ihl),

    .src_ready          (src_ready_sb)
  );

  //------------------------//
  // Squash and Stall Logic //
  //------------------------//
  
  // Squash instruction in I if a valid branch in X is taken
  wire squash_Ihl = ( inst_val_X0hl && brj_taken_X0hl );

  // Aggregate Stall Signal
  wire stall_i0_Ihl = ( stall_i0_sb_Ihl && !ir0_squash_total_Ihl );

  wire stall_i1_Ihl = ( stall_i1_sb_Ihl || stall_i1_steer_Ihl && !ir1_squashed_Ihl );

  wire stall_Ihl    = ( stall_X0hl || stall_i0_Ihl || stall_i1_Ihl )
                      && !brj_taken_Ihl && !brj_taken_X0hl;


  // Next bubble bit
  // Send a bubble to the next stage if either ir0 is valid and stalled,
  // or if ir0 is invalid and i1 is stalled
  wire bubble_sel_Ihl  = ( squash_Ihl || stall_X0hl || stall_i0_Ihl
                        || ( stall_i1_Ihl && squash_first_I_inst_Ihl ) );
  wire bubble_next_Ihl = ( !bubble_sel_Ihl ) ? bubble_Ihl
                       : ( bubble_sel_Ihl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X0 <- I
  //----------------------------------------------------------------------

  reg  [2:0] br_sel_X0hl;
  reg        muldivreq_val_X0hl;
  reg  [2:0] muldivreq_msg_fn_X0hl;
  reg        muldiv_mux_sel_X0hl;
  reg        execute_mux_sel_X0hl;
  reg        memex_mux_sel_X0hl;
  reg        dmemreq_msg_rw_X0hl;
  reg  [1:0] dmemreq_msg_len_X0hl;
  reg        dmemreq_val_X0hl;
  reg  [2:0] dmemresp_mux_sel_X0hl;
  reg        csr_wen_X0hl;
  reg [11:0] csr_addr_X0hl;

  reg        bubble_X0hl;

  reg [31:0] irA_X0hl;
  reg  [3:0] aluA_fn_X0hl;
  reg        rfA_wen_X0hl;
  reg  [4:0] rfA_waddr_X0hl;
  reg  [4:0] ROB_fill_slot_A_X0hl;
  reg        ROB_fill_val_A_X0hl;

  reg [31:0] irB_X0hl;
  reg  [3:0] aluB_fn_X0hl;
  reg        rfB_wen_X0hl;
  reg  [4:0] rfB_waddr_X0hl;
  reg  [4:0] ROB_fill_slot_B_X0hl;
  reg        ROB_fill_val_B_X0hl;


  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X0hl             <= 1'b1;
      squash_first_I_inst_Ihl <= 1'b0;
    end
    else if( !stall_X0hl ) begin

      br_sel_X0hl           <= br_sel_Ihl;
      
      muldivreq_val_X0hl    <= muldivreq_val_Ihl;
      muldivreq_msg_fn_X0hl <= muldivreq_msg_fn_Ihl;
      muldiv_mux_sel_X0hl   <= muldiv_mux_sel_Ihl;
      execute_mux_sel_X0hl  <= execute_mux_sel_Ihl;
      memex_mux_sel_X0hl    <= memex_mux_sel_Ihl;
      dmemreq_msg_rw_X0hl   <= dmemreq_msg_rw_Ihl;
      dmemreq_msg_len_X0hl  <= dmemreq_msg_len_Ihl;
      dmemreq_val_X0hl      <= dmemreq_val_Ihl;
      dmemresp_mux_sel_X0hl <= dmemresp_mux_sel_Ihl;
      
      csr_wen_X0hl          <= csr_wen_Ihl;
      csr_addr_X0hl         <= csr_addr_Ihl;

      bubble_X0hl           <= bubble_next_Ihl;


      irA_X0hl              <= irA_Ihl;
      aluA_fn_X0hl          <= aluA_fn_Ihl;
      rfA_wen_X0hl          <= rfA_wen_Ihl;
      rfA_waddr_X0hl        <= rfA_waddr_Ihl;
      ROB_fill_slot_A_X0hl  <= ROB_fill_slot_A_Ihl;
      ROB_fill_val_A_X0hl   <= ROB_fill_val_A_Ihl && !bubble_next_Ihl;

      irB_X0hl              <= irB_Ihl;
      aluB_fn_X0hl          <= aluB_fn_Ihl;
      rfB_wen_X0hl          <= rfB_wen_Ihl;
      rfB_waddr_X0hl        <= rfB_waddr_Ihl;
      ROB_fill_slot_B_X0hl  <= ROB_fill_slot_B_Ihl;
      ROB_fill_val_B_X0hl   <= ROB_fill_val_B_Ihl && !bubble_next_Ihl;

      // Squash the first I instruction if only instruction 0 is issued
      // and it is not a jump

      if( ir0_issued_Ihl && !ir1_issued_Ihl && !brj_taken_Ihl ) begin
        squash_first_I_inst_Ihl <= 1'b1;
      end
      else if( ir1_issued_Ihl || brj_taken_Ihl || brj_taken_X0hl ) begin
        squash_first_I_inst_Ihl <= 1'b0;
      end
    end
  end

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_X0hl = ( !bubble_X0hl && !squash_X0hl );

  // Muldiv request

  assign muldivreq_val = muldivreq_val_Ihl && inst_val_Ihl && (!stall_X0hl);
  assign muldivresp_rdy = !stall_X3hl;

  // Only send a valid dmem request if not stalled and not speculative

  assign dmemreq_msg_rw  = dmemreq_msg_rw_X0hl;
  assign dmemreq_msg_len = dmemreq_msg_len_X0hl;
  assign dmemreq_val     = ( inst_val_X0hl && !stall_X0hl && dmemreq_val_X0hl );

  // Resolve Branch

  wire bne_taken_X0hl  = ( ( br_sel_X0hl == br_bne ) && branch_cond_ne_X0hl );
  wire beq_taken_X0hl  = ( ( br_sel_X0hl == br_beq ) && branch_cond_eq_X0hl );
  wire blt_taken_X0hl  = ( ( br_sel_X0hl == br_blt ) && branch_cond_lt_X0hl );
  wire bltu_taken_X0hl = ( ( br_sel_X0hl == br_bltu) && branch_cond_ltu_X0hl);
  wire bge_taken_X0hl  = ( ( br_sel_X0hl == br_bge ) && branch_cond_ge_X0hl );
  wire bgeu_taken_X0hl = ( ( br_sel_X0hl == br_bgeu) && branch_cond_geu_X0hl);

  wire brj_resolved_X0hl = ( inst_val_X0hl && (br_sel_X0hl != br_none) && !stall_X0hl );


  wire any_br_taken_X0hl  = (  beq_taken_X0hl
                          ||   bne_taken_X0hl
                          ||   blt_taken_X0hl
                          ||   bltu_taken_X0hl
                          ||   bge_taken_X0hl
                          ||   bgeu_taken_X0hl );

  wire brj_taken_X0hl = ( inst_val_X0hl && any_br_taken_X0hl && !stall_X0hl );

  // Dummy Squash Signal

  wire squash_X0hl = 1'b0;

  // Dummy muldiv stall signal

  wire stall_muldiv_X0hl = 1'b0;

  // Stall in X if imem is not ready

  wire stall_imem_X0hl = !imemreq0_rdy || !imemreq1_rdy;

  // Stall in X if dmem is not ready and there was a valid request

  wire stall_dmem_X0hl = ( dmemreq_val_X0hl && inst_val_X0hl && !dmemreq_rdy );

  // Aggregate Stall Signal

  assign stall_X0hl = ( stall_X1hl || stall_muldiv_X0hl || stall_imem_X0hl || stall_dmem_X0hl );

  // Next bubble bit

  wire bubble_sel_X0hl  = ( squash_X0hl || stall_X0hl );
  wire bubble_next_X0hl = ( !bubble_sel_X0hl ) ? bubble_X0hl
                        : ( bubble_sel_X0hl )  ? 1'b1
                        :                        1'bx;

  //----------------------------------------------------------------------
  // X1 <- X0
  //----------------------------------------------------------------------

  reg [31:0] irA_X1hl;
  reg        rfA_wen_X1hl;
  reg  [4:0] rfA_waddr_X1hl;
  reg  [4:0] ROB_fill_slot_A_X1hl;
  reg        ROB_fill_val_A_X1hl;

  reg        dmemreq_val_X1hl;
  reg  [2:0] dmemresp_mux_sel_X1hl;
  reg        memex_mux_sel_X1hl;
  reg        execute_mux_sel_X1hl;
  reg        muldiv_mux_sel_X1hl;
  reg        csr_wen_X1hl;
  reg  [4:0] csr_addr_X1hl;

  reg        bubble_X1hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      dmemreq_val_X1hl <= 1'b0;
      bubble_X1hl <= 1'b1;
    end
    else if( !stall_X1hl ) begin
      irA_X1hl              <= irA_X0hl;
      rfA_wen_X1hl          <= rfA_wen_X0hl;
      rfA_waddr_X1hl        <= rfA_waddr_X0hl;

      ROB_fill_slot_A_X1hl  <= ROB_fill_slot_A_X0hl;
      ROB_fill_val_A_X1hl   <= ROB_fill_val_A_X0hl && !bubble_next_X0hl;

      dmemreq_val_X1hl      <= dmemreq_val;
      dmemresp_mux_sel_X1hl <= dmemresp_mux_sel_X0hl;
      memex_mux_sel_X1hl    <= memex_mux_sel_X0hl;
      execute_mux_sel_X1hl  <= execute_mux_sel_X0hl;
      muldiv_mux_sel_X1hl   <= muldiv_mux_sel_X0hl;

      csr_wen_X1hl          <= csr_wen_X0hl;
      csr_addr_X1hl         <= csr_addr_X0hl;

      bubble_X1hl           <= !(ROB_fill_val_A_X0hl && !bubble_next_X0hl);
    end
  end

  //----------------------------------------------------------------------
  // X1 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X1hl = ( !bubble_X1hl && !squash_X1hl );

  // Data memory queue control signals

  assign dmemresp_queue_en_X1hl = ( stall_X1hl && dmemresp_val );
  wire   dmemresp_queue_val_next_X1hl = stall_X1hl && ( dmemresp_val || dmemresp_queue_val_X1hl );

  // Dummy Squash Signal

  wire squash_X1hl = 1'b0;

  // Stall in X1 if memory response is not returned for a valid request

  wire stall_dmem_X1hl =  ( !reset && dmemreq_val_X1hl && 
                            inst_val_X1hl && !dmemresp_val && 
                            !dmemresp_queue_val_X1hl );
  wire stall_imem_X1hl =  ( !reset && imemreq_val_Fhl && 
                            inst_val_Fhl && !imemresp0_val && 
                            !imemresp0_queue_val_Fhl ) || 
                          ( !reset && imemreq_val_Fhl && 
                            inst_val_Fhl && !imemresp1_val && 
                            !imemresp1_queue_val_Fhl );

  // Aggregate Stall Signal

  wire stall_X1hl = ( stall_X2hl || stall_imem_X1hl || stall_dmem_X1hl );

  // Next bubble bit

  wire bubble_sel_X1hl  = ( squash_X1hl || stall_X1hl );
  wire bubble_next_X1hl = ( !bubble_sel_X1hl ) ? bubble_X1hl
                        : ( bubble_sel_X1hl )  ? 1'b1
                        :  1'bx;

  //----------------------------------------------------------------------
  // X2 <- X1
  //----------------------------------------------------------------------

  reg [31:0] irA_X2hl;
  reg        rfA_wen_X2hl;
  reg  [4:0] rfA_waddr_X2hl;
  reg  [4:0] ROB_fill_slot_A_X2hl;
  reg        ROB_fill_val_A_X2hl;

  reg        dmemresp_queue_val_X1hl;
  reg        csr_wen_X2hl;
  reg  [4:0] csr_addr_X2hl;
  reg        execute_mux_sel_X2hl;
  reg        muldiv_mux_sel_X2hl;

  reg        bubble_X2hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X2hl <= 1'b1;
    end
    else if( !stall_X2hl ) begin
      irA_X2hl              <= irA_X1hl;
      rfA_wen_X2hl          <= rfA_wen_X1hl;
      rfA_waddr_X2hl        <= rfA_waddr_X1hl;
      ROB_fill_slot_A_X2hl  <= ROB_fill_slot_A_X1hl;
      ROB_fill_val_A_X2hl   <= ROB_fill_val_A_X1hl && !bubble_next_X1hl;

      muldiv_mux_sel_X2hl   <= muldiv_mux_sel_X1hl;
      csr_wen_X2hl          <= csr_wen_X1hl;
      csr_addr_X2hl         <= csr_addr_X1hl;
      execute_mux_sel_X2hl  <= execute_mux_sel_X1hl;

      bubble_X2hl           <= !(ROB_fill_val_A_X1hl && !bubble_next_X1hl);
    end
    dmemresp_queue_val_X1hl <= dmemresp_queue_val_next_X1hl;
  end

  //----------------------------------------------------------------------
  // X2 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X2hl = ( !bubble_X2hl && !squash_X2hl );

  // Dummy Squash Signal

  wire squash_X2hl = 1'b0;

  // Dummy Stall Signal

  wire stall_X2hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X2hl  = ( squash_X2hl || stall_X2hl );
  wire bubble_next_X2hl = ( !bubble_sel_X2hl ) ? bubble_X2hl
                        : ( bubble_sel_X2hl )  ? 1'b1
                        :                        1'bx;

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------

  reg [31:0] irA_X3hl;
  reg        rfA_wen_X3hl;
  reg  [4:0] rfA_waddr_X3hl;
  reg  [4:0] ROB_fill_slot_A_X3hl;
  reg        ROB_fill_val_A_X3hl;

  reg        csr_wen_X3hl;
  reg  [4:0] csr_addr_X3hl;
  reg        execute_mux_sel_X3hl;
  reg        muldiv_mux_sel_X3hl;

  reg        bubble_X3hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X3hl <= 1'b1;
    end
    else if( !stall_X3hl ) begin
      irA_X3hl              <= irA_X2hl;
      rfA_wen_X3hl          <= rfA_wen_X2hl;
      rfA_waddr_X3hl        <= rfA_waddr_X2hl;
  
      ROB_fill_slot_A_X3hl  <= ROB_fill_slot_A_X2hl;
      ROB_fill_val_A_X3hl   <= ROB_fill_val_A_X2hl && !bubble_next_X2hl;

      muldiv_mux_sel_X3hl   <= muldiv_mux_sel_X2hl;
  
      csr_wen_X3hl          <= csr_wen_X2hl;
      csr_addr_X3hl         <= csr_addr_X2hl;
      execute_mux_sel_X3hl  <= execute_mux_sel_X2hl;

      bubble_X3hl           <= !(ROB_fill_val_A_X2hl && !bubble_next_X2hl);
    end
  end

  //----------------------------------------------------------------------
  // X3 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X3hl = ( !bubble_X3hl && !squash_X3hl );

  // Dummy Squash Signal

  wire squash_X3hl = 1'b0;

  // Dummy Stall Signal

  wire stall_X3hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X3hl  = ( squash_X3hl || stall_X3hl );
  wire bubble_next_X3hl = ( !bubble_sel_X3hl ) ? bubble_X3hl
                        : ( bubble_sel_X3hl )  ? 1'b1
                        :                        1'bx;

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg [31:0] irA_Whl;
  reg        rfA_wen_Whl;
  reg  [4:0] rfA_waddr_Whl;
  reg  [4:0] ROB_fill_slot_A_Whl;
  reg        ROB_fill_val_A_Whl;

  reg [31:0] irB_Whl;
  reg        rfB_wen_Whl;
  reg  [4:0] rfB_waddr_Whl;
  reg  [4:0] ROB_fill_slot_B_Whl;
  reg        ROB_fill_val_B_Whl;

  reg        csr_wen_Whl;
  reg  [4:0] csr_addr_Whl;
  reg        bubble_Whl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Whl <= 1'b1;
    end
    else if( !stall_Whl ) begin
      irA_Whl             <= irA_X3hl;
      rfA_wen_Whl         <= rfA_wen_X3hl;
      rfA_waddr_Whl       <= rfA_waddr_X3hl;
  
      ROB_fill_slot_A_Whl <= ROB_fill_slot_A_X3hl;
      ROB_fill_val_A_Whl  <= ROB_fill_val_A_X3hl && !bubble_next_X3hl;

      irB_Whl             <= irB_X0hl;
      rfB_wen_Whl         <= rfB_wen_X0hl;
      rfB_waddr_Whl       <= rfB_waddr_X0hl;
  
      ROB_fill_slot_B_Whl <= ROB_fill_slot_B_X0hl;
      ROB_fill_val_B_Whl  <= ROB_fill_val_B_X0hl && !bubble_next_X0hl;

      csr_wen_Whl         <= csr_wen_X3hl;
      csr_addr_Whl        <= csr_addr_X3hl;

      bubble_Whl          <= !(ROB_fill_val_B_X0hl && !bubble_next_X0hl) &&
                             !(ROB_fill_val_A_X3hl && !bubble_next_X3hl);
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_Whl = ( !bubble_Whl && !squash_Whl );

  // Only set register file wen if instruction is valid

  wire rfA_wen_out_Whl = ( ROB_fill_val_A_Whl && !stall_Whl && rfA_wen_Whl );
  wire rfB_wen_out_Whl = ( ROB_fill_val_B_Whl && !stall_Whl && rfB_wen_Whl );

  // Dummy squash and stall signals

  wire squash_Whl = 1'b0;
  wire stall_Whl  = 1'b0;

  // Reorder Buffer signals

  wire ROB_fill_wen_A_Whl = rfA_wen_out_Whl && ROB_fill_val_A_Whl;
  wire ROB_fill_wen_B_Whl = rfB_wen_out_Whl && ROB_fill_val_B_Whl;

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] irA_debug;
  reg [31:0] irB_debug;
  reg        inst_val_debug;

  always @ ( posedge clk ) begin
    irA_debug       <= irA_Whl;
    inst_val_debug <= inst_val_Whl;
    irB_debug       <= irB_Whl;   
  end

  //----------------------------------------------------------------------
  // CSR register
  //----------------------------------------------------------------------

  reg  [31:0] csr_status;
  reg         csr_stats;

  always @ ( posedge clk ) begin
    if ( csr_wen_Whl && ROB_fill_val_A_Whl ) begin
      case ( csr_addr_Whl )
        12'd10 : csr_stats  <= proc2csr_data_Whl[0];
        12'd21 : csr_status <= proc2csr_data_Whl;
      endcase
    end
  end

//========================================================================
// Disassemble instructions
//========================================================================

  `ifndef SYNTHESIS

  riscv_InstMsgDisasm inst0_msg_disasm_D
  (
    .msg ( ir0_Dhl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X0
  (
    .msg ( irA_X0hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X1
  (
    .msg ( irA_X1hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X2
  (
    .msg ( irA_X2hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_X3
  (
    .msg ( irA_X3hl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_W
  (
    .msg ( irA_Whl )
  );

  riscv_InstMsgDisasm instA_msg_disasm_debug
  (
    .msg ( irA_debug )
  );

  riscv_InstMsgDisasm inst1_msg_disasm_D
  (
    .msg ( ir1_Dhl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X0
  (
    .msg ( irB_X0hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X1
  (
    .msg ( irB_X0hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X2
  (
    .msg ( irB_X0hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_X3
  (
    .msg ( irB_X0hl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_W
  (
    .msg ( irB_Whl )
  );

  riscv_InstMsgDisasm instB_msg_disasm_debug
  (
    .msg ( irB_debug )
  );

  `endif

//========================================================================
// Assertions
//========================================================================
// Detect illegal instructions and terminate the simulation if multiple
// illegal instructions are detected in succession.

  `ifndef SYNTHESIS

  reg overload = 1'b0;

  always @ ( posedge clk ) begin
    if (( !cs0[`RISCV_INST_MSG_INST_VAL] && !reset ) 
     || ( !cs1[`RISCV_INST_MSG_INST_VAL] && !reset )) begin
      $display(" RTL-ERROR : %m : Illegal instruction!");

      if ( overload == 1'b1 ) begin
        $finish;
      end

      overload = 1'b1;
    end
    else begin
      overload = 1'b0;
    end
  end

  `endif

//========================================================================
// Stats
//========================================================================

  `ifndef SYNTHESIS

  reg [31:0] num_inst    = 32'b0;
  reg [31:0] num_cycles  = 32'b0;
  reg        stats_en    = 1'b0; // Used for enabling stats on asm tests

  wire ROB_commit_spec_A;
  wire ROB_commit_spec_B;

  wire count0 = ROB_commit_ready_A && !ROB_commit_spec_A;
  wire count1 = ROB_commit_ready_B && !ROB_commit_spec_B;

  always @( posedge clk ) begin
    if ( !reset ) begin

      // Count cycles if stats are enabled

      if ( stats_en || csr_stats ) begin
        num_cycles = num_cycles + 1;

        // Count instructions that reach writeback
        if ( count0 && count1 ) begin
          num_inst = num_inst + 2;
        end
        else if ( count0 || count1 ) begin
          num_inst = num_inst + 1;
        end
      end

    end
  end

  `endif

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
