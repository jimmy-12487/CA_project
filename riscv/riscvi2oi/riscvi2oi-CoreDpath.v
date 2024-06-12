//=========================================================================
// 7-Stage RISCV Datapath
//=========================================================================

`ifndef RISCV_CORE_DPATH_V
`define RISCV_CORE_DPATH_V

`include "riscvi2oi-CoreDpathPipeMulDiv.v"
`include "riscvi2oi-InstMsg.v"
`include "riscvi2oi-CoreDpathAlu.v"
`include "riscvi2oi-CoreDpathRegfile.v"

module riscv_CoreDpath
(
  input clk,
  input reset,

  // Instruction Memory Port

  output [31:0] imemreq0_msg_addr,
  output [31:0] imemreq1_msg_addr,

  // Data Memory Port

  output [31:0] dmemreq_msg_addr,
  output [31:0] dmemreq_msg_data,
  input  [31:0] dmemresp_msg_data,

  // Controls Signals (ctrl->dpath)

  input   [1:0] pc_mux_sel_Phl,
  input         steering_mux_sel_Ihl,
  input   [3:0] opA0_byp_mux_sel_Ihl,
  input   [1:0] opA0_mux_sel_Ihl,
  input   [3:0] opA1_byp_mux_sel_Ihl,
  input   [2:0] opA1_mux_sel_Ihl,
  input   [3:0] opB0_byp_mux_sel_Ihl,
  input   [1:0] opB0_mux_sel_Ihl,
  input   [3:0] opB1_byp_mux_sel_Ihl,
  input   [2:0] opB1_mux_sel_Ihl,
  input  [31:0] instA_Ihl,
  input  [31:0] instB_Ihl,
  input   [3:0] aluA_fn_X0hl,
  input   [3:0] aluB_fn_X0hl,
  input   [2:0] muldivreq_msg_fn_Ihl,
  input         muldivreq_val,
  output        muldivreq_rdy,
  output        muldivresp_val,
  input         muldivresp_rdy,
  input   [2:0] dmemresp_mux_sel_X1hl,
  input         dmemresp_queue_en_X1hl,
  input         dmemresp_queue_val_X1hl,
  input         execute_mux_sel_X3hl,
  input         muldiv_mux_sel_X3hl,
  input         memex_mux_sel_X1hl,
  input         stall_Fhl,
  input         stall_Dhl,
  input         stall_Ihl,
  input         stall_X0hl,
  input         stall_X1hl,
  input         stall_X2hl,
  input         stall_X3hl,
  input         stall_Whl,

  // Control Signals (dpath->ctrl)

  output        branch_cond_eq_X0hl,
  output        branch_cond_ne_X0hl,
  output        branch_cond_lt_X0hl,
  output        branch_cond_ltu_X0hl,
  output        branch_cond_ge_X0hl,
  output        branch_cond_geu_X0hl,
  output [31:0] proc2csr_data_Whl,

  // Reorder Buffer Signals (ctrl->dpath)

  input [ 4:0] opA0_byp_ROB_slot_Ihl,
  input [ 4:0] opA1_byp_ROB_slot_Ihl,
  input [ 4:0] opB0_byp_ROB_slot_Ihl,
  input [ 4:0] opB1_byp_ROB_slot_Ihl,

  input        ROB_fill_wen_A_Whl,
  input [ 4:0] ROB_commit_req_slot_A_Whl,
  input        ROB_fill_wen_B_Whl,
  input [ 4:0] ROB_commit_req_slot_B_Whl,

  input        ROB_commit_wen_1_Chl,
  input [ 4:0] ROB_commit_slot_1_Chl,
  input [ 4:0] ROB_commit_waddr_1_Chl,
  input        ROB_commit_wen_2_Chl,
  input [ 4:0] ROB_commit_slot_2_Chl,
  input [ 4:0] ROB_commit_waddr_2_Chl
);

  //--------------------------------------------------------------------
  // PC Logic Stage
  //--------------------------------------------------------------------

  // PC mux

  wire [31:0] pc_plus8_Phl;
  wire [31:0] branch_targ_Phl;
  wire [31:0] jump_targ_Phl;
  wire [31:0] jumpreg_targ_Phl;
  wire [31:0] pc_mux_out_Phl;

  wire [31:0] reset_vector = 32'h00080000;

  // Pull mux inputs from later stages

  assign pc_plus8_Phl       = pc_plus8_Fhl;
  assign branch_targ_Phl    = branch_targ_X0hl;
  assign jump_targ_Phl      = jump_targ_Ihl;
  assign jumpreg_targ_Phl   = jumpreg_targ_Ihl;

  assign pc_mux_out_Phl
    = ( pc_mux_sel_Phl == 2'd0 ) ? pc_plus8_Phl
    : ( pc_mux_sel_Phl == 2'd1 ) ? branch_targ_Phl
    : ( pc_mux_sel_Phl == 2'd2 ) ? jump_targ_Phl
    : ( pc_mux_sel_Phl == 2'd3 ) ? jumpreg_targ_Phl
    :                              32'bx;

  // Send out imem request early

  assign imemreq0_msg_addr
    = ( reset ) ? reset_vector
    :             pc_mux_out_Phl;
  assign imemreq1_msg_addr
    = ( reset ) ? reset_vector + 32'd4
    :             pc_mux_out_Phl + 32'd4;

  //----------------------------------------------------------------------
  // F <- P
  //----------------------------------------------------------------------

  reg  [31:0] pc_Fhl;

  always @ (posedge clk) begin
    if( reset ) begin
      pc_Fhl <= reset_vector;
    end
    else if( !stall_Fhl ) begin
      pc_Fhl <= pc_mux_out_Phl;
    end
  end

  //--------------------------------------------------------------------
  // Fetch StageopB1_byp_ROB_slot_Ihl
  // PC incrementer

  wire [31:0] pc_plus4_Fhl;
  wire [31:0] pc_plus8_Fhl;

  assign pc_plus4_Fhl = pc_Fhl + 32'd4;
  assign pc_plus8_Fhl = pc_Fhl + 32'd8;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] pc_Dhl;
  reg [31:0] pc_plus4_Dhl;
  reg [31:0] pc_plus8_Dhl;

  always @ (posedge clk) begin
    if( !stall_Dhl ) begin
      pc_Dhl       <= pc_Fhl;
      pc_plus4_Dhl <= pc_plus4_Fhl;
      pc_plus8_Dhl <= pc_plus8_Fhl;
    end
  end

  //----------------------------------------------------------------------
  // I <- D
  //----------------------------------------------------------------------

  reg [31:0] pc_Ihl;
  reg [31:0] pc_plus4_Ihl;
  reg [31:0] pc_plus8_Ihl;

  always @ (posedge clk) begin
    if( !stall_Ihl ) begin
      pc_Ihl       <= pc_Dhl;
      pc_plus4_Ihl <= pc_plus4_Dhl;
      pc_plus8_Ihl <= pc_plus8_Dhl;
    end
  end

  //--------------------------------------------------------------------
  // Issue Stage (Register Read)
  //--------------------------------------------------------------------

  // Parse instruction fields

  wire   [4:0] instA_rs1_Ihl;
  wire   [4:0] instA_rs2_Ihl;
  wire   [4:0] instA_rd_Ihl;
  wire   [4:0] instA_shamt_Ihl;
  wire  [31:0] instA_imm_i_Ihl;
  wire  [31:0] instA_imm_u_Ihl;
  wire  [31:0] instA_imm_uj_Ihl;
  wire  [31:0] instA_imm_s_Ihl;
  wire  [31:0] instA_imm_sb_Ihl;

  riscv_InstMsgFromBits instA_msg_from_bits
  (
    .msg      (instA_Ihl),
    .opcode   (),
    .rs1      (instA_rs1_Ihl),
    .rs2      (instA_rs2_Ihl),
    .rd       (instA_rd_Ihl),
    .funct3   (),
    .funct7   (),
    .shamt    (instA_shamt_Ihl),
    .imm_i    (instA_imm_i_Ihl),
    .imm_s    (instA_imm_s_Ihl),
    .imm_sb   (instA_imm_sb_Ihl),
    .imm_u    (instA_imm_u_Ihl),
    .imm_uj   (instA_imm_uj_Ihl)
  );

  wire   [4:0] instB_rs1_Ihl;
  wire   [4:0] instB_rs2_Ihl;
  wire   [4:0] instB_rd_Ihl;
  wire   [4:0] instB_shamt_Ihl;
  wire  [31:0] instB_imm_i_Ihl;
  wire  [31:0] instB_imm_u_Ihl;
  wire  [31:0] instB_imm_uj_Ihl;
  wire  [31:0] instB_imm_s_Ihl;
  wire  [31:0] instB_imm_sb_Ihl;

  riscv_InstMsgFromBits instB_msg_from_bits
  (
    .msg      (instB_Ihl),
    .opcode   (),
    .rs1      (instB_rs1_Ihl),
    .rs2      (instB_rs2_Ihl),
    .rd       (instB_rd_Ihl),
    .funct3   (),
    .funct7   (),
    .shamt    (instB_shamt_Ihl),
    .imm_i    (instB_imm_i_Ihl),
    .imm_s    (instB_imm_s_Ihl),
    .imm_sb   (instB_imm_sb_Ihl),
    .imm_u    (instB_imm_u_Ihl),
    .imm_uj   (instB_imm_uj_Ihl)
  );

  // Branch and jump address generation

  wire [31:0] pcA_Ihl;
  wire [31:0] pcA_plus4_Ihl;
  wire [31:0] pcB_Ihl;
  wire [31:0] pcB_plus4_Ihl;
  wire [31:0] branch_targ_Ihl;
  wire [31:0] jump_targ_Ihl;

  assign pcA_Ihl = 
    (steering_mux_sel_Ihl == 1'b0) ? pc_Ihl :
    (steering_mux_sel_Ihl == 1'b1) ? pc_plus4_Ihl :
                                     32'bx;

  assign pcA_plus4_Ihl =
    (steering_mux_sel_Ihl == 1'b0) ? pc_plus4_Ihl :
    (steering_mux_sel_Ihl == 1'b1) ? pc_plus8_Ihl :
                                     32'bx;

  assign pcB_Ihl = 
    (steering_mux_sel_Ihl == 1'b0) ? pc_plus4_Ihl :
    (steering_mux_sel_Ihl == 1'b1) ? pc_Ihl :
                                     32'bx;

  assign pcB_plus4_Ihl =
    (steering_mux_sel_Ihl == 1'b0) ? pc_plus8_Ihl :
    (steering_mux_sel_Ihl == 1'b1) ? pc_plus4_Ihl :
                                     32'bx;

  assign branch_targ_Ihl = pcA_Ihl + instA_imm_sb_Ihl;
  assign jump_targ_Ihl   = pcA_Ihl + instA_imm_uj_Ihl;

  // Register file

  wire [ 4:0] rfA_raddr0_Ihl = instA_rs1_Ihl;
  wire [31:0] rfA_rdata0_Ihl;
  wire [ 4:0] rfA_raddr1_Ihl = instA_rs2_Ihl;
  wire [31:0] rfA_rdata1_Ihl;

  wire [ 4:0] rfB_raddr0_Ihl = instB_rs1_Ihl;
  wire [31:0] rfB_rdata0_Ihl;
  wire [ 4:0] rfB_raddr1_Ihl = instB_rs2_Ihl;
  wire [31:0] rfB_rdata1_Ihl;

  // Jump reg address

  wire [31:0] jumpreg_targ_Ihl;

  wire [31:0] jumpreg_targ_pretruncate_Ihl = opA0_byp_mux_out_Ihl + instA_imm_i_Ihl;
  assign jumpreg_targ_Ihl  = {jumpreg_targ_pretruncate_Ihl[31:1], 1'b0};

  // Shift amount immediate

  wire [31:0] shamtA_Ihl = { 27'b0, instA_shamt_Ihl };
  wire [31:0] shamtB_Ihl = { 27'b0, instB_shamt_Ihl };

  // Constant operand mux inputs

  wire [31:0] const0    = 32'd0;

  // Operand A0 bypass mux

  wire [31:0] opA0_byp_mux_out_Ihl
    = ( opA0_byp_mux_sel_Ihl == 4'd0 )  ? rfA_rdata0_Ihl
    : ( opA0_byp_mux_sel_Ihl == 4'd1 )  ? aluA_out_X0hl
    : ( opA0_byp_mux_sel_Ihl == 4'd2 )  ? memexA_mux_out_X1hl
    : ( opA0_byp_mux_sel_Ihl == 4'd3 )  ? memexA_mux_out_X2hl
    : ( opA0_byp_mux_sel_Ihl == 4'd4 )  ? executeA_mux_out_X3hl
    : ( opA0_byp_mux_sel_Ihl == 4'd5 )  ? wbA_mux_out_Whl
    : ( opA0_byp_mux_sel_Ihl == 4'd6 )  ? aluB_out_X0hl
    : ( opA0_byp_mux_sel_Ihl == 4'd7 )  ? aluB_out_X1hl
    : ( opA0_byp_mux_sel_Ihl == 4'd8 )  ? aluB_out_X2hl
    : ( opA0_byp_mux_sel_Ihl == 4'd9 )  ? aluB_out_X3hl
    : ( opA0_byp_mux_sel_Ihl == 4'd10 ) ? aluB_out_Whl
    : ( opA0_byp_mux_sel_Ihl == 4'd11 ) ? ROB_data[opA0_byp_ROB_slot_Ihl]
    :                                    32'bx;

  // Operand A0 mux

  wire [31:0] opA0_mux_out_Ihl
    = ( opA0_mux_sel_Ihl == 2'd0 ) ? opA0_byp_mux_out_Ihl
    : ( opA0_mux_sel_Ihl == 2'd1 ) ? pcA_Ihl
    : ( opA0_mux_sel_Ihl == 2'd2 ) ? pcA_plus4_Ihl
    : ( opA0_mux_sel_Ihl == 2'd3 ) ? const0
    :                               32'bx;

  // Operand A1 bypass mux

  wire [31:0] opA1_byp_mux_out_Ihl
    = ( opA1_byp_mux_sel_Ihl == 4'd0 )  ? rfA_rdata1_Ihl
    : ( opA1_byp_mux_sel_Ihl == 4'd1 )  ? aluA_out_X0hl
    : ( opA1_byp_mux_sel_Ihl == 4'd2 )  ? memexA_mux_out_X1hl
    : ( opA1_byp_mux_sel_Ihl == 4'd3 )  ? memexA_mux_out_X2hl
    : ( opA1_byp_mux_sel_Ihl == 4'd4 )  ? executeA_mux_out_X3hl
    : ( opA1_byp_mux_sel_Ihl == 4'd5 )  ? wbA_mux_out_Whl
    : ( opA1_byp_mux_sel_Ihl == 4'd6 )  ? aluB_out_X0hl
    : ( opA1_byp_mux_sel_Ihl == 4'd7 )  ? aluB_out_X1hl
    : ( opA1_byp_mux_sel_Ihl == 4'd8 )  ? aluB_out_X2hl
    : ( opA1_byp_mux_sel_Ihl == 4'd9 )  ? aluB_out_X3hl
    : ( opA1_byp_mux_sel_Ihl == 4'd10 ) ? aluB_out_Whl
    : ( opA1_byp_mux_sel_Ihl == 4'd11 ) ? ROB_data[opA1_byp_ROB_slot_Ihl]
    :                                   32'bx;

  // Operand A1 mux

  wire [31:0] opA1_mux_out_Ihl
    = ( opA1_mux_sel_Ihl == 3'd0 ) ? opA1_byp_mux_out_Ihl
    : ( opA1_mux_sel_Ihl == 3'd1 ) ? shamtA_Ihl
    : ( opA1_mux_sel_Ihl == 3'd2 ) ? instA_imm_u_Ihl
    : ( opA1_mux_sel_Ihl == 3'd3 ) ? instA_imm_sb_Ihl
    : ( opA1_mux_sel_Ihl == 3'd4 ) ? instA_imm_i_Ihl
    : ( opA1_mux_sel_Ihl == 3'd5 ) ? instA_imm_s_Ihl
    : ( opA1_mux_sel_Ihl == 3'd6 ) ? const0
    :                               32'bx;

  // Operand B0 bypass mux

  wire [31:0] opB0_byp_mux_out_Ihl
    = ( opB0_byp_mux_sel_Ihl == 4'd0 )  ? rfB_rdata0_Ihl
    : ( opB0_byp_mux_sel_Ihl == 4'd1 )  ? aluA_out_X0hl
    : ( opB0_byp_mux_sel_Ihl == 4'd2 )  ? memexA_mux_out_X1hl
    : ( opB0_byp_mux_sel_Ihl == 4'd3 )  ? memexA_mux_out_X2hl
    : ( opB0_byp_mux_sel_Ihl == 4'd4 )  ? executeA_mux_out_X3hl
    : ( opB0_byp_mux_sel_Ihl == 4'd5 )  ? wbA_mux_out_Whl
    : ( opB0_byp_mux_sel_Ihl == 4'd6 )  ? aluB_out_X0hl
    : ( opB0_byp_mux_sel_Ihl == 4'd7 )  ? aluB_out_X1hl
    : ( opB0_byp_mux_sel_Ihl == 4'd8 )  ? aluB_out_X2hl
    : ( opB0_byp_mux_sel_Ihl == 4'd9 )  ? aluB_out_X3hl
    : ( opB0_byp_mux_sel_Ihl == 4'd10 ) ? aluB_out_Whl
    : ( opB0_byp_mux_sel_Ihl == 4'd11 ) ? ROB_data[opB0_byp_ROB_slot_Ihl]
    :                                    32'bx;

  // Operand B0 mux

  wire [31:0] opB0_mux_out_Ihl
    = ( opB0_mux_sel_Ihl == 2'd0 ) ? opB0_byp_mux_out_Ihl
    : ( opB0_mux_sel_Ihl == 2'd1 ) ? pcB_Ihl
    : ( opB0_mux_sel_Ihl == 2'd2 ) ? pcB_plus4_Ihl
    : ( opB0_mux_sel_Ihl == 2'd3 ) ? const0
    :                               32'bx;

  // Operand B1 bypass mux

  wire [31:0] opB1_byp_mux_out_Ihl
    = ( opB1_byp_mux_sel_Ihl == 4'd0 )  ? rfB_rdata1_Ihl
    : ( opB1_byp_mux_sel_Ihl == 4'd1 )  ? aluA_out_X0hl
    : ( opB1_byp_mux_sel_Ihl == 4'd2 )  ? memexA_mux_out_X1hl
    : ( opB1_byp_mux_sel_Ihl == 4'd3 )  ? memexA_mux_out_X2hl
    : ( opB1_byp_mux_sel_Ihl == 4'd4 )  ? executeA_mux_out_X3hl
    : ( opB1_byp_mux_sel_Ihl == 4'd5 )  ? wbA_mux_out_Whl
    : ( opB1_byp_mux_sel_Ihl == 4'd6 )  ? aluB_out_X0hl
    : ( opB1_byp_mux_sel_Ihl == 4'd7 )  ? aluB_out_X1hl
    : ( opB1_byp_mux_sel_Ihl == 4'd8 )  ? aluB_out_X2hl
    : ( opB1_byp_mux_sel_Ihl == 4'd9 )  ? aluB_out_X3hl
    : ( opB1_byp_mux_sel_Ihl == 4'd10 ) ? aluB_out_Whl
    : ( opB1_byp_mux_sel_Ihl == 4'd11 ) ? ROB_data[opB1_byp_ROB_slot_Ihl]
    :                                   32'bx;

  // Operand B1 mux

  wire [31:0] opB1_mux_out_Ihl
    = ( opB1_mux_sel_Ihl == 3'd0 ) ? opB1_byp_mux_out_Ihl
    : ( opB1_mux_sel_Ihl == 3'd1 ) ? shamtB_Ihl
    : ( opB1_mux_sel_Ihl == 3'd2 ) ? instB_imm_u_Ihl
    : ( opB1_mux_sel_Ihl == 3'd3 ) ? instB_imm_sb_Ihl
    : ( opB1_mux_sel_Ihl == 3'd4 ) ? instB_imm_i_Ihl
    : ( opB1_mux_sel_Ihl == 3'd5 ) ? instB_imm_s_Ihl
    : ( opB1_mux_sel_Ihl == 3'd6 ) ? const0
    :                               32'bx;

  // wdata with bypassing

  wire [31:0] wdata_Ihl = opA1_byp_mux_out_Ihl;

  //----------------------------------------------------------------------
  // X0 <- I
  //----------------------------------------------------------------------

  reg [31:0] pcA_X0hl;
  reg [31:0] pcB_X0hl;
  reg [31:0] branch_targ_X0hl;
  reg [31:0] opA0_mux_out_X0hl;
  reg [31:0] opA1_mux_out_X0hl;
  reg [31:0] opB0_mux_out_X0hl;
  reg [31:0] opB1_mux_out_X0hl;
  reg [31:0] wdata_X0hl;

  always @ (posedge clk) begin
    if( !stall_X0hl ) begin
      pcA_X0hl          <= pcA_Ihl;
      pcB_X0hl          <= pcB_Ihl;
      branch_targ_X0hl  <= branch_targ_Ihl;
      opA0_mux_out_X0hl <= opA0_mux_out_Ihl;
      opA1_mux_out_X0hl <= opA1_mux_out_Ihl;
      opB0_mux_out_X0hl <= opB0_mux_out_Ihl;
      opB1_mux_out_X0hl <= opB1_mux_out_Ihl;
      wdata_X0hl        <= wdata_Ihl;
    end
  end

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  // ALU

  wire [31:0] aluA_out_X0hl;
  wire [31:0] aluB_out_X0hl;

  riscv_CoreDpathAlu aluA
  (
    .in0  (opA0_mux_out_X0hl),
    .in1  (opA1_mux_out_X0hl),
    .fn   (aluA_fn_X0hl),
    .out  (aluA_out_X0hl)
  );

  riscv_CoreDpathAlu aluB
  (
    .in0  (opB0_mux_out_X0hl),
    .in1  (opB1_mux_out_X0hl),
    .fn   (aluB_fn_X0hl),
    .out  (aluB_out_X0hl)
  );

  // Branch condition logic

  wire   diffSigns_X0hl         = opA0_mux_out_X0hl[31] ^ opA1_mux_out_X0hl[31];
  assign branch_cond_eq_X0hl    = ( aluA_out_X0hl == 32'd0 );
  assign branch_cond_ne_X0hl    = ~branch_cond_eq_X0hl;
  assign branch_cond_lt_X0hl    = diffSigns_X0hl ? opA0_mux_out_X0hl[31] : aluA_out_X0hl[31];
  assign branch_cond_ltu_X0hl   = diffSigns_X0hl ? opA1_mux_out_X0hl[31] : aluA_out_X0hl[31];
  assign branch_cond_ge_X0hl    = diffSigns_X0hl ? opA1_mux_out_X0hl[31] : ~aluA_out_X0hl[31];
  assign branch_cond_geu_X0hl   = diffSigns_X0hl ? opA0_mux_out_X0hl[31] : ~aluA_out_X0hl[31];

  // Send out memory request during X, response returns in M

  assign dmemreq_msg_addr = aluA_out_X0hl;
  assign dmemreq_msg_data = wdata_X0hl;

  // Muldiv Unit

  wire [63:0] muldivresp_msg_result_X3hl;

  riscv_CoreDpathPipeMulDiv imuldiv
  (
    .clk                   (clk),
    .reset                 (reset),
    .muldivreq_msg_fn      (muldivreq_msg_fn_Ihl),
    .muldivreq_msg_a       (opA0_mux_out_Ihl),
    .muldivreq_msg_b       (opA1_mux_out_Ihl),
    .muldivreq_val         (muldivreq_val),
    .muldivreq_rdy         (muldivreq_rdy),
    .muldivresp_msg_result (muldivresp_msg_result_X3hl),
    .muldivresp_val        (muldivresp_val),
    .muldivresp_rdy        (muldivresp_rdy),

    .stall_Xhl             (stall_X0hl),
    .stall_Mhl             (stall_X1hl),
    .stall_X2hl            (stall_X2hl),
    .stall_X3hl            (stall_X3hl)
  );


  //----------------------------------------------------------------------
  // X1 <- X0
  //----------------------------------------------------------------------

  reg  [31:0] pcA_X1hl;
  reg  [31:0] pcB_X1hl;
  reg  [31:0] executeA_mux_out_X1hl;
  reg  [31:0] wdata_X1hl;
  reg  [31:0] aluB_out_X1hl;

  always @ (posedge clk) begin
    if( !stall_X1hl ) begin
      pcA_X1hl              <= pcA_X0hl;
      executeA_mux_out_X1hl <= aluA_out_X0hl;
      wdata_X1hl            <= wdata_X0hl;
    end
  end

  //----------------------------------------------------------------------
  // X1 Stage
  //----------------------------------------------------------------------

  // Data memory subword adjustment mux

  wire [31:0] dmemresp_lb_X1hl
    = { {24{dmemresp_msg_data[7]}}, dmemresp_msg_data[7:0] };

  wire [31:0] dmemresp_lbu_X1hl
    = { {24{1'b0}}, dmemresp_msg_data[7:0] };

  wire [31:0] dmemresp_lh_X1hl
    = { {16{dmemresp_msg_data[15]}}, dmemresp_msg_data[15:0] };

  wire [31:0] dmemresp_lhu_X1hl
    = { {16{1'b0}}, dmemresp_msg_data[15:0] };

  wire [31:0] dmemresp_mux_out_X1hl
    = ( dmemresp_mux_sel_X1hl == 3'd0 ) ? dmemresp_msg_data
    : ( dmemresp_mux_sel_X1hl == 3'd1 ) ? dmemresp_lb_X1hl
    : ( dmemresp_mux_sel_X1hl == 3'd2 ) ? dmemresp_lbu_X1hl
    : ( dmemresp_mux_sel_X1hl == 3'd3 ) ? dmemresp_lh_X1hl
    : ( dmemresp_mux_sel_X1hl == 3'd4 ) ? dmemresp_lhu_X1hl
    :                                    32'bx;

  //----------------------------------------------------------------------
  // Queue for data memory response
  //----------------------------------------------------------------------

  reg [31:0] dmemresp_queue_reg_X1hl;

  always @ ( posedge clk ) begin
    if ( dmemresp_queue_en_X1hl ) begin
      dmemresp_queue_reg_X1hl <= dmemresp_mux_out_X1hl;
    end
  end

  //----------------------------------------------------------------------
  // Data memory queue mux
  //----------------------------------------------------------------------

  wire [31:0] dmemresp_queue_mux_out_X1hl
    = ( !dmemresp_queue_val_X1hl ) ? dmemresp_mux_out_X1hl
    : ( dmemresp_queue_val_X1hl )  ? dmemresp_queue_reg_X1hl
    :                               32'bx;

  //----------------------------------------------------------------------
  // Writeback mux
  //----------------------------------------------------------------------

  wire [31:0] memexA_mux_out_X1hl
    = ( memex_mux_sel_X1hl == 1'd0 ) ? executeA_mux_out_X1hl
    : ( memex_mux_sel_X1hl == 1'd1 ) ? dmemresp_queue_mux_out_X1hl
    :                              32'bx;

  //----------------------------------------------------------------------
  // X2 <- X1
  //----------------------------------------------------------------------

  reg  [31:0] pcA_X2hl;
  reg  [31:0] pcB_X2hl;
  reg  [31:0] memexA_mux_out_X2hl;
  reg  [31:0] aluB_out_X2hl;

  always @ (posedge clk) begin
    if( !stall_X2hl ) begin
      pcA_X2hl                 <= pcA_X1hl;
      memexA_mux_out_X2hl      <= memexA_mux_out_X1hl;
    end
  end

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------

  reg  [31:0] pcA_X3hl;
  reg  [31:0] pcB_X3hl;
  reg  [31:0] memexA_mux_out_X3hl;
  reg  [31:0] aluB_out_X3hl;

  always @ (posedge clk) begin
    if( !stall_X3hl ) begin
      pcA_X3hl                 <= pcA_X2hl;
      memexA_mux_out_X3hl      <= memexA_mux_out_X2hl;
    end
  end
 
  //----------------------------------------------------------------------
  // Stage X3
  //----------------------------------------------------------------------

  // Muldiv Result Mux

  wire [31:0] muldiv_mux_out_X3hl
    = ( muldiv_mux_sel_X3hl == 1'd0 ) ? muldivresp_msg_result_X3hl[31:0]
    : ( muldiv_mux_sel_X3hl == 1'd1 ) ? muldivresp_msg_result_X3hl[63:32]
    :                                  32'bx;

  // Execute Result Mux

  wire [31:0] executeA_mux_out_X3hl
    = ( execute_mux_sel_X3hl == 1'd0 ) ? memexA_mux_out_X3hl
    : ( execute_mux_sel_X3hl == 1'd1 ) ? muldiv_mux_out_X3hl
    :                                   32'bx;

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg  [31:0] pcA_Whl;
  reg  [31:0] pcB_Whl;
  reg  [31:0] wbA_mux_out_Whl;
  reg  [31:0] aluB_out_Whl;

  always @ (posedge clk) begin
    if( !stall_Whl ) beginpB0_byp_mux_sel_Ihl,
  output  [1:0] opB0_mux_sel_Ihl,
  output  [3:0] opB1_byp_mux_sel_Ihl,
  output  [2:0] opB1_mux_sel_Ihl,
  output [31:0] instA_Ihl,
  output [31:0] instB_Ihl,
  output  [3:0] aluA_fn_X0hl,
  output  [3:0] aluB_fn_X0hl,
  output  [2:0] muldivreq_msg_fn_Ihl,
  output        muldivreq_val,
  input         muldi
      pcA_Whl                 <= pcA_X3hl;
      wbA_mux_out_Whl         <= executeA_mux_out_X3hl;

      pcB_Whl                 <= pcB_X0hl;
      aluB_out_Whl            <= aluB_out_X0hl;
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  reg [31:0] ROB_data [31:0];

  always @ ( posedge clk ) begin
    if (ROB_fill_wen_A_Whl) begin
      ROB_data[ROB_commit_req_slot_A_Whl] <= wbA_mux_out_Whl;
    end
    if (ROB_fill_wen_B_Whl) begin
      ROB_data[ROB_commit_req_slot_B_Whl] <= aluB_out_Whl;
    end
  end

  // CSR write data

  assign proc2csr_data_Whl = wbA_mux_out_Whl;

  //----------------------------------------------------------------------
  // Commit Stage
  //----------------------------------------------------------------------

  wire [31:0] rf1_wdata_Chl = ROB_data[ROB_commit_slot_1_Chl];
  wire [ 4:0] rf1_waddr_Chl = ROB_commit_waddr_1_Chl;
  wire        rf1_wen_Chl   = ROB_commit_wen_1_Chl;

  wire [31:0] rf2_wdata_Chl = ROB_data[ROB_commit_slot_2_Chl];
  wire [ 4:0] rf2_waddr_Chl = ROB_commit_waddr_2_Chl;
  wire        rf2_wen_Chl   = ROB_commit_wen_2_Chl;

  riscv_CoreDpathRegfile rfile
  (
    .clk      (clk),
    .raddr0   (rfA_raddr0_Ihl),
    .rdata0   (rfA_rdata0_Ihl),
    .raddr1   (rfA_raddr1_Ihl),
    .rdata1   (rfA_rdata1_Ihl),
    .raddr2   (rfB_raddr0_Ihl),
    .rdata2   (rfB_rdata0_Ihl),
    .raddr3   (rfB_raddr1_Ihl),
    .rdata3   (rfB_rdata1_Ihl),

    .wen0_p   (rf1_wen_Chl),
    .waddr0_p (rf1_waddr_Chl),
    .wdata0_p (rf1_wdata_Chl),
    .wen1_p   (rf2_wen_Chl),
    .waddr1_p (rf2_waddr_Chl),
    .wdata1_p (rf2_wdata_Chl)
  );

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] pcA_debug;
  reg [31:0] pcB_debug;

  always @ ( posedge clk ) begin
    pcA_debug <= pcA_Whl;
    pcB_debug <= pcB_Whl;
  end

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
