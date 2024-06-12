//=========================================================================
// 7-Stage RISCV Core
//=========================================================================

`ifndef RISCV_CORE_V
`define RISCV_CORE_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "riscvi2oi-CoreCtrl.v"
`include "riscvi2oi-CoreDpath.v"

module riscv_Core
(
  input         clk,
  input         reset,

  // Instruction Memory Request Port
  output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq0_msg,
  output                                 imemreq0_val,
  input                                  imemreq0_rdy,

  // Instruction Memory Response Port
  input [`VC_MEM_RESP_MSG_SZ(32)-1:0] imemresp0_msg,
  input                               imemresp0_val,

  // Instruction Memory Request Port
  output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq1_msg,
  output                                 imemreq1_val,
  input                                  imemreq1_rdy,

  // Instruction Memory Response Port
  input [`VC_MEM_RESP_MSG_SZ(32)-1:0] imemresp1_msg,
  input                               imemresp1_val,

  // Data Memory Request Port
  output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] dmemreq_msg,
  output                                 dmemreq_val,
  input                                  dmemreq_rdy,

  // Data Memory Response Port
  input [`VC_MEM_RESP_MSG_SZ(32)-1:0] dmemresp_msg,
  input                               dmemresp_val,

  // CSR Status Register Output to Host

  output [31:0] csr_status
);

  wire [31:0] imemreq0_msg_addr;
  wire [31:0] imemresp0_msg_data;

  wire [31:0] imemreq1_msg_addr;
  wire [31:0] imemresp1_msg_data;

  wire        dmemreq_msg_rw;
  wire  [1:0] dmemreq_msg_len;
  wire [31:0] dmemreq_msg_addr;
  wire [31:0] dmemreq_msg_data;
  wire [31:0] dmemresp_msg_data;

  wire  [1:0] pc_mux_sel_Phl;
  wire        steering_mux_sel_Ihl;
  wire  [3:0] opA0_byp_mux_sel_Ihl;
  wire  [1:0] opA0_mux_sel_Ihl;
  wire  [3:0] opA1_byp_mux_sel_Ihl;
  wire  [2:0] opA1_mux_sel_Ihl;
  wire  [3:0] opB0_byp_mux_sel_Ihl;
  wire  [1:0] opB0_mux_sel_Ihl;
  wire  [3:0] opB1_byp_mux_sel_Ihl;
  wire  [2:0] opB1_mux_sel_Ihl;
  wire [31:0] instA_Ihl;
  wire [31:0] instB_Ihl;
  wire  [3:0] aluA_fn_X0hl;
  wire  [3:0] aluB_fn_X0hl;
  wire  [2:0] muldivreq_msg_fn_Ihl;
  wire        muldivreq_val;
  wire        muldivreq_rdy;
  wire        muldivresp_val;
  wire        muldivresp_rdy;
  wire  [2:0] dmemresp_mux_sel_X1hl;
  wire        dmemresp_queue_en_X1hl;
  wire        dmemresp_queue_val_X1hl;
  wire        muldiv_mux_sel_X3hl;
  wire        execute_mux_sel_X3hl;
  wire        memex_mux_sel_X1hl;
  wire        stall_Fhl;
  wire        stall_Dhl;
  wire        stall_Ihl;
  wire        stall_X0hl;
  wire        stall_X1hl;
  wire        stall_X2hl;
  wire        stall_X3hl;
  wire        stall_Whl;

  wire        branch_cond_eq_X0hl;
  wire        branch_cond_ne_X0hl;
  wire        branch_cond_lt_X0hl;
  wire        branch_cond_ltu_X0hl;
  wire        branch_cond_ge_X0hl;
  wire        branch_cond_geu_X0hl;
  wire [31:0] proc2csr_data_Whl;

  wire [ 4:0] opA0_byp_ROB_slot_Ihl;
  wire [ 4:0] opA1_byp_ROB_slot_Ihl;
  wire [ 4:0] opB0_byp_ROB_slot_Ihl;
  wire [ 4:0] opB1_byp_ROB_slot_Ihl;

  wire        ROB_fill_wen_A_Whl;
  wire [ 4:0] ROB_commit_req_slot_A_Whl;
  wire        ROB_fill_wen_B_Whl;
  wire [ 4:0] ROB_commit_req_slot_B_Whl;

  wire        ROB_commit_wen_1_Chl;
  wire [ 4:0] ROB_commit_slot_1_Chl;
  wire [ 4:0] ROB_commit_waddr_1_Chl;
  wire        ROB_commit_wen_2_Chl;
  wire [ 4:0] ROB_commit_slot_2_Chl;
  wire [ 4:0] ROB_commit_waddr_2_Chl;

  //----------------------------------------------------------------------
  // Pack Memory Request Messages
  //----------------------------------------------------------------------

  vc_MemReqMsgToBits#(32,32) imemreq0_msg_to_bits
  (
    .type (`VC_MEM_REQ_MSG_TYPE_READ),
    .addr (imemreq0_msg_addr),
    .len  (2'd0),
    .data (32'bx),
    .bits (imemreq0_msg)
  );

  vc_MemReqMsgToBits#(32,32) imemreq1_msg_to_bits
  (
    .type (`VC_MEM_REQ_MSG_TYPE_READ),
    .addr (imemreq1_msg_addr),
    .len  (2'd0),
    .data (32'bx),
    .bits (imemreq1_msg)
  );

  vc_MemReqMsgToBits#(32,32) dmemreq_msg_to_bits
  (
    .type (dmemreq_msg_rw),
    .addr (dmemreq_msg_addr),
    .len  (dmemreq_msg_len),
    .data (dmemreq_msg_data),
    .bits (dmemreq_msg)
  );

  //----------------------------------------------------------------------
  // Unpack Memory Response Messages
  //----------------------------------------------------------------------

  vc_MemRespMsgFromBits#(32) imemresp0_msg_from_bits
  (
    .bits (imemresp0_msg),
    .type (),
    .len  (),
    .data (imemresp0_msg_data)
  );

  vc_MemRespMsgFromBits#(32) imemresp1_msg_from_bits
  (
    .bits (imemresp1_msg),
    .type (),
    .len  (),
    .data (imemresp1_msg_data)
  );

  vc_MemRespMsgFromBits#(32) dmemresp_msg_from_bits
  (
    .bits (dmemresp_msg),
    .type (),
    .len  (),
    .data (dmemresp_msg_data)
  );

  //----------------------------------------------------------------------
  // Control Unit
  //----------------------------------------------------------------------

  riscv_CoreCtrl ctrl
  (
    .clk                    (clk),
    .reset                  (reset),

    // Instruction Memory Port
    .imemreq0_val            (imemreq0_val),
    .imemreq0_rdy            (imemreq0_rdy),
    .imemresp0_msg_data      (imemresp0_msg_data),
    .imemresp0_val           (imemresp0_val),

    // Instruction Memory Port
    .imemreq1_val            (imemreq1_val),
    .imemreq1_rdy            (imemreq1_rdy),
    .imemresp1_msg_data      (imemresp1_msg_data),
    .imemresp1_val           (imemresp1_val),

    // Data Memory Port
    .dmemreq_msg_rw         (dmemreq_msg_rw),
    .dmemreq_msg_len        (dmemreq_msg_len),
    .dmemreq_val            (dmemreq_val),
    .dmemreq_rdy            (dmemreq_rdy),
    .dmemresp_val           (dmemresp_val),

    // Controls Signals (ctrl->dpath)
    
    .pc_mux_sel_Phl          (pc_mux_sel_Phl),
    .steering_mux_sel_Ihl    (steering_mux_sel_Ihl),
    .opA0_byp_mux_sel_Ihl    (opA0_byp_mux_sel_Ihl),
    .opA0_mux_sel_Ihl        (opA0_mux_sel_Ihl),
    .opA1_byp_mux_sel_Ihl    (opA1_byp_mux_sel_Ihl),
    .opA1_mux_sel_Ihl        (opA1_mux_sel_Ihl),
    .opB0_byp_mux_sel_Ihl    (opB0_byp_mux_sel_Ihl),
    .opB0_mux_sel_Ihl        (opB0_mux_sel_Ihl),
    .opB1_byp_mux_sel_Ihl    (opB1_byp_mux_sel_Ihl),
    .opB1_mux_sel_Ihl        (opB1_mux_sel_Ihl),
    .instA_Ihl               (instA_Ihl),
    .instB_Ihl               (instB_Ihl),
    .aluA_fn_X0hl            (aluA_fn_X0hl),
    .aluB_fn_X0hl            (aluB_fn_X0hl),
    .muldivreq_msg_fn_Ihl    (muldivreq_msg_fn_Ihl),
    .muldivreq_val           (muldivreq_val),
    .muldivreq_rdy           (muldivreq_rdy),
    .muldivresp_val          (muldivresp_val),
    .muldivresp_rdy          (muldivresp_rdy),
    .dmemresp_mux_sel_X1hl   (dmemresp_mux_sel_X1hl),
    .dmemresp_queue_en_X1hl  (dmemresp_queue_en_X1hl),
    .dmemresp_queue_val_X1hl (dmemresp_queue_val_X1hl),
    .muldiv_mux_sel_X3hl     (muldiv_mux_sel_X3hl),
    .execute_mux_sel_X3hl    (execute_mux_sel_X3hl),
    .memex_mux_sel_X1hl      (memex_mux_sel_X1hl),
    .stall_Fhl               (stall_Fhl),
    .stall_Dhl               (stall_Dhl),
    .stall_Ihl               (stall_Ihl),
    .stall_X0hl              (stall_X0hl),
    .stall_X1hl              (stall_X1hl),
    .stall_X2hl              (stall_X2hl),
    .stall_X3hl              (stall_X3hl),
    .stall_Whl               (stall_Whl),

    // Control Signals (dpath->ctrl)

    .branch_cond_eq_X0hl	  (branch_cond_eq_X0hl),
    .branch_cond_ne_X0hl	  (branch_cond_ne_X0hl),
    .branch_cond_lt_X0hl	  (branch_cond_lt_X0hl),
    .branch_cond_ltu_X0hl	  (branch_cond_ltu_X0hl),
    .branch_cond_ge_X0hl	  (branch_cond_ge_X0hl),
    .branch_cond_geu_X0hl	  (branch_cond_geu_X0hl),
    .proc2csr_data_Whl      (proc2csr_data_Whl),

    // Reorder Buffer Signals (ctrl->dpath)
    .opA0_byp_ROB_slot_Ihl  (opA0_byp_ROB_slot_Ihl),
    .opA1_byp_ROB_slot_Ihl  (opA1_byp_ROB_slot_Ihl),
    .opB0_byp_ROB_slot_Ihl  (opB0_byp_ROB_slot_Ihl),
    .opB1_byp_ROB_slot_Ihl  (opB1_byp_ROB_slot_Ihl),

    .ROB_fill_wen_A_Whl     (ROB_fill_wen_A_Whl),
    .ROB_commit_req_slot_A_Whl    (ROB_commit_req_slot_A_Whl),
    .ROB_fill_wen_B_Whl     (ROB_fill_wen_B_Whl),
    .ROB_commit_req_slot_B_Whl    (ROB_commit_req_slot_B_Whl),

    .ROB_commit_wen_1_Chl   (ROB_commit_wen_1_Chl),
    .ROB_commit_slot_1_Chl  (ROB_commit_slot_1_Chl),
    .ROB_commit_waddr_1_Chl (ROB_commit_waddr_1_Chl),
    .ROB_commit_wen_2_Chl   (ROB_commit_wen_2_Chl),
    .ROB_commit_slot_2_Chl  (ROB_commit_slot_2_Chl),
    .ROB_commit_waddr_2_Chl (ROB_commit_waddr_2_Chl),

    // CSR Status

    .csr_status             (csr_status)
  );

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  riscv_CoreDpath dpath
  (
    .clk                     (clk),
    .reset                   (reset),

    // Instruction Memory Port

    .imemreq0_msg_addr       (imemreq0_msg_addr),
    .imemreq1_msg_addr       (imemreq1_msg_addr),

    // Data Memory Port

    .dmemreq_msg_addr        (dmemreq_msg_addr),
    .dmemreq_msg_data        (dmemreq_msg_data),
    .dmemresp_msg_data       (dmemresp_msg_data),

    // Controls Signals (ctrl->dpath)

    .pc_mux_sel_Phl           (pc_mux_sel_Phl),
    .steering_mux_sel_Ihl     (steering_mux_sel_Ihl),
    .opA0_byp_mux_sel_Ihl     (opA0_byp_mux_sel_Ihl),
    .opA0_mux_sel_Ihl         (opA0_mux_sel_Ihl),
    .opA1_byp_mux_sel_Ihl     (opA1_byp_mux_sel_Ihl),
    .opA1_mux_sel_Ihl         (opA1_mux_sel_Ihl),
    .opB0_byp_mux_sel_Ihl     (opB0_byp_mux_sel_Ihl),
    .opB0_mux_sel_Ihl         (opB0_mux_sel_Ihl),
    .opB1_byp_mux_sel_Ihl     (opB1_byp_mux_sel_Ihl),
    .opB1_mux_sel_Ihl         (opB1_mux_sel_Ihl),
    .instA_Ihl                (instA_Ihl),
    .instB_Ihl                (instB_Ihl),
    .aluA_fn_X0hl             (aluA_fn_X0hl),
    .aluB_fn_X0hl             (aluB_fn_X0hl),
    .muldivreq_msg_fn_Ihl     (muldivreq_msg_fn_Ihl),
    .muldivreq_val            (muldivreq_val),
    .muldivreq_rdy            (muldivreq_rdy),
    .muldivresp_val           (muldivresp_val),
    .muldivresp_rdy           (muldivresp_rdy),
    .dmemresp_mux_sel_X1hl    (dmemresp_mux_sel_X1hl),
    .dmemresp_queue_en_X1hl   (dmemresp_queue_en_X1hl),
    .dmemresp_queue_val_X1hl  (dmemresp_queue_val_X1hl),
    .muldiv_mux_sel_X3hl      (muldiv_mux_sel_X3hl),
    .execute_mux_sel_X3hl     (execute_mux_sel_X3hl),
    .memex_mux_sel_X1hl       (memex_mux_sel_X1hl),
    .stall_Fhl                (stall_Fhl),
    .stall_Dhl                (stall_Dhl),
    .stall_Ihl                (stall_Ihl),
    
    .stall_X0hl               (stall_X0hl),
    .stall_X1hl               (stall_X1hl),
    .stall_X2hl               (stall_X2hl),
    .stall_X3hl               (stall_X3hl),
    .stall_Whl                (stall_Whl),

    // Control Signals (dpath->ctrl)

    .branch_cond_eq_X0hl	   (branch_cond_eq_X0hl),
    .branch_cond_ne_X0hl	   (branch_cond_ne_X0hl),
    .branch_cond_lt_X0hl	   (branch_cond_lt_X0hl),
    .branch_cond_ltu_X0hl	   (branch_cond_ltu_X0hl),
    .branch_cond_ge_X0hl	   (branch_cond_ge_X0hl),
    .branch_cond_geu_X0hl	   (branch_cond_geu_X0hl),
    .proc2csr_data_Whl       (proc2csr_data_Whl),

    // Reorder Buffer Signals (ctrl->dpath)
    .opA0_byp_ROB_slot_Ihl  (opA0_byp_ROB_slot_Ihl),
    .opA1_byp_ROB_slot_Ihl  (opA1_byp_ROB_slot_Ihl),
    .opB0_byp_ROB_slot_Ihl  (opB0_byp_ROB_slot_Ihl),
    .opB1_byp_ROB_slot_Ihl  (opB1_byp_ROB_slot_Ihl),
    
    .ROB_fill_wen_A_Whl     (ROB_fill_wen_A_Whl),
    .ROB_commit_req_slot_A_Whl    (ROB_commit_req_slot_A_Whl),
    .ROB_fill_wen_B_Whl     (ROB_fill_wen_B_Whl),
    .ROB_commit_req_slot_B_Whl    (ROB_commit_req_slot_B_Whl),

    .ROB_commit_wen_1_Chl   (ROB_commit_wen_1_Chl),
    .ROB_commit_slot_1_Chl  (ROB_commit_slot_1_Chl),
    .ROB_commit_waddr_1_Chl (ROB_commit_waddr_1_Chl),
    .ROB_commit_wen_2_Chl   (ROB_commit_wen_2_Chl),
    .ROB_commit_slot_2_Chl  (ROB_commit_slot_2_Chl),
    .ROB_commit_waddr_2_Chl (ROB_commit_waddr_2_Chl)
  );

endmodule

`endif

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
