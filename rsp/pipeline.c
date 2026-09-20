//
// rsp/pipeline.c: RSP processor pipeline.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "rsp/cp0.h"
#include "rsp/cp2.h"
#include "rsp/cpu.h"
#include "rsp/decoder.h"
#include "rsp/opcodes.h"
#include "rsp/pipeline.h"
#include <string.h>
#include "rsp/rsp.h"

// Prints out instructions and their address as they are executed.
//#define PRINT_EXEC

typedef void (*pipeline_function)(struct rsp *rsp);

// Instruction cache fetch stage.
static inline void rsp_if_stage(struct rsp *rsp) {
  struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;
  uint32_t pc = ifrd_latch->pc;
  uint32_t iw;

  assert(!(pc & 0x1000) || "RSP $PC points past IMEM.");
  ifrd_latch->pc = (pc + 4) & 0xFFC;

  memcpy(&iw, rsp->mem + 0x1000 + pc, sizeof(iw));

  ifrd_latch->common.pc = pc;
  ifrd_latch->opcode = rsp->opcode_cache[pc >> 2];
  ifrd_latch->iw = iw;
}

// Register fetch and decode stage.
static inline int rsp_rd_stage(struct rsp *rsp) {
  struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;
  struct rsp_ifrd_latch *ifrd_latch = &rsp->pipeline.ifrd_latch;

  uint32_t previous_insn_flags = rdex_latch->opcode.flags;
  uint32_t iw = ifrd_latch->iw;

  rdex_latch->common = ifrd_latch->common;
  rdex_latch->opcode = ifrd_latch->opcode;
  rdex_latch->iw = iw;

  // Check for load-use stalls.
  if (previous_insn_flags & OPCODE_INFO_LOAD) {
    const struct rsp_opcode *opcode = &rdex_latch->opcode;
    unsigned dest = rsp->pipeline.exdf_latch.result.dest;
    unsigned rs = GET_RS(iw);
    unsigned rt = GET_RT(iw);

    if (unlikely(dest && (
      (dest == rs && (opcode->flags & OPCODE_INFO_NEEDRS)) ||
      (dest == rt && (opcode->flags & OPCODE_INFO_NEEDRT))
    ))) {
      static const struct rsp_opcode rsp_rf_kill_op = {RSP_OPCODE_SLL, 0x0};

      rdex_latch->opcode = rsp_rf_kill_op;
      rdex_latch->iw = 0x00000000U;
      rsp->hw.bubble = true;

      return 1;
    }
  }

  return 0;
}

// Execution stage.
cen64_flatten static inline void rsp_ex_stage(struct rsp *rsp) {
  struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;
  struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
  struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;

  uint32_t rs_reg, rt_reg, temp;
  unsigned rs, rt;
  uint32_t iw;

  exdf_latch->common = rdex_latch->common;

  if (rdex_latch->opcode.flags & OPCODE_INFO_VECTOR)
    return;

  iw = rdex_latch->iw;
  rs = GET_RS(iw);
  rt = GET_RT(iw);

  // Forward results from DF/WB.
  temp = rsp->regs[dfwb_latch->result.dest];
  rsp->regs[dfwb_latch->result.dest] = dfwb_latch->result.result;
  rsp->regs[RSP_REGISTER_R0] = 0x00000000U;

  rs_reg = rsp->regs[rs];
  rt_reg = rsp->regs[rt];

  rsp->regs[dfwb_latch->result.dest] = temp;

  // Finally, execute the instruction.
#ifdef PRINT_EXEC
  debug("%.8X: %s\n", rdex_latch->common.pc,
    rsp_opcode_mnemonics[rdex_latch->opcode.id]);
#endif

  return rsp_function_table[rdex_latch->opcode.id](
    rsp, iw, rs_reg, rt_reg);
}

// Execution stage (vector).
cen64_flatten static inline void rsp_v_ex_stage(struct rsp *rsp) {
  struct rsp_rdex_latch *rdex_latch = &rsp->pipeline.rdex_latch;

  rsp_vect_t vd_reg, vs_reg, vt_shuf_reg, zero;

  unsigned vs, vt, vd, e;
  uint32_t iw;

  if (!(rdex_latch->opcode.flags & OPCODE_INFO_VECTOR))
    return;

  iw = rdex_latch->iw;
  vs = GET_VS(iw);
  vt = GET_VT(iw);
  vd = GET_VD(iw);
  e  = GET_E (iw);

  vs_reg = rsp_vect_load_unshuffled_operand(rsp->cp2.regs[vs].e);
  vt_shuf_reg = rsp_vect_load_and_shuffle_operand(rsp->cp2.regs[vt].e, e);
  zero = rsp_vzero();

  // Finally, execute the instruction.
#ifdef PRINT_EXEC
  debug("%.8X: %s\n", rdex_latch->common.pc,
    rsp_vector_opcode_mnemonics[rdex_latch->opcode.id]);
#endif

  vd_reg = rsp_vector_function_table[rdex_latch->opcode.id](
    rsp, iw, vt_shuf_reg, vs_reg, zero);

  rsp_vect_write_operand(rsp->cp2.regs[vd].e, vd_reg);
}

// Data cache fetch stage.
cen64_flatten static inline void rsp_df_stage(struct rsp *rsp) {
  struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;
  struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
  const struct rsp_mem_request *request = &exdf_latch->request;
  uint32_t addr;

  dfwb_latch->common = exdf_latch->common;
  dfwb_latch->result = exdf_latch->result;

  if (request->type == RSP_MEM_REQUEST_NONE)
    return;

  addr = request->addr & 0xFFF;

  // Scalar unit DMEM access.
  if (request->type == RSP_MEM_REQUEST_INT_MEM) {
    uint32_t rdqm = request->packet.p_int.rdqm;
    uint32_t wdqm = request->packet.p_int.wdqm;
    uint32_t data = request->packet.p_int.data;
    unsigned rshift = request->packet.p_int.rshift;
    uint32_t word;

    memcpy(&word, rsp->mem + addr, sizeof(word));

    word = byteswap_32(word);
    dfwb_latch->result.result = rdqm & (((int32_t) word) >> rshift);
    word = byteswap_32((word & ~wdqm) | (data & wdqm));

    memcpy(rsp->mem + addr, &word, sizeof(word));
  }
  // Transposed vector unit DMEM access.
  else if (request->type == RSP_MEM_REQUEST_TRANSPOSE) {
    unsigned element = request->packet.p_transpose.element;
    unsigned vt = request->packet.p_transpose.vt;

    exdf_latch->request.packet.p_transpose.transpose_func(
      rsp, addr, element, vt);
  }
  // Vector unit DMEM access.
  else {
    uint16_t *regp = rsp->cp2.regs[request->packet.p_vect.dest].e;
    unsigned element = request->packet.p_vect.element;
    rsp_vect_t reg, dqm;

    reg = rsp_vect_load_unshuffled_operand(regp);
    dqm = rsp_vect_load_unshuffled_operand(exdf_latch->
      request.packet.p_vect.vdqm.e);

    // Make sure the vector data doesn't get
    // written into the scalar part of the RF.
    dfwb_latch->result.dest = 0;

    exdf_latch->request.packet.p_vect.vldst_func(
      rsp, addr, element, regp, reg, dqm);
  }

}

// Writeback stage.
static inline bool rsp_wb_stage(struct rsp *rsp) {
  const struct rsp_dfwb_latch *dfwb_latch = &rsp->pipeline.dfwb_latch;

  if (dfwb_latch->result.dest == RSP_CP0_REGISTER_SP_STATUS) {
    rsp_status_write(rsp, dfwb_latch->result.result);
    return !(rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT);
  } else
    rsp->regs[dfwb_latch->result.dest] = dfwb_latch->result.result;
  return true;
}

// Advances the processor pipeline by one clock.
void rsp_cycle_(struct rsp *rsp) {
  if (unlikely(!rsp_wb_stage(rsp)))
    return;
  if (!rsp->hw.enabled)
    rsp_prof_add(rsp, rsp->pipeline.rdex_latch.common.pc, 1);
  rsp_df_stage(rsp);

  rsp->pipeline.exdf_latch.result.dest = RSP_REGISTER_R0;
  rsp->pipeline.exdf_latch.request.type = RSP_MEM_REQUEST_NONE;

  rsp_v_ex_stage(rsp);
  rsp_ex_stage(rsp);

  if (likely(!rsp_rd_stage(rsp)))
    rsp_if_stage(rsp);
}

// Initializes the pipeline with default values.
void rsp_pipeline_init(struct rsp_pipeline *pipeline) {
  memset(pipeline, 0, sizeof(*pipeline));
}



// ---------------------------------------------------------------------------
// -rsphw: RSP issue timing (dual issue and register latencies), see cpu.h.
// The functional pipeline above still executes one instruction per call; this
// layer decides how many device cycles each instruction is worth on the real
// chip and executes it only when that many credits have accumulated (a
// dual-issued partner is free and runs in the same device cycle).
// ---------------------------------------------------------------------------
#include <stdio.h>

#define HW_LAT_VU   4  // vector register write -> readable
#define HW_LAT_LOAD 3  // DMEM load / mfc0 / mfc2 / cfc2 -> readable

struct rsp_hw_insn {
  bool vector, branch, load, store, copmove, nop;
  uint32_t vread, vwrite;   // vector register masks
  uint32_t sread;           // scalar registers read
  int swrite;               // scalar register written with load latency (-1: none)
};

static uint32_t hw_vmask8(unsigned vt) {
  unsigned base = vt & ~7u;
  return 0xFFu << base;
}

static void rsp_hw_classify(const struct rsp_opcode *op, uint32_t iw, struct rsp_hw_insn *d) {
  memset(d, 0, sizeof(*d));
  d->swrite = -1;
  if (op->flags & OPCODE_INFO_VECTOR) {
    d->vector = true;
    if (op->id == RSP_OPCODE_VNOP || op->id == RSP_OPCODE_VNULL || op->id == RSP_OPCODE_VINVALID) {
      d->nop = true;
      return;
    }
    if (op->flags & OPCODE_INFO_NEEDVS) d->vread |= 1u << GET_VS(iw);
    if (op->flags & OPCODE_INFO_NEEDVT) d->vread |= 1u << GET_VT(iw);
    d->vwrite |= 1u << GET_VD(iw);
    return;
  }
  switch (op->id) {
    case RSP_OPCODE_BEQ: case RSP_OPCODE_BNE: case RSP_OPCODE_BLEZ: case RSP_OPCODE_BGTZ:
    case RSP_OPCODE_BLTZ: case RSP_OPCODE_BGEZ: case RSP_OPCODE_BLTZAL: case RSP_OPCODE_BGEZAL:
    case RSP_OPCODE_J: case RSP_OPCODE_JAL: case RSP_OPCODE_JR: case RSP_OPCODE_JALR:
      d->branch = true;
      goto generic_operands;
    case RSP_OPCODE_LB: case RSP_OPCODE_LBU: case RSP_OPCODE_LH: case RSP_OPCODE_LHU: case RSP_OPCODE_LW:
      d->load = true; d->swrite = GET_RT(iw); d->sread |= 1u << GET_RS(iw);
      break;
    case RSP_OPCODE_SB: case RSP_OPCODE_SH: case RSP_OPCODE_SW:
      d->store = true; d->sread |= (1u << GET_RS(iw)) | (1u << GET_RT(iw));
      break;
    case RSP_OPCODE_LBV: case RSP_OPCODE_LSV: case RSP_OPCODE_LLV: case RSP_OPCODE_LDV:
    case RSP_OPCODE_LQV: case RSP_OPCODE_LRV: case RSP_OPCODE_LPV: case RSP_OPCODE_LUV:
    case RSP_OPCODE_LHV: case RSP_OPCODE_LFV:
      d->load = true; d->vwrite |= 1u << GET_VT(iw); d->sread |= 1u << GET_RS(iw);
      break;
    case RSP_OPCODE_LTV:
      d->load = true; d->vwrite |= hw_vmask8(GET_VT(iw)); d->sread |= 1u << GET_RS(iw);
      break;
    case RSP_OPCODE_SBV: case RSP_OPCODE_SSV: case RSP_OPCODE_SLV: case RSP_OPCODE_SDV:
    case RSP_OPCODE_SQV: case RSP_OPCODE_SRV: case RSP_OPCODE_SPV: case RSP_OPCODE_SUV:
    case RSP_OPCODE_SHV: case RSP_OPCODE_SFV: case RSP_OPCODE_SWV:
      d->store = true; d->vread |= 1u << GET_VT(iw); d->sread |= 1u << GET_RS(iw);
      break;
    case RSP_OPCODE_STV:
      d->store = true; d->vread |= hw_vmask8(GET_VT(iw)); d->sread |= 1u << GET_RS(iw);
      break;
    case RSP_OPCODE_MTC2:
      d->copmove = true; d->vwrite |= 1u << GET_RD(iw); d->sread |= 1u << GET_RT(iw);
      break;
    case RSP_OPCODE_MFC2:
      d->copmove = true; d->vread |= 1u << GET_RD(iw); d->swrite = GET_RT(iw);
      break;
    case RSP_OPCODE_CFC2: case RSP_OPCODE_MFC0:
      d->copmove = true; d->swrite = GET_RT(iw);
      break;
    case RSP_OPCODE_CTC2: case RSP_OPCODE_MTC0:
      d->copmove = true; d->sread |= 1u << GET_RT(iw);
      break;
    case RSP_OPCODE_NOP: case RSP_OPCODE_INVALID:
      d->nop = (iw == 0);
      break;
    default:
    generic_operands:
      if (op->flags & OPCODE_INFO_NEEDRS) d->sread |= 1u << GET_RS(iw);
      if (op->flags & OPCODE_INFO_NEEDRT) d->sread |= 1u << GET_RT(iw);
      break;
  }
  d->sread &= ~1u; // $zero
}

// Cost in device cycles of the instruction that the next rsp_cycle_() call
// executes (the one in the RD/EX latch); records the decision in rsp->hw.pend_*.
static int rsp_hw_cost(struct rsp *rsp) {
  struct rsp_hwtiming *hw = &rsp->hw;
  const struct rsp_rdex_latch *rdex = &rsp->pipeline.rdex_latch;
  struct rsp_hw_insn d;
  int64_t t, tready;
  unsigned r;

  hw->pend_pair = false;
  hw->pend_bubble_only = false;
  if (hw->bubble) {
    // cen64's own load-use bubble: the real stall is charged to the consumer below.
    hw->pend_bubble_only = true;
    hw->pend_cost = 0;
    return 0;
  }
  rsp_hw_classify(&rdex->opcode, rdex->iw, &d);

  // Earliest cycle at which the operands are ready.
  tready = 0;
  for (r = 0; r < 32; r++) {
    if ((d.sread >> r) & 1) { if (hw->sready[r] > tready) tready = hw->sready[r]; }
    if ((d.vread >> r) & 1) { if (hw->vready[r] > tready) tready = hw->vready[r]; }
  }

  // Dual issue with the previous instruction?
  if (hw->prev_valid && !hw->prev_paired && !hw->prev_branch && !hw->prev_delay
      && hw->prev_vector != d.vector && tready <= hw->prev_issue
      && !(hw->prev_vwrite & (d.vread | d.vwrite))
      && !(hw->prev_target && (hw->prev_pc & 4))) {
    bool ok = true;
    if ((d.store || d.copmove) && hw->prev_issue == hw->last_load + 2) ok = false;
    if (ok) {
      hw->pend_pair = true;
      hw->pend_issue = hw->prev_issue;
      hw->pend_cost = 0;
      return 0;
    }
  }

  t = hw->clock + hw->pending_bubble;
  if (tready > t) t = tready;
  if ((d.store || d.copmove) && t == hw->last_load + 2) t++;
  hw->pend_issue = t;
  hw->pend_cost = (int) (t + 1 - hw->clock);
  return hw->pend_cost;
}

static void rsp_hw_commit(struct rsp *rsp) {
  struct rsp_hwtiming *hw = &rsp->hw;
  const struct rsp_rdex_latch *rdex = &rsp->pipeline.rdex_latch;
  struct rsp_hw_insn d;
  int64_t t = hw->pend_issue;
  bool delay = hw->prev_valid && hw->prev_branch;
  bool target = hw->prev_valid && hw->prev_delay && rdex->common.pc != ((hw->prev_pc + 4) & 0xFFC);
  unsigned r;

  if (hw->pend_bubble_only) {
    hw->bubble = false;
    return;
  }
  rsp_hw_classify(&rdex->opcode, rdex->iw, &d);
  hw->n_insn++;
  if (hw->pend_pair) {
    hw->n_pair++;
  } else {
    hw->n_stall += (uint64_t) (t - hw->clock);
    hw->clock = t + 1;
    hw->pending_bubble = 0;
  }
  if (delay) {
    hw->pending_bubble = 1; // the branch's third cycle
    hw->n_branch++;
  }
  for (r = 0; r < 32; r++) {
    if ((d.vwrite >> r) & 1) hw->vready[r] = t + HW_LAT_VU;
  }
  if (d.swrite > 0) hw->sready[d.swrite] = t + HW_LAT_LOAD;
  if (d.load || d.copmove) hw->last_load = t;

  hw->prev_valid = true;
  hw->prev_vector = d.vector;
  hw->prev_branch = d.branch;
  hw->prev_delay = delay;
  hw->prev_paired = hw->pend_pair;
  hw->prev_target = target;
  hw->prev_issue = t;
  hw->prev_pc = rdex->common.pc;
  hw->prev_vwrite = d.vwrite;
}

void rsp_cycle_hw(struct rsp *rsp) {
  struct rsp_hwtiming *hw = &rsp->hw;
  hw->credit++;
  for (;;) {
    int cost;
    if (unlikely(rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT))
      return;
    cost = rsp_hw_cost(rsp);
    if (cost > hw->credit)
      return;
    hw->credit -= cost;
    rsp_prof_add(rsp, rsp->pipeline.rdex_latch.common.pc, cost);
    rsp_hw_commit(rsp);
    rsp_cycle_(rsp);
    // Loop on: a dual-issued partner or a load-use bubble costs 0 and runs in
    // this same device cycle; anything else costs at least 1 and waits.
  }
}

void rsp_hw_print_stats(const struct rsp *rsp) {
  const struct rsp_hwtiming *hw = &rsp->hw;
  fprintf(stderr, "RSP hw timing: %llu instructions, %llu dual-issued (%.1f%%), %llu stall cycles, %llu branches, %llu cycles\n",
          (unsigned long long) hw->n_insn, (unsigned long long) hw->n_pair,
          hw->n_insn ? 100.0 * hw->n_pair / hw->n_insn : 0.0,
          (unsigned long long) hw->n_stall, (unsigned long long) hw->n_branch,
          (unsigned long long) hw->clock);
}
