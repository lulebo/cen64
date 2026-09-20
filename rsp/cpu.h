//
// rsp/cpu.h: RSP processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef __rsp_cpu_h__
#define __rsp_cpu_h__
#include "common.h"
#include "os/dynarec.h"
#include "rsp/cp0.h"
#include "rsp/cp2.h"
#include "rsp/pipeline.h"

enum rsp_register {
  RSP_REGISTER_R0, RSP_REGISTER_AT, RSP_REGISTER_V0,
  RSP_REGISTER_V1, RSP_REGISTER_A0, RSP_REGISTER_A1,
  RSP_REGISTER_A2, RSP_REGISTER_A3, RSP_REGISTER_T0,
  RSP_REGISTER_T1, RSP_REGISTER_T2, RSP_REGISTER_T3,
  RSP_REGISTER_T4, RSP_REGISTER_R5, RSP_REGISTER_T6,
  RSP_REGISTER_T7, RSP_REGISTER_S0, RSP_REGISTER_S1,
  RSP_REGISTER_S2, RSP_REGISTER_S3, RSP_REGISTER_S4,
  RSP_REGISTER_S5, RSP_REGISTER_S6, RSP_REGISTER_S7,
  RSP_REGISTER_T8, RSP_REGISTER_T9, RSP_REGISTER_K0,
  RSP_REGISTER_K1, RSP_REGISTER_GP, RSP_REGISTER_SP,
  RSP_REGISTER_FP, RSP_REGISTER_RA,

  // CP0 registers.
  RSP_REGISTER_CP0_0, RSP_REGISTER_CP0_1, RSP_REGISTER_CP0_2,
  RSP_REGISTER_CP0_3, RSP_REGISTER_CP0_4, RSP_REGISTER_CP0_5,
  RSP_REGISTER_CP0_6, RSP_REGISTER_CP0_7,

  // Miscellanious registers.
  NUM_RSP_REGISTERS
};

enum sp_register {
#define X(reg) reg,
#include "rsp/registers.md"
#undef X
  NUM_SP_REGISTERS,
  SP_REGISTER_OFFSET = RSP_REGISTER_CP0_0
};

#ifdef DEBUG_MMIO_REGISTER_ACCESS
extern const char *sp_register_mnemonics[NUM_SP_REGISTERS];
#endif

// -rsphw: issue timing model (dual issue, register latencies), see pipeline.c.
struct rsp_hwtiming {
  bool enabled;
  bool bubble;              // the RD stage inserted a load-use bubble (free in this model)
  int32_t credit;           // device cycles available to spend
  int64_t clock;            // earliest issue cycle of the next instruction
  int64_t vready[32];       // cycle when a vector register becomes readable
  int64_t sready[32];       // same for scalar registers written by loads / cop moves
  int64_t last_load;        // issue cycle of the last load or cop move
  int pending_bubble;       // branch bubble owed after the delay slot
  // the previously issued instruction
  bool prev_valid, prev_vector, prev_branch, prev_delay, prev_paired, prev_target;
  int64_t prev_issue;
  uint32_t prev_pc, prev_vwrite;
  // decision for the instruction about to execute (filled by rsp_hw_cost)
  int pend_cost;
  bool pend_pair, pend_bubble_only;
  int64_t pend_issue;
  // statistics
  uint64_t n_insn, n_pair, n_stall, n_branch;
};
extern bool g_rsp_hw_timing;
extern struct rsp_hwtiming *g_rsp_hw_stats;

struct rsp {
  struct bus_controller *bus;
  struct rsp_pipeline pipeline;
  struct rsp_cp2 cp2;

  uint32_t regs[NUM_RSP_REGISTERS];
  uint8_t mem[0x2000];

  // Instead of redecoding the instructions (there's only 256 words)
  // every cycle, we maintain a 256-word decoded instruction cache.
  struct rsp_opcode opcode_cache[0x1000 / 4];

  // TODO: Only for IA32/x86_64 SSE2; sloppy?
  struct dynarec_slab vload_dynarec;
  struct dynarec_slab vstore_dynarec;

  // -rsphw timing model state; last so that mem[] keeps its 16-byte alignment.
  struct rsp_hwtiming hw;
};

cen64_cold int rsp_init(struct rsp *rsp, struct bus_controller *bus);
cen64_cold void rsp_late_init(struct rsp *rsp);
cen64_cold void rsp_destroy(struct rsp *rsp);

cen64_flatten cen64_hot void rsp_cycle_(struct rsp *rsp);

cen64_hot void rsp_cycle_hw(struct rsp *rsp);
cen64_cold void rsp_hw_print_stats(const struct rsp *rsp);

cen64_flatten cen64_hot static inline void rsp_cycle(struct rsp *rsp) {
  if (unlikely(rsp->regs[RSP_CP0_REGISTER_SP_STATUS] & SP_STATUS_HALT))
    return;

  if (rsp->hw.enabled)
    rsp_cycle_hw(rsp);
  else
    rsp_cycle_(rsp);
}

#endif

