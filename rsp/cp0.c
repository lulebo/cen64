//
// rsp/cp0.c: RSP control coprocessor.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "bus/address.h"
#include "bus/controller.h"
#include "rdp/interface.h"
#include "rsp/cp0.h"
#include "rsp/cpu.h"
#include "rsp/decoder.h"
#include "rsp/interface.h"
#include "bus/rdram_model.h"
#include "vr4300/interface.h"
#ifdef _WIN32
  #include <windows.h>
#endif

//
// MFC0
//
void RSP_MFC0(struct rsp *rsp,
  uint32_t iw, uint32_t rs, uint32_t rt) {
  struct rsp_exdf_latch *exdf_latch = &rsp->pipeline.exdf_latch;
  unsigned dest;

  dest = GET_RT(iw);
  rt = rsp_read_cp0_reg(rsp, GET_RD(iw) & 0x0f);

  exdf_latch->result.result = rt;
  exdf_latch->result.dest = dest;
}

//
// MTC0
//
void RSP_MTC0(struct rsp *rsp,
  uint32_t iw, uint32_t rs, uint32_t rt) {
  unsigned dest;

  dest = GET_RD(iw);
  rsp_write_cp0_reg(rsp, dest & 0x0f, rt);
}

// Reads a value from the control coprocessor.
uint32_t rsp_read_cp0_reg(struct rsp *rsp, unsigned src) {
  uint32_t word;

  src = SP_REGISTER_OFFSET + src;

  switch(src) {
    case RSP_CP0_REGISTER_SP_STATUS:
      if (g_rdram.on) {
        unsigned n = rdram_rsp_dma_inflight();
        return *((volatile uint32_t *) &rsp->regs[RSP_CP0_REGISTER_SP_STATUS]) |
          (n >= 1 ? SP_STATUS_DMA_BUSY : 0) | (n >= 2 ? SP_STATUS_DMA_FULL : 0);
      }
      return *((volatile uint32_t *) &rsp->regs[RSP_CP0_REGISTER_SP_STATUS]);

    case RSP_CP0_REGISTER_SP_RESERVED:
#ifdef _WIN32
      return _InterlockedCompareExchange((volatile long *)
          (&rsp->regs[RSP_CP0_REGISTER_SP_RESERVED]), 1, 0) != 0;
#else
      return !__sync_bool_compare_and_swap(
          &rsp->regs[RSP_CP0_REGISTER_SP_RESERVED], 0, 1);
#endif

    case RSP_CP0_REGISTER_DMA_FULL:
      return g_rdram.on && rdram_rsp_dma_inflight() >= 2;
    case RSP_CP0_REGISTER_DMA_BUSY:
      return g_rdram.on && rdram_rsp_dma_inflight() >= 1;

    // RDP aliases.
    case RSP_CP0_REGISTER_CMD_START:
    case RSP_CP0_REGISTER_CMD_END:
    case RSP_CP0_REGISTER_CMD_CURRENT:
    case RSP_CP0_REGISTER_CMD_STATUS:
    case RSP_CP0_REGISTER_CMD_CLOCK:
    case RSP_CP0_REGISTER_CMD_BUSY:
    case RSP_CP0_REGISTER_CMD_PIPE_BUSY:
    case RSP_CP0_REGISTER_CMD_TMEM_BUSY:
      src -= RSP_CP0_REGISTER_CMD_START;

      read_dp_regs(rsp->bus->rdp, DP_REGS_BASE_ADDRESS + src * 4, &word);
      return word;

    default:
      return rsp->regs[src];
  }

  return 0;
}

// Updates the SP_STATUS register according to bitmask in rt.
int rsp_audtask_on = -1, rsp_audtask_active = 0;
uint32_t rsp_audtask_hash = 2166136261u;
uint32_t rsp_audtask_rhash = 2166136261u;
static unsigned rsp_audtask_n = 0;
unsigned rsp_audtask_cur = 0;

static uint32_t rsp_audtask_dmem_word(struct rsp *rsp, unsigned offset) {
  uint32_t w;
  memcpy(&w, rsp->mem + offset, sizeof(w));
  return byteswap_32(w);
}

int g_rsp_task_is_audio = 0;
static void rsp_audtask_start(struct rsp *rsp) {
  g_rsp_task_is_audio = rsp_audtask_dmem_word(rsp, 0xFC0) == 2; /* OSTask.type == M_AUDTASK */
  if (rsp_audtask_on < 0) rsp_audtask_on = getenv("CEN64_AUDTASK_HASH") != NULL;
  if (!rsp_audtask_on) return;
  if (rsp_audtask_active)
    printf("ATO,%u,%08x,%08x\n", rsp_audtask_n - 1, rsp_audtask_hash, rsp_audtask_rhash);
  rsp_audtask_active = 0;
  if (rsp_audtask_dmem_word(rsp, 0xFC0) == 2) {           // OSTask.type == M_AUDTASK
    uint32_t ptr = rsp_audtask_dmem_word(rsp, 0xFC0 + 0x30) & 0x7FFFFC;
    uint32_t size = rsp_audtask_dmem_word(rsp, 0xFC0 + 0x34), i, h = 2166136261u;
    static int cmddump = -2;
    if (cmddump == -2) cmddump = getenv("CEN64_AUDTASK_CMDDUMP") ? atoi(getenv("CEN64_AUDTASK_CMDDUMP")) : -1;
    for (i = 0; i < size; i += 4) {
      uint32_t w, k;
      bus_read_word(rsp->bus, ptr + i, &w);
      if ((int) rsp_audtask_n == cmddump) printf("ATC,%u,%u,%08x\n", rsp_audtask_n, i, w);
      for (k = 0; k < 4; k++) { h ^= (w >> (24 - 8 * k)) & 0xFF; h *= 16777619u; }
    }
    {
      static int clr = -1;
      if (clr < 0) clr = getenv("CEN64_AUDTASK_CLEAR") != NULL;
      if (clr) memset(rsp->mem, 0, 0xFC0);
    }
    rsp_audtask_cur = rsp_audtask_n;
    printf("ATI,%u,%u,%08x\n", rsp_audtask_n++, size, h);
    rsp_audtask_active = 1;
    rsp_audtask_hash = 2166136261u;
    rsp_audtask_rhash = 2166136261u;
  }
}

void rsp_status_write(struct rsp *rsp, uint32_t rt) {
  uint32_t prev_status, status;

  do {
    prev_status = rsp->regs[RSP_CP0_REGISTER_SP_STATUS];
    status = prev_status;

    if ((rt & SP_CLR_HALT) && (status & SP_STATUS_HALT)) {
      // Save PC around pipeline init
      uint32_t pc = rsp->pipeline.rdex_latch.common.pc;
      rsp_pipeline_init(&rsp->pipeline);
      rsp->pipeline.ifrd_latch.pc = pc;
      rsp_prof_task_start(rsp);
      rsp_audtask_start(rsp);

      status &= ~SP_STATUS_HALT;
    }

    else if (rt & SP_SET_HALT)
      status |= SP_STATUS_HALT;

    if (rt & SP_CLR_BROKE)
      status &= ~SP_STATUS_BROKE;
    else if (rt & SP_SET_BROKE)
      status |= SP_STATUS_BROKE;

    if (rt & SP_CLR_INTR)
      clear_rcp_interrupt(rsp->bus->vr4300, MI_INTR_SP);
    else if (rt & SP_SET_INTR)
      signal_rcp_interrupt(rsp->bus->vr4300, MI_INTR_SP);

    if (rt & SP_CLR_SSTEP)
      status &= ~SP_STATUS_SSTEP;
    else if (rt & SP_SET_SSTEP)
      status |= SP_STATUS_SSTEP;

    if (rt & SP_CLR_INTR_BREAK)
      status &= ~SP_STATUS_INTR_BREAK;
    else if  (rt & SP_SET_INTR_BREAK)
      status |= SP_STATUS_INTR_BREAK;

    if (rt & SP_CLR_SIG0)
      status &= ~SP_STATUS_SIG0;
    else if (rt & SP_SET_SIG0)
      status |= SP_STATUS_SIG0;

    if (rt & SP_CLR_SIG1)
      status &= ~SP_STATUS_SIG1;
    else if (rt & SP_SET_SIG1)
      status |= SP_STATUS_SIG1;

    if (rt & SP_CLR_SIG2)
      status &= ~SP_STATUS_SIG2;
    else if (rt & SP_SET_SIG2)
      status |= SP_STATUS_SIG2;

    if (rt & SP_CLR_SIG3)
      status &= ~SP_STATUS_SIG3;
    else if (rt & SP_SET_SIG3)
      status |= SP_STATUS_SIG3;

    if (rt & SP_CLR_SIG4)
      status &= ~SP_STATUS_SIG4;
    else if (rt & SP_SET_SIG4)
      status |= SP_STATUS_SIG4;

    if (rt & SP_CLR_SIG5)
      status &= ~SP_STATUS_SIG5;
    else if (rt & SP_SET_SIG5)
      status |= SP_STATUS_SIG5;

    if (rt & SP_CLR_SIG6)
      status &= ~SP_STATUS_SIG6;
    else if (rt & SP_SET_SIG6)
      status |= SP_STATUS_SIG6;

    if (rt & SP_CLR_SIG7)
      status &= ~SP_STATUS_SIG7;
    else if (rt & SP_SET_SIG7)
      status |= SP_STATUS_SIG7;
#ifdef _WIN32
  } while (!(_InterlockedCompareExchange((volatile long *)
    (&rsp->regs[RSP_CP0_REGISTER_SP_STATUS]),
    status, prev_status) == prev_status));
#else
  } while (!__sync_bool_compare_and_swap(
    &rsp->regs[RSP_CP0_REGISTER_SP_STATUS],
    prev_status, status));
#endif
}

// Writes a value to the control processor.
void rsp_write_cp0_reg(struct rsp *rsp, unsigned dest, uint32_t rt) {
  dest = SP_REGISTER_OFFSET + dest;

  switch(dest) {
    case RSP_CP0_REGISTER_DMA_CACHE:
      rsp->regs[RSP_CP0_REGISTER_DMA_CACHE] = rt & 0x1FFF;
      break;

    case RSP_CP0_REGISTER_DMA_DRAM:
      rsp->regs[RSP_CP0_REGISTER_DMA_DRAM] = rt & 0xFFFFFF;
      break;

    case RSP_CP0_REGISTER_DMA_READ_LENGTH:
      rsp->regs[RSP_CP0_REGISTER_DMA_READ_LENGTH] = rt;
      rsp_dma_read(rsp);
      break;

    case RSP_CP0_REGISTER_DMA_WRITE_LENGTH:
      rsp->regs[RSP_CP0_REGISTER_DMA_WRITE_LENGTH] = rt;
      rsp_dma_write(rsp);
      break;

    case RSP_CP0_REGISTER_SP_STATUS:
      // Mask out the synthetic SP_SET_BROKE bit which is used internally
      // but does not exist in real hardware (and has no effect if set).
      rsp_status_write(rsp, rt & ~SP_SET_BROKE);
      break;

    case RSP_CP0_REGISTER_SP_RESERVED:
      if (rt == 0) {
        *((volatile uint32_t *) &rsp->regs[RSP_CP0_REGISTER_SP_RESERVED]) = 0;
#ifdef _MSC_VER
        _ReadWriteBarrier();
#else
        __asm__ __volatile__("" ::: "memory");
#endif
      }

      break;

    // RDP aliases.
    case RSP_CP0_REGISTER_CMD_START:
    case RSP_CP0_REGISTER_CMD_END:
    case RSP_CP0_REGISTER_CMD_CURRENT:
    case RSP_CP0_REGISTER_CMD_STATUS:
    case RSP_CP0_REGISTER_CMD_CLOCK:
    case RSP_CP0_REGISTER_CMD_BUSY:
    case RSP_CP0_REGISTER_CMD_PIPE_BUSY:
    case RSP_CP0_REGISTER_CMD_TMEM_BUSY:
      dest -= RSP_CP0_REGISTER_CMD_START;

      write_dp_regs(rsp->bus->rdp, DP_REGS_BASE_ADDRESS + 4 * dest, rt, ~0);
      break;

    default:
      rsp->regs[dest] = rt;
      break;
  }
}

// Initializes the coprocessor.
void rsp_cp0_init(struct rsp *rsp) {
  rsp->regs[RSP_CP0_REGISTER_SP_STATUS] = SP_STATUS_HALT;
}

