//
// rdp/interface.c: RDP interface.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include "bus/address.h"
#include "rdp/cpu.h"
#include "rdp/interface.h"

#define DP_XBUS_DMEM_DMA          0x00000001
#define DP_FREEZE                 0x00000002
#define DP_FLUSH                  0x00000004

#define DP_CLEAR_XBUS_DMEM_DMA    0x00000001
#define DP_SET_XBUS_DMEM_DMA      0x00000002
#define DP_CLEAR_FREEZE           0x00000004
#define DP_SET_FREEZE             0x00000008
#define DP_CLEAR_FLUSH            0x00000010
#define DP_SET_FLUSH              0x00000020

void rdp_process_list(void);

// RDPDBG=1 in the environment traces the first DPC register accesses (-rdptime only).
static int rdp_dbg(void) {
  static int dbg = -1;
  if (dbg < 0) dbg = getenv("RDPDBG") != NULL;
  return dbg;
}

static uint32_t rdp_rsp_status_none = 1;
uint32_t *g_rdp_rsp_status = &rdp_rsp_status_none;
static uint64_t rcp_now_none = 0;
uint64_t *g_rcp_now = &rcp_now_none;
int g_isv_ts = 0;

void rdp_idle_interval_end(struct rdp_timing *t) {
  uint64_t len = t->now - t->idle_start;
  t->was_idle = 0;
  if (len > 6250) // 0.1 ms
    printf("RIDLE,%llu,%llu,%llu,%llu,%llu\n", (unsigned long long) t->idle_start, (unsigned long long) len,
           (unsigned long long) t->idle_cls[0], (unsigned long long) t->idle_cls[1],
           (unsigned long long) t->idle_cls[2]);
}

// CEN64_DPC_PREFETCH: the fetch pointer - the end of the entries within n bytes of the executing
// one, not past a queued transfer (kind 1) - as DPC_CURRENT.
static int dpc_prefetch = -1;
static uint32_t dpc_fetch_pointer(struct rdp_timing *t) {
  unsigned i = t->head;
  uint32_t base = t->ring[t->head].cur, p = t->ring[t->head].cur;
  while (i != t->tail) {
    const struct rdp_timing_entry *e = &t->ring[i];
    if (e->kind == 1 && i != t->head)
      break;
    if (e->kind != 1)
      p = e->next;
    if (e->next - base >= (uint32_t) dpc_prefetch)
      break;
    i = (i + 1) & (RDP_TIMING_RING - 1);
  }
  return p;
}

// Reads a word from the DP MMIO register space.
int read_dp_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct rdp *rdp = (struct rdp *) opaque;
  uint32_t offset = address - DP_REGS_BASE_ADDRESS;
  enum dp_register reg = (offset >> 2);

  *word = rdp->regs[reg];
  if (rdp->timing.on) {
    struct rdp_timing *t = &rdp->timing;
    static int n = 0;
    if (dpc_prefetch < 0)
      dpc_prefetch = getenv("CEN64_DPC_PREFETCH") ? atoi(getenv("CEN64_DPC_PREFETCH")) : 0;
    if (reg == DPC_CURRENT_REG && t->head != t->tail)
      *word = dpc_prefetch > 0 && t->ring[t->head].kind != 1 ? dpc_fetch_pointer(t) : t->ring[t->head].cur;
    else if (reg == DPC_CLOCK_REG)
      *word = (uint32_t) (t->now - t->dpc_clock0) & 0xFFFFFF;
    else if (reg == DPC_PIPEBUSY_REG)
      *word = (uint32_t) (t->dpc_busy - t->dpc_pipe0) & 0xFFFFFF;
    else if (reg == DPC_BUFBUSY_REG)
      *word = (uint32_t) (t->dpc_busy - t->dpc_cmd0) & 0xFFFFFF;
    else if (reg == DPC_TMEM_REG)
      *word = 0;
    else if (reg == DPC_STATUS_REG)
      *word |= (t->start_pending ? 0x400 : 0) | (t->start_valid ? 0x200 : 0) | (t->head != t->tail ? 0x160 : 0); // START_VALID; END_VALID; DMA/CMD/PIPE busy
    {
      static uint64_t last_print = 0;
      if (0) {
        last_print = t->now;
        fprintf(stderr, "STALL? reg %u q=%u sp=%u sv=%u now=%llu head.finish=%llu busy=%llu cur=%08x start=%08x end=%08x%c", (unsigned) reg,
                (t->tail - t->head) & (RDP_TIMING_RING - 1), t->start_pending, t->start_valid, (unsigned long long) t->now,
                (unsigned long long) t->ring[t->head].finish, (unsigned long long) t->busy_until,
                rdp->regs[DPC_CURRENT_REG], rdp->regs[DPC_START_REG], rdp->regs[DPC_END_REG], 10);
      }
    }
    if (rdp_dbg() && n < 600) {
      n++;
      fprintf(stderr, "RD %u -> %08x q=%u sp=%u now=%llu busy=%llu\n", (unsigned) reg, *word,
              (t->tail - t->head) & (RDP_TIMING_RING - 1), t->start_pending,
              (unsigned long long) t->now, (unsigned long long) t->busy_until);
    }
  }
  debug_mmio_read(dp, dp_register_mnemonics[reg], *word);
  return 0;
}

// Writes a word to the DP MMIO register space.
int write_dp_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct rdp *rdp = (struct rdp *) opaque;
  uint32_t offset = address - DP_REGS_BASE_ADDRESS;
  enum dp_register reg = (offset >> 2);

  debug_mmio_write(dp, dp_register_mnemonics[reg], word, dqm);
  if (rdp->timing.on && rdp_dbg()) {
    static int n = 0;
    if (n < 600) { n++; fprintf(stderr, "WR %u <- %08x\n", (unsigned) reg, word); }
  }

  switch (reg) {
    case DPC_START_REG:
      // Hardware: the start is latched (START_VALID) and only taken by the next
      // DPC_END write, which begins the new transfer once the running one is done.
      rdp->regs[DPC_START_REG] = word;
      if (rdp->timing.on) {
        rdp->timing.start_pending = 1;
      } else {
        rdp->exec_current = word;
        rdp->regs[DPC_CURRENT_REG] = word;
      }
      break;

    case DPC_END_REG:
      rdp->regs[DPC_END_REG] = word;
      if (rdp->timing.on && rdp->timing.start_pending) {
        struct rdp_timing *t = &rdp->timing;
        uint32_t start = rdp->regs[DPC_START_REG];
        t->start_pending = 0;
        rdp->exec_current = start;
        if (t->head != t->tail) {
          // still busy: the new transfer is queued behind the running one (END_VALID)
          struct rdp_timing_entry *e = &t->ring[t->tail];
          if (start == word) {
            static unsigned dpzero = 0;
            if (dpzero++ < 50)
              printf("DPZERO,%llu,%06x,%06x\n", (unsigned long long) t->now, start,
                     t->ring[t->head].cur);
          }
          e->cur = e->next = start;
          e->finish = t->busy_until;
          e->kind = 1;
          if (g_rdram.on) {
            e->finish = UINT64_MAX;
            rdram_rdp_mark(t->tail);
          }
          t->tail = (t->tail + 1) & (RDP_TIMING_RING - 1);
          t->start_valid++;
        } else {
          rdp->regs[DPC_CURRENT_REG] = start;
        }
      }
      rdp_process_list();
      break;

    case DPC_STATUS_REG:
      if (word & DP_CLEAR_XBUS_DMEM_DMA)
        rdp->regs[DPC_STATUS_REG] &= ~DP_XBUS_DMEM_DMA;
      else if (word & DP_SET_XBUS_DMEM_DMA)
        rdp->regs[DPC_STATUS_REG] |= DP_XBUS_DMEM_DMA;

      if (word & DP_CLEAR_FREEZE)
        rdp->regs[DPC_STATUS_REG] &= ~DP_FREEZE;
//      else if (word & DP_SET_FREEZE)
//        rdp->regs[DPC_STATUS_REG] |= DP_FREEZE;

      if (word & DP_CLEAR_FLUSH)
        rdp->regs[DPC_STATUS_REG] &= ~DP_FLUSH;
      else if (word & DP_SET_FLUSH)
        rdp->regs[DPC_STATUS_REG] |= DP_FLUSH;

      // counters (bit 6 tmem, 7 pipe, 8 cmd, 9 clock): kept by the timing model
      if (word & 0x200) rdp->timing.dpc_clock0 = rdp->timing.now;
      if (word & 0x080) rdp->timing.dpc_pipe0 = rdp->timing.dpc_busy;
      if (word & 0x100) rdp->timing.dpc_cmd0 = rdp->timing.dpc_busy;
      break;

    default:
      rdp->regs[reg] &= ~dqm;
      rdp->regs[reg] |= word;
      break;
  }

  return 0;
}

