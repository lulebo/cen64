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

// Reads a word from the DP MMIO register space.
int read_dp_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct rdp *rdp = (struct rdp *) opaque;
  uint32_t offset = address - DP_REGS_BASE_ADDRESS;
  enum dp_register reg = (offset >> 2);

  *word = rdp->regs[reg];
  if (rdp->timing.on) {
    struct rdp_timing *t = &rdp->timing;
    static int n = 0;
    if (reg == DPC_CURRENT_REG && t->head != t->tail)
      *word = t->ring[t->head].cur;
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
          e->cur = e->next = start;
          e->finish = t->busy_until;
          e->kind = 1;
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
      break;

    default:
      rdp->regs[reg] &= ~dqm;
      rdp->regs[reg] |= word;
      break;
  }

  return 0;
}

