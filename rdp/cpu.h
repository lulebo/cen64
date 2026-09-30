//
// rdp/cpu.h: RDP processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef __rdp_cpu_h__
#define __rdp_cpu_h__
#include "common.h"
#include "bus/rdram_model.h"

enum dp_register {
#define X(reg) reg,
#include "rdp/registers.md"
#undef X
  NUM_DP_REGISTERS
};

#ifdef DEBUG_MMIO_REGISTER_ACCESS
extern const char *dp_register_mnemonics[NUM_DP_REGISTERS];
#endif

// -rdptime: RDP timing model. Every command is executed immediately (pixels), but
// costs modelled RDP cycles; a ring of executed commands with their modelled finish
// time drives DPC_CURRENT, the busy/START_VALID status bits and the full-sync
// interrupt, so the RDP appears to run behind the RSP as on hardware.
#define RDP_TIMING_RING 65536
struct rdp_timing_entry {
  uint32_t cur;     // address reported by DPC_CURRENT while this entry executes
  uint32_t next;    // address reported once it is done
  uint64_t finish;  // modelled cycle at which it is done
  uint32_t kind;    // 0 command, 1 DPC_START taken (new buffer), 2 full sync (interrupt)
};
struct rdp_timing {
  struct rdp_timing_entry ring[RDP_TIMING_RING];
  unsigned head, tail;
  uint64_t now, busy_until;
  unsigned start_valid;      // queued transfers (END written behind a running one): END_VALID
  unsigned start_pending;    // DPC_START latched, not yet taken by a DPC_END write: START_VALID
  unsigned on;
  double c1, c2, cfill, ccopy, cz, ctri, crect, ccmd, ctmem, csync; // cycles: per pixel by mode, per z pixel, per command kind, full sync
  double ctmemb; // cycles per byte of a texture load (on top of ctmem per load)
  double cspan, cspanfb, cspanz; // cycles per span: base, extra with image read, extra with z compare
  uint64_t stat_busy, stat_busy_frame;
  uint64_t fpx1, fpx2, fpxfill, fpxcopy, fpxz, ftri, fcmd, ftmem; // per-frame work behind the cost
  uint64_t fspans, fspansfb, fspansz;
  uint64_t fbusycyc;         // cycles with work queued (per frame)
  uint64_t fidle[3];         // idle cycles while the RSP runs gfx / audio / is halted (per frame)
  uint64_t idle_start, idle_cls[3]; // the idle interval in progress (CEN64_ISV_TS: RIDLE lines)
  int was_idle;
  uint64_t dpc_busy;         // cycles with work queued, ever (DPC_PIPEBUSY / DPC_BUFBUSY)
  uint64_t dpc_clock0, dpc_pipe0, dpc_cmd0; // values at the last counter clear
};
extern int g_rdp_timing;
extern uint32_t *g_rdp_rsp_status; // the RSP's SP_STATUS (set by the device): idle attribution
extern int g_rsp_task_is_audio;
extern int g_isv_ts;               // CEN64_ISV_TS: timestamps and RIDLE lines
extern uint64_t *g_rcp_now;        // the modelled RCP clock (rdp timing), for the IS-Viewer prefix
void rdp_idle_interval_end(struct rdp_timing *t);
extern const char *g_rdp_model;

struct rdp {
  uint32_t regs[NUM_DP_REGISTERS];
  struct bus_controller *bus;
  uint32_t exec_current;     // where rdp_process_list executes from (DPC_CURRENT when no model)
  struct rdp_timing timing;
};

void rdp_timing_advance(struct rdp *rdp);
// One RCP cycle of the modelled RDP clock.
static inline void rdp_timing_tick(struct rdp *rdp) {
  struct rdp_timing *t = &rdp->timing;
  if (!t->on) return;
  t->now++;
  if (g_rdram.on) rdram_tick();
  if (t->head != t->tail) {
    t->fbusycyc++;
    t->dpc_busy++;
    if (unlikely(t->was_idle)) rdp_idle_interval_end(t);
    if (t->now >= t->ring[t->head].finish) rdp_timing_advance(rdp);
  } else {
    unsigned cls = (*g_rdp_rsp_status & 1) ? 2 : g_rsp_task_is_audio ? 1 : 0; // SP_STATUS_HALT
    t->fidle[cls]++;
    if (g_isv_ts) {
      if (!t->was_idle) {
        t->was_idle = 1;
        t->idle_start = t->now;
        t->idle_cls[0] = t->idle_cls[1] = t->idle_cls[2] = 0;
      }
      t->idle_cls[cls]++;
    }
  }
}

cen64_cold int rdp_init(struct rdp *rdp, struct bus_controller *bus);

#endif

