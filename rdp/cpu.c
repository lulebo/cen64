//
// rdp/cpu.c: RDP processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "rdp/cpu.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vr4300/interface.h"
#include "bus/controller.h"

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *dp_register_mnemonics[NUM_DP_REGISTERS] = {
#define X(reg) #reg,
#include "rdp/registers.md"
#undef X
};
#endif

// Sets the opaque pointer used for external accesses.
static void rdp_connect_bus(struct rdp *rdp, struct bus_controller *bus) {
  rdp->bus = bus;
}

// Initializes the RDP component.
int g_rdp_timing = 0;
const char *g_rdp_model = NULL;

// Pop every entry the modelled RDP has finished: DPC_START markers move
// DPC_CURRENT to the new buffer, full syncs raise the DP interrupt.
void rdp_timing_advance(struct rdp *rdp) {
  struct rdp_timing *t = &rdp->timing;
  while (t->head != t->tail && t->now >= t->ring[t->head].finish) {
    struct rdp_timing_entry *e = &t->ring[t->head];
    rdp->regs[DPC_CURRENT_REG] = e->next;
    if (e->kind == 1) t->start_valid--;
    else if (e->kind == 2) signal_rcp_interrupt(rdp->bus->vr4300, MI_INTR_DP);
    t->head = (t->head + 1) & (RDP_TIMING_RING - 1);
  }
}

static void rdp_timing_init(struct rdp *rdp) {
  struct rdp_timing *t = &rdp->timing;
  double *p[9] = { &t->c1, &t->c2, &t->cfill, &t->ccopy, &t->cz, &t->ctri, &t->crect, &t->ccmd, &t->ctmem };
  double def[9] = { 1.0, 2.0, 0.25, 0.25, 0.5, 64, 32, 8, 256 };
  const char *m = g_rdp_model ? g_rdp_model : getenv("CEN64_RDP_MODEL");
  int i;
  for (i = 0; i < 9; i++) *p[i] = def[i];
  if (m) {
    for (i = 0; i < 9 && m && *m; i++) {
      *p[i] = atof(m);
      m = strchr(m, ',');
      if (m) m++;
    }
  }
  t->on = g_rdp_timing != 0;
  if (t->on)
    fprintf(stderr, "RDP timing model: px1 %g px2 %g fill %g copy %g z %g tri %g rect %g cmd %g tmem %g\n",
            t->c1, t->c2, t->cfill, t->ccopy, t->cz, t->ctri, t->crect, t->ccmd, t->ctmem);
}

int rdp_init(struct rdp *rdp, struct bus_controller *bus) {
  rdp_timing_init(rdp);
  rdp_connect_bus(rdp, bus);

  return 0;
}

