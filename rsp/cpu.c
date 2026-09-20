//
// rsp/cpu.c: RSP processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "rsp/cpu.h"
#include <string.h>

bool g_rsp_hw_timing = false;
struct rsp_hwtiming *g_rsp_hw_stats = NULL; // for the IS-Viewer profile windows
bool g_rsp_profile = false;
struct rsp_prof *g_rsp_prof = NULL;
#include "rsp/cp0.h"

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *sp_register_mnemonics[NUM_SP_REGISTERS] = {
#define X(reg) #reg,
#include "rsp/registers.md"
#undef X
};
#endif

// Sets the opaque pointer used for external accesses.
static void rsp_connect_bus(struct rsp *rsp, struct bus_controller *bus) {
  rsp->bus = bus;
}

// Releases memory acquired for the RSP component.
void rsp_destroy(struct rsp *rsp) {
  if (rsp->hw.enabled)
    rsp_hw_print_stats(rsp);

  arch_rsp_destroy(rsp);
}

// Initializes the RSP component.
int rsp_init(struct rsp *rsp, struct bus_controller *bus) {
  rsp_connect_bus(rsp, bus);

  rsp_cp0_init(rsp);
  rsp_pipeline_init(&rsp->pipeline);
  memset(&rsp->hw, 0, sizeof(rsp->hw));
  rsp->hw.enabled = g_rsp_hw_timing;
  g_rsp_hw_stats = &rsp->hw;
  memset(&rsp->prof, 0, sizeof(rsp->prof));
  rsp->prof.enabled = g_rsp_profile;
  rsp->prof.cur = -1;
  g_rsp_prof = &rsp->prof;

  return arch_rsp_init(rsp);
}

// Initializes (host) registers.
void rsp_late_init(struct rsp *rsp) {
  write_acc_lo(rsp->cp2.acc.e, rsp_vzero());
  write_acc_md(rsp->cp2.acc.e, rsp_vzero());
  write_acc_hi(rsp->cp2.acc.e, rsp_vzero());

  write_vcc_lo(rsp->cp2.flags[RSP_VCC].e, rsp_vzero());
  write_vcc_hi(rsp->cp2.flags[RSP_VCC].e, rsp_vzero());
  write_vco_lo(rsp->cp2.flags[RSP_VCO].e, rsp_vzero());
  write_vco_hi(rsp->cp2.flags[RSP_VCO].e, rsp_vzero());
  write_vce   (rsp->cp2.flags[RSP_VCE].e, rsp_vzero());
}


// Called when the CPU un-halts the RSP: pick the histogram bucket from the task's ucode address.
void rsp_prof_task_start(struct rsp *rsp) {
  uint32_t ucode;
  int i;
  if (!rsp->prof.enabled)
    return;
  memcpy(&ucode, rsp->mem + 0xFC0 + 0x10, sizeof(ucode)); // OSTask.ucode (DMEM end)
  for (i = 0; i < RSP_PROF_UCODES; i++) {
    if (rsp->prof.ucode[i] == ucode) { rsp->prof.cur = i; return; }
    if (rsp->prof.ucode[i] == 0) { rsp->prof.ucode[i] = ucode; rsp->prof.cur = i; return; }
  }
  rsp->prof.cur = -1;
}
