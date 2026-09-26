//
// vr4300/cpu.c: VR4300 processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "vr4300/interface.h"
#include "vr4300/cp0.h"
#include "vr4300/cp1.h"
#include "vr4300/cpu.h"
#include "vr4300/icache.h"
#include "vr4300/pipeline.h"

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *mi_register_mnemonics[NUM_MI_REGISTERS] = {
#define X(reg) #reg,
#include "vr4300/registers.md"
#undef X
};
#endif

uint64_t *g_vr4300_profile_samples = NULL;
#include "common/bus_traffic.h"
struct bus_traffic g_bus;

int g_drange_n = 0;
static uint32_t drange_lo[8], drange_hi[8];
#define DRANGE_HASH (1 << 16)
static struct { uint32_t key_pc; uint8_t range; uint8_t used; uint64_t fills; } drange_tab[DRANGE_HASH];
static void drange_init(void) {
  const char *e = getenv("CEN64_DRANGE");
  while (e && *e && g_drange_n < 8) {
    char *end;
    drange_lo[g_drange_n] = (uint32_t) strtoul(e, &end, 16);
    if (*end != ':') break;
    drange_hi[g_drange_n] = (uint32_t) strtoul(end + 1, &end, 16);
    g_drange_n++;
    e = *end == ',' ? end + 1 : end;
  }
}
void drange_fill(uint32_t paddr, uint32_t pc) {
  int r;
  paddr &= 0x1FFFFFFF;
  for (r = 0; r < g_drange_n; r++) {
    if (paddr >= drange_lo[r] && paddr < drange_hi[r]) {
      uint32_t h = ((pc >> 2) * 2654435761u + r) & (DRANGE_HASH - 1), n;
      for (n = 0; n < DRANGE_HASH; n++, h = (h + 1) & (DRANGE_HASH - 1)) {
        if (!drange_tab[h].used) { drange_tab[h].used = 1; drange_tab[h].key_pc = pc; drange_tab[h].range = (uint8_t) r; }
        if (drange_tab[h].key_pc == pc && drange_tab[h].range == r) { drange_tab[h].fills++; break; }
      }
    }
  }
}
void drange_reset(void) { memset(drange_tab, 0, sizeof(drange_tab)); }
void drange_dump(const char *path) {
  FILE *f; unsigned i;
  if (!g_drange_n || !(f = fopen(path, "w"))) return;
  for (i = 0; i < DRANGE_HASH; i++)
    if (drange_tab[i].used)
      fprintf(f, "%u %x %llu\n", drange_tab[i].range, drange_tab[i].key_pc, (unsigned long long) drange_tab[i].fills);
  fclose(f);
}
uint64_t *g_dline_prof = NULL;

void vr4300_cycle(struct vr4300 *vr4300) {
  struct vr4300_pipeline *pipeline = &vr4300->pipeline;

  // Increment counters.
  vr4300->regs[VR4300_CP0_REGISTER_COUNT]++;
  g_bus.cpu_cycles++;

  // Profiling: charge this cycle to the instruction in the DC stage, so that a
  // stall lands on the load/store (or the instruction after an I-fetch) that
  // caused it.
  if (vr4300->profile_samples) {
    uint32_t idx = (uint32_t) pipeline->exdc_latch.common.pc - 0x80000000;
    idx &= (8 * 1024 * 1024) - 1;
    vr4300->profile_samples[idx + 2 * (8 * 1024 * 1024)]++;
  }

  // We're stalling for something...
  if (pipeline->cycles_to_stall > 0)
    pipeline->cycles_to_stall--;

  else
    vr4300_cycle_(vr4300);

  if ((vr4300->regs[VR4300_CP0_REGISTER_COUNT] & 1) == 1 &&
    (uint32_t) (vr4300->regs[VR4300_CP0_REGISTER_COUNT] >> 1) ==
    (uint32_t) vr4300->regs[VR4300_CP0_REGISTER_COMPARE]) {
    vr4300->regs[VR4300_CP0_REGISTER_CAUSE] |= 0x8000;
  }
}

// Sets the opaque pointer used for external accesses.
static void vr4300_connect_bus(struct vr4300 *vr4300,
  struct bus_controller *bus) {
  vr4300->bus = bus;
}

// Initializes the VR4300 component.
int vr4300_init(struct vr4300 *vr4300, struct bus_controller *bus, bool profiling) {
  vr4300_connect_bus(vr4300, bus);

  vr4300_cp0_init(vr4300);
  vr4300_cp1_init(vr4300);

  vr4300_dcache_init(&vr4300->dcache);
  vr4300_icache_init(&vr4300->icache);

  vr4300_pipeline_init(&vr4300->pipeline);
  vr4300->signals = VR4300_SIGNAL_COLDRESET;

  // MESS uses this version, so we will too?
  vr4300->mi_regs[MI_VERSION_REG] = 0x01010101;
  vr4300->mi_regs[MI_INIT_MODE_REG] = 0x80;

  if (profiling) {
    vr4300->profile_samples = calloc(PROF_REGIONS * PROF_REGION, sizeof(uint64_t));
    g_vr4300_profile_samples = vr4300->profile_samples;
    g_dline_prof = calloc(2 * DLINES, sizeof(uint64_t));
    drange_init();
  } else
    vr4300->profile_samples = NULL;

  vr4300_debug_init(&vr4300->debug);

  return 0;
}

// Prints out simulation information to stdout.
void vr4300_print_summary(struct vr4300_stats *stats) {
  unsigned i, j;
  float secs;
  float cpi;

  // Print banner.
  printf("###############################\n"
         " NEC VR4300 Simulation Summary\n"
         "###############################\n"
         "\n"
  );

  // Print configuration, summary, whatever.
  secs = stats->total_cycles / 93750000.0f;

  printf("   %16s: %.1f sec.\n"
         "\n\n",

    "Actual runtime", secs
  );

  // Print performance statistics.
  cpi = (float) stats->executed_instructions / stats->total_cycles;

  printf(" * Performance statistics:\n\n"
         "   %16s: %lu\n"
         "   %16s: %lu\n"
         "   %16s: %1.2f\n"
         "\n\n",

    "Elapsed pcycles", stats->total_cycles,
    "Insns executed", stats->executed_instructions,
    "Average CPI", cpi
  );

  // Print executed opcode counts.
  printf(" * Executed instruction counts:\n\n");

  for (i = 1; i < NUM_VR4300_OPCODES; i += 2) {
    for (j = 0; i + j < NUM_VR4300_OPCODES && j < 2; j++) {
      printf("   %16s: %16lu", vr4300_opcode_mnemonics[i + j],
        stats->opcode_counts[i + j]);

      if (j == 0)
        printf("\t");
    }

    printf("\n");
  }
}

uint64_t vr4300_get_register(struct vr4300 *vr4300, size_t i) {
  return vr4300->regs[i];
}

uint64_t vr4300_get_pc(struct vr4300 *vr4300) {
  return vr4300->pipeline.dcwb_latch.common.pc;
}

cen64_cold void vr4300_signal_break(struct vr4300 *vr4300) {
  vr4300_debug_signal(&vr4300->debug, VR4300_DEBUG_SIGNALS_BREAK);
}

cen64_cold void vr4300_set_breakpoint(struct vr4300 *vr4300, uint64_t at) {
  vr4300_debug_set_breakpoint(&vr4300->debug, at);
}

cen64_cold void vr4300_remove_breakpoint(struct vr4300 *vr4300, uint64_t at) {
  vr4300_debug_remove_breakpoint(&vr4300->debug, at);
}

struct vr4300* vr4300_alloc() {
    struct vr4300* ptr = (struct vr4300*)malloc(sizeof(struct vr4300));
    memset(ptr, 0, sizeof(struct vr4300));
    return ptr;
}

cen64_cold void vr4300_free(struct vr4300* ptr) {
    vr4300_debug_cleanup(&ptr->debug);
    free(ptr);
}

cen64_cold struct vr4300_stats* vr4300_stats_alloc() {
    struct vr4300_stats* ptr = (struct vr4300_stats*)malloc(sizeof(struct vr4300_stats));
    memset(ptr, 0, sizeof(struct vr4300_stats));
    return ptr;
}

cen64_cold void vr4300_stats_free(struct vr4300_stats* ptr) {
    free(ptr);
}

cen64_cold void vr4300_connect_debugger(struct vr4300 *vr4300, void* break_handler_data, vr4300_debug_break_handler break_handler) {
  vr4300->debug.break_handler = break_handler;
  vr4300->debug.break_handler_data = break_handler_data;
}