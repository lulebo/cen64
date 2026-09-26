#ifndef CEN64_COMMON_BUS_TRAFFIC_H
#define CEN64_COMMON_BUS_TRAFFIC_H
#include <stdint.h>

/* RDRAM traffic by agent (see PROFILING.md, "RDRAM traffic"). Counts, not time: cen64 does not
 * model bus contention. Reset at BENCH_START, printed as a BUS, line at BENCH_END. */
struct bus_traffic {
  uint64_t cpu_ifill;        /* I-cache line fills, 32 bytes each */
  uint64_t cpu_ifetch_unc;   /* uncached instruction fetches, 4 bytes */
  uint64_t cpu_dfill;        /* D-cache line fills, 16 bytes */
  uint64_t cpu_dwb;          /* dirty lines written back by a miss, 16 bytes */
  uint64_t cpu_dwb_op;       /* dirty lines written back by a cache instruction, 16 bytes */
  uint64_t cpu_unc_r, cpu_unc_rb, cpu_unc_w, cpu_unc_wb; /* uncached RDRAM accesses / bytes */
  uint64_t cpu_mmio;         /* uncached accesses outside RDRAM (registers, cart) */
  uint64_t rsp_dma_r, rsp_dma_rb, rsp_dma_w, rsp_dma_wb; /* DMAs / bytes, RDRAM -> SP, SP -> RDRAM */
  uint64_t rsp_aud_rb, rsp_aud_wb;                        /* of which the audio task's bytes */
  uint64_t rdp_cmd_b;        /* command bytes fetched from RDRAM (not XBUS) */
  uint64_t rdp_tex_b;        /* texture / TLUT load bytes */
  uint64_t rdp_fbr, rdp_fbw, rdp_fill, rdp_zr, rdp_zw, rdp_frames; /* pixels, full syncs */
  uint64_t vi_b;             /* framebuffer bytes fetched by the VI */
  uint64_t ai_b;             /* AI DMA bytes */
  uint64_t pi_w_b, pi_r_b;   /* PI DMA bytes cart -> RDRAM, RDRAM -> cart */
  uint64_t cpu_cycles;       /* CPU cycles in the window */
};
extern struct bus_traffic g_bus;
extern int g_rsp_task_is_audio;

/* per 16-byte RDRAM line: [0, DLINES) fills, [DLINES, 2 * DLINES) dirty write-backs */
#define DLINES (8 * 1024 * 1024 / 16)
extern uint64_t *g_dline_prof;
#define DLINE_FILL(pa) do { if (g_dline_prof) g_dline_prof[((pa) & 0x7FFFFF) >> 4]++; } while (0)
#define DLINE_WB(pa) do { if (g_dline_prof) g_dline_prof[DLINES + (((pa) & 0x7FFFFF) >> 4)]++; } while (0)
/* CEN64_DRANGE: D-cache fills inside physical ranges, per (range, PC) */
void drange_fill(uint32_t paddr, uint32_t pc);
void drange_reset(void);
void drange_dump(const char *path);
extern int g_drange_n;
#define DRANGE_FILL(pa, pc) do { if (g_drange_n) drange_fill((pa), (pc)); } while (0)
#define PROF_REGION (8 * 1024 * 1024)
#define PROF_REGIONS 6 /* ins, l1d (misses + uncached), cycles, icache, dirty write-backs, uncached RDRAM */
#endif
