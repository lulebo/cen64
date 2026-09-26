//
// bus/rdram_model.h: RDRAM channel contention model (-rdram, see PROFILING.md).
//
// One RDRAM channel, served first-come-first-served in RDRAM clocks (4 ns, 4 per RCP
// cycle). 1 MB banks, each with one open row (2 KB): an access to another row of
// the bank pays a row miss. Every agent books its real transactions on it:
//   CPU  D/I-cache fills, dirty write-backs, cache-instruction write-backs, uncached
//        accesses; the CPU stalls for the fixed path outside the RDRAM plus the actual
//        wait and service time (instead of cen64's fixed 46/50/38 cycles);
//   RSP  DMAs, split into bursts; SP_DMA_BUSY / SP_DMA_FULL stay set until done;
//   RDP  per command: command fetch, texture load rows, and per span the z / colour
//        reads and writes plus pixel cycles, replayed on the RDP's own clock (the
//        pixels are still produced immediately; DPC_CURRENT and the DP interrupt
//        follow the modelled clock, as with -rdptime);
//   VI   one line fetch per active line; a refresh per line (closes the open rows);
//   PI   cart -> RDRAM writes in 128-byte bursts at the PI's pace.
// Not validated against hardware: parameters are calibration knobs (-rdrammodel).
//
#ifndef CEN64_BUS_RDRAM_MODEL_H
#define CEN64_BUS_RDRAM_MODEL_H
#include <stdint.h>

enum rdram_agent {
  RA_CPU_I, RA_CPU_D, RA_CPU_WB, RA_CPU_UNC, RA_RSP, RA_RDP_CMD, RA_RDP_TEX,
  RA_RDP_FB, RA_RDP_Z, RA_VI, RA_PI, RA_REFRESH, RA_N
};

struct rdram_params {
  double req;        // request packet + turnaround, RDRAM clocks
  double rdelay;     // read delay (RI ReadDelay = 7)
  double wdelay;     // write delay (RI WriteDelay = 1)
  double per_byte;   // clocks per data byte (0.5: 9-bit channel, both edges)
  double row_miss;   // extra clocks when the bank's open row is another one
  double refresh;    // channel clocks per refresh (one per VI line)
  double refresh_close; // 1: a refresh closes every open row
  double cpu_d, cpu_i, cpu_unc, cpu_wb, cpu_uncw; // CPU cycles outside the RDRAM
  double burst;      // max bytes per RSP / RDP / VI / PI transaction
  double px1, px2, fill, copy; // RDP cycles per pixel by cycle type
  double tri, rect, cmd, tmem8, sync, span; // RDP cycles per triangle, rectangle, command,
                                            // 8 TMEM bytes, full sync, span
  double overlap;    // 1: a span's pixel cycles overlap its memory accesses (max), 0: add
  double bank_bits;  // log2 of the bank size (20 = 1 MB)
  double row_bits;   // log2 of the row size (11 = 2 KB)
  double pi_gap;     // RCP cycles between the PI's bursts
  double cmd_fetch;  // bytes per RDP command fetch
  double idle;       // RCP cycles an idle RDP needs before its first transaction
};
#define RDRAM_NPARAMS 30

struct rdram_stat { uint64_t n, bytes, busy, wait, hits, misses; };

struct rdram_model {
  int on;
  uint64_t now;       // RDRAM clocks
  uint64_t bus_free;  // the channel is free from here
  int32_t open_row[16];
  struct rdram_params p;
  struct rdram_stat st[RA_N];
  uint64_t cpu_stall, cpu_idle;   // CPU stall cycles charged / what an idle channel would cost
  uint64_t rdp_busy;              // RDRAM clocks with RDP work pending
  uint64_t rdp_busy_total;        // same, never reset
  uint64_t rsp_dma_busy;          // RDRAM clocks with an RSP DMA in flight
  uint64_t window_start;
};
extern struct rdram_model g_rdram;
extern const char *g_rdram_params;
extern int g_rdram_enable;

void rdram_model_init(void);
uint64_t rdram_access(uint64_t t, uint32_t addr, uint32_t bytes, int write, int agent);
struct rdp;
extern struct rdp *g_rdram_rdp;

// CPU (return the stall in CPU cycles, total)
unsigned rdram_cpu_read(uint32_t paddr, unsigned bytes, int agent);
void rdram_cpu_victim(uint32_t paddr);
unsigned rdram_cpu_cacheop_wb(uint32_t paddr);
unsigned rdram_cpu_uncached_write(uint32_t paddr, unsigned bytes);

// RSP DMA
void rdram_rsp_dma(uint32_t dram, uint32_t length, uint32_t count, uint32_t skip, int write);
unsigned rdram_rsp_dma_inflight(void);

// RDP work (emitted while angrylion executes the command)
void rdram_rdp_cmd_begin(uint32_t cmd, uint32_t addr, uint32_t len, int xbus);
void rdram_rdp_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                    int image_read, int z_compare, int z_update);
void rdram_rdp_texload(uint32_t addr, uint32_t bytes);
void rdram_rdp_mark(unsigned ring_index);
int rdram_rdp_pending(void);
void rdram_rdp_flush(void);

// clocks, VI, PI
void rdram_tick(void);
void rdram_vi_line(uint32_t addr, uint32_t bytes);
void rdram_refresh(void);
void rdram_pi_write(uint32_t addr, uint32_t bytes);

// bench windows
void rdram_window_reset(void);
void rdram_window_report(const char *name);
#endif
