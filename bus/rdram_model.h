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
//
// eng=1 (default): the RDP as a pipeline (front end, pixel stage, posted writes). A span
// is cut into chunks of `chunk` pixels; a chunk's reads (z, colour) are issued by the
// front end up to rdp_d chunks ahead of the pixel stage, its data arrives rd_lat clocks
// after the channel transfer; the pixel stage takes px1/px2 cycles per pixel (+ span at a
// span's first chunk) once the data is there; the chunk's writes (only the pixels that
// passed z) are posted and go out FCFS; the pixel stage waits when rdp_dw chunks' writes
// are outstanding. rdp_sb=1: a span's first read waits for all earlier writes (no
// read-before-write hazard). Pipe / full syncs drain the pipeline and the writes.
// eng=0: the earlier serial replay (reads, pixels, writes in order).
// Calibrated against the console's HWCAL boot tests (tools: rdram_replay + rdpfit.py).
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
  double overlap;    // eng=0: 1 = a span's pixel cycles overlap its memory accesses (max), 0: add
  double bank_bits;  // log2 of the bank size (20 = 1 MB)
  double row_bits;   // log2 of the row size (11 = 2 KB)
  double pi_gap;     // RCP cycles between the PI's bursts
  double cmd_fetch;  // bytes per RDP command fetch
  double idle;       // RCP cycles an idle RDP needs before its first transaction
  double cpu_bus, rsp_bus; // experiments: share of their transactions' time the CPU / RSP hold the channel
  double cpu_rows, rsp_rows; // experiments: 1 = their open rows are tracked apart (never share a bank)
  double iso1_lo, iso1_hi, iso2_lo, iso2_hi, iso3_lo, iso3_hi; // experiments: address ranges (physical) with a bank of their own
  double chunk;      // pixels per RDP span chunk; 0 = whole span
  double eng;        // 1: the pipelined RDP engine, 0: the serial replay
  double rd_lat;     // eng=1: clocks from the end of an RDP read's transfer to its data in the pipeline
  double rdp_d;      // eng=1: chunks whose reads may be issued ahead of the pixel stage (>= 1)
  double rdp_dw;     // eng=1: chunks whose writes may be outstanding (0: no limit)
  double rdp_sb;     // eng=1: span barrier: 1 = a span's reads wait for earlier writes, 2 = for earlier pixels
  double rdp_rprio;  // eng=1: 1 = the RDP's reads go before its writes requested at the same time
  double span_r;     // eng=1: front-end RCP cycles per span (rasterizer)
  double tta;        // channel clocks when the direction (read / write) changes back to back
  double psync;      // eng=1: RCP cycles of a pipe sync after the drain
  double fchunk;     // eng=1: pixels per fill / copy chunk
  double fetch_block; // eng=1: 1 = the front end waits for its command fetches
  double span_rs;    // eng=1: RCP cycles per read stream (z, colour) at a span's start
  double span_ws;    // eng=1: RCP cycles per write stream at the start of a span that writes
  double align8;     // eng=1: 1 = RDP transfers cover whole octbytes, 2 = chunks aligned in memory too
  double rd_occ;     // eng=1: share of the read delay an RDP read holds the channel (the rest is latency)
  double bank_busy;  // clocks from an access's start before its bank takes the next one (0: off)
  double cpu_wbv;    // CPU cycles of a dirty victim's write-back before a D-cache refill
  double bank_rdp;   // 1: bank_busy applies between the RDP's own accesses only
  double cpu_rocc;   // extra clocks a CPU cache-line fill holds the channel
  double cpu_prio;   // >= 0: a CPU transfer cuts into another agent's after this many clocks
};
#define RDRAM_NPARAMS 51

struct rdram_stat { uint64_t n, bytes, busy, wait, hits, misses; };

struct rdram_model {
  int on;
  uint64_t now;       // RDRAM clocks
  uint64_t bus_free;  // the channel is free from here
  int last_dir;       // direction of the last transfer (0 read, 1 write, -1 none)
  int last_agent;     // agent of the last transfer (-1 none)
  int32_t open_row[32];
  uint64_t bank_free[32]; // bank_busy: the bank takes its next access from here
  int8_t row_owner[32];
  uint64_t conflict[RA_N][RA_N][8];
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
unsigned rdram_cpu_victim(uint32_t paddr);
unsigned rdram_cpu_cacheop_wb(uint32_t paddr);
unsigned rdram_cpu_uncached_write(uint32_t paddr, unsigned bytes);

// RSP DMA
void rdram_rsp_dma(uint32_t dram, uint32_t length, uint32_t count, uint32_t skip, int write);
unsigned rdram_rsp_dma_inflight(void);

// RDP work (emitted while angrylion executes the command)
void rdram_rdp_cmd_begin(uint32_t cmd, uint32_t addr, uint32_t len, int xbus);
void rdram_rdp_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                    int image_read, int z_compare, int z_update, int y, int xmin);
void rdram_px_written(int x);
void rdram_rdp_span_written(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                            int image_read, int z_compare, int z_update, int wn, int wlo, int whi);
void rdram_rdp_texload(uint32_t addr, uint32_t bytes);
void rdram_rdp_mark(unsigned ring_index);
int rdram_rdp_pending(void);
void rdram_rdp_flush(void);
uint64_t rdram_rdp_done_time(void);

// clocks, VI, PI
void rdram_tick(void);
void rdram_vi_line(uint32_t addr, uint32_t bytes);
void rdram_refresh(void);
void rdram_pi_write(uint32_t addr, uint32_t bytes);

// bench windows
void rdram_window_reset(void);
void rdram_window_report(const char *name);
#endif
