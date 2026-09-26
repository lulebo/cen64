//
// bus/rdram_model.c: RDRAM channel contention model (-rdram). See rdram_model.h.
//
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bus/rdram_model.h"
#include "rdp/cpu.h"

struct rdram_model g_rdram;
const char *g_rdram_params = NULL;
int g_rdram_enable = 0;
struct rdp *g_rdram_rdp = NULL;

static const char *agent_name[RA_N] = {
  "cpu_i", "cpu_d", "cpu_wb", "cpu_unc", "rsp", "rdp_cmd", "rdp_tex", "rdp_fb", "rdp_z", "vi", "pi", "refresh"
};

#define P(x) { #x, offsetof(struct rdram_params, x) }
static const struct { const char *name; size_t off; } param_tab[] = {
  P(req), P(rdelay), P(wdelay), P(per_byte), P(row_miss), P(refresh), P(refresh_close),
  P(cpu_d), P(cpu_i), P(cpu_unc), P(cpu_wb), P(cpu_uncw), P(burst), P(px1), P(px2), P(fill),
  P(copy), P(tri), P(rect), P(cmd), P(tmem8), P(sync), P(span), P(overlap), P(bank_bits),
  P(row_bits), P(pi_gap), P(cmd_fetch), P(idle), P(cpu_bus), P(rsp_bus), P(cpu_rows), P(rsp_rows)
};
#undef P
#define NPARAM (sizeof(param_tab) / sizeof(param_tab[0]))

// ---- work queues (one per agent that is not the CPU) ---------------------------------------
enum { IT_CYC, IT_RD, IT_WR, IT_MARK, IT_PIXO, IT_SPANEND, IT_DMAEND };
struct item { uint32_t addr, v; uint8_t kind, agent; };
struct queue { struct item *q; unsigned mask, head, tail; uint64_t ready, pix_until; };
enum { Q_RSP, Q_VI, Q_PI, Q_RDP, Q_N };
static struct queue queues[Q_N];
static unsigned rsp_inflight = 0;
static uint64_t uncw_done = 0;
static uint32_t fetch_lo = 0, fetch_hi = 0;
static double rdp_frac = 0; // sub-clock pixel cycles carried between items

static void queue_init(struct queue *q, unsigned log2n) {
  q->q = calloc((size_t) 1 << log2n, sizeof(struct item));
  q->mask = (1u << log2n) - 1;
  q->head = q->tail = 0;
  q->ready = q->pix_until = 0;
}

uint64_t rdram_access(uint64_t t, uint32_t addr, uint32_t bytes, int write, int agent) {
  struct rdram_model *m = &g_rdram;
  struct rdram_stat *s = &m->st[agent];
  uint64_t start = t > m->bus_free ? t : m->bus_free;
  unsigned bank = (addr >> (unsigned) m->p.bank_bits) & 15;
  if ((agent <= RA_CPU_UNC && m->p.cpu_rows > 0) || (agent == RA_RSP && m->p.rsp_rows > 0))
    bank += 16;
  int32_t row = (int32_t) (addr >> (unsigned) m->p.row_bits);
  double cost = m->p.req + (write ? m->p.wdelay : m->p.rdelay) + bytes * m->p.per_byte;
  uint64_t c;
  if (m->open_row[bank] != row) {
    cost += m->p.row_miss;
    m->open_row[bank] = row;
    s->misses++;
  } else
    s->hits++;
  c = (uint64_t) (cost + 0.5);
  if (c < 1) c = 1;
  {
    double share = agent <= RA_CPU_UNC ? m->p.cpu_bus : agent == RA_RSP ? m->p.rsp_bus : 1.0;
    uint64_t cb = (uint64_t) (c * share + 0.5);
    m->bus_free = start + cb;
    s->n++; s->bytes += bytes; s->busy += cb; s->wait += start - t;
  }
  return start + c;
}

static void mark_done(unsigned idx, uint64_t clocks) {
  if (g_rdram_rdp != NULL)
    g_rdram_rdp->timing.ring[idx & (RDP_TIMING_RING - 1)].finish = (clocks + 3) / 4;
}

static void run_queue(struct queue *q, int force) {
  struct rdram_model *m = &g_rdram;
  while (q->head != q->tail && (force || q->ready <= m->now)) {
    struct item *it = &q->q[q->head];
    switch (it->kind) {
      case IT_CYC: q->ready += it->v; break;
      case IT_RD: case IT_WR:
        q->ready = rdram_access(q->ready, it->addr, it->v, it->kind == IT_WR, it->agent);
        break;
      case IT_PIXO: { uint64_t e = q->ready + it->v; if (e > q->pix_until) q->pix_until = e; break; }
      case IT_SPANEND: if (q->pix_until > q->ready) q->ready = q->pix_until; break;
      case IT_MARK: mark_done(it->v, q->ready); break;
      case IT_DMAEND: if (rsp_inflight) rsp_inflight--; break;
    }
    q->head = (q->head + 1) & q->mask;
    if (force) return;
  }
}

static void push(int qi, int kind, int agent, uint32_t addr, uint32_t v) {
  struct queue *q = &queues[qi];
  struct item *it;
  if (q->head == q->tail && q->ready < g_rdram.now) {
    q->ready = g_rdram.now;
    // the RDP pays its command fetch latency again after having been idle
    if (qi == Q_RDP) q->ready += (uint64_t) (g_rdram.p.idle * 4);
  }
  if (((q->tail + 1) & q->mask) == q->head) {
    static int warned = 0;
    if (!warned) { warned = 1; fprintf(stderr, "rdram model: queue %d full, forcing\n", qi); }
    run_queue(q, 1);
  }
  it = &q->q[q->tail];
  it->addr = addr; it->v = v; it->kind = (uint8_t) kind; it->agent = (uint8_t) agent;
  q->tail = (q->tail + 1) & q->mask;
}

// Split a transfer into bursts that stay inside one row and are at most p.burst bytes.
static void push_mem(int qi, int write, int agent, uint32_t addr, uint32_t bytes) {
  uint32_t rowsz = 1u << (unsigned) g_rdram.p.row_bits;
  uint32_t burst = (uint32_t) g_rdram.p.burst;
  if (burst < 8) burst = 8;
  while (bytes > 0) {
    uint32_t n = bytes < burst ? bytes : burst;
    uint32_t to_row = rowsz - (addr & (rowsz - 1));
    if (n > to_row) n = to_row;
    push(qi, write ? IT_WR : IT_RD, agent, addr & 0x7FFFFF, n);
    addr += n; bytes -= n;
  }
}

static void push_cycles(int qi, double rcp_cycles) {
  double clocks = rcp_cycles * 4 + rdp_frac;
  uint32_t c = (uint32_t) clocks;
  rdp_frac = clocks - c;
  if (c) push(qi, IT_CYC, 0, 0, c);
}

// ---- CPU ----------------------------------------------------------------------------------------
// Idle-channel row-hit service of a transfer, in RDRAM clocks.
static double service(unsigned bytes, int write) {
  return g_rdram.p.req + (write ? g_rdram.p.wdelay : g_rdram.p.rdelay) + bytes * g_rdram.p.per_byte;
}

unsigned rdram_cpu_read(uint32_t paddr, unsigned bytes, int agent) {
  struct rdram_model *m = &g_rdram;
  uint64_t t = m->now, end = rdram_access(t, paddr, bytes, 0, agent);
  double fixed = agent == RA_CPU_I ? m->p.cpu_i : agent == RA_CPU_D ? m->p.cpu_d : m->p.cpu_unc;
  unsigned stall = (unsigned) (fixed + (end - t) * 0.375 + 0.5);
  m->cpu_stall += stall;
  m->cpu_idle += (uint64_t) (fixed + service(bytes, 0) * 0.375 + 0.5);
  return stall;
}

void rdram_cpu_victim(uint32_t paddr) {
  rdram_access(g_rdram.now, paddr, 16, 1, RA_CPU_WB);
}

unsigned rdram_cpu_cacheop_wb(uint32_t paddr) {
  struct rdram_model *m = &g_rdram;
  uint64_t t = m->now, end = rdram_access(t, paddr, 16, 1, RA_CPU_WB);
  unsigned stall = (unsigned) (m->p.cpu_wb + (end - t) * 0.375 + 0.5);
  m->cpu_stall += stall;
  m->cpu_idle += (uint64_t) (m->p.cpu_wb + service(16, 1) * 0.375 + 0.5);
  return stall;
}

// One-entry write buffer: the store waits only for the previous uncached write.
unsigned rdram_cpu_uncached_write(uint32_t paddr, unsigned bytes) {
  struct rdram_model *m = &g_rdram;
  uint64_t t = m->now, w = uncw_done > t ? uncw_done - t : 0;
  unsigned stall;
  uncw_done = rdram_access(t + w, paddr, bytes, 1, RA_CPU_UNC);
  stall = (unsigned) (m->p.cpu_uncw + w * 0.375 + 0.5);
  m->cpu_stall += stall;
  m->cpu_idle += (uint64_t) (m->p.cpu_uncw + 0.5);
  return stall;
}

// ---- RSP DMA --------------------------------------------------------------------------------------
void rdram_rsp_dma(uint32_t dram, uint32_t length, uint32_t count, uint32_t skip, int write) {
  uint32_t i;
  for (i = 0; i <= count; i++) {
    push_mem(Q_RSP, write, RA_RSP, dram, length);
    dram += length + skip;
  }
  push(Q_RSP, IT_DMAEND, 0, 0, 0);
  rsp_inflight++;
}

unsigned rdram_rsp_dma_inflight(void) { return rsp_inflight; }

// ---- RDP --------------------------------------------------------------------------------------------
void rdram_rdp_cmd_begin(uint32_t cmd, uint32_t addr, uint32_t len, int xbus) {
  struct rdram_params *p = &g_rdram.p;
  double cyc = p->cmd;
  if (!xbus) {
    uint32_t end = addr + len;
    if (addr < fetch_lo || end > fetch_hi) {
      uint32_t from = (addr >= fetch_lo && addr < fetch_hi) ? fetch_hi : addr;
      uint32_t chunk = (uint32_t) p->cmd_fetch, to;
      if (chunk < 8) chunk = 8;
      to = (end + chunk - 1) / chunk * chunk;
      if (from < to) push_mem(Q_RDP, 0, RA_RDP_CMD, from, to - from);
      if (from == addr) fetch_lo = addr;
      fetch_hi = to;
    }
  }
  if (cmd >= 0x08 && cmd <= 0x0F) cyc += p->tri;
  else if (cmd == 0x24 || cmd == 0x25 || cmd == 0x36) cyc += p->rect;
  else if (cmd == 0x29) cyc += p->sync;
  push_cycles(Q_RDP, cyc);
}

// cycle_type: 0 1-cycle, 1 2-cycle, 2 copy, 3 fill; bpp4 = bits per pixel / 4
void rdram_rdp_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                    int image_read, int z_compare, int z_update) {
  struct rdram_params *p = &g_rdram.p;
  uint32_t fbbytes, zbytes;
  double pc;
  if (len <= 0) return;
  fbbytes = ((uint32_t) len * (uint32_t) bpp4 + 1) / 2;
  zbytes = (uint32_t) len * 2;
  if (cycle_type >= 2) {
    pc = len * (cycle_type == 3 ? p->fill * (bpp4 >= 8 ? 2 : 1) : p->copy) + p->span;
    if (p->overlap > 0) {
      push(Q_RDP, IT_PIXO, 0, 0, (uint32_t) (pc * 4));
      push_mem(Q_RDP, 1, RA_RDP_FB, fb, fbbytes);
      push(Q_RDP, IT_SPANEND, 0, 0, 0);
    } else {
      push_cycles(Q_RDP, pc);
      push_mem(Q_RDP, 1, RA_RDP_FB, fb, fbbytes);
    }
    return;
  }
  pc = len * (cycle_type == 1 ? p->px2 : p->px1) + p->span;
  if (p->overlap > 0) push(Q_RDP, IT_PIXO, 0, 0, (uint32_t) (pc * 4));
  if (z_compare) push_mem(Q_RDP, 0, RA_RDP_Z, z, zbytes);
  if (image_read) push_mem(Q_RDP, 0, RA_RDP_FB, fb, fbbytes);
  if (p->overlap <= 0) push_cycles(Q_RDP, pc);
  push_mem(Q_RDP, 1, RA_RDP_FB, fb, fbbytes);
  if (z_update) push_mem(Q_RDP, 1, RA_RDP_Z, z, zbytes);
  if (p->overlap > 0) push(Q_RDP, IT_SPANEND, 0, 0, 0);
}

void rdram_rdp_texload(uint32_t addr, uint32_t bytes) {
  if (bytes == 0) return;
  push_mem(Q_RDP, 0, RA_RDP_TEX, addr, bytes);
  push_cycles(Q_RDP, (bytes + 7) / 8 * g_rdram.p.tmem8);
}

void rdram_rdp_mark(unsigned ring_index) { push(Q_RDP, IT_MARK, 0, 0, ring_index); }

int rdram_rdp_pending(void) {
  struct queue *q = &queues[Q_RDP];
  return q->head != q->tail || q->ready > g_rdram.now;
}

void rdram_rdp_flush(void) {
  while (queues[Q_RDP].head != queues[Q_RDP].tail) run_queue(&queues[Q_RDP], 1);
}

// ---- clock, VI, refresh, PI ---------------------------------------------------------------------
void rdram_tick(void) {
  struct rdram_model *m = &g_rdram;
  int i;
  m->now += 4;
  for (i = 0; i < Q_N; i++)
    if (queues[i].head != queues[i].tail) run_queue(&queues[i], 0);
  if (queues[Q_RDP].head != queues[Q_RDP].tail || queues[Q_RDP].ready > m->now) { m->rdp_busy += 4; m->rdp_busy_total += 4; }
  if (rsp_inflight) m->rsp_dma_busy += 4;
}

void rdram_vi_line(uint32_t addr, uint32_t bytes) { push_mem(Q_VI, 0, RA_VI, addr, bytes); }

void rdram_refresh(void) {
  struct rdram_model *m = &g_rdram;
  struct rdram_stat *s = &m->st[RA_REFRESH];
  uint64_t start = m->now > m->bus_free ? m->now : m->bus_free;
  uint64_t c = (uint64_t) (m->p.refresh + 0.5);
  m->bus_free = start + c;
  s->n++; s->busy += c; s->wait += start - m->now;
  if (m->p.refresh_close > 0) {
    int i;
    for (i = 0; i < 32; i++) m->open_row[i] = -1;
  }
}

void rdram_pi_write(uint32_t addr, uint32_t bytes) {
  while (bytes > 0) {
    uint32_t n = bytes < 128 ? bytes : 128;
    push_mem(Q_PI, 1, RA_PI, addr, n);
    push(Q_PI, IT_CYC, 0, 0, (uint32_t) (g_rdram.p.pi_gap * 4));
    addr += n; bytes -= n;
  }
}

// ---- setup and reporting ----------------------------------------------------------------------
void rdram_model_init(void) {
  struct rdram_params d = {
    // Calibrated 2026-09-26 against a ModRetro M64 (firmware 1.8.0 beta): BoB and castle grounds spawn
    // views, overlap and no-overlap pipelines (F C R P of the HWSTATS line within ~7% rms). RDRAM-datasheet
    // style values instead: rdelay=7,row_miss=25,span=4,cpu_d=39.25,cpu_i=40.25,cpu_unc=29.5,px1=1,px2=2.
    3, 20, 1, 0.5, 40, 54, 1,           // req rdelay wdelay per_byte row_miss refresh refresh_close
    45, 46, 24.6, 39.5, 1,              // cpu_d cpu_i cpu_unc cpu_wb cpu_uncw (idle row hit = 56.6/60.6/34/44)
    128,                                // burst
    1.08, 2.16, 0.25, 0.25,             // px1 px2 fill copy
    32, 16, 2, 1, 200, 8,               // tri rect cmd tmem8 sync span
    0, 20, 11, 1591, 64, 64,            // overlap bank_bits row_bits pi_gap cmd_fetch idle
    1, 1,                               // cpu_bus rsp_bus (experiments: share of the channel time they occupy)
    0, 0                                // cpu_rows rsp_rows (experiments: separate open-row state)
  };
  const char *s = g_rdram_params ? g_rdram_params : getenv("CEN64_RDRAM_MODEL");
  int i;
  memset(&g_rdram, 0, sizeof(g_rdram));
  g_rdram.p = d;
  for (i = 0; i < 32; i++) g_rdram.open_row[i] = -1;
  while (s && *s) {
    char key[32]; size_t k = 0; unsigned j;
    while (*s && *s != '=' && *s != ',' && k < sizeof(key) - 1) key[k++] = *s++;
    key[k] = 0;
    if (*s == '=') {
      double v = atof(++s);
      for (j = 0; j < NPARAM; j++)
        if (!strcmp(key, param_tab[j].name)) { *(double *) ((char *) &g_rdram.p + param_tab[j].off) = v; break; }
      if (j == NPARAM) fprintf(stderr, "rdram model: unknown parameter '%s'\n", key);
    }
    while (*s && *s != ',') s++;
    if (*s == ',') s++;
  }
  queue_init(&queues[Q_RSP], 16);
  queue_init(&queues[Q_VI], 13);
  queue_init(&queues[Q_PI], 15);
  queue_init(&queues[Q_RDP], 22);
  g_rdram.on = 1;
  fprintf(stderr, "RDRAM model:");
  for (i = 0; i < (int) NPARAM; i++)
    fprintf(stderr, " %s=%g", param_tab[i].name, *(double *) ((char *) &g_rdram.p + param_tab[i].off));
  fprintf(stderr, "\n");
}

void rdram_window_reset(void) {
  struct rdram_model *m = &g_rdram;
  memset(m->st, 0, sizeof(m->st));
  m->cpu_stall = m->cpu_idle = m->rdp_busy = m->rsp_dma_busy = 0;
  m->window_start = m->now;
}

// RDRAM,<name>,<window clocks>,<cpu stall>,<cpu idle-channel stall>,<rdp busy clocks>,<rsp dma busy clocks>,
// then per agent: <name>:<n>:<bytes>:<busy>:<wait>:<hits>:<misses>
void rdram_window_report(const char *name) {
  struct rdram_model *m = &g_rdram;
  int i;
  printf("RDRAM,%s,%llu,%llu,%llu,%llu,%llu", name, (unsigned long long) (m->now - m->window_start),
         (unsigned long long) m->cpu_stall, (unsigned long long) m->cpu_idle,
         (unsigned long long) m->rdp_busy, (unsigned long long) m->rsp_dma_busy);
  for (i = 0; i < RA_N; i++) {
    struct rdram_stat *s = &m->st[i];
    printf(",%s:%llu:%llu:%llu:%llu:%llu:%llu", agent_name[i], (unsigned long long) s->n,
           (unsigned long long) s->bytes, (unsigned long long) s->busy, (unsigned long long) s->wait,
           (unsigned long long) s->hits, (unsigned long long) s->misses);
  }
  printf("\n");
}
