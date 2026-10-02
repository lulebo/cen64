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
  P(row_bits), P(pi_gap), P(cmd_fetch), P(idle), P(cpu_bus), P(rsp_bus), P(cpu_rows), P(rsp_rows),
  P(iso1_lo), P(iso1_hi), P(iso2_lo), P(iso2_hi), P(iso3_lo), P(iso3_hi), P(chunk),
  P(eng), P(rd_lat), P(rdp_d), P(rdp_dw), P(rdp_sb), P(rdp_rprio), P(span_r), P(tta), P(psync),
  P(fchunk), P(fetch_block), P(span_rs), P(span_ws), P(align8), P(rd_occ), P(bank_busy), P(cpu_wbv), P(bank_rdp), P(cpu_rocc), P(cpu_prio), P(span_wfull), P(ew_line), P(row_xmiss), P(vi_rows), P(vi_burst)
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

// The span being rendered: its memory items wait until its pixels are done (written count and range).
static struct {
  int valid, len, bpp4, ct, ir, zc, zu, y, xmin, n, lo, hi;
  uint32_t fb, z;
} pend;
static FILE *span_trace = NULL;
static void span_flush(void);
static void span_emit(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type, int image_read,
                      int z_compare, int z_update, int wlo, int whi);
static void eng_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type, int image_read,
                     int z_compare, int z_update, int wlo, int whi);
static void eng_run(uint64_t limit);
static int eng_busy(void);

static void queue_init(struct queue *q, unsigned log2n) {
  if (q->q == NULL) q->q = calloc((size_t) 1 << log2n, sizeof(struct item));
  q->mask = (1u << log2n) - 1;
  q->head = q->tail = 0;
  q->ready = q->pix_until = 0;
}

uint64_t rdram_access(uint64_t t, uint32_t addr, uint32_t bytes, int write, int agent) {
  struct rdram_model *m = &g_rdram;
  struct rdram_stat *s = &m->st[agent];
  uint64_t start = t > m->bus_free ? t : m->bus_free;
  int cut = 0; // a CPU transfer cutting into another agent's
  if (agent <= RA_CPU_UNC && m->p.cpu_prio >= 0 && t < m->bus_free && m->last_agent > RA_CPU_UNC) {
    uint64_t at = t + (uint64_t) (m->p.cpu_prio + 0.5);
    if (at < m->bus_free) { start = at; cut = 1; }
  }
  unsigned bank = (addr >> (unsigned) m->p.bank_bits) & 15;
  if ((agent <= RA_CPU_UNC && m->p.cpu_rows > 0) || (agent == RA_RSP && m->p.rsp_rows > 0)
      || (agent == RA_VI && m->p.vi_rows > 0))
    bank += 16;
  else if (addr >= (uint32_t) m->p.iso1_lo && addr < (uint32_t) m->p.iso1_hi) bank = 29;
  else if (addr >= (uint32_t) m->p.iso2_lo && addr < (uint32_t) m->p.iso2_hi) bank = 30;
  else if (addr >= (uint32_t) m->p.iso3_lo && addr < (uint32_t) m->p.iso3_hi) bank = 31;
  int32_t row = (int32_t) (addr >> (unsigned) m->p.row_bits);
  double cost = m->p.req + (write ? m->p.wdelay : m->p.rdelay) + bytes * m->p.per_byte;
  int rdp_agent = agent >= RA_RDP_CMD && agent <= RA_RDP_Z;
  int use_bank = m->p.bank_busy > 0 && (m->p.bank_rdp <= 0 || rdp_agent);
  if (use_bank && m->bank_free[bank] > start) start = m->bank_free[bank];
  if (!write && (agent == RA_CPU_D || agent == RA_CPU_I)) cost += m->p.cpu_rocc;
  uint64_t c;
  if (m->open_row[bank] != row) {
    cost += m->p.row_miss;
    if (m->row_owner[bank] >= 0 && m->row_owner[bank] != agent) cost += m->p.row_xmiss;
    if (m->open_row[bank] >= 0 && m->row_owner[bank] >= 0)
      m->conflict[agent][m->row_owner[bank]][(addr >> 20) & 7]++;
    m->open_row[bank] = row;
    s->misses++;
  } else
    s->hits++;
  // a back-to-back transfer in the other direction turns the channel around
  if (m->p.tta > 0 && m->last_dir >= 0 && m->last_dir != write && t <= m->bus_free) cost += m->p.tta;
  m->last_dir = write;
  m->row_owner[bank] = (int8_t) agent;
  c = (uint64_t) (cost + 0.5);
  if (c < 1) c = 1;
  {
    double share = agent <= RA_CPU_UNC ? m->p.cpu_bus : agent == RA_RSP ? m->p.rsp_bus : 1.0;
    uint64_t cb = (uint64_t) (c * share + 0.5);
    if (!write && agent >= RA_RDP_CMD && agent <= RA_RDP_Z && m->p.eng > 0 && m->p.rd_occ < 1) {
      double occ = cost - (1 - (m->p.rd_occ < 0 ? 0 : m->p.rd_occ)) * m->p.rdelay;
      cb = (uint64_t) (occ + 0.5);
      if (cb < 1) cb = 1;
    }
    if (cut) m->bus_free += cb; // the interrupted transfer finishes after the CPU's
    else m->bus_free = start + cb;
    m->last_agent = agent;
    if (use_bank) m->bank_free[bank] = start + (c > m->p.bank_busy ? c : (uint64_t) (m->p.bank_busy + 0.5));
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

static uint64_t cpu_after = 0; // a dirty victim's write-back: the refill waits for it

unsigned rdram_cpu_read(uint32_t paddr, unsigned bytes, int agent) {
  struct rdram_model *m = &g_rdram;
  uint64_t t = m->now, end = rdram_access(cpu_after > t ? cpu_after : t, paddr, bytes, 0, agent);
  cpu_after = 0;
  double fixed = agent == RA_CPU_I ? m->p.cpu_i : agent == RA_CPU_D ? m->p.cpu_d : m->p.cpu_unc;
  unsigned stall = (unsigned) (fixed + (end - t) * 0.375 + 0.5);
  m->cpu_stall += stall;
  m->cpu_idle += (uint64_t) (fixed + service(bytes, 0) * 0.375 + 0.5);
  return stall;
}

// before the refill (rdram_cpu_read): the CPU waits for the write-back, the refill follows it
unsigned rdram_cpu_victim(uint32_t paddr) {
  struct rdram_model *m = &g_rdram;
  cpu_after = rdram_access(m->now, paddr, 16, 1, RA_CPU_WB);
  m->cpu_stall += (uint64_t) (m->p.cpu_wbv + 0.5);
  return (unsigned) (m->p.cpu_wbv + 0.5);
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
  uint64_t t = m->now, w = uncw_done > t ? uncw_done - t : 0, at = t + w;
  uint64_t lat = (uint64_t) (m->p.cpu_uncw * 8.0 / 3.0 + 0.5); // CPU cycles -> RDRAM clocks
  unsigned stall, k, words = bytes > 4 ? 2 : 1;
  for (k = 0; k < words; k++)
    at = rdram_access(at, paddr + 4 * k, bytes > 4 ? 4 : bytes, 1, RA_CPU_UNC) + lat;
  uncw_done = at;
  stall = (unsigned) (w * 0.375 + 0.5);
  m->cpu_stall += stall;
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

// ---- RDP engine (eng=1) -----------------------------------------------------------------------------
// Items in RDP order. The front end walks them (command costs, syncs, fetches, texture loads, a chunk's
// read requests), the pixel stage processes chunks, the write stage posts their writes; an item retires
// when all of it is done, a mark when everything before it has retired.
enum { RK_CHUNK, RK_FRONT, RK_DRAIN, RK_LOAD, RK_FETCH, RK_MARK };
#define RF_SPAN0 1
#define RF_PIXONLY 2
#define TNONE UINT64_MAX
#define RMAX 4
struct rwork {
  uint8_t kind, flags, nr, nw;
  uint8_t ra[RMAX], wa[RMAX];
  uint32_t raddr[RMAX], rbytes[RMAX], waddr[RMAX], wbytes[RMAX];
  uint32_t clk;                  // chunk: pixel clocks; front / drain: clocks; load: TMEM clocks; mark: ring index
  uint64_t arrive;               // when the emulator produced it
  uint64_t rdone, pend, wdone;   // reads' data in, pixels done, writes done (TNONE until known)
};
#define ENG_LOG2 18
static struct {
  struct rwork *w;
  unsigned mask, head, tail;     // head: oldest item not retired
  unsigned ri, pi, wi;           // next item for the front end / the pixel stage / the write stage
  uint64_t front, pipe, wmax, retired;
  unsigned cs_r, cs_p, cs_w;     // chunk sequence numbers: reads issued / processed / writes issued
  uint64_t pend_seq[64], wdone_seq[64];
  int load_active;               // the front end is in the middle of a texture load
} E;

static void eng_init(void) {
  if (E.w == NULL) E.w = calloc((size_t) 1 << ENG_LOG2, sizeof(struct rwork));
  E.mask = (1u << ENG_LOG2) - 1;
  E.head = E.tail = E.ri = E.pi = E.wi = 0;
  E.front = E.pipe = E.wmax = E.retired = 0;
  E.cs_r = E.cs_p = E.cs_w = 0;
  E.load_active = 0;
}

static int eng_busy(void) {
  uint64_t now = g_rdram.now;
  return E.head != E.tail || E.retired > now;
}

uint64_t rdram_rdp_done_time(void) { return g_rdram.p.eng > 0 ? E.retired : queues[Q_RDP].ready; }

static struct rwork *eng_new(int kind) {
  struct rwork *w;
  if (((E.tail + 1) & E.mask) == E.head) {
    static int warned = 0;
    if (!warned) { warned = 1; fprintf(stderr, "rdram model: RDP engine full, forcing\n"); }
    while (((E.tail + 1) & E.mask) == E.head) eng_run(TNONE);
  }
  w = &E.w[E.tail];
  memset(w, 0, sizeof(*w));
  w->kind = (uint8_t) kind;
  w->arrive = g_rdram.now;
  if (E.head == E.tail && E.retired <= g_rdram.now)
    w->arrive += (uint64_t) (g_rdram.p.idle * 4); // command fetch latency after an idle RDP
  w->rdone = w->pend = w->wdone = TNONE;
  E.tail = (E.tail + 1) & E.mask;
  return w;
}

// a transfer split at row boundaries into the item's read or write slots
static void eng_mem(struct rwork *w, int write, int agent, uint32_t addr, uint32_t bytes) {
  uint32_t rowsz = 1u << (unsigned) g_rdram.p.row_bits;
  addr &= 0x7FFFFF;
  if (g_rdram.p.align8 > 0 && bytes > 0) { // whole octbytes
    uint32_t end = (addr + bytes + 7) & ~7u;
    addr &= ~7u;
    bytes = end - addr;
  }
  while (bytes > 0) {
    uint32_t n = bytes, to_row = rowsz - (addr & (rowsz - 1));
    if (n > to_row) n = to_row;
    if (write) {
      if (w->nw < RMAX) { w->waddr[w->nw] = addr; w->wbytes[w->nw] = n; w->wa[w->nw] = (uint8_t) agent; w->nw++; }
    } else {
      if (w->nr < RMAX) { w->raddr[w->nr] = addr; w->rbytes[w->nr] = n; w->ra[w->nr] = (uint8_t) agent; w->nr++; }
    }
    addr += n; bytes -= n;
  }
}

static void eng_front(double rcp_cycles) {
  struct rwork *w;
  uint32_t c = (uint32_t) (rcp_cycles * 4 + 0.5);
  if (c == 0) return;
  w = eng_new(RK_FRONT);
  w->clk = c;
}

// The front end's next request time for item ri (TNONE: blocked on the pixel or write stage).
static uint64_t eng_front_ready(struct rwork *w) {
  struct rdram_params *p = &g_rdram.p;
  uint64_t t = E.front > w->arrive ? E.front : w->arrive;
  if (w->kind == RK_CHUNK) {
    unsigned d = p->rdp_d < 1 ? 1 : (unsigned) p->rdp_d;
    if (d > 32) d = 32;
    if (w->flags & RF_SPAN0) t += (uint64_t) (p->span_r * 4 + 0.5);
    if (E.cs_r >= d) {
      unsigned s = E.cs_r - d;
      if (E.cs_p <= s) return TNONE;
      if (E.pend_seq[s & 63] > t) t = E.pend_seq[s & 63];
    }
    if ((w->flags & RF_SPAN0) && p->rdp_sb > 0 && E.cs_r > 0) {
      if (p->rdp_sb >= 2) {
        if (E.cs_p < E.cs_r) return TNONE;
        if (E.pipe > t) t = E.pipe;
      } else {
        if (E.wi != E.ri) return TNONE;
        if (E.wmax > t) t = E.wmax;
      }
    }
  } else if (w->kind == RK_DRAIN) {
    if (E.pi != E.ri) return TNONE;
    if (E.pipe > t) t = E.pipe;
    if (!(w->flags & RF_PIXONLY)) {
      if (E.wi != E.ri) return TNONE;
      if (E.wmax > t) t = E.wmax;
    }
  }
  return t;
}

// Run the engine up to `limit` (RDRAM clocks): every channel request with a request time <= limit is issued,
// in request-time order (the front end's reads against the write stage's writes).
static void eng_run(uint64_t limit) {
  struct rdram_params *p = &g_rdram.p;
  for (;;) {
    int progress = 0, chan;
    uint64_t tr = TNONE, tw = TNONE;
    struct rwork *fw = NULL, *ww = NULL;
    // pixel stage: chunks whose reads are issued
    while (E.pi != E.ri) {
      struct rwork *w = &E.w[E.pi];
      if (w->kind == RK_CHUNK) {
        uint64_t t = w->rdone;
        unsigned dw = p->rdp_dw < 1 ? 0 : (unsigned) p->rdp_dw;
        if (dw > 32) dw = 32;
        if (E.pipe > t) t = E.pipe;
        if (dw && E.cs_p >= dw) {
          unsigned s = E.cs_p - dw;
          if (E.cs_w <= s) break;
          if (E.wdone_seq[s & 63] > t) t = E.wdone_seq[s & 63];
        }
        if (w->flags & RF_SPAN0) t += (uint64_t) (p->span * 4 + 0.5);
        w->pend = t + w->clk;
        E.pipe = w->pend;
        E.pend_seq[E.cs_p & 63] = w->pend;
        E.cs_p++;
      }
      E.pi = (E.pi + 1) & E.mask;
      progress = 1;
    }
    // write stage: items the pixel stage is done with
    while (E.wi != E.pi) {
      struct rwork *w = &E.w[E.wi];
      if (w->kind == RK_CHUNK && w->nw > 0) break; // a channel request: below
      if (w->kind == RK_CHUNK) {
        w->wdone = w->pend;
        if (w->wdone > E.wmax) E.wmax = w->wdone;
        E.wdone_seq[E.cs_w & 63] = w->wdone;
        E.cs_w++;
      }
      E.wi = (E.wi + 1) & E.mask;
      progress = 1;
    }
    if (E.wi != E.pi) { ww = &E.w[E.wi]; tw = ww->pend; }
    // front end: non-channel items are taken at once, a channel item gives its request time
    while (E.ri != E.tail) {
      struct rwork *w = &E.w[E.ri];
      uint64_t t;
      if (w->kind == RK_MARK) { E.ri = (E.ri + 1) & E.mask; progress = 1; continue; }
      t = eng_front_ready(w);
      if (t == TNONE) break;
      if (w->kind == RK_FRONT || w->kind == RK_DRAIN) {
        E.front = t + w->clk;
        w->pend = E.front;
        E.ri = (E.ri + 1) & E.mask;
        progress = 1;
        continue;
      }
      if (w->kind == RK_CHUNK && w->nr == 0) {
        w->rdone = t;
        E.front = t;
        E.cs_r++;
        E.ri = (E.ri + 1) & E.mask;
        progress = 1;
        continue;
      }
      fw = w; tr = t;
      break;
    }
    // one transfer at a time: the RDP asks for the channel only when it is free (other agents get their turn);
    // the earlier request goes first
    chan = !(limit != TNONE && g_rdram.bus_free > limit);
    if (chan && fw != NULL && (tw == TNONE || tr < tw || (tr == tw && p->rdp_rprio > 0)) && tr <= limit) {
      struct rwork *w = fw;
      if (w->kind == RK_CHUNK) {
        uint64_t done = tr;
        int i;
        for (i = 0; i < w->nr; i++) {
          uint64_t e = rdram_access(tr, w->raddr[i], w->rbytes[i], 0, w->ra[i]) + (uint64_t) (p->rd_lat + 0.5);
          if (e > done) done = e;
        }
        w->rdone = done;
        E.front = tr;
        E.cs_r++;
        E.ri = (E.ri + 1) & E.mask;
      } else if (w->kind == RK_FETCH) {
        uint64_t done = tr;
        int i;
        for (i = 0; i < w->nr; i++) {
          uint64_t e = rdram_access(tr, w->raddr[i], w->rbytes[i], 0, w->ra[i]) + (uint64_t) (p->rd_lat + 0.5);
          if (e > done) done = e;
        }
        w->pend = done;
        E.front = p->fetch_block > 0 ? done : tr;
        E.ri = (E.ri + 1) & E.mask;
      } else { // RK_LOAD: one burst per request, each after the previous one's data
        uint32_t burst = (uint32_t) p->burst, n;
        uint64_t e;
        if (burst < 8) burst = 8;
        n = w->rbytes[0] < burst ? w->rbytes[0] : burst;
        {
          uint32_t rowsz = 1u << (unsigned) p->row_bits, to_row = rowsz - (w->raddr[0] & (rowsz - 1));
          if (n > to_row) n = to_row;
        }
        e = rdram_access(tr, w->raddr[0], n, 0, RA_RDP_TEX) + (uint64_t) (p->rd_lat + 0.5);
        w->raddr[0] += n; w->rbytes[0] -= n;
        E.front = e;
        if (w->rbytes[0] == 0) {
          E.front = e + w->clk;
          w->pend = E.front;
          E.ri = (E.ri + 1) & E.mask;
        }
      }
      progress = 1;
    } else if (chan && ww != NULL && tw <= limit) {
      struct rwork *w = ww;
      uint64_t done = tw;
      int i;
      for (i = 0; i < w->nw; i++) {
        uint64_t e = rdram_access(tw, w->waddr[i], w->wbytes[i], 1, w->wa[i]);
        if (e > done) done = e;
      }
      w->wdone = done;
      if (done > E.wmax) E.wmax = done;
      E.wdone_seq[E.cs_w & 63] = done;
      E.cs_w++;
      E.wi = (E.wi + 1) & E.mask;
      progress = 1;
    }
    // retire
    while (E.head != E.wi) {
      struct rwork *w = &E.w[E.head];
      uint64_t done;
      if (w->kind == RK_MARK) {
        mark_done(w->clk, E.retired);
      } else {
        done = w->kind == RK_CHUNK ? (w->wdone > w->pend ? w->wdone : w->pend) : w->pend;
        if (done != TNONE && done > E.retired) E.retired = done;
      }
      E.head = (E.head + 1) & E.mask;
      progress = 1;
    }
    if (!progress) break;
  }
}

// cycle_type: 0 1-cycle, 1 2-cycle, 2 copy, 3 fill; bpp4 = bits per pixel / 4
static void eng_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type, int image_read,
                     int z_compare, int z_update, int wlo, int whi) {
  struct rdram_params *p = &g_rdram.p;
  double cpp = cycle_type == 0 ? p->px1 : cycle_type == 1 ? p->px2
             : cycle_type == 3 ? p->fill * (bpp4 >= 8 ? 2 : 1) : p->copy;
  int chunk = (int) (cycle_type >= 2 ? p->fchunk : p->chunk), off, nxt;
  int streams_r = cycle_type < 2 ? (z_compare != 0) + (image_read != 0) : 0;
  int streams_w = whi >= wlo ? 1 + (z_update && cycle_type < 2) : 0;
  if (chunk <= 0 || chunk > len) chunk = len;
  for (off = 0; off < len; off = nxt) {
    int n;
    struct rwork *w;
    double c;
    nxt = off + chunk;
    if (p->align8 >= 2 && chunk < len) { // chunk boundaries at multiples of the chunk in memory (16-bit pixels)
      uint32_t pa = (fb / 2) + (uint32_t) off;
      nxt = off + (chunk - (int) (pa % (uint32_t) chunk));
    }
    if (nxt > len) nxt = len;
    n = nxt - off;
    w = eng_new(RK_CHUNK);
    c = n * cpp * 4;
    if (off == 0) {
      w->flags |= RF_SPAN0;
      c += (streams_r * p->span_rs + streams_w * p->span_ws) * 4;
    }
    if (cycle_type < 2) {
      if (z_compare) eng_mem(w, 0, RA_RDP_Z, z + (uint32_t) off * 2, (uint32_t) n * 2);
      if (image_read) eng_mem(w, 0, RA_RDP_FB, fb + ((uint32_t) off * (uint32_t) bpp4) / 2,
                              ((uint32_t) n * (uint32_t) bpp4 + 1) / 2);
    }
    if (whi >= off && wlo < off + n) {
      int a = wlo > off ? wlo : off, b = whi < off + n - 1 ? whi : off + n - 1;
      eng_mem(w, 1, RA_RDP_FB, fb + ((uint32_t) a * (uint32_t) bpp4) / 2,
              ((uint32_t) (b - a + 1) * (uint32_t) bpp4 + 1) / 2);
      if (z_update && cycle_type < 2) eng_mem(w, 1, RA_RDP_Z, z + (uint32_t) a * 2, (uint32_t) (b - a + 1) * 2);
    }
    w->clk = (uint32_t) (c + 0.5);
  }
}

// ---- RDP --------------------------------------------------------------------------------------------
void rdram_rdp_cmd_begin(uint32_t cmd, uint32_t addr, uint32_t len, int xbus) {
  struct rdram_params *p = &g_rdram.p;
  span_flush();
  if (span_trace) { fprintf(span_trace, "C %x %x %u %d\n", cmd, addr, len, xbus); if (cmd == 0x29) fflush(span_trace); }
  double cyc = p->cmd;
  if (!xbus) {
    uint32_t end = addr + len;
    if (addr < fetch_lo || end > fetch_hi) {
      uint32_t from = (addr >= fetch_lo && addr < fetch_hi) ? fetch_hi : addr;
      uint32_t chunk = (uint32_t) p->cmd_fetch, to;
      if (chunk < 8) chunk = 8;
      to = (end + chunk - 1) / chunk * chunk;
      if (from < to) {
        if (p->eng > 0) {
          uint32_t a = from;
          while (a < to) {
            uint32_t n = to - a < chunk ? to - a : chunk;
            struct rwork *w = eng_new(RK_FETCH);
            eng_mem(w, 0, RA_RDP_CMD, a, n);
            a += n;
          }
        } else
          push_mem(Q_RDP, 0, RA_RDP_CMD, from, to - from);
      }
      if (from == addr) fetch_lo = addr;
      fetch_hi = to;
    }
  }
  if (cmd >= 0x08 && cmd <= 0x0F) cyc += p->tri;
  else if (cmd == 0x24 || cmd == 0x25 || cmd == 0x36) cyc += p->rect;
  else if (cmd == 0x29) cyc += p->sync;
  if (p->eng > 0) {
    if (cmd == 0x27 || cmd == 0x29) { // pipe sync, full sync: the pixels and their writes drain
      struct rwork *w = eng_new(RK_DRAIN);
      w->clk = (uint32_t) ((cyc + (cmd == 0x27 ? p->psync : 0)) * 4 + 0.5);
    } else
      eng_front(cyc);
    return;
  }
  push_cycles(Q_RDP, cyc);
}

void rdram_px_written(int x) {
  if (pend.n == 0 || x < pend.lo) pend.lo = x;
  if (pend.n == 0 || x > pend.hi) pend.hi = x;
  pend.n++;
}

void rdram_rdp_span(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                    int image_read, int z_compare, int z_update, int y, int xmin) {
  span_flush();
  if (len <= 0) return;
  if (cycle_type >= 2) { // fill / copy: every pixel, no pixel loop to wait for
    if (span_trace) fprintf(span_trace, "S %d %d %d %d %d %d %d %d %d %d %d %x %x\n", y, xmin, len, bpp4, cycle_type, 0, 0, 0, len, 0, len - 1, fb, z);
    span_emit(fb, z, len, bpp4, cycle_type, 0, 0, 0, 0, len - 1);
    return;
  }
  pend.valid = 1; pend.fb = fb; pend.z = z; pend.len = len; pend.bpp4 = bpp4; pend.ct = cycle_type;
  pend.ir = image_read; pend.zc = z_compare; pend.zu = z_update; pend.y = y; pend.xmin = xmin;
  pend.n = 0; pend.lo = 0; pend.hi = -1;
}

// a span whose written pixels are known (the replay tool)
void rdram_rdp_span_written(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type,
                            int image_read, int z_compare, int z_update, int wn, int wlo, int whi) {
  span_flush();
  if (len <= 0) return;
  span_emit(fb, z, len, bpp4, cycle_type, image_read, z_compare, z_update, wn ? wlo : 0, wn ? whi : -1);
}

static void span_flush(void) {
  int lo, hi;
  if (!pend.valid) return;
  pend.valid = 0;
  lo = pend.n ? pend.lo - pend.xmin : 0;
  hi = pend.n ? pend.hi - pend.xmin : -1;
  if (span_trace)
    fprintf(span_trace, "S %d %d %d %d %d %d %d %d %d %d %d %x %x\n", pend.y, pend.xmin, pend.len, pend.bpp4, pend.ct,
            pend.ir, pend.zc, pend.zu, pend.n, lo, hi, pend.fb, pend.z);
  span_emit(pend.fb, pend.z, pend.len, pend.bpp4, pend.ct, pend.ir, pend.zc, pend.zu, lo, hi);
}

// One span's items. Writes cover the written pixels [wlo, whi] (offsets in the span; none if whi < wlo).
static void span_emit(uint32_t fb, uint32_t z, int len, int bpp4, int cycle_type, int image_read,
                      int z_compare, int z_update, int wlo, int whi) {
  struct rdram_params *p = &g_rdram.p;
  uint32_t fbbytes, zbytes;
  double pc;
  if (len <= 0) return;
  if (p->eng > 0) {
    if (p->span_wfull > 0 && whi >= wlo) { wlo = 0; whi = len - 1; }
    eng_span(fb, z, len, bpp4, cycle_type, image_read, z_compare, z_update, wlo, whi);
    return;
  }
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
  {
    int chunk = (int) p->chunk, off;
    if (chunk <= 0 || chunk > len) chunk = len;
    for (off = 0; off < len; off += chunk) {
      int n = len - off < chunk ? len - off : chunk;
      uint32_t fbo = ((uint32_t) off * (uint32_t) bpp4) / 2, zo = (uint32_t) off * 2;
      uint32_t fbn = ((uint32_t) n * (uint32_t) bpp4 + 1) / 2, zn = (uint32_t) n * 2;
      pc = n * (cycle_type == 1 ? p->px2 : p->px1) + (off == 0 ? p->span : 0);
      if (p->overlap > 0) push(Q_RDP, IT_PIXO, 0, 0, (uint32_t) (pc * 4));
      if (z_compare) push_mem(Q_RDP, 0, RA_RDP_Z, z + zo, zn);
      if (image_read) push_mem(Q_RDP, 0, RA_RDP_FB, fb + fbo, fbn);
      if (p->overlap <= 0) push_cycles(Q_RDP, pc);
      if (whi >= off && wlo < off + n) {
        int a = wlo > off ? wlo : off, b = whi < off + n - 1 ? whi : off + n - 1;
        uint32_t wfo = ((uint32_t) a * (uint32_t) bpp4) / 2, wfn = ((uint32_t) (b - a + 1) * (uint32_t) bpp4 + 1) / 2;
        push_mem(Q_RDP, 1, RA_RDP_FB, fb + wfo, wfn);
        if (z_update) push_mem(Q_RDP, 1, RA_RDP_Z, z + (uint32_t) a * 2, (uint32_t) (b - a + 1) * 2);
      }
      if (p->overlap > 0) push(Q_RDP, IT_SPANEND, 0, 0, 0);
    }
  }
  (void) fbbytes; (void) zbytes;
}

void rdram_rdp_texload(uint32_t addr, uint32_t bytes) {
  span_flush();
  if (span_trace) fprintf(span_trace, "T %x %u\n", addr, bytes);
  if (bytes == 0) return;
  if (g_rdram.p.eng > 0) {
    struct rwork *w = eng_new(RK_LOAD);
    w->raddr[0] = addr & 0x7FFFFF; w->rbytes[0] = bytes; w->nr = 1;
    w->clk = (uint32_t) ((bytes + 7) / 8 * g_rdram.p.tmem8 * 4 + 0.5);
    return;
  }
  push_mem(Q_RDP, 0, RA_RDP_TEX, addr, bytes);
  push_cycles(Q_RDP, (bytes + 7) / 8 * g_rdram.p.tmem8);
}

// a triangle's scanlines above the scissor box: the edge walker steps them before the first span
static uint64_t walk_lines = 0, walk_tris = 0; // since the last window reset

void rdram_rdp_tri_walk(int lines) {
  span_flush();
  if (lines <= 0) return;
  walk_lines += (uint64_t) lines;
  walk_tris++;
  if (span_trace) fprintf(span_trace, "W %d\n", lines);
  if (g_rdram.p.eng > 0) eng_front(lines * g_rdram.p.ew_line);
  else push_cycles(Q_RDP, lines * g_rdram.p.ew_line);
}

void rdram_rdp_mark(unsigned ring_index) {
  span_flush();
  if (g_rdram.p.eng > 0) {
    struct rwork *w = eng_new(RK_MARK);
    w->clk = ring_index;
    return;
  }
  push(Q_RDP, IT_MARK, 0, 0, ring_index);
}

int rdram_rdp_pending(void) {
  span_flush();
  if (g_rdram.p.eng > 0) return eng_busy();
  struct queue *q = &queues[Q_RDP];
  return q->head != q->tail || q->ready > g_rdram.now;
}

void rdram_rdp_flush(void) {
  span_flush();
  if (g_rdram.p.eng > 0) {
    while (E.head != E.tail) eng_run(TNONE);
    return;
  }
  while (queues[Q_RDP].head != queues[Q_RDP].tail) run_queue(&queues[Q_RDP], 1);
}

// ---- clock, VI, refresh, PI ---------------------------------------------------------------------
void rdram_tick(void) {
  struct rdram_model *m = &g_rdram;
  int i;
  m->now += 4;
  for (i = 0; i < Q_N; i++)
    if (queues[i].head != queues[i].tail) run_queue(&queues[i], 0);
  if (m->p.eng > 0) {
    if (E.head != E.tail) eng_run(m->now);
    if (eng_busy()) { m->rdp_busy += 4; m->rdp_busy_total += 4; }
  } else if (queues[Q_RDP].head != queues[Q_RDP].tail || queues[Q_RDP].ready > m->now) { m->rdp_busy += 4; m->rdp_busy_total += 4; }
  if (rsp_inflight) m->rsp_dma_busy += 4;
}

void rdram_vi_line(uint32_t addr, uint32_t bytes) {
  // the VI's own transfer size (vi_burst), split at rows like the other agents' DMAs
  uint32_t rowsz = 1u << (unsigned) g_rdram.p.row_bits;
  uint32_t burst = (uint32_t) (g_rdram.p.vi_burst > 0 ? g_rdram.p.vi_burst : g_rdram.p.burst);
  if (burst < 8) burst = 8;
  while (bytes > 0) {
    uint32_t n = bytes < burst ? bytes : burst, to_row = rowsz - (addr & (rowsz - 1));
    if (n > to_row) n = to_row;
    push(Q_VI, IT_RD, RA_VI, addr & 0x7FFFFF, n);
    addr += n; bytes -= n;
  }
}

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
  {
    const char *tf = getenv("CEN64_SPAN_TRACE");
    if (tf != NULL && tf[0] && span_trace == NULL) span_trace = fopen(tf, "w");
  }
  struct rdram_params d = {
    // Fitted 2026-10-01 to the console's HWCAL boot tests (rc6 cal ROM + the RDPLAB ROM): RDP tests through the
    // span-trace replay (5.8% rms over 70 tests), CPU and CPU-while-RDP tests through emulator runs (11% rms).
    // The earlier M64-calibrated serial model: eng=0,req=3,rdelay=20,wdelay=1,per_byte=0.5,row_miss=40,cpu_d=45,
    // cpu_i=46,cpu_unc=24.6,cpu_wb=39.5,burst=128,px1=1.08,px2=2.16,fill=0.25,tri=32,rect=16,cmd=2,tmem8=1,
    // sync=200,span=8,cmd_fetch=64,chunk=0,bank_busy=0,cpu_prio=-1 (its uncached-store and victim paths differ).
    4.75, 21.2, 0.784, 0.621, 6.85, 54, 1, // req rdelay wdelay per_byte row_miss refresh refresh_close
    21.69, 28.03, 21.46, 3.20, 13.91,   // cpu_d cpu_i cpu_unc cpu_wb cpu_uncw
    64,                                 // burst
    1.006, 2.052, 0.181, 0.25,          // px1 px2 fill copy
    5.81, 28.99, 1.535, 0.879, 59.37, 0.553, // tri rect cmd tmem8 sync span
    0, 20, 11, 1591, 128, 64,           // overlap bank_bits row_bits pi_gap cmd_fetch idle
    1, 1,                               // cpu_bus rsp_bus (experiments: share of the channel time they occupy)
    0, 0,                               // cpu_rows rsp_rows (experiments: separate open-row state)
    0, 0, 0, 0, 0, 0,                   // iso ranges (experiments)
    8,                                  // chunk
    1, 10.58, 2, 16, 2, 1, 4.02, 4.23, 3.72, 8, 0, // eng rd_lat rdp_d rdp_dw rdp_sb rdp_rprio span_r tta psync fchunk fetch_block
    14.15, 12.76, 2, 0.722,             // span_rs span_ws align8 rd_occ
    17.5,                               // bank_busy
    16.96,                              // cpu_wbv
    1, 4.83,                            // bank_rdp cpu_rocc
    21.73,                              // cpu_prio
    0,                                  // span_wfull
    8,                                  // ew_line
    0,                                  // row_xmiss
    0,                                  // vi_rows
    0                                   // vi_burst (0: burst)
  };
  const char *s = g_rdram_params ? g_rdram_params : getenv("CEN64_RDRAM_MODEL");
  int i;
  memset(&g_rdram, 0, sizeof(g_rdram));
  g_rdram.p = d;
  g_rdram.last_dir = -1;
  g_rdram.last_agent = -1;
  for (i = 0; i < 32; i++) { g_rdram.open_row[i] = -1; g_rdram.row_owner[i] = -1; }
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
  eng_init();
  rsp_inflight = 0; uncw_done = 0; cpu_after = 0; fetch_lo = fetch_hi = 0; rdp_frac = 0;
  memset(&pend, 0, sizeof(pend));
  g_rdram.on = 1;
  if (!getenv("CEN64_RDRAM_QUIET")) {
    fprintf(stderr, "RDRAM model:");
    for (i = 0; i < (int) NPARAM; i++)
      fprintf(stderr, " %s=%g", param_tab[i].name, *(double *) ((char *) &g_rdram.p + param_tab[i].off));
    fprintf(stderr, "\n");
  }
}

void rdram_window_reset(void) {
  struct rdram_model *m = &g_rdram;
  memset(m->st, 0, sizeof(m->st));
  memset(m->conflict, 0, sizeof(m->conflict));
  m->cpu_stall = m->cpu_idle = m->rdp_busy = m->rsp_dma_busy = 0;
  m->window_start = m->now;
  walk_lines = walk_tris = 0;
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
  /* RDRAMW,<name>,<triangles with scanlines above the scissor>,<those scanlines> (edge walker) */
  printf("RDRAMW,%s,%llu,%llu\n", name, (unsigned long long) walk_tris, (unsigned long long) walk_lines);
  /* RDRAMX,<name>,<victim>,<opener>,<bank>,<row misses>: which agent's access closed whose row */
  {
    int a, b, k;
    for (a = 0; a < RA_N; a++)
      for (b = 0; b < RA_N; b++)
        for (k = 0; k < 8; k++)
          if (m->conflict[a][b][k] > 0)
            printf("RDRAMX,%s,%s,%s,%d,%llu\n", name, agent_name[a], agent_name[b], k,
                   (unsigned long long) m->conflict[a][b][k]);
  }
}
