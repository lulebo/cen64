// rdram_replay.c: replay CEN64_SPAN_TRACE files through the -rdram model's RDP engine, without the emulator.
// usage: rdram_replay trace1 [trace2 ...] < params   (one -rdrammodel string per stdin line)
// For every parameter line: per trace, per segment (a list ending in a full sync), the RDP cycles from the
// segment's first item to its last one done; output: "R <trace#> <segment#> <cycles>" lines, then "END".
// The segments run back to back with the RDP idle in between (as the console's boot tests do).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "bus/rdram_model.h"

struct ev { uint8_t k; uint8_t ct, ir, zc, zu, bpp4; uint16_t len; int16_t n, lo, hi; uint32_t a, b, c; };
struct trace { struct ev *e; size_t n, cap; };

static void add(struct trace *t, struct ev *e) {
  if (t->n == t->cap) { t->cap = t->cap ? t->cap * 2 : 65536; t->e = realloc(t->e, t->cap * sizeof(*e)); }
  t->e[t->n++] = *e;
}

static void load(struct trace *t, const char *path) {
  FILE *f = fopen(path, "r");
  char line[256];
  if (!f) { perror(path); exit(1); }
  while (fgets(line, sizeof(line), f)) {
    struct ev e;
    memset(&e, 0, sizeof(e));
    if (line[0] == 'C') {
      unsigned cmd, addr, len; int xbus;
      if (sscanf(line + 2, "%x %x %u %d", &cmd, &addr, &len, &xbus) != 4) continue;
      e.k = 'C'; e.a = cmd; e.b = addr; e.c = len; e.ir = (uint8_t) xbus;
    } else if (line[0] == 'S') {
      int y, x, len, bpp4, ct, ir, zc, zu, n, lo, hi; unsigned fb, z;
      if (sscanf(line + 2, "%d %d %d %d %d %d %d %d %d %d %d %x %x", &y, &x, &len, &bpp4, &ct, &ir, &zc, &zu, &n,
                 &lo, &hi, &fb, &z) != 13) continue;
      e.k = 'S'; e.len = (uint16_t) len; e.bpp4 = (uint8_t) bpp4; e.ct = (uint8_t) ct; e.ir = (uint8_t) ir;
      e.zc = (uint8_t) zc; e.zu = (uint8_t) zu; e.n = (int16_t) n; e.lo = (int16_t) lo; e.hi = (int16_t) hi;
      e.a = fb; e.b = z;
    } else if (line[0] == 'T') {
      unsigned addr, bytes;
      if (sscanf(line + 2, "%x %u", &addr, &bytes) != 2) continue;
      e.k = 'T'; e.a = addr; e.b = bytes;
    } else
      continue;
    add(t, &e);
  }
  fclose(f);
}

// The VI as the console's boot leaves it (CALV: 16-bit, 320 wide, lines 18..254 of 262, origin ~0x280):
// a refresh every line (3968.5 RCP cycles) and a 640-byte fetch on every active line. -novi: none.
// -vimap FILE: lines "<trace#> <segment#> <origin hex | off>" set the VI for single segments (the refresh
// always runs; "off" blanks the VI, an origin scans 320x240 16-bit there).
static int vi_on = 1;
static uint64_t vi_next = 0;
static unsigned vi_line = 0;
static int32_t vi_origin = 0x280; // -1: blank
#define VIMAP_MAX 4096
static struct { int t, s; int32_t origin; } vimap[VIMAP_MAX];
static int nvimap = 0;
static void vi_set(int t, int seg) {
  int i;
  vi_origin = 0x280;
  for (i = 0; i < nvimap; i++)
    if (vimap[i].t == t && vimap[i].s == seg) vi_origin = vimap[i].origin;
}
static void vi_step(void) {
  while (vi_on && g_rdram.now >= vi_next) {
    rdram_refresh();
    if (vi_origin >= 0 && vi_line >= 18 && vi_line < 255) rdram_vi_line((uint32_t) vi_origin + (vi_line - 18) * 640, 640);
    vi_line = (vi_line + 1) % 262;
    vi_next += 15874; // RDRAM clocks per line (3968.5 RCP cycles)
  }
}

static void run_until_idle(void) {
  if (!vi_on) {
    rdram_rdp_flush();
    if (rdram_rdp_done_time() > g_rdram.now) g_rdram.now = rdram_rdp_done_time();
    return;
  }
  while (rdram_rdp_pending()) { rdram_tick(); vi_step(); }
}

static void idle_wait(void) { run_until_idle(); }

int main(int argc, char **argv) {
  struct trace tr[8];
  int nt = argc - 1, i;
  char params[4096];
  memset(tr, 0, sizeof(tr));
  if (nt < 1 || nt > 8) { fprintf(stderr, "usage: rdram_replay trace... < params\n"); return 1; }
  while (nt > 0 && argv[1][0] == '-') {
    if (!strcmp(argv[1], "-novi")) { vi_on = 0; argv++; nt--; }
    else if (!strcmp(argv[1], "-vimap") && nt > 1) {
      FILE *f = fopen(argv[2], "r");
      char l[128];
      if (!f) { perror(argv[2]); return 1; }
      while (fgets(l, sizeof(l), f) && nvimap < VIMAP_MAX) {
        char o[32];
        if (sscanf(l, "%d %d %31s", &vimap[nvimap].t, &vimap[nvimap].s, o) == 3) {
          vimap[nvimap].origin = !strcmp(o, "off") ? -1 : (int32_t) strtol(o, NULL, 16);
          nvimap++;
        }
      }
      fclose(f);
      argv += 2; nt -= 2;
    } else break;
  }
  for (i = 0; i < nt; i++) load(&tr[i], argv[i + 1]);
  setenv("CEN64_RDRAM_QUIET", "1", 1);
  while (fgets(params, sizeof(params), stdin)) {
    params[strcspn(params, "\r\n")] = 0;
    g_rdram_params = params;
    rdram_model_init();
    vi_next = 0; vi_line = 0;
    for (i = 0; i < nt; i++) {
      size_t j = 0;
      int seg = 0;
      while (j < tr[i].n) {
        uint64_t t0;
        idle_wait();
        vi_set(i, seg);
        { uint64_t gap = g_rdram.now + 400; while (g_rdram.now < gap) { rdram_tick(); vi_step(); } } // the CPU starts the next list a while later
        t0 = g_rdram.now;
        for (; j < tr[i].n; j++) {
          struct ev *e = &tr[i].e[j];
          if (e->k == 'C') {
            rdram_rdp_cmd_begin(e->a, e->b, e->c, e->ir);
            if (e->a == 0x29) { j++; break; }
          } else if (e->k == 'S') {
            rdram_rdp_span_written(e->a, e->b, e->len, e->bpp4, e->ct, e->ir, e->zc, e->zu, e->n, e->lo, e->hi);
          } else
            rdram_rdp_texload(e->a, e->b);
        }
        run_until_idle();
        printf("R %d %d %llu\n", i, seg, (unsigned long long) ((rdram_rdp_done_time() - t0 + 3) / 4));
        seg++;
      }
    }
    printf("END\n");
    fflush(stdout);
  }
  return 0;
}
