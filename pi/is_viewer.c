#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "rdp/cpu.h"
#include "is_viewer.h"
#include <stdio.h>

extern uint64_t *g_vr4300_profile_samples;
#include "rsp/cpu.h"
#include "common/bus_traffic.h"
#include "bus/rdram_model.h"

/* CEN64_PROFILE_DIR: per-benchmark-scenario CPU profiles (see vr4300/cpu.c). */
static void profile_window(const char *line) {
  static int inited = 0; static const char *dir = NULL;
  const size_t n = 8 * 1024 * 1024;
  if (!inited) { inited = 1; dir = getenv("CEN64_PROFILE_DIR"); }
  /* -rsphw: per-scenario issue statistics on stdout, next to the ROM's own lines. */
  if (g_rsp_hw_stats != NULL && g_rsp_hw_stats->enabled) {
    struct rsp_hwtiming *hw = g_rsp_hw_stats;
    if (!strncmp(line, "BENCH_START,", 12)) {
      hw->n_insn = hw->n_pair = hw->n_stall = hw->n_branch = 0;
    } else if (!strncmp(line, "BENCH_END,", 10)) {
      printf("RSPHW,%llu,%llu,%llu,%llu\n", (unsigned long long) hw->n_insn,
             (unsigned long long) hw->n_pair, (unsigned long long) hw->n_stall,
             (unsigned long long) hw->n_branch);
    }
  }
  if (dir != NULL && g_rsp_prof != NULL && g_rsp_prof->enabled) {
    if (!strncmp(line, "BENCH_START,", 12)) {
      memset(g_rsp_prof->cycles, 0, sizeof(g_rsp_prof->cycles));
    } else if (!strncmp(line, "BENCH_END,", 10)) {
      char name[64], path[512]; size_t k = 0; int b, i; FILE *f;
      const char *p = line + 10;
      while (*p && *p != ',' && *p != '\n' && k < sizeof(name) - 1) name[k++] = *p++;
      name[k] = 0;
      snprintf(path, sizeof(path), "%s/%s.rspprof", dir, name);
      f = fopen(path, "w");
      if (f != NULL) {
        for (b = 0; b < RSP_PROF_UCODES; b++) {
          if (g_rsp_prof->ucode[b] == 0) continue;
          for (i = 0; i < 0x400; i++) {
            if (g_rsp_prof->cycles[b][i] != 0)
              fprintf(f, "%x %x %llu\n", (unsigned) g_rsp_prof->ucode[b], (unsigned) (i * 4),
                      (unsigned long long) g_rsp_prof->cycles[b][i]);
          }
        }
        fclose(f);
      }
    }
  }
  if (!strncmp(line, "BENCH_START,", 12)) {
    memset(&g_bus, 0, sizeof(g_bus));
    if (g_rdram.on) rdram_window_reset();
    itrace_start();
    dtrace_start();
  } else if (!strncmp(line, "BENCH_END,", 10)) {
    itrace_stop();
    dtrace_stop();
    char name[64]; size_t k = 0;
    const char *p = line + 10;
    while (*p && *p != ',' && *p != '\n' && k < sizeof(name) - 1) name[k++] = *p++;
    name[k] = 0;
    printf("BUS,%s,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,"
           "%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu\n", name,
           (unsigned long long) g_bus.cpu_cycles, (unsigned long long) g_bus.rdp_frames,
           (unsigned long long) g_bus.cpu_ifill, (unsigned long long) g_bus.cpu_ifetch_unc,
           (unsigned long long) g_bus.cpu_dfill, (unsigned long long) g_bus.cpu_dwb,
           (unsigned long long) g_bus.cpu_dwb_op, (unsigned long long) g_bus.cpu_unc_r,
           (unsigned long long) g_bus.cpu_unc_rb, (unsigned long long) g_bus.cpu_unc_w,
           (unsigned long long) g_bus.cpu_unc_wb, (unsigned long long) g_bus.cpu_mmio,
           (unsigned long long) g_bus.rsp_dma_r, (unsigned long long) g_bus.rsp_dma_rb,
           (unsigned long long) g_bus.rsp_dma_w, (unsigned long long) g_bus.rsp_dma_wb,
           (unsigned long long) g_bus.rsp_aud_rb, (unsigned long long) g_bus.rsp_aud_wb,
           (unsigned long long) g_bus.rdp_cmd_b, (unsigned long long) g_bus.rdp_tex_b,
           (unsigned long long) g_bus.rdp_fbr, (unsigned long long) g_bus.rdp_fbw,
           (unsigned long long) g_bus.rdp_fill, (unsigned long long) g_bus.rdp_zr,
           (unsigned long long) g_bus.rdp_zw, (unsigned long long) g_bus.vi_b,
           (unsigned long long) g_bus.ai_b, (unsigned long long) g_bus.pi_w_b,
           (unsigned long long) g_bus.pi_r_b, (unsigned long long) 0);
    if (g_rdram.on) rdram_window_report(name);
  }
  if (dir == NULL || g_vr4300_profile_samples == NULL) return;
  if (!strncmp(line, "BENCH_START,", 12)) {
    memset(g_vr4300_profile_samples, 0, PROF_REGIONS * n * sizeof(uint64_t));
    if (g_dline_prof) memset(g_dline_prof, 0, 2 * DLINES * sizeof(uint64_t));
    drange_reset();
    dacc_reset();
  } else if (!strncmp(line, "BENCH_END,", 10)) {
    char name[64], path[512]; size_t i, k = 0; FILE *f;
    const char *p = line + 10;
    while (*p && *p != ',' && *p != '\n' && k < sizeof(name) - 1) name[k++] = *p++;
    name[k] = 0;
    snprintf(path, sizeof(path), "%s/%s.drange", dir, name);
    drange_dump(path);
    snprintf(path, sizeof(path), "%s/%s.daccess", dir, name);
    dacc_dump(path);
    if (g_dline_prof) {
      snprintf(path, sizeof(path), "%s/%s.dprofile", dir, name);
      f = fopen(path, "w");
      if (f != NULL) {
        for (i = 0; i < DLINES; i++) {
          if (g_dline_prof[i] || g_dline_prof[DLINES + i])
            fprintf(f, "%x %llu %llu\n", (unsigned) (i << 4), (unsigned long long) g_dline_prof[i],
                    (unsigned long long) g_dline_prof[DLINES + i]);
        }
        fclose(f);
      }
    }
    snprintf(path, sizeof(path), "%s/%s.profile", dir, name);
    f = fopen(path, "w");
    if (f == NULL) return;
    for (i = 0; i < n; i++) {
      uint64_t ins = g_vr4300_profile_samples[i], l1d = g_vr4300_profile_samples[i + n],
               cyc = g_vr4300_profile_samples[i + 2 * n], ic = g_vr4300_profile_samples[i + 3 * n],
               wb = g_vr4300_profile_samples[i + 4 * n], unc = g_vr4300_profile_samples[i + 5 * n];
      if (cyc < 20 && ins < 20 && l1d < 20 && ic < 20 && wb == 0 && unc == 0) continue;
      fprintf(f, "%x %llu %llu %llu %llu %llu %llu\n", (unsigned) (i + 0x80000000),
              (unsigned long long) ins, (unsigned long long) l1d, (unsigned long long) cyc,
              (unsigned long long) ic, (unsigned long long) wb, (unsigned long long) unc);
    }
    fclose(f);
  }
}

// Minus text buffer base offset, plus NULL terminator
#define IS_BUFFER_SIZE IS_VIEWER_ADDRESS_LEN - 0x20 + 1

int is_viewer_init(struct is_viewer *is, int is_viewer_output) {
  memset(is, 0, sizeof(*is));

  // TODO support other addresses
  is->base_address = IS_VIEWER_BASE_ADDRESS;
  is->len = IS_VIEWER_ADDRESS_LEN;
  if (getenv("CEN64_ISV_RING") != NULL && atoi(getenv("CEN64_ISV_RING")) != 0) {
    is->ring = atoi(getenv("CEN64_ISV_RING"));
    is->len = IS_VIEWER_RING_LEN;
    is->line = calloc(IS_VIEWER_RING_LEN, 1);
  }

  is->buffer = calloc(is->len, 1);
  is->output_buffer = calloc(IS_BUFFER_SIZE, 1);
  is->output_buffer_conv = calloc(IS_BUFFER_SIZE * 3, 1);
  is->show_output = is_viewer_output;

  is->cd = iconv_open("UTF-8", "EUC-JP");

  if (is->buffer == NULL || is->output_buffer == NULL ||
      is->output_buffer_conv == NULL)
    return 0;
  else
    return 1;
}

int is_viewer_map(struct is_viewer *is, uint32_t address) {
  return address >= is->base_address && address + 4 <= is->base_address + is->len;
}

int read_is_viewer(struct is_viewer *is, uint32_t address, uint32_t *word) {
  uint32_t offset = address - is->base_address;
  assert(offset + 4 <= is->len);

  memcpy(word, is->buffer + offset, sizeof(*word));
  *word = byteswap_32(*word);
  if (is->ring > 1 && offset < 0x20)
    fprintf(stderr, "ISVR rd +%x -> %x\n", offset, *word);

  return 0;
}

// Ring mode: one complete line of output (no EUC conversion: the ROM prints ASCII).
static void is_viewer_ring_line(struct is_viewer *is) {
  is->line[is->line_pos] = 0;
  profile_window((const char *) is->line);
  if (is->show_output) {
    if (g_isv_ts)
      printf("@%llu,", (unsigned long long) *g_rcp_now);
    printf("%s", (const char *) is->line);
  }
  is->line_pos = 0;
}

static uint32_t is_viewer_word(struct is_viewer *is, uint32_t offset) {
  uint32_t w;
  memcpy(&w, is->buffer + offset, 4);
  return byteswap_32(w);
}

int write_is_viewer(struct is_viewer *is, uint32_t address, uint32_t word, uint32_t dqm) {
  uint32_t offset = address - is->base_address;
  assert(offset + 4 <= is->len);

  if (is->ring) {
    // The host side of the SummerCart64's emulation: when the write pointer moves and the token
    // is present, everything from the read pointer to it is printed and the read pointer follows.
    uint32_t w = byteswap_32(word);
    memcpy(is->buffer + offset, &w, sizeof(w));
    if (is->ring > 1 && offset < 0x20)
      fprintf(stderr, "ISVR wr +%x = %x (token %x rp %x)\n", offset, word, is_viewer_word(is, 0), is_viewer_word(is, 4));
    if (offset == 0x14 && is_viewer_word(is, 0) == 0x49533634) {
      const uint32_t size = IS_VIEWER_RING_LEN - 0x20;
      uint32_t rp = is_viewer_word(is, 4), wp = word;
      if (rp < size && wp < size) {
        while (rp != wp) {
          uint8_t c = is->buffer[0x20 + rp];
          if (is->ring > 2) fputc(c, stderr);
          rp = rp + 1 == size ? 0 : rp + 1;
          if (is->line_pos < IS_VIEWER_RING_LEN - 2)
            is->line[is->line_pos++] = c;
          if (c == '\n')
            is_viewer_ring_line(is);
        }
        w = byteswap_32(rp);
        memcpy(is->buffer + 4, &w, sizeof(w));
      }
    }
    return 0;
  }

  if (offset == 0x14) {
    if (word > 0) {
      assert(is->output_buffer_pos + word + 0x20 < is->len);
      memcpy(is->output_buffer + is->output_buffer_pos, is->buffer + 0x20, word);
      is->output_buffer_pos += word;
      is->output_buffer[is->output_buffer_pos] = '\0';

      // once a full line is present, convert the output from EUC to UTF-8
      if (memchr(is->output_buffer, '\n', is->output_buffer_pos)) {
        char *inptr = (char *)is->output_buffer;
        size_t len = strlen(inptr);
        size_t outlen = 3 * len;
        char *outptr = (char *)is->output_buffer_conv;
        memset(is->output_buffer_conv, 0, IS_BUFFER_SIZE * 3);
        iconv(is->cd, &inptr, &len, &outptr, &outlen);

        profile_window((const char *) is->output_buffer_conv);
        if (is->show_output) {
          if (g_isv_ts)
            printf("@%llu,", (unsigned long long) *g_rcp_now);
          printf("%s", is->output_buffer_conv);
        }
        else if (!is->output_warning) {
          printf("ISViewer debugging output detected and suppressed.\nRun cen64 with option -is-viewer to display it\n");
          is->output_warning = 1;
        }

        memset(is->output_buffer, 0, is->output_buffer_pos);
        is->output_buffer_pos = 0;
      }
    }
    memset(is->buffer + 0x20, 0, word);
  } else {
    word = byteswap_32(word);
    memcpy(is->buffer + offset, &word, sizeof(word));
  }

  return 0;
}
