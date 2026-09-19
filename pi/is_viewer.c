#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "is_viewer.h"
#include <stdio.h>

extern uint64_t *g_vr4300_profile_samples;

/* CEN64_PROFILE_DIR: per-benchmark-scenario CPU profiles (see vr4300/cpu.c). */
static void profile_window(const char *line) {
  static int inited = 0; static const char *dir = NULL;
  const size_t n = 8 * 1024 * 1024;
  if (!inited) { inited = 1; dir = getenv("CEN64_PROFILE_DIR"); }
  if (dir == NULL || g_vr4300_profile_samples == NULL) return;
  if (!strncmp(line, "BENCH_START,", 12)) {
    memset(g_vr4300_profile_samples, 0, 4 * n * sizeof(uint64_t));
  } else if (!strncmp(line, "BENCH_END,", 10)) {
    char name[64], path[512]; size_t i, k = 0; FILE *f;
    const char *p = line + 10;
    while (*p && *p != ',' && *p != '\n' && k < sizeof(name) - 1) name[k++] = *p++;
    name[k] = 0;
    snprintf(path, sizeof(path), "%s/%s.profile", dir, name);
    f = fopen(path, "w");
    if (f == NULL) return;
    for (i = 0; i < n; i++) {
      uint64_t ins = g_vr4300_profile_samples[i], l1d = g_vr4300_profile_samples[i + n],
               cyc = g_vr4300_profile_samples[i + 2 * n], ic = g_vr4300_profile_samples[i + 3 * n];
      if (cyc < 20 && ins < 20 && l1d < 20 && ic < 20) continue;
      fprintf(f, "%x %llu %llu %llu %llu\n", (unsigned) (i + 0x80000000),
              (unsigned long long) ins, (unsigned long long) l1d, (unsigned long long) cyc,
              (unsigned long long) ic);
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

  is->buffer = calloc(IS_VIEWER_ADDRESS_LEN, 1);
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

  return 0;
}

int write_is_viewer(struct is_viewer *is, uint32_t address, uint32_t word, uint32_t dqm) {
  uint32_t offset = address - is->base_address;
  assert(offset + 4 <= is->len);

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
        if (is->show_output)
          printf("%s", is->output_buffer_conv);
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
