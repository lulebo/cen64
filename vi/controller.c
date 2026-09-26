//
// vi/controller.c: Video interface controller.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "context.h"
#include "bus/address.h"
#include "bus/controller.h"
#include "device/device.h"
#include "os/main.h"
#include "timer.h"
#include "ri/controller.h"
#include "vi/controller.h"
#include "bus/rdram_model.h"
#include "vi/render.h"
#include "vi/window.h"
#include "vr4300/interface.h"

#define VI_COUNTER_START ((62500000.0 / 60.0) + 1)
#define VI_BLANKING_DONE (unsigned) ((VI_COUNTER_START - VI_COUNTER_START / 525.0 * 39))

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *vi_register_mnemonics[NUM_VI_REGISTERS] = {
#define X(reg) #reg,
#include "vi/registers.md"
#undef X
};
#endif

// Reads a word from the VI MMIO register space.
int read_vi_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct vi_controller *vi = (struct vi_controller *) opaque;
  unsigned offset = address - VI_REGS_BASE_ADDRESS;
  enum vi_register reg = (offset >> 2);

  vi->regs[VI_CURRENT_REG] = 0;

  // Prevent division by zero (field number doesn't count).
  if (vi->regs[VI_V_SYNC_REG] >= 0x2) {
    vi->regs[VI_CURRENT_REG] =
      (VI_COUNTER_START - (vi->counter)) /
      (VI_COUNTER_START / (vi->regs[VI_V_SYNC_REG] >> 1));

    vi->regs[VI_CURRENT_REG] = (vi->regs[VI_CURRENT_REG] << 1);

    // Interlaced fields should get the current field number.
    // Non-interlaced modes should always get a constant field.
    if (!(vi->regs[VI_V_SYNC_REG] & 0x1))
      vi->regs[VI_CURRENT_REG] |= vi->field;
  }

  *word = vi->regs[reg];
  debug_mmio_read(vi, vi_register_mnemonics[reg], *word);
  return 0;
}

// Advances the controller by one clock cycle.
void vi_cycle(struct vi_controller *vi) {
  cen64_gl_window window;
  size_t copy_size;

  unsigned counter;
  struct render_area *ra = &vi->render_area;
  struct bus_controller *bus;
  float hcoeff, vcoeff;

  counter = --(vi->counter);

  // Wrap the counter around when it hits zero.
  if (unlikely(counter == 0))
    vi->counter = VI_COUNTER_START;

  // Throw an interrupt when VI_INTR_REG == VI_CURRENT_REG.
  // We use a counter so we don't have to recalc each cycle.
  if (unlikely(counter == vi->intr_counter))
    signal_rcp_interrupt(vi->bus->vr4300, MI_INTR_VI);

  // -rdram: one refresh per line, and the line fetch of every active line.
  if (g_rdram.on) {
    static unsigned last_line = ~0u;
    unsigned line = (unsigned) (((uint64_t) ((unsigned) VI_COUNTER_START - counter)) * 2 / 7937);
    if (line != last_line) {
      unsigned vs = vi->regs[VI_V_START_REG], type = vi->regs[VI_STATUS_REG] & 3;
      unsigned first = (vs >> 16 & 0x3FF) >> 1, last = (vs & 0x3FF) >> 1;
      last_line = line;
      rdram_refresh();
      if (type >= 2 && line >= first && line < last) {
        unsigned width = vi->regs[VI_WIDTH_REG] & 0xFFF, bpp = type == 3 ? 4 : 2;
        unsigned yscale = vi->regs[VI_Y_SCALE_REG] & 0xFFF;
        unsigned fbline = (line - first) * yscale / 1024;
        rdram_vi_line((vi->regs[VI_ORIGIN_REG] & 0xFFFFFF) + fbline * width * bpp, width * bpp);
      }
    }
  }

  // NTSC reserves the first 39 lines for vertical blanking
  // according to the literature I've read. Normally after this
  // time, the VI would slowly push out the analog signal. For
  // now, let's just toss the GPU the framebuffer the moment
  // we step out of the vertical blanking interval.
  if (likely(counter != VI_BLANKING_DONE))
    return;

  vi->field = !vi->field;
  window = vi->window;

  // Calculate the bounding positions.
  ra->x.start = vi->regs[VI_H_START_REG] >> 16 & 0x3FF;
  ra->x.end = vi->regs[VI_H_START_REG] & 0x3FF;
  ra->y.start = vi->regs[VI_V_START_REG] >> 16 & 0x3FF;
  ra->y.end = vi->regs[VI_V_START_REG] & 0x3FF;

  hcoeff = (float) (vi->regs[VI_X_SCALE_REG] & 0xFFF) / (1 << 10);
  vcoeff = (float) (vi->regs[VI_Y_SCALE_REG] & 0xFFF) / (1 << 10);

  // Interact with the user interface?
  if (likely(window)) {
    cen64_mutex_lock(&window->event_mutex);

    if (unlikely(window->exit_requested)) {
      cen64_mutex_unlock(&window->event_mutex);
      device_exit(vi->bus);
    }

    cen64_mutex_unlock(&window->event_mutex);
    cen64_mutex_lock(&window->render_mutex);

    // Calculate the height and width of the frame.
    window->frame_vres = ra->height =((ra->y.end - ra->y.start) >> 1) * vcoeff;
    window->frame_hres = ra->width = ((ra->x.end - ra->x.start)) * hcoeff;
    window->frame_hskip = ra->hskip = vi->regs[VI_WIDTH_REG] - ra->width;
    window->frame_type = vi->regs[VI_STATUS_REG] & 0x3;

    if (window->frame_hres <= 0 || window->frame_vres <= 0)
      window->frame_type = 0;

    // Copy the frame data into a temporary buffer.
    copy_size = sizeof(bus->ri->ram) - (vi->regs[VI_ORIGIN_REG] & MAX_RDRAM_SIZE_MASK);

    if (copy_size > sizeof(vi->window->frame_buffer))
      copy_size = sizeof(vi->window->frame_buffer);

    memcpy(&bus, vi, sizeof(bus));
    memcpy(vi->window->frame_buffer,
      bus->ri->ram + (vi->regs[VI_ORIGIN_REG] & MAX_RDRAM_SIZE_MASK),
      copy_size);

    cen64_mutex_unlock(&vi->window->render_mutex);
    cen64_gl_window_push_frame(window);
  }

  else if (++(vi->frame_count) == 60) {
    cen64_time current_time;
    float ns;

    get_time(&current_time);
    ns = compute_time_difference(&current_time, &vi->last_update_time);
    vi->last_update_time = current_time;
    vi->frame_count = 0;

    printf("VI/s: %.2f\n", (60 / (ns / NS_PER_SEC)));
  }
}

// Initializes the VI.
int vi_init(struct vi_controller *vi,
  struct bus_controller *bus, bool no_interface) {
  vi->counter = VI_COUNTER_START;
  vi->bus = bus;

  if (!no_interface) {
    if (vi_create_window(vi))
      return -1;

    gl_window_init(vi);
  }

  return 0;
}

// Writes a word to the VI MMIO register space.
int write_vi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct vi_controller *vi = (struct vi_controller *) opaque;
  unsigned offset = address - VI_REGS_BASE_ADDRESS;
  enum vi_register reg = (offset >> 2);

  debug_mmio_write(vi, vi_register_mnemonics[reg], word, dqm);

  if (reg == VI_CURRENT_REG)
    clear_rcp_interrupt(vi->bus->vr4300, MI_INTR_VI);

  else if (reg == VI_INTR_REG) {

    // TODO: This seems... all kinds of wrong for interlaced modes.
    // Do we fire two interrupts in interlaced modes? Have to test.
    // I'm not an NTSC signal expert, so this'll have to do for now.
    vi->intr_counter = VI_COUNTER_START - VI_COUNTER_START / 525 * (word >> 1);

    vi->regs[reg] &= ~dqm;
    vi->regs[reg] |= word;
  }

  else {
    vi->regs[reg] &= ~dqm;
    vi->regs[reg] |= word;
  }

  return 0;
}

