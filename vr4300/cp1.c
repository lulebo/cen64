//
// vr4300/cp1.c: VR4300 floating point unit coprocessor.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "fpu/fpu.h"
#include "vr4300/cp1.h"
#include "vr4300/cpu.h"
#include "vr4300/decoder.h"
#include "vr4300/fault.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//
// FPU exceptions as on hardware (VR4300 user's manual ch. 7; the model ares implements). An operand
// that is denormal or NaN raises Unimplemented Operation (E, which cannot be masked); a NaN result
// from ordinary operands raises Invalid (V) when enabled; a denormal result is flushed to zero when
// FS is set, else E; conversions to integers of NaN, infinity or out-of-range values raise E.
// libultra enables V for every thread and stops the thread that faults: the game hangs.
// CEN64_FPE=0 disables all of it (the old behaviour).
//
#define FCR31_CAUSE_E (1u << 17)
#define FCR31_CAUSE_V (1u << 16)
#define FCR31_CAUSE_Z (1u << 15)
#define FCR31_ENABLE_V (1u << 11)
#define FCR31_ENABLE_Z (1u << 10)
#define FCR31_ENABLE_UI (3u << 7)
#define FCR31_FS (1u << 24)

static int vr4300_fpe_mode = -1;

static inline int vr4300_fpe_on(void) {
  if (unlikely(vr4300_fpe_mode < 0)) {
    const char *e = getenv("CEN64_FPE");
    vr4300_fpe_mode = !(e != NULL && e[0] == '0');
  }
  return vr4300_fpe_mode;
}

static inline int fpe_bad32(uint32_t x) { // denormal or NaN
  return (x & 0x7FFFFF) != 0 && ((x & 0x7F800000) == 0 || (x & 0x7F800000) == 0x7F800000);
}

static inline int fpe_bad64(uint64_t x) {
  return (x & 0xFFFFFFFFFFFFFULL) != 0 && ((x & 0x7FF0000000000000ULL) == 0 ||
    (x & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL);
}

static inline int fpe_nan32(uint32_t x) {
  return (x & 0x7F800000) == 0x7F800000 && (x & 0x7FFFFF) != 0;
}

static inline int fpe_nan64(uint64_t x) {
  return (x & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL && (x & 0xFFFFFFFFFFFFFULL) != 0;
}

cen64_cold static int vr4300_fpe_raise(struct vr4300 *vr4300,
  uint32_t iw, uint32_t cause, uint64_t a, uint64_t b) {
  uint32_t fcr31 = vr4300->regs[VR4300_CP1_FCR31];

  vr4300->regs[VR4300_CP1_FCR31] = (fcr31 & ~0x3F000u) | cause;
  fprintf(stderr, "FPE,pc=%08x,iw=%08x,cause=%s,a=%llx,b=%llx\n",
    (uint32_t) vr4300->pipeline.rfex_latch.common.pc, iw,
    cause == FCR31_CAUSE_E ? "E" : cause == FCR31_CAUSE_V ? "V" : "Z",
    (unsigned long long) a, (unsigned long long) b);
  VR4300_FPE(vr4300);
  return 1;
}

// Operands of an arithmetic/abs/neg/format conversion (fmt S or D).
static inline int vr4300_fpe_in(struct vr4300 *vr4300, uint32_t iw,
  enum vr4300_fmt fmt, uint64_t fs, uint64_t ft, int two) {
  if (!vr4300_fpe_on())
    return 0;

  if (fmt == VR4300_FMT_S) {
    if (unlikely(fpe_bad32(fs) || (two && fpe_bad32(ft))))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, two ? ft : 0);
  }

  else if (fmt == VR4300_FMT_D) {
    if (unlikely(fpe_bad64(fs) || (two && fpe_bad64(ft))))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, two ? ft : 0);
  }

  return 0;
}

// x / 0 for finite non-zero x: Z when enabled (0 / 0 is an invalid operation: the result check).
static inline int vr4300_fpe_div(struct vr4300 *vr4300, uint32_t iw,
  enum vr4300_fmt fmt, uint64_t fs, uint64_t ft) {
  int zero, xzero;

  if (!vr4300_fpe_on() || !(vr4300->regs[VR4300_CP1_FCR31] & FCR31_ENABLE_Z))
    return 0;

  if (fmt == VR4300_FMT_S) {
    zero = ((uint32_t) ft & 0x7FFFFFFF) == 0;
    xzero = ((uint32_t) fs & 0x7FFFFFFF) == 0;
  } else {
    zero = (ft & 0x7FFFFFFFFFFFFFFFULL) == 0;
    xzero = (fs & 0x7FFFFFFFFFFFFFFFULL) == 0;
  }

  return zero && !xzero ? vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_Z, fs, ft) : 0;
}

// Result of an operation whose operands passed vr4300_fpe_in.
static inline int vr4300_fpe_out(struct vr4300 *vr4300, uint32_t iw,
  int is_d, uint64_t *result, uint64_t fs, uint64_t ft) {
  uint32_t fcr31;

  if (!vr4300_fpe_on())
    return 0;

  fcr31 = vr4300->regs[VR4300_CP1_FCR31];

  if (!is_d) {
    uint32_t r = *result, e = r & 0x7F800000, m = r & 0x7FFFFF;

    if (likely(e != 0 && e != 0x7F800000) || m == 0)
      return 0;

    if (e == 0x7F800000) {
      if (fcr31 & FCR31_ENABLE_V)
        return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_V, fs, ft);

      *result = 0x7FBFFFFF;
      return 0;
    }

    if (!(fcr31 & FCR31_FS) || (fcr31 & FCR31_ENABLE_UI))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, ft);

    switch (fcr31 & 3) {
      case 2: *result = (r & 0x80000000) ? 0x80000000 : 0x00800000; break;
      case 3: *result = (r & 0x80000000) ? 0x80800000 : 0x00000000; break;
      default: *result = r & 0x80000000; break;
    }
  }

  else {
    uint64_t r = *result, e = r & 0x7FF0000000000000ULL, m = r & 0xFFFFFFFFFFFFFULL;

    if (likely(e != 0 && e != 0x7FF0000000000000ULL) || m == 0)
      return 0;

    if (e == 0x7FF0000000000000ULL) {
      if (fcr31 & FCR31_ENABLE_V)
        return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_V, fs, ft);

      *result = 0x7FF7FFFFFFFFFFFFULL;
      return 0;
    }

    if (!(fcr31 & FCR31_FS) || (fcr31 & FCR31_ENABLE_UI))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, ft);

    switch (fcr31 & 3) {
      case 2: *result = (r >> 63) ? 0x8000000000000000ULL : 0x0010000000000000ULL; break;
      case 3: *result = (r >> 63) ? 0x8010000000000000ULL : 0; break;
      default: *result = r & 0x8000000000000000ULL; break;
    }
  }

  return 0;
}

// Conversion to a 32- or 64-bit integer; mode 0 nearest (even), 1 truncate, 2 ceil, 3 floor,
// -1 the FCR31 rounding mode.
static inline int vr4300_fpe_cvt_int(struct vr4300 *vr4300, uint32_t iw,
  enum vr4300_fmt fmt, uint64_t fs, int bits, int mode) {
  double x, r;

  if (!vr4300_fpe_on())
    return 0;

  if (fmt == VR4300_FMT_S) {
    uint32_t v = fs;
    float f;

    if ((v & 0x7F800000) == 0x7F800000 || fpe_bad32(v))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, 0);

    memcpy(&f, &v, sizeof(f));
    x = f;
  }

  else if (fmt == VR4300_FMT_D) {
    if ((fs & 0x7FF0000000000000ULL) == 0x7FF0000000000000ULL || fpe_bad64(fs))
      return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, 0);

    memcpy(&x, &fs, sizeof(x));
  }

  else
    return 0;

  if (mode < 0) // FCR31.RM: 0 nearest, 1 toward zero, 2 toward +inf, 3 toward -inf
    mode = vr4300->regs[VR4300_CP1_FCR31] & 3;

  switch (mode) {
    case 1: r = trunc(x); break;
    case 2: r = ceil(x); break;
    case 3: r = floor(x); break;
    default:
      r = floor(x + 0.5);
      if (r - x == 0.5 && fmod(r, 2.0) != 0.0)
        r -= 1.0;
      break;
  }

  if (bits == 32 ? (r < -2147483648.0 || r > 2147483647.0) :
    (r < -9223372036854775808.0 || r >= 9223372036854775808.0))
    return vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_E, fs, 0);

  return 0;
}

// C.cond: a NaN operand in a signalling compare (cond bit 3) is an invalid operation.
static inline int vr4300_fpe_cmp(struct vr4300 *vr4300, uint32_t iw,
  enum vr4300_fmt fmt, uint64_t fs, uint64_t ft) {
  int nan;

  if (!vr4300_fpe_on() || !(iw & 0x8) || !(vr4300->regs[VR4300_CP1_FCR31] & FCR31_ENABLE_V))
    return 0;

  nan = fmt == VR4300_FMT_S ? fpe_nan32(fs) || fpe_nan32(ft) : fpe_nan64(fs) || fpe_nan64(ft);
  return nan ? vr4300_fpe_raise(vr4300, iw, FCR31_CAUSE_V, fs, ft) : 0;
}

//
// Raises a MCI interlock for a set number of cycles.
//
static inline int vr4300_do_mci(struct vr4300 *vr4300, unsigned cycles) {
  vr4300->pipeline.cycles_to_stall = cycles - 1;
  vr4300->regs[PIPELINE_CYCLE_TYPE] = 3;
  return 1;
}

//
// ABS.fmt
//
int VR4300_CP1_ABS(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, 0, 0)))
    return 0;

  uint32_t fs32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_abs_32(&fs32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_abs_64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 3);
}

//
// ADD.fmt
//
int VR4300_CP1_ADD(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, ft, 1)))
    return 0;

  uint32_t fs32, ft32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      fpu_add_32(&fs32, &ft32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_add_64(&fs, &ft, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (unlikely(vr4300_fpe_out(vr4300, iw, fmt == VR4300_FMT_D, &result, fs, ft)))
    return 0;
  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 3);
}

//
// BC1F
// BC1FL
// BC1T
// BC1TL
//
int VR4300_BC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_icrf_latch *icrf_latch = &vr4300->pipeline.icrf_latch;
  struct vr4300_rfex_latch *rfex_latch = &vr4300->pipeline.rfex_latch;
  unsigned opcode = (iw >> 16) & 0x3;

  uint64_t offset = (uint64_t) ((int16_t) iw) << 2;
  uint64_t taken_pc = rfex_latch->common.pc + (offset + 4);
  uint32_t cond = vr4300->regs[VR4300_CP1_FCR31];

  // XXX: The VR4300 manual says that the results of a FPU
  // FCR writes aren't ready on the next cycle, but it seems
  // that this might actually not be a limitiation of the real
  // hardware?
  if (vr4300->pipeline.dcwb_latch.dest == VR4300_CP1_FCR31)
    cond = vr4300->pipeline.dcwb_latch.result;

  switch (opcode) {
    case 0x0: // BC1F
      if (!(cond >> 23 & 0x1))
        icrf_latch->pc = taken_pc;
      break;

    case 0x1: // BC1T
      if (cond >> 23 & 0x1)
        icrf_latch->pc = taken_pc;
      break;

    case 0x2: // BC1FL
      if (!(cond >> 23 & 0x1))
        icrf_latch->pc = taken_pc;
      else
        rfex_latch->iw_mask = 0;
      break;

    case 0x3: // BC1TL
      if (cond >> 23 & 0x1)
        icrf_latch->pc = taken_pc;
      else
        rfex_latch->iw_mask = 0;
      break;
  }

  return 0;
}

//
// C.eq.fmt
// C.seq.fmt
//
int VR4300_CP1_C_EQ_C_SEQ(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_eq_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_eq_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.f.fmt
// C.sf.fmt
//
int VR4300_CP1_C_F_C_SF(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_f_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_f_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.ole.fmt
// C.le.fmt
//
int VR4300_CP1_C_OLE_C_LE(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_ole_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_ole_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.olt.fmt
// C.lt.fmt
//
int VR4300_CP1_C_OLT_C_LT(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_olt_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_olt_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.ueq.fmt
// C.ngl.fmt
//
int VR4300_CP1_C_UEQ_C_NGL(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_ueq_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_ueq_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.ule.fmt
// C.ngt.fmt
//
int VR4300_CP1_C_ULE_C_NGT(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_ule_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_ule_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.ult.fmt
// C.nge.fmt
//
int VR4300_CP1_C_ULT_C_NGE(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_ult_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_ult_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// C.un.fmt
// C.ngle.fmt
//
int VR4300_CP1_C_UN_C_NGLE(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = VR4300_CP1_FCR31;
  if (unlikely(vr4300_fpe_cmp(vr4300, iw, fmt, fs, ft)))
    return 0;
  uint64_t result = vr4300->regs[dest];

  uint32_t fs32, ft32;
  uint8_t flag;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      result &= ~(1 << 23);
      flag = fpu_cmp_un_32(&fs32, &ft32);
      break;

    case VR4300_FMT_D:
      result &= ~(1 << 23);
      flag = fpu_cmp_un_64(&fs, &ft);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result | (flag << 23);
  exdc_latch->dest = dest;
  return 0;
}

//
// CEIL.l.fmt
//
int VR4300_CP1_CEIL_L(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 64, 2)))
    return 0;

  uint32_t fs32;
  uint64_t result;

#ifndef CEN64_ARCH_HAS_CEIL
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_POSINF);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_ceil_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_ceil_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// CEIL.w.fmt
//
int VR4300_CP1_CEIL_W(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 32, 2)))
    return 0;

  uint32_t fs32;
  uint32_t result;

#ifndef CEN64_ARCH_HAS_CEIL
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_POSINF);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_ceil_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_ceil_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// CFC1
//
int VR4300_CFC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t rt) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  unsigned dest = GET_RT(iw);
  unsigned src = GET_RD(iw);
  uint64_t result;

  switch (src) {
    case 0: src = VR4300_CP1_FCR0; break;
    case 31: src = VR4300_CP1_FCR31; break;

    default:
      src = 0;

      assert(0 && "CFC1: Read reserved FCR.");
      break;
  }

  result = vr4300->regs[src];

  // XXX: The VR4300 manual says that the results of a FPU
  // FCR writes aren't ready on the next cycle, but it seems
  // that this might actually not be a limitiation of the real
  // hardware?
  if (vr4300->pipeline.dcwb_latch.dest == VR4300_CP1_FCR31)
    result = vr4300->pipeline.dcwb_latch.result;

  // Undefined while the next instruction
  // executes, so we can cheat and use the RF.
  exdc_latch->result = (int32_t) result;
  exdc_latch->dest = dest;
  return 0;
}

//
// CTC1
//
// XXX: Raise exception on cause/enable.
// XXX: In such cases, ensure write occurs.
//
int VR4300_CTC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t rt) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  unsigned dest = GET_RD(iw);

  if (dest == 31)
    dest = VR4300_CP1_FCR31;

  else {
    assert(0 && "CTC1: Write to fixed/reserved FCR.");

    dest = 0;
    rt = 0;
  }

  // Undefined while the next instruction
  // executes, so we can cheat and use WB.
  exdc_latch->result = rt;
  exdc_latch->dest = dest;
  return 0;
}

//
// CVT.d.fmt
//
int VR4300_CP1_CVT_D(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (fmt == VR4300_FMT_S && unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, 0, 0)))
    return 0;

  uint32_t fs32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_f64_f32(&fs32, &result);
      break;

    case VR4300_FMT_W:
      fs32 = fs;

      fpu_cvt_f64_i32(&fs32, &result);
      break;

    case VR4300_FMT_L:
      fpu_cvt_f64_i64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return fmt != VR4300_FMT_S
    ? vr4300_do_mci(vr4300, 5)
    : 0;
}

//
// CVT.l.fmt
//
int VR4300_CP1_CVT_L(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 64, -1)))
    return 0;

  uint32_t fs32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// CVT.s.fmt
//
int VR4300_CP1_CVT_S(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (fmt == VR4300_FMT_D && unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, 0, 0)))
    return 0;

  uint32_t fs32;
  uint32_t result;

  switch (fmt) {
    case VR4300_FMT_D:
      fpu_cvt_f32_f64(&fs, &result);
      break;

    case VR4300_FMT_W:
      fs32 = fs;

      fpu_cvt_f32_i32(&fs32, &result);
      break;

    case VR4300_FMT_L:
      fpu_cvt_f32_i64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (fmt == VR4300_FMT_D) {
    uint64_t r64 = result;

    if (unlikely(vr4300_fpe_out(vr4300, iw, 0, &r64, fs, 0)))
      return 0;

    result = r64;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300,
    fmt == VR4300_FMT_D ? 2 : 5);
}

//
// CVT.w.fmt
//
int VR4300_CP1_CVT_W(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 32, -1)))
    return 0;

  uint32_t fs32;
  uint32_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// DIV.fmt
//
int VR4300_CP1_DIV(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, ft, 1)))
    return 0;
  if (unlikely(vr4300_fpe_div(vr4300, iw, fmt, fs, ft)))
    return 0;

  uint32_t fs32, ft32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      fpu_div_32(&fs32, &ft32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_div_64(&fs, &ft, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (unlikely(vr4300_fpe_out(vr4300, iw, fmt == VR4300_FMT_D, &result, fs, ft)))
    return 0;
  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300,
    fmt == VR4300_FMT_D ? 58 : 29);
}

//
// DMFC1
//
int VR4300_DMFC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t unused(rt)) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  unsigned dest = GET_RT(iw);

  exdc_latch->result = fs;
  exdc_latch->dest = dest;
  return 0;
}

//
// DMTC1
//
int VR4300_DMTC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t rt) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  unsigned dest = GET_FS(iw);

  exdc_latch->result = rt;
  exdc_latch->dest = dest;
  return 0;
}

//
// FLOOR.l.fmt
//
int VR4300_CP1_FLOOR_L(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 64, 3)))
    return 0;

  uint32_t fs32;
  uint64_t result;

#ifndef CEN64_ARCH_HAS_FLOOR
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_NEGINF);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_floor_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_floor_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// FLOOR.w.fmt
//
int VR4300_CP1_FLOOR_W(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 32, 3)))
    return 0;

  uint32_t fs32;
  uint32_t result;

#ifndef CEN64_ARCH_HAS_FLOOR
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_NEGINF);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_floor_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_floor_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// LDC1
//
// TODO/FIXME: Check for unaligned addresses.
//
int VR4300_LDC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t rt) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  unsigned dest = GET_FT(iw);

  exdc_latch->request.vaddr = rs + (int16_t) iw;
  exdc_latch->request.data = ~0ULL;
  exdc_latch->request.wdqm = 0ULL;
  exdc_latch->request.postshift = 0;
  exdc_latch->request.access_type = VR4300_ACCESS_DWORD;
  exdc_latch->request.type = VR4300_BUS_REQUEST_READ;
  exdc_latch->request.size = 8;

  exdc_latch->dest = dest;
  exdc_latch->result = 0;
  return 0;
}

//
// LWC1
//
// TODO/FIXME: Check for unaligned addresses.
//
int VR4300_LWC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  uint32_t status = vr4300->regs[VR4300_CP0_REGISTER_STATUS];
  uint64_t address = (rs + (int16_t) iw);
  unsigned dest = GET_FT(iw);

  uint64_t result = 0;
  unsigned postshift = 0;

  if (!(status & 0x04000000)) {
    result = dest & 0x1
      ? ft & 0x00000000FFFFFFFFULL
      : ft & 0xFFFFFFFF00000000ULL;

    postshift = (dest & 0x1) << 5;
    dest &= ~0x1;
  }

  exdc_latch->request.vaddr = address;
  exdc_latch->request.data = ~0U;
  exdc_latch->request.wdqm = 0ULL;
  exdc_latch->request.postshift = postshift;
  exdc_latch->request.access_type = VR4300_ACCESS_WORD;
  exdc_latch->request.type = VR4300_BUS_REQUEST_READ;
  exdc_latch->request.size = 4;

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return 0;
}

//
// MUL.fmt
//
int VR4300_CP1_MUL(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, ft, 1)))
    return 0;

  uint32_t fs32, ft32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      fpu_mul_32(&fs32, &ft32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_mul_64(&fs, &ft, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (unlikely(vr4300_fpe_out(vr4300, iw, fmt == VR4300_FMT_D, &result, fs, ft)))
    return 0;
  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300,
    fmt == VR4300_FMT_D ? 8 : 5);
}

//
// MFC1
//
int VR4300_MFC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t unused(rt)) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  uint32_t status = vr4300->regs[VR4300_CP0_REGISTER_STATUS];
  unsigned dest = GET_RT(iw);
  uint64_t result;

  if (status & 0x04000000)
    result = (int32_t) fs;

  else {
    result = (GET_FS(iw) & 0x1)
      ? (int32_t) (fs >> 32)
      : (int32_t) (fs);
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return 0;
}

//
// MOV.fmt
//
int VR4300_CP1_MOV(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  unsigned dest = GET_FD(iw);

  exdc_latch->result = fs;
  exdc_latch->dest = dest;
  return 0;
}

//
// MTC1
//
int VR4300_MTC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t rt) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  uint32_t status = vr4300->regs[VR4300_CP0_REGISTER_STATUS];
  uint64_t result = (int32_t) rt;
  unsigned dest = GET_FS(iw);

  if (!(status & 0x04000000)) {
    result = (dest & 0x1)
      ? ((uint32_t) fs) | (rt << 32)
      : (fs & ~0xFFFFFFFFULL) | ((uint32_t) rt);

    dest &= ~0x1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return 0;
}

//
// NEG.fmt
//
int VR4300_CP1_NEG(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, 0, 0)))
    return 0;

  uint32_t fs32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_neg_32(&fs32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_neg_64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return 0;
}

//
// ROUND.l.fmt
//
int VR4300_CP1_ROUND_L(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 64, 0)))
    return 0;

  uint32_t fs32;
  uint64_t result;

#ifndef CEN64_ARCH_HAS_ROUND
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_NEAREST);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_round_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_round_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// ROUND.w.fmt
//
int VR4300_CP1_ROUND_W(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 32, 0)))
    return 0;

  uint32_t fs32;
  uint32_t result;

#ifndef CEN64_ARCH_HAS_ROUND
  fpu_state_t saved_state = fpu_get_state();
  fpu_set_state((saved_state & ~FPU_ROUND_MASK) | FPU_ROUND_NEAREST);

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_cvt_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_cvt_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;

  }

  fpu_set_state((saved_state & FPU_ROUND_MASK) |
    (fpu_get_state() & ~FPU_ROUND_MASK));
#else
  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_round_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_round_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }
#endif

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// SDC1
//
// TODO/FIXME: Check for unaligned addresses.
//
int VR4300_SDC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;

  exdc_latch->request.vaddr = rs + (int16_t) iw;
  exdc_latch->request.data = ft;
  exdc_latch->request.wdqm = ~0ULL;
  exdc_latch->request.access_type = VR4300_ACCESS_DWORD;
  exdc_latch->request.type = VR4300_BUS_REQUEST_WRITE;
  exdc_latch->request.size = 8;

  return 0;
}

//
// SQRT.fmt
//
int VR4300_CP1_SQRT(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, 0, 0)))
    return 0;

  uint32_t fs32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_sqrt_32(&fs32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_sqrt_64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (unlikely(vr4300_fpe_out(vr4300, iw, fmt == VR4300_FMT_D, &result, fs, ft)))
    return 0;
  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300,
    fmt == VR4300_FMT_D ? 58 : 29);
}

//
// SUB.fmt
//
int VR4300_CP1_SUB(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_in(vr4300, iw, fmt, fs, ft, 1)))
    return 0;

  uint32_t fs32, ft32, fd32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;
      ft32 = ft;

      fpu_sub_32(&fs32, &ft32, &fd32);
      result = fd32;
      break;

    case VR4300_FMT_D:
      fpu_sub_64(&fs, &ft, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  if (unlikely(vr4300_fpe_out(vr4300, iw, fmt == VR4300_FMT_D, &result, fs, ft)))
    return 0;
  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 3);
}

//
// SWC1
//
// TODO/FIXME: Check for unaligned addresses.
//
int VR4300_SWC1(struct vr4300 *vr4300,
  uint32_t iw, uint64_t rs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  uint32_t status = vr4300->regs[VR4300_CP0_REGISTER_STATUS];
  unsigned ft_reg = GET_FT(iw);

  if (!(status & 0x04000000))
    ft >>= ((ft_reg & 0x1) << 5);

  exdc_latch->request.vaddr = rs + (int16_t) iw;
  exdc_latch->request.data = ft;
  exdc_latch->request.wdqm = ~0U;
  exdc_latch->request.access_type = VR4300_ACCESS_WORD;
  exdc_latch->request.type = VR4300_BUS_REQUEST_WRITE;
  exdc_latch->request.size = 4;

  return 0;
}

//
// TRUNC.l.fmt
//
int VR4300_CP1_TRUNC_L(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 64, 1)))
    return 0;

  uint32_t fs32;
  uint64_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_trunc_i64_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_trunc_i64_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

//
// TRUNC.w.fmt
//
int VR4300_CP1_TRUNC_W(struct vr4300 *vr4300,
  uint32_t iw, uint64_t fs, uint64_t ft) {
  struct vr4300_exdc_latch *exdc_latch = &vr4300->pipeline.exdc_latch;
  enum vr4300_fmt fmt = GET_FMT(iw);
  unsigned dest = GET_FD(iw);
  if (unlikely(vr4300_fpe_cvt_int(vr4300, iw, fmt, fs, 32, 1)))
    return 0;

  uint32_t fs32;
  uint32_t result;

  switch (fmt) {
    case VR4300_FMT_S:
      fs32 = fs;

      fpu_trunc_i32_f32(&fs32, &result);
      break;

    case VR4300_FMT_D:
      fpu_trunc_i32_f64(&fs, &result);
      break;

    default:
      VR4300_INV(vr4300);
      return 1;
  }

  exdc_latch->result = result;
  exdc_latch->dest = dest;
  return vr4300_do_mci(vr4300, 5);
}

// Initializes the coprocessor.
void vr4300_cp1_init(struct vr4300 *vr4300) {
  fpu_set_state(FPU_ROUND_NEAREST | FPU_MASK_EXCPS);
  vr4300->regs[VR4300_CP1_FCR0] = 0xa00; // hardcoded fpu version of both 0xb22 and 0xb10 N64s
}
