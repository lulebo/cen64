# cen64 profiling / benchmarking extras

This branch adds headless measurement hooks for profiling and benchmarking N64 code.
Everything is opt-in through command-line flags and environment variables; default behaviour
is unchanged.

## Headless runs
- `-headless` now blocks until the emulated device thread ends (upstream returned at once),
  and stdout is line-buffered so piped output arrives per line. Combine with `-is-viewer` to
  read the ROM's IS-Viewer text output as it is printed.

## RDP work counters (rdp/n64video.c)
On every RDP full sync (once per rendered frame) a line is printed:

    RDP,frame,tris,tris_1cyc,tris_2cyc,tris_copy,tris_fill,rects,fillrects,tmem_loads,fbwrite,fbfill,fbread,zread,zwrite,fbhash,imem_dma

`fbhash` is an FNV hash of the colour framebuffer at that moment (a per-frame pixel
fingerprint), `imem_dma` the number of DMAs into RSP IMEM since the last frame (microcode
loads and F3DEX2 overlay swaps, i.e. clipping work). Note that cen64/angrylion does not model
RDP execution *time*; these are work counts.

- `CEN64_DUMP_FB="<dir>:<every>[:<from>:<to>]"` writes `fb_<frame>.ppm` (binary P6) of the
  16-bit framebuffer for every N-th frame in the range.
- `CEN64_DUMP_CMD="<from>:<to>"` prints every RDP command of those frames as
  `CMD,<frame>,<opcode>,<hex words...>` — diffing two ROMs' streams shows exactly which
  triangles or vertices changed.

## CPU profiler (vr4300)
`-profile` (upstream's per-PC instruction and L1D-miss counters) is extended with two regions:
- cycles: every CPU cycle is charged to the instruction in the DC stage, so cache-miss stalls
  land on the load/store that caused them;
- I-cache misses, charged to the fetched address.

`CEN64_PROFILE_DIR=<dir>` resets the counters when the ROM prints an IS-Viewer line starting
with `BENCH_START,` and writes `<dir>/<name>.profile` on `BENCH_END,<name>,...`, one line per
address: `pc instructions l1d_misses cycles icache_misses`. Aggregate by function with the
ELF symbol table (`nm -n` and a nearest-preceding-symbol lookup). Without the environment
variable, upstream's `<rom>.profile` at exit still works. Profiling slows emulation by
roughly a third.

The N64's I-cache is 16 KB direct-mapped: functions whose addresses are equal modulo 16 KB
evict each other on every call. The I-cache column makes that visible per function, which
is what you need to decide a linker-level code placement.

## RSP issue timing model (`-rsphw`)

The stock RSP pipeline issues one instruction per cycle. The real RSP dual-issues one
scalar (SU) and one vector (VU) instruction per cycle and has register latencies that the
stock model does not have, so hand-scheduled microcode (F3DEX2, F3DEX3) runs faster on the
chip than in the emulator, and the difference is not uniform between microcodes.
`-rsphw` layers an issue-timing model on the functional pipeline, following the rules
documented at n64brew (Reality Signal Processor / CPU Pipeline):

- SU + VU neighbours dual-issue, in either order, unless the first writes a vector
  register the second reads or writes; a branch delay slot never dual-issues; the first
  instruction at a taken branch's target pairs only when 8-byte aligned.
- A vector register written by any instruction (VU op, vector load, `mtc2`) is readable
  4 cycles later; a scalar register written by a DMEM load, `mfc0`, `mfc2` or `cfc2` after
  3 cycles; readers stall until then.
- A store (or any cop0/cop2 move) issued exactly 2 cycles after a load stalls 1 cycle.
- Branches take 3 cycles including the delay slot.

Each device cycle grants one credit; an instruction executes once `1 + stalls` credits are
available, a dual-issued partner costs 0 and runs in the same device cycle. Everything else
(DMA, RDP, interrupts) is unchanged, so the model only changes *when* instructions run.
Not validated against hardware: use it for relative comparisons between microcodes or
display lists, and confirm on a console. `RSPHW,instructions,pairs,stallcycles,branches`
is printed on stdout at every `BENCH_END,` IS-Viewer line (counters reset at
`BENCH_START,`), and a summary goes to stderr at exit.

## RSP program-counter profile (`-rspprof`)

Cycles per IMEM word, one histogram per microcode (keyed by the OSTask ucode address
read from DMEM when the RSP is started). With `CEN64_PROFILE_DIR` set, each `BENCH_END,`
IS-Viewer line writes `<name>.rspprof` (`ucode pc cycles`, reset at `BENCH_START,`).
Under `-rsphw` the credit cost of each instruction is attributed. Symbolise with an
armips `-sym` file of the microcode (e.g. Wiseguy's F3DEX2 build, or a hand-made list of
the audio ucode's command handlers from its dispatch table).
