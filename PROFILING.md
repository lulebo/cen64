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

## RDP timing model (`-rdptime`, `-rdpmodel`) and a slower RSP (`-rspslow`)

Stock cen64 renders every RDP command the moment `DPC_END` is written and raises the DP
interrupt immediately, so the RDP is infinitely fast: the RSP never waits on a full FIFO,
the game never sees an RDP tail after the RSP finished, and frame pacing that depends on
that (60 fps pipelines) cannot be judged. `-rdptime` keeps the immediate execution (pixels
are unchanged) but charges every command to a modelled RDP clock (62.5 MHz) and lets the
visible side effects lag behind it:

- `DPC_CURRENT` reads back the address of the command the modelled RDP is executing;
  `DPC_STATUS` shows START_VALID (a DPC_START latched but not yet taken by a DPC_END),
  END_VALID (a transfer queued behind the running one) and the busy bits; a DPC_START
  written while busy is taken when the running transfer has drained, as on hardware.
- The DP interrupt of a full sync fires when the model reaches it.
- `RDPT,<frame>,<cycles>,<px 1-cycle>,<px 2-cycle>,<px fill>,<px copy>,<z pixels>,<tris>,<cmds>,<tmem loads>,<spans>,<spans with image read>,<spans with z compare>`
  is printed with every `RDP,` line: the modelled busy cycles of the frame and the work
  behind them. A span is one valid scanline of one primitive; an FPGA RDP (MiSTer) fetches
  the framebuffer line (image read on) and then the z line (z compare on) from DDR3 at the
  start of every span, sequentially, before it draws a pixel, so spans are a cost of their own.

Cost per command = cmd + tri/rect/tmem overhead + pixels * per-mode cost + z pixels * z cost,
with pixels = max(colour writes, z reads) of that command (so z-rejected pixels count).
`-rdpmodel px1,px2,fill,copy,z,tri,rect,cmd,tmem[,sync,span,spanfb,spanz]` sets the cycle costs (defaults
`1,2,0.25,0.25,0.5,64,32,8,256,200,0,0,0`; span = per span, spanfb / spanz = extra per span with image read / z compare); `CEN64_RDP_MODEL` in the environment does the same.
These are guesses to be calibrated against hardware (the ROM's HWSTATS line prints the
RDP tail after the RSP, `P`, and the RSP idle share, `I`).

`-rspslow N` inserts one stall cycle every N RSP cycles (N=10 = an RSP 10% slower than
cen64's), to reproduce the overloaded regime of a slower RSP implementation such as an FPGA
core without editing the microcode.

Ring size: 65536 commands in flight (about ten frames); the model forces the oldest to
finish if it ever fills.
