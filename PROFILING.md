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
