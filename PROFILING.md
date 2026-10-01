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

## Overdraw tracer (`CEN64_OVERDRAW`)

`CEN64_OVERDRAW="<from>:<to>[:<dir>[:p]]"` traces RDP frames `from..to` (frames are counted at
full syncs, like the `RDP,` lines); `CEN64_OVERDRAW_EVERY=N` limits the per-primitive lines and the
pixel dumps to frames divisible by N. Pixels are unchanged; the tracer only counts.

- Every primitive (triangle, texture rectangle, fill rectangle) records the pixels it
  rasterized, how many failed the z test or the alpha/coverage test, how many it wrote
  *directly* (the result does not depend on the framebuffer: no blender input and no stored
  coverage uses the memory value) or *dependently*, how many of its written pixels a later
  primitive overwrote directly or dependently, how many it still owns when the frame ends, and
  how many final pixels are built on its direct write (`base`). Its spans and walked lines
  (including lines above the scissor) are counted too.
- Groups come from marker commands: a SetConvert (0xEC) whose first word has 0xA in bits 21..18
  starts a group; bits 17..0 and the second word are the group's tag (the ROM decides what they
  mean). Kind 5 (bits 17..15) is printed once per frame as `OVB,<frame>,<slot>,<addr>` instead.
- Output per traced frame: `OVF,<frame>,<prims>,<groups>,<px 1-cycle>,<px 2-cycle>,<px fill>,
  <final pixels>,<traced pixels>`; per group `OVG,<frame>,<tag hi>,<tag lo>,<prims>,<px1>,<px2>,
  <pxfill>,<zfail>,<afail>,<wdir>,<wdep>,<odir>,<odep>,<final>,<spans>,<spans z>,<spans fb>,<rows>,
  <rows above>,<hidden prims>,<hidden px1>,<hidden px2>,<hidden pxfill>,<hidden spans>,
  <hidden spans z>,<hidden spans fb>,<2-cycle px of prims with shade alpha 0>` where *hidden*
  primitives own no final pixel and were never blended into one (dropping them leaves the frame
  identical); with `p`, `OVP` per primitive; with a directory, `ovpix_<frame>.bin` = u32 width,
  u32 height, then per pixel u32 final owner (primitive index + 1), u16 rasterization count and
  u32 base primitive.

## MiSTer core RSP timing (`-rspmister`)

The RSP of the MiSTer N64 core (MiSTer-devel/N64_MiSTer, rtl/RSP_core.vhd) issues one
instruction per cycle with no branch bubble, but a scalar DMEM load (lb/lbu/lh/lhu/lw/lwu) or
mfc0 freezes the pipeline until its writeback (+2 cycles, whether or not the result is used),
and a COP2 instruction whose raw vs field (bits 15..11) or vt field (20..16), or a vector store
whose vt field, names the target of one of the last three decode cycles' COP2 instructions or
vector loads stalls 3, 2 or 1 cycles. The core compares raw fields, so the element field of
vrcp/vmov, the scalar register of mfc2/mtc2 and consecutive mfc2 reads of one register create
dependencies too; load freezes do not advance that window. `-rspmister` layers this on the
functional pipeline like `-rsphw` (with `-rspprof` it attributes MiSTer cycles per IMEM word).
The `RSPHW,` line then reads instructions, load-freeze cycles, vector stall cycles, branches.

## RDP command hash (`CEN64_CMD_HASH=1`)

`CMDH,<frame>,<commands>,<hash>,<triangles>,<triangle hash>` per frame: FNV-1a over every RDP
command word, and over the triangle commands (0x08-0x0F) alone. The second word of no-op and
sync commands is skipped (microcodes only write the first; the rest is stale buffer memory the
RDP does not read). Used to prove that two microcodes produce identical output.

## RDRAM traffic counters (`BUS,` lines)

Every `BENCH_END,` IS-Viewer line prints
`BUS,<name>,<cpu cycles>,<rdp full syncs>,<cpu I fills>,<uncached I fetches>,<D fills>,<dirty D write-backs on a miss>,
<write-backs by cache instructions>,<uncached RDRAM reads>,<bytes>,<uncached writes>,<bytes>,<uncached non-RDRAM accesses>,
<RSP DMAs in>,<bytes>,<RSP DMAs out>,<bytes>,<audio task bytes in>,<out>,<RDP command bytes>,<texture load bytes>,
<fb pixels read>,<written>,<filled>,<z read>,<z written>,<VI bytes>,<AI bytes>,<PI bytes cart->RDRAM>,<RDRAM->cart>,0`
(counts since `BENCH_START,`). With `CEN64_PROFILE_DIR`, the CPU profile gains two columns
(`pc ins l1d cyc ic dwb unc`: dirty write-backs charged to the load/store or cache instruction that
caused them, uncached RDRAM accesses), and `<name>.dprofile` lists D-cache fills and write-backs per
16-byte RDRAM line (`addr fills writebacks`): which data, not which code, costs RAM reads.

## RDRAM contention model (`-rdram`, `-rdrammodel name=value,...`)

cen64 charges every CPU cache miss a fixed number of cycles and completes RSP DMAs, RDP memory
accesses and VI fetches instantly, so no agent ever waits for another. `-rdram` (implies `-rdptime`,
needs the default single-threaded loop) books every transaction on one RDRAM channel instead:

- The channel serves transactions first-come-first-served in RDRAM clocks (4 ns, 4 per RCP cycle).
  Cost = `req + rdelay` (read) or `req + wdelay` (write) `+ bytes * per_byte`, plus `row_miss` when
  the bank's open row is another one, plus `tta` when a back-to-back transfer turns the channel
  around. Banks are `2^bank_bits` bytes (1 MB), each keeps one open row of `2^row_bits` bytes (2 KB)
  and, with `bank_busy`, takes its next RDP access no earlier than `bank_busy` clocks after the last
  one started (`bank_rdp=1`: between RDP accesses only). A refresh per VI line costs `refresh` clocks
  and closes all rows (`refresh_close`).
- CPU: D-cache fills (16 bytes), I-cache fills (32), uncached reads stall for `cpu_d` / `cpu_i` /
  `cpu_unc` cycles outside the RDRAM plus the actual wait and service time; a fill holds the channel
  `cpu_rocc` extra clocks. A dirty victim is written back first (`cpu_wbv` + its transfer), then the
  fill. Cache-instruction write-backs stall `cpu_wb` + wait + service. An uncached store completes
  `cpu_uncw` CPU cycles after its transfer (a doubleword is two word writes); the CPU waits only for
  the previous one. With `cpu_prio >= 0` a CPU transfer that finds another agent's transfer on the
  channel cuts in after `cpu_prio` clocks and the interrupted transfer finishes after it.
- RSP: a DMA is split into bursts of at most `burst` bytes inside one row; SP_DMA_BUSY / SP_DMA_FULL
  (and the SP_STATUS bits) stay set until the bursts are done. The data still moves at once.
- RDP (`eng=1`, the default): while angrylion executes a command it emits work items, replayed on
  the RDP's own clock as a pipeline. The front end takes command costs (`cmd`, + `tri` / `rect`),
  command fetches (`cmd_fetch` bytes; `fetch_block=1` waits for them), texture loads (bursts, then
  `tmem8` cycles per 8 bytes) and per span `span_r` cycles. A span is cut into chunks of `chunk`
  pixels (`align8=2`: chunk boundaries and transfers on octbytes); the front end issues a chunk's
  reads (z for z compare, colour for image read) once the pixel stage has finished the chunk
  `rdp_d` back; a read's data arrives `rd_lat` clocks after its transfer, which holds the channel
  for `rd_occ` of the read delay. The pixel stage takes `px1` / `px2` (`fill`, `copy`) cycles per
  pixel, + `span` + `span_rs` per read stream + `span_ws` per write stream at a span's first chunk,
  and posts the chunk's writes - only over the pixels that passed z (the span hook counts them).
  `rdp_dw` caps the chunks with writes outstanding (0: none), `rdp_rprio=1` puts a read before a
  write requested at the same time, `rdp_sb` makes a span's first read wait for the earlier writes
  (1) or pixels (2). Pipe and full syncs drain pixels and writes (+ `psync` / `sync`). The RDP takes
  the channel one transfer at a time, so other agents get their turn. An idle RDP pays `idle` cycles
  first. DPC_CURRENT, the busy bits and the DP interrupt follow the replay, as with `-rdptime`.
  `eng=0`: the earlier serial replay (reads, `chunk` / span pixels, writes; `overlap`).
- VI: one fetch of `VI_WIDTH * bpp` bytes per active line; PI: cart -> RDRAM writes in 128-byte
  bursts `pi_gap` RCP cycles apart. AI and SI are not modelled (tiny).

`RDRAM,<name>,<window clocks>,<cpu stall cycles>,<what an idle channel would have cost>,<rdp busy clocks>,
<rsp dma busy clocks>,<agent>:<transactions>:<bytes>:<busy clocks>:<wait clocks>:<row hits>:<row misses>...`
is printed at every `BENCH_END,` (agents cpu_i, cpu_d, cpu_wb, cpu_unc, rsp, rdp_cmd, rdp_tex, rdp_fb,
rdp_z, vi, pi, refresh), then `RDRAMX,<name>,<victim>,<opener>,<bank>,<row misses>` (whose transfer
closed whose open row), and the `RDPT,` busy column becomes the replayed RDP time. Experiments:
`cpu_rows` / `rsp_rows` give the CPU / RSP open rows of their own, `iso1..3_lo/hi` give address
ranges banks of their own, `cpu_bus` / `rsp_bus` scale their channel time.

**Calibration.** The defaults are fitted (2026-10-01) to a console's boot calibration tests (the
Super Mario 64 port's HWCAL: full-screen layers per render mode, 32x4 against 4x32 rects, z pass
against z fail, triangles, colour and z in one bank or two, command fetch from DMEM, CPU misses /
stores / uncached accesses alone and while the RDP draws): the RDP tests within 5.8% rms, the CPU and
contention tests within ~11%. What the console showed: a span costs a fixed ~14 cycles per read
stream and ~13 per write stream on top of its pixels, a z-failing span writes nothing, two read
streams in two banks are cheaper per byte than one in one bank, and a CPU miss waits little behind
the RDP but costs it ~14 cycles.

`CEN64_SPAN_TRACE=<file>` logs the RDP's work (`C <cmd> <addr> <len> <xbus>`, `S <y> <x> <len> <bpp/4>
<cycle type> <image read> <z compare> <z update> <pixels written> <first> <last> <colour addr> <z addr>`,
`T <addr> <bytes>`); `tools/rdram_replay.c` (build: `gcc -O2 -I. -Ibuild -Ios/unix/x86_64 -Ios/unix
-Iarch/x86_64 tools/rdram_replay.c bus/rdram_model.c`) replays traces through the model without the
emulator, one parameter string per stdin line, a few hundred lists per second: `R <trace> <list>
<cycles>` per list ending in a full sync, with the VI scanning (`-novi`: none; `-vimap <file>`:
`<trace> <list> <origin|off>` per list). That is how the RDP parameters were fitted. Use the model for
relative comparisons, then confirm on hardware.
