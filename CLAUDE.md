# daisy_patch_sampler — Project Reference

## Concept

Ping-pong buffer sampler for Daisy Patch (Eurorack). Two SDRAM buffer pairs alternate
roles: one pair records the live audio input while the other loops its previously
captured content. A Gate/Trigger on GATE_1 swaps the roles. Stereo split: ch1 on
IN1/OUT1, ch2 on IN2/OUT2.

```
  boot
   │
   ▼
RECORD_A+C / PLAY_B+D ──── GATE_1 rising edge ────► RECORD_B+D / PLAY_A+C
         ▲                                                    │
         └─────────────── GATE_1 rising edge ─────────────────┘
```

---

## Hardware I/O

| Signal     | Assignment                                                      |
|------------|-----------------------------------------------------------------|
| Audio In 1 | Ch1 source signal                                               |
| Audio In 2 | Ch2 source signal                                               |
| Audio Out 1| Ch1 output                                                      |
| Audio Out 2| Ch2 output                                                      |
| GATE_1     | Buffer swap trigger (rising edge) — processed even when stopped |
| CTRL_1     | Playback speed (0.25× → 1.0× → 4.0×, exponential, OLA)         |
| CTRL_2     | Playback volume (0.0 → 1.0, linear)                             |
| CTRL_3     | Input gain (0.0 → 2.0, unity at centre)                         |
| CTRL_4     | Dry / wet mix (0.0 = dry, 1.0 = wet) — active on all pages      |

---

## Buffers

- **Location:** SDRAM (`DSY_SDRAM_BSS`) — 64 MB available
- **Format:** `float`, mono, 48 kHz
- **Size:** 60 s × 48 000 samples/s = 2 880 000 samples per buffer
- **Memory:** 4 × 2 880 000 × 4 bytes ≈ **44 MB total** (~20 MB headroom)

```cpp
constexpr size_t MAX_SAMPLES = 48000u * 60u;   // 2,880,000
float DSY_SDRAM_BSS buf_a[MAX_SAMPLES];  // ch1 ping
float DSY_SDRAM_BSS buf_b[MAX_SAMPLES];  // ch1 pong
float DSY_SDRAM_BSS buf_c[MAX_SAMPLES];  // ch2 ping
float DSY_SDRAM_BSS buf_d[MAX_SAMPLES];  // ch2 pong
```

Record length is fixed at MAX_SAMPLES (60 s). The gate trigger — not a knob —
controls loop length by swapping buffers early.

---

## State Machine

### Variables (volatile — shared between audio ISR and main loop)

| Variable       | Written by    | Read by            | Meaning                              |
|----------------|---------------|--------------------|--------------------------------------|
| `g_state`      | audio ISR     | main loop (display)| Current state enum                   |
| `g_rec_pos`    | audio ISR     | main loop (display)| Write head in active rec buffer      |
| `g_play_pos`   | audio ISR     | main loop (display)| Grain 0 position (for display bar)   |
| `g_play_len_a` | audio ISR     | audio ISR          | Frozen length of buf_a/buf_c         |
| `g_play_len_b` | audio ISR     | audio ISR          | Frozen length of buf_b/buf_d         |
| `g_play_speed` | main loop     | audio ISR          | OLA playback rate (0.25×–4.0×)       |
| `g_play_level` | main loop     | audio ISR          | Playback volume                      |
| `g_input_gain` | main loop     | audio ISR          | Input gain                           |
| `g_wet_mix`    | main loop     | audio ISR          | Dry/wet balance                      |
| `g_filter_type`| main loop     | audio ISR          | Filter engine (OFF/COMB/COMB2/3/CONV)|
| `g_comb_delay` | main loop     | audio ISR          | Comb filter delay in samples         |
| `g_comb_alpha` | main loop     | audio ISR          | Comb filter feedback coefficient     |
| `g_ir_sel`     | main loop     | audio ISR          | Selected IR slot index (CONV mode)   |
| `g_running`    | main loop     | audio ISR          | Pause/resume toggle                  |
| `g_enc_delta`  | audio ISR     | main loop          | Encoder rotation accumulator         |
| `g_enc_pressed`| audio ISR     | main loop          | Encoder press flag                   |

Thread safety: 32-bit reads/writes are atomic on Cortex-M7. `volatile` alone is
sufficient — no mutexes needed.

### Swap logic (called in audio callback on gate trigger, even when stopped)

```
if RECORD_A_PLAY_B:
    play_len_a = rec_pos      // freeze what was just recorded
    state      = RECORD_B_PLAY_A
else:
    play_len_b = rec_pos
    state      = RECORD_A_PLAY_B
rec_pos  = 0
grain_pos[0,1] = 0            // OLA grains restart from buffer head
```

### Recording behaviour

- Writes `in[n] * g_input_gain` into the active buffer every sample.
- **Freeze** mode: when `rec_pos >= MAX_SAMPLES` (60 s), recording stops and
  waits for the next gate trigger. Gate trigger is honoured even while stopped.

---

## OLA Playback (Pitch-Invariant Speed Control)

Playback uses two-grain Overlap-Add (OLA) so pitch stays fixed regardless of speed.

```
GRAIN_SIZE = 2400 samples (50 ms at 48 kHz)
Overlap    = 50%  (grains staggered by GRAIN_SIZE/2 in window phase)
Window     = Hann — satisfies COLA so amplitude is constant at all speeds

grain 0:  [↑──────────↓]
grain 1:       [↑──────────↓]
output  = grain0 * hann(phase0) + grain1 * hann(phase1)  ← always sums to 1.0
```

Each grain reads the playback buffer at its own fractional position using linear
interpolation and advances by `speed` samples per output sample. Pitch stays
fixed because each grain always reads at rate 1; only the position advances faster
or slower.

**Speed mapping (CTRL_1):**
```
g_play_speed = powf(16.0f, k1 - 0.5f)
k1 = 0.0  →  0.25×  (−2 oct)
k1 = 0.5  →  1.0×   (unity)
k1 = 1.0  →  4.0×   (+2 oct)
```

**Known artefact:** At speeds below ~0.3×, the 50 ms grain period becomes audible
as a subtle flutter at ~20 Hz. To push the artefact below 10 Hz and make it
inaudible on most material, increase `GRAIN_SIZE` to `4800u` (100 ms). This uses
an additional ~10 KB of SRAM.

---

## Filter Bank (Page 2)

Applied to the loop signal (after OLA, before output mix). Five modes on K1:

| FilterType | K1 zone   | Description |
|------------|-----------|-------------|
| `OFF`      | 0.00–0.20 | Bypass |
| `COMB`     | 0.20–0.40 | 1-tap feedback comb: y[n] = x[n] + α·y[n-D] |
| `COMB2`    | 0.40–0.60 | 2 taps at D₁, D₁×0.75 |
| `COMB3`    | 0.60–0.80 | 3 taps at D₁, D₁×0.75, D₁×0.55 |
| `CONV`     | 0.80–1.00 | Time-domain FIR convolution against a loaded IR |

### Comb modes

All taps share one circular buffer (`CombState`, ~75 KB SRAM).
Room-shape constants at top of filter bank section: `ALPHA_DECAY`, `TAP2_RATIO`, `TAP3_RATIO`.

**K2:** delay 1–100 ms logarithmic (fundamental = Fs/delay)
**K3:** feedback α₁ = 0.0–0.95

Comb state is cleared when transitioning from OFF → any comb type.

### Convolution mode (`CONV`)

Time-domain FIR with IR loaded from SD card. Input history buffer (`ConvState`) lives
in DTCM (`DTCM_MEM_SECTION`) for 0-wait-state access. IR data (SDRAM) is streamed
sequentially and fits in the M7's 16 KB D-cache after the first block.

**K2:** IR slot selector (quantised across loaded IRs)
**K3:** unused in CONV mode

**IR file format:** 48 kHz, mono, 16-bit PCM WAV named `IR_00.wav`…`IR_07.wav` at
SD card root. Loaded at startup (before `StartAudio`), normalised to peak = 1.0.
Unmount happens after loading — SD card not accessed during audio.

**CPU cost:** N=1024 (21 ms) ≈ 20% extra; N=2048 (43 ms) ≈ 41% extra.
**Max IR length:** `MAX_IR_LEN = 2048` samples. Longer IRs are truncated at load time.

**FatFS note:** `ffascii.c` in the source directory provides minimal `ff_convert` /
`ff_wtoupper` stubs (ASCII-only). The full `ccsbcs.c` code-page table exceeds the
FLASH budget by ~1.1 KB and must not be used.

```cpp
constexpr uint8_t MAX_IR_SLOTS = 8u;
constexpr size_t  MAX_IR_LEN   = 1024u;  // 1024 taps ≈ 21 ms; safe with ARM_MATH_LOOPUNROLL (~26% CPU)
float DSY_SDRAM_BSS ir_data[MAX_IR_SLOTS][MAX_IR_LEN];  // 32 KB in SDRAM
static ConvState DTCM_MEM_SECTION conv;                  // 8 KB in DTCM
```

### "I Am Sitting in a Room" technique

Route OUT back to IN externally. Each record/playback cycle reinforces the room's
resonant frequencies. COMB2/COMB3 model multiple room dimensions simultaneously.
CONV uses a real measured IR for early-reflection accuracy.
A peak limiter (instant attack, 200 ms release, threshold 0.8) on the loop signal
prevents blow-up over iterations.

---

## Display Layout (128 × 64 OLED, ~30 fps, 3 pages)

### Page 1/3 — Sampler
```
┌──────────────────────────────┐
│ SAMPLER        RUN      1/3 │
│──────────────────────────────│
│R  [████████░░░░░░░░░░░░░░░] │  R* = frozen at 60 s
│P  [████░░░░░░░░░░░░░░░░░░░] │
│──────────────────────────────│
│ SP1.0x LV0.8 GN1.0          │
│ REC:A+C LOOP:B+D            │
└──────────────────────────────┘
```

### Page 2/3 — Filter (comb mode)
```
┌──────────────────────────────┐
│ FILTER                  2/3 │
│──────────────────────────────│
│ TYPE: COMB2 (2 tap)         │
│ FREQ: 43Hz                  │
│ ALPHA: 0.75                 │
│──────────────────────────────│
│ K1=Type K2=Freq K3=Gain     │
└──────────────────────────────┘
```

### Page 2/3 — Filter (CONV mode)
```
┌──────────────────────────────┐
│ FILTER                  2/3 │
│──────────────────────────────│
│ TYPE: CONV IR               │
│ IR_00     (1/3)             │
│ LEN: 2048smp (43ms)         │
│──────────────────────────────│
│ K1=Type  K2=IR sel          │
└──────────────────────────────┘
```
(Shows "NO IRs ON SD CARD" if no files were found at boot.)

### Page 3/3 — I/O Reference
```
┌──────────────────────────────┐
│ I/O REFERENCE           3/3 │
│──────────────────────────────│
│ IN1=ch1   IN2=ch2           │
│ OUT1=ch1  OUT2=ch2          │
│ GT1  swap channels          │
│──────────────────────────────│
│ K1=Speed    K2=Vol          │
│ K3=Gain     K4=Mix          │
└──────────────────────────────┘
```

Encoder press = run/stop. Encoder rotate = cycle pages.
Knob pickup prevents parameter jumps when switching between sampler and filter pages.

---

## Project Structure

```
daisy_patch_sampler/              ← repo root
├── README.md
├── CLAUDE.md                     ← this file
├── libDaisy/                     ← git submodule
├── DaisySP/                      ← git submodule
└── daisy_patch_sampler/          ← source directory
    ├── daisy_patch_sampler.cpp
    ├── ffascii.c                 ← minimal FatFS ff_convert/ff_wtoupper stubs
    └── Makefile
```

Makefile uses `../libDaisy` and `../DaisySP`.

---

## Build & Flash Commands

The project runs from **external QSPI flash** (`APP_TYPE = BOOT_QSPI`). Flashing
uses the Daisy DFU bootloader over USB — `make program` (SWD/OpenOCD) does not
work for QSPI builds.

```bash
# From daisy_patch_sampler/daisy_patch_sampler/

# Build
make clean && make           # release build
make clean && DEBUG=1 make   # debug build (for attaching debugger)

# Flash (two steps — build must finish before tapping RESET)
# 1. Run the build above, wait for it to complete
# 2. Tap RESET on the Daisy Seed (opens 2-second DFU window)
# 3. Immediately run:
make program-dfu

# One-time bootloader install (only needed if bootloader not already present)
# Hold BOOT + tap RESET to enter ST DFU mode, then:
make program-boot
```

## Debugging

SWD probe (ST-Link) works for debugging even though DFU is used for flashing.
The debugger **attaches** to the running app — it does not reflash.

Workflow:
1. Build with `DEBUG=1 make`, flash with `make program-dfu` (see above)
2. Connect SWD probe
3. Launch "Cortex Debug" in VSCode — it attaches using hardware breakpoints

**Known limitation:** Breakpoints in `main()` do not work. The Daisy bootloader
jumps to the app after a 2-second DFU window, and at that point the debugger is
not yet attached, so breakpoints set before the app starts are missed. Breakpoints
in functions called from the main loop (e.g. `UpdateControls`, `UpdateDisplay`,
`AudioCallback`) work correctly once the debugger has attached to the running app.

---

## CONV mode — resolved (2026-03-20)

CONV mode is now working. `arm_fir_f32` (block-based, DTCM state, SDRAM
coefficients) is used with `-DARM_MATH_LOOPUNROLL` defined in the Makefile.
FLASH usage: 120 KB / 128 KB (91.6%, ~8 KB spare).

### History / why it was hard to fix

The original per-sample naive FIR cost ~786K cycles (1.6× overrun). Moving to
`arm_fir_f32` block processing appeared to fix the root cause, but the freeze
persisted. The missing piece was **`-DARM_MATH_LOOPUNROLL`** in the Makefile.

Without that flag, `arm_fir_f32` (CMSIS-DSP v1.9.0) falls through to a
single-accumulator inner loop:

```c
while (i > 0U) { acc0 += *px++ * *pb++; i--; }
```

On the M7, `VMLA.F32` has a **5-cycle latency**. With every iteration reading
the previous `acc0`, the loop is dependency-limited to one MAC per 5 cycles
instead of one per cycle. Actual ISR cost without `ARM_MATH_LOOPUNROLL`:

```
1024 taps × 48 samples × ~6 cycles × 2 channels ≈ 590K cycles
Budget: 480 MHz / 48 kHz = 480K cycles  → 123% → tail-chains → freeze
```

With `-DARM_MATH_LOOPUNROLL`, the code uses 8 accumulators and computes 8
output samples simultaneously, hiding the latency completely:

```
1024 taps × 48 samples × ~1.25 cycles × 2 channels ≈ 123K cycles → 26%
```

### Key lesson
`ARM_MATH_CM7` (set by libDaisy) does **not** enable loop unrolling in CMSIS-DSP
v1.9.0. `ARM_MATH_LOOPUNROLL` must be set explicitly in the project Makefile.

---

## OPEN TASKS

### 1. Debugger breakpoints don't work in main()

See "Debugging" section above. The bootloader jumps to the app before the SWD
probe attaches, so any breakpoint set at or before `main()` entry is missed.
Breakpoints inside functions called from the main loop work fine.

Potential fix: add an explicit `__asm("bkpt")` or a short `System::Delay()` at
the top of `main()` to give the debugger time to attach, then remove it after.
Alternatively, investigate whether OpenOCD can be configured to halt on QSPI app
entry (would require an external flash loader for the target config).

### 2. IR filter output gain control (CONV mode)

Currently the CONV filter has no gain control — the IR is normalised to peak=1.0
at load time and the output level is fixed. K3 is listed as "unused in CONV mode"
on the display.

Feature: map K3 to a post-convolution gain scalar (e.g. 0.0–2.0, unity at
centre) applied to the FIR output before the peak limiter. This lets the user
compensate for IRs that are perceptually quiet after normalisation (e.g. large
diffuse spaces with low peak but high RMS).
