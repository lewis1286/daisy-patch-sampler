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
| `g_filter_type`| main loop     | audio ISR          | Comb filter type (OFF/COMB/COMB2/3)  |
| `g_comb_delay` | main loop     | audio ISR          | Comb filter delay in samples         |
| `g_comb_alpha` | main loop     | audio ISR          | Comb filter feedback coefficient     |
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

Feedback comb filter applied to the loop signal (after OLA, before output mix).

| FilterType | Taps | Delays              | Alphas                              |
|------------|------|---------------------|-------------------------------------|
| `OFF`      | 0    | —                   | — (bypassed entirely)               |
| `COMB`     | 1    | D₁                  | α₁                                  |
| `COMB2`    | 2    | D₁, D₁×0.75        | α₁, α₁×0.9                         |
| `COMB3`    | 3    | D₁, D₁×0.75, D₁×0.55| α₁, α₁×0.9, α₁×0.81              |

All taps share a single circular buffer (`CombState`, ~75 KB SRAM).
Room-shape constants at top of filter bank section: `ALPHA_DECAY`, `TAP2_RATIO`, `TAP3_RATIO`.

**K1 on filter page:** 0.00–0.25=OFF, 0.25–0.50=COMB, 0.50–0.75=COMB2, 0.75–1.00=COMB3
**K2:** delay 1–100 ms logarithmic (frequency = Fs/delay)
**K3:** feedback α₁ = 0.0–0.95

Comb state is cleared when the filter transitions from OFF to active.

### "I Am Sitting in a Room" technique

Route OUT back to IN externally. Each record/playback cycle reinforces the room's
resonant frequencies (Fs/D, 2Fs/D, …). COMB2/COMB3 model multiple room dimensions
simultaneously. A peak limiter (instant attack, 200 ms release, threshold 0.8)
on the loop signal prevents blow-up over iterations.

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

### Page 2/3 — Filter
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
    └── Makefile
```

Makefile uses `../libDaisy` and `../DaisySP`.

---

## Build Commands

```bash
# From daisy_patch_sampler/daisy_patch_sampler/
make clean && make      # full build
make program            # flash via debugger
```
