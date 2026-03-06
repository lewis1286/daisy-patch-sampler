# daisy_patch_sampler — Project Reference

## Concept

Ping-pong buffer sampler for Daisy Patch (Eurorack). Two SDRAM buffers alternate
roles: one records the live audio input while the other loops its previously
captured content. A Gate/Trigger on GATE_1 swaps the roles.

```
  boot
   │
   ▼
RECORD_A / PLAY_B ──── GATE_1 rising edge ────► RECORD_B / PLAY_A
       ▲                                               │
       └────────────── GATE_1 rising edge ─────────────┘
```

---

## Hardware I/O

| Signal       | Assignment                                    |
|--------------|-----------------------------------------------|
| Audio In 1   | Ch1 source signal                             |
| Audio In 2   | Ch2 source signal                             |
| Audio Out 1+2| Ch1 output (same signal L + R)                |
| Audio Out 3+4| Ch2 output (same signal L + R)                |
| GATE_1       | Buffer swap trigger (rising edge, both ch)    |
| CTRL_1       | Max record length (0.1 s → 60 s, logarithmic) |
| CTRL_2     | Playback volume (0.0 → 1.0, linear)           |
| CTRL_3     | Input gain (0.0 → 2.0, unity at centre)       |
| CTRL_4     | Dry / wet mix (0.0 = dry, 1.0 = wet)          |

---

## Buffers

- **Location:** SDRAM (`DSY_SDRAM_BSS`) — 64 MB available
- **Format:** `float`, mono, 48 kHz
- **Size:** 60 s × 48 000 samples/s = 2 880 000 samples per buffer
- **Memory:** 4 × 2 880 000 × 4 bytes = **~44 MB total** (~20 MB headroom)

```cpp
constexpr size_t MAX_SAMPLES = 48000u * 60u;   // 2,880,000
float DSY_SDRAM_BSS buf_a[MAX_SAMPLES];  // ch1 ping
float DSY_SDRAM_BSS buf_b[MAX_SAMPLES];  // ch1 pong
float DSY_SDRAM_BSS buf_c[MAX_SAMPLES];  // ch2 ping
float DSY_SDRAM_BSS buf_d[MAX_SAMPLES];  // ch2 pong
```

---

## State Machine

### Variables (volatile — shared between audio ISR and main loop)

| Variable       | Written by    | Read by            | Meaning                          |
|----------------|---------------|--------------------|----------------------------------|
| `g_state`      | audio ISR     | main loop (display)| Current state enum               |
| `g_rec_pos`    | audio ISR     | main loop (display)| Write head in active rec buffer  |
| `g_play_pos`   | audio ISR     | main loop (display)| Read head in active play buffer  |
| `g_play_len_a` | audio ISR     | audio ISR          | Frozen length of buf_a           |
| `g_play_len_b` | audio ISR     | audio ISR          | Frozen length of buf_b           |
| `g_max_rec_len`| main loop     | audio ISR          | Current max record samples       |
| `g_play_level` | main loop     | audio ISR          | Playback volume                  |
| `g_input_gain` | main loop     | audio ISR          | Input gain                       |
| `g_wet_mix`    | main loop     | audio ISR          | Dry/wet balance                  |

Thread safety: 32-bit reads/writes are atomic on Cortex-M7. `volatile` alone is
sufficient — no mutexes needed.

### Swap logic (called in audio callback on gate trigger)

```
if RECORD_A_PLAY_B:
    play_len_a = rec_pos      // freeze what was just recorded
    state      = RECORD_B_PLAY_A
else:
    play_len_b = rec_pos
    state      = RECORD_A_PLAY_B
rec_pos  = 0
play_pos = 0                  // playback always restarts from 0
```

### Recording behaviour

- Writes `in[0][i] * g_input_gain` into the active buffer every sample.
- **Freeze** mode: when `rec_pos >= g_max_rec_len`, recording stops and waits
  for the next gate trigger. Does **not** overdub.

### Playback behaviour

- Reads the inactive buffer at `play_pos`, increments, wraps to 0 at
  `play_len_X`.
- Before the first swap, the playback buffer has zero length → silent output.

---

## CTRL_1 Logarithmic Curve

Knob raw value `k` (0.0 – 1.0) maps to record length via:

```
len_sec = 0.1 * (600.0)^k         // 0.1 s at k=0, 60 s at k=1
```

Implemented as:

```cpp
float len_sec = 0.1f * powf(600.0f, k1);   // 0.1 s – 60.0 s
g_max_rec_len = (size_t)(len_sec * 48000.0f);
```

`powf(600, 0) = 1` → 0.1 s × 1 = 0.1 s
`powf(600, 1) = 600` → 0.1 s × 600 = 60 s ✓

---

## Audio Callback Structure

```
ProcessAnalogControls()
ProcessDigitalControls()

if GATE_1.Trig() → SwapBuffers()

snapshot volatile params (once per block)

for each sample:
    in_sig   = in[0][i] * gain
    if rec_pos < max_rec: rec_buf[rec_pos++] = in_sig   // freeze when full
    loop_out = (play_len > 0) ? play_buf[play_pos++] : 0.0f
    if play_pos >= play_len: play_pos = 0
    out[0][i] = out[1][i] = in_sig * dry + loop_out * level * wet
    out[2][i] = out[3][i] = 0.0f

write back rec_pos, play_pos
```

---

## Display Layout (128 × 64 OLED, ~30 fps)

```
┌──────────────────────────────┐  y=0
│ SAMPLER              A->B   │  Font_7x10
│──────────────────────────────│  y=12 separator
│R* [████████░░░░░░░░░░░░░░░] │  y=14  rec bar (R* = frozen)
│P  [████░░░░░░░░░░░░░░░░░░░] │  y=27  play bar
│──────────────────────────────│  y=38 separator
│ LEN 2.5s  LVL 0.8           │  y=42  Font_6x8
│ IN->A  LOOP:B               │  y=54  Font_6x8
└──────────────────────────────┘  y=64
```

- `R*` = recording frozen (full); `R ` = recording active
- Progress bars use `DrawLine` with all coords clamped to [0,127] × [0,63]
  to prevent the ARM `uint_fast8_t` overflow freeze bug

---

## Project Structure

```
daisy_patch_sampler/              ← repo root
├── CLAUDE.md                     ← this file
├── libDaisy/                     ← git submodule
├── DaisySP/                      ← git submodule
└── daisy_patch_sampler/          ← source directory
    ├── daisy_patch_sampler.cpp
    └── Makefile
```

Makefile uses `../libDaisy` and `../DaisySP` (mirrors mumurator layout).

---

## Implementation Roadmap

- [x] Write CLAUDE.md with design spec
- [x] Create source directory `daisy_patch_sampler/daisy_patch_sampler/`
- [x] Write `Makefile`
- [x] Write `daisy_patch_sampler.cpp`
  - [x] SDRAM buffer declarations
  - [x] State machine variables
  - [x] `SwapBuffers()` helper
  - [x] `AudioCallback()` — record + playback + gate detection
  - [x] `UpdateControls()` — knob reads + log curve for CTRL_1
  - [x] `DrawProgressBar()` helper with clamped DrawLine calls
  - [x] `UpdateDisplay()` — OLED layout
  - [x] `main()` — init, start audio, main loop
- [x] Set up git submodules (libDaisy + DaisySP)
- [x] Build libDaisy (`cd libDaisy && make`)
- [x] First build: `make clean && make`
- [ ] Flash and test: `make program`

---

## Build Commands

```bash
# From daisy_patch_sampler/daisy_patch_sampler/
make clean && make          # full build
make clean && make ui-only  # no audio (UI testing)
make program                # flash via debugger
```
