# daisy_patch_sampler

Ping-pong buffer sampler for the [Daisy Patch](https://electro-smith.com/products/patch) Eurorack module.

## What it does

Two stereo buffer pairs alternate between recording and looping. A gate trigger on GATE_1 freezes the current recording and swaps roles — the just-recorded material becomes the loop while the other buffer starts recording fresh. Designed for the *I Am Sitting in a Room* feedback technique (Alvin Lucier): route the output back into the input and let the room's resonant frequencies build over iterations.

```
RECORD_A+C / PLAY_B+D  ──[GATE_1]──►  RECORD_B+D / PLAY_A+C
         ▲                                        │
         └──────────────[GATE_1]──────────────────┘
```

## Controls

| Control | Function |
|---------|----------|
| GATE_1  | Swap buffers (rising edge) |
| CTRL_1  | Playback speed: 0.25× to 4× exponential, pitch-invariant OLA |
| CTRL_2  | Playback volume |
| CTRL_3  | Input gain (0–2×, unity at centre) |
| CTRL_4  | Dry / wet mix (global, active on all pages) |
| Encoder rotate | Cycle display pages |
| Encoder press  | Run / stop |

## Audio I/O

| Jack    | Signal |
|---------|--------|
| In 1    | Ch1 input  |
| In 2    | Ch2 input  |
| Out 1   | Ch1 output |
| Out 2   | Ch2 output |

## Display pages

**1/3 Sampler** — record/playback bars, speed, level, gain, signal flow
**2/3 Filter** — comb filter type (OFF / COMB / COMB2 / COMB3), frequency, alpha
**3/3 I/O Reference** — quick control map

## Filter bank

Feedback comb filter applied to the loop signal, simulating room resonance accumulation. Three multi-tap modes share a single delay buffer:

- **COMB** — 1 tap, one resonant frequency
- **COMB2** — 2 taps at 100% / 75% of the base delay, simulating two room dimensions
- **COMB3** — 3 taps at 100% / 75% / 55%, simulating three room dimensions

Each additional tap's alpha coefficient decays by 90% from the previous (configurable via `ALPHA_DECAY` in source). A peak limiter (instant attack, 200 ms release) on the loop signal prevents blow-up across Lucier iterations.

## Playback speed (OLA)

Speed control is pitch-invariant using two-grain Overlap-Add (OLA). Two 50 ms Hann-windowed grains read the playback buffer at independent fractional positions, overlapping at 50%. Their windows sum to exactly 1.0 at every sample (COLA property), so output amplitude is constant at all speeds.

> **Note:** At speeds below ~0.3×, the 50 ms grain period becomes audible as a subtle flutter at ~20 Hz. If this is intrusive, increase `GRAIN_SIZE` from `2400u` to `4800u` (100 ms) in the source — this pushes the artefact below 10 Hz and makes it inaudible on most material.

## Memory

| Region | Usage |
|--------|-------|
| SDRAM  | 4 × 60 s buffers ≈ 44 MB (68.7% of 64 MB) |
| SRAM   | Comb delay line ~75 KB + Hann table ~9.6 KB ≈ 27% of 512 KB |
| FLASH  | Firmware ~106 KB (81% of 128 KB) |

## Build

```bash
# Requires arm-none-eabi-gcc toolchain
cd daisy_patch_sampler
make clean && make      # build
make program            # flash via ST-Link
```

Dependencies (git submodules): `libDaisy`, `DaisySP`

## Hardware

[Electrosmith Daisy Patch](https://electro-smith.com/products/patch) — STM32H750 Cortex-M7 @ 400 MHz, 64 MB SDRAM, 128×64 OLED, 4 knobs, encoder, gate inputs.
