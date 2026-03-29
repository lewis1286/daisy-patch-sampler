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
| CTRL_1  | Playback volume (sampler pages) / Filter type (filter page) |
| CTRL_2  | Input gain 0–2× (sampler pages) / Comb freq or IR select (filter page) |
| CTRL_3  | Unused (sampler pages) / Comb feedback alpha (filter page, comb modes) |
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

**1/3 Sampler** — record/playback bars, level, gain, signal flow

**2/3 Filter** — filter engine selection and parameter readout (see Filter Bank below)

**3/3 I/O Reference** — quick control map

## Filter bank

Applied to the loop signal after playback, before the output mix. Five modes selected via K1 on the filter page:

### Comb modes (K1 = 0.0–0.8)

Feedback comb filter simulating room resonance accumulation. Three multi-tap modes share a single delay buffer:

| Mode | K1 zone | Description |
|------|---------|-------------|
| OFF   | 0.00–0.20 | Bypass |
| COMB  | 0.20–0.40 | 1 tap — one resonant frequency |
| COMB2 | 0.40–0.60 | 2 taps at 100% / 75% of base delay |
| COMB3 | 0.60–0.80 | 3 taps at 100% / 75% / 55% — three room dimensions |

- **K2** — comb delay / fundamental frequency (1 ms–100 ms, logarithmic)
- **K3** — feedback coefficient α (0.0–0.95)

Each tap's alpha decays by 90% from the previous (set by `ALPHA_DECAY` in source). A peak limiter (instant attack, 200 ms release) on the loop signal prevents blow-up across Lucier iterations.

### Convolution mode (K1 = 0.8–1.0)

Time-domain FIR convolution against a measured impulse response loaded from the SD card at startup. Uses the actual acoustics of a real space — early reflections, room geometry, spectral colouration — rather than a synthetic comb approximation.

- **K2** — select IR slot (cycles through all loaded IRs)
- IR files are normalised to peak = 1.0 at load time

**Preparing IR files:**

1. Record or download an impulse response WAV file
2. Convert to 48 kHz mono 16-bit PCM, trimmed to ≤ 21 ms (≤ 1024 samples):
   ```bash
   ffmpeg -i input.wav -ac 1 -ar 48000 -ss 0 -t 0.02133 -acodec pcm_s16le IR_00.wav
   ```
3. Copy `IR_00.wav` … `IR_07.wav` to the root of the SD card (up to 8 slots)
4. Power cycle — IRs load at boot before audio starts

Longer files are accepted but truncated to 1024 samples at load time.

**SD card requirements:** FAT32 formatted microSD or microSDHC (≤ 32 GB). SDXC cards (64 GB+) ship as exFAT which is not supported — reformat as FAT32 first, or just use a smaller card. SDHC cards (4–32 GB) work out of the box.

> **CPU budget:** 1024-sample IR (~21 ms) uses ~26% of the audio block budget using CMSIS-DSP `arm_fir_f32` with loop unrolling (`ARM_MATH_LOOPUNROLL`).

## Memory

| Region     | Usage |
|------------|-------|
| QSPI flash | Firmware ~120 KB (1.5% of 7.9 MB) |
| SDRAM      | 4 × 60 s sample buffers ≈ 44 MB + up to 8 IR slots × 1024 samples ≈ 32 KB (68.7% of 64 MB) |
| DTCMRAM    | FIR convolution state buffers ~8.4 KB (6.6% of 128 KB) |
| SRAM       | Comb delay line ~75 KB + globals ≈ 26% of 512 KB |

## Build & flash

The firmware runs from external QSPI flash via the Daisy bootloader. Flashing uses USB DFU — not the SWD debug probe.

**Requirements:** `arm-none-eabi-gcc` toolchain, `dfu-util`

```bash
cd daisy_patch_sampler/daisy_patch_sampler

# Build
make clean && make

# Flash — build first, then tap RESET on the Daisy Seed,
# then run this within the 2-second DFU window:
make program-dfu
```

**First-time only — install the Daisy bootloader:**

Hold `BOOT` + tap `RESET` to enter ST DFU mode, then:
```bash
make program-boot
```
This only needs to be done once per device.

## Debugging

The SWD debug probe (ST-Link / J-Link) works for debugging even though flashing uses DFU. The debugger attaches to the running firmware — it does not reflash it.

```bash
# Build with debug symbols
make clean && DEBUG=1 make

# Flash as normal (tap RESET, then):
make program-dfu
```

Then connect the SWD probe and launch **Cortex Debug** in VSCode. It will attach to the running app using hardware breakpoints.

> **Note:** Breakpoints inside `main()` itself will not be hit — the bootloader jumps to the app before the debugger attaches. Breakpoints in functions called from the main loop (`UpdateControls`, `UpdateDisplay`, `AudioCallback`, etc.) work normally once attached.

Dependencies (git submodules): `libDaisy`, `DaisySP`

## Hardware

[Electrosmith Daisy Patch](https://electro-smith.com/products/patch) — STM32H750 Cortex-M7 @ 480 MHz, 64 MB SDRAM, 128×64 OLED, 4 knobs, encoder, gate inputs, micro SD card slot.
