#include "daisysp.h"
#include "daisy_patch.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

// ─── Hardware ────────────────────────────────────────────────────────────────
DaisyPatch patch;

// ─── Buffers — 120 s each in SDRAM (~46 MB total, ~18 MB headroom) ───────────
static constexpr size_t MAX_SAMPLES = 48000u * 120u;  // 5,760,000 samples

float DSY_SDRAM_BSS buf_a[MAX_SAMPLES];
float DSY_SDRAM_BSS buf_b[MAX_SAMPLES];

// ─── State ───────────────────────────────────────────────────────────────────
// All variables shared between the audio ISR and main loop are volatile.
// 32-bit reads/writes are atomic on Cortex-M7, so volatile alone is sufficient.

enum class State : uint8_t { RECORD_A_PLAY_B, RECORD_B_PLAY_A };

volatile State  g_state      = State::RECORD_A_PLAY_B;
volatile size_t g_rec_pos    = 0;   // write head in active record buffer
volatile size_t g_play_pos   = 0;   // read head in active playback buffer
volatile size_t g_play_len_a = 0;   // frozen playback length of buf_a
volatile size_t g_play_len_b = 0;   // frozen playback length of buf_b

// ─── Params (written by main loop, read by audio ISR) ────────────────────────
volatile size_t g_max_rec_len = MAX_SAMPLES;
volatile float  g_play_level  = 1.0f;
volatile float  g_input_gain  = 1.0f;
volatile float  g_wet_mix     = 1.0f;

// ─── Display timing ──────────────────────────────────────────────────────────
static uint32_t last_display_ms          = 0u;
static constexpr uint32_t DISPLAY_MS     = 33u;  // ~30 fps

// ─── Buffer swap ─────────────────────────────────────────────────────────────
// Called inside the audio callback on GATE_1 rising edge.
static inline void SwapBuffers()
{
    if (g_state == State::RECORD_A_PLAY_B)
    {
        g_play_len_a = g_rec_pos;          // freeze recorded length of A
        g_state      = State::RECORD_B_PLAY_A;
    }
    else
    {
        g_play_len_b = g_rec_pos;          // freeze recorded length of B
        g_state      = State::RECORD_A_PLAY_B;
    }
    g_rec_pos  = 0;
    g_play_pos = 0;                        // playback always restarts from 0
}

// ─── Audio Callback ──────────────────────────────────────────────────────────
#ifndef PROJECT_UI_ONLY
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();

    if (patch.gate_input[0].Trig())
        SwapBuffers();

    // Snapshot volatile params once per block to avoid repeated volatile reads
    const bool   is_rec_a = (g_state == State::RECORD_A_PLAY_B);
    const size_t max_rec  = g_max_rec_len;
    const float  gain     = g_input_gain;
    const float  level    = g_play_level;
    const float  wet      = g_wet_mix;
    const float  dry      = 1.0f - wet;
    const size_t play_len = is_rec_a ? g_play_len_b : g_play_len_a;

    float* const rec_buf  = is_rec_a ? buf_a : buf_b;
    float* const play_buf = is_rec_a ? buf_b : buf_a;

    size_t r = g_rec_pos;
    size_t p = g_play_pos;

    for (size_t i = 0; i < size; i++)
    {
        const float in_sig = in[0][i] * gain;

        // Record — freeze (stop writing) when max length reached
        if (r < max_rec)
            rec_buf[r++] = in_sig;

        // Playback — loop at frozen length; silence before first swap
        float loop_out = 0.0f;
        if (play_len > 0u)
        {
            loop_out = play_buf[p];
            if (++p >= play_len)
                p = 0;
        }

        const float out_sig = in_sig * dry + loop_out * level * wet;
        out[0][i] = out_sig;
        out[1][i] = out_sig;
        out[2][i] = 0.0f;
        out[3][i] = 0.0f;
    }

    g_rec_pos  = r;
    g_play_pos = p;
}
#endif  // PROJECT_UI_ONLY

// ─── Display helpers ─────────────────────────────────────────────────────────
// Clamp helpers guard against the ARM uint_fast8_t overflow freeze bug in
// DrawLine (negative int wraps to ~4 billion → infinite Bresenham loop).
static inline int ClampX(int v) { return v < 0 ? 0 : v > 127 ? 127 : v; }
static inline int ClampY(int v) { return v < 0 ? 0 : v >  63 ?  63 : v; }

static void DrawProgressBar(int x, int y, int w, int h, float fill)
{
    auto& d  = patch.display;
    const int x1 = x + w - 1;
    const int y1 = y + h - 1;

    // Outline
    d.DrawLine(ClampX(x),  ClampY(y),  ClampX(x1), ClampY(y),  true);
    d.DrawLine(ClampX(x),  ClampY(y1), ClampX(x1), ClampY(y1), true);
    d.DrawLine(ClampX(x),  ClampY(y),  ClampX(x),  ClampY(y1), true);
    d.DrawLine(ClampX(x1), ClampY(y),  ClampX(x1), ClampY(y1), true);

    // Fill
    const float clamped = fill < 0.0f ? 0.0f : fill > 1.0f ? 1.0f : fill;
    const int   px      = (int)((w - 2) * clamped);
    if (px > 0)
    {
        for (int row = y + 1; row < y1; row++)
            d.DrawLine(ClampX(x + 1), ClampY(row), ClampX(x + px), ClampY(row), true);
    }
}

// ─── Display update (~30 fps, main loop) ─────────────────────────────────────
static void UpdateDisplay()
{
    // Snapshot volatile state for this frame
    const State  cur_state  = g_state;
    const size_t rec_pos    = g_rec_pos;
    const size_t play_pos   = g_play_pos;
    const size_t max_rec    = g_max_rec_len;
    const size_t play_len_a = g_play_len_a;
    const size_t play_len_b = g_play_len_b;
    const float  play_level = g_play_level;

    const bool   is_rec_a   = (cur_state == State::RECORD_A_PLAY_B);
    const size_t play_len   = is_rec_a ? play_len_b : play_len_a;
    const bool   rec_frozen = (max_rec > 0u && rec_pos >= max_rec);

    auto& disp = patch.display;
    disp.Fill(false);

    // ── Title + state ────────────────────────────────────────────────────────
    disp.SetCursor(0, 0);
    disp.WriteString("SAMPLER", Font_7x10, true);
    disp.SetCursor(88, 0);
    disp.WriteString(is_rec_a ? "A->B" : "B->A", Font_7x10, true);
    disp.DrawLine(0, 12, 127, 12, true);

    // ── Record progress bar ──────────────────────────────────────────────────
    disp.SetCursor(0, 16);
    disp.WriteString(rec_frozen ? "R*" : "R ", Font_6x8, true);
    {
        const float fill = (max_rec > 0u)
            ? (float)rec_pos / (float)max_rec
            : 0.0f;
        DrawProgressBar(14, 15, 113, 9, fill);
    }

    // ── Playback position bar ────────────────────────────────────────────────
    disp.SetCursor(0, 28);
    disp.WriteString("P ", Font_6x8, true);
    {
        const float fill = (play_len > 0u)
            ? (float)(play_pos % play_len) / (float)play_len
            : 0.0f;
        DrawProgressBar(14, 27, 113, 9, fill);
    }

    disp.DrawLine(0, 39, 127, 39, true);

    // ── Parameter readout ────────────────────────────────────────────────────
    // Float printf not available with newlib-nano; format manually as integers.
    {
        char tmp[32];
        const float ls  = (float)max_rec / 48000.0f;
        const int   lw  = (int)ls;
        const int   lf  = (int)((ls - (float)lw) * 10.0f);
        const int   lvl = (int)(play_level * 10.0f);
        snprintf(tmp, sizeof(tmp), "LEN %d.%ds  LVL 0.%d", lw, lf, lvl);
        disp.SetCursor(0, 42);
        disp.WriteString(tmp, Font_6x8, true);
    }

    // ── Signal flow ──────────────────────────────────────────────────────────
    disp.SetCursor(0, 54);
    disp.WriteString(is_rec_a ? "IN->A  LOOP:B" : "IN->B  LOOP:A", Font_6x8, true);

    disp.Update();
}

// ─── Controls update (main loop) ─────────────────────────────────────────────
// ProcessAnalogControls/ProcessDigitalControls are called in the audio callback,
// so knob values here are already refreshed — just read the cached result.
static void UpdateControls()
{
    const float k1 = patch.GetKnobValue(DaisyPatch::CTRL_1);
    const float k2 = patch.GetKnobValue(DaisyPatch::CTRL_2);
    const float k3 = patch.GetKnobValue(DaisyPatch::CTRL_3);
    const float k4 = patch.GetKnobValue(DaisyPatch::CTRL_4);

    // CTRL_1: logarithmic curve  0.1 s (k=0) → 120 s (k=1)
    // len = 0.1 * 1200^k  →  at k=0: 0.1*1 = 0.1 s,  at k=1: 0.1*1200 = 120 s
    const float len_sec   = 0.1f * powf(1200.0f, k1);
    g_max_rec_len = (size_t)(len_sec * 48000.0f);

    g_play_level = k2;           // CTRL_2: playback level 0.0–1.0
    g_input_gain = k3 * 2.0f;   // CTRL_3: input gain 0.0–2.0 (unity at centre)
    g_wet_mix    = k4;           // CTRL_4: dry/wet 0.0–1.0
}

// ─── Main ────────────────────────────────────────────────────────────────────
int main()
{
    patch.Init();

#ifndef PROJECT_UI_ONLY
    patch.StartAudio(AudioCallback);
#endif

    patch.StartAdc();

    while (1)
    {
        UpdateControls();

        const uint32_t now = System::GetNow();
        if (now - last_display_ms >= DISPLAY_MS)
        {
            UpdateDisplay();
            last_display_ms = now;
        }
    }
}
