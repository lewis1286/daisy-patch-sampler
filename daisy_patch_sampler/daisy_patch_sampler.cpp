#include "daisysp.h"
#include "daisy_patch.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

// ─── Hardware ────────────────────────────────────────────────────────────────
DaisyPatch patch;

// ─── Buffers — 60 s each in SDRAM (~44 MB total for 4 buffers, ~20 MB headroom)
static constexpr size_t MAX_SAMPLES = 48000u * 60u;   // 2,880,000 samples

float DSY_SDRAM_BSS buf_a[MAX_SAMPLES];  // ch1 ping
float DSY_SDRAM_BSS buf_b[MAX_SAMPLES];  // ch1 pong
float DSY_SDRAM_BSS buf_c[MAX_SAMPLES];  // ch2 ping
float DSY_SDRAM_BSS buf_d[MAX_SAMPLES];  // ch2 pong

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

// ─── Run state (written by main loop, read by audio ISR) ─────────────────────
volatile bool    g_running     = true;  // encoder press toggles pause; starts running
volatile int32_t g_enc_delta   = 0;     // encoder rotation accumulator; written by ISR, read by main loop
volatile bool    g_enc_pressed = false; // encoder press flag; written by ISR, read + cleared by main loop

// ─── Crossfade state (audio ISR only — no volatile, never touched by main loop)
// On buffer swap, we blend out the old loop while blending in the new one over
// XFADE_LEN samples to eliminate the click caused by the discontinuous jump.
static constexpr size_t XFADE_LEN = 240u;  // 5 ms at 48 kHz

static const float* xfade_old1   = nullptr;  // old ch1 playback buffer
static const float* xfade_old2   = nullptr;  // old ch2 playback buffer
static size_t       xfade_pos    = 0u;        // read position in old buffers
static size_t       xfade_oldlen = 0u;        // length of old loop
static size_t       xfade_rem    = 0u;        // samples remaining (0 = idle)

// ─── Display timing + page ───────────────────────────────────────────────────
static uint32_t           last_display_ms = 0u;
static constexpr uint32_t DISPLAY_MS      = 33u;   // ~30 fps
static constexpr uint8_t  NUM_PAGES       = 2u;
static uint8_t            g_page          = 0u;    // encoder rotation cycles pages

// ─── Buffer swap ─────────────────────────────────────────────────────────────
// Called inside the audio callback on GATE_1 rising edge.
static inline void SwapBuffers()
{
    const bool is_rec_a = (g_state == State::RECORD_A_PLAY_B);

    // Capture the current playback state before switching so the crossfade can
    // keep reading the old loop while the new one fades in.
    xfade_old1   = is_rec_a ? buf_b : buf_a;
    xfade_old2   = is_rec_a ? buf_d : buf_c;
    xfade_pos    = g_play_pos;
    xfade_oldlen = is_rec_a ? g_play_len_b : g_play_len_a;
    xfade_rem    = (xfade_oldlen > 0u) ? XFADE_LEN : 0u;

    if (is_rec_a)
    {
        g_play_len_a = g_rec_pos;
        g_state      = State::RECORD_B_PLAY_A;
    }
    else
    {
        g_play_len_b = g_rec_pos;
        g_state      = State::RECORD_A_PLAY_B;
    }
    g_rec_pos  = 0;
    g_play_pos = 0;
}

// ─── Audio Callback ──────────────────────────────────────────────────────────
#ifndef PROJECT_UI_ONLY
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    patch.ProcessAnalogControls();
    patch.ProcessDigitalControls();
    g_enc_delta += patch.encoder.Increment();  // capture before next Debounce tick resets inc_
    if (patch.encoder.RisingEdge())            // RisingEdge() is true for a full 1ms tick;
        g_enc_pressed = true;                  // capture as a flag so main loop sees it exactly once

    // When stopped: silence all outputs — buffers are preserved at their current positions
    if (!g_running)
    {
        for (size_t i = 0; i < size; i++)
            out[0][i] = out[1][i] = out[2][i] = out[3][i] = 0.0f;
        return;
    }

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

    float* const rec_buf   = is_rec_a ? buf_a : buf_b;
    float* const play_buf  = is_rec_a ? buf_b : buf_a;
    float* const rec_buf2  = is_rec_a ? buf_c : buf_d;
    float* const play_buf2 = is_rec_a ? buf_d : buf_c;

    size_t r = g_rec_pos;
    size_t p = g_play_pos;

    for (size_t i = 0; i < size; i++)
    {
        const float in1 = in[0][i] * gain;
        const float in2 = in[1][i] * gain;

        // Record both channels — freeze (stop writing) when max length reached
        if (r < max_rec)
        {
            rec_buf[r]  = in1;
            rec_buf2[r] = in2;
            r++;
        }

        // Playback both channels — loop at frozen length; silence before first swap
        float loop1 = 0.0f, loop2 = 0.0f;
        if (play_len > 0u)
        {
            loop1 = play_buf[p];
            loop2 = play_buf2[p];
            if (++p >= play_len)
                p = 0;
        }

        // Crossfade: blend old loop out (fade_out) and new loop in (fade_in)
        // over XFADE_LEN samples to eliminate the click at buffer swap.
        if (xfade_rem > 0u)
        {
            const float fade_out = (float)xfade_rem / (float)XFADE_LEN;  // 1.0 → 0.0
            const float fade_in  = 1.0f - fade_out;                       // 0.0 → 1.0
            loop1 = xfade_old1[xfade_pos] * fade_out + loop1 * fade_in;
            loop2 = xfade_old2[xfade_pos] * fade_out + loop2 * fade_in;
            if (++xfade_pos >= xfade_oldlen) xfade_pos = 0u;
            --xfade_rem;
        }

        out[0][i] = out[1][i] = in1 * dry + loop1 * level * wet;
        out[2][i] = out[3][i] = in2 * dry + loop2 * level * wet;
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

// ─── Page 0: main sampler view ───────────────────────────────────────────────
static void DrawMainPage()
{
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
    const bool   running    = g_running;

    auto& disp = patch.display;

    // ── Title + run state + page indicator ───────────────────────────────────
    disp.SetCursor(0, 0);
    disp.WriteString("SAMPLER", Font_7x10, true);
    disp.SetCursor(60, 2);
    disp.WriteString(running ? "RUN" : "STP", Font_6x8, true);
    disp.SetCursor(109, 2);
    disp.WriteString("1/2", Font_6x8, true);
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
    // Float printf not available with newlib-nano; format as integers.
    {
        char tmp[32];
        // Convert to integer tenths of a second first, then split.
        // Avoids floating-point boundary flicker: e.g. 1.9999 and 2.0001 both
        // round to the same integer (20 tenths) so the display digit stays stable.
        const int   ls10 = (int)roundf((float)max_rec / 48000.0f * 10.0f);
        const int   lw   = ls10 / 10;
        const int   lf   = ls10 % 10;
        const int   lvl  = (int)(play_level * 10.0f);
        snprintf(tmp, sizeof(tmp), "LEN %d.%ds  LVL 0.%d", lw, lf, lvl);
        disp.SetCursor(0, 42);
        disp.WriteString(tmp, Font_6x8, true);
    }

    // ── Signal flow ──────────────────────────────────────────────────────────
    disp.SetCursor(0, 54);
    disp.WriteString(is_rec_a ? "REC:A+C LOOP:B+D" : "REC:B+D LOOP:A+C", Font_6x8, true);
}

// ─── Page 1: I/O reference ───────────────────────────────────────────────────
static void DrawIOPage()
{
    auto& disp = patch.display;

    disp.SetCursor(0, 0);
    disp.WriteString("I/O REFERENCE", Font_7x10, true);
    disp.SetCursor(109, 2);
    disp.WriteString("2/2", Font_6x8, true);
    disp.DrawLine(0, 12, 127, 12, true);

    disp.SetCursor(0, 14);
    disp.WriteString("IN1 ch1     IN2 ch2", Font_6x8, true);
    disp.SetCursor(0, 23);
    disp.WriteString("OUT1/2 ch1 OUT3/4 ch2", Font_6x8, true);
    disp.SetCursor(0, 32);
    disp.WriteString("GT1  swap channels", Font_6x8, true);

    disp.DrawLine(0, 41, 127, 41, true);

    disp.SetCursor(0, 43);
    disp.WriteString("K1=RecLen   K2=Vol", Font_6x8, true);
    disp.SetCursor(0, 52);
    disp.WriteString("K3=Gain     K4=Mix", Font_6x8, true);
}

// ─── Display update (~30 fps, main loop) ─────────────────────────────────────
static void UpdateDisplay()
{
    auto& disp = patch.display;
    disp.Fill(false);

    if (g_page == 0u)
        DrawMainPage();
    else
        DrawIOPage();

    disp.Update();
}

// ─── Controls update (main loop) ─────────────────────────────────────────────
// ProcessAnalogControls/ProcessDigitalControls are called in the audio callback,
// so knob values and encoder state here are already refreshed — just read them.
static void UpdateControls()
{
    // Encoder press → pause / resume (flag set in ISR; cleared here to fire exactly once)
    if (g_enc_pressed)
    {
        g_enc_pressed = false;
        g_running = !g_running;
    }

    // Encoder rotation → cycle display pages.
    // Read from the ISR accumulator (not Increment() directly). Debounce() sets
    // inc_ for an entire 1ms tick; calling Increment() in the main loop hundreds
    // of times per tick causes repeated page toggles that cancel out. The ISR
    // captures it once per tick into g_enc_delta so we get exactly ±1 per detent.
    const int32_t enc = g_enc_delta;
    g_enc_delta = 0;
    if (enc > 0)
        g_page = (g_page + 1u) % NUM_PAGES;
    else if (enc < 0)
        g_page = (g_page + NUM_PAGES - 1u) % NUM_PAGES;

    const float k1 = patch.GetKnobValue(DaisyPatch::CTRL_1);
    const float k2 = patch.GetKnobValue(DaisyPatch::CTRL_2);
    const float k3 = patch.GetKnobValue(DaisyPatch::CTRL_3);
    const float k4 = patch.GetKnobValue(DaisyPatch::CTRL_4);

    // CTRL_1: logarithmic curve  0.1 s (k=0) → 60 s (k=1)
    // len = 0.1 * 600^k  →  at k=0: 0.1*1 = 0.1 s,  at k=1: 0.1*600 = 60 s
    const float len_sec   = 0.1f * powf(600.0f, k1);
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
