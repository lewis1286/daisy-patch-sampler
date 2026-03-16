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

// ─── Filter bank ─────────────────────────────────────────────────────────────
// FilterType::OFF skips the filter block entirely (zero CPU cost).
// FilterType::COMB is a feedback comb filter:  y[n] = x[n] + α·y[n-D]
// which reinforces frequencies at Fs/D, 2Fs/D, … simulating room resonance.
enum class FilterType : uint8_t { OFF, COMB };

// CombState lives in SRAM (ISR-only, never touched by main loop).
// MAX = 9600 samples ≈ 200 ms at 48 kHz.
struct CombState
{
    static constexpr size_t MAX = 9600u;
    float  buf1[MAX];
    float  buf2[MAX];
    size_t write = 0u;
    void reset()
    {
        write = 0u;
        std::fill(buf1, buf1 + MAX, 0.0f);
        std::fill(buf2, buf2 + MAX, 0.0f);
    }
};
static CombState comb;

volatile FilterType g_filter_type = FilterType::OFF;
volatile size_t     g_comb_delay  = 480u;   // 10 ms → fundamental at 100 Hz
volatile float      g_comb_alpha  = 0.0f;   // feedback coefficient 0.0–0.95

// ─── Run state (written by main loop, read by audio ISR) ─────────────────────
volatile bool    g_running     = true;  // encoder press toggles pause; starts running
volatile int32_t g_enc_delta   = 0;     // encoder rotation accumulator; written by ISR, read by main loop
volatile bool    g_enc_pressed = false; // encoder press flag; written by ISR, read + cleared by main loop

// ─── Crossfade state (audio ISR only — no volatile, never touched by main loop)
// On buffer swap, we blend out the old loop while blending in the new one over
// XFADE_LEN samples to eliminate the click caused by the discontinuous jump.
static constexpr size_t XFADE_LEN = 240u;  // 5 ms at 48 kHz

// ─── Peak limiter state (audio ISR only) ─────────────────────────────────────
// Instant attack, 200 ms release. Applied to the loop signal (both channels
// share one envelope so stereo balance is preserved). Prevents the Lucier
// feedback loop from blowing up across iterations.
static constexpr float LIM_THRESHOLD = 0.8f;
static constexpr float LIM_RELEASE   = 0.999896f;  // exp(-1 / (0.2 * 48000))
static float           lim_env       = 0.0f;

static const float* xfade_old1   = nullptr;  // old ch1 playback buffer
static const float* xfade_old2   = nullptr;  // old ch2 playback buffer
static size_t       xfade_pos    = 0u;        // read position in old buffers
static size_t       xfade_oldlen = 0u;        // length of old loop
static size_t       xfade_rem    = 0u;        // samples remaining (0 = idle)

// ─── Display timing + page ───────────────────────────────────────────────────
static uint32_t           last_display_ms = 0u;
static constexpr uint32_t DISPLAY_MS      = 33u;   // ~30 fps
static constexpr uint8_t  NUM_PAGES       = 3u;
static uint8_t            g_page          = 0u;    // encoder rotation cycles pages

// ─── Filter dispatch ─────────────────────────────────────────────────────────
static inline void ApplyFilter(float& s1, float& s2,
                                FilterType type, size_t delay, float alpha)
{
    switch (type)
    {
        case FilterType::COMB:
        {
            const size_t rd = (comb.write + CombState::MAX - delay) % CombState::MAX;
            const float  y1 = s1 + alpha * comb.buf1[rd];
            const float  y2 = s2 + alpha * comb.buf2[rd];
            comb.buf1[comb.write] = y1;
            comb.buf2[comb.write] = y2;
            comb.write = (comb.write + 1u < CombState::MAX) ? comb.write + 1u : 0u;
            s1 = y1;
            s2 = y2;
            break;
        }
        case FilterType::OFF:
        default:
            break;
    }
}

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

    // Snapshot filter params; reset comb delay line when filter is newly activated
    const FilterType filter_type  = g_filter_type;
    const size_t     comb_delay   = g_comb_delay;
    const float      comb_alpha   = g_comb_alpha;
    static FilterType prev_filter = FilterType::OFF;
    if (filter_type != FilterType::OFF && prev_filter == FilterType::OFF)
        comb.reset();
    prev_filter = filter_type;

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

        ApplyFilter(loop1, loop2, filter_type, comb_delay, comb_alpha);

        // Peak limiter: instant attack, 200 ms release.
        // Both channels share the same envelope so stereo image is preserved.
        const float lim_peak = fabsf(loop1) > fabsf(loop2) ? fabsf(loop1) : fabsf(loop2);
        lim_env = lim_peak > lim_env ? lim_peak : lim_env * LIM_RELEASE;
        if (lim_env > LIM_THRESHOLD)
        {
            const float lim_gain = LIM_THRESHOLD / lim_env;
            loop1 *= lim_gain;
            loop2 *= lim_gain;
        }

        out[0][i] = in1 * dry + loop1 * level * wet;
        out[1][i] = in2 * dry + loop2 * level * wet;
        out[2][i] = out[3][i] = 0.0f;
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

    const float  input_gain = g_input_gain;
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
    disp.WriteString("1/3", Font_6x8, true);
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
        const int ls10 = (int)roundf((float)max_rec / 48000.0f * 10.0f);
        const int lv10 = (int)roundf(play_level  * 10.0f);
        const int gn10 = (int)roundf(input_gain  * 10.0f);
        snprintf(tmp, sizeof(tmp), "LN%d.%ds LV%d.%d GN%d.%d",
                 ls10/10, ls10%10, lv10/10, lv10%10, gn10/10, gn10%10);
        disp.SetCursor(0, 42);
        disp.WriteString(tmp, Font_6x8, true);
    }

    // ── Signal flow ──────────────────────────────────────────────────────────
    disp.SetCursor(0, 54);
    disp.WriteString(is_rec_a ? "REC:A+C LOOP:B+D" : "REC:B+D LOOP:A+C", Font_6x8, true);
}

// ─── Page 1: filter control ───────────────────────────────────────────────────
static void DrawFilterPage()
{
    auto& disp = patch.display;

    disp.SetCursor(0, 0);
    disp.WriteString("FILTER", Font_7x10, true);
    disp.SetCursor(109, 2);
    disp.WriteString("2/3", Font_6x8, true);
    disp.DrawLine(0, 12, 127, 12, true);

    const FilterType ftype = g_filter_type;
    const size_t     delay = g_comb_delay;
    const float      alpha = g_comb_alpha;

    // Filter type label
    disp.SetCursor(0, 14);
    if (ftype == FilterType::OFF)
        disp.WriteString("TYPE: OFF", Font_6x8, true);
    else
        disp.WriteString("TYPE: COMB", Font_6x8, true);

    // Comb delay → frequency (Fs/D)
    {
        char tmp[32];
        const int freq_hz = (delay > 0u) ? (int)(48000u / delay) : 0;
        snprintf(tmp, sizeof(tmp), "FREQ: %dHz", freq_hz);
        disp.SetCursor(0, 24);
        disp.WriteString(tmp, Font_6x8, true);
    }

    // Alpha coefficient
    {
        char tmp[32];
        const int a10 = (int)roundf(alpha * 100.0f);
        snprintf(tmp, sizeof(tmp), "ALPHA: 0.%02d", a10);
        disp.SetCursor(0, 34);
        disp.WriteString(tmp, Font_6x8, true);
    }

    disp.DrawLine(0, 44, 127, 44, true);

    disp.SetCursor(0, 46);
    disp.WriteString("K1=Type K2=Freq K3=Gain", Font_6x8, true);
}

// ─── Page 2: I/O reference ───────────────────────────────────────────────────
static void DrawIOPage()
{
    auto& disp = patch.display;

    disp.SetCursor(0, 0);
    disp.WriteString("I/O REFERENCE", Font_7x10, true);
    disp.SetCursor(109, 2);
    disp.WriteString("3/3", Font_6x8, true);
    disp.DrawLine(0, 12, 127, 12, true);

    disp.SetCursor(0, 14);
    disp.WriteString("IN1=ch1   IN2=ch2", Font_6x8, true);
    disp.SetCursor(0, 23);
    disp.WriteString("OUT1=ch1  OUT2=ch2", Font_6x8, true);
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

    switch (g_page)
    {
        case 1u:  DrawFilterPage(); break;
        case 2u:  DrawIOPage();     break;
        default:  DrawMainPage();   break;
    }

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

    // ── Knob pickup ───────────────────────────────────────────────────────────
    // K1/K2/K3 have different meanings on the filter page vs sampler pages.
    // To prevent parameter jumps when switching contexts, each knob is "frozen"
    // at its last-known value until the physical position catches up (passes within
    // 0.04 of the saved value). K4 is always dry/wet — no context switch needed.
    //
    // ctx 0 = sampler (pages 0 and 2), ctx 1 = filter (page 1)
    struct PickupCtx { float saved[3]; bool live[3]; };
    static PickupCtx pu[2] = {
        {{0.5f, 0.5f, 0.5f}, {true, true, true}},   // sampler: start live
        {{0.5f, 0.5f, 0.5f}, {true, true, true}}    // filter:  start live
    };
    static uint8_t prev_pu_ctx = 0u;  // matches initial g_page = 0

    const uint8_t pu_ctx = (g_page == 1u) ? 1u : 0u;
    if (pu_ctx != prev_pu_ctx)
    {
        pu[pu_ctx].live[0] = pu[pu_ctx].live[1] = pu[pu_ctx].live[2] = false;
        prev_pu_ctx = pu_ctx;
    }

    // Resolve knob: hold saved value until physical position catches up
    auto pickup = [&](int j, float raw) -> float {
        if (!pu[pu_ctx].live[j] && fabsf(raw - pu[pu_ctx].saved[j]) < 0.04f)
            pu[pu_ctx].live[j] = true;
        if (pu[pu_ctx].live[j]) { pu[pu_ctx].saved[j] = raw; return raw; }
        return pu[pu_ctx].saved[j];
    };

    const float k1 = pickup(0, patch.GetKnobValue(DaisyPatch::CTRL_1));
    const float k2 = pickup(1, patch.GetKnobValue(DaisyPatch::CTRL_2));
    const float k3 = pickup(2, patch.GetKnobValue(DaisyPatch::CTRL_3));
    const float k4 = patch.GetKnobValue(DaisyPatch::CTRL_4);

    // K4 = dry/wet mix on every page
    g_wet_mix = k4;

    if (g_page == 1u)
    {
        // ── Filter page: K1=FilterType, K2=comb delay (log), K3=comb alpha ──
        // K1: 0.0–0.5 = OFF, 0.5–1.0 = COMB
        g_filter_type = (k1 >= 0.5f) ? FilterType::COMB : FilterType::OFF;

        // K2: logarithmic comb delay 48–4800 samples (1 ms–100 ms at 48 kHz)
        // delay = 48 * 100^k  →  k=0: 48 samples (1 ms),  k=1: 4800 samples (100 ms)
        const float delay_f = 48.0f * powf(100.0f, k2);
        g_comb_delay = (size_t)(delay_f);
        if (g_comb_delay < 1u) g_comb_delay = 1u;
        if (g_comb_delay > CombState::MAX) g_comb_delay = CombState::MAX;

        // K3: feedback coefficient 0.0–0.95
        g_comb_alpha = k3 * 0.95f;
    }
    else
    {
        // ── Sampler pages (0 and 2): K1=RecLen, K2=Vol, K3=Gain ─────────────
        // CTRL_1: logarithmic curve  0.1 s (k=0) → 60 s (k=1)
        const float len_sec = 0.1f * powf(600.0f, k1);
        g_max_rec_len = (size_t)(len_sec * 48000.0f);

        g_play_level = k2;          // CTRL_2: playback level 0.0–1.0
        g_input_gain = k3 * 2.0f;  // CTRL_3: input gain 0.0–2.0 (unity at centre)
    }
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
