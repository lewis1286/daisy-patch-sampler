// sampler_core.cpp — all sampler application logic.
// Include exactly once per build via daisy_patch_sampler.cpp or algorithms/sampler.cpp.
// Never add to CPP_SOURCES — always pulled in by the #include in the entry file.
#include "daisysp.h"
#include "daisy_patch.h"
#include "per/sdmmc.h"
#include "sys/fatfs.h"
#include "arm_math.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace daisy;
using namespace daisysp;

// ─── Hardware ────────────────────────────────────────────────────────────────
// s_sdmmc and s_fsi MUST be file-scope globals (AXI SRAM).
// The Daisy MSP stack lives in DTCM which SDMMC DMA cannot reach.
static DaisyPatch*     s_patch = nullptr;
static SdmmcHandler    s_sdmmc;
static FatFSInterface  s_fsi;

// ─── Audio block size (must match DaisyPatch default; used to size FIR state)
static constexpr uint32_t AUDIO_BLOCK_SIZE = 48u;

// ─── Buffers — 60 s each in SDRAM (~44 MB total for 4 buffers, ~20 MB headroom)
static constexpr size_t MAX_SAMPLES = 48000u * 60u;   // 2,880,000 samples

float DSY_SDRAM_BSS buf_a[MAX_SAMPLES];  // ch1 ping
float DSY_SDRAM_BSS buf_b[MAX_SAMPLES];  // ch1 pong
float DSY_SDRAM_BSS buf_c[MAX_SAMPLES];  // ch2 ping
float DSY_SDRAM_BSS buf_d[MAX_SAMPLES];  // ch2 pong

// ─── IR buffers — up to 8 IRs of up to 1024 samples each (32 KB in SDRAM)
// Written once at startup (before StartAudio), then read-only.
// arm_fir_f32 with -DARM_MATH_LOOPUNROLL (8 accumulators, 8-wide outer loop) achieves
// ~1.25 cycles/tap/sample effective throughput, so 1024 taps × 2 ch × 48 samples ≈ 123K
// cycles (~26% of the 480K budget). Longer WAV files are truncated at load time.
static constexpr uint8_t MAX_IR_SLOTS = 8u;
static constexpr size_t  MAX_IR_LEN   = 1024u;

float DSY_SDRAM_BSS ir_data[MAX_IR_SLOTS][MAX_IR_LEN];

static uint8_t  g_ir_count              = 0;
static size_t   ir_slot_len[MAX_IR_SLOTS] = {};
static char     ir_names[MAX_IR_SLOTS][16] = {};

volatile uint8_t g_ir_sel = 0;

// ─── State ───────────────────────────────────────────────────────────────────
enum class State : uint8_t { RECORD_A_PLAY_B, RECORD_B_PLAY_A };

volatile State  g_state      = State::RECORD_A_PLAY_B;
volatile size_t g_rec_pos    = 0;
volatile size_t g_play_pos   = 0;
volatile size_t g_play_len_a = 0;
volatile size_t g_play_len_b = 0;

// ─── Params (written by main loop, read by audio ISR) ────────────────────────
volatile float g_play_level = 1.0f;
volatile float g_input_gain = 1.0f;
volatile float g_wet_mix    = 1.0f;

// ─── Filter bank ─────────────────────────────────────────────────────────────
static constexpr float ALPHA_DECAY = 0.9f;
static constexpr float TAP2_RATIO  = 0.75f;
static constexpr float TAP3_RATIO  = 0.55f;

enum class FilterType : uint8_t { OFF, COMB, COMB2, COMB3, CONV };

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
volatile size_t     g_comb_delay  = 480u;
volatile float      g_comb_alpha  = 0.0f;

// ─── CMSIS-DSP FIR state (DTCM — 0-wait-state access for state/history buffers)
static constexpr size_t FIR_STATE_LEN = MAX_IR_LEN + AUDIO_BLOCK_SIZE - 1u;

struct FirState
{
    arm_fir_instance_f32 ch1;
    arm_fir_instance_f32 ch2;
    float state1[FIR_STATE_LEN];
    float state2[FIR_STATE_LEN];
};
static FirState DTCM_MEM_SECTION fir;

// ─── Run state ───────────────────────────────────────────────────────────────
volatile bool g_running = true;

// ─── Crossfade state (audio ISR only) ────────────────────────────────────────
static constexpr size_t XFADE_LEN = 240u;  // 5 ms at 48 kHz

// ─── Peak limiter state (audio ISR only) ─────────────────────────────────────
static constexpr float LIM_THRESHOLD = 0.8f;
static constexpr float LIM_RELEASE   = 0.999896f;  // exp(-1 / (0.2 * 48000))
static float           lim_env       = 0.0f;

static const float* xfade_old1   = nullptr;
static const float* xfade_old2   = nullptr;
static size_t       xfade_pos    = 0u;
static size_t       xfade_oldlen = 0u;
static size_t       xfade_rem    = 0u;

// ─── Display timing + page ───────────────────────────────────────────────────
static uint32_t           last_display_ms = 0u;
static constexpr uint32_t DISPLAY_MS      = 33u;
static constexpr uint8_t  NUM_PAGES       = 3u;
static uint8_t            g_page          = 0u;

// ─── IR loading from SD card ─────────────────────────────────────────────────
static bool LoadWAV(const char* path, float* dst, size_t* out_len)
{
    static FIL fil;  // static: AXI SRAM — FIL.buf[512] is the DMA target
    UINT br;

    const FRESULT fres = f_open(&fil, path, FA_READ);
    if (fres != FR_OK)
        return false;

    uint8_t hdr[256];
    f_read(&fil, hdr, sizeof(hdr), &br);
    if (br < 44) { f_close(&fil); return false; }

    if (hdr[0]!='R'||hdr[1]!='I'||hdr[2]!='F'||hdr[3]!='F') { f_close(&fil); return false; }
    if (hdr[8]!='W'||hdr[9]!='A'||hdr[10]!='V'||hdr[11]!='E') { f_close(&fil); return false; }

    uint16_t num_ch    = 0;
    uint16_t bit_depth = 0;
    uint32_t samp_rate = 0;
    uint32_t data_size = 0;
    uint32_t data_off  = 0;

    uint32_t pos = 12u;
    while (pos + 8u <= br)
    {
        const char*    id = reinterpret_cast<const char*>(&hdr[pos]);
        const uint32_t chunk_sz = static_cast<uint32_t>(hdr[pos+4])
                                | (static_cast<uint32_t>(hdr[pos+5]) << 8)
                                | (static_cast<uint32_t>(hdr[pos+6]) << 16)
                                | (static_cast<uint32_t>(hdr[pos+7]) << 24);

        if (id[0]=='f' && id[1]=='m' && id[2]=='t' && id[3]==' ')
        {
            if (pos + 8u + 18u <= br)
            {
                num_ch    = static_cast<uint16_t>(hdr[pos+10]) | (static_cast<uint16_t>(hdr[pos+11]) << 8);
                samp_rate = static_cast<uint32_t>(hdr[pos+12])
                          | (static_cast<uint32_t>(hdr[pos+13]) << 8)
                          | (static_cast<uint32_t>(hdr[pos+14]) << 16)
                          | (static_cast<uint32_t>(hdr[pos+15]) << 24);
                bit_depth = static_cast<uint16_t>(hdr[pos+22]) | (static_cast<uint16_t>(hdr[pos+23]) << 8);
            }
        }
        else if (id[0]=='d' && id[1]=='a' && id[2]=='t' && id[3]=='a')
        {
            data_size = chunk_sz;
            data_off  = pos + 8u;
            break;
        }

        pos += 8u + chunk_sz;
        if (chunk_sz & 1u) ++pos;
    }

    if (num_ch != 1 || samp_rate != 48000 || bit_depth != 16 || data_size == 0)
    {
        f_close(&fil);
        return false;
    }

    f_lseek(&fil, data_off);

    static int16_t tmp[MAX_IR_LEN];
    const size_t   max_bytes  = MAX_IR_LEN * 2u;
    const size_t   read_bytes = (data_size < max_bytes) ? data_size : max_bytes;
    f_read(&fil, tmp, static_cast<UINT>(read_bytes), &br);
    f_close(&fil);

    const size_t n = br / 2u;
    if (n == 0) return false;

    float peak = 0.0f;
    for (size_t i = 0; i < n; ++i)
    {
        dst[i] = static_cast<float>(tmp[i]) / 32768.0f;
        const float a = dst[i] < 0.0f ? -dst[i] : dst[i];
        if (a > peak) peak = a;
    }

    if (peak > 1e-6f)
    {
        const float inv_peak = 1.0f / peak;
        for (size_t i = 0; i < n; ++i)
            dst[i] *= inv_peak;
    }

    // arm_fir_f32 expects coefficients in time-reversed order {h[N-1],...,h[0]}
    for (size_t i = 0u, j = n - 1u; i < j; ++i, --j)
        std::swap(dst[i], dst[j]);

    *out_len = n;
    return true;
}

static void LoadIRs()
{
    SdmmcHandler::Config sdcfg;
    sdcfg.Defaults();
    s_sdmmc.Init(sdcfg);
    s_fsi.Init(FatFSInterface::Config::MEDIA_SD);
    if (f_mount(&s_fsi.GetSDFileSystem(), "/", 1) != FR_OK) return;

    g_ir_count = 0;

    for (uint8_t slot = 0; slot < MAX_IR_SLOTS; ++slot)
    {
        char path[16];
        snprintf(path, sizeof(path), "IR_%02u.wav", slot);

        size_t len = 0;
        if (!LoadWAV(path, ir_data[slot], &len))
            break;

        ir_slot_len[slot] = len;
        snprintf(ir_names[slot], sizeof(ir_names[0]), "IR_%02u", slot);
        g_ir_count = slot + 1u;
    }

    f_mount(nullptr, "/", 0);
}

// ─── Filter dispatch (comb modes) ────────────────────────────────────────────
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
            comb.buf1[comb.write] = y1 >  4.0f ?  4.0f : y1 < -4.0f ? -4.0f : y1;
            comb.buf2[comb.write] = y2 >  4.0f ?  4.0f : y2 < -4.0f ? -4.0f : y2;
            comb.write = (comb.write + 1u < CombState::MAX) ? comb.write + 1u : 0u;
            s1 = y1;
            s2 = y2;
            break;
        }
        case FilterType::COMB2:
        {
            const size_t d2  = (size_t)(delay * TAP2_RATIO);
            const size_t rd1 = (comb.write + CombState::MAX - delay) % CombState::MAX;
            const size_t rd2 = (comb.write + CombState::MAX - (d2 < 1u ? 1u : d2)) % CombState::MAX;
            const float  a2  = alpha * ALPHA_DECAY;
            const float  y1  = s1 + alpha * comb.buf1[rd1] + a2 * comb.buf1[rd2];
            const float  y2  = s2 + alpha * comb.buf2[rd1] + a2 * comb.buf2[rd2];
            comb.buf1[comb.write] = y1 >  4.0f ?  4.0f : y1 < -4.0f ? -4.0f : y1;
            comb.buf2[comb.write] = y2 >  4.0f ?  4.0f : y2 < -4.0f ? -4.0f : y2;
            comb.write = (comb.write + 1u < CombState::MAX) ? comb.write + 1u : 0u;
            s1 = y1;
            s2 = y2;
            break;
        }
        case FilterType::COMB3:
        {
            const size_t d2  = (size_t)(delay * TAP2_RATIO);
            const size_t d3  = (size_t)(delay * TAP3_RATIO);
            const size_t rd1 = (comb.write + CombState::MAX - delay) % CombState::MAX;
            const size_t rd2 = (comb.write + CombState::MAX - (d2 < 1u ? 1u : d2)) % CombState::MAX;
            const size_t rd3 = (comb.write + CombState::MAX - (d3 < 1u ? 1u : d3)) % CombState::MAX;
            const float  a2  = alpha * ALPHA_DECAY;
            const float  a3  = a2    * ALPHA_DECAY;
            const float  y1  = s1 + alpha * comb.buf1[rd1] + a2 * comb.buf1[rd2] + a3 * comb.buf1[rd3];
            const float  y2  = s2 + alpha * comb.buf2[rd1] + a2 * comb.buf2[rd2] + a3 * comb.buf2[rd3];
            comb.buf1[comb.write] = y1 >  4.0f ?  4.0f : y1 < -4.0f ? -4.0f : y1;
            comb.buf2[comb.write] = y2 >  4.0f ?  4.0f : y2 < -4.0f ? -4.0f : y2;
            comb.write = (comb.write + 1u < CombState::MAX) ? comb.write + 1u : 0u;
            s1 = y1;
            s2 = y2;
            break;
        }
        case FilterType::OFF:
        case FilterType::CONV:
        default:
            break;
    }
}

// ─── CMSIS-DSP FIR initialisation ────────────────────────────────────────────
static void InitFIR(uint8_t slot)
{
    const size_t ir_len = ir_slot_len[slot];
    if (ir_len == 0u) return;

    const uint16_t taps = static_cast<uint16_t>(ir_len);
    arm_fir_init_f32(&fir.ch1, taps, ir_data[slot], fir.state1, AUDIO_BLOCK_SIZE);
    arm_fir_init_f32(&fir.ch2, taps, ir_data[slot], fir.state2, AUDIO_BLOCK_SIZE);
}

// ─── Buffer swap ─────────────────────────────────────────────────────────────
static inline void SwapBuffers()
{
    const bool is_rec_a = (g_state == State::RECORD_A_PLAY_B);

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

// ─── Display helpers ─────────────────────────────────────────────────────────
static inline int ClampX(int v) { return v < 0 ? 0 : v > 127 ? 127 : v; }
static inline int ClampY(int v) { return v < 0 ? 0 : v >  63 ?  63 : v; }

static void DrawProgressBar(int x, int y, int w, int h, float fill)
{
    auto& d  = s_patch->display;
    const int x1 = x + w - 1;
    const int y1 = y + h - 1;

    d.DrawLine(ClampX(x),  ClampY(y),  ClampX(x1), ClampY(y),  true);
    d.DrawLine(ClampX(x),  ClampY(y1), ClampX(x1), ClampY(y1), true);
    d.DrawLine(ClampX(x),  ClampY(y),  ClampX(x),  ClampY(y1), true);
    d.DrawLine(ClampX(x1), ClampY(y),  ClampX(x1), ClampY(y1), true);

    const float clamped = fill < 0.0f ? 0.0f : fill > 1.0f ? 1.0f : fill;
    const int   px      = (int)((w - 2) * clamped);
    if (px > 0)
    {
        for (int row = y + 1; row < y1; row++)
            d.DrawLine(ClampX(x + 1), ClampY(row), ClampX(x + px), ClampY(row), true);
    }
}

static void DrawMainPage()
{
    const State  cur_state  = g_state;
    const size_t rec_pos    = g_rec_pos;
    const size_t play_pos   = g_play_pos;
    const size_t play_len_a = g_play_len_a;
    const size_t play_len_b = g_play_len_b;
    const float  play_level = g_play_level;
    const float  input_gain = g_input_gain;

    const bool   is_rec_a   = (cur_state == State::RECORD_A_PLAY_B);
    const size_t play_len   = is_rec_a ? play_len_b : play_len_a;
    const bool   rec_frozen = (rec_pos >= MAX_SAMPLES);
    const bool   running    = g_running;

    auto& disp = s_patch->display;

    disp.SetCursor(0, 0);
    disp.WriteString("SAMPLER", Font_6x8, true);
    disp.SetCursor(60, 2);
    disp.WriteString(running ? "RUN" : "STP", Font_6x8, true);
    disp.SetCursor(109, 2);
    disp.WriteString("1/3", Font_6x8, true);
    disp.DrawLine(0, 12, 127, 12, true);

    disp.SetCursor(0, 16);
    disp.WriteString(rec_frozen ? "R*" : "R ", Font_6x8, true);
    {
        const float fill = (float)rec_pos / (float)MAX_SAMPLES;
        DrawProgressBar(14, 15, 113, 9, fill);
    }

    disp.SetCursor(0, 28);
    disp.WriteString("P ", Font_6x8, true);
    {
        const float fill = (play_len > 0u)
            ? (float)(play_pos % play_len) / (float)play_len
            : 0.0f;
        DrawProgressBar(14, 27, 113, 9, fill);
    }

    disp.DrawLine(0, 39, 127, 39, true);

    {
        char tmp[48];
        const int lv10 = (int)roundf(fminf(fmaxf(play_level, 0.0f), 9.9f) * 10.0f);
        const int gn10 = (int)roundf(fminf(fmaxf(input_gain, 0.0f), 9.9f) * 10.0f);
        snprintf(tmp, sizeof(tmp), "LVL %d.%d   GAIN %d.%d",
                 lv10/10, lv10%10, gn10/10, gn10%10);
        disp.SetCursor(0, 42);
        disp.WriteString(tmp, Font_6x8, true);
    }

    disp.SetCursor(0, 54);
    disp.WriteString(is_rec_a ? "REC:A+C LOOP:B+D" : "REC:B+D LOOP:A+C", Font_6x8, true);
}

static void DrawFilterPage()
{
    auto& disp = s_patch->display;

    disp.SetCursor(0, 0);
    disp.WriteString("FILTER", Font_6x8, true);
    disp.SetCursor(109, 2);
    disp.WriteString("2/3", Font_6x8, true);
    disp.DrawLine(0, 12, 127, 12, true);

    const FilterType ftype = g_filter_type;

    if (ftype == FilterType::CONV)
    {
        disp.SetCursor(0, 14);
        disp.WriteString("TYPE: CONV IR", Font_6x8, true);

        if (g_ir_count == 0)
        {
            disp.SetCursor(0, 24);
            disp.WriteString("NO IRs ON SD CARD", Font_6x8, true);
            disp.SetCursor(0, 34);
            disp.WriteString("Copy IR_00.wav etc", Font_6x8, true);
        }
        else
        {
            const uint8_t sel = g_ir_sel < g_ir_count ? g_ir_sel : 0u;
            char tmp[32];

            snprintf(tmp, sizeof(tmp), "%.10s (%u/%u)",
                     ir_names[sel], sel + 1u, g_ir_count);
            disp.SetCursor(0, 24);
            disp.WriteString(tmp, Font_6x8, true);

            const size_t len = ir_slot_len[sel];
            const int    ms  = (int)((len * 1000u) / 48000u);
            snprintf(tmp, sizeof(tmp), "LEN: %usmp (%dms)",
                     (unsigned)len, ms);
            disp.SetCursor(0, 34);
            disp.WriteString(tmp, Font_6x8, true);
        }

        disp.DrawLine(0, 44, 127, 44, true);
        disp.SetCursor(0, 46);
        disp.WriteString("K1=Type  K2=IR sel", Font_6x8, true);
    }
    else
    {
        const size_t delay = g_comb_delay;
        const float  alpha = g_comb_alpha;

        disp.SetCursor(0, 14);
        const char* type_str = "TYPE: OFF";
        if      (ftype == FilterType::COMB)  type_str = "TYPE: COMB (1 tap)";
        else if (ftype == FilterType::COMB2) type_str = "TYPE: COMB2 (2 tap)";
        else if (ftype == FilterType::COMB3) type_str = "TYPE: COMB3 (3 tap)";
        disp.WriteString(type_str, Font_6x8, true);

        {
            char tmp[32];
            const int freq_hz = (delay > 0u) ? (int)(48000u / delay) : 0;
            snprintf(tmp, sizeof(tmp), "FREQ: %dHz", freq_hz);
            disp.SetCursor(0, 24);
            disp.WriteString(tmp, Font_6x8, true);
        }

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
}

static void DrawIOPage()
{
    auto& disp = s_patch->display;

    disp.SetCursor(0, 0);
    disp.WriteString("I/O REFERENCE", Font_6x8, true);
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
    disp.WriteString("K1=Vol      K2=Gain", Font_6x8, true);
    disp.SetCursor(0, 52);
    disp.WriteString("K3=--       K4=Mix", Font_6x8, true);
}

static void UpdateSamplerDisplay()
{
    auto& disp = s_patch->display;
    disp.Fill(false);

    switch (g_page)
    {
        case 1u:  DrawFilterPage(); break;
        case 2u:  DrawIOPage();     break;
        default:  DrawMainPage();   break;
    }

    disp.Update();
}

static void UpdateSamplerControls()
{
    // ProcessAnalogControls/ProcessDigitalControls are called by the entry point's
    // AudioCallback — do not call them here.

    struct PickupCtx { float saved[3]; bool live[3]; };
    static PickupCtx pu[2];
    static bool      pu_inited = false;
    if (!pu_inited)
    {
        for (int c = 0; c < 2; ++c)
            pu[c].live[0] = pu[c].live[1] = pu[c].live[2] = true;
        pu_inited = true;
    }
    static uint8_t prev_pu_ctx = 0u;

    const uint8_t pu_ctx = (g_page == 1u) ? 1u : 0u;
    if (pu_ctx != prev_pu_ctx)
    {
        if (pu[pu_ctx].saved[0] != 0.0f)
            pu[pu_ctx].live[0] = pu[pu_ctx].live[1] = pu[pu_ctx].live[2] = false;
        prev_pu_ctx = pu_ctx;
    }

    auto pickup = [&](int j, float raw) -> float {
        if (!pu[pu_ctx].live[j] && fabsf(raw - pu[pu_ctx].saved[j]) < 0.04f)
            pu[pu_ctx].live[j] = true;
        if (pu[pu_ctx].live[j]) { pu[pu_ctx].saved[j] = raw; return raw; }
        return pu[pu_ctx].saved[j];
    };

    const float k1 = pickup(0, s_patch->GetKnobValue(DaisyPatch::CTRL_1));
    const float k2 = pickup(1, s_patch->GetKnobValue(DaisyPatch::CTRL_2));
    const float k3 = pickup(2, s_patch->GetKnobValue(DaisyPatch::CTRL_3));
    const float k4 = s_patch->GetKnobValue(DaisyPatch::CTRL_4);

    g_wet_mix = k4;

    if (g_page == 1u)
    {
        FilterType new_type;
        if      (k1 < 0.20f) new_type = FilterType::OFF;
        else if (k1 < 0.40f) new_type = FilterType::COMB;
        else if (k1 < 0.60f) new_type = FilterType::COMB2;
        else if (k1 < 0.80f) new_type = FilterType::COMB3;
        else                  new_type = FilterType::CONV;

        g_filter_type = new_type;

        if (new_type == FilterType::CONV)
        {
            if (g_ir_count > 0)
            {
                const uint8_t sel = (uint8_t)(k2 * (float)g_ir_count);
                g_ir_sel = (sel >= g_ir_count) ? g_ir_count - 1u : sel;
            }
        }
        else
        {
            const float delay_f = 48.0f * powf(100.0f, k2);
            g_comb_delay = (size_t)(delay_f);
            if (g_comb_delay < 1u) g_comb_delay = 1u;
            if (g_comb_delay > CombState::MAX) g_comb_delay = CombState::MAX;

            g_comb_alpha = k3 * 0.95f;
        }
    }
    else
    {
        g_play_level = k1;
        g_input_gain = k2 * 2.0f;
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

void SamplerInit(DaisyPatch* p)
{
    s_patch = p;
    LoadIRs();
}

void SamplerOnActivate()
{
    last_display_ms = 0u;
}

void SamplerOnDeactivate() {}

void SamplerProcessBlock(AudioHandle::InputBuffer  in,
                         AudioHandle::OutputBuffer out,
                         size_t                    size)
{
    // ProcessAnalogControls/ProcessDigitalControls are called by the entry point's
    // AudioCallback before dispatching here — do not call them again.

    if (s_patch->gate_input[0].Trig())
        SwapBuffers();

    if (!g_running)
    {
        for (size_t i = 0; i < size; i++)
            out[0][i] = out[1][i] = out[2][i] = out[3][i] = 0.0f;
        return;
    }

    const bool   is_rec_a   = (g_state == State::RECORD_A_PLAY_B);
    const float  gain       = g_input_gain;
    const float  level      = g_play_level;
    const float  wet        = g_wet_mix;
    const float  dry        = 1.0f - wet;
    const size_t play_len   = is_rec_a ? g_play_len_b : g_play_len_a;

    float* const rec_buf   = is_rec_a ? buf_a : buf_b;
    float* const play_buf  = is_rec_a ? buf_b : buf_a;
    float* const rec_buf2  = is_rec_a ? buf_c : buf_d;
    float* const play_buf2 = is_rec_a ? buf_d : buf_c;

    const FilterType filter_type = g_filter_type;
    const size_t     comb_delay  = g_comb_delay;
    const float      comb_alpha  = g_comb_alpha;
    const uint8_t    ir_sel      = g_ir_sel;

    static FilterType prev_filter     = FilterType::OFF;
    static uint8_t    fir_active_slot = 0xFFu;
    const bool was_off = (prev_filter == FilterType::OFF);
    const bool is_comb = (filter_type == FilterType::COMB  ||
                          filter_type == FilterType::COMB2 ||
                          filter_type == FilterType::COMB3);
    const bool is_conv = (filter_type == FilterType::CONV);

    if (is_comb && was_off)
        comb.reset();

    prev_filter = filter_type;

    float loop1_buf[AUDIO_BLOCK_SIZE];
    float loop2_buf[AUDIO_BLOCK_SIZE];

    size_t r = g_rec_pos;
    size_t p = g_play_pos;

    for (size_t i = 0; i < size; i++)
    {
        const float in1 = in[0][i] * gain;
        const float in2 = in[1][i] * gain;

        if (r < MAX_SAMPLES)
        {
            rec_buf[r]  = in1  >  2.0f ?  2.0f : in1  < -2.0f ? -2.0f : in1;
            rec_buf2[r] = in2  >  2.0f ?  2.0f : in2  < -2.0f ? -2.0f : in2;
            r++;
        }

        float loop1 = 0.0f, loop2 = 0.0f;
        if (play_len > 0u)
        {
            loop1 = play_buf[p];
            loop2 = play_buf2[p];
            if (++p >= play_len) p = 0u;
        }

        if (xfade_rem > 0u)
        {
            const float fade_out = (float)xfade_rem / (float)XFADE_LEN;
            const float fade_in  = 1.0f - fade_out;
            loop1 = xfade_old1[xfade_pos] * fade_out + loop1 * fade_in;
            loop2 = xfade_old2[xfade_pos] * fade_out + loop2 * fade_in;
            if (++xfade_pos >= xfade_oldlen) xfade_pos = 0u;
            --xfade_rem;
        }

        if (is_comb)
            ApplyFilter(loop1, loop2, filter_type, comb_delay, comb_alpha);

        loop1_buf[i] = loop1;
        loop2_buf[i] = loop2;
    }

    g_rec_pos  = r;
    g_play_pos = p;

    if (is_conv && g_ir_count > 0u)
    {
        if (fir_active_slot != ir_sel)
        {
            InitFIR(ir_sel);
            fir_active_slot = ir_sel;
        }
        arm_fir_f32(&fir.ch1, loop1_buf, loop1_buf, static_cast<uint32_t>(size));
        arm_fir_f32(&fir.ch2, loop2_buf, loop2_buf, static_cast<uint32_t>(size));
    }

    for (size_t i = 0; i < size; i++)
    {
        const float in1  = in[0][i] * gain;
        const float in2  = in[1][i] * gain;
        float       loop1 = loop1_buf[i];
        float       loop2 = loop2_buf[i];

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
}

void SamplerOnEncoderShortPress()
{
    g_running = !g_running;
}

void SamplerOnEncoderIncrement(int32_t delta)
{
    if (delta > 0)
        g_page = (g_page + 1u) % NUM_PAGES;
    else if (delta < 0)
        g_page = (g_page + NUM_PAGES - 1u) % NUM_PAGES;
}

void SamplerUpdateUI(DaisyPatch& /*patch*/)
{
    UpdateSamplerControls();

    const uint32_t now = System::GetNow();
    if (now - last_display_ms >= DISPLAY_MS)
    {
        UpdateSamplerDisplay();
        last_display_ms = now;
    }
}
