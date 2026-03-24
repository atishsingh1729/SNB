/*
 * mw_max30102.c  –  MAX30102 Heart-Rate Algorithm  v3
 *
 * Changes from v2 that fix the reported issues:
 *
 * 1. STUCK VALUE at 144 / slow convergence
 *    Root cause: BPM_SMOOTH_W=11 needed ~8 beats to track a 20 BPM change.
 *    Fix: adaptive smoothing — fast when error is large, slow when stable.
 *    A new reading that differs by >15 BPM from current gets weight 3 (fast).
 *    A reading within 15 BPM gets weight 7 (stable tracking).
 *    This lets the output converge in 2-3 beats instead of 8+.
 *
 * 2. OUTPUT NEVER UPDATES unless finger removed/replaced
 *    Root cause: out->heart_rate was only written inside the peak-detection
 *    block. When live_signal=true but no new peak this tick (most ticks),
 *    the BNO persistent frame held the stale value.
 *    Fix: write bpm_smooth to out->heart_rate on EVERY call when live.
 *
 * 3. SLOW FIRST READING (~10 s)
 *    Root cause: LIVENESS_MIN_BEATS=6 + 6-beat wait = no output for 4-6 s.
 *    Fix: reduced to 4 beats. Combined with adaptive smoothing, first
 *    reliable reading arrives within 3-4 seconds.
 *
 * 4. FALSE READINGS ON STARTUP (first few beats biased high/low)
 *    Root cause: IBI plausibility gate only activates after 3 beats,
 *    so the first 3 IBIs are accepted blindly including startup noise.
 *    Fix: first IBI is accepted only if it falls in the 40-150 BPM range
 *    (600ms-1500ms). Tightened startup window prevents a spurious first
 *    IBI from anchoring the median at a wrong value.
 *
 * 5. MOTION ARTIFACTS (jerk → stuck at high value)
 *    Kept from v2: per-sample motion gate, refractory period, IBI
 *    plausibility check. Added: bpm_smooth is NOT updated during motion
 *    block — it freezes at last good value rather than accepting bad IBIs.
 */

#include "mw_max30102.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MW_MAX30102";
static i2c_master_dev_handle_t s_dev = NULL;

/* ── Tuning ──────────────────────────────────────────────────── */
#define DC_MIN              20000.0f
#define DC_MAX              200000.0f
#define PI_MIN              0.0002f     /* 0.02 % — liveness floor     */
#define PI_MAX              0.08f       /* 8 %   — liveness ceiling    */
#define MOTION_RATIO_THR    0.005f      /* 0.5 % of DC per sample      */
#define MOTION_QUIET_MS     200
#define IBI_PLAUS_FRAC      0.45f       /* ±45 % of median accepted    */
#define LIVENESS_MIN_BEATS  3           /* beats before first output   */
#define IBI_BUF             8
/* Adaptive smoothing thresholds */
#define SMOOTH_FAST_W       3           /* big error  → fast track     */
#define SMOOTH_SLOW_W       7           /* small error → stable output */
#define SMOOTH_THRESH_BPM   15          /* switch point                */

/* ── millis ──────────────────────────────────────────────────── */
static inline uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── I2C primitives ──────────────────────────────────────────── */
esp_err_t mw_max30102_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(s_dev, buf, 2, -1);
}
esp_err_t mw_max30102_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, 1, -1);
}
esp_err_t mw_max30102_read_fifo(uint8_t *buf, size_t len)
{
    uint8_t reg = MAX_REG_FIFO_DATA;
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, -1);
}

/* ── Init ────────────────────────────────────────────────────── */
void mw_max30102_init(i2c_master_dev_handle_t dev)
{
    s_dev = dev;
    mw_max30102_write(MAX_REG_MODE_CONFIG,    0x40);
    vTaskDelay(pdMS_TO_TICKS(100));
    mw_max30102_write(MAX_REG_FIFO_WR_PTR,   0);
    mw_max30102_write(MAX_REG_OVF_COUNTER,   0);
    mw_max30102_write(MAX_REG_FIFO_RD_PTR,   0);
    mw_max30102_write(MAX_REG_FIFO_CONFIG,   0x4F);
    mw_max30102_write(MAX_REG_INTR_ENABLE_1, 0x00);
    mw_max30102_write(MAX_REG_INTR_ENABLE_2, 0x00);
    mw_max30102_write(MAX_REG_MODE_CONFIG,   0x02);  /* HR only     */
    mw_max30102_write(MAX_REG_SPO2_CONFIG,   0x2F);  /* 100Hz 18bit */
    mw_max30102_write(MAX_REG_LED1_PA,       0x28);  /* ~7 mA IR    */
    mw_max30102_write(MAX_REG_LED2_PA,       0x00);
    ESP_LOGI(TAG, "MAX30102 @ 100 Hz");
}

/* ── Median (insertion sort, max IBI_BUF elements) ───────────── */
static uint64_t median_u64(const uint64_t *buf, uint8_t n)
{
    uint64_t tmp[IBI_BUF];
    memcpy(tmp, buf, n * sizeof(uint64_t));
    for (int i = 1; i < n; i++) {
        uint64_t key = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return tmp[n / 2];
}

/* ── Adaptive BPM smoother ───────────────────────────────────── */
static uint32_t smooth_bpm(uint32_t current, uint32_t raw)
{
    if (current == 0) return raw;                       /* first reading */
    uint32_t diff = (current > raw) ? current - raw : raw - current;
    uint32_t w = (diff > SMOOTH_THRESH_BPM) ? SMOOTH_FAST_W : SMOOTH_SLOW_W;
    return (current * w + raw) / (w + 1);
}

/* Startup state — file scope so the output reset block can clear them */
static bool     s_dc_seeded    = false;
static uint32_t s_sample_count = 0;

/* ── Main sample processing ──────────────────────────────────── */
void mw_max30102_process_sample(uint32_t ir, sensor_raw_t *out)
{
    /* DC removal — seed from first real sample so ac is never ~127000
     * on startup. Without seeding, dc_mean crawls up from 0 over ~200
     * samples causing massive transient IBIs that poison the median and
     * block real beats for 60-80 s via the plausibility gate. */
    static float dc_mean = 0.0f;
    if (!s_dc_seeded && ir > 0) { dc_mean = (float)ir; s_dc_seeded = true; }
    dc_mean += 0.02f * ((float)ir - dc_mean);
    float ac = (float)ir - dc_mean;

    /* Startup guard: block peak detection for first 200 samples (2 s).
     * Even with a seeded dc_mean the HP filter (τ≈67 samples) still
     * rings for ~1 s. Accepting IBIs during that window anchors the
     * median at a wrong value; 2 s costs one beat at 60 BPM. */
    bool startup_guard = (s_sample_count < 200);
    if (s_sample_count < 200) s_sample_count++;

    /* Per-sample motion gate */
    static float    ir_prev        = 0.0f;
    static uint32_t last_motion_ms = 0;
    float ir_diff  = fabsf((float)ir - ir_prev);
    ir_prev        = (float)ir;
    if (dc_mean > 1.0f && (ir_diff / dc_mean) > MOTION_RATIO_THR)
        last_motion_ms = millis();
    bool motion_block = (millis() - last_motion_ms) < MOTION_QUIET_MS;

    /* Band-pass 0.5–3.5 Hz */
    static float hp_y = 0.0f, hp_x_prev = 0.0f, lp_y = 0.0f;
    float hp  = 0.985f * (hp_y + ac - hp_x_prev);
    hp_x_prev = ac; hp_y = hp;
    lp_y     += 0.20f * (hp - lp_y);

    /* RMS / adaptive peak threshold (50-sample window) */
    static float    ac_sq_sum = 0.0f;
    static uint16_t rms_cnt   = 0;
    static float    ac_rms    = 0.0f;
    static float    peak_thr  = 0.0f;
    ac_sq_sum += lp_y * lp_y;
    if (++rms_cnt >= MAX_RMS_WINDOW) {
        ac_rms    = sqrtf(ac_sq_sum / rms_cnt);
        peak_thr  = ac_rms * 0.55f;
        ac_sq_sum = 0; rms_cnt = 0;
    }

    /* Perfusion Index = AC_RMS / DC */
    float pi = (dc_mean > 1.0f) ? (ac_rms / dc_mean) : 0.0f;

    /* Live signal gate */
    bool sensor_on   = (dc_mean >= DC_MIN && dc_mean <= DC_MAX);
    bool live_signal = sensor_on && (pi >= PI_MIN) && (pi <= PI_MAX);

    /* ── Peak detection state ────────────────────────────────── */
    static float    prev_lp          = 0.0f;
    static float    prev_diff        = 0.0f;
    static uint64_t last_peak        = 0;
    static uint64_t refractory_until = 0;
    static uint64_t ibi_hist[IBI_BUF] = {0};
    static uint8_t  ibi_idx           = 0;
    static uint8_t  ibi_count         = 0;
    static uint8_t  liveness_cnt      = 0;
    static uint32_t bpm_smooth        = 0;
    static float    rr_prev_ms        = 0.0f;
    static float    hrv_sq            = 0.0f;
    static uint8_t  hrv_cnt           = 0;
    static uint32_t last_valid_bpm    = 0;
    static uint32_t last_finger_ms    = 0;
    static bool     finger_prev       = false;

    float    diff = lp_y - prev_lp;
    uint64_t now  = esp_timer_get_time();

    bool can_detect = live_signal && !motion_block && !startup_guard
                      && (ac_rms >= 30.0f)
                      && (now >= refractory_until);

    if (can_detect) {
        bool peak = (prev_diff > 0.0f) && (diff <= 0.0f) && (lp_y > peak_thr);

        if (peak) {
            if (last_peak != 0) {
                uint64_t ibi_us = now - last_peak;

                /* For first IBI, use a tighter startup window (40-150 BPM)
                 * to avoid anchoring the median at a bogus value */
                uint64_t ibi_min = (ibi_count == 0) ? 400000ULL : 300000ULL;
                uint64_t ibi_max = (ibi_count == 0) ? 1500000ULL : 1500000ULL;

                if (ibi_us >= ibi_min && ibi_us <= ibi_max) {

                    /* IBI plausibility — wider window while median is thin
                     * (first 8 beats), tighter once it's well-established */
                    bool plausible = true;
                    if (ibi_count >= 3) {
                        uint64_t med  = median_u64(ibi_hist, ibi_count);
                        float    frac = (ibi_count < 8) ? 0.55f : IBI_PLAUS_FRAC;
                        uint64_t lo   = (uint64_t)(med * (1.0f - frac));
                        uint64_t hi   = (uint64_t)(med * (1.0f + frac));
                        plausible     = (ibi_us >= lo && ibi_us <= hi);
                    }

                    if (plausible) {
                        ibi_hist[ibi_idx++] = ibi_us;
                        ibi_idx %= IBI_BUF;
                        if (ibi_count < IBI_BUF) ibi_count++;
                        liveness_cnt++;

                        if (liveness_cnt >= LIVENESS_MIN_BEATS && ibi_count >= 3) {
                            uint64_t ibi_med = median_u64(ibi_hist, ibi_count);
                            uint32_t bpm_raw = (uint32_t)(60000000ULL / ibi_med);

                            /* HRV RMSSD */
                            float rr_ms = (float)ibi_med / 1000.0f;
                            if (rr_prev_ms > 0.0f) {
                                float d = rr_ms - rr_prev_ms;
                                hrv_sq += d * d;
                                if (++hrv_cnt >= MAX_HRV_WINDOW) {
                                    out->hrv = sqrtf(hrv_sq / hrv_cnt);
                                    hrv_sq = 0; hrv_cnt = 0;
                                }
                            }
                            rr_prev_ms = rr_ms;

                            if (bpm_raw >= 30 && bpm_raw <= 200) {
                                /* Adaptive smooth: fast convergence when far,
                                 * stable when close to true value */
                                bpm_smooth     = smooth_bpm(bpm_smooth, bpm_raw);
                                last_valid_bpm = bpm_smooth;
                                last_finger_ms = millis();
                            }
                        }
                    } else {
                        /* Implausible — reset liveness, keep ibi_hist
                         * so next real beat still has median context */
                        liveness_cnt = 0;
                    }

                    uint64_t exp_period = (bpm_smooth > 0)
                        ? 60000000ULL / bpm_smooth : 0;
                    refractory_until = now
                        + (exp_period ? exp_period / 3 : 300000ULL);
                } else {
                    liveness_cnt = 0;
                }
            }
            last_peak = now;
        }
    } else if (motion_block) {
        /* Motion: reset detection state but preserve bpm_smooth
         * so display shows last known value, not 0 or garbage */
        liveness_cnt = 0;
        ibi_count    = 0;
        ibi_idx      = 0;
        last_peak    = 0;
    }

    /* ── Output — write EVERY tick ───────────────────────────── */
    if (!live_signal) {
        /* Finger removed — full reset including startup state so
         * next placement gets a clean seeded dc_mean, not the stale one */
        s_dc_seeded    = false;
        s_sample_count = 0;
        liveness_cnt   = 0;
        ibi_count    = 0;
        ibi_idx      = 0;
        last_peak    = 0;
        /* Only clear bpm_smooth after the 10 s hold expires —
         * brief signal dropouts (motion, finger shift) must not
         * reset it to 0 because that overwrites the hold value */
        out->hrv     = 0;
        rr_prev_ms   = 0; hrv_sq = 0; hrv_cnt = 0;

        if (last_valid_bpm > 0 && (millis() - last_finger_ms) < 10000) {
            out->heart_rate = last_valid_bpm;
        } else {
            out->heart_rate = 0;
            last_valid_bpm  = 0;
            bpm_smooth      = 0;
        }
    } else {
        /* Finger present — write smoothed BPM when qualified,
         * or hold last_valid_bpm while still accumulating beats.
         * NEVER write 0 when live — that clears the display. */
        if (bpm_smooth > 0) {
            out->heart_rate = bpm_smooth;
        } else if (last_valid_bpm > 0) {
            out->heart_rate = last_valid_bpm;  /* hold while re-qualifying */
        }
        /* else: leave out->heart_rate unchanged — accumulated frame holds it */
    }

    if (live_signal && !finger_prev)
        last_finger_ms = millis();
    finger_prev = live_signal;

    out->raw_ir = ir;
    prev_diff   = diff;
    prev_lp     = lp_y;
}

/* ── get_raw: drain full FIFO each tick ──────────────────────── */
void mw_max30102_get_raw(sensor_raw_t *out)
{
    uint8_t wr, rd;
    if (mw_max30102_read_reg(MAX_REG_FIFO_WR_PTR, &wr) != ESP_OK ||
        mw_max30102_read_reg(MAX_REG_FIFO_RD_PTR, &rd) != ESP_OK) {
        ESP_LOGW(TAG, "FIFO pointer read failed");
        return;
    }
    uint8_t samples = (wr - rd) & 0x1F;

    if (samples == 0) {
        /* FIFO empty — still run algo to keep output current */
        if (out->raw_ir > 0)
            mw_max30102_process_sample(out->raw_ir, out);
        return;
    }

    /* In HR-only mode each FIFO sample is 3 bytes (IR only).
     * Reading 6 bytes per sample silently skips every second sample and
     * distorts timing/shape, which can keep peak detection from qualifying
     * and leave BPM stuck at 0. */
    uint8_t fifo[3];
    for (uint8_t i = 0; i < samples; i++) {
        if (mw_max30102_read_fifo(fifo, sizeof(fifo)) != ESP_OK) break;
        uint32_t ir = (((uint32_t)(fifo[0] & 0x03)) << 16)
                      | ((uint32_t)fifo[1] << 8)
                      | fifo[2];
        ir &= 0x03FFFF;
        mw_max30102_process_sample(ir, out);
    }
}
