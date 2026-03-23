#pragma once

namespace constants {

// -----------------------------------------------------------------------------
// Hardware / ADC Compensation
// -----------------------------------------------------------------------------

// Knob/slider deadzone compensation
constexpr float DEFAULT_KNOB_DEADZONE = 0.002f;
constexpr float FEEDBACK_KNOB_DEADZONE = 0.028f;
constexpr float SPREAD_KNOB_DEADZONE = 0.0008f;
constexpr float TIME_KNOB_DEADZONE = 0.0008f;

// CV calibration threshold
constexpr float CV_ZERO_THRESHOLD = 0.05f;

// -----------------------------------------------------------------------------
// Slew / Smoothing Rates
// -----------------------------------------------------------------------------
// Slew coefficient determines smoothing speed.
// Use msToSlewCoef() to convert milliseconds to coefficient at 48kHz.

constexpr float SAMPLE_RATE = 48000.0f;

// Convert time constant in milliseconds to slew coefficient
// Formula: coef = 1000 / (ms * sampleRate)
constexpr float msToSlewCoef(float ms) {
    return 1000.0f / (ms * SAMPLE_RATE);
}

constexpr float SLEW_DEFAULT_COEF = msToSlewCoef(21.0f);   // 21ms
constexpr float SLEW_KNOB_COEF = msToSlewCoef(21.0f);      // 21ms
constexpr float SLEW_CV_COEF = msToSlewCoef(4.0f);         // 4ms
constexpr float SLEW_FEEDBACK_COEF = msToSlewCoef(2.0f);   // 2ms
constexpr float SLEW_AMP_ATTACK = 1.0f;                    // Instant (coef=1.0)
constexpr float SLEW_AMP_RELEASE = msToSlewCoef(210.0f);   // 210ms

constexpr float CV_NOISE_FLOOR = 0.005f;                   // Ignore CV changes < 0.5%
constexpr float KNOB_NOISE_FLOOR = 0.005f;                 // Ignore knob changes < 0.5%

// Settle detection: samples to wait before considering input stable
// 96 samples at 48kHz = 2ms
constexpr int SLEW_SETTLE_SAMPLES = 96;

// Sentinel value indicating slew coefficients need recalculation
constexpr float SLEW_COEF_UNINITIALIZED = -100.0f;

// DC blocker coefficient
// At 48kHz: time constant ~2 seconds, cutoff ~0.08Hz
constexpr float DC_BLOCKER_COEF = 0.00001f;

// -----------------------------------------------------------------------------
// Limiter / Dynamics
// -----------------------------------------------------------------------------

// LoudnessDetector asymmetric envelope timings
// Fast attack to catch transients before they arrive (must be < lookahead time)
// Slow release to avoid pumping artifacts
constexpr float LOUDNESS_ATTACK_MS = 3.0f;           // Fast attack (< 10ms lookahead)
constexpr float LOUDNESS_RELEASE_MS = 75.0f;         // Slow release for smoothness

// Feedback ceiling dynamically caps effective feedback to keep buffer bounded
constexpr float FB_TARGET_LEVEL = 1.0f;              // Buffer stabilizes around +/-1.0

// Number of sample points to check in lookahead window (performance vs accuracy)
constexpr int LOOKAHEAD_SAMPLE_POINTS = 4;

// -----------------------------------------------------------------------------
// ReadHead / Crossfade
// -----------------------------------------------------------------------------

// Crossfade time when delay tap parameters change
// delta = 5.0/sampleRate, which gives 200ms crossfade
constexpr float CROSSFADE_RATE = 5.0f;               // Results in ~200ms at 48kHz

// -----------------------------------------------------------------------------
// Control Response Curves
// -----------------------------------------------------------------------------

// Input 0.45-0.55 maps to constant 0.5 output (10% deadzone at center)
constexpr float WARP_POINT_B_IN = 0.45f;             // Start of center plateau
constexpr float WARP_POINT_B_OUT = 0.5f;
constexpr float WARP_POINT_C_IN = 0.55f;             // End of center plateau
constexpr float WARP_POINT_C_OUT = 0.5f;

constexpr float SPREAD_EXPONENT = 2.5f;

// 1.0 = constant power, 1.5 = slightly more aggressive center cut
constexpr float PAN_LAW_EXPONENT = 1.5f;

// 0.0 = no resonance (gentle 12dB/oct), 0.3 = tight response (~2-3dB peak)
constexpr float FILTER_RESONANCE = 0.3f;

constexpr float SCHMIDT_HYSTERESIS = 0.333f;

// -----------------------------------------------------------------------------
// Time / Clock Sync
// -----------------------------------------------------------------------------

// Clock sync quantization steps
constexpr int CLOCK_KNOB_STEPS = 12;                 // Knob: 12 steps = +/-6 octaves
constexpr int CLOCK_CV_STEPS = 10;                   // CV: 10 steps = +/-5 octaves
constexpr float CLOCK_CENTER_DIVISOR = 64.0f;        // 2^6, centers knob at 1x

// Free-running time parameters
constexpr float TIME_MAX_SECONDS = 8.0f;             // Max delay at knob=1.0
constexpr float TIME_KNOB_CURVE = 2.0f;              // Quadratic for fine control at short times
constexpr float TIME_CV_OCTAVE_RANGE = 5.0f;         // CV scales +/-5 octaves (32x range)

// -----------------------------------------------------------------------------
// Feedback
// -----------------------------------------------------------------------------

// Feedback gain scaling
constexpr float FEEDBACK_GAIN_SCALE = 2.0f;          // Multiplier on knob value
constexpr float FEEDBACK_MAX = 3.0f;                 // Maximum feedback value

// -----------------------------------------------------------------------------
// Dynamics Processor (bespoke ducking/limiting system)
// -----------------------------------------------------------------------------

// Lookahead: 10ms at 48kHz allows peak detection before transients hit
constexpr int LOOKAHEAD_SAMPLES = 480;

// Dry envelope: instant attack captures transients, slow release for smooth ducking
constexpr float DRY_ENV_ATTACK_MS = 0.0f;            // Instant attack
constexpr float DRY_ENV_RELEASE_MS = 100.0f;         // 100ms release

// Ducking: wet signal reduced when dry is loud to prevent dry+wet clipping
constexpr float DUCK_HEADROOM = 0.9f;                // Target combined level
constexpr float DUCK_RELEASE_MS = 100.0f;            // Smooth return to full wet

// Limiting: soft ceiling on wet signal peaks
constexpr float LIMIT_THRESHOLD = 0.95f;             // Start limiting at 95%
constexpr float LIMIT_RELEASE_MS = 50.0f;            // Fast release for transparency

// Feedback limiting: tighter threshold to prevent runaway
constexpr float FB_LIMIT_THRESHOLD = 0.7f;           // 70% threshold for feedback path

// Gain smoothing: prevents clicks/pops from sudden gain changes
constexpr float GAIN_SMOOTHING_COEF = 0.002f;        // ~10ms at 48kHz


} // namespace constants
