#pragma once

#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "biquad.h"
#include "constants.h"
#include "dynamics.h"

#define HALF_PI 1.570796326794897


int wrap_buffer_index(int x, int size)
{
    while(x >= size) x -= size;
    while(x < 0) x += size;
    return x;
}


int seconds_to_samples(float x, float sampleRate)
{
    return (int)(x * sampleRate);
}


float clamp(float x, float a, float b)
{
    return std::max(a,std::min(b,x));
}


float fourPointWarp(float x,
                    float ai=0.0,
                    float av=0.0,
                    float bi=constants::WARP_POINT_B_IN,
                    float bv=constants::WARP_POINT_B_OUT,
                    float ci=constants::WARP_POINT_C_IN,
                    float cv=constants::WARP_POINT_C_OUT,
                    float di=1.0,
                    float dv=1.0)
{
    if (x < ai)
    {
        return av;
    }

    if (x < bi)
    {
        x = (x - ai) / (bi - ai);
        return av * (1.0 - x) + bv * x;
    }

    if (x < ci)
    {
        x = (x - bi) / (ci - bi);
        return bv * (1.0 - x) + cv * x;
    }

    if (x < di)
    {
        x = (x - ci) / (di - ci);
        return cv * (1.0 - x) + dv * x;
    }

    return dv;
}


float minMaxKnob(float in, float dz=constants::DEFAULT_KNOB_DEADZONE)
{
    in = in - (dz * 0.5);
    in = in * (1.0 + dz);
    return std::min(1.0f, std::max(0.0f, in));
}


float minMaxSlider(float in, float dz=0.002)
{
    return minMaxKnob(in, dz);
}

float spreadTaps(float x, float s, float e=constants::SPREAD_EXPONENT)
{
    s = clamp(s, 0.0, 1.0);
    if (s > 0.5)
    {
        s = (s - 0.5) * 2.0;
        s = s*e+1.0;
        return 1.0 - pow(1.0-x, s);
    }

    if (s < 0.5)
    {
        s = 1.0 - (s * 2.0);
        s = s*e+1.0;
        return pow(x, s);
    }

    return x;
}

// This was previously putting both channels at ~0.67 at center, which screws up
// the noon-unity feedback scaling we need to allow users to "freeze" the delay
void panToVolume(float pan, float *left, float *right)
{
	// Balance-style panning: center = both at 1.0
	// Panning reduces the opposite channel sinusoidally
	if (pan >= 0.5f) {
		// Panning right: reduce right channel, left stays at 1.0
		float t = (pan - 0.5f) * 2.0f;  // 0 at center, 1 at hard right
		*left = 1.0f;
		*right = cosf(t * HALF_PI);      // 1 at center, 0 at hard right
	} else {
		// Panning left: reduce left channel, right stays at 1.0
		float t = (0.5f - pan) * 2.0f;  // 0 at center, 1 at hard left
		*left = cosf(t * HALF_PI);       // 1 at center, 0 at hard left
		*right = 1.0f;
	}
}


class PreciseSlew
{
  public:
    PreciseSlew() {}
    ~PreciseSlew() {}

    void Init(float sample_rate, float htime)
    {
        lastVal  = 0;
        prvhtim_ = constants::SLEW_COEF_UNINITIALIZED;
        htime_   = htime;
        sample_rate_ = sample_rate;
        onedsr_      = 1.0 / sample_rate_;
    }

    float Process(float in)
    {
        if (prvhtim_ != htime_)
        {
            c2_      = pow(0.5, onedsr_ / htime_);
            c1_      = 1.0 - c2_;
            prvhtim_ = htime_;
        }

        return lastVal = c1_ * in + c2_ * lastVal;
    }

    inline void SetHtime(float htime)
    {
        htime_ = htime;
    }

    inline float GetHtime()
    {
        return htime_;
    }

    float htime_;
    float c1_, c2_, lastVal, prvhtim_;
    float sample_rate_, onedsr_;
};


class Slew
{
public:
    double lastVal = 0.0;
    double coef = constants::SLEW_DEFAULT_COEF;
    double noiseFloor = 0.0;
    double noiseCoef = 0.0;
    int settleSamples = 0;
    int settleSamplesThreshold = constants::SLEW_SETTLE_SAMPLES;

    void Init(double coef = constants::SLEW_DEFAULT_COEF, double nf = 0.0)
    {
        this->coef = coef;
        this->noiseFloor = nf;
    }

    float Process(float x)
    {
        double c = coef;
        double d = (x - lastVal);

        // if we've set a noise floor
        if (noiseFloor > 0.0)
        {
            // if we're under the noise floor
            if (abs(d) < noiseFloor)
            {
                if (settleSamples < settleSamplesThreshold)
                {
                    // if the input needs to settle, keep sampling
                    settleSamples++;
                }
                else
                {
                    // if the input is done settling don't change the value
                    d = 0.0;
                }
            }
            else
            {
                // We're over the noise floor so reset the settle wait time
                settleSamples = 0;
            }
        }

        lastVal = lastVal + d * c;
        return lastVal;
    }
};


// Asymmetric slew: different rates for rising vs falling values
// Useful for dynamics processing where attack and release should differ
class AsymmetricSlew
{
public:
    float lastVal = 0.0f;
    float attackCoef = 1.0f;    // Coefficient when value is falling (instant by default)
    float releaseCoef = 0.001f; // Coefficient when value is rising (slow by default)

    void Init(float attack, float release, float initialVal = 1.0f)
    {
        attackCoef = attack;
        releaseCoef = release;
        lastVal = initialVal;
    }

    float Process(float x)
    {
        float d = x - lastVal;
        // Attack = value falling (e.g., gain reduction kicking in)
        // Release = value rising (e.g., gain returning to normal)
        float coef = (d < 0.0f) ? attackCoef : releaseCoef;
        lastVal = lastVal + d * coef;
        return lastVal;
    }
};


class ContSchmidt
{
public:
    float val = 0.0;

    float Process(float x, float h = constants::SCHMIDT_HYSTERESIS)
    {
        float i, f;
        float sign = x < 0.0 ? -1.0 : 1.0;
        x = abs(x);
	    f = modf(x, &i);
        if (f < h) val = i;
        if (f > 1.0 - h) val = i + 1;
        return val * sign;
    }
};


class UltraSlowDCBlocker
{
public:
    Slew slew;

    void Init(float coef = constants::DC_BLOCKER_COEF)
    {
        slew.Init(coef);
    }

    float Process(float x)
    {
        return x - slew.Process(x);
    }
};


// Asymmetric envelope follower for loudness detection.
// Fast attack to respond before peaks arrive (when combined with lookahead),
// slow release to avoid distortion and mainting "memory" of encountered loudness
class LoudnessDetector
{
public:
    float envelope = 0.0f;
    float attackCoef = 0.0f;
    float releaseCoef = 0.0f;

    void Init(float sampleRate)
    {
        envelope = 0.0f;
        // Attack: ~3ms
        attackCoef = 1.0f - expf(-1000.0f / (constants::LOUDNESS_ATTACK_MS * sampleRate));
        // Release: ~75ms
        releaseCoef = 1.0f - expf(-1000.0f / (constants::LOUDNESS_RELEASE_MS * sampleRate));
    }

    float Get() const { return envelope; }

    // Process current sample and optional lookahead peak
    // Returns the input unchanged (for chaining), updates internal envelope
    float Process(float current, float lookaheadPeak = 0.0f)
    {
        // Use max of current sample and lookahead peak
        float peak = fmaxf(fabsf(current), lookaheadPeak);

        if (peak > envelope)
        {
            // Fast attack
            envelope += (peak - envelope) * attackCoef;
        }
        else
        {
            // Slow release
            envelope += (peak - envelope) * releaseCoef;
        }

        return current;
    }
};

// StereoTap: Cache-optimized stereo delay tap
// TODO: re-add blur/jitter
class StereoTap
{
public:
    LoudnessDetector loudness;  // For LED display (max of L/R)
    float sampleRate;
    int bufferSize;  // Number of stereo pairs (actual buffer is 2x this)

    // Shared timing state (delay is same for L/R)
    float delayA = 0.0f;
    float delayB = 0.0f;
    float targetDelay = -1.0f;

    // Per-channel amplitude for pan
    float ampL_A = 0.0f;
    float ampL_B = 0.0f;
    float targetAmpL = -1.0f;

    float ampR_A = 0.0f;
    float ampR_B = 0.0f;
    float targetAmpR = -1.0f;

    // Current effective amplitude (updated each Process call)
    // Used for ampCoefSum and loudness calculations
    float currentAmpL = 0.0f;
    float currentAmpR = 0.0f;

    float phase = 1.0f;
    float delta;
    float blurAmount = 0.0f;

    void Init(float sampleRate, int bufferSize)
    {
        this->sampleRate = sampleRate;
        this->bufferSize = bufferSize;
        this->delta = constants::CROSSFADE_RATE / sampleRate;
        loudness.Init(sampleRate);
    }

    void Set(float delay, float ampL, float ampR, float blur = 0.0f)
    {
        this->targetDelay = delay;
        this->targetAmpL = ampL;
        this->targetAmpR = ampR;
        this->blurAmount = blur;
    }

    // Peek ahead in the buffer to predict future output for this tap.
    float GetLookaheadPeak(float* buffer, int readPos, int writePos)
    {
        // How far ahead can we look? Limited by distance to write head.
        // availableLookahead = distance from read position to write position
        int availableLookahead = wrap_buffer_index(writePos - readPos, bufferSize);

        // Clamp to our maximum lookahead window and ensure we don't read past write head
        int lookaheadSamples = std::min(constants::LOOKAHEAD_SAMPLES, availableLookahead - 1);

        if (lookaheadSamples <= 0)
        {
            // Very short delay or tap at write head, no lookahead possible
            return 0.0f;
        }

        // Sample N points across the lookahead window
        // This gives us a good estimate of the peak while being (hopefully) cache-friendly?
        float peak = 0.0f;
        int step = std::max(1, lookaheadSamples / constants::LOOKAHEAD_SAMPLE_POINTS);

        for (int i = step; i <= lookaheadSamples; i += step)
        {
            int pos = wrap_buffer_index(readPos + i, bufferSize);
            // Read both L and R at this position
            float absL = fabsf(buffer[pos * 2]);
            float absR = fabsf(buffer[pos * 2 + 1]);
            float localPeak = fmaxf(absL, absR);
            if (localPeak > peak) peak = localPeak;
        }

        return peak;
    }

    // Process both L/R channels together from interleaved buffer
    // buffer layout: [L0][R0][L1][R1]... (2 floats per stereo sample)
    // Returns amplified L/R outputs, also provides unamplified for
    // feedback (IDK how we want to handle tap8 feedback, it's so weird
    // and not something that I would have added to Time Machine, though
    // think I get why)
    void Process(float* buffer, int writePos,
                 float* outL, float* outR,
                 float* unamplifiedL, float* unamplifiedR)
    {
        // Check if we need to start a new crossfade
        if (phase >= 1.0f && (targetDelay >= 0.0f || targetAmpL >= 0.0f || targetAmpR >= 0.0f))
        {
            // Shift B values to A
            delayA = delayB;
            delayB = targetDelay;
            targetDelay = -1.0f;

            ampL_A = ampL_B;
            ampL_B = targetAmpL;
            targetAmpL = -1.0f;

            ampR_A = ampR_B;
            ampR_B = targetAmpR;
            targetAmpR = -1.0f;

            phase = 0.0f;
            delta = (constants::CROSSFADE_RATE + daisy::Random::GetFloat(-blurAmount, blurAmount)) / sampleRate;
        }

        // Calculate buffer positions for crossfade endpoints
        // Same position for L/R (they share the delay time)
        int posA = wrap_buffer_index(writePos - seconds_to_samples(delayA, sampleRate), bufferSize);
        int posB = wrap_buffer_index(writePos - seconds_to_samples(delayB, sampleRate), bufferSize);

        // Read L/R together
        // same cache line with interleaved buffer
        // buffer[pos*2] = Left, buffer[pos*2+1] = Right
        float rawL_A = buffer[posA * 2];
        float rawR_A = buffer[posA * 2 + 1];
        float rawL_B = buffer[posB * 2];
        float rawR_B = buffer[posB * 2 + 1];

        // Crossfade between A and B positions
        float oneMinusPhase = 1.0f - phase;
        float rawL = oneMinusPhase * rawL_A + phase * rawL_B;
        float rawR = oneMinusPhase * rawR_A + phase * rawR_B;

        // Provide unamplified output for feedback path
        *unamplifiedL = rawL;
        *unamplifiedR = rawR;

        // Crossfade amplitudes
        float ampL = oneMinusPhase * ampL_A + phase * ampL_B;
        float ampR = oneMinusPhase * ampR_A + phase * ampR_B;

        // Store current effective amplitude for external use (ampCoefSum, etc.)
        currentAmpL = ampL;
        currentAmpR = ampR;

        // Amplified output
        *outL = rawL * ampL;
        *outR = rawR * ampR;

        // Advance phase
        if (phase < 1.0f) phase += delta;
        if (phase > 1.0f) phase = 1.0f;

        // Lookahead: peek ahead in buffer to predict future output
        // Use target delay position (posB) since that's where we're transitioning to
        float lookaheadPeak = GetLookaheadPeak(buffer, posB, writePos);

        // Track loudness with lookahead for predictive dynamics
        float maxAmp = fmaxf(ampL, ampR);
        float scaledCurrent = fmaxf(fabsf(rawL), fabsf(rawR)) * maxAmp;
        float scaledLookahead = lookaheadPeak * maxAmp;
        loudness.Process(scaledCurrent, scaledLookahead);
    }
};


//=============================================================================
// SVF: Clean linear state variable filter
// Unity gain response, no saturation, well-behaved at all levels
// Resonance range 0-1 where 1 is just below self-oscillation
//=============================================================================
class SVF {
public:
    float lp = 0.0f;
    float bp = 0.0f;
    float hp = 0.0f;

    void Init(float sampleRate) {
        _sampleRate = sampleRate;
        _nyquist = sampleRate * 0.5f;
        Reset();
    }

    void Reset() {
        _ic1eq = 0.0f;
        _ic2eq = 0.0f;
        lp = bp = hp = 0.0f;
    }

    void SetCutoff(float freqHz) {
        _cutoffHz = clamp(freqHz, 20.0f, _nyquist * 0.98f);
        UpdateCoeffs();
    }

    void SetResonance(float res) {
        // 0-1 range: 0=no resonance, 1=just below self-oscillation
        _resonance = clamp(res, 0.0f, 1.0f);
        UpdateCoeffs();
    }

    void SetParams(float freqHz, float res) {
        _cutoffHz = clamp(freqHz, 20.0f, _nyquist * 0.98f);
        _resonance = clamp(res, 0.0f, 1.0f);
        UpdateCoeffs();
    }

    void Process(float input) {
        // Standard SVF topology (Hal Chamberlin / Andrew Simper style)
        float v3 = input - _ic2eq;
        float v1 = _a1 * _ic1eq + _a2 * v3;
        float v2 = _ic2eq + _a2 * _ic1eq + _a3 * v3;

        // Linear state update (no saturation)
        _ic1eq = 2.0f * v1 - _ic1eq;
        _ic2eq = 2.0f * v2 - _ic2eq;

        lp = v2;
        bp = v1;
        hp = input - _k * v1 - v2;
    }

    float GetCutoff() const { return _cutoffHz; }
    float GetResonance() const { return _resonance; }

private:
    float _sampleRate = 48000.0f;
    float _nyquist = 24000.0f;

    float _ic1eq = 0.0f;
    float _ic2eq = 0.0f;

    float _cutoffHz = 1000.0f;
    float _resonance = 0.0f;

    // Precomputed coefficients
    float _g = 0.0f;
    float _k = 2.0f;
    float _a1 = 0.0f;
    float _a2 = 0.0f;
    float _a3 = 0.0f;

    void UpdateCoeffs() {
        float normalized = 3.14159265f * _cutoffHz / _sampleRate;
        _g = fastTan(normalized);

        // k controls resonance: k=2 is no resonance, k=0 is self-oscillation
        // Map resonance 0-1 to k 2-0.01 (never quite reach 0 to stay stable)
        _k = 2.0f - _resonance * 1.99f;

        _a1 = 1.0f / (1.0f + _g * (_g + _k));
        _a2 = _g * _a1;
        _a3 = _g * _a2;
    }

    // Fast tan approximation for filter coefficient calculation
    static float fastTan(float x) {
        float x2 = x * x;
        return x * (1.0f + x2 * (0.333333333f + x2 * (0.133333333f + x2 * 0.053968254f)));
    }
};

// First-class stereo implementation of Time Machine. Uses stereo read heads to optimize
// cache accesses and do efficent lookahead for predictive dynamics adjustment. It's still
// pretty hard on the CPU and rough with the cache, but we are able to get a lot more out
// of the Daisy this way. We still need to add "blur" back in, probably with some per-read-head
// delays and perhaps some allpass filters to add a certain amount of per-read-head jitter
class StereoTimeMachine
{
public:
    // Single interleaved buffer pointer (layout: [L0][R0][L1][R1]...)
    float* buffer;
    int bufferSize;  // Number of stereo sample pairs
    int writePos;
    float sampleRate;

    // 9 StereoTaps: tap[0] = dry, tap[1-8] = delay taps
    StereoTap taps[9];

    // Input loudness detector (for dry LED)
    LoudnessDetector inputLoudness;

    // Per-channel SVF filters (allows stereo width variation)
    // Using clean linear SVF for transparent filtering
    SVF lowpassL, lowpassR;
    SVF highpassL, highpassR;
    SVF fbLowpassL, fbLowpassR;
    SVF fbHighpassL, fbHighpassR;

    // DC blockers
    UltraSlowDCBlocker fbDcblkL, fbDcblkR;

    // Slew filters for smooth parameter changes
    Slew feedbackSlew;
    AsymmetricSlew ampCoefSlew;

    // Parameters
    float feedback;
    bool isLastTapFeedback;
    bool isPreFilter;

    // Dynamics processor for output limiting
    DynamicsProcessor dynamics;

    // Feedback dynamics tracking (for logging/debugging)
    float lastUserFeedback = 0.0f;
    float lastEffectiveFeedback = 0.0f;
    float lastPredictedPeak = 0.0f;
    float lastAmpCoef = 1.0f;
    float lastWetGain = 1.0f;

    // Output min/max tracking (pre-clamp, for verifying dynamics processing)
    float outputMinL = 0.0f;
    float outputMaxL = 0.0f;
    float outputMinR = 0.0f;
    float outputMaxR = 0.0f;

    // Output buffer
    float outputs[2];

    void Init(float sampleRate, float maxDelay, float* interleavedBuffer)
    {
        this->sampleRate = sampleRate;
        this->bufferSize = seconds_to_samples(maxDelay, sampleRate);
        this->buffer = interleavedBuffer;
        this->writePos = 0;

        // Clear interleaved buffer (2 floats per stereo sample)
        for (int i = 0; i < bufferSize * 2; i++)
        {
            buffer[i] = 0.0f;
        }

        // Initialize all 9 taps
        for (int i = 0; i < 9; i++)
        {
            taps[i].Init(sampleRate, bufferSize);
        }

        // Initialize loudness detector for input/dry LED
        inputLoudness.Init(sampleRate);

        // Initialize SVF filters
        lowpassL.Init(sampleRate);
        lowpassR.Init(sampleRate);
        highpassL.Init(sampleRate);
        highpassR.Init(sampleRate);

        fbLowpassL.Init(sampleRate);
        fbLowpassR.Init(sampleRate);
        fbHighpassL.Init(sampleRate);
        fbHighpassR.Init(sampleRate);

        // Set initial filter parameters (will be updated by Set())
        // Resonance adds tighter rolloff without obvious peaking
        lowpassL.SetParams(20000.0f, constants::FILTER_RESONANCE);
        lowpassR.SetParams(20000.0f, constants::FILTER_RESONANCE);
        highpassL.SetParams(20.0f, constants::FILTER_RESONANCE);
        highpassR.SetParams(20.0f, constants::FILTER_RESONANCE);

        fbLowpassL.SetParams(20000.0f, constants::FILTER_RESONANCE);
        fbLowpassR.SetParams(20000.0f, constants::FILTER_RESONANCE);
        fbHighpassL.SetParams(20.0f, constants::FILTER_RESONANCE);
        fbHighpassR.SetParams(20.0f, constants::FILTER_RESONANCE);

        // Initialize DC blockers
        fbDcblkL.Init();
        fbDcblkR.Init();

        // Initialize slews
        feedbackSlew.Init(constants::SLEW_FEEDBACK_COEF);
        ampCoefSlew.Init(constants::SLEW_AMP_ATTACK, constants::SLEW_AMP_RELEASE);

        // Initialize dynamics
        dynamics.Init(sampleRate);

        // Default parameters
        feedback = 0.0f;
        isLastTapFeedback = false;
        isPreFilter = false;
    }

    // Set global parameters (called once per block, not per tap)
    void Set(float feedback, float highFc, float lowFc, bool isLastTapFeedback, bool isPreFilter)
    {
        this->feedback = feedback;
        this->isLastTapFeedback = isLastTapFeedback;
        this->isPreFilter = isPreFilter;

        // Exponential frequency mapping: 0-1 knob range maps to 20Hz - 22kHz
        // This gives musical octave-based scaling (each equal knob increment doubles frequency)
        float minFreq = 20.0f;
        float maxFreq = 22000.0f;  // Close to Nyquist (24kHz at 48kHz sample rate)
        float freqRatio = maxFreq / minFreq;
        float lowpassHz = minFreq * powf(freqRatio, lowFc);
        float highpassHz = minFreq * powf(freqRatio, highFc);

        // Set filter frequencies (same for L/R)
        lowpassL.SetCutoff(lowpassHz);
        lowpassR.SetCutoff(lowpassHz);
        highpassL.SetCutoff(highpassHz);
        highpassR.SetCutoff(highpassHz);
        fbLowpassL.SetCutoff(lowpassHz);
        fbLowpassR.SetCutoff(lowpassHz);
        fbHighpassL.SetCutoff(highpassHz);
        fbHighpassR.SetCutoff(highpassHz);
    }

    // Set dry tap (tap 0)
    void SetDryTap(float ampL, float ampR)
    {
        taps[0].Set(0.0f, ampL, ampR, 0.0f);
    }

    // Set delay tap (taps 1-8)
    void SetDelayTap(int tapIndex, float delay, float ampL, float ampR, float blur)
    {
        if (tapIndex >= 1 && tapIndex <= 8)
        {
            taps[tapIndex].Set(delay, ampL, ampR, blur);
        }
    }

    // Get loudness for LED display
    // tap 0 = dry (input loudness), taps 1-8 = delay tap loudness
    float GetTapLoudness(int tapIndex)
    {
        if (tapIndex == 0)
        {
            return inputLoudness.Get();
        }
        if (tapIndex >= 1 && tapIndex <= 8)
        {
            return taps[tapIndex].loudness.Get();
        }
        return 0.0f;
    }

    // Get the maximum predicted loudness across all delay taps (for feedback ceiling)
    float GetMaxPredictedLoudness()
    {
        float maxLoudness = 0.0f;
        for (int i = 1; i <= 8; i++)
        {
            float tapLoudness = taps[i].loudness.Get();
            if (tapLoudness > maxLoudness) maxLoudness = tapLoudness;
        }
        return maxLoudness;
    }

    // Get dynamics info for logging/debugging
    // Returns: userFeedback, effectiveFeedback, predictedPeak, ampCoef, wetGain
    void GetDynamicsInfo(float* userFb, float* effectiveFb, float* peak, float* ampCoef, float* wetGain)
    {
        *userFb = lastUserFeedback;
        *effectiveFb = lastEffectiveFeedback;
        *peak = lastPredictedPeak;
        *ampCoef = lastAmpCoef;
        *wetGain = lastWetGain;
    }

    // Get output min/max (pre-clamp values) for verifying dynamics processing
    // Values > 1.0 or < -1.0 indicate clipping would occur without dynamics
    void GetOutputMinMax(float* minL, float* maxL, float* minR, float* maxR)
    {
        *minL = outputMinL;
        *maxL = outputMaxL;
        *minR = outputMinR;
        *maxR = outputMaxR;
    }

    // Reset output min/max tracking (call periodically to get fresh readings)
    void ResetOutputMinMax()
    {
        outputMinL = 0.0f;
        outputMaxL = 0.0f;
        outputMinR = 0.0f;
        outputMaxR = 0.0f;
    }

    float* Process(float inLeft, float inRight)
    {
        // === 1. Write input to interleaved buffer ===
        buffer[writePos * 2] = inLeft;
        buffer[writePos * 2 + 1] = inRight;

        // Track input loudness for dry LED
        inputLoudness.Process(fmaxf(fabsf(inLeft), fabsf(inRight)));

        // === 2. Process dry tap (tap 0) ===
        float dryL, dryR, dryRawL, dryRawR;
        taps[0].Process(buffer, writePos, &dryL, &dryR, &dryRawL, &dryRawR);

        // Process dry envelope for ducking
        float actualDryPeak = fmaxf(fabsf(dryL), fabsf(dryR));
        dynamics.ProcessDryInput(actualDryPeak);

        // === 3. Process all delay taps (taps 1-8), sum wet signal ===
        float wetL = 0.0f;
        float wetR = 0.0f;
        float lastTapL = 0.0f;
        float lastTapR = 0.0f;
        float ampCoefSum = 0.0f;

        for (int i = 1; i <= 8; i++)
        {
            float tapL, tapR, rawL, rawR;
            taps[i].Process(buffer, writePos, &tapL, &tapR, &rawL, &rawR);
            wetL += tapL;
            wetR += tapR;

            // Keep track of last tap for feedback
            if (i == 8)
            {
                lastTapL = rawL;
                lastTapR = rawR;
            }

            // Sum current amplitudes for amp coefficient (pan * level, interpolated during crossfade)
            ampCoefSum += taps[i].currentAmpL + taps[i].currentAmpR;
        }

        // Calculate amplitude coefficient to prevent sum from exceeding reasonable levels
        float ampCoef = ampCoefSlew.Process(1.0f / std::max(1.0f, ampCoefSum * 0.5f));

        // === 4. Apply filters and calculate feedback ===
        float fbL, fbR;

        if (isPreFilter)
        {
            // Highpass then lowpass on wet signal
            highpassL.Process(wetL);
            highpassR.Process(wetR);
            wetL = highpassL.hp;
            wetR = highpassR.hp;

            lowpassL.Process(wetL);
            lowpassR.Process(wetR);
            wetL = lowpassL.lp;
            wetR = lowpassR.lp;

            if (isLastTapFeedback)
            {
                // Separate filtering for last tap feedback
                fbL = fbDcblkL.Process(lastTapL);
                fbR = fbDcblkR.Process(lastTapR);

                fbHighpassL.Process(fbL);
                fbHighpassR.Process(fbR);
                fbL = fbHighpassL.hp;
                fbR = fbHighpassR.hp;

                fbLowpassL.Process(fbL);
                fbLowpassR.Process(fbR);
                fbL = fbLowpassL.lp;
                fbR = fbLowpassR.lp;
            }
            else
            {
                fbL = wetL;
                fbR = wetR;
            }
        }
        else
        {
            if (isLastTapFeedback)
            {
                fbL = lastTapL;
                fbR = lastTapR;
            }
            else
            {
                fbL = wetL;
                fbR = wetR;
            }

            // Filter the feedback path
            fbL = fbDcblkL.Process(fbL);
            fbR = fbDcblkR.Process(fbR);

            fbHighpassL.Process(fbL);
            fbHighpassR.Process(fbR);
            fbL = fbHighpassL.hp;
            fbR = fbHighpassR.hp;

            fbLowpassL.Process(fbL);
            fbLowpassR.Process(fbR);
            fbL = fbLowpassL.lp;
            fbR = fbLowpassR.lp;
        }

        // === 5. Apply feedback with dynamic ceiling ===
        float predictedPeak = GetMaxPredictedLoudness();
        float userFeedback = feedbackSlew.Process(feedback);

        float effectiveFeedback;
        if (predictedPeak > 0.001f)
        {
            float feedbackCeiling = constants::FB_TARGET_LEVEL / predictedPeak;
            effectiveFeedback = std::min(userFeedback, feedbackCeiling);
        }
        else
        {
            effectiveFeedback = userFeedback;
        }

        float fbGain = effectiveFeedback * ampCoef;

        // Track for logging
        lastUserFeedback = userFeedback;
        lastEffectiveFeedback = effectiveFeedback;
        lastPredictedPeak = predictedPeak;

        // Write feedback back to buffer (phase inverted)
        buffer[writePos * 2] = -(buffer[writePos * 2] + fbL * fbGain);
        buffer[writePos * 2 + 1] = -(buffer[writePos * 2 + 1] + fbR * fbGain);

        // Advance write position
        writePos = wrap_buffer_index(writePos + 1, bufferSize);

        // Final wet gain step
        float wetPeak = fmaxf(fabsf(wetL), fabsf(wetR));
        float gain = dynamics.GetWetGain(wetPeak);

        // Track for logging
        lastAmpCoef = ampCoef;
        lastWetGain = gain;

        wetL *= gain;
        wetR *= gain;

        // Final output: wet + dry
        float outL = wetL + dryL;
        float outR = wetR + dryR;

        // Track min/max before clamping (for verifying dynamics processing)
        if (outL < outputMinL) outputMinL = outL;
        if (outL > outputMaxL) outputMaxL = outL;
        if (outR < outputMinR) outputMinR = outR;
        if (outR > outputMaxR) outputMaxR = outR;

        outputs[0] = clamp(outL, -1.0f, 1.0f);
        outputs[1] = clamp(outR, -1.0f, 1.0f);

        return outputs;
    }
};


class ClockRateDetector
{
public:
    int samplesSinceLastClock;
    int lastIntervalInSamples;
    bool lastVal;
    float sampleRate;

    ClockRateDetector()
    {
        samplesSinceLastClock = 0;
        lastIntervalInSamples = 0;
        lastVal = false;
    }

    void Init(int sr)
    {
        sampleRate = sr;
    }

    bool isStale()
    {
        return samplesSinceLastClock > sampleRate * 2;
    }

    float GetInterval()
    {
        float interval = lastIntervalInSamples / sampleRate;
        return isStale() ? 0.0 : interval;
    }

    void Process(bool triggered)
    {
        if(triggered && (lastVal != triggered))
        {
            if (isStale())
            {
                lastIntervalInSamples = samplesSinceLastClock;
            }
            else
            {
                lastIntervalInSamples = (lastIntervalInSamples + samplesSinceLastClock) * 0.5;
            }

            samplesSinceLastClock = 0;
        }
        else
        {
            samplesSinceLastClock++;
        }

        lastVal = triggered;
    }
};

