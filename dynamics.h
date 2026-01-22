#pragma once

#include "constants.h"
#include <cmath>
#include <algorithm>

// Asymmetric envelope follower with configurable attack and release.
// Used to track signal levels for dynamics processing.
// Attack/release of 0ms = instant (coefficient of 1.0)
class EnvelopeFollower
{
public:
    float envelope;
    float attackCoef;
    float releaseCoef;

    EnvelopeFollower() : envelope(0.0f), attackCoef(1.0f), releaseCoef(0.0f) {}

    void Init(float sampleRate, float attackMs, float releaseMs)
    {
        envelope = 0.0f;

        // Attack coefficient: 0ms = instant (coefficient of 1.0)
        if (attackMs > 0.0f)
        {
            attackCoef = 1.0f - expf(-1000.0f / (attackMs * sampleRate));
        }
        else
        {
            attackCoef = 1.0f; // Instant attack
        }

        // Release coefficient: 0ms = instant (coefficient of 1.0)
        if (releaseMs > 0.0f)
        {
            releaseCoef = 1.0f - expf(-1000.0f / (releaseMs * sampleRate));
        }
        else
        {
            releaseCoef = 1.0f; // Instant release
        }
    }

    float Process(float input)
    {
        float rectified = fabsf(input);

        if (rectified > envelope)
        {
            // Attack
            envelope += (rectified - envelope) * attackCoef;
        }
        else
        {
            // Release
            envelope += (rectified - envelope) * releaseCoef;
        }

        return envelope;
    }

    float Get() const { return envelope; }
};


// Dynamics processor for stereo-linked wet signal control.
// The feedback ceiling (Layer 2) is handled separately in
// TimeMachine using ReadHead lookahead.
class DynamicsProcessor
{
public:
    EnvelopeFollower dryEnvelope;
    float smoothedGain;
    float smoothingCoef;

    DynamicsProcessor()
        : smoothedGain(1.0f), smoothingCoef(0.0f)
    {
    }

    void Init(float sampleRate)
    {
        dryEnvelope.Init(sampleRate,
                         constants::DRY_ENV_ATTACK_MS,
                         constants::DRY_ENV_RELEASE_MS);

        smoothedGain = 1.0f;
        smoothingCoef = constants::GAIN_SMOOTHING_COEF;
    }

    // Call this with the actual dry contribution (input * dryAmp) each sample
    // Use max(|L|, |R|) for stereo-linked processing
    void ProcessDryInput(float actualDryPeak)
    {
        dryEnvelope.Process(actualDryPeak);
    }

    // Get the gain to apply to wet signal, given its peak level
    // Call with max(|wetL|, |wetR|) to get stereo-linked gain
    float GetWetGain(float wetPeak)
    {
        float dryLevel = dryEnvelope.Get();

        // Calculate how much headroom is available after dry signal
        float availableHeadroom = 1.0f - dryLevel;

        float targetGain;

        if (wetPeak < 0.001f)
        {
            // Wet is silent, no gain reduction needed
            targetGain = 1.0f;
        }
        else if (availableHeadroom <= 0.0f)
        {
            // Crush wet if dry is super loud
            targetGain = 0.0f;
        }
        else
        {
            // Calculate gain to fit wet within available headroom
            targetGain = availableHeadroom / wetPeak;
            if (targetGain > 1.0f) targetGain = 1.0f;
        }

        // Smooth the gain to prevent clicks
        // Fast attack (can reduce gain quickly), slower release
        if (targetGain < smoothedGain)
        {
            // Instant sttack
            smoothedGain = targetGain;
        }
        else
        {
            // Smooth release
            smoothedGain += (targetGain - smoothedGain) * smoothingCoef;
        }

        return smoothedGain;
    }
};
