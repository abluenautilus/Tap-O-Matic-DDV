#include "daisysp.h"
#include "biquad.h"

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


float mix(float x, float a, float b)
{
    return x*(1-x) + b*x;
}


float clamp(float x, float a, float b)
{
    return std::max(a,std::min(b,x));
}


float fourPointWarp(float x,
                    float ai=0.0,
                    float av=0.0,
                    float bi=0.45,
                    float bv=0.5,
                    float ci=0.55,
                    float cv=0.5,
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


float minMaxKnob(float in, float dz=0.002)
{
    in = in - (dz * 0.5);
    in = in * (1.0 + dz);
    return std::min(1.0f, std::max(0.0f, in));
}


float minMaxSlider(float in, float dz=0.002)
{
    return minMaxKnob(in, dz);
}


float softClip(float x, float kneeStart=0.9, float kneeCurve=5.0)
{
    float linPart = clamp(x, -kneeStart, kneeStart);
    float clipPart = x - linPart;
    clipPart = atan(clipPart * kneeCurve) /  kneeCurve;
    return linPart + clipPart;
}


float spreadTaps(float x, float s, float e=2.5)
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


void panToVolume(float pan, float *left, float *right)
{
	*right = pow( sin((1 - pan) * HALF_PI) , 1.5);
	*left = pow( sin(pan * HALF_PI), 1.5);
}


class PreciseSlew
{
  public:
    PreciseSlew() {}
    ~PreciseSlew() {}

    void Init(float sample_rate, float htime)
    {
        lastVal  = 0;
        prvhtim_ = -100.0;
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
    double coef = 0.001;
    double noiseFloor = 0.0;
    double noiseCoef = 0.0;
    int settleSamples = 0;
    int settleSamplesThreshold = 96;

    void Init(double coef = 0.001, double nf = 0.0)
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
                // We're over the noise floor - reset the settle wait time
                settleSamples = 0;
            }
        }

        lastVal = lastVal + d * c;
        return lastVal;
    }
};


class ContSchmidt
{
public:
    float val = 0.0;

    float Process(float x, float h = 0.333)
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

    void Init(float coef = 0.00001)
    {
        slew.Init(coef);
    }

    float Process(float x)
    {
        return x - slew.Process(x);
    }
};


class LoudnessDetector
{
public:
    Slew slew;
    float lastVal = 0;

    void Init() { slew.Init(); }
    float Get() { return this->lastVal; }

    float Process(float x)
    {
        lastVal = slew.Process(abs(x));
        return x;
    }
};


class Limiter {
public:
    float gainCoef;
    float attackCoef;
    float releaseCoef;

    void Init(float sampleRate)
    {
        gainCoef = 1;
        releaseCoef = 16.0 / sampleRate;
    }


    float Process(float in)
    {
        float targetGainCoef = 1.0 / std::max(abs(in), (float)1.0);

        if (targetGainCoef < gainCoef)
        {
            gainCoef = targetGainCoef;
        }
        else
        {
            gainCoef = gainCoef * (1.0 - releaseCoef) + targetGainCoef*releaseCoef;
        }

        //return in gainCoef;
        // Prevent rare case of output exceeding -1.0 or 1.0 due to smoothing
        float out = in * gainCoef;
        // Ensure output is always within [-1.0, 1.0]
        if(out > 1.0f) out = 1.0f;
        else if(out < -1.0f) out = -1.0f;
        return out;
    }
};


class ReadHead
{
public:
    LoudnessDetector loudness;
    float* buffer;
    int bufferSize;
    float delayA = 0.0;
    float delayB = 0.0;
    float targetDelay = -1;
    float ampA = 0.0;
    float ampB = 0.0;
    float targetAmp = -1;
	float sampleRate;
    float phase = 1.0;
    float delta;
    float blurAmount;

    void Init(float sampleRate, float* buffer, int bufferSize)
    {
        this->sampleRate = sampleRate;
        this->delta = 5.0 / sampleRate;
        this->buffer = buffer;
        this->bufferSize = bufferSize;
        this->blurAmount = 0.0;
    }

    void Set(float delay, float amp, float blur = 0)
    {
        this->targetDelay = delay;
        this->targetAmp = amp;
        this->blurAmount = blur;
    }

    float Process(float writeHeadPosition, float *unamplifiedOut)
    {
        if(phase >= 1.0 && (targetDelay >= 0.0 || targetAmp > 0.0))
        {
            delayA = delayB;
            delayB = targetDelay;
            targetDelay = -1.0;
            ampA = ampB;
            ampB = targetAmp;
            targetAmp = -1.0;
            phase = 0.0;
            delta = (5.0 + daisy::Random::GetFloat(-blurAmount, blurAmount)) / sampleRate;
        }

        float outputA = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayA, this->sampleRate), bufferSize)];
        float outputB = this->buffer[wrap_buffer_index(writeHeadPosition - seconds_to_samples(this->delayB, this->sampleRate), bufferSize)];
        float output = ((1 - phase) * outputA) + (phase * outputB);
        *unamplifiedOut = output;

        float outputAmp = ((1 - phase) * ampA) + (phase * ampB);
        phase = phase <= 1.0 ? phase + delta : 1.0;

        return loudness.Process(output) * outputAmp;
    }
};


class TimeMachine
{
public:
    ReadHead readHeads[8];
    LoudnessDetector loudness;
	float sampleRate;
    float* buffer;
    int bufferSize;
    int writeHeadPosition;
    float dryAmp;
    float feedback;
    float blur;
    Slew dryAmpSlew;
    Slew feedbackSlew;
    Slew ampCoefSlew;
    Slew blurSlew;
    Limiter outputLimiter;
    Limiter feedbackLimiter;
    UltraSlowDCBlocker dcblk;
    UltraSlowDCBlocker fbDcblk;
    daisysp::Compressor compressor;
    daisysp::Compressor fbCompressor;
    Biquad lowpass;
    Biquad highpass;
    Biquad fbLowpass;
    Biquad fbHighpass;
    bool isLastTapFeedback;
    bool isPreFilter;

    void Init(float sampleRate, float maxDelay, float* buffer)
    {
		this->sampleRate = sampleRate;
        this->bufferSize = seconds_to_samples(maxDelay, sampleRate);
        this->buffer = buffer;

        for(int i=0; i<bufferSize; i++) buffer[i] = 0;
        for(int i=0; i<8; i++) readHeads[i].Init(sampleRate, buffer, bufferSize);

        writeHeadPosition = 0;
        dryAmpSlew.Init();
        feedbackSlew.Init(0.01);
        ampCoefSlew.Init(0.0001);
        blurSlew.Init();
        outputLimiter.Init(sampleRate);
        feedbackLimiter.Init(sampleRate);
        loudness.Init();

        compressor.Init(sampleRate);
        compressor.SetAttack(0.02);
        compressor.SetRelease(0.2);
        compressor.SetRatio(5.0);
        compressor.SetThreshold(0.0);

        lowpass.setBiquad(bq_type_lowpass_1p1z, 0.5, 0.01, 0.0);
        highpass.setBiquad(bq_type_highpass_1p1z, 0.0, 0.6, 0.0);

        fbCompressor.Init(sampleRate);
        fbCompressor.SetAttack(0.02);
        fbCompressor.SetRelease(0.2);
        fbCompressor.SetRatio(5.0);
        fbCompressor.SetThreshold(0.0);

        fbLowpass.setBiquad(bq_type_lowpass_1p1z, 0.5, 0.01, 0.0);
        fbHighpass.setBiquad(bq_type_highpass_1p1z, 0.0, 0.6, 0.0);
    }

    void Set(float dryAmp, float feedback, float blur, float highFc, float lowFc, bool isLastTapFeedback, bool isPreFilter)
    {
        this->dryAmp = dryAmp;
        this->feedback = feedback;
        this->blur = blur;
        this->lowpass.setFc(lowFc);
        this->highpass.setFc(highFc);
        this->fbLowpass.setFc(lowFc);
        this->fbHighpass.setFc(highFc);
        this->isLastTapFeedback = isLastTapFeedback;
        this->isPreFilter = isPreFilter;
    }

    float Process(float in)
    {
        float out = 0.0;
        float ampCoef = 0.0;

        for (int i = 0; i < 8; i++)
        {
            ampCoef += readHeads[i].targetAmp;
        }

        ampCoef = ampCoefSlew.Process(1.0 / std::max(1.0f, ampCoef));

        buffer[writeHeadPosition] = in;
        loudness.Process(in);

        float lastTapOut;
        for (int i = 0; i < 8; i++)
        {
            out += readHeads[i].Process(writeHeadPosition, &lastTapOut);
        }

        float fbValue;

        if (isPreFilter)
        {
            out = dcblk.Process(out);
            out = highpass.process(out);
            out = lowpass.process(out);
            out = compressor.Process(out, buffer[writeHeadPosition] + out);

            if (isLastTapFeedback)
            {
                // We need to apply separate versions of all the conditioning processes because
                // we are not copying the whole of out.
                fbValue = lastTapOut;
                fbValue = fbDcblk.Process(fbValue);
                fbValue = fbHighpass.process(fbValue);
                fbValue = fbLowpass.process(fbValue);
                fbValue = fbCompressor.Process(fbValue, buffer[writeHeadPosition] + fbValue);
            }
            else
            {
                fbValue = out;
            }
        }
        else
        {
            out = dcblk.Process(out);
            out = compressor.Process(out, buffer[writeHeadPosition] + out);

            if (isLastTapFeedback)
            {
                fbValue = lastTapOut;
            }
            else
            {
                fbValue = out;
            }

            // Then we can condition the feedback value the same way regardless of isLastTapFeedback.
            fbValue = fbDcblk.Process(fbValue);
            fbValue = fbHighpass.process(fbValue);
            fbValue = fbLowpass.process(fbValue);
            fbValue = fbCompressor.Process(fbValue, buffer[writeHeadPosition] + fbValue);
        }

        buffer[writeHeadPosition] = -(feedbackLimiter.Process(buffer[writeHeadPosition] + (fbValue * feedbackSlew.Process(feedback) * ampCoef)));

        out = outputLimiter.Process(out + in * dryAmpSlew.Process(dryAmp));
        writeHeadPosition = wrap_buffer_index(writeHeadPosition + 1, bufferSize);

        return out;
    }
};


class StereoTimeMachine
{
public:
    TimeMachine timeMachineLeft;
    TimeMachine timeMachineRight;
    float outputs[2];

    void Init(float sampleRate, float maxDelay, float* bufferLeft, float* bufferRight)
    {
        timeMachineLeft.Init(sampleRate, maxDelay, bufferLeft);
        timeMachineRight.Init(sampleRate, maxDelay, bufferRight);
    }

    void Set(float dryAmp, float feedback, float blur, float highFc, float lowFc, bool isLastTapFeedback, bool isPreFilter)
    {
        timeMachineLeft.Set(dryAmp, feedback, blur, highFc, lowFc, isLastTapFeedback, isPreFilter);
        timeMachineRight.Set(dryAmp, feedback, blur, highFc, lowFc, isLastTapFeedback, isPreFilter);
    }

    float* Process(float inLeft, float inRight)
    {
        outputs[0] = timeMachineLeft.Process(inLeft);
        outputs[1] = timeMachineRight.Process(inRight);
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