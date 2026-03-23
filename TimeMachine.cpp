#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dsp.h"
#include "time_machine_hardware.h"
#include "constants.h"

using namespace daisy;
using namespace oam;
using namespace time_machine;
using namespace std;

#define N_TAPS 9
#define TIME_SECONDS 150
#define CALIBRATION_SAMPLES 100
#define BUFFER_WIGGLE_ROOM_SAMPLES 1000

// #define LOGGING_ENABLED
// #define CPU_DIAGNOSTICS

// ------------------- //
// --- ERIS: TO DO --- //
// ------------------- //
// [x] steal soft clip from DSnomia (done, not using)
// [x] design custom multi-head-read lookadhead compressor
// [x] steal SVF from DSnomia (done, but using normal SVF)
// [x] run diagnostics and offset profiling
// [x] optimize CPU usage and attempt to reduce block size to eliminate hf noise
// [ ] hook up to balanced outs and do noise profiling
// [ ] figure out how to 3v calibrate time knob (and filter knobs?)
// [ ] make sure compressor starts kicking in before 0dB
// [ ] calibrate panning knobs

TimeMachineHardware hw;

// Setting Struct containing parameters we want to save to flash
// IMPORTANT: version field is at the where it is to maintain backwards compatibility
// with old calibration data that didn't have a version field. Old data will
// have garbage in the version field, which we detect and handle by resetting.
struct CvCalibrationData
{
	static constexpr uint32_t CURRENT_VERSION = 1;

	// Keep these fields in original order for backwards compatibility
	float timeCvOffset;
	float spreadCvOffset;
	float feedbackCvOffset;
	float lowpassCvOffset;
	float highpassCvOffset;
	float levelsCvOffset[N_TAPS];

	// Version field at here to not shift existing field offsets
	uint32_t version;

	// Additional fields go here so as to not throw off version field/struct ordering

	void Init()
	{
		timeCvOffset = 0.0;
		spreadCvOffset = 0.0;
		feedbackCvOffset = 0.0;
		lowpassCvOffset = 0.0;
		highpassCvOffset = 0.0;

		for (int i = 0 ; i < N_TAPS; i++)
		{
			levelsCvOffset[i] = 0.0;
		}

		version = CURRENT_VERSION;
	}

	bool IsValid() const
	{
		return version == CURRENT_VERSION;
	}

	// Overloading the != operator
	// This is necessary as this operator is used in the PersistentStorage source code
	bool operator!=(const CvCalibrationData& a) const
	{
		if (a.version != version) 					return true;
		if (a.timeCvOffset != timeCvOffset) 		return true;
		if (a.spreadCvOffset != spreadCvOffset) 	return true;
		if (a.feedbackCvOffset != feedbackCvOffset) return true;
		if (a.lowpassCvOffset != lowpassCvOffset) 	return true;
		if (a.highpassCvOffset != highpassCvOffset) return true;

		for (int i = 0; i < N_TAPS; i++)
		{
			if (a.levelsCvOffset[i] != levelsCvOffset[i]) return true;
		}

		return false;
    }
};


// Gather together all the values and Slew objects we need to handle a control
// consisting of a knob/slider and a CV input (optional).
class ControlHandler
{
public:
	char name[10];

	float cv;
	float cvOffset;
	Slew cvSlew;

	float knob;
	Slew knobSlew;

	float value;

	ControlHandler(const char *dumpName = "")
	{
		strncpy(name, dumpName, 9);

		cv = 0.0;
		cvSlew.Init(constants::SLEW_CV_COEF, constants::CV_NOISE_FLOOR);
		cvOffset = 0.0;

		knob = 0.0;
		knobSlew.Init(constants::SLEW_KNOB_COEF, constants::KNOB_NOISE_FLOOR);

		value = 0.0;
	}

	void SetDumpName(const char *dumpName)
	{
		strncpy(name, dumpName, sizeof(name));
	}

	void Dump()
	{
		hw.PrintLine(
			"%-10s   value:" FLT_FMT(5) "   knob:" FLT_FMT(5) "   cv:" FLT_FMT(5) "   offset:" FLT_FMT(5),
			name,
			FLT_VAR(5, value),
			FLT_VAR(5, knob),
			FLT_VAR(5, cv),
			FLT_VAR(5, cvOffset));
	}
};


// Interleaved stereo buffer for cache-optimized SDRAM access
// Layout: [L0][R0][L1][R1]... L/R at same position share cache lines
// This reduces cache misses by ~2x compared to separate L/R buffers
// Size: (samples per channel) * 2 (for stereo interleaving)
float DSY_SDRAM_BSS buffer[(48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES) * 2]; // we should probably remove wiggle room? IDK, ain't broke don't fix it

ControlHandler time("time");
ControlHandler feedback("feedback");
ControlHandler spread("spread");
ControlHandler highpass("highpass");
ControlHandler lowpass("lowpass");
ControlHandler levels[N_TAPS];
ControlHandler pans[N_TAPS];

PersistentStorage<CvCalibrationData> CalibrationDataStorage(hw.qspi);
GateIn gate;
Led leds[N_TAPS];
GPIO feedbackModeSwitch;
GPIO filterPositionSwitch;

StereoTimeMachine timeMachine;
ClockRateDetector clockRateDetector;
ContSchmidt timeKnobSchmidt;
ContSchmidt timeCvSchmidt;

// delay setting LEDs for startup sequences
bool setLeds = false;

// Performance metrics.
CpuLoadMeter cpuMeter;
int droppedFrames = 0;


void updateControlHandlers()
{
	// condition feedback knob to have deadzone in the middle, add CV
	feedback.knob = fourPointWarp(1.0 - minMaxKnob(hw.GetFeedbackKnob(), constants::FEEDBACK_KNOB_DEADZONE));
	feedback.cv = clamp(hw.GetAdcValue(FEEDBACK_CV) - feedback.cvOffset, -1.0, 1.0);
	feedback.value = clamp(fourPointWarp(feedback.knobSlew.Process(feedback.knob)) * constants::FEEDBACK_GAIN_SCALE + feedback.cvSlew.Process(feedback.cv), 0, constants::FEEDBACK_MAX);

	// condition spread knob value to have deadzone in the middle, add CV
	spread.knob = fourPointWarp(1.0 - minMaxKnob(hw.GetSpreadKnob(), constants::SPREAD_KNOB_DEADZONE));
	spread.cv = clamp(hw.GetAdcValue(SPREAD_CV) - spread.cvOffset, -1.0, 1.0);
	spread.value = fourPointWarp(spread.knobSlew.Process(spread.knob)) + spread.cvSlew.Process(spread.cv);

	lowpass.knob = 1.0f - hw.GetLowpassKnob();
	lowpass.cv = clamp(hw.GetAdcValue(LOWPASS_CV) - lowpass.cvOffset, -1.0, 1.0);
	lowpass.value = clamp(lowpass.knobSlew.Process(lowpass.knob) + lowpass.cvSlew.Process(lowpass.cv), 0.0, 1.0);

	highpass.knob = 1.0f - hw.GetHighpassKnob();
	highpass.cv = clamp(hw.GetAdcValue(HIGHPASS_CV) - highpass.cvOffset, -1.0, 1.0);
	highpass.value = clamp(highpass.knobSlew.Process(highpass.knob) + highpass.cvSlew.Process(highpass.cv), 0.0, 1.0);

	// Handle the levels and pans.
	for (int i = 0; i < N_TAPS; i++)
	{
		ControlHandler &s = levels[i];
		s.knob = minMaxSlider(1.0 - hw.GetLevelSlider(i));
		s.cv = clamp((hw.GetLevelCV(i)) - s.cvOffset, -1.0, 1.0);
		s.value = clamp(s.knobSlew.Process(s.knob) + s.cvSlew.Process(s.cv), 0.0, 1.0);

		ControlHandler &p = pans[i];
		p.knob = hw.GetPanKnob(i);
		p.value = clamp(p.knobSlew.Process(p.knob), -1.0, 1.0);
	}

	// Handle the time stuff, which is a bit more involved.
	time.knob = minMaxKnob(1.0 - hw.GetTimeKnob(), constants::TIME_KNOB_DEADZONE);
	time.cv = clamp(hw.GetAdcValue(TIME_CV) - time.cvOffset, -1.0, 1.0);

	// calculate time based on clock if present, otherwise simple time
	time.value = 0.0;
	if (clockRateDetector.GetInterval() > 0.0)
	{
		// Quantized steps for clock sync: knob controls octave divisions, CV adds/subtracts octaves
		float timeCoef = pow(2.0, (timeKnobSchmidt.Process((1.0 - time.knob) * constants::CLOCK_KNOB_STEPS)) + (timeCvSchmidt.Process(time.cv * constants::CLOCK_CV_STEPS))) / constants::CLOCK_CENTER_DIVISOR;
		time.value = clockRateDetector.GetInterval() / timeCoef;

		// make sure time is a power of two less than the max time available in the buffer
		while(time.value > TIME_SECONDS) time.value *= 0.5;
	}
	else
	{
		// time quadratic with knob (finer control at short times), scaled v/oct style with CV
		time.value = pow(time.knobSlew.Process(time.knob), constants::TIME_KNOB_CURVE) * constants::TIME_MAX_SECONDS / pow(2.0, time.cvSlew.Process(time.cv) * constants::TIME_CV_OCTAVE_RANGE);
	}

	// force time down to a max value (taking whichever is lesser, the max or the time)
	time.value = std::min((float)TIME_SECONDS, time.value);
}


// called every N samples (search for SetAudioBlockSize)
void audioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
#ifdef CPU_DIAGNOSTICS
	// cpu meter measurements start
	cpuMeter.OnBlockStart();
	droppedFrames++;
#endif

	// process controls at the ADC level.
	hw.ProcessAllControls();

	// Then we can use those values when updating our handlers.
	updateControlHandlers();

	// Set LEDs based on loudness for all 9 taps
	// Tap 0 = dry (input), Taps 1-8 = delay taps
	if (setLeds)
	{
		for (int i = 0; i < N_TAPS; i++)
		{
			float loudness = timeMachine.GetTapLoudness(i);
			leds[i].Set(loudness);
			leds[i].Update();
		}
	}

	// Set global time machine parameters (feedback, filters, modes)
	timeMachine.Set(feedback.value, highpass.value, lowpass.value, feedbackModeSwitch.Read(), filterPositionSwitch.Read());

	// Set dry tap (tap 0) with pan
	float dryPanL, dryPanR;
	panToVolume(hw.GetPanKnob(0), &dryPanL, &dryPanR);
	timeMachine.SetDryTap(levels[0].value * dryPanL, levels[0].value * dryPanR);

	// Set delay taps (taps 1-8) with delay, pan, and blur
	for (int i = 1; i < N_TAPS; i++)
	{
		float tapPanL, tapPanR;
		panToVolume(hw.GetPanKnob(i), &tapPanL, &tapPanR);

		float delay = spreadTaps((i / (float)(N_TAPS - 1)), spread.value) * time.value;
		float blur = max(0.0f, feedback.value - 1.0f);

		timeMachine.SetDelayTap(i, delay, tapPanL * levels[i].value, tapPanR * levels[i].value, blur);
	}

	for (size_t i = 0; i < size; i++)
	{
		// process gate for clock rate detector at audio rate (per-sample) so it calculates clock correctly
		clockRateDetector.Process(hw.gate_in_1.State());

		// process input into time machine
		float* output = timeMachine.Process(in[0][i], in[1][i]);

		// set hardware output to time machine output
		out[0][i] = output[0];
		out[1][i] = output[1];
	}

#ifdef CPU_DIAGNOSTICS
	//cpu meter measurement stop
	cpuMeter.OnBlockEnd();
	droppedFrames--;
#endif
}


bool isCvCloseToZero(float val)
{
	return (val > -constants::CV_ZERO_THRESHOLD) && (val < constants::CV_ZERO_THRESHOLD);
}


bool shouldCalibrate()
{
	hw.PrintLine("=== Calibration Check ===");

	float gateState = hw.gate_in_1.State() ? 1.0f : 0.0f;
	hw.PrintLine("Gate: " FLT_FMT(3) " (need HIGH)", FLT_VAR(3, gateState));
	if (!hw.gate_in_1.State()) return false;

	// CV inputs need to be close to zero
	float spreadCv = hw.GetAdcValue(SPREAD_CV);
	float timeCv = hw.GetAdcValue(TIME_CV);
	float feedbackCv = hw.GetAdcValue(FEEDBACK_CV);
	float highpassCv = hw.GetAdcValue(HIGHPASS_CV);
	float lowpassCv = hw.GetAdcValue(LOWPASS_CV);
	float levelDryCv = hw.GetAdcValue(LEVEL_DRY_CV);

	hw.PrintLine("CV inputs (need ~0, threshold " FLT_FMT(3) "):", FLT_VAR(3, constants::CV_ZERO_THRESHOLD));
	hw.PrintLine("  Spread:   " FLT_FMT(4) " %s", FLT_VAR(4, spreadCv), isCvCloseToZero(spreadCv) ? "OK" : "FAIL");
	hw.PrintLine("  Time:     " FLT_FMT(4) " %s", FLT_VAR(4, timeCv), isCvCloseToZero(timeCv) ? "OK" : "FAIL");
	hw.PrintLine("  Feedback: " FLT_FMT(4) " %s", FLT_VAR(4, feedbackCv), isCvCloseToZero(feedbackCv) ? "OK" : "FAIL");
	hw.PrintLine("  Highpass: " FLT_FMT(4) " %s", FLT_VAR(4, highpassCv), isCvCloseToZero(highpassCv) ? "OK" : "FAIL");
	hw.PrintLine("  Lowpass:  " FLT_FMT(4) " %s", FLT_VAR(4, lowpassCv), isCvCloseToZero(lowpassCv) ? "OK" : "FAIL");
	hw.PrintLine("  LevelDry: " FLT_FMT(4) " %s", FLT_VAR(4, levelDryCv), isCvCloseToZero(levelDryCv) ? "OK" : "FAIL");

	if (!isCvCloseToZero(spreadCv)) return false;
	if (!isCvCloseToZero(timeCv)) return false;
	if (!isCvCloseToZero(feedbackCv)) return false;
	if (!isCvCloseToZero(highpassCv)) return false;
	if (!isCvCloseToZero(lowpassCv)) return false;
	if (!isCvCloseToZero(levelDryCv)) return false;

	// Knobs need to be fully CCW
	float timeKnob = hw.GetTimeKnob();
	float spreadKnob = hw.GetSpreadKnob();
	float feedbackKnob = hw.GetFeedbackKnob();
	float lowpassKnob = hw.GetLowpassKnob();
	float highpassKnob = hw.GetHighpassKnob();

	hw.PrintLine("Knobs (need <0.05 for CCW):");
	hw.PrintLine("  Time:     " FLT_FMT(4) " %s", FLT_VAR(4, timeKnob), minMaxKnob(1.0 - timeKnob) >= 0.95 ? "OK" : "FAIL");
	hw.PrintLine("  Spread:   " FLT_FMT(4) " %s", FLT_VAR(4, spreadKnob), minMaxKnob(1.0 - spreadKnob) >= 0.95 ? "OK" : "FAIL");
	hw.PrintLine("  Feedback: " FLT_FMT(4) " %s", FLT_VAR(4, feedbackKnob), minMaxKnob(1.0 - feedbackKnob) >= 0.95 ? "OK" : "FAIL");
	hw.PrintLine("  Lowpass:  " FLT_FMT(4) " %s", FLT_VAR(4, lowpassKnob), minMaxKnob(1.0 - lowpassKnob) >= 0.95 ? "OK" : "FAIL");
	hw.PrintLine("  Highpass: " FLT_FMT(4) " %s", FLT_VAR(4, highpassKnob), minMaxKnob(1.0 - highpassKnob) >= 0.95 ? "OK" : "FAIL");

	if (minMaxKnob(1.0 - timeKnob) < 0.95) return false;
	if (minMaxKnob(1.0 - spreadKnob) < 0.95) return false;
	if (minMaxKnob(1.0 - feedbackKnob) < 0.95) return false;
	if (minMaxKnob(1.0 - lowpassKnob) < 0.95) return false;
	if (minMaxKnob(1.0 - highpassKnob) < 0.95) return false;

	// Sliders and level CVs
	hw.PrintLine("Sliders (need <0.05) and Level CVs (need ~0):");
	for (int i = 0; i < N_TAPS; i++)
	{
		float levelCv = hw.GetLevelCV(i);
		float levelSlider = hw.GetLevelSlider(i);
		bool cvOk = isCvCloseToZero(levelCv);
		bool sliderOk = minMaxSlider(1.0 - levelSlider) >= 0.95;

		hw.PrintLine("  [%d] CV:" FLT_FMT(4) " %s  Slider:" FLT_FMT(4) " %s",
			i, FLT_VAR(4, levelCv), cvOk ? "OK" : "FAIL",
			FLT_VAR(4, levelSlider), sliderOk ? "OK" : "FAIL");

		if (!cvOk) return false;
		if (!sliderOk) return false;
	}

	hw.PrintLine("=== All checks passed - calibrating ===");
	return true;
}


void calibrate(CvCalibrationData &saved, int ledSeqDelay)
{
	// do reverse LED startup sequence while
	// checking that we definitely want to calibrate
	for (int i = 0; i < (5000 / ledSeqDelay); i++)
	{
		for (int j = 0; j < N_TAPS; j++)
		{
			leds[j].Set(j == ((N_TAPS - 1) - (i % N_TAPS)) ? 1.0 : 0.0);
			leds[j].Update();
		}

		System::Delay(ledSeqDelay);
		if (!shouldCalibrate()) return;
	}

	saved.Init();

	hw.PrintLine("Starting calibration");

	// perform calibration routine
	for (int i = 0; i < CALIBRATION_SAMPLES; i++)
	{
		// accumulate raw ADC values (not ControlHandler.cv which has offsets applied)
		saved.timeCvOffset += hw.GetAdcValue(TIME_CV);
		saved.spreadCvOffset += hw.GetAdcValue(SPREAD_CV);
		saved.feedbackCvOffset += hw.GetAdcValue(FEEDBACK_CV);
		saved.lowpassCvOffset += hw.GetAdcValue(LOWPASS_CV);
		saved.highpassCvOffset += hw.GetAdcValue(HIGHPASS_CV);

		for (int j = 0; j < N_TAPS; j++)
		{
			saved.levelsCvOffset[j] += hw.GetLevelCV(j);
		}

		// wait 1ms
		System::Delay(1);

		// set LEDs
		for (int ledIndex = 0; ledIndex < N_TAPS; ledIndex++)
		{
			leds[ledIndex].Set(i % (N_TAPS - 1) < 4 ? 1.0f : 0.0f);
			leds[ledIndex].Update();
		}
	}

	// divide CVs by number of samples taken to get average

	float n = (float) CALIBRATION_SAMPLES;
	saved.timeCvOffset = saved.timeCvOffset / n;
	saved.spreadCvOffset = saved.spreadCvOffset / n;
	saved.feedbackCvOffset = saved.feedbackCvOffset / n;
	saved.lowpassCvOffset = saved.lowpassCvOffset / n;
	saved.highpassCvOffset = saved.highpassCvOffset / n;

	for (int i = 0; i < N_TAPS; i++)
	{
		saved.levelsCvOffset[i] = saved.levelsCvOffset[i] / n;
	}

	// Validate calibration
	// Reject if any offset is not close to zero
	// This catches cases where voltage was patched during calibration
	bool valid = true;

	if (!isCvCloseToZero(saved.timeCvOffset)) valid = false;
	if (!isCvCloseToZero(saved.spreadCvOffset)) valid = false;
	if (!isCvCloseToZero(saved.feedbackCvOffset)) valid = false;
	if (!isCvCloseToZero(saved.lowpassCvOffset)) valid = false;
	if (!isCvCloseToZero(saved.highpassCvOffset)) valid = false;

	for (int i = 0; i < N_TAPS; i++)
	{
		if (!isCvCloseToZero(saved.levelsCvOffset[i])) valid = false;
	}

	if (!valid)
	{
		hw.PrintLine("Calibration rejected: offsets out of range");
		saved.Init();  // Reset to defaults

		// Flash all LEDs to indicate error
		for (int i = 0; i < 6; i++)
		{
			float brightness = (i % 2 == 0) ? 1.0f : 0.0f;
			for (int j = 0; j < N_TAPS; j++)
			{
				leds[j].Set(brightness);
				leds[j].Update();
			}
			System::Delay(200);
		}
		return;
	}

	hw.PrintLine("Calibration successful");

	// save calibration data
	CalibrationDataStorage.Save();
}

void applyCalibrationOffsets(const CvCalibrationData &calibration)
{
	time.cvOffset = calibration.timeCvOffset;
	spread.cvOffset = calibration.spreadCvOffset;
	feedback.cvOffset = calibration.feedbackCvOffset;
	highpass.cvOffset = calibration.highpassCvOffset;
	lowpass.cvOffset = calibration.lowpassCvOffset;

	for (int i = 0; i < N_TAPS; i++)
	{
		levels[i].cvOffset = calibration.levelsCvOffset[i];
	}
}


// Log raw ADC values for deadzone analysis
// Run this while moving controls to their min/max positions to determine optimal deadzones
void logControlDiagnostics()
{
	hw.PrintLine("=== CONTROL DIAGNOSTICS (raw ADC values) ===");
	hw.PrintLine("Move controls to min/max to see endpoint behavior");
	hw.PrintLine("");

	// Main knobs: raw values
	hw.PrintLine("KNOBS (raw):");
	hw.PrintLine("  Time:     " FLT_FMT(6), FLT_VAR(6, hw.GetTimeKnob()));
	hw.PrintLine("  Spread:   " FLT_FMT(6), FLT_VAR(6, hw.GetSpreadKnob()));
	hw.PrintLine("  Feedback: " FLT_FMT(6), FLT_VAR(6, hw.GetFeedbackKnob()));
	hw.PrintLine("  Lowpass:  " FLT_FMT(6), FLT_VAR(6, hw.GetLowpassKnob()));
	hw.PrintLine("  Highpass: " FLT_FMT(6), FLT_VAR(6, hw.GetHighpassKnob()));
	hw.PrintLine("");

	// CV inputs: raw values
	hw.PrintLine("CV INPUTS (raw, should be ~0 when unpatched):");
	hw.PrintLine("  Time CV:     " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(TIME_CV)));
	hw.PrintLine("  Spread CV:   " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(SPREAD_CV)));
	hw.PrintLine("  Feedback CV: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(FEEDBACK_CV)));
	hw.PrintLine("  Lowpass CV:  " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(LOWPASS_CV)));
	hw.PrintLine("  Highpass CV: " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(HIGHPASS_CV)));
	hw.PrintLine("  Dry CV:      " FLT_FMT(6), FLT_VAR(6, hw.GetAdcValue(LEVEL_DRY_CV)));
	hw.PrintLine("");

	// Level sliders: raw values
	hw.PrintLine("SLIDERS (raw):");
	for (int i = 0; i < N_TAPS; i++)
	{
		hw.PrintLine("  Slider %d: " FLT_FMT(6), i, FLT_VAR(6, hw.GetLevelSlider(i)));
	}
	hw.PrintLine("");

	// Level CVs: raw values
	hw.PrintLine("LEVEL CVs (raw):");
	for (int i = 0; i < N_TAPS; i++)
	{
		hw.PrintLine("  Level CV %d: " FLT_FMT(6), i, FLT_VAR(6, hw.GetLevelCV(i)));
	}
	hw.PrintLine("");

	// Pan knobs: raw values
	hw.PrintLine("PAN KNOBS (raw):");
	for (int i = 0; i < N_TAPS; i++)
	{
		hw.PrintLine("  Pan %d: " FLT_FMT(6), i, FLT_VAR(6, hw.GetPanKnob(i)));
	}
	hw.PrintLine("");
}


void logState()
{
	hw.PrintLine("CPU AVG: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetAvgCpuLoad()));
	hw.PrintLine("CPU MIN: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMinCpuLoad()));
	hw.PrintLine("CPU MAX: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMaxCpuLoad()));
	hw.PrintLine("DROPPED FRAMES: %d", droppedFrames);

	// hw.PrintLine("");
	// hw.PrintLine("clock: %s", hw.gate_in_1.State() ? "on" : "off");

	spread.Dump();
	time.Dump();
	feedback.Dump();
	highpass.Dump();
	lowpass.Dump();

	// hw.PrintLine("");

	// for (int i = 0; i < N_TAPS; i++)
	// {
	// 	levels[i].Dump();
	// }

	// // Debug: log raw level CV 8 value
	// hw.PrintLine("DEBUG level_8 raw: GetLevelCV(8)=" FLT_FMT(5), FLT_VAR(5, hw.GetLevelCV(8)));

	// hw.PrintLine("");

	// for (int i = 0; i < N_TAPS; i++)
	// {
	// 	pans[i].Dump();
	// }

	hw.PrintLine("");

	// Dynamics info
	float userFb, effectiveFb, peak, ampCoef, wetGain;
	timeMachine.GetDynamicsInfo(&userFb, &effectiveFb, &peak, &ampCoef, &wetGain);

	float fbReduction = (userFb > 0.001f) ? (effectiveFb / userFb) : 1.0f;
	float totalOutputGain = ampCoef * wetGain;

	hw.PrintLine("=== Dynamics ===");
	hw.PrintLine("Feedback: user=" FLT_FMT(3) " effective=" FLT_FMT(3) " (ceiling=" FLT_FMT(1) "%%)",
		FLT_VAR(3, userFb), FLT_VAR(3, effectiveFb), FLT_VAR(1, fbReduction * 100.0f));
	hw.PrintLine("Peak (predicted): " FLT_FMT(3), FLT_VAR(3, peak));
	hw.PrintLine("Output: ampCoef=" FLT_FMT(3) " wetGain=" FLT_FMT(3) " total=" FLT_FMT(3),
		FLT_VAR(3, ampCoef), FLT_VAR(3, wetGain), FLT_VAR(3, totalOutputGain));

	float minL, maxL, minR, maxR;
	timeMachine.GetOutputMinMax(&minL, &maxL, &minR, &maxR);
	hw.PrintLine("Output range L:[" FLT_FMT(3) "," FLT_FMT(3) "] R:[" FLT_FMT(3) "," FLT_FMT(3) "]",
		FLT_VAR(3, minL), FLT_VAR(3, maxL), FLT_VAR(3, minR), FLT_VAR(3, maxR));
	timeMachine.ResetOutputMinMax();
	hw.PrintLine("");
}


int main(void)
{
	// init time machine hardware
  hw.Init();
	hw.StartLog();

	hw.SetAudioBlockSize(7); // number of samples handled per callback


	Pin gatePin = CLOCK;
	gate.Init(gatePin);

	// initialize LEDs
	leds[0].Init(LED_DRY, false);
	leds[1].Init(LED_1, false);
	leds[2].Init(LED_2, false);
	leds[3].Init(LED_3, false);
	leds[4].Init(LED_4, false);
	leds[5].Init(LED_5, false);
	leds[6].Init(LED_6, false);
	leds[7].Init(LED_7, false);
	leds[8].Init(LED_8, false);

	for (int i = 0; i < N_TAPS; i++)
	{
		char buf[10];
		sprintf(buf, "pan_%d", i);
		pans[i].SetDumpName(buf);

		sprintf(buf, "level_%d", i);
		levels[i].SetDumpName(buf);
	}

  // Initialize our switch.
  feedbackModeSwitch.Init(FEEDBACK_MODE_SWITCH, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
	filterPositionSwitch.Init(FILTER_POSITION_SWITCH, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

	// set sample rate
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

	// init clock rate detector
	clockRateDetector.Init(hw.AudioSampleRate());

	// init time machine with interleaved stereo buffer
  timeMachine.Init(hw.AudioSampleRate(), TIME_SECONDS + (((float) BUFFER_WIGGLE_ROOM_SAMPLES) * 0.5 / hw.AudioSampleRate()), buffer);

	// load calibration data, using sensible defaults
	CvCalibrationData defaults;
	defaults.Init();

	CalibrationDataStorage.Init(defaults);
	CvCalibrationData &savedCalibrationData = CalibrationDataStorage.GetSettings();

	// Log calibration state and handle version mismatch
	auto storageState = CalibrationDataStorage.GetState();
	if (storageState == PersistentStorage<CvCalibrationData>::State::FACTORY)
	{
		hw.PrintLine("First boot - using factory defaults");
	}
	else if (!savedCalibrationData.IsValid())
	{
		// Version mismatch: old or corrupted calibration data
		// Reset to defaults to avoid garbage values
		hw.PrintLine("Calibration data version mismatch (got %lu, expected %lu) - resetting to defaults",
			savedCalibrationData.version, CvCalibrationData::CURRENT_VERSION);
		savedCalibrationData.Init();
		CalibrationDataStorage.Save();
	}
	else
	{
		hw.PrintLine("Loaded user calibration data (version %lu)", savedCalibrationData.version);
	}

	// Apply calibration offsets BEFORE starting audio so audio callback
	// has correct offsets from the first sample
	applyCalibrationOffsets(savedCalibrationData);

	// init cpu meter
	cpuMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());

	// start time machine hardware audio and logging
  hw.StartAudio(audioCallback);

	// LED startup sequence
	int ledSeqDelay = 100;
	for(int i = 0; i < N_TAPS; i++)
	{
		for(int j = 0; j < N_TAPS; j++)
		{
			leds[j].Set(j == i ? 1.0 : 0.0);
			leds[j].Update();
		}

		System::Delay(ledSeqDelay);
	}

	if (shouldCalibrate())
	{
		calibrate(savedCalibrationData, ledSeqDelay);
		// Re-apply new calibration offsets after calibration
		applyCalibrationOffsets(savedCalibrationData);
	}

	setLeds = true;

	// FPU busywork state keeps the FPU active between audio interrupts so
	// that hopefully the noise is less bad????
	static volatile float fpu_dummy = 1.0f;
	uint32_t last_log_ms = 0;

	while (true)
	{
#ifdef LOGGING_ENABLED
		// Throttle slow tasks using the millisecond tick instead of blocking delays
		uint32_t now = System::GetNow();

		if (now - last_log_ms >= 500)
		{
			logState();
			last_log_ms = now;
		}
#endif

		// Keep FPU active to level power draw. volatile prevents optimization.
		// Multiplier kept near 1.0 to avoid NaN/Inf accumulation over time.
		// I honestly have no idea if this is worthwhile to do but it can't hurt.
		fpu_dummy = fpu_dummy * 0.99999f + 0.00001f;
	}
}
