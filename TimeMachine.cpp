#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "dsp.h"
#include "time_machine_hardware.h"

using namespace daisy;
using namespace oam;
using namespace time_machine;
using namespace std;

#define N_TAPS 9
#define TIME_SECONDS 150
#define CALIBRATION_SAMPLES 128
#define BUFFER_WIGGLE_ROOM_SAMPLES 1000

TimeMachineHardware hw;

// Setting Struct containing parameters we want to save to flash
struct CvCalibrationData
{
	float timeCvOffset;
	float spreadCvOffset;
	float feedbackCvOffset;
	float lowpassCvOffset;
	float highpassCvOffset;
	float levelsCvOffset[N_TAPS];

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
	}

	// Overloading the != operator
	// This is necessary as this operator is used in the PersistentStorage source code
	bool operator!=(const CvCalibrationData& a) const
	{
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
		cvSlew.Init(0.5, 0.0005);
		cvOffset = 0.0;

		knob = 0.0;
		knobSlew.Init(0.5, 0.0005);

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


// init buffers - add an extra second just in case we somehow end up slightly beyond max time
// due to precision loss in floating point arithmetic (maybe use doubles for time values???)
float DSY_SDRAM_BSS bufferLeft[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];
float DSY_SDRAM_BSS bufferRight[48000 * TIME_SECONDS + BUFFER_WIGGLE_ROOM_SAMPLES];

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
	feedback.knob = fourPointWarp(1.0 - minMaxKnob(hw.GetFeedbackKnob(), 0.028));
	feedback.cv = clamp(hw.GetAdcValue(FEEDBACK_CV) - feedback.cvOffset, -1.0, 1.0);
	feedback.value = clamp(fourPointWarp(feedback.knobSlew.Process(feedback.knob)) * 2.0 + feedback.cvSlew.Process(feedback.cv), 0, 3);

	// condition spread knob value to have deadzone in the middle, add CV
	spread.knob = fourPointWarp(1.0 - minMaxKnob(hw.GetSpreadKnob(), 0.0008));
	spread.cv = clamp(hw.GetAdcValue(SPREAD_CV) - spread.cvOffset, -1.0, 1.0);
	spread.value = fourPointWarp(spread.knobSlew.Process(spread.knob)) + spread.cvSlew.Process(spread.cv);

	lowpass.knob = 1.0f - hw.GetLowpassKnob();
	lowpass.cv = clamp(hw.GetAdcValue(LOWPASS_CV) - lowpass.cvOffset, -1.0, 1.0);
	//lowpass.value = clamp(lowpass.knobSlew.Process(lowpass.knob) + lowpass.cvSlew.Process(lowpass.cv), 0.001, 1.0) / 2.0;
	lowpass.value = clamp(lowpass.knobSlew.Process(lowpass.knob) + lowpass.cvSlew.Process(lowpass.cv), 0.001, 1.0);
	lowpass.value = (exp(lowpass.value * 3.0) - 1.0) / (1.72 * 22.2);

	highpass.knob = 1.0f - hw.GetHighpassKnob();
	highpass.cv = clamp(hw.GetAdcValue(HIGHPASS_CV) - highpass.cvOffset, -1.0, 1.0);
	//highpass.value = clamp(highpass.knobSlew.Process(highpass.knob) + highpass.cvSlew.Process(highpass.cv), 0.001, 1.0) / 2.0;
	highpass.value = clamp(highpass.knobSlew.Process(highpass.knob) + highpass.cvSlew.Process(highpass.cv), 0.001, 1.0);
	highpass.value = (exp(highpass.value * 3.0) - 1.0) / (1.72 * 22.2);

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

	time.knob = minMaxKnob(1.0 - hw.GetTimeKnob(), 0.0008);
	time.cv = clamp(hw.GetAdcValue(TIME_CV) - time.cvOffset, -1.0, 1.0);

	// calculate time based on clock if present, otherwise simple time
	time.value = 0.0;
	if (clockRateDetector.GetInterval() > 0.0)
	{
		// 12 quantized steps for knob, 10 for CV (idk what these quanta should actually be)
		// time doubles and halves with each step, they are additive/subtractive
		float timeCoef = pow(2.0, (timeKnobSchmidt.Process((1.0 - time.knob) * 12)) + (timeCvSchmidt.Process(time.cv * 10))) / pow(2.0, 6.0);
		time.value = clockRateDetector.GetInterval() / timeCoef;

		// make sure time is a power of two less than the max time available in the buffer
		while(time.value > TIME_SECONDS) time.value *= 0.5;
	}
	else
	{
		// time linear with knob, scaled v/oct style with CV
		time.value = pow(time.knobSlew.Process(time.knob), 2.0) * 8.0 / pow(2.0, time.cvSlew.Process(time.cv) * 5.0);
	}

	// force time down to a max value (taking whichever is lesser, the max or the time)
	time.value = std::min((float)TIME_SECONDS, time.value);
}


// called every N samples (search for SetAudioBlockSize)
void audioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// cpu meter measurements start
	cpuMeter.OnBlockStart();
	droppedFrames++;

	// process controls at the ADC level.
	hw.ProcessAllControls();

	// Then we can use those values when updating our handlers.
	updateControlHandlers();

	for (int i = 1; i < N_TAPS; i++)
	{
		// set LEDs based on loudness for last 8 sliders
		float loudness = timeMachine.timeMachineLeft.readHeads[i - 1].loudness.Get();
		loudness = max(loudness, timeMachine.timeMachineRight.readHeads[i - 1].loudness.Get());
		if (setLeds)
		{
			leds[i].Set(loudness);
			leds[i].Update();
		}
	}

	// set LEDs based on loudness for first slider
	float loudness = timeMachine.timeMachineLeft.loudness.Get();
	loudness = max(loudness, timeMachine.timeMachineRight.loudness.Get());
	if (setLeds)
	{
		leds[0].Set(loudness);
		leds[0].Update();
	}

	// set time machine dry slider value, feedback, "blur" which is semi-deprecated
	timeMachine.Set(levels[0].value, feedback.value, feedback.value, highpass.value, lowpass.value, feedbackModeSwitch.Read(), filterPositionSwitch.Read());

	for (int i = 1; i < N_TAPS; i++)
	{
		float leftPan, rightPan;
		panToVolume(hw.GetPanKnob(i), &leftPan, &rightPan);

		// let last 8 slider time/amp/blur values for left channel time machine instance
        timeMachine.timeMachineLeft.readHeads[i - 1].Set(
            spreadTaps((i / (float) (N_TAPS - 1)), spread.value) * time.value,
            leftPan * levels[i].value,
			max(0.0, feedback.value - 1.0)
        );

		// let last 8 slider time/amp/blur values for right channel time machine instance
		timeMachine.timeMachineRight.readHeads[i - 1].Set(
            spreadTaps((i / (float) (N_TAPS - 1)), spread.value) * time.value,
            rightPan * levels[i].value,
			max(0.0, feedback.value -1.0)
        );
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

	//cpu meter measurement stop
	cpuMeter.OnBlockEnd();
	droppedFrames--;
}


bool isCvCloseToZero(float val)
{
	return (val > -0.03) && (val < 0.03);
}


bool shouldCalibrate()
{
	hw.PrintLine("Considering calibration:");

	if (!hw.gate_in_1.State()) return false;

	if (!isCvCloseToZero(hw.GetAdcValue(SPREAD_CV))) 	return false;
	if (!isCvCloseToZero(hw.GetAdcValue(TIME_CV))) 		return false;
	if (!isCvCloseToZero(hw.GetAdcValue(FEEDBACK_CV))) 	return false;
	if (!isCvCloseToZero(hw.GetAdcValue(HIGHPASS_CV))) 	return false;
	if (!isCvCloseToZero(hw.GetAdcValue(LOWPASS_CV))) 	return false;
	if (!isCvCloseToZero(hw.GetAdcValue(LEVEL_DRY_CV))) return false;

	if (minMaxKnob(1.0 - hw.GetTimeKnob()) < 0.95) 		return false;
	if (minMaxKnob(1.0 - hw.GetSpreadKnob()) < 0.95) 	return false;
	if (minMaxKnob(1.0 - hw.GetFeedbackKnob()) < 0.95) 	return false;
	if (minMaxKnob(1.0 - hw.GetLowpassKnob()) < 0.95) 	return false;
	if (minMaxKnob(1.0 - hw.GetHighpassKnob()) < 0.95) 	return false;

	for (int i = 0; i < N_TAPS; i++)
	{
		if (!isCvCloseToZero(hw.GetLevelCV(i))) 				return false;
		if (minMaxSlider(1.0 - hw.GetLevelSlider(i)) < 0.95) 	return false;
	}

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
		// accumulate cv values
		saved.timeCvOffset += time.cv;
		saved.spreadCvOffset += spread.cv;
		saved.feedbackCvOffset += feedback.cv;
		saved.lowpassCvOffset += lowpass.cv;
		saved.highpassCvOffset += highpass.cv;

		for (int j = 0; j < N_TAPS; j++)
		{
			saved.levelsCvOffset[j] += levels[j].cv;
		}

		// wait 10ms
		System::Delay(10);

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

	// save calibration data
	CalibrationDataStorage.Save();
}

void logState()
{
	hw.PrintLine("CPU AVG: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetAvgCpuLoad()));
	hw.PrintLine("CPU MIN: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMinCpuLoad()));
	hw.PrintLine("CPU MAX: " FLT_FMT(6), FLT_VAR(6, cpuMeter.GetMaxCpuLoad()));
	hw.PrintLine("DROPPED FRAMES: %d", droppedFrames);

	hw.PrintLine("");

	hw.PrintLine("clock: %s", hw.gate_in_1.State() ? "on" : "off");

	spread.Dump();
	time.Dump();
	feedback.Dump();
	highpass.Dump();
	lowpass.Dump();

	hw.PrintLine("");

	for (int i = 0; i < N_TAPS; i++)
	{
		levels[i].Dump();
	}

	hw.PrintLine("");

	for (int i = 0; i < N_TAPS; i++)
	{
		pans[i].Dump();
	}

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

	// init time machine
    timeMachine.Init(hw.AudioSampleRate(), TIME_SECONDS + (((float) BUFFER_WIGGLE_ROOM_SAMPLES) * 0.5 / hw.AudioSampleRate()), bufferLeft, bufferRight);

	// load calibration data, using sensible defaults
	CvCalibrationData defaults;
	defaults.Init();

	CalibrationDataStorage.Init(defaults);
	CvCalibrationData &savedCalibrationData = CalibrationDataStorage.GetSettings();

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
	}

	time.cvOffset = savedCalibrationData.timeCvOffset;
	spread.cvOffset = savedCalibrationData.spreadCvOffset;
	feedback.cvOffset = savedCalibrationData.feedbackCvOffset;
	highpass.cvOffset = savedCalibrationData.highpassCvOffset;
	lowpass.cvOffset = savedCalibrationData.lowpassCvOffset;

	for (int i = 0; i < N_TAPS; i++)
	{
		levels[i].cvOffset = savedCalibrationData.levelsCvOffset[i];
	}

	setLeds = true;

	while (true)
	{
		logState();
		System::Delay(333);
	}
}
