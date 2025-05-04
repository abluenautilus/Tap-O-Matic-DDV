#pragma once

#define BODGE false

#include "daisy.h"
#include "daisy_patch_sm.h"

using namespace daisy;
using namespace patch_sm;

#define LED_DRY DaisyPatchSM::D7
#define LED_1 DaisyPatchSM::D6
#define LED_2 DaisyPatchSM::D5
#define LED_3 DaisyPatchSM::A9
#define LED_4 DaisyPatchSM::A8
#define LED_5 DaisyPatchSM::D1
#define LED_6 DaisyPatchSM::A2
#define LED_7 DaisyPatchSM::B7
#define LED_8 DaisyPatchSM::B8

#define CLOCK DaisyPatchSM::B10
#define FEEDBACK_MODE_SWITCH DaisyPatchSM::D10
#define FILTER_POSITION_SWITCH DaisyPatchSM::B9

#define MUX_A DaisyPatchSM::D2
#define MUX_B DaisyPatchSM::D3
#define MUX_C DaisyPatchSM::D4

#define LEVEL_CV_MUX CV_1
#define PAN_MUX ADC_10
#define LEVEL_MUX ADC_11
#define MISC_MUX ADC_12

#define LEVEL_DRY_CV CV_2
#define HIGHPASS_CV CV_3
#define LOWPASS_CV CV_4
#define SPREAD_CV CV_5
#define TIME_CV CV_6
#define FEEDBACK_CV CV_7



namespace oam
{
namespace time_machine
{
    /** Accessors for the Analog Controls.
     *  These cover the 8x Bipolar CV inputs
     *  as well as the 4x 0-3V3 ADC inputs on
     *  the hardware
     *
     *  When reading a value with TimeMachineHardware::GetAdcValue()
     *
     *  patch.GetAdcValue(patch_sm::CV_1);
     */

    /*
    enum
    {
        CV_1 = 0,
        CV_2,
        CV_3,
        CV_4,
        CV_5,
        CV_6,
        CV_7,
        CV_8,
        ADC_9,
        ADC_10,
        ADC_11,
        ADC_12,
        ADC_LAST,
    };*/

    /** Enum for addressing the CV Outputs via the WriteCvOut function. */
    /*
    enum
    {
        CV_OUT_BOTH = 0,
        CV_OUT_1,
        CV_OUT_2,
    };*/


    /** @brief Board support file for TimeMachineHardware hardware
     *  @author shensley
     *  @ingroup boards
     *
     *  Daisy Patch SM is a complete Eurorack module DSP engine.
     *  Based on the Daisy Seed, with circuits added for
     *  interfacing directly with eurorack modular synthesizers.
     */
    class TimeMachineHardware
    {
      public:
        /** Helper for mapping pins, and accessing them using the `GetPin` function */
        enum class PinBank
        {
            A,
            B,
            C,
            D
        };

        TimeMachineHardware() {}
        ~TimeMachineHardware() {}

        /** Initializes the memories, and core peripherals for the Daisy Patch SM */
        void Init();

        /** Starts a non-interleaving audio callback */
        void StartAudio(AudioHandle::AudioCallback cb);

        /** Starts an interleaving audio callback */
        void StartAudio(AudioHandle::InterleavingAudioCallback cb);

        /** Changes the callback that is executing.
         *  This may cause clicks if done while audio is processing.
         */
        void ChangeAudioCallback(AudioHandle::AudioCallback cb);

        /** Changes the callback that is executing.
         *  This may cause clicks if done while audio is processing.
         */
        void ChangeAudioCallback(AudioHandle::InterleavingAudioCallback cb);

        /** Stops the transmission of audio. */
        void StopAudio();

        /** Sets the number of samples processed in an audio callback.
         *  This will only take effect on the next invocation of `StartAudio`
         */
        void SetAudioBlockSize(size_t size);

        /** Sets the samplerate for the audio engine
         *  This will set it to the closest valid samplerate. Options being:
         *  8kHz, 16kHz, 32kHz, 48kHz, and 96kHz
         */
        void SetAudioSampleRate(float sr);

        void SetAudioSampleRate(SaiHandle::Config::SampleRate sample_rate);

        /** Returns the number of samples processed in an audio callback */
        size_t AudioBlockSize();

        /** Returns the audio engine's samplerate in Hz */
        float AudioSampleRate();

        /** Returns the rate at which the audio callback will be called in Hz */
        float AudioCallbackRate();

        /** Starts the Control ADCs
         *
         *  This is started automatically when Init() is called.
         */
        void StartAdc();

        /** Stops the Control ADCs */
        void StopAdc();

        /** Reads and filters all of the analog control inputs */
        void ProcessAnalogControls();

        /** Reads and debounces any of the digital control inputs
         *  This does nothing on this board at this time.
         */
        void ProcessDigitalControls();

        /** Does both of the above */
        void ProcessAllControls()
        {
            ProcessAnalogControls();
            ProcessDigitalControls();
        }

        /** Returns the current value for one of the ADCs */
        float GetAdcValue(int idx);

        float GetLevelSlider(int idx);
        float GetLevelCV(int idx);
        float GetPanKnob(int idx);
        float GetSpreadKnob();
        float GetTimeKnob();
        float GetFeedbackKnob();
        float GetHighpassKnob();
        float GetLowpassKnob();

        /** Returns the STM32 port/pin combo for the desired pin (or an invalid pin for HW only pins)
         *
         *  Macros at top of file can be used in place of separate arguments (i.e. GetPin(A4), etc.)
         *
         *  \param bank should be one of the PinBank options above
         *  \param idx pin number between 1 and 10 for each of the pins on each header.
         */
        daisy::Pin GetPin(const PinBank bank, const int idx);

        /** Starts the DAC for the CV Outputs
         *
         *  By default this starts by running the
         *  internal callback at 48kHz, which will
         *  update the values based on the SetCvOut
         *  function.
         *
         *  This is started automatically when Init() is called.
         */
        void StartDac(DacHandle::DacCallback callback = nullptr);

        /** Stop the DAC from updating.
         *  This will suspend the CV Outputs from changing
         */
        void StopDac();

        /** Sets specified DAC channel to the target voltage.
         *  This may not be 100% accurate without calibration.
         *
         *  \todo Add Calibration to CV Outputs
         *
         *  \param channel desired channel to update. 0 is both, otherwise 1 or 2 are valid.
         *  \param voltage value in Volts that you'd like to write to the DAC. The valid range is 0-5V.
         */
        void WriteCvOut(const int channel, float voltage);

        /** Here are some wrappers around libDaisy Static functions
         *  to provide simpler syntax to those who prefer it. */

        /** Delays for a specified number of milliseconds */
        inline void Delay(uint32_t milliseconds)
        {
            System::Delay(milliseconds);
        }

        /** Gets a random 32-bit value */
        inline uint32_t GetRandomValue() { return Random::GetValue(); }

        /** Gets a random floating point value between the specified minimum, and maxmimum */
        inline float GetRandomFloat(float min = 0.f, float max = 1.f)
        {
            return Random::GetFloat(min, max);
        }

        void SetLed(bool state);

        /** Print formatted debug log message
         */
        template <typename... VA>
        static void Print(const char* format, VA... va)
        {
            Log::Print(format, va...);
        }

        /** Print formatted debug log message with automatic line termination
         */
        template <typename... VA>
        static void PrintLine(const char* format, VA... va)
        {
            Log::PrintLine(format, va...);
        }

        /** Start the logging session. Optionally wait for terminal connection before proceeding.
         */
        static void StartLog(bool wait_for_pc = false)
        {
            Log::StartLog(wait_for_pc);
        }

        /** @brief Tests entirety of SDRAM for validity
         *         This will wipe contents of SDRAM when testing.
         *
         *  @note   If using the SDRAM for the default bss, or heap,
         *          and using constructors as initializers do not
         *          call this function. Otherwise, it could
         *          overwrite changes performed by constructors.
         *
         *  \retval returns true if SDRAM is okay, otherwise false
         */
        bool ValidateSDRAM();

        /** @brief Tests the QSPI for validity
         *         This will wipe contents of QSPI when testing.
         *
         *  @note  If called with quick = false, this will erase all memory
         *         the "quick" test starts 0x400000 bytes into the memory and
         *         test 16kB of data
         *
         *  \param quick if this is true the test will only test a small piece of the QSPI
         *               checking the entire 8MB can take roughly over a minute.
         *
         *  \retval returns true if SDRAM is okay, otherwise false
         */
        bool ValidateQSPI(bool quick = true);

        /** Direct Access Structs/Classes */
        System      system;
        SdramHandle sdram;
        QSPIHandle  qspi;
        AudioHandle audio;
        AdcHandle   adc;
        UsbHandle   usb;
        Pcm3060     codec;
        DacHandle   dac;

        /** Dedicated Function Pins */
        GPIO      user_led;
        AnalogControl controls[ADC_LAST];
        GateIn        gate_in_1, gate_in_2;
        GPIO      gate_out_1, gate_out_2;

        /** Pin Accessors for the TimeMachineHardware hardware
         *  Used for initializing various GPIO, etc.
         */
        constexpr static Pin A1  = Pin(PORTX, 0); /**< A1  - -12V Power Input */
        constexpr static Pin A2  = Pin(PORTA, 1); /**< A2  - UART1 Rx */
        constexpr static Pin A3  = Pin(PORTA, 0); /**< A3  - UART1 Tx */
        constexpr static Pin A4  = Pin(PORTX, 0); /**< A4  - GND */
        constexpr static Pin A5  = Pin(PORTX, 0); /**< A5  - +12V Power Input */
        constexpr static Pin A6  = Pin(PORTX, 0); /**< A6  - +5V Power Output */
        constexpr static Pin A7  = Pin(PORTX, 0); /**< A7  - GND */
        constexpr static Pin A8  = Pin(PORTB, 14); /**< A8  - USB DM */
        constexpr static Pin A9  = Pin(PORTB, 15); /**< A9  - USB DP */
        constexpr static Pin A10 = Pin(PORTX, 0);  /**< A10 - +3V3 Power Out */

        constexpr static Pin B1  = Pin(PORTX, 0);  /**< B1  - Audio Out Right */
        constexpr static Pin B2  = Pin(PORTX, 0);  /**< B2  - Audio Out Left*/
        constexpr static Pin B3  = Pin(PORTX, 0);  /**< B3  - Audio In Right */
        constexpr static Pin B4  = Pin(PORTX, 0);  /**< B4  - Audio In Left */
        constexpr static Pin B5  = Pin(PORTC, 14); /**< B5  - GATE OUT 1 */
        constexpr static Pin B6  = Pin(PORTC, 13); /**< B6  - GATE OUT 2 */
        constexpr static Pin B7  = Pin(PORTB, 8);  /**< B7  - I2C1 SCL */
        constexpr static Pin B8  = Pin(PORTB, 9);  /**< B8  - I2C1 SDA */
        constexpr static Pin B9  = Pin(PORTG, 14); /**< B9  - GATE IN 2 */
        constexpr static Pin B10 = Pin(PORTG, 13); /**< B10 - GATE IN 1 */

        constexpr static Pin C1  = Pin(PORTA, 5); /**< C1  - CV Out 2 */
        constexpr static Pin C2  = Pin(PORTA, 7); /**< C2  - CV In 4 */
        constexpr static Pin C3  = Pin(PORTA, 2); /**< C3  - CV In 3 */
        constexpr static Pin C4  = Pin(PORTA, 6); /**< C4  - CV In 2 */
        constexpr static Pin C5  = Pin(PORTA, 3); /**< C5  - CV In 1 */
        constexpr static Pin C6  = Pin(PORTB, 1); /**< C6  - CV In 5 */
        constexpr static Pin C7  = Pin(PORTC, 4); /**< C7  - CV In 6 */
        constexpr static Pin C8  = Pin(PORTC, 0); /**< C8  - CV In 7 */
        constexpr static Pin C9  = Pin(PORTC, 1); /**< C9  - CV In 8 */
        constexpr static Pin C10 = Pin(PORTA, 4); /**< C10 - CV Out 1 */

        constexpr static Pin D1  = Pin(PORTB, 4);  /**< D1  - SPI2 CS */
        constexpr static Pin D2  = Pin(PORTC, 11); /**< D2  - SDMMC D3 */
        constexpr static Pin D3  = Pin(PORTC, 10); /**< D3  - SDMMC D2*/
        constexpr static Pin D4  = Pin(PORTC, 9);  /**< D4  - SDMMC D1*/
        constexpr static Pin D5  = Pin(PORTC, 8);  /**< D5  - SDMMC D0 */
        constexpr static Pin D6  = Pin(PORTC, 12); /**< D6  - SDMMC CK */
        constexpr static Pin D7  = Pin(PORTD, 2);  /**< D7  - SDMMC CMD */
        constexpr static Pin D8  = Pin(PORTC, 2);  /**< D8  - SPI2 MISO */
        constexpr static Pin D9  = Pin(PORTC, 3);  /**< D9  - SPI2 MOSI */
        constexpr static Pin D10 = Pin(PORTD, 3);  /**< D10 - SPI2 SCK  */
        class Impl;

      private:
        using Log = Logger<LOGGER_INTERNAL>;

        float callback_rate_;

        /** Background callback for updating the DACs. */
        Impl* pimpl_;
    };

} // namespace patch_sm

} // namespace daisy
