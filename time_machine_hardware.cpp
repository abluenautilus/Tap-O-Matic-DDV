#include "time_machine_hardware.h"
#include <vector>

using namespace daisy;
namespace oam
{
namespace time_machine
{
    /** Const definitions */
    static constexpr Pin DUMMYPIN        = Pin(PORTX, 0);
    static constexpr Pin PIN_ADC_CTRL_1  = TimeMachineHardware::C5;
    static constexpr Pin PIN_ADC_CTRL_2  = TimeMachineHardware::C4;
    static constexpr Pin PIN_ADC_CTRL_3  = TimeMachineHardware::C3;
    static constexpr Pin PIN_ADC_CTRL_4  = TimeMachineHardware::C2;
    static constexpr Pin PIN_ADC_CTRL_5  = TimeMachineHardware::C6;
    static constexpr Pin PIN_ADC_CTRL_6  = TimeMachineHardware::C7;
    static constexpr Pin PIN_ADC_CTRL_7  = TimeMachineHardware::C8;
    static constexpr Pin PIN_ADC_CTRL_8  = TimeMachineHardware::C9;
    static constexpr Pin PIN_ADC_CTRL_9  = TimeMachineHardware::A2;
    static constexpr Pin PIN_ADC_CTRL_10 = TimeMachineHardware::A3;
    static constexpr Pin PIN_ADC_CTRL_11 = TimeMachineHardware::D9;
    static constexpr Pin PIN_ADC_CTRL_12 = TimeMachineHardware::D8;
    static constexpr Pin PIN_USER_LED    = Pin(PORTC, 7);

    constexpr Pin kPinMap[4][10] =
    // Bank A
    {{TimeMachineHardware::A1,
      TimeMachineHardware::A2,
      TimeMachineHardware::A3,
      TimeMachineHardware::A4,
      TimeMachineHardware::A5,
      TimeMachineHardware::A6,
      TimeMachineHardware::A7,
      TimeMachineHardware::A8,
      TimeMachineHardware::A9,
      TimeMachineHardware::A10},

     // Bank B
     {TimeMachineHardware::B1,
      TimeMachineHardware::B2,
      TimeMachineHardware::B3,
      TimeMachineHardware::B4,
      TimeMachineHardware::B5,
      TimeMachineHardware::B6,
      TimeMachineHardware::B7,
      TimeMachineHardware::B8,
      TimeMachineHardware::B9,
      TimeMachineHardware::B10},

     // Bank C
     {TimeMachineHardware::C1,
      TimeMachineHardware::C2,
      TimeMachineHardware::C3,
      TimeMachineHardware::C4,
      TimeMachineHardware::C5,
      TimeMachineHardware::C6,
      TimeMachineHardware::C7,
      TimeMachineHardware::C8,
      TimeMachineHardware::C9,
      TimeMachineHardware::C10},

     // Bank D
     {TimeMachineHardware::D1,
      TimeMachineHardware::D2,
      TimeMachineHardware::D3,
      TimeMachineHardware::D4,
      TimeMachineHardware::D5,
      TimeMachineHardware::D6,
      TimeMachineHardware::D7,
      TimeMachineHardware::D8,
      TimeMachineHardware::D9,
      TimeMachineHardware::D10}};

 
      constexpr Pin TimeMachineHardware::A1;
      constexpr Pin TimeMachineHardware::A2;
      constexpr Pin TimeMachineHardware::A3;
      constexpr Pin TimeMachineHardware::A4;
      constexpr Pin TimeMachineHardware::A5;
      constexpr Pin TimeMachineHardware::A6;
      constexpr Pin TimeMachineHardware::A7;
      constexpr Pin TimeMachineHardware::A8;
      constexpr Pin TimeMachineHardware::A9;
      constexpr Pin TimeMachineHardware::A10;
  
      constexpr Pin TimeMachineHardware::B1;
      constexpr Pin TimeMachineHardware::B2;
      constexpr Pin TimeMachineHardware::B3;
      constexpr Pin TimeMachineHardware::B4;
      constexpr Pin TimeMachineHardware::B5;
      constexpr Pin TimeMachineHardware::B6;
      constexpr Pin TimeMachineHardware::B7;
      constexpr Pin TimeMachineHardware::B8;
      constexpr Pin TimeMachineHardware::B9;
      constexpr Pin TimeMachineHardware::B10;
  
      constexpr Pin TimeMachineHardware::C1;
      constexpr Pin TimeMachineHardware::C2;
      constexpr Pin TimeMachineHardware::C3;
      constexpr Pin TimeMachineHardware::C4;
      constexpr Pin TimeMachineHardware::C5;
      constexpr Pin TimeMachineHardware::C6;
      constexpr Pin TimeMachineHardware::C7;
      constexpr Pin TimeMachineHardware::C8;
      constexpr Pin TimeMachineHardware::C9;
      constexpr Pin TimeMachineHardware::C10;
  
      constexpr Pin TimeMachineHardware::D1;
      constexpr Pin TimeMachineHardware::D2;
      constexpr Pin TimeMachineHardware::D3;
      constexpr Pin TimeMachineHardware::D4;
      constexpr Pin TimeMachineHardware::D5;
      constexpr Pin TimeMachineHardware::D6;
      constexpr Pin TimeMachineHardware::D7;
      constexpr Pin TimeMachineHardware::D8;
      constexpr Pin TimeMachineHardware::D9;
      constexpr Pin TimeMachineHardware::D10;

    /** outside of class static buffer(s) for DMA access */
    uint16_t DMA_BUFFER_MEM_SECTION dsy_patch_sm_dac_buffer[2][48];

    class TimeMachineHardware::Impl
    {
      public:
        Impl()
        {
            dac_running_            = false;
            dac_buffer_size_        = 48;
            dac_output_[0]          = 0;
            dac_output_[1]          = 0;
            internal_dac_buffer_[0] = dsy_patch_sm_dac_buffer[0];
            internal_dac_buffer_[1] = dsy_patch_sm_dac_buffer[1];
        }

        void InitDac();

        void StartDac(DacHandle::DacCallback callback);

        void StopDac();

        static void InternalDacCallback(uint16_t **output, size_t size);

        /** Based on a 0-5V output with a 0-4095 12-bit DAC */
        static inline uint16_t VoltageToCode(float input)
        {
            float pre = input * 819.f;
            if(pre > 4095.f)
                pre = 4095.f;
            else if(pre < 0.f)
                pre = 0.f;
            return (uint16_t)pre;
        }

        inline void WriteCvOut(int channel, float voltage)
        {
            if(channel == 0 || channel == 1)
                dac_output_[0] = VoltageToCode(voltage);
            if(channel == 0 || channel == 2)
                dac_output_[1] = VoltageToCode(voltage);
        }

        size_t    dac_buffer_size_;
        uint16_t *internal_dac_buffer_[2];
        uint16_t  dac_output_[2];
        DacHandle dac_;

      private:
        bool dac_running_;
    };

    /** Static Local Object */
    static TimeMachineHardware::Impl patch_sm_hw;

    /** Impl function definintions */

    void TimeMachineHardware::Impl::InitDac()
    {
        DacHandle::Config dac_config;
        dac_config.mode     = DacHandle::Mode::DMA;
        dac_config.bitdepth = DacHandle::BitDepth::
            BITS_12; /**< Sets the output value to 0-4095 */
        dac_config.chn               = DacHandle::Channel::BOTH;
        dac_config.buff_state        = DacHandle::BufferState::ENABLED;
        dac_config.target_samplerate = 48000;
        dac_.Init(dac_config);
    }

    void TimeMachineHardware::Impl::StartDac(DacHandle::DacCallback callback)
    {
        if(dac_running_)
            dac_.Stop();
        dac_.Start(internal_dac_buffer_[0],
                   internal_dac_buffer_[1],
                   dac_buffer_size_,
                   callback == nullptr ? InternalDacCallback : callback);
        dac_running_ = true;
    }

    void TimeMachineHardware::Impl::StopDac()
    {
        dac_.Stop();
        dac_running_ = false;
    }


    void TimeMachineHardware::Impl::InternalDacCallback(uint16_t **output, size_t size)
    {
        /** We could add some smoothing, interp, or something to make this a bit less waste-y */
        // std::fill(&output[0][0], &output[0][size], patch_sm_hw.dac_output_[0]);
        // std::fill(&output[1][1], &output[1][size], patch_sm_hw.dac_output_[1]);
        for(size_t i = 0; i < size; i++)
        {
            output[0][i] = patch_sm_hw.dac_output_[0];
            output[1][i] = patch_sm_hw.dac_output_[1];
        }
    }

/** Actual TimeMachineHardware implementation
 *  With the pimpl model in place, we can/should probably
 *  move the rest of the implementation to the Impl class
 */

    void TimeMachineHardware::Init()
    {
        /** Assign pimpl pointer */
        pimpl_ = &patch_sm_hw;
        /** Initialize the MCU and clock tree */
        System::Config syscfg;
        syscfg.Boost();

        auto memory = System::GetProgramMemoryRegion();
        if (memory != System::MemoryRegion::INTERNAL_FLASH)
        {
            syscfg.skip_clocks = true;
        }

        system.Init(syscfg);

        /** Memories */
        if (memory == System::MemoryRegion::INTERNAL_FLASH)
        {
            /** FMC SDRAM */
            sdram.Init();
        }

        if (memory != System::MemoryRegion::QSPI)
        {
            /** QUADSPI FLASH */
            QSPIHandle::Config qspi_config;
            qspi_config.device = QSPIHandle::Config::Device::IS25LP064A;
            qspi_config.mode   = QSPIHandle::Config::Mode::MEMORY_MAPPED;
            qspi_config.pin_config.io0 = Pin(PORTF,8);
            qspi_config.pin_config.io1 = Pin(PORTF, 9);
            qspi_config.pin_config.io2 = Pin(PORTF, 7);
            qspi_config.pin_config.io3 = Pin(PORTF, 6);
            qspi_config.pin_config.clk = Pin(PORTF, 10);
            qspi_config.pin_config.ncs = Pin(PORTF, 6);
            qspi.Init(qspi_config);
        }

        /** Audio */

        // Audio Init
        SaiHandle::Config sai_config;
        sai_config.periph          = SaiHandle::Config::Peripheral::SAI_1;
        sai_config.sr              = SaiHandle::Config::SampleRate::SAI_48KHZ;
        sai_config.bit_depth       = SaiHandle::Config::BitDepth::SAI_24BIT;
        sai_config.a_sync          = SaiHandle::Config::Sync::MASTER;
        sai_config.b_sync          = SaiHandle::Config::Sync::SLAVE;
        sai_config.a_dir           = SaiHandle::Config::Direction::RECEIVE;
        sai_config.b_dir           = SaiHandle::Config::Direction::TRANSMIT;
        sai_config.pin_config.fs   = {PORTE, 4};
        sai_config.pin_config.mclk = {PORTE, 2};
        sai_config.pin_config.sck  = {PORTE, 5};
        sai_config.pin_config.sa   = {PORTE, 6};
        sai_config.pin_config.sb   = {PORTE, 3};
        SaiHandle sai_1_handle;
        sai_1_handle.Init(sai_config);

        I2CHandle::Config i2c_cfg;
        i2c_cfg.periph         = I2CHandle::Config::Peripheral::I2C_2;
        i2c_cfg.mode           = I2CHandle::Config::Mode::I2C_MASTER;
        i2c_cfg.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
        i2c_cfg.pin_config.scl = {PORTB, 10};
        i2c_cfg.pin_config.sda = {PORTB, 11};
        I2CHandle i2c2;
        i2c2.Init(i2c_cfg);
        codec.Init(i2c2);

        AudioHandle::Config audio_config;
        audio_config.blocksize  = 48;
        audio_config.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
        audio_config.postgain   = 1.f;
        audio.Init(audio_config, sai_1_handle);
        callback_rate_ = AudioSampleRate() / AudioBlockSize();

        /** ADC Init */
        AdcChannelConfig adc_config[ADC_LAST + 7];

        /** Order of pins to match enum expectations */
        Pin adc_pins[] = {
            PIN_ADC_CTRL_1,
            PIN_ADC_CTRL_2,
            PIN_ADC_CTRL_3,
            PIN_ADC_CTRL_4,
            PIN_ADC_CTRL_8,
            PIN_ADC_CTRL_7,
            PIN_ADC_CTRL_5,
            PIN_ADC_CTRL_6,
            PIN_ADC_CTRL_9,
            PIN_ADC_CTRL_10,
            PIN_ADC_CTRL_11,
            PIN_ADC_CTRL_12
        };

        for (int i = 0; i < ADC_LAST; i++)
        {
            switch (i)
            {
                case CV_1:
                case ADC_10:
                case ADC_11:
                case ADC_12:
                    adc_config[i].InitMux(adc_pins[i], 8, MUX_A, MUX_B, MUX_C);
                    break;

                default:
                    adc_config[i].InitSingle(adc_pins[i]);
                    break;
            }
        }

        adc.Init(adc_config, ADC_LAST);

        /** Control Init */
        for (size_t i = 0; i < ADC_LAST; i++)
        {
            switch (i) {
                case CV_1:
                    for(int muxChannel = 0; muxChannel < 8; muxChannel++) {
                        controls[i].InitBipolarCv(adc.GetMuxPtr(i, muxChannel), callback_rate_);
                    }
                    break;

                case CV_2:
                case CV_3:
                case CV_4:
                case CV_5:
                case CV_6:
                case CV_7:
                case CV_8:
                    controls[i].InitBipolarCv(adc.GetPtr(i), callback_rate_);
                    break;

                case ADC_9:
                    controls[i].Init(adc.GetPtr(i), callback_rate_);
                    break;

                case ADC_10:
                case ADC_11:
                case ADC_12:
                    for(int muxChannel = 0; muxChannel < 8; muxChannel++) {
                        controls[i].Init(adc.GetMuxPtr(i, muxChannel), callback_rate_);
                    }
                    break;
            }
        }

        /** Fixed-function Digital I/O */
        user_led.Init(PIN_USER_LED, GPIO::Mode::OUTPUT);
        gate_in_1.Init(B10);
        gate_in_2.Init(B9);

        /** DAC init */
        pimpl_->InitDac();

        /** Start any background stuff */
        StartAdc();
        StartDac();
    }

    void TimeMachineHardware::StartAudio(AudioHandle::AudioCallback cb)
    {
        audio.Start(cb);
    }

    void TimeMachineHardware::StartAudio(AudioHandle::InterleavingAudioCallback cb)
    {
        audio.Start(cb);
    }

    void TimeMachineHardware::ChangeAudioCallback(AudioHandle::AudioCallback cb)
    {
        audio.ChangeCallback(cb);
    }

    void TimeMachineHardware::ChangeAudioCallback(AudioHandle::InterleavingAudioCallback cb)
    {
        audio.ChangeCallback(cb);
    }

    void TimeMachineHardware::StopAudio() { audio.Stop(); }

    void TimeMachineHardware::SetAudioBlockSize(size_t size)
    {
        audio.SetBlockSize(size);
        callback_rate_ = AudioSampleRate() / AudioBlockSize();
        for (size_t i = 0; i < ADC_LAST; i++)
        {
            controls[i].SetSampleRate(callback_rate_);
        }
    }

    void TimeMachineHardware::SetAudioSampleRate(float sr)
    {
        SaiHandle::Config::SampleRate sai_sr;
        switch (int(sr))
        {
            case 8000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_8KHZ;
                break;

            case 16000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_16KHZ;
                break;

            case 32000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_32KHZ;
                break;

            case 48000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
                break;

            case 96000:
                sai_sr = SaiHandle::Config::SampleRate::SAI_96KHZ;
                break;

            default:
                sai_sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
                break;
        }

        audio.SetSampleRate(sai_sr);
        callback_rate_ = AudioSampleRate() / AudioBlockSize();
        for (size_t i = 0; i < ADC_LAST; i++)
        {
            controls[i].SetSampleRate(callback_rate_);
        }
    }

    void TimeMachineHardware::SetAudioSampleRate(SaiHandle::Config::SampleRate sample_rate)
    {
        audio.SetSampleRate(sample_rate);
        callback_rate_ = AudioSampleRate() / AudioBlockSize();
        for (size_t i = 0; i < ADC_LAST; i++)
        {
            controls[i].SetSampleRate(callback_rate_);
        }
    }

    size_t TimeMachineHardware::AudioBlockSize()
    {
        return audio.GetConfig().blocksize;
    }

    float TimeMachineHardware::AudioSampleRate() { return audio.GetSampleRate(); }

    float TimeMachineHardware::AudioCallbackRate() { return callback_rate_; }

    void TimeMachineHardware::StartAdc() { adc.Start(); }

    void TimeMachineHardware::StopAdc() { adc.Stop(); }

    void TimeMachineHardware::ProcessAnalogControls()
    {
        for (int i = 0; i < ADC_LAST; i++)
        {
            controls[i].Process();
        }
    }

    void TimeMachineHardware::ProcessDigitalControls() {}

    float TimeMachineHardware::GetAdcValue(int idx) { return controls[idx].Value(); }

    float TimeMachineHardware::GetLevelSlider(int idx) {
        const int muxMapping[] = {5, 7, 6, 4, 3, 0, 1, 2};

        if (idx == 0) {
            return 1.0 - adc.GetMuxFloat(MISC_MUX, 3);
        } else {
            return 1.0 - adc.GetMuxFloat(LEVEL_MUX, muxMapping[idx - 1]);
        }
    }

    float TimeMachineHardware::GetLevelCV(int idx) {
        const int muxMapping[] = {3, 2, 1, 0, 4, 5, 6, 7};

        if (idx == 0)
        {
            return GetAdcValue(LEVEL_DRY_CV);
        }
        else
        {
            return ((1.0 - adc.GetMuxFloat(LEVEL_CV_MUX, muxMapping[idx - 1])) * 2.0) - 1.0;
        }
    }

    float TimeMachineHardware::GetPanKnob(int idx) {
        const int muxMapping[] = {5, 7, 6, 4, 1, 2, 0, 3};

        // The '1.0 - x' bit here is because our pan pots are wired backwards!
        if (idx == 0)
        {
            return 1.0 - adc.GetMuxFloat(MISC_MUX, 6);
        }
        else
        {
            return 1.0 - adc.GetMuxFloat(PAN_MUX, muxMapping[idx - 1]);
        }
    }

    float TimeMachineHardware::GetSpreadKnob()
    {
        return adc.GetMuxFloat(MISC_MUX, 2);
    }

    float TimeMachineHardware::GetTimeKnob()
    {
        return adc.GetMuxFloat(MISC_MUX, 4);
    }

    float TimeMachineHardware::GetFeedbackKnob()
    {
        return adc.GetMuxFloat(MISC_MUX, 1);
    }

    float TimeMachineHardware::GetHighpassKnob()
    {
        return adc.GetMuxFloat(MISC_MUX, 0);
    }

    float TimeMachineHardware::GetLowpassKnob()
    {
        return adc.GetMuxFloat(MISC_MUX, 5);
    }

    Pin TimeMachineHardware::GetPin(const PinBank bank, const int idx)
    {
        if(idx <= 0 || idx > 10)
        {
            return DUMMYPIN;
        }
        else
        {
            return kPinMap[static_cast<int>(bank)][idx - 1];
        }
    }

    void TimeMachineHardware::StartDac(DacHandle::DacCallback callback)
    {
        pimpl_->StartDac(callback);
    }

    void TimeMachineHardware::StopDac() { pimpl_->StopDac(); }

    void TimeMachineHardware::WriteCvOut(const int channel, float voltage)
    {
        pimpl_->WriteCvOut(channel, voltage);
    }

    void TimeMachineHardware::SetLed(bool state) { user_led.Write(state); }

    bool TimeMachineHardware::ValidateSDRAM()
    {
        uint32_t *sdramptr      = (uint32_t *)0xc0000000;
        uint32_t  size_in_words = 16777216;
        uint32_t  testval       = 0xdeadbeef;
        uint32_t  num_failed    = 0;

        /** Write test val */
        for (uint32_t i = 0; i < size_in_words; i++)
        {
            uint32_t *word = sdramptr + i;
            *word          = testval;
        }

        /** Compare written */
        for (uint32_t i = 0; i < size_in_words; i++)
        {
            uint32_t *word = sdramptr + i;
            if (*word != testval)
            {
                num_failed++;
            }
        }

        /** Write Zeroes */
        for (uint32_t i = 0; i < size_in_words; i++)
        {
            uint32_t *word = sdramptr + i;
            *word = 0x00000000;
        }

        /** Compare Cleared */
        for (uint32_t i = 0; i < size_in_words; i++)
        {
            uint32_t *word = sdramptr + i;
            if (*word != 0) {
                num_failed++;
            }
        }

        return num_failed == 0;
    }

    bool TimeMachineHardware::ValidateQSPI(bool quick)
    {
        uint32_t start;
        uint32_t size;
        if (quick)
        {
            start = 0x400000;
            size  = 0x4000;
        }
        else
        {
            start = 0;
            size  = 0x800000;
        }
        // Erase the section to be tested
        qspi.Erase(start, start + size);

        // Create some test data
        std::vector<uint8_t> test;
        test.resize(size);
        uint8_t *testmem = test.data();

        for (size_t i = 0; i < size; i++) {
            testmem[i] = (uint8_t) (i & 0xff);
        }

        // Write the test data to the device
        qspi.Write(start, size, testmem);

        // Read it all back and count any/all errors
        // I supppose any byte where ((data & 0xff) == data)
        // would be able to false-pass..

        size_t fail_cnt = 0;

        for (size_t i = 0; i < size; i++)
        {
            if (testmem[i] != (uint8_t) (i & 0xff)) {
                fail_cnt++;
            }
        }

        return fail_cnt == 0;
    }

} // namespace patch_sm

} // namespace daisy