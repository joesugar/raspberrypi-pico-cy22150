#pragma once

#include <utility>

#include "hardware/structs/i2c.h"

class CY22150
{
public:

    static const uint8_t I2C_ADDRESS = 0x69;
    static constexpr float FREQ_DEFAULT = 4000000.0;

    /**
     * @brief  Constructor
     * 
     * @param  i2c            I2C interface to be used to communicate 
     *                        wtih the chip
     * @param  clock_freq_hz  Clock signal frequency, in Hz
     */
    CY22150(i2c_inst_t* i2c, float clock_freq_hz, float frequency = FREQ_DEFAULT)
        :i2c_(i2c)
        ,clock_freq_hz_(clock_freq_hz)
        ,current_state_({frequency, DISABLE })
        ,temp_state_({frequency, DISABLE })
        ,default_state_({frequency, ENABLE })
    { };

    /**
     * @brief  Initialize the chip registers.
     * 
     * @note   Must be called before using the chip.
     */
    auto init() -> void
    {
        // Initialize the clock drive.
        //
        int8_t xdrv = 
            (clock_freq_hz_ <=   1000000) ? 0x00 :
            (clock_freq_hz_ <=  25000000) ? 0x20 :
            (clock_freq_hz_ <=  50000000) ? 0x28 :
            (clock_freq_hz_ <=  90000000) ? 0x30 :
            (clock_freq_hz_ <= 133000000) ? 0x38 :
            0x00;
        write_reg(XDRV, xdrv);

        // Set the clock generator to the default state.
        //
        disable_clock_commit();
        set_frequency(default_state_.frequency);
        set_enabled(default_state_.enable);
        commit();
    }

    /**
     * @brief  Set the flag to enable/disable the clock.
     * @param  enable  Enable clock if true, false otherwise.
     */
    auto set_enabled(bool enable) -> void
    {
        temp_state_.enable = enable;
    }

    /**
     * @brief  Return the current clock state.
     */
    auto get_enabled() -> bool
    {
        return current_state_.enable;
    }

    /**
     * @brief  Set the clock frequency.
     * @param  frequency  Desired clock frequency, in Hz.
     */
    auto set_frequency(float frequency) -> void
    {
        temp_state_.frequency = frequency;
    }

    /**
     * @brief  Return the current clock frequency.
     */
    auto get_frequency() -> float
    {
        return current_state_.frequency;
    }

    /**
     * @brief  Commit changes to the CY22150.
     */
    auto commit() -> void
    {
        // Commit state to the CY22150.
        //
        current_state_.frequency = temp_state_.frequency;
        current_state_.enable = temp_state_.enable;

        disable_clock_commit();
        float frequency = frequency_commit(current_state_.frequency);
        current_state_.enable ? enable_clock_commit() : disable_clock_commit();

        // Update the state structures in case the actual calculated
        // frequency is not the same as the requested frequency.
        //
        current_state_.frequency = frequency;
        temp_state_.frequency = frequency;
    }

private:

    /**
     * @brief  Set the flag to enable/disable the clock.
     * @param  enable  Enable clock if true, false otherwise.
     */
    auto disable_clock_commit() -> void
    {
        commit_clock_enable(NONE);
    }

    /**
     * @brief  Set the flag to enable/disable the clock.
     * @param  enable  Enable clock if true, false otherwise.
     */
    auto enable_clock_commit() -> void
    {
        commit_clock_enable(CLOCK2);
    }

    /**
     * @brief  Enable the clock output
     * @param  clock_mask  Mask identifying the clocks to be enabled.
     * 
     * @note   Sets the registers to divide the VCO by N
     * @note   Only enables clocks 1 - 4.
     */
    auto commit_clock_enable(uint8_t clock_mask) -> void
    {
        // Disable all but clocks 1 - 4
        //
        clock_mask = clock_mask & 0x0F;

        // If the clock mask == 0x00 that means disable the clock.
        //
        if (clock_mask != 0x00)
        {
            // Indices into the array containing values to be written
            // to the registers.
            //
            const int reg44 = 0;
            const int reg45 = 1;
            const int reg46 = 2;

            uint8_t regs[3] = {0x00, 0x00, 0x00 };
            if ((clock_mask & 0x01) == 0x01)
            {
                regs[reg44] |= 0x20;
            }
            if ((clock_mask & 0x02) == 0x02)
            {
                regs[reg44] |= 0x04;
            }
            if ((clock_mask & 0x04) == 0x04)
            {
                regs[reg45] |= 0x80;
            }
            if ((clock_mask & 0x08) == 0x08)
            {
                regs[reg45] |= 0x10;
            }
            regs[reg46] = 0x3F;

            // Write to the registers.
            //
            write_reg(REG44, regs[reg44]);
            write_reg(REG45, regs[reg45]);
            write_reg(REG46, regs[reg46]);
        }
        write_reg(CLKOE, clock_mask);
    }

    /**
     * @brief  Calculate constants and set the clock frequency.
     * 
     * @param  frequency_hz  Desire clock frequency, in Hz
     * 
     * @return Actual programmed frequency.
     */
    auto frequency_commit(float frequency_hz) -> float
    {  
        float q_min = 2; 
        float q_max = (int)(clock_freq_hz_ / 250000.0);
        if (q_max > 127.0) { q_max = 127.0; }

        float d_min = (int)(1.0 + 100000000.0 / frequency_hz); 
        float d_max = (int)(1.0 + 400000000.0 / frequency_hz) - 1.0;
        if (d_max > 127.0) { d_max = 127.0; }

        float p_min = 16.0;
        float p_max = 1023.0;

        float f_test, f_diff; 
        float f_track = frequency_hz; 
        float p_test, q_test, d_test; 
        
        uint16_t p, q, d; 
        for (q_test = q_min; (q_test <= q_max) && (f_track > 0.5); q_test++) 
        { 
            for (d_test = d_max; (d_test >= d_min) && (f_track > 0.5); d_test--) 
            { 
                // Calculating the p value.  It has to fall between 16 and 1023.
                // If the calculated value does not fall in that range, just
                // bound it.
                //
                p_test = (frequency_hz / clock_freq_hz_) * q_test * d_test; 
                p_test = ((p_test - (int)p_test) > .5) ? (int)(p_test + 1.0) : (int)p_test;
                if (p_test < p_min)
                    p_test = p_min;
                else if (p_test > p_max)
                    p_test = p_max;

                // Now calculate the prorammed frequency and see if it's
                // a better fit that the previous.
                //
                f_test = (clock_freq_hz_ * p_test) / (q_test * d_test); 
                f_diff = ((f_test - frequency_hz) > 0.0) ? (f_test - frequency_hz) : (frequency_hz - f_test); 
                if (f_diff < f_track) 
                { 
                    f_track = f_diff; 
                    p = static_cast<uint16_t>(p_test); 
                    q = static_cast<uint16_t>(q_test); 
                    d = static_cast<uint16_t>(d_test); 
                } 
            } 
        }
        return frequency_commit(q, p, d);
    }

    /**
     * @brief  Sets the clock frequency according to the constraints
     *         listed in the datasheet.
     * 
     * @param  q_total  Value for the q counter (2 - 129)
     * @param  p_total  Value for the p counter (8 - 2055)
     * 
     * @return Actual programmed frequency.
     */
    auto frequency_commit(uint16_t q_total, uint16_t p_total, uint16_t divider) -> float
    {
        // Set the q counter value.
        //
        float q_total_f   = static_cast<float>(q_total);
        float q_total_max = static_cast<uint8_t>(clock_freq_hz_ / 250000.0);
        
        if (q_total_f > q_total_max)
            q_total_f = q_total_max;
        if (q_total_f < 2.0)
            q_total_f = 2.0;
        if (q_total_f > 129.0)
            q_total_f = 129;

        q_total = 1 + static_cast<uint16_t>(q_total_f - 1.0);

        // Set the p counter values.
        //       
        float p_total_f   = static_cast<float>(p_total);
        float p_total_max = (400000000.0 / clock_freq_hz_) * q_total; 
        float p_total_min = (100000000.0 / clock_freq_hz_) * q_total;

        if (p_total_f > p_total_max)
            p_total_f = p_total_max;
        if (p_total_f < p_total_min)
            p_total_f = p_total_min;
        if (p_total_f > 1023.0)     // 2055
            p_total_f = 1023.0;     // 2055
        if (p_total_f < 16.0)       // 8
            p_total_f = 16.0;       // 8

        p_total = 1 + static_cast<uint16_t>(p_total_f - 1.0);

        // Set the divider value.
        //
        if (divider < 4)
            divider = 4;
        if (divider > 127)
            divider = 127;

        // Calculate charge pump values.
        //
        uint8_t cp = (p_total <  45) ? 0x00 :
                     (p_total < 480) ? 0x01 :
                     (p_total < 640) ? 0x02 :
                     (p_total < 800) ? 0x03 :
                      0x04;
    
        // Write vlaues to the registers.
        //
        uint8_t  po = p_total % 2;
        uint16_t pb = ((p_total - po) / 2) - 4;
        uint8_t  q  = (q_total - 2);

        uint8_t reg40 = 0xC0 | (cp << 2) | static_cast<uint8_t>(pb >> 8);
        uint8_t reg41 = static_cast<uint8_t>(pb & 0x00FF);
        uint8_t reg42 = (po << 7) | q;
        uint8_t dvdr  = 0x00 | static_cast<uint8_t>(divider);

        write_reg(REG40, reg40);
        write_reg(REG41, reg41);
        write_reg(REG42, reg42);
        write_reg(DVDR,  dvdr);

        // Return the actual programmed frequency.
        //
        float numer = (clock_freq_hz_ * static_cast<float>(p_total));
        float denom = (static_cast<float>(q_total) * static_cast<float>(divider));
        float frequency = numer / denom;
        return frequency;
    }

    /**
     * @brief  Write to an 8 bit register.
     * 
     * @param  address  Register address to which to write.
     * @param  value    Value to be written.
     */
    void write_reg(uint16_t address, uint8_t value)
    {
        uint8_t data[2];

        // Need to make sure the address is big-endian.
        //
        data[0] = address & 0x00FF;
        data[1] = value;

        i2c_write_blocking(i2c_, I2C_ADDRESS, data, sizeof(data), false);
    }

    // Register definitions.
    //
    static const uint16_t CLKOE = 0x09;
    static const uint16_t DVDR  = 0x0C;
    static const uint16_t XDRV  = 0x12;

    static const uint16_t REG09 = 0x09;
    static const uint16_t REG0C = 0x0C;
    static const uint16_t REG12 = 0x12;
    static const uint16_t REG40 = 0x40;
    static const uint16_t REG41 = 0x41;
    static const uint16_t REG42 = 0x42;
    static const uint16_t REG44 = 0x44;
    static const uint16_t REG45 = 0x45;
    static const uint16_t REG46 = 0x46;

    static const uint8_t NONE   = 0x00;
    static const uint8_t CLOCK2 = 0x02;

    static const bool ENABLE  = true;
    static const bool DISABLE = false;

    i2c_inst_t* i2c_;
    float clock_freq_hz_;

    // State definitions.
    //
    using cy22150_state = struct {
        float frequency;
        bool enable;
    };
    
    cy22150_state current_state_;
    cy22150_state temp_state_;
    cy22150_state default_state_;
};