#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "command_processor.hpp"
#include "cy22150.hpp"
#include "pico_cy22150.pio.h"
#include "tiny-json.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for 
// information on GPIO assignments
//
#define I2C_PORT    i2c0
#define I2C_SDA     8
#define I2C_SCL     9
#define I2C_ADDR    0x69    // cy22150 i2c address

/**
 * @brief  Print the error to the stdout in json format.
 * @param  command  Structure containing the returned error.
 */
void show_error(command_t command)
{
    std::cout << 
        R"({)" << 
        R"(  "command_number":)" << command.command_number << "," 
        R"(  "error":)"          << R"(")"  << command.error.value() << R"(")" <<
        R"(})" << std::endl;
}

/**
 * @brief  Acknowledges the given command by pringing the 
 *         current DDS state.
 * @param  command_number   Identifier for command being acked.
 * @param  dds              Current dds from which state is being pulled.
 */
void ack_command(int command_number, CY22150 dds)
{
    std::cout << 
        R"({)" << 
        R"(  "command_number":)" <<  command_number << "," 
        R"(  "frequency":)"      <<  static_cast<uint32_t>(dds.get_frequency()) << ","
        R"(  "enable_out":)"     << (dds.get_enabled() ? "true" : "false") <<
        R"(})" << std::endl;
}

/**
 * @brief  Main routine.
 */
int main()
{
    std::vector<command_t> commands {};

    // Initialization.
    //
    stdio_init_all();

    // Set system clock to 100 MHz for easy division.
    //
    set_sys_clock_hz(100000000 /* 100 MHz */, true);

    // Choose which PIO instance to use (there are two instances)
    //
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    //
    uint offset = pio_add_program(pio, &pico_cy22150_program);

    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    //
    uint sm = pio_claim_unused_sm(pio, true);

    // Now that you have the pio, the program offset, and the state machine
    // you can initialize the pio program.
    //
    // The first three are ones you'll need any time you do this.
    // The rest of the parameters are user defined and program
    // specific.
    //
    float pio_frequency_hz = 25000000;   // 25 MHz
    pico_cy22150_program_init(pio, sm, offset, pio_frequency_hz);

    // I2C Initialisation. Using it at 400Khz.
    //
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    i2c_init(I2C_PORT, 100*1000);
   
    // Confirm you can address the cy22150 chip by trying
    // to read the i2c address.
    //
    uint8_t rxdata;
    if (i2c_read_blocking(I2C_PORT, CY22150::I2C_ADDRESS, &rxdata, 1, false) < 0)
        std::cout << "CY22150 chip not found at address " << I2C_ADDR << std::endl;
    else
        std::cout << "CY22150 chip found at address " << I2C_ADDR << std::endl;

    // Create an instance of the frequency generator.
    //
    // Because of the way the clock frequency is generated, the CY22150 
    // clock frequency is 1/2 the pio frequency.
    //
    CY22150 cy22150(I2C_PORT, pio_frequency_hz / 2);
    cy22150.init();

    // Create an instance of the command processor and
    // start the main loop.
    //
    CommandProcessor command_processor;
    while (true)
    {
        command_processor.loop();

        if (command_processor.command_is_available())
        {
            // If there is a command available, set the values, then
            // commit them.
            //
            // An error cancels any action so just loop back to the
            // top of the loop.
            //
            command_t command = command_processor.get_command();

            if (command.error.has_value())
            {
                show_error(command);
                continue;
            }

            if (command.frequency_hz.has_value())
            {
                float frequency = static_cast<float>(command.frequency_hz.value());
                cy22150.set_frequency(frequency);
            }
            
            if (command.enable_out.has_value())
            {
                cy22150.enable_clock(command.enable_out.value());
            }

            cy22150.commit();

            // All went well so acknowledge the command.
            //
            ack_command(command.command_number, cy22150);
        }
    }
}
