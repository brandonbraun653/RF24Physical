
/* C++ Includes */
#include <array>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/threading.hpp>

/* Project Includes s*/
#include "nrf24l01.hpp"

using namespace NRF24L;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static uint32_t delayTime = 500;
const uint8_t address[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
std::array<char, 33> receiveText;

void receiverThread(void * argument)
{
    bool initialized = false;

    NRF24L01_sPtr radio;

    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;
    GPIOClass led = GPIOClass(Port::PORTB, 0);
    led.mode(Drive::OUTPUT_PUSH_PULL);
    led.write(State::LOW);

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 1000000;
    spiSetup.bitOrder = BitOrder::MSB_FIRST;
    spiSetup.clockMode = ClockMode::MODE0;
    spiSetup.mode = Chimera::SPI::Mode::MASTER;

    spiSetup.CS.pin = 7;
    spiSetup.CS.port = Port::PORTF;
    spiSetup.CS.alternate = Thor::Peripheral::GPIO::NOALTERNATE;
    spiSetup.CS.mode = Drive::OUTPUT_PUSH_PULL;
    spiSetup.CS.state = State::HIGH;

    spi->setChipSelectControlMode(ChipSelectMode::MANUAL);

    spi->init(spiSetup);
    spi->setPeripheralMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);

    chip_enable = std::make_shared<GPIOClass>(Port::PORTF, 6);
    chip_enable->mode(Drive::OUTPUT_PUSH_PULL);
    chip_enable->write(State::HIGH);

    radio = std::make_shared<NRF24L01>(spi, chip_enable);

    signalThreadSetupComplete();

    if (!radio->begin())
    {
        delayTime = 100;
        initialized = false;
    }
    else
    {
        initialized = true;
        radio->setChannel(0);
        radio->setPALevel(PowerAmplitude::MAX);
        radio->openReadPipe(0, address);
        radio->startListening();
    }

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        if (initialized && radio->available())
        {
            radio->read(receiveText);
            led.toggle();
        }
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
    }

}


void ledThread(void* argument)
{
    GPIOClass led = GPIOClass(Port::PORTB, 7);
    led.mode(Drive::OUTPUT_PUSH_PULL);
    led.write(State::LOW);

    signalThreadSetupComplete();

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        led.write(State::LOW);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
        led.write(State::HIGH);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
    }
}
