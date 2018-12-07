
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
const uint8_t address[5] = { 0x00, 0x00, 0x00, 0x00, 0x01 };
const std::array<uint8_t, 5> testArray = { 1, 2, 3, 4, 5 };
const std::array<char, sizeof("hello world")> testText = {"hello world"};

void senderThread(void * argument)
{
    bool initialized = false;

    NRF24L01_sPtr radio;

    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 1000000;
    spiSetup.bitOrder = BitOrder::MSB_FIRST;
    spiSetup.clockMode = ClockMode::MODE0;
    spiSetup.mode = Mode::MASTER;

    spiSetup.CS.pin = 15;
    spiSetup.CS.port = Port::PORTA;
    spiSetup.CS.alternate = Thor::Peripheral::GPIO::NOALTERNATE;
    spiSetup.CS.mode = Drive::OUTPUT_PUSH_PULL;
    spiSetup.CS.state = State::HIGH;

    spi->setChipSelectControlMode(ChipSelectMode::MANUAL);

    spi->init(spiSetup);
    spi->setPeripheralMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);

    chip_enable = std::make_shared<GPIOClass>(Port::PORTC, 1);
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
        radio->openWritePipe(address);
        radio->setPALevel(PowerAmplitude::MAX);
        radio->stopListening();
    }

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        if (initialized)
        {
            if (!radio->write(testText))
            {
                //printf("Write failed\r\n");
            }
        }
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
    }

}


void ledThread(void* argument)
{
    GPIOClass led = GPIOClass(Port::PORTA, 5);
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
