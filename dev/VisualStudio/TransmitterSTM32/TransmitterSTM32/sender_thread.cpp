
/* C++ Includes */
#include <array>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/threading.hpp>

/* Project Includes */
#include "nrf24l01.hpp"

using namespace NRF24L;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static uint32_t delayTime = 500;
const uint64_t address = 0xCCCCCCCC3C;

constexpr std::array<char, sizeof("hello world")> testText = {"hello world"};

char helloWorld[] = "I pity the fool.\r\n";

void senderThread(void * argument)
{
    bool initialized = false;

    NRF24L01_sPtr radio;

    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 12000000;
    spiSetup.bitOrder = BitOrder::MSB_FIRST;
    spiSetup.clockMode = ClockMode::MODE0;
    spiSetup.mode = Chimera::SPI::Mode::MASTER;

    spiSetup.CS.pin = 15;
    spiSetup.CS.port = Port::PORTA;
    spiSetup.CS.alternate = Thor::Definitions::GPIO::NOALTERNATE;
    spiSetup.CS.mode = Drive::OUTPUT_PUSH_PULL;
    spiSetup.CS.state = State::HIGH;

    spi->setChipSelectControlMode(ChipSelectMode::MANUAL);

    spi->init(spiSetup);
    spi->setPeripheralMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);

    chip_enable = std::make_shared<GPIOClass>();
    chip_enable->init(Port::PORTC, 1);
    chip_enable->setMode(Drive::OUTPUT_PUSH_PULL, false);
    chip_enable->setState(State::HIGH);

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
        radio->setChannel(90);
        radio->openWritePipe(address);
        radio->setPALevel(PowerAmplitude::LOW);
        radio->setAutoAck(0, true);
        radio->stopListening();
    }

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        if (initialized)
        {
            radio->writeFast(reinterpret_cast<uint8_t*>(helloWorld), sizeof(helloWorld), false);

            if (radio->txStandBy(100))
            {
                printf("Write succeeded\r\n");
            }
            else
            {
                printf("Write failed with error code: %d\r\n", (int)radio->getFailureCode());
            }
        }
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
    }

}


void ledThread(void* argument)
{
    GPIOClass led;
    led.init(Port::PORTA, 5);
    led.setMode(Drive::OUTPUT_PUSH_PULL, false);
    led.setState(State::LOW);

    signalThreadSetupComplete();

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        led.setState(State::LOW);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
        led.setState(State::HIGH);
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(delayTime));
    }
}
