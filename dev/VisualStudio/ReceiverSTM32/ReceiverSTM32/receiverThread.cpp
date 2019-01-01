
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
const uint64_t address = 0xCCCCCCCC3C;
uint8_t rxBuffer[40];

void receiverThread(void * argument)
{
    bool initialized = false;

    NRF24L01_sPtr radio;

    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;
    GPIOClass led;
    led.init(Port::PORTB, 0);
    led.setMode(Drive::OUTPUT_PUSH_PULL, false);
    led.setState(State::LOW);

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 1000000;
    spiSetup.bitOrder = BitOrder::MSB_FIRST;
    spiSetup.clockMode = ClockMode::MODE0;
    spiSetup.mode = Chimera::SPI::Mode::MASTER;

    spiSetup.CS.pin = 7;
    spiSetup.CS.port = Port::PORTF;
    spiSetup.CS.alternate = Thor::Definitions::GPIO::NOALTERNATE;
    spiSetup.CS.mode = Drive::OUTPUT_PUSH_PULL;
    spiSetup.CS.state = State::HIGH;

    spi->setChipSelectControlMode(ChipSelectMode::MANUAL);

    spi->init(spiSetup);
    spi->setPeripheralMode(SubPeripheral::TXRX, SubPeripheralMode::BLOCKING);

    chip_enable = std::make_shared<GPIOClass>();
    chip_enable->init(Port::PORTF, 6);
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
        radio->setPALevel(PowerAmplitude::LOW);
        radio->setAutoAck(0, true);
        radio->openReadPipe(0, address);
        radio->startListening();
    }

    uint8_t pipeNum = 0xFF;
    uint8_t numBytes = 0;

    TickType_t lastTimeWoken = xTaskGetTickCount();
    for(;;)
    {
        if (radio->available(pipeNum))
        {
            numBytes = radio->readRegister(Register::RX_PW_P0);

            radio->read(rxBuffer, numBytes);
            printf("%s\r\n", rxBuffer);

            led.toggle();
        }
        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(100));
    }

}


void ledThread(void* argument)
{
    GPIOClass led;
    led.init(Port::PORTB, 7);
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
