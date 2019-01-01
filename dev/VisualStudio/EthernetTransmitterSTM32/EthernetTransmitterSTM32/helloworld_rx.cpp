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
#include "RF24Network.h"

using namespace NRF24L;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static NRF24L01 radio;
static RF24Network network(radio);

static const uint16_t this_node = 00;     // Address of our node in Octal format ( 04,031, etc)
static const uint16_t other_node = 01;    // Address of the other node in Octal format

struct payload_t
{
    uint32_t ms;
    uint32_t counter;
};

void helloWorldRXThread(void *arguments)
{
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

    signalThreadSetupComplete();
    TickType_t lastTimeWoken = xTaskGetTickCount();


    radio = NRF24L01(spi, chip_enable);
    radio.begin();
    network.begin(90, this_node);

    for(;;)
    {
        network.update();

        if (network.available())
        {
            while (network.available())
            {
                RF24NetworkHeader header;
                payload_t payload;

                network.read(header, &payload, sizeof(payload));
                printf("Received packet # %d at %d.\r\n", (int)payload.counter, (int)payload.ms);
            }
        }

        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
    }
}
