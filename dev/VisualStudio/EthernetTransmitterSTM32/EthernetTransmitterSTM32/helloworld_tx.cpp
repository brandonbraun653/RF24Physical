#include "helloworld_tx.hpp"

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
#include "RF24Network.hpp"

using namespace NRF24L;
using namespace RF24Network;
using namespace Chimera;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static NRF24L01 radio;
static Network network(radio);

static const uint16_t this_node = 01;         // Address of our node in Octal format
static const uint16_t other_node = 00;        // Address of the other node in Octal format

static const uint32_t interval = 2000;  //ms  // How often to send 'hello world to the other unit

static uint32_t last_sent;              // When did we last send?
static uint32_t packets_sent;          // How many have we sent already

struct payload_t
{
    uint32_t ms;
    uint32_t counter;
};

void helloWorldTXThread(void *arguments)
{
    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 4000000;
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
    network.begin(90, this_node);

    uint32_t now = millis();

    for (;;)
    {
        network.update();

        now = millis();
        if ((now - last_sent) >= interval)
        {
            last_sent = now;
            printf("Sending...\r\n");
            payload_t payload = {millis(), packets_sent++};
            Header header(other_node);

            bool ok = network.write(header, &payload, sizeof(payload));

            if (ok)
            {
                printf("Ok.\r\n");
            }
            else
            {
                printf("Failed.\r\n");
            }
        }

        vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
    }
}
