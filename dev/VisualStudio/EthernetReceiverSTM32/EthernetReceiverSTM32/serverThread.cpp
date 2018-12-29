
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
#include "RF24Mesh.h"
#include "RF24Network.h"
#include "RF24Ethernet.h"

using namespace NRF24L;
using namespace Chimera::Threading;
using namespace Chimera::GPIO;
using namespace Chimera::SPI;

static uint32_t delayTime = 500;
const uint64_t address = 0x01;
const std::array<uint8_t, 5> testArray = { 1, 2, 3, 4, 5 };
const char * helloWorld = "hello world";

constexpr std::array<char, sizeof("hello world")> testText = {"hello world"};

IPAddress myIP(10, 10, 2, 4);
IPAddress serverIP(10, 10, 2, 2);
uint32_t serverPort = 1000;

NRF24L01 radio;
RF24Network network(radio);
RF24Mesh mesh(radio, network);
RF24EthernetClass RF24Ethernet(radio, network, mesh);

void serverThread(void * argument)
{
    Setup spiSetup;
    SPIClass_sPtr spi;
    GPIOClass_sPtr chip_enable;

    spi = std::make_shared<SPIClass>(3);

    spiSetup.clockFrequency = 8000000;
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


    EthernetClient client;
    Ethernet.begin(myIP);
    mesh.begin();

    size_t size = 0;
    volatile bool connected = false;

    for(;;)
    {

        if (!client.connected())
        {
            client.stop();
            vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
            connected = client.connect(serverIP, serverPort);
        }
        else
        {
            client.write(reinterpret_cast<const uint8_t*>(helloWorld), sizeof(helloWorld));
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
