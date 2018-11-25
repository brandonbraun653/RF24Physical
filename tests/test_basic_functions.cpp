#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"

#if defined(HW_TEST)
#include <Chimera/types.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/gpio.hpp>

using namespace Chimera::SPI;
using namespace Chimera::GPIO;
#endif

TEST_GROUP(BasicFunctions)
{
    #if defined(HW_TEST)
    Chimera::SPI::Setup spiSetup;
    Chimera::SPI::SPIClass_sPtr spi;
    Chimera::GPIO::GPIOClass_sPtr chip_enable;

    NRF24L01_sPtr nrf;
    #endif

    void setup() override
    {
        if (!nrf)
        {
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

            nrf = std::make_shared<NRF24L01>(this->spi, this->chip_enable);
        }
    }

    void teardown() override
    {

    }

    void reset_test()
    {

    }
};

/*-------------------------------------------------
Tests that only run on the development system
-------------------------------------------------*/
#if defined(MOD_TEST)

#endif /* MOD_TEST */

/*-------------------------------------------------
Tests that only run when connected to real hardware
-------------------------------------------------*/
#if defined(HW_TEST)
TEST(BasicFunctions, testReadWriteRegister)
{
    reset_test();

    uint8_t write_val = 0x0C;
    uint8_t read_val = 0x00;

    nrf->write_register(REG_CONFIG, write_val);
    read_val = nrf->read_register(REG_CONFIG);

    CHECK_EQUAL(write_val, read_val);
}

TEST(BasicFunctions, testBegin)
{
    reset_test();
    bool result = nrf->begin();
    CHECK_TRUE(result);
}
#endif /* HW_TEST */
