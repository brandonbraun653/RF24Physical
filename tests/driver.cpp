#include "driver.hpp"

#if defined(HARDWARE_TEST) && defined(EMBEDDED)
using namespace Chimera::SPI;
using namespace Chimera::GPIO;
#endif 


NRF24L01_Test::NRF24L01_Test()
{
    static_assert(sizeof(test_rx_buffer) == NRF24L::SPI_BUFFER_LEN, "Invalid test rx_buffer len");
    static_assert(sizeof(test_tx_buffer) == NRF24L::SPI_BUFFER_LEN, "Invalid test tx_buffer len");

    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    memset(test_tx_buffer, 0, sizeof(test_tx_buffer));

    #if defined(HARDWARE_TEST) && defined(EMBEDDED)
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
    #endif
}

void NRF24L01_Test::init()
{

}

void NRF24L01_Test::teardown()
{

}

void NRF24L01_Test::reset()
{
    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    memset(test_tx_buffer, 0, sizeof(test_tx_buffer));

    bytes_written = 0;
}


#if defined(HARDWARE_TEST) && defined(EMBEDDED)
size_t NRF24L01_Test::spi_write(const uint8_t *const tx_buffer, size_t &len)
{
    spi->writeBytes(tx_buffer, len, false);
    return len;
}

size_t NRF24L01_Test::spi_read(uint8_t *const rx_buffer, size_t &len)
{
    spi->readBytes(rx_buffer, len, false);
    return len;
}

size_t NRF24L01_Test::spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t &len)
{   
    spi->readWriteBytes(tx_buffer, rx_buffer, len, false);
    return len;
}

void NRF24L01_Test::begin_transaction()
{
    spi->setChipSelect(State::LOW);
}

void NRF24L01_Test::end_transaction()
{
    spi->setChipSelect(State::HIGH);
}
#else

void NRF24L01_Test::set_spi_return(uint8_t * const buffer, size_t len, bool clear_rx_buffer)
{
    if(!buffer)
    {
        printf("ERROR: set_spi_return() was passed a null pointer");
        return;
    }
    else if(len > NRF24L::SPI_BUFFER_LEN)
    {
        printf("ERROR: set_spi_return() exceeded length of internal buffers");
        return;
    }

    if(clear_rx_buffer)
    {
        memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    }

    memcpy(test_rx_buffer, buffer, len);
    spi_return_data_available = true;
}

void NRF24L01_Test::set_spi_return(const uint8_t &val, bool clear_rx_buffer)
{
    if(clear_rx_buffer)
    {
        memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    }

    memcpy(test_rx_buffer, &val, 1);
    spi_return_data_available = true;
}

size_t NRF24L01_Test::spi_write(const uint8_t *const tx_buffer, size_t &len)
{
    if(len > sizeof(test_tx_buffer))
    {
        printf("ERROR: spi_write() was asked to send/receive way too much data");
        return 0;
    }

    return len;
}

size_t NRF24L01_Test::spi_read(uint8_t *const rx_buffer, size_t &len)
{
    return len;
}

size_t NRF24L01_Test::spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t &len)
{
    if((len > sizeof(test_tx_buffer)) || (len > sizeof(test_rx_buffer)))
    {
        printf("ERROR: spi_write_read() was asked to send/receive way too much data");
        return 0;
    }
    else
    {
        bytes_written = len;
    }

    /*-------------------------------------------------
    Set the return data for the calling function
    -------------------------------------------------*/
    if(spi_return_data_available)
    {
        memcpy(rx_buffer, test_rx_buffer, len);
        spi_return_data_available = false;
    }
    else
    {
        memset(rx_buffer, 0, len);
    }

    /*-------------------------------------------------
    Copy the transmit buffer for validation later 
    -------------------------------------------------*/
    memset(test_tx_buffer, 0, sizeof(test_tx_buffer));
    memcpy(test_tx_buffer, tx_buffer, len);

    return len;
}

void NRF24L01_Test::begin_transaction()
{
    //Do nothing. The embedded system will handle this as needed.
}

void NRF24L01_Test::end_transaction()
{
    //Do nothing. The embedded system will handle this as needed.
}

#endif /* HARDWARE_TEST */

