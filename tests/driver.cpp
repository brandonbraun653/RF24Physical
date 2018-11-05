#include "driver.hpp"





NRF24L01_SystemTest::NRF24L01_SystemTest()
{
    static_assert(sizeof(test_rx_buffer) == NRF24L_SPI_BUFFER_LEN, "Invalid test rx_buffer len");
    static_assert(sizeof(test_tx_buffer) == NRF24L_SPI_BUFFER_LEN, "Invalid test tx_buffer len");

    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    memset(test_tx_buffer, 0, sizeof(test_tx_buffer));
}

void NRF24L01_SystemTest::set_spi_return(uint8_t * const buffer, size_t len, bool clear_rx_buffer)
{
    if(!buffer)
    {
        std::cout << "ERROR: set_spi_return() was passed a null pointer" << std::endl;
        return;
    }

    if(len > NRF24L_SPI_BUFFER_LEN)
    {
        std::cout << "ERROR: set_spi_return() exceeded length of internal buffers" << std::endl;
        return;
    }

    if(clear_rx_buffer)
    {
        memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    }

    memcpy(test_rx_buffer, buffer, len);
    spi_return_data_available = true;
}

void NRF24L01_SystemTest::set_spi_return(const uint8_t &val, bool clear_rx_buffer)
{
    if(clear_rx_buffer)
    {
        memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    }

    memcpy(test_rx_buffer, &val, 1);
    spi_return_data_available = true;
}

size_t NRF24L01_SystemTest::spi_write(uint8_t* tx_buffer, size_t len)
{
    if(len > sizeof(test_tx_buffer))
    {
        std::cout << "ERROR: spi_write() was asked to send/receive way too much data" << std::endl;
        return 0;
    }

    return len;
}

size_t NRF24L01_SystemTest::spi_read(uint8_t* rx_buffer, size_t len)
{
    return len;
}

size_t NRF24L01_SystemTest::spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len)
{
    if((len > sizeof(test_tx_buffer)) || (len > sizeof(test_rx_buffer)))
    {
        std::cout << "ERROR: spi_write_read() was asked to send/receive way too much data" << std::endl;
        return 0;
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

void NRF24L01_SystemTest::begin_transaction()
{
    //Do nothing. The embedded system will handle this as needed.
}

void NRF24L01_SystemTest::end_transaction()
{
    //Do nothing. The embedded system will handle this as needed.
}
