#include "driver.hpp"



NRF24L01_Test::NRF24L01_Test()
{
    static_assert(sizeof(test_rx_buffer) == NRF24L::SPI_BUFFER_LEN, "Invalid test rx_buffer len");
    static_assert(sizeof(test_tx_buffer) == NRF24L::SPI_BUFFER_LEN, "Invalid test tx_buffer len");

    memset(test_rx_buffer, 0, sizeof(test_rx_buffer));
    memset(test_tx_buffer, 0, sizeof(test_tx_buffer));
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
// Add versions of the functions that talk to real hardware
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

size_t NRF24L01_Test::spi_write(uint8_t* tx_buffer, size_t len)
{
    if(len > sizeof(test_tx_buffer))
    {
        printf("ERROR: spi_write() was asked to send/receive way too much data");
        return 0;
    }

    return len;
}

size_t NRF24L01_Test::spi_read(uint8_t* rx_buffer, size_t len)
{
    return len;
}

size_t NRF24L01_Test::spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len)
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