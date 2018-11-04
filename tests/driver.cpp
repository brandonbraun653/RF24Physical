#include "driver.hpp"

size_t NRF24L01_SystemTest::spi_write(uint8_t* tx_buffer, size_t len)
{
    return len;
}


size_t NRF24L01_SystemTest::spi_read(uint8_t* rx_buffer, size_t len)
{
    return len;
}


size_t NRF24L01_SystemTest::spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len)
{
    read_tx_buffer = tx_buffer;
    rx_buffer = inject_rx_buffer;
    return len;
}

void NRF24L01_SystemTest::begin_transaction()
{
    
}

void NRF24L01_SystemTest::end_transaction()
{
    
}
