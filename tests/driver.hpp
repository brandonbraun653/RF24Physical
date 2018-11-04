#include "nrf24l01.hpp"

/*
This class inherits the properties of the NRF24L01 class and implements
the required low level drivers.
*/
class NRF24L01_SystemTest : protected NRF24L01
{
public:
    uint8_t * inject_rx_buffer;
    uint8_t * read_tx_buffer;

protected:

    size_t spi_write(uint8_t* tx_buffer, size_t len) override;


    size_t spi_read(uint8_t* rx_buffer, size_t len) override;


    size_t spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len) override;


    void begin_transaction() override;


    void end_transaction() override;

private:

};