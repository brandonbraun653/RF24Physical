#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include <stdint.h>

#include "nrf24l01_definitions.hpp"


class NRF24L01Base
{
public:

    /** User defined function that will perform an SPI write/read. This must
    *   be overwritten otherwise the program will not compile.
    *
    *   @param[in]  tx_buffer   Data buffer from which to transmit
    *   @param[in]  len         The number of bytes to write
    *
    *   @return The total number of bytes that were written
    **/
    virtual uint8_t spi_write(uint8_t* tx_buffer, size_t len) = 0;

    /** User defined function that will perform an SPI read. This must
    *   be overwritten otherwise the program will not compile.
    *
    *   @param[in]  rx_buffer   Data buffer to read information into
    *   @param[in]  len         The number of bytes to read
    *   
    *   @return The total number of bytes read. 
    **/
    virtual uint8_t spi_read(uint8_t* rx_buffer, size_t len) = 0;


    /** User defined function that will perform an SPI write/read. This must
    *   be overwritten otherwise the program will not compile.
    *
    *   @param[in]  tx_buffer   Data buffer from which to transmit
    *   @param[in]  rx_buffer   Data buffer to read information into
    *   @param[in]  len         The number of bytes to write/read
    *   
    *   @return The total number of bytes that were written/read
    **/
    virtual uint8_t spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len) = 0;


    NRF24L01Base();
    ~NRF24L01Base() = default;

protected:
    //Vars that need to be exposed to inherited classes/testing go here

private:
    //Vars only this class can see


};


#endif /* NRF24L01_HPP */