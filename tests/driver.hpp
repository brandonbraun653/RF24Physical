#include "nrf24l01.hpp"

#include <iostream>
#include <memory>


#include <CppUTest/MemoryLeakDetectorMallocMacros.h>
#include <CppUTest/MemoryLeakDetectorNewMacros.h>

/**
*   This class inherits the properties of the NRF24L01 class and implements
*   the required low level drivers.
*/
class NRF24L01_SystemTest : protected NRF24L01
{
public:
    
    NRF24L01_SystemTest();
    ~NRF24L01_SystemTest() = default;

    /** Manually set the return of an SPI transaction. This allows simulating the slave 
    *   device fairly cleanly and helps in mocking bad data responses.
    *   
    *   @param[in]  buffer              Buffer of containing the return data
    *   @param[in]  len                 How many bytes to set
    *   @param[in]  clear_rx_buffer     Optionally wipe the internal rx buffer before writing
    */
    void set_spi_return(uint8_t * const buffer, size_t len, bool clear_rx_buffer = true);

    /** Manually set the return of an SPI transaction. This allows simulating the slave
    *   device fairly cleanly and helps in mocking bad data responses.
    *
    *   @param[in]  val                 Return byte
    *   @param[in]  clear_rx_buffer     Optionally wipe the internal rx buffer before writing
    */
    void set_spi_return(const uint8_t &val, bool clear_rx_buffer = true);


    /** Expose protected function to the test interface */
    uint8_t write_register(uint8_t reg, uint8_t value) { return NRF24L01::write_register(reg, value); }

    uint8_t test_rx_buffer[NRF24L_SPI_BUFFER_LEN];
    uint8_t test_tx_buffer[NRF24L_SPI_BUFFER_LEN];

protected:

    size_t spi_write(uint8_t* tx_buffer, size_t len) override;


    size_t spi_read(uint8_t* rx_buffer, size_t len) override;


    size_t spi_write_read(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t len) override;


    void begin_transaction() override;


    void end_transaction() override;

private:

    bool spi_return_data_available = false;

    

};

typedef std::shared_ptr<NRF24L01_SystemTest> NRF24L01_SystemTest_sPtr;