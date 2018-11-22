#include "nrf24l01.hpp"

#include <cstring>

#if defined(EMBEDDED) && defined(HARDWARE_TEST)
#include <Thor/include/spi.hpp>
#include <Thor/include/gpio.hpp>

using namespace Thor::Peripheral::SPI;
using namespace Thor::Peripheral::GPIO;
#endif


using namespace NRF24L;

/**
*   Class for testing behavior of the NRF24L01 driver. The hope is that this class
*   will compile and execute both on the development system (Windows) as well as the
*   target system (microcontroller).
*/
class NRF24L01_Test : protected NRF24L01
{
public:
    
    NRF24L01_Test();
    ~NRF24L01_Test() = default;

    void init();

    void teardown();

    void reset();

    #if defined(EMBEDDED) && defined(HARDWARE_TEST)
    //Add functions only for real hardware here
    #else

    /** Manually set the return of an SPI transaction. This allows simulating the slave 
    *   device fairly cleanly and helps in mocking bad data responses.
    *   
    *   @param[in]  buffer              Buffer of containing the return data
    *   @param[in]  len                 How many bytes to set
    *   @param[in]  clear_rx_buffer     Optionally wipe the internal rx buffer before writing
    */
    void set_spi_return(uint8_t *const buffer, size_t len, bool clear_rx_buffer = true);

    /** Manually set the return of an SPI transaction. This allows simulating the slave
    *   device fairly cleanly and helps in mocking bad data responses.
    *
    *   @param[in]  val                 Return byte
    *   @param[in]  clear_rx_buffer     Optionally wipe the internal rx buffer before writing
    */
    void set_spi_return(const uint8_t &val, bool clear_rx_buffer = true);

    #endif /* HARDWARE_TEST */

    /** Expose protected function to the test interface */
    uint8_t write_register(uint8_t reg, uint8_t value)
    { 
        return NRF24L01::write_register(reg, value);
    }

    uint8_t write_register(uint8_t reg, const uint8_t* buf, size_t len)
    {
        return NRF24L01::write_register(reg, buf, len);
    }

    uint8_t read_register(uint8_t reg)
    {
        return NRF24L01::read_register(reg);
    }

    uint8_t read_register(uint8_t reg, uint8_t* buf, size_t len)
    {
        return NRF24L01::read_register(reg, buf, len);
    }


    size_t bytes_written = 0;
    uint8_t test_rx_buffer[NRF24L::SPI_BUFFER_LEN];
    uint8_t test_tx_buffer[NRF24L::SPI_BUFFER_LEN];

    #if defined(EMBEDDED) && defined(HARDWARE_TEST)
    Thor::Peripheral::SPI::SPIClass_sPtr spi;
    Thor::Peripheral::GPIO::GPIOClass_sPtr chip_enable;
    #endif

protected:

    size_t spi_write(const uint8_t *const tx_buffer, size_t &len) override;

    size_t spi_read(uint8_t *const rx_buffer, size_t &len) override;

    size_t spi_write_read(const uint8_t *const tx_buffer, uint8_t *const rx_buffer, size_t &len) override;

    void begin_transaction() override;

    void end_transaction() override;

private:

    bool spi_return_data_available = false;
    
    
};
