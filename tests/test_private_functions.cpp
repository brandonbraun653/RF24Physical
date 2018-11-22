#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"


#include <stdint.h>


TEST_GROUP(PrivateFunctions)
{
    NRF24L01_Test nrf;

    void setup() override
    {
        nrf.init();
    }

    void teardown() override
    {
        nrf.teardown();
    }

    void reset_test()
    {
        nrf.reset();
    }
};

/*-------------------------------------------------
Tests that run on either the development system or target system
-------------------------------------------------*/
#if defined(WIN32) || defined(EMBEDDED)



#endif /* WIN32 || EMBEDDED */

/*-------------------------------------------------
Tests that only run on the development system
-------------------------------------------------*/
#if defined(WIN32)

TEST(PrivateFunctions, write_register_single_value)
{
    /*-------------------------------------------------
    Was the correct data returned?
    -------------------------------------------------*/
    {
        printf("Verify the correct data will be returned for a mocked write\n");
        reset_test();

        /* Test Variables */
        uint8_t status_code;
        uint8_t expected_result = 0x33;

        /*-------------------------------------------------
        Mock the return data and call FUT
        -------------------------------------------------*/
        nrf.set_spi_return(expected_result);
        status_code = nrf.write_register(0x88, 0x43);

        CHECK_EQUAL(expected_result, status_code);
    }

    /*-------------------------------------------------
    Was the correct data transmitted?
    -------------------------------------------------*/
    {
        printf("Verify the correct data will be sent for a mocked write\n");
        reset_test();

        /* Test Variables */
        uint8_t reg = NRF24L::REG_STATUS;
        uint8_t payload = 0x34;

        /* Expected Results */
        uint8_t exp0 = (NRF24L::CMD_W_REGISTER | (NRF24L::CMD_REGISTER_MASK & reg));
        uint8_t exp1 = payload;

        /* Call FUT and validate the correct information was written to the tx_buffers */
        nrf.write_register(reg, payload);

        CHECK_EQUAL(exp0, nrf.test_tx_buffer[0]);
        CHECK_EQUAL(exp1, nrf.test_tx_buffer[1]);
        CHECK_EQUAL(0, nrf.test_tx_buffer[2]);
    }

}

TEST(PrivateFunctions, write_register_multiple_values)
{
    /*-------------------------------------------------
    Normal Case
    -------------------------------------------------*/
    {
        /*-------------------------------------------------
        Verify expected return code
        -------------------------------------------------*/
        {
            printf("Verify the correct return code for a multi-byte write\n");
            reset_test();

            uint8_t status_code = 0u;
            uint8_t expected = 0u;
            uint8_t multiple_vals[] = { 0x33, 0x44, 0x55 };

            nrf.set_spi_return(expected);
            status_code = nrf.write_register(0x88, multiple_vals, sizeof(multiple_vals));

            CHECK_EQUAL(expected, status_code);
        }

        /*-------------------------------------------------
        Verify data is transmitted correctly
        -------------------------------------------------*/
        {
            printf("Verify the correct multi-byte data is transmitted\n");
            reset_test();

            /* Test Variables */
            uint8_t reg = NRF24L::REG_STATUS;
            uint8_t multiple_vals[] = { 0x33, 0x44, 0x55 };

            /* Expected Results */
            uint8_t exp0 = (NRF24L::CMD_W_REGISTER | (NRF24L::CMD_REGISTER_MASK & reg));

            /* Call FUT and validate the correct information was written to the tx_buffers */
            nrf.write_register(reg, multiple_vals, sizeof(multiple_vals));

            CHECK_EQUAL(exp0, nrf.test_tx_buffer[0]);
            CHECK_FALSE(memcmp(&nrf.test_tx_buffer[1], multiple_vals, sizeof(multiple_vals)));
        }
    }

    /*-------------------------------------------------
    Abnormal transmit lengths
    -------------------------------------------------*/
    {
        printf("Verify too much data gets truncated\n");
        reset_test();

        uint8_t reg = NRF24L::REG_STATUS;
        uint8_t way_too_many_bytes[NRF24L::PAYLOAD_LEN + 10];
        memset(way_too_many_bytes, 0xAA, sizeof(way_too_many_bytes));

        nrf.write_register(reg, way_too_many_bytes, sizeof(way_too_many_bytes));

        CHECK_EQUAL(NRF24L::PAYLOAD_LEN + 1, nrf.bytes_written);
        CHECK_FALSE(memcmp(&nrf.test_tx_buffer[1], way_too_many_bytes, NRF24L::PAYLOAD_LEN));
    }
}

TEST(PrivateFunctions, read_register_single_value)
{
    /*-------------------------------------------------
    Normal Case
    -------------------------------------------------*/
    {

    }

}

TEST(PrivateFunctions, read_register_multiple_values)
{
    /*-------------------------------------------------
    Normal Case
    -------------------------------------------------*/
    {

    }

}

#endif /* WIN32 */

/*-------------------------------------------------
Tests that only run on the target system
-------------------------------------------------*/
#if defined(EMBEDDED)

#endif /* EMBEDDED */

/*-------------------------------------------------
Tests that only run when connected to real hardware
-------------------------------------------------*/
#if defined(EMBEDDED) && defined(HARDWARE_TEST)
TEST(PrivateFunctions, read_register_single_value)
{
    {
        nrf.chip_enable->write(State::LOW);
        uint8_t reset_config_value = 0x00;
        uint8_t actual_config_value = nrf.read_register(NRF24L::REG_CONFIG);

        CHECK_EQUAL(reset_config_value, actual_config_value);
    }
}
#endif /* EMBEDDED && HARDWARE_TEST */
