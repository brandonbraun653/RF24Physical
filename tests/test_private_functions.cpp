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

TEST(PrivateFunctions, write_register_NormalCase)
{
    /*-------------------------------------------------
    Was the correct data returned?
    -------------------------------------------------*/
    {
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
        reset_test();

        /* Test Variables */
        uint8_t reg = NRF24L_REG_STATUS;
        uint8_t payload = 0x34;

        /* Expected Results */
        uint8_t exp0 = (NRF24L_CMD_W_REGISTER | (NRF24L_CMD_REGISTER_MASK & reg));
        uint8_t exp1 = payload;

        /* Call FUT and validate the correct information was written to the tx_buffers */
        nrf.write_register(reg, payload);

        CHECK_EQUAL(exp0, nrf.test_tx_buffer[0]);
        CHECK_EQUAL(exp1, nrf.test_tx_buffer[1]);
        CHECK_EQUAL(0, nrf.test_tx_buffer[2]);
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

#endif /* EMBEDDED && HARDWARE_TEST */
