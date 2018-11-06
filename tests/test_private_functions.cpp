#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"

//#include <iostream>
#include <stdint.h>


//TEST_GROUP(PrivateFunctions)
//{
//    NRF24L01_SystemTest_sPtr nrf;
//
//    void setup() override
//    {
//        nrf = std::make_shared<NRF24L01_SystemTest>();
//    }
//
//    void teardown() override
//    {
//    }
//
//    void reset_test()
//    {
//        
//    }
//};
//
//TEST(PrivateFunctions, write_register_NormalCase)
//{
//    /*-------------------------------------------------
//    Was the correct data returned?
//    -------------------------------------------------*/
//    {
//        reset_test();
//
//        /* Test Variables */
//        uint8_t status_code;
//        uint8_t expected_result = 0x33;
//        
//        /* Mock the return data and call FUT */
//        nrf->set_spi_return(expected_result);
//        status_code = nrf->write_register(0x88, 0x43);
//
//        CHECK_EQUAL(expected_result, status_code);
//    }
//    
//
//    /*-------------------------------------------------
//    Was the correct data transmitted?
//    -------------------------------------------------*/
//    {
//        reset_test();
//
//        /* Test Variables */
//        uint8_t reg = NRF24L_REG_STATUS;
//        uint8_t payload = 0x34;
//
//        /* Expected Results */
//        uint8_t exp0 = (NRF24L_CMD_W_REGISTER | (NRF24L_CMD_REGISTER_MASK & reg));
//        uint8_t exp1 = payload;
//
//        /* Call FUT and validate the correct information was written to the tx_buffers */
//        nrf->write_register(reg, payload);
//
//        CHECK_EQUAL(exp0, nrf->test_tx_buffer[0]);
//        CHECK_EQUAL(exp1, nrf->test_tx_buffer[1]);
//        CHECK_EQUAL(0, nrf->test_tx_buffer[2]);
//    }
//    
//    
//
//
//}