#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"

TEST_GROUP(NRF24L01_Init)
{
    void setup() override
    {

    }

    void teardown() override
    {

    }

};

TEST(NRF24L01_Init, FirstTest)
{
    CHECK_EQUAL(0, 0);
}