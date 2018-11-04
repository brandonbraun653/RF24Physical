#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"

#include <iostream>
#include <stdint.h>

TEST_GROUP(Reconfiguration)
{
    void setup() override
    {

    }

    void teardown() override
    {

    }

};

TEST(Reconfiguration, FirstTest)
{
    CHECK_EQUAL(0, 0);
    std::cout << "Reconfiguration test" << std::endl;
}