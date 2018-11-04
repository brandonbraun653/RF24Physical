#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"

#include <iostream>
#include <stdint.h>

TEST_GROUP(AdvancedFunctions)
{
    void setup() override
    {

    }

    void teardown() override
    {

    }

};

TEST(AdvancedFunctions, FirstTest)
{
    CHECK_EQUAL(0, 0);
    std::cout << "Advanced Functions test" << std::endl;
}