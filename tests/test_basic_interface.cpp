#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"

#include <iostream>
#include <stdint.h>

TEST_GROUP(BasicInterface)
{
    void setup() override
    {

    }

    void teardown() override
    {

    }

};

TEST(BasicInterface, FirstTest)
{
    CHECK_EQUAL(0, 0);
    std::cout << "Basic Interface test" << std::endl;
}