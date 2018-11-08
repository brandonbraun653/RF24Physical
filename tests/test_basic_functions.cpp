#include "CppUTest/TestHarness.h"

#include "nrf24l01.hpp"
#include "driver.hpp"


TEST_GROUP(BasicFunctions)
{
    void setup() override
    {

    }

    void teardown() override
    {

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