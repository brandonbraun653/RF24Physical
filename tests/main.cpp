#include <CppUTest/CommandLineTestRunner.h>
#include <driver.hpp>

#if defined(HW_TEST)
#include <Chimera/chimera.hpp>
#include <Chimera/gpio.hpp>

using namespace Chimera;
using namespace Chimera::GPIO;

int main(void)
{
    ChimeraInit();

    GPIOClass led = GPIOClass(Port::PORTA, 5);
    led.mode(Drive::OUTPUT_PUSH_PULL);
    led.write(State::LOW);

    const char *p = "";
    auto test_result = CommandLineTestRunner::RunAllTests(0, &p);

    /*-------------------------------------------------
     OK: 850 mS delay (seems leisurely and carefree)
    !OK: 100 mS delay
    -------------------------------------------------*/
    uint32_t delay_ms = 100;
    if(test_result == 0)
    {
        delay_ms = 850;
    }

    while(1)
    {
        led.write(State::LOW);
        delayMilliseconds(delay_ms);
        led.write(State::HIGH);
        delayMilliseconds(delay_ms);
    }
}
#else
int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
#endif 


