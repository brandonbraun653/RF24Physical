#include <CppUTest/CommandLineTestRunner.h>

#if defined(EMBEDDED_TEST)
#include <stm32f4xx_hal.h>
#include <stm32_hal_legacy.h>

#ifdef __cplusplus
extern "C"
#endif
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

int main(void)
{
    HAL_Init();

    const char *p = "";
    CommandLineTestRunner::RunAllTests(0, &p);
}
#else
int main(int ac, char** av)
{
    printf("Yo man");
    return CommandLineTestRunner::RunAllTests(ac, av);
}
#endif 


