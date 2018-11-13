#include <CppUTest/CommandLineTestRunner.h>

#if defined(EMBEDDED) || defined(HARDWARE_TEST)
#include <Thor/include/thor.hpp>

int main(void)
{
    HAL_Init();
    ThorInit();

    const char *p = "";
    CommandLineTestRunner::RunAllTests(0, &p);

    while(1);
}
#else
int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
#endif 


