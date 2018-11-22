#include <CppUTest/CommandLineTestRunner.h>

#if defined(EMBEDDED) || defined(HARDWARE_TEST)
#include <Chimera/chimera.hpp>

int main(void)
{
    ChimeraInit();

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


