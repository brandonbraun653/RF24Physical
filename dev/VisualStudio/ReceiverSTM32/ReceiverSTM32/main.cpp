#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

#include <Chimera/chimera.hpp>
#include <Chimera/config.hpp>
#include <Chimera/threading.hpp>

#include "receiverThread.hpp"

using namespace Chimera::Threading;

TaskHandle_t senderTask;
TaskHandle_t ledTask;


int main(void)
{
    ChimeraInit();

    #ifdef DEBUG
    InitializeSamplingProfiler();
    InitializeInstrumentingProfiler();
    #endif


    addThread(ledThread, "led", 200, NULL, 2, &ledTask);
    addThread(receiverThread, "sender", 500, NULL, 2, &senderTask);
    startScheduler();

    /* Should never reach here as scheduler should be running */
    for(;;)
    {

    }
}


