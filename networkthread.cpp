#include "networkthread.h"
#include "controller.h"

NetworkThread::NetworkThread(Controller &c) :
    QThread(NULL), c_(c)
{
}

void NetworkThread::run()
{
    while(true)
    {
        c_.iterateNetwork();
        sleep(0);
    }
}
