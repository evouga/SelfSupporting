#include "networkthread.h"
#include "controller.h"

NetworkThread::NetworkThread(Controller &c) :
    QThread(NULL), c_(c), stop_(false)
{
}

void NetworkThread::run()
{
    while(!isStopped())
    {
        c_.iterateNetwork();
        emit updateUI();
        msleep(20);
    }
}

void NetworkThread::stop()
{
    stopMutex_.lock();
    stop_ = true;
    stopMutex_.unlock();
}

bool NetworkThread::isStopped()
{
    stopMutex_.lock();
    bool result = stop_;
    stopMutex_.unlock();
    return result;
}
