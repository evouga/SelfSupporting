#include "networkthread.h"
#include "controller.h"

NetworkThread::NetworkThread(Controller &c) :
    QThread(NULL), c_(c), state_(TS_RUNNING)
{
}

void NetworkThread::run()
{
    while(getState() != TS_STOPPED)
    {
        if(getState() == TS_RUNNING)
        {
            c_.iterateNetwork();
            emit updateUI();
        }
        msleep(20);
    }
}

NetworkThread::ThreadState NetworkThread::getState()
{
    ThreadState result;
    stateMutex_.lock();
    result = state_;
    stateMutex_.unlock();
    return result;
}

void NetworkThread::stop()
{
    stateMutex_.lock();
    state_ = TS_STOPPED;
    stateMutex_.unlock();
}

void NetworkThread::pause()
{
    stateMutex_.lock();
    if(state_ == TS_RUNNING)
        state_ = TS_PAUSED;
    stateMutex_.unlock();
}

void NetworkThread::unpause()
{
    stateMutex_.lock();
    if(state_ == TS_PAUSED)
        state_ = TS_RUNNING;
    stateMutex_.unlock();
}
