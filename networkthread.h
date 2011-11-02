#ifndef NETWORKTHREAD_H
#define NETWORKTHREAD_H

#include <QThread>
#include <QMutex>

class Controller;

class NetworkThread : public QThread
{
    Q_OBJECT
public:
    explicit NetworkThread(Controller &c);
    void stop();
    void pause();
    void unpause();

protected:
    void run();

signals:
    void updateUI();

public slots:

private:
    enum ThreadState {TS_RUNNING, TS_PAUSED, TS_STOPPED};

    ThreadState getState();
    void setState(ThreadState state);

    Controller &c_;
    QMutex stateMutex_;
    ThreadState state_;

};

#endif // NETWORKTHREAD_H
