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
    bool isStopped();

protected:
    void run();

signals:
    void updateUI();

public slots:

private:
    Controller &c_;

    QMutex stopMutex_;
    bool stop_;

};

#endif // NETWORKTHREAD_H
