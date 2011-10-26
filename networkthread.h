#ifndef NETWORKTHREAD_H
#define NETWORKTHREAD_H

#include <QThread>

class Controller;

class NetworkThread : public QThread
{
    Q_OBJECT
public:
    explicit NetworkThread(Controller &c);

protected:
    void run();

signals:

public slots:

private:
    Controller &c_;

};

#endif // NETWORKTHREAD_H
