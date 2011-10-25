#ifndef VECTORPOOL_H
#define VECTORPOOL_H

#include <Eigen/Core>
#include <map>
#include <stack>

class VectorPool;

class TempVector
{
public:
    TempVector(VectorPool &pool, int n);
    ~TempVector();
    operator Eigen::VectorXd *();
private:
    Eigen::VectorXd *v_;
    VectorPool &pool_;
};

class VectorPool
{
public:
    VectorPool();
    ~VectorPool();
private:
    friend class TempVector;

    void returnToPool(Eigen::VectorXd *v);
    Eigen::VectorXd *getTemporaryVector(int n);

    std::map<int, std::stack<Eigen::VectorXd *> > pool_;
};

#endif // VECTORPOOL_H
