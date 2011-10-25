#include "vectorpool.h"

using namespace std;
using namespace Eigen;

TempVector::TempVector(VectorPool &pool, int n) : pool_(pool)
{
    v_ = pool_.getTemporaryVector(n);
}

TempVector::~TempVector()
{
    pool_.returnToPool(v_);
}

TempVector::operator VectorXd *()
{
    return v_;
}


VectorPool::VectorPool()
{
}

VectorPool::~VectorPool()
{
    for(map<int, stack<VectorXd *> >::iterator it = pool_.begin(); it != pool_.end(); ++it)
    {
        while(!it->second.empty())
        {
            delete it->second.top();
            it->second.pop();
        }
    }
}

void VectorPool::returnToPool(VectorXd *v)
{
    int n = v->size();
    pool_[n].push(v);
}

VectorXd *VectorPool::getTemporaryVector(int n)
{
    stack<VectorXd *> &s = pool_[n];
    if(s.empty())
    {
        VectorXd *newv = new VectorXd(n);
        return newv;
    }
    VectorXd *ret = s.top();
    s.pop();
    return ret;
}
