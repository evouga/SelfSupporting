#ifndef SOLVERS_H
#define SOLVERS_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include "vectorpool.h"

class Solvers
{
public:
    Solvers();
    ~Solvers();

    // bcls may or may not modify b, lb, ub. Haven't checked
    void solveBCLS(const Eigen::SparseMatrix<double, Eigen::RowMajor> &A, Eigen::VectorXd &b, Eigen::VectorXd &lb, Eigen::VectorXd &ub, Eigen::VectorXd &result, double tol);

    // Positive definite matrix A
    void linearSolveCG(const Eigen::SparseMatrix<double, Eigen::RowMajor> &A, const Eigen::VectorXd &rhs, Eigen::VectorXd &result);
    // Positive semidefinite matrix
    void linearSolveLDLT(const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &rhs, Eigen::VectorXd &result);

private:

    friend int Aprod( int mode, int m, int n, int nix, int ix[], double x[], double y[], void *UsrWrk );
    static VectorPool pool_;
};

#endif // SOLVERS_H
