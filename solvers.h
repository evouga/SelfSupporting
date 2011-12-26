#ifndef SOLVERS_H
#define SOLVERS_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include "vectorpool.h"

class MatrixBlock
{
public:
    // Does not copy matrices!
    MatrixBlock(int rows, int cols, std::vector<const Eigen::SparseMatrix<double> *> blocks);
    void product(const Eigen::VectorXd &b, Eigen::VectorXd &result) const;
    void transposeProduct(const Eigen::VectorXd &b, Eigen::VectorXd &result) const;
    int rows() const {return rows_;}
    int cols() const {return cols_;}

private:
    // Not implemented
    MatrixBlock(const MatrixBlock &other);
    MatrixBlock &operator=(const MatrixBlock &other);

    int n_;
    int m_;
    int rows_;
    int cols_;
    std::vector<const Eigen::SparseMatrix<double> *> blocks_;
};

class Solvers
{
public:
    Solvers();
    ~Solvers();

    // bcls may or may not modify b, lb, ub. Haven't checked
    void solveBCLS(const Eigen::SparseMatrix<double> &A, Eigen::VectorXd &b, Eigen::VectorXd &lb, Eigen::VectorXd &ub, Eigen::VectorXd &result);
    void solveBCLS(const MatrixBlock &B, Eigen::VectorXd &b, Eigen::VectorXd &lb, Eigen::VectorXd &ub, Eigen::VectorXd &result);

    // Solves min_q \| q - q_0\|^2_M   s.t.   C^T q + c_0 = 0,
    // with M diagonal and nonsingular.
    void solveWeightedLSE(const Eigen::VectorXd &M, const Eigen::VectorXd &q0, const Eigen::SparseMatrix<double> &C, const Eigen::VectorXd &c0, Eigen::VectorXd &result);
    void solveWeightedLSE(const Eigen::SparseMatrix<double> &M, const Eigen::VectorXd &q0, const Eigen::SparseMatrix<double> &C, const Eigen::VectorXd &c0, Eigen::VectorXd &result);

    // Positive definite matrix A
    void linearSolveCG(const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &rhs, Eigen::VectorXd &result);
    // Positive semidefinite matrix
    void linearSolveLDLT(const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &rhs, Eigen::VectorXd &result);
    // Arbitrary nonsingular
    void linearSolveLU(const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &rhs, Eigen::VectorXd &result);

    // Arbitrary A
    void linearSolveBCLS(const MatrixBlock &B, Eigen::VectorXd &rhs, Eigen::VectorXd &result);

private:

    friend int Aprod( int mode, int m, int n, int nix, int ix[], double x[], double y[], void *UsrWrk );
    static VectorPool pool_;
};

#endif // SOLVERS_H
