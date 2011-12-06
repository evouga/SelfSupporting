#include "solvers.h"
#include "bcls.h"

#include <iostream>
#include <unsupported/Eigen/SparseExtra>

using namespace Eigen;
using namespace std;

MatrixBlock::MatrixBlock(int rows, int cols, vector<const SparseMatrix<double> *> blocks) : n_(rows), m_(cols), blocks_(blocks)
{
    assert(n_*m_ == (int)blocks_.size());
    int totcols=0;
    int totrows=0;
    for(int i=0; i<n_; i++)
    {
        int currows = blocks_[i*m_]->rows();
        totrows += currows;
        int curcols = 0;
        for(int j=0; j<m_; j++)
        {
            curcols += blocks_[i*m_+j]->cols();
            assert(blocks_[i*m_+j]->rows() == currows);
        }
        if(i==0)
            totcols = curcols;
        else
        {
            assert(totcols == curcols);
        }
    }
    rows_ = totrows;
    cols_ = totcols;
}

void MatrixBlock::product(const VectorXd &b, VectorXd &result) const
{
    assert(b.size() == cols_);
    result.resize(rows_);
    result.setZero();
    int currow = 0;
    for(int i=0; i<n_; i++)
    {
        int rowsize = blocks_[i*m_]->rows();
        int curcol = 0;
        for(int j=0; j<m_; j++)
        {
            int colsize = blocks_[i*m_+j]->cols();
            result.segment(currow, rowsize) += (*blocks_[i*m_+j])*b.segment(curcol, colsize);
            curcol += colsize;
        }
        currow += rowsize;
    }
}

void MatrixBlock::transposeProduct(const VectorXd &b, VectorXd &result) const
{
    assert(b.size() == rows_);
    result.resize(cols_);
    result.setZero();
    int curcol = 0;
    for(int j=0; j<m_; j++)
    {
        int colsize = blocks_[j]->cols();
        int currow = 0;
        for(int i=0; i<n_; i++)
        {
            int rowsize = blocks_[i*m_+j]->rows();
            result.segment(curcol, colsize) += blocks_[i*m_+j]->transpose()*b.segment(currow, rowsize);
            currow += rowsize;
        }
        curcol += colsize;
    }
}

// ---------------------------------------------------------------------
// Aprod
// Matrix-vector products.
//
// If     mode == BCLS_PROD_A  (0), compute y <- A *x, with x untouched;
// and if mode == BCLS_PROD_At (1), compute x <- A'*y, with y untouched.
// ---------------------------------------------------------------------
int Aprod( int mode, int m, int n, int nix, int ix[], double x[], double y[], void *UsrWrk ) {
    const MatrixBlock &B = *(const MatrixBlock *)UsrWrk;
    assert(B.rows() == m);
    assert(B.cols() == n);
    switch(mode)
    {
    case BCLS_PROD_A:
    {
        TempVector Tscratchn(Solvers::pool_, n);
        VectorXd &scratchn = *Tscratchn;
        scratchn.setZero();
        for(int i=0; i<nix; i++)
        {
            scratchn[ix[i]] = x[ix[i]];
        }

        TempVector Tscratchm(Solvers::pool_, m);
        VectorXd &scratchm = *Tscratchm;
        B.product(scratchn, scratchm);
        double norm = 0;
        for(int i=0; i<m; i++)
        {
            y[i] = scratchm[i];
            norm += y[i]*y[i];
        }
        break;
    }
    case BCLS_PROD_At:
    {
        TempVector Tscratchm(Solvers::pool_, m);
        VectorXd &scratchm = *Tscratchm;
        for(int i=0; i<m; i++)
            scratchm[i] = y[i];
        TempVector Tscratchn(Solvers::pool_, n);
        VectorXd &scratchn = *Tscratchn;
        B.transposeProduct(scratchm, scratchn);
        for(int i=0; i<nix; i++)
        {
            int row = ix[i];
            x[row] = scratchn[row];
        }
        break;
    }
    }
    return 0;
}

Solvers::Solvers()
{
}

Solvers::~Solvers()
{
}

VectorPool Solvers::pool_;

void Solvers::solveBCLS(const MatrixBlock &B, VectorXd &b, VectorXd &lb, VectorXd &ub, VectorXd &result)
{
    int m = B.rows();
    int n = B.cols();

    BCLS *ls = bcls_create_prob(m,n);

    assert(b.size() == m);
    assert(lb.size() == n);
    assert(ub.size() == n);

    bcls_set_problem_data(ls, m, n, Aprod, (void *)&B, 0, result.data(), b.data(), NULL, lb.data(), ub.data());
    ls->print_level = 0;
    bcls_solve_prob(ls);
    /*bcls_init_prob(ls);
    cout << "=========== 0 ===========" << endl;
    bcls_set_problem_data(ls, m, n, Aprod, (void *)&B, 0, result.data(), b.data(), NULL, lb.data(), ub.data());
    bcls_solve_prob(ls);
*/
    bcls_free_prob(ls);
}

void Solvers::solveBCLS(const SparseMatrix<double> &A, VectorXd &b, VectorXd &lb, VectorXd &ub, VectorXd &result)
{
    vector<const SparseMatrix<double> *> blocks;
    blocks.push_back(&A);
    MatrixBlock B(1,1,blocks);
    solveBCLS(B, b, lb, ub, result);
}

void Solvers::solveWeightedLSE(const SparseMatrix<double> &M, const VectorXd &q0, const SparseMatrix<double> &C, const VectorXd &c0, VectorXd &result)
{
    int n = M.cols();
    assert(M.rows() == n);
    assert(q0.size() == n);
    assert(C.rows() == n);
    result.resize(n);;
    int m = C.cols();
    assert(c0.size() == m);

    SparseMatrix<double> zero(m,m);
    zero.finalize();

    SparseMatrix<double> CT = C.transpose();
    vector<const SparseMatrix<double> *> blocks;
    blocks.push_back(&M);
    blocks.push_back(&C);
    blocks.push_back(&CT);
    blocks.push_back(&zero);
    MatrixBlock B(2,2,blocks);

    VectorXd augrhs(n+m);
    augrhs.segment(0,n) = M*q0;
    augrhs.segment(n,m) = -c0;

    VectorXd augresult(n+m);
    augresult.setZero();

    linearSolveBCLS(B, augrhs, augresult);
    result = augresult.segment(0,n);
}

void Solvers::solveWeightedLSE(const VectorXd &M, const VectorXd &q0, const SparseMatrix<double> &C, const VectorXd &c0, VectorXd &result)
{
    int n = M.size();
    assert(q0.size() == n);
    assert(C.rows() == n);
    assert(result.size() == n);
    int m = C.cols();
    assert(c0.size() == m);

/*    SparseMatrix<double> Mm(n,n);
    for(int i=0; i<n; i++)
    {
        Mm.startVec(i);
        Mm.insertBack(i,i) = M[i];
    }
    Mm.finalize();

    SparseMatrix<double> zero(m,m);
    zero.finalize();

    SparseMatrix<double> CT = C.transpose();

    vector<const SparseMatrix<double> *> blocks;
    blocks.push_back(&Mm);
    blocks.push_back(&C);
    blocks.push_back(&CT);
    blocks.push_back(&zero);
    MatrixBlock B(2,2,blocks);

    VectorXd augrhs(n+m);
    augrhs.segment(0, n) = Mm*q0;
    augrhs.segment(n, m) = -c0;

    VectorXd augresult(n+m);
    augresult.setZero();

    linearSolveBCLS(B, augrhs, augresult);
    result = augresult.segment(0,n);*/



    SparseMatrix<double> Minv(n,n);
    for(int i=0; i<n; i++)
    {
        Minv.startVec(i);
        Minv.insertBack(i,i) = 1.0/M[i];
    }
    Minv.finalize();

    SparseMatrix<double> CTMinvC = C.transpose()*Minv*C;

    VectorXd lambda(m);
    lambda.setZero();
    VectorXd rhs = C.transpose()*q0 + c0;
    linearSolveCG(CTMinvC, rhs, lambda);
    result = q0 - Minv*(C*lambda);
}

void Solvers::linearSolveCG(const SparseMatrix<double> &A, const VectorXd &rhs, VectorXd &result)
{
    int n = A.rows();
    assert(A.cols() == n);
    VectorXd residual = rhs - A*result;
    VectorXd p = residual;
    double rorig = residual.dot(residual);
    VectorXd tmp(p.size());
    for(int i=0; i<n; i++)
    {
        double rnorm = residual.dot(residual);
        if(rnorm/rorig < 1e-8 || rnorm < 1e-12)
        {
            //cout << "Converged after " << i << " iterations" << endl;
            return;
        }
        tmp = A*p;
        double denom = p.dot(tmp);
        double alpha = rnorm/denom;
        result += alpha*p;
        residual -= alpha*(tmp);
        double beta = residual.dot(residual) / rnorm;
        p = residual + beta*p;
    }
    cout << "Did not converge, residual " << sqrt(residual.dot(residual)) << endl;
}

void Solvers::linearSolveBCLS(const MatrixBlock &B, VectorXd &rhs, VectorXd &result)
{
    TempVector Tlb(pool_, B.rows());
    TempVector Tub(pool_, B.rows());
    for(int i=0; i<B.rows(); i++)
    {
        (*Tlb)[i] = -std::numeric_limits<double>::infinity();
        (*Tub)[i] = std::numeric_limits<double>::infinity();
    }
    result.resize(B.rows());
    result.setZero();
    solveBCLS(B, rhs, *Tlb, *Tub, result);
}
