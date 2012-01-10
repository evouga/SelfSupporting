#include "solvers.h"
#include "bcls.h"

#include <iostream>
#include <unsupported/Eigen/SparseExtra>

using namespace Eigen;
using namespace std;

void matrixProduct(const SparseMatrix<double, RowMajor> &B, VectorXd &b, const VectorXd &a)
{
    b = B*a;
}

void matrixTProduct(const SparseMatrix<double, RowMajor> &B, VectorXd &b, const VectorXd &a)
{
    b = B.transpose()*a;
}


// ---------------------------------------------------------------------
// Aprod
// Matrix-vector products.
//
// If     mode == BCLS_PROD_A  (0), compute y <- A *x, with x untouched;
// and if mode == BCLS_PROD_At (1), compute x <- A'*y, with y untouched.
// ---------------------------------------------------------------------
int Aprod( int mode, int m, int n, int nix, int ix[], double x[], double y[], void *UsrWrk ) {
    const SparseMatrix<double, RowMajor> &B = *(const SparseMatrix<double, RowMajor> *)UsrWrk;
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
        matrixProduct(B, scratchm, scratchn);
        for(int i=0; i<m; i++)
        {
            y[i] = scratchm[i];
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
        matrixTProduct(B, scratchn, scratchm);
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

void Solvers::solveBCLS(const SparseMatrix<double, RowMajor> &A, VectorXd &b, VectorXd &lb, VectorXd &ub, VectorXd &result)
{
    int m = A.rows();
    int n = A.cols();

    BCLS *ls = bcls_create_prob(m,n);

    assert(b.size() == m);
    assert(lb.size() == n);
    assert(ub.size() == n);
    double *anorm = new double[n];
    bcls_compute_anorm(ls, n, m, Aprod, (void *)&A, anorm);

    bcls_set_anorm(ls, anorm);
    bcls_set_problem_data(ls, m, n, Aprod, (void *)&A, 0, result.data(), b.data(), NULL, lb.data(), ub.data());
    ls->newton_step = BCLS_NEWTON_STEP_CGLS;
    ls->optTol = 1e-4;
    ls->print_level =0 ;
    bcls_solve_prob(ls);
    bcls_free_prob(ls);
    delete[] anorm;
}

void Solvers::linearSolveLDLT(const SparseMatrix<double> &A, const VectorXd &rhs, VectorXd &result)
{
    assert(A.cols() == rhs.size());
    assert(A.rows() == A.cols());
    assert(A.rows() == result.size());
    SparseLDLT<SparseMatrix<double> > ldlt(A);
    result = ldlt.solve(rhs);
}

void Solvers::linearSolveCG(const SparseMatrix<double, RowMajor> &A, const VectorXd &rhs, VectorXd &result)
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
