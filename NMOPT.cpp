#include "networkmesh.h"
#include <map>

using namespace Ipopt;
using namespace std;

bool NMOPT::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                          Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    int constraints = 0;
    int nz = 0;
    int nzh = 0;
    for(MyMesh::VertexIter vi = nm_.getMesh().vertices_begin(); vi != nm_.getMesh().vertices_end(); ++vi)
    {
        if(!nm_.getMesh().data(vi).pinned())
        {
            constraints++;
            nz += 3;
        }
        nzh += 3;
        for(MyMesh::VertexVertexIter vvi = nm_.getMesh().vv_iter(vi); vvi; ++vvi)
        {
            if(!nm_.getMesh().data(vi).pinned())
                nz += 3;
        }
        for(MyMesh::VertexEdgeIter vei = nm_.getMesh().ve_iter(vi); vei; ++vei)
        {
            if(!nm_.getMesh().data(vi).pinned())
                nz += 3;
            nzh += 3;
        }
    }
    n = 3*v+e;
    m = 3*constraints;

    nnz_jac_g = nz;
    nnz_h_lag = nzh;

    index_style = C_STYLE;
    return true;
}

bool NMOPT::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u, Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    for(int i=0; i<3*v; i++)
    {
        x_l[i] = -std::numeric_limits<double>::infinity();
        x_u[i] = std::numeric_limits<double>::infinity();
    }
    for(int i=0; i<e; i++)
    {
        x_l[3*v+i] = 0;
        x_u[3*v+i] = std::numeric_limits<double>::infinity();
    }

    for(int i=0; i<m; i++)
    {
        g_l[i] = 0;
        g_u[i] = 0;
    }
    return true;
}

bool NMOPT::get_starting_point(Ipopt::Index n, bool , Ipopt::Number *x, bool init_z, Ipopt::Number *, Ipopt::Number *, Ipopt::Index , bool init_lambda, Ipopt::Number *)
{
    assert(!init_z);
    assert(!init_lambda);
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);
    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        MyMesh::Point pt = nm_.getMesh().point(vh);
        for(int j=0; j<3; j++)
        {
            x[3*i+j] = pt[j];
        }
    }
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = nm_.getMesh().edge_handle(i);
        double weight = nm_.getMesh().data(eh).weight();
        x[3*v+i] = weight;
    }
    return true;
}

bool NMOPT::eval_f(Index n, const Number* x,
                    bool , Number& obj_value)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    double fval = 0;
    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        MyMesh::Point pt = nm_.getMesh().point(vh);
        for(int j=0; j<3; j++)
            fval += (pt[j]-x[3*i+j])*(pt[j]-x[3*i+j]);
    }
    obj_value = fval;
    return true;
}

bool NMOPT::eval_grad_f(Index n, const Number* x, bool ,
                         Number* grad_f)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        MyMesh::Point pt = nm_.getMesh().point(vh);
        for(int j=0; j<3; j++)
        {
            grad_f[3*i+j] = 2*x[3*i+j] - 2*pt[j];
        }
    }
    for(int i=0; i<e; i++)
    {
        grad_f[3*v+i] = 0;
    }
    return true;
}

bool NMOPT::eval_g(Index n, const Number* x,
                    bool , Index m, Number* g)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    int row = 0;
    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        if(nm_.getMesh().data(vh).pinned())
            continue;

        double sum[3];
        sum[0] = sum[1] = sum[2] = 0;
        for(MyMesh::VertexOHalfedgeIter voh = nm_.getMesh().voh_iter(vh); voh; ++voh)
        {
            MyMesh::HalfedgeHandle heh = voh;
            MyMesh::EdgeHandle eh = nm_.getMesh().edge_handle(heh);
            int edgeidx = eh.idx();
            double weight = x[3*v+edgeidx];
            int adjidx = nm_.getMesh().to_vertex_handle(heh).idx();
            for(int j=0; j<3; j++)
                sum[j] += (x[3*i+j]-x[3*adjidx+j])*weight;
        }
        sum[1] += nm_.getMesh().data(vh).load();
        for(int j=0; j<3; j++)
            g[3*row+j] = sum[j];
        row++;
    }
    assert(3*row == m);
    return true;
}

bool NMOPT::eval_jac_g(Index n, const Number* x, bool ,
                        Index m, Index nele_jac, Index* iRow,
                        Index *jCol, Number* values)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);
    int nz = 0;
    int row = 0;
    if(values)
    {
        for(int j=0; j<nele_jac; j++)
            values[j] = 0;
    }

    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        if(nm_.getMesh().data(vh).pinned())
            continue;

        if(values == NULL)
        {
            for(int j=0; j<3; j++)
            {
                iRow[nz+j] = row+j;
                jCol[nz+j] = 3*i+j;
            }
            nz += 3;
            for(MyMesh::VertexOHalfedgeIter voh = nm_.getMesh().voh_iter(vh); voh; ++voh)
            {
                MyMesh::EdgeHandle eh = nm_.getMesh().edge_handle(voh.handle());
                MyMesh::VertexHandle adj = nm_.getMesh().to_vertex_handle(voh.handle());

                int vidx = adj.idx();
                int eidx = eh.idx();
                for(int j=0; j<3; j++)
                {
                    iRow[nz+j] = row+j;
                    jCol[nz+j] = 3*vidx+j;
                }
                nz += 3;

                for(int j=0; j<3; j++)
                {
                    iRow[nz+j] = row+j;
                    jCol[nz+j] = 3*v + eidx;
                }
                nz += 3;
            }
        }
        else
        {
            for(MyMesh::VertexEdgeIter vei = nm_.getMesh().ve_iter(vh); vei; ++vei)
            {
                int eidx = vei.handle().idx();
                for(int j=0; j<3; j++)
                    values[nz+j] += x[3*v+eidx];
            }
            nz += 3;
            for(MyMesh::VertexOHalfedgeIter voh = nm_.getMesh().voh_iter(vh); voh; ++voh)
            {
                MyMesh::VertexHandle adj = nm_.getMesh().to_vertex_handle(voh.handle());
                MyMesh::EdgeHandle eh = nm_.getMesh().edge_handle(voh.handle());

                int vidx = adj.idx();
                int eidx = eh.idx();
                for(int j=0; j<3; j++)
                    values[nz+j] -= x[3*v+eidx];
                nz += 3;
                for(int j=0; j<3; j++)
                {
                    double diff = x[3*i+j] - x[3*vidx+j];
                    values[nz+j] += diff;
                }
                nz += 3;
            }
        }
        row+= 3;
    }
    assert(row == m);
    assert(nz == nele_jac);
    return true;
}

bool NMOPT::eval_h(Index n, const Number* , bool ,
                    Number obj_factor, Index m, const Number* lambda,
                    bool , Index nele_hess, Index* iRow,
                    Index* jCol, Number* values)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    if(values)
        for(int j=0; j<nele_hess; j++)
            values[j] = 0;

    map<int, int> vidx2gidx;
    int row =0;
    for(int i=0; i<v; i++)
    {
        if(nm_.getMesh().data(nm_.getMesh().vertex_handle(i)).pinned())
            continue;
        vidx2gidx[i] = row;
        row++;
    }
    assert(3*row == m);

    int nz = 0;
    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        if(!values)
        {
            for(int j=0; j<3; j++)
            {
                iRow[nz+j] = 3*i+j;
                jCol[nz+j] = 3*i+j;
            }
            nz += 3;

            for(MyMesh::VertexOHalfedgeIter voh = nm_.getMesh().voh_iter(vh); voh; ++voh)
            {
                int eidx = nm_.getMesh().edge_handle(voh.handle()).idx();
                for(int j=0; j<3; j++)
                {
                    iRow[nz+j] = 3*i+j;
                    jCol[nz+j] = 3*v+eidx;
                }
                nz += 3;
            }
        }
        else
        {
            for(int j=0; j<3; j++)
            {
                values[nz+j] += 2*obj_factor;
            }
            nz += 3;

            for(MyMesh::VertexOHalfedgeIter voh = nm_.getMesh().voh_iter(vh); voh; ++voh)
            {
                int vidx = nm_.getMesh().to_vertex_handle(voh.handle()).idx();
                for(int j=0; j<3; j++)
                {
                    double val = 0;
                    if(!nm_.getMesh().data(vh).pinned())
                    {
                        val += lambda[3*vidx2gidx[i]+j];
                    }
                    if(!nm_.getMesh().data(nm_.getMesh().to_vertex_handle(voh.handle())).pinned())
                    {
                        val -= lambda[3*vidx2gidx[vidx]+j];
                    }
                    values[nz+j] += val;
                }
                nz += 3;
            }
        }
    }
    assert(nz == nele_hess);
    return true;
}


void NMOPT::finalize_solution(SolverReturn status, Index n,
                               const Number* x, const Number* z_L,
                               const Number* z_U, Index m, const Number* g,
                               const Number* lambda, Number obj_value,
                               const IpoptData* ip_data,
                               IpoptCalculatedQuantities* ip_cq)
{
    int v = nm_.getMesh().n_vertices();
    int e = nm_.getMesh().n_edges();
    assert(n == 3*v+e);

    for(int i=0; i<v; i++)
    {
        MyMesh::VertexHandle vh = nm_.getMesh().vertex_handle(i);
        MyMesh::Point &pt = nm_.getMesh().point(vh);
        for(int j=0; j<3; j++)
        {
            pt[j] = x[3*i+j];
        }
    }
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = nm_.getMesh().edge_handle(i);
        double newweight = x[3*v+i];
        if(nm_.edgePinned(eh))
            newweight = 0;
        nm_.getMesh().data(eh).set_weight(newweight);
    }
}
