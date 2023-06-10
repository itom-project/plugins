/* licence TODO */

#ifndef NUMERICALRECIPES_H
#define NUMERICALRECIPES_H

#include "DataObject/dataobj.h"
#include "nr3.h"




class SVD
{
public:
    SVD(MatDoub_I &a) :
        m(a.nrows()),
        n(a.ncols()),
        u(a)
        //v(n,n), don't assign these values here, since they depend on n and on some compilers it is not sure that n(a.ncols()) is evaluated before
        //w(n)
    {
        v = MatDoub(n,n);
        w = VecDoub(n);
        eps = std::numeric_limits<Doub>::epsilon();
        decompose();
        reorder();
        tsh = 0.5 * sqrt(m+n+1.)*w[0]*eps;
    }

    void solve(VecDoub_I &b, VecDoub_O &x, Doub thresh = -1.);
    void solve(MatDoub_I &b, MatDoub_O &x, Doub thresh = -1.);

    MatDoub u,v;
    VecDoub w;
    Doub eps, tsh;

private:
    Int m,n;


    Int rank(Doub thresh = -1.);
    Int nullity(Doub thresh = -1.);
    MatDoub range(Doub thresh = -1.);
    MatDoub nullspace(Doub tresh = -1.);

    Doub inv_condition()
    {
        return (w[0] <= 0. || w[n-1] <= 0.) ? 0. : w[n-1]/w[0];
    }

    void decompose();
    void reorder();
    Doub pythag(const Doub a, const Doub b);
};

//class FitSVD
//{
//public:
//    FitSVD(VecDoub_I &xx, VecDoub_I &yy, VecDoub_I &ssig, VecDoub funks(const Doub), const Doub TOL=1.e-12) :
//        ndat(yy.size()),
//        x(&xx),
//        xmd(NULL),
//        y(yy),
//        sig(ssig),
//        funcs(funks),
//        tol(TOL)
//    {}
//
//    FitSVD(MatDoub_I &xx, VecDoub_I &yy, VecDoub_I &ssig, VecDoub funks(VecDoub_I &), const Doub TOL=1.e-12)
//        :
//        ndat(yy.size()),
//        x(NULL),
//        xmd(&xx),
//        y(yy),
//        sig(ssig),
//        funcsmd(funks),
//        tol(TOL)
//    {}
//
//    void fit();
//
//    VecDoub row(MatDoub_I &a, const Int i)
//    {
//        Int j,n=a.ncols();
//        VecDoub ans(n);
//        for (j=0;j<n;j++) ans[j] = a[i][j];
//        return ans;
//    }
//
//    template<int order> static VecDoub fpoly(const Doub x)
//    {
//        Int j;
//        VecDoub p(order);
//        p[0] = 1.0;
//        for (j=1;j<order;++j) p[j]=p[j-1]*x;
//        return p;
//    }
//
//    inline static VecDoub poly1d_1(const Doub x) { return fpoly<1>(x); }
//    inline static VecDoub poly1d_2(const Doub x) { return fpoly<2>(x); }
//    inline static VecDoub poly1d_3(const Doub x) { return fpoly<3>(x); }
//    inline static VecDoub poly1d_4(const Doub x) { return fpoly<4>(x); }
//    inline static VecDoub poly1d_5(const Doub x) { return fpoly<5>(x); }
//    inline static VecDoub poly1d_6(const Doub x) { return fpoly<6>(x); }
//    inline static VecDoub poly1d_7(const Doub x) { return fpoly<7>(x); }
//    inline static VecDoub poly1d_8(const Doub x) { return fpoly<8>(x); }
//    inline static VecDoub poly1d_9(const Doub x) { return fpoly<9>(x); }
//
//private:
//    Int ndat, ma;
//    Doub tol;
//    VecDoub_I *x, &y, &sig;
//    VecDoub (*funcs)(const Doub);
//    VecDoub a;
//    MatDoub covar;
//    Doub chisq;
//
//    MatDoub_I *xmd;
//    VecDoub (*funcsmd)(VecDoub_I &);
//};


class FitSVDSimple
{
public:
    FitSVDSimple(VecDoub funks(const Doub), const Doub TOL=1.e-12) :
        funcs(funks),
        tol(TOL)
    {}

    void fit(VecDoub_I &x, VecDoub_I &y, VecDoub_I &sig, VecDoub_O &p, Doub &chisq) const;

    template<int order> static VecDoub fpoly(const Doub x)
    {
        Int j;
        VecDoub p(order+1);
        p[0] = 1.0;
        for (j=1;j<=order;++j) p[j]=p[j-1]*x;
        return p;
    }

    inline static VecDoub poly1d_1(const Doub x) { return fpoly<1>(x); }
    inline static VecDoub poly1d_2(const Doub x) { return fpoly<2>(x); }
    inline static VecDoub poly1d_3(const Doub x) { return fpoly<3>(x); }
    inline static VecDoub poly1d_4(const Doub x) { return fpoly<4>(x); }
    inline static VecDoub poly1d_5(const Doub x) { return fpoly<5>(x); }
    inline static VecDoub poly1d_6(const Doub x) { return fpoly<6>(x); }
    inline static VecDoub poly1d_7(const Doub x) { return fpoly<7>(x); }
    inline static VecDoub poly1d_8(const Doub x) { return fpoly<8>(x); }
    inline static VecDoub poly1d_9(const Doub x) { return fpoly<9>(x); }

private:
    Doub tol;
    VecDoub (*funcs)(const Doub);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // NUMERICALRECIPES_H
