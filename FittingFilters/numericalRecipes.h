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
        u(a),
        v(n,n),
        w(n)
    {
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

class FitSVD
{
public:
    FitSVD(VecDoub_I &xx, VecDoub_I &yy, VecDoub_I &ssig, VecDoub funks(const Doub), const Doub TOL=1.e-12) :
        ndat(yy.size()),
        x(&xx),
        xmd(NULL),
        y(yy),
        sig(ssig),
        funcs(funks),
        tol(TOL)
    {}

    FitSVD(MatDoub_I &xx, VecDoub_I &yy, VecDoub_I &ssig, VecDoub funks(VecDoub_I &), const Doub TOL=1.e-12)
	    : 
        ndat(yy.size()), 
        x(NULL), 
        xmd(&xx), 
        y(yy), 
        sig(ssig),
	    funcsmd(funks), 
        tol(TOL) 
    {}

    void fit();

	VecDoub row(MatDoub_I &a, const Int i) 
    {
		Int j,n=a.ncols();
		VecDoub ans(n);
		for (j=0;j<n;j++) ans[j] = a[i][j];
		return ans;
	}

    /*template<int XGrad, int YGrad> coeffFunc(VecDoub_I &xx)
    {
        
    }

    typedef coeffFunc<1,1> coeffFunc1D;
    typedef coeffFunc<2,2> coeffFunc2D;
    typedef coeffFunc<3,3> coeffFunc3D;*/

private:
    Int ndat, ma;
    Doub tol;
    VecDoub_I *x, &y, &sig;
    VecDoub (*funcs)(const Doub);
    VecDoub a;
    MatDoub covar;
    Doub chisq;

    MatDoub_I *xmd;
	VecDoub (*funcsmd)(VecDoub_I &);

    
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // NUMERICALRECIPES_H
