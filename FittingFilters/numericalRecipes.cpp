/* licence TODO */

#include "numericalRecipes.h"

#include <qnumeric.h>


void SVD::solve(VecDoub_I &b, VecDoub_O &x, Doub thresh /*= -1.*/)
{
    Int i,j,jj;
    Doub s;

    //if (b.size() != m || x.size() != n) throw("SVD::solve bad sizes"); //this don't allow openMP parallelization

    VecDoub tmp(n);
    tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);

    for (j=0;j<n;j++)
    {
        s=0.0;
        if (w[j] > tsh) {
        for (i=0;i<m;i++) s += u[i][j]*b[i];
        s /= w[j];
        }
        tmp[j]=s;
    }

    for (j=0;j<n;j++)
    {
        s=0.0;
        for (jj=0;jj<n;jj++) s += v[j][jj]*tmp[jj];
        x[j]=s;
    }
}

void SVD::solve(MatDoub_I &b, MatDoub_O &x, Doub thresh /*= -1.*/)
{
    int i,j,m=b.ncols();
    //if (b.nrows() != n || x.nrows() != n || b.ncols() != x.ncols()) throw("SVD::solve bad sizes"); //this don't allow openMP parallelization
    VecDoub xx(n);
    for (j=0;j<m;j++)
    {
        for (i=0;i<n;i++) xx[i] = b[i][j];
        solve(xx,xx,thresh);
        for (i=0;i<n;i++) x[i][j] = xx[i];
    }
}
Int SVD::rank(Doub thresh /*= -1.*/)
{
    Int j,nr=0;
    tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);
    for (j=0;j<n;j++) if (w[j] > tsh) nr++;
    return nr;
}

Int SVD::nullity(Doub thresh /*= -1.*/)
{
    Int j,nn=0;
    tsh = (thresh >= 0. ? thresh : 0.5*sqrt(m+n+1.)*w[0]*eps);
    for (j=0;j<n;j++) if (w[j] <= tsh) nn++;
    return nn;
}

MatDoub SVD::range(Doub thresh /*= -1.*/)
{
    Int i,j,nr=0;
    MatDoub rnge(m,rank(thresh));
    for (j=0;j<n;j++)
    {
        if (w[j] > tsh)
        {
            for (i=0;i<m;i++) rnge[i][nr] = u[i][j];
            nr++;
        }
    }
    return rnge;
}

MatDoub SVD::nullspace(Doub thresh /*= -1.*/)
{
    Int j,jj,nn=0;
    MatDoub nullsp(n,nullity(thresh));
    for (j=0;j<n;j++)
    {
        if (w[j] <= tsh)
        {
            for (jj=0;jj<n;jj++) nullsp[jj][nn] = v[jj][j];
            nn++;
        }
    }
    return nullsp;
}

void SVD::decompose()
{
    //Given the matrix A stored in u[0..m-1][0..n-1], this routine computes its singular value
    //decomposition, A D U W  VT and stores the results in the matrices u and v, and the vector w.
    bool flag;
    Int i,its,j,jj,k,l,nm;
    Doub anorm,c,f,g,h,s,scale,x,y,z;
    VecDoub rv1(n);
    g = scale = anorm = 0.0; //Householder reduction to bidiagonal form.
    for (i=0;i<n;i++)
    {
    l=i+2;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i < m)
    {
        for (k=i;k<m;k++) scale += abs(u[k][i]);
        if (scale != 0.0)
        {
            for (k=i;k<m;k++)
            {
                u[k][i] /= scale;
                s += u[k][i]*u[k][i];
            }
            f=u[i][i];
            g = -NR_SIGN(sqrt(s),f);
            h=f*g-s;
            u[i][i]=f-g;
            for (j=l-1;j<n;j++)
            {
                for (s=0.0,k=i;k<m;k++) s += u[k][i]*u[k][j];
                f=s/h;
                for (k=i;k<m;k++) u[k][j] += f*u[k][i];
            }
            for (k=i;k<m;k++) u[k][i] *= scale;
        }
    }
    w[i]=scale *g;
    g=s=scale=0.0;
    if (i+1 <= m && i+1 != n)
    {
        for (k=l-1;k<n;k++) scale += abs(u[i][k]);
        if (scale != 0.0)
        {
            for (k=l-1;k<n;k++)
            {
                u[i][k] /= scale;
                s += u[i][k]*u[i][k];
            }
            f=u[i][l-1];
            g = -NR_SIGN(sqrt(s),f);
            h=f*g-s;
            u[i][l-1]=f-g;
            for (k=l-1;k<n;k++) rv1[k]=u[i][k]/h;
            for (j=l-1;j<m;j++)
            {
                for (s=0.0,k=l-1;k<n;k++) s += u[j][k]*u[i][k];
                for (k=l-1;k<n;k++) u[j][k] += s*rv1[k];
            }
            for (k=l-1;k<n;k++) u[i][k] *= scale;
        }
    }
    anorm=MAX(anorm,(abs(w[i])+abs(rv1[i])));
    }
    for (i=n-1;i>=0;i--)
    { //Accumulation of right-hand transformations.
        if (i < n-1)
        {
            if (g != 0.0)
            {
                for (j=l;j<n;j++) //Double division to avoid possible underflow.
                v[j][i]=(u[i][j]/u[i][l])/g;
                for (j=l;j<n;j++)
                {
                    for (s=0.0,k=l;k<n;k++) s += u[i][k]*v[k][j];
                    for (k=l;k<n;k++) v[k][j] += s*v[k][i];
                }
            }
            for (j=l;j<n;j++) v[i][j]=v[j][i]=0.0;
        }
        v[i][i]=1.0;
        g=rv1[i];
        l=i;
    }
    for (i=MIN(m,n)-1;i>=0;i--)
    { //Accumulation of left-hand transformations.
        l=i+1;
        g=w[i];
        for (j=l;j<n;j++) u[i][j]=0.0;
        if (g != 0.0)
        {
            g=1.0/g;
            for (j=l;j<n;j++)
            {
                for (s=0.0,k=l;k<m;k++) s += u[k][i]*u[k][j];
                f=(s/u[i][i])*g;
                for (k=i;k<m;k++) u[k][j] += f*u[k][i];
            }
            for (j=i;j<m;j++) u[j][i] *= g;
        }
        else
        {
            for (j=i;j<m;j++) u[j][i]=0.0;
        }
        ++u[i][i];
    }
    for (k=n-1;k>=0;k--) //Diagonalization of the bidiagonal form: Loop over
    {
        for (its=0;its<30;its++)
        { //singular values, and over allowed iterations.
            flag=true;
            for (l=k;l>=0;l--)
            { //Test for splitting.
                nm=l-1;
                if (l == 0 || abs(rv1[l]) <= eps*anorm)
                {
                    flag=false;
                    break;
                }
                if (abs(w[nm]) <= eps*anorm) break;
            }
            if (flag)
            {
                c=0.0; //Cancellation of rv1[l], if l > 0.
                s=1.0;
                for (i=l;i<k+1;i++)
                {
                    f=s*rv1[i];
                    rv1[i]=c*rv1[i];
                    if (abs(f) <= eps*anorm) break;
                    g=w[i];
                    h=pythag(f,g);
                    w[i]=h;
                    h=1.0/h;
                    c=g*h;
                    s = -f*h;
                    for (j=0;j<m;j++)
                    {
                        y=u[j][nm];
                        z=u[j][i];
                        u[j][nm]=y*c+z*s;
                        u[j][i]=z*c-y*s;
                    }
                }
            }
            z=w[k];
            if (l == k)
            { //Convergence.
                if (z < 0.0)
                { //Singular value is made nonnegative.
                    w[k] = -z;
                    for (j=0;j<n;j++) v[j][k] = -v[j][k];
                }
                break;
            }
            if (its == 29) throw("no convergence in 30 svdcmp iterations");
            x=w[l]; //Shift from bottom 2-by-2 minor.
            nm=k-1;
            y=w[nm];
            g=rv1[nm];
            h=rv1[k];
            f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
            g=pythag(f,1.0);
            f=((x-z)*(x+z)+h*((y/(f+NR_SIGN(g,f)))-h))/x;
            c=s=1.0; //Next QR transformation:
            for (j=l;j<=nm;j++)
            {
                i=j+1;
                g=rv1[i];
                y=w[i];
                h=s*g;
                g=c*g;
                z=pythag(f,h);
                rv1[j]=z;
                c=f/z;
                s=h/z;
                f=x*c+g*s;
                g=g*c-x*s;
                h=y*s;
                y *= c;
                for (jj=0;jj<n;jj++)
                {
                    x=v[jj][j];
                    z=v[jj][i];
                    v[jj][j]=x*c+z*s;
                    v[jj][i]=z*c-x*s;
                }
                z=pythag(f,h);
                w[j]=z; //Rotation can be arbitrary if z D 0.
                if (z)
                {
                    z=1.0/z;
                    c=f*z;
                    s=h*z;
                }
                f=c*g+s*y;
                x=c*y-s*g;
                for (jj=0;jj<m;jj++)
                {
                    y=u[jj][j];
                    z=u[jj][i];
                    u[jj][j]=y*c+z*s;
                    u[jj][i]=z*c-y*s;
                }
            }
            rv1[l]=0.0;
            rv1[k]=f;
            w[k]=x;
        }
    }
}



void SVD::reorder()
{
    //Given the output of decompose, this routine sorts the singular values, and corresponding columns
    //of u and v, by decreasing magnitude. Also, signs of corresponding columns are
    //ipped so as to
    //maximize the number of positive elements.
    Int i,j,k,s,inc=1;
    Doub sw;
    VecDoub su(m), sv(n);
    do { inc *= 3; inc++; } while (inc <= n); //Sort. The method is Shell's sort.
    /*(The work is negligible as compared
    to that already done in
    decompose.)*/
    do
    {
        inc /= 3;
        for (i=inc;i<n;i++)
        {
            sw = w[i];
            for (k=0;k<m;k++) su[k] = u[k][i];
            for (k=0;k<n;k++) sv[k] = v[k][i];
            j = i;
            while (w[j-inc] < sw)
            {
                w[j] = w[j-inc];
                for (k=0;k<m;k++) u[k][j] = u[k][j-inc];
                for (k=0;k<n;k++) v[k][j] = v[k][j-inc];
                j -= inc;
                if (j < inc) break;
            }
            w[j] = sw;
            for (k=0;k<m;k++) u[k][j] = su[k];
            for (k=0;k<n;k++) v[k][j] = sv[k];
        }
    }
    while (inc > 1);

    for (k=0;k<n;k++)  //Flip signs.
    {
        s=0;
        for (i=0;i<m;i++) if (u[i][k] < 0.) s++;
        for (j=0;j<n;j++) if (v[j][k] < 0.) s++;
        if (s > (m+n)/2)
        {
            for (i=0;i<m;i++) u[i][k] = -u[i][k];
            for (j=0;j<n;j++) v[j][k] = -v[j][k];
        }
    }
}

Doub SVD::pythag(const Doub a, const Doub b)
{
    //Computes .a2 Cb2/1=2 without destructive underflow or overflow.
    Doub absa=abs(a);
    Doub absb=abs(b);
    return (absa > absb ? absa*sqrt(1.0+NR_SQR(absb/absa)) :
    (absb == 0.0 ? 0.0 : absb*sqrt(1.0+NR_SQR(absa/absb))));
}





//void FitSVD::fit()
//{
//    Int i,j,k;
//    Doub tmp,thresh,sum;
//    if (x)
//    {
//        ma = funcs((*x)[0]).size(); //one dimensional case
//    }
//    else
//    {
//        ma = funcsmd(row(*xmd,0)).size(); //multi dimensional case
//    }
//
//    a.resize(ma);
//    covar.resize(ma,ma);
//    MatDoub aa(ndat,ma);
//    VecDoub b(ndat),afunc(ma);
//    for (i=0;i<ndat;i++)
//    {
//        if (x)
//        {
//            afunc=funcs((*x)[i]); //one dimensional case
//        }
//        else
//        {
//            afunc=funcsmd(row(*xmd,i)); //multi dimensional case
//        }
//        tmp=1.0/sig[i];
//        for (j=0;j<ma;j++) aa[i][j]=afunc[j]*tmp;
//        b[i]=y[i]*tmp;
//    }
//
//    SVD svd(aa);
//    thresh = (tol > 0. ? tol*svd.w[0] : -1.);
//    svd.solve(b,a,thresh);
//    chisq=0.0;
//
//    for (i=0;i<ndat;i++)
//    {
//        sum=0.;
//        for (j=0;j<ma;j++) sum += aa[i][j]*a[j];
//        chisq += NR_SQR(sum-b[i]);
//    }
//
//    for (i=0;i<ma;i++)
//    {
//        for (j=0;j<i+1;j++)
//        {
//            sum=0.0;
//            for (k=0;k<ma;k++) if (svd.w[k] > svd.tsh)
//            {
//                sum += svd.v[i][k]*svd.v[j][k]/NR_SQR(svd.w[k]);
//            }
//            covar[j][i]=covar[i][j]=sum;
//        }
//    }
//
//}











void FitSVDSimple::fit(VecDoub_I &x, VecDoub_I &y, VecDoub_I &weights, VecDoub_O &p, Doub &chisq) const
{
    Int i,j;
    Doub thresh,sum;

    Int ndat = y.size();
    VecInt idxMap(ndat);
    Int ndat_real = 0;
    Int i_;

    for (i=0;i<ndat;i++)
    {
        if (qIsFinite(y[i]) && qIsFinite(x[i]) && weights[i] > 0)
        {
            idxMap[ndat_real++] = i;
        }
    }

    Int ma = funcs((x)[0]).size(); //one dimensional case

    p.resize(ma);

    if (ndat_real < ma) //too few input values
    {
        for (i=0;i<ma;i++) p[i] = std::numeric_limits<double>::quiet_NaN();
        chisq = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    MatDoub aa(ndat_real,ma);
    VecDoub b(ndat_real);
    VecDoub afunc(ma);

    for (i=0;i<ndat_real;i++)
    {
        i_ = idxMap[i];
        afunc=funcs((x)[i_]); //one dimensional case
        for (j=0;j<ma;j++) aa[i][j]=afunc[j]*weights[i_];
        b[i]=y[i_]*weights[i_];
    }

    SVD svd(aa);
    thresh = (tol > 0. ? tol*svd.w[0] : -1.);
    svd.solve(b,p,thresh);
    chisq=0.0;

    for (i=0;i<ndat_real;i++)
    {
        sum=0.;
        for (j=0;j<ma;j++) sum += aa[i][j]*p[j];
        chisq += NR_SQR(sum-b[i]);
    }

    /*for (i=0;i<ma;i++)
    {
        for (j=0;j<i+1;j++)
        {
            sum=0.0;
            for (k=0;k<ma;k++) if (svd.w[k] > svd.tsh)
            {
                sum += svd.v[i][k]*svd.v[j][k]/NR_SQR(svd.w[k]);
            }
            covar[j][i]=covar[i][j]=sum;
        }
    }*/

}
