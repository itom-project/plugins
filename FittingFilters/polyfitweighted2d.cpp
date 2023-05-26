/* ********************************************************************
    Plugin "FittingFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "fittingfilters.h"

#include "DataObject/dataObjectFuncs.h"

#include <qvariant.h>
#include <qnumeric.h>

//#undef LAPACKE
#define FITWITHDOUBLEPRECISION 1 //1 if double precision, else 0 (float)

#if FITWITHDOUBLEPRECISION == 1 //double -> much more precise for big matrices
    typedef ito::float64 MATTYPE;
    #define TYPEID ito::tFloat64
    #define CVTYPEID CV_64FC1
    #define LAPACKE_FCT_NAME(SUFFIX) LAPACKE_d ## SUFFIX
#else
    typedef ito::float32 MATTYPE;
    #define TYPEID ito::tFloat32
    #define CVTYPEID CV_32FC1
    #define LAPACKE_FCT_NAME(SUFFIX) LAPACKE_s ## SUFFIX
#endif

#define CVMATREF_2DREAL(mat,m,n)  *( (MATTYPE*)(mat->data + mat->step[0]*(m) + mat->step[1]*(n)) )

#if defined LAPACKE
    #include "lapacke.h"

    #define DEF_REALMAT(NAME) MATTYPE *NAME = NULL; int NAME ## __rows;
    #define ALLOC_REALMAT(NAME, ROWS, COLS) NAME = new (std::nothrow) MATTYPE[(ROWS)*(COLS)]; NAME ## __rows = (ROWS);
    #define DELETE_REALMAT(NAME) delete[] NAME; NAME = NULL;
    #define REALMAT_REF(MAT,M,N)  MAT[(N) * MAT ##__rows + (M)]
#else
    #define DEF_REALMAT(NAME) cv::Mat *NAME = NULL;
    #define ALLOC_REALMAT(NAME, ROWS, COLS)  NAME = new (std::nothrow) cv::Mat((ROWS), (COLS), CVTYPEID);
    #define DELETE_REALMAT(NAME) delete NAME; NAME = NULL;
    #define REALMAT_REF(MAT,M,N)  CVMATREF_2DREAL(MAT,M,N)
#endif

//-------------------------------------------------------------------------------------------------------------------------
void printFortranMatrix(const char* name, float *vals, int m, int n)
{
    std::cout << "Matrix " << name << "\n" << std::endl;
    for (int i = 0 ; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            std::cout << vals[j * m + i] << ", ";
        }
        std::cout << "\n" << std::endl;
    }
}

//-------------------------------------------------------------------------------------------------------------------------
void printFortranMatrix(const char* name, double *vals, int m, int n)
{
    std::cout << "Matrix " << name << "\n" << std::endl;
    for (int i = 0 ; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            std::cout << vals[j * m + i] << ", ";
        }
        std::cout << "\n" << std::endl;
    }
}

//-------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::calcPolyfitWeighted2DRegularGrid(const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, double reduceFactor /*= -1.0*/, const ito::DataObject *weights /*= NULL*/)
{

    typedef MATTYPE _Tp;
    ito::RetVal retval;

    ito::DataObject *dataZFloat = apiCreateFromDataObject(dataZ, 2, TYPEID, NULL, &retval);

    if (retval.containsError()) return retval;

    int sizes[] = {dataZFloat->getSize(0), dataZFloat->getSize(0), dataZFloat->getSize(1), dataZFloat->getSize(1)};
    ito::DataObject *weightsFloat = weights ? apiCreateFromDataObject(weights,2,TYPEID,sizes,&retval) : NULL;

    if (orderX < 0 || orderY < 0)
    {
        retval += ito::RetVal(ito::retError,0,"orderX and orderY must be positive");
    }

    //int64 timer;

    if (!retval.containsError())
    {

        _Tp *Z = NULL;
        _Tp *X = NULL;
        _Tp *Y = NULL;
        _Tp *W = NULL;

        int nrOfPoints = 0;

        float offsets[] = { (float)dataZ->getAxisOffset(0), (float)dataZ->getAxisOffset(1) };
        float scales[] = { (float)dataZ->getAxisScale(0), (float)dataZ->getAxisScale(1) };
        _Tp *rowPtr;
        _Tp *rowPtrW;

        //timer = cv::getTickCount();

        int pointsY = qRound(reduceFactor*(orderY+1));
        int pointsX = qRound(reduceFactor*(orderX+1));

        if (reduceFactor < 1.0 || (pointsY > dataZ->getSize(0) && pointsX > dataZ->getSize(1))) //take all points
        {
            Z = new _Tp[ dataZ->getTotal() ];
            X = new _Tp[ dataZ->getTotal() ];
            Y = new _Tp[ dataZ->getTotal() ];
            W = new _Tp[ dataZ->getTotal() ];

            for (int m = 0; m < dataZ->getSize(0); ++m)
            {
                rowPtr = (_Tp*)dataZFloat->rowPtr(0,m);
                rowPtrW = weightsFloat ? (_Tp*)weightsFloat->rowPtr(0,m) : NULL;

                for (int n = 0; n < dataZ->getSize(1); ++n)
                {
                    W[nrOfPoints] = rowPtrW ? rowPtrW[n] : 1.0f;

                    if (qIsFinite(rowPtr[n]) && W[nrOfPoints] > 0.0)
                    {
                        Z[nrOfPoints] = rowPtr[n];
                        Y[nrOfPoints]  = ((_Tp)m - offsets[0])*scales[0];
                        X[nrOfPoints]  = ((_Tp)n - offsets[1])*scales[1];
                        nrOfPoints++;
                    }
                }
            }
        }
        else
        {
            pointsX = std::min( (int)dataZ->getSize(1), pointsX );
            pointsY = std::min( (int)dataZ->getSize(0), pointsY );

            Z = new _Tp[ pointsX * pointsY ];
            X = new _Tp[ pointsX * pointsY ];
            Y = new _Tp[ pointsX * pointsY ];
            W = new _Tp[ pointsX * pointsY ];

            int *xRanges = new int[2*pointsX];
            int *yRanges = new int[2*pointsY];
            int ystep = floor(dataZ->getSize(0) / (float)(pointsY));
            int xstep = floor(dataZ->getSize(1) / (float)(pointsX));

            for (int m = 0; m < pointsY; ++m)
            {
                yRanges[2*m] = m * ystep; //inclusive
                yRanges[2*m+1] = m * ystep+ ystep; //non-inclusive
            }

            for (int n = 0; n < pointsX; ++n)
            {
                xRanges[2*n] = n * xstep; //inclusive
                xRanges[2*n+1] = n * xstep + xstep; //non-inclusive
            }

            yRanges[2*pointsY-1] = dataZ->getSize(0);
            xRanges[2*pointsX-1] = dataZ->getSize(1);

            cv::RNG random((uint64)cv::getTickCount() ^ 0xa8e5f936);
            int randRow, randCol;
            cv::Mat_<_Tp> *cvPlane = (cv::Mat_<_Tp>*)(dataZFloat->get_mdata()[ dataZFloat->seekMat(0) ]);
            cv::Mat_<_Tp> *cvWPlane = weights ? (cv::Mat_<_Tp>*)(weights->get_mdata()[ dataZFloat->seekMat(0) ]) : NULL;

            int maxIterations = (xstep * ystep);
            int iter = 0;

            for (int m = 0; m < (pointsY); ++m)
            {
                for (int n = 0; n < (pointsX); ++n)
                {
                    iter = 0;
                    while (iter++ < maxIterations) //if all values in the rect are NaN it should break sometimes and don't use any point from this. then X,Y,Z,W are longer than number of elements. this is not important
                    {
                        randRow = random.uniform( yRanges[m*2], yRanges[m*2+1] );
                        randCol = random.uniform( xRanges[n*2], xRanges[n*2+1] );

                        W[nrOfPoints] = cvWPlane ? CVMATREF_2DREAL(cvWPlane,randRow,randCol) : 1.0;

                        if (qIsFinite(CVMATREF_2DREAL(cvPlane,randRow,randCol)) && W[nrOfPoints] > 0.0)
                        {
                            Z[nrOfPoints] = CVMATREF_2DREAL(cvPlane,randRow,randCol);
                            Y[nrOfPoints]  = ((_Tp)randRow - offsets[0])*scales[0];
                            X[nrOfPoints]  = ((_Tp)randCol - offsets[1])*scales[1];
                            nrOfPoints++;
                            break;
                        }
                    }
                }
            }

            delete[] xRanges; xRanges = NULL;
            delete[] yRanges; yRanges = NULL;

        }

        retval += calcPolyfitWeighted2D(X, Y, Z, W, orderX, orderY, nrOfPoints, coefficients);

        delete[] Z;
        delete[] X;
        delete[] Y;
        delete[] W;
    }

    delete dataZFloat;
    if (weightsFloat)
    {
        delete weightsFloat;
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::calcPolyfitWeighted2DSinglePoints(const ito::DataObject *dataX, const ito::DataObject *dataY, const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, const ito::DataObject *weights /*= NULL*/)
{

    typedef MATTYPE _Tp;
    ito::RetVal retval;

    ito::DataObject dataXFloat = ito::dObjHelper::squeezeConvertCheck2DDataObject(dataX, "dataX", ito::Range::all(), ito::Range::all(), retval, TYPEID, 0);
    ito::Range ySize(dataXFloat.getSize(0), dataXFloat.getSize(0));
    ito::Range xSize(dataXFloat.getSize(1), dataXFloat.getSize(1));
    ito::DataObject dataYFloat = ito::dObjHelper::squeezeConvertCheck2DDataObject(dataY, "dataY", ySize, xSize, retval, TYPEID, 0);
    ito::DataObject dataZFloat = ito::dObjHelper::squeezeConvertCheck2DDataObject(dataZ, "dataZ", ySize, xSize, retval, TYPEID, 0);

    if (retval.containsError()) return retval;

    int sizes[] = {dataZFloat.getSize(0), dataZFloat.getSize(0), dataZFloat.getSize(1), dataZFloat.getSize(1)};
    ito::DataObject *weightsFloat = weights ? apiCreateFromDataObject(weights,2,TYPEID,sizes,&retval) : NULL;

    if (orderX < 0 || orderY < 0)
    {
        retval += ito::RetVal(ito::retError,0,"orderX and orderY must be positive");
    }

    if (!retval.containsError())
    {
        _Tp *Z = NULL;
        _Tp *X = NULL;
        _Tp *Y = NULL;
        _Tp *W = NULL;

        int nrOfPoints = 0;

        _Tp *rowPtrX, *rowPtrY, *rowPtrZ;
        _Tp *rowPtrW;

        Z = new _Tp[ dataXFloat.getTotal() ];
        X = new _Tp[ dataXFloat.getTotal() ];
        Y = new _Tp[ dataXFloat.getTotal() ];
        W = new _Tp[ dataXFloat.getTotal() ];

        for (int m = 0; m < dataZ->getSize(0); ++m)
        {
            rowPtrZ = (_Tp*)dataZFloat.rowPtr(0,m);
            rowPtrX = (_Tp*)dataXFloat.rowPtr(0,m);
            rowPtrY = (_Tp*)dataYFloat.rowPtr(0,m);
            rowPtrW = weightsFloat ? (_Tp*)weightsFloat->rowPtr(0,m) : NULL;

            for (int n = 0; n < dataZ->getSize(1); ++n)
            {
                W[nrOfPoints] = rowPtrW ? rowPtrW[n] : 1.0f;

                if (qIsFinite(rowPtrZ[n]) && qIsFinite(rowPtrY[n]) && qIsFinite(rowPtrX[n]) && W[nrOfPoints] > 0.0)
                {
                    Z[nrOfPoints] = rowPtrZ[n];
                    Y[nrOfPoints]  = rowPtrY[n];
                    X[nrOfPoints]  = rowPtrX[n];
                    nrOfPoints++;
                }
            }
        }

        retval += calcPolyfitWeighted2D(X, Y, Z, W, orderX, orderY, nrOfPoints, coefficients);

        delete[] Z;
        delete[] X;
        delete[] Y;
        delete[] W;
    }

    if (weightsFloat)
    {
        delete weightsFloat;
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*static*/ ito::RetVal FittingFilters::calcPolyfitWeighted2D(const _Tp *X, const _Tp *Y, const _Tp *Z, const _Tp *W, int orderX, int orderY, int nrOfPoints, std::vector<double> &coefficients)
{
    ito::RetVal retval;

    //http://www.mathworks.com/matlabcentral/fileexchange/13719-2d-weighted-polynomial-fitting-and-evaluation
    // modified vandermore matrix for different orders in x and y come from
    // http://poquitopicante.blogspot.de/2013/03/horners-method-in-2d.html

    //std::cout << "vector-aufbau: "  << (double)(cv::getTickCount() - timer)/cv::getTickFrequency() << "\n" << std::endl;
    //timer = cv::getTickCount();

    //Polynomial is:
    //
    // if (order_x=n) <= (order_y=m):
    // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
    //
    // else:
    // f(x,y) = \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
    //
    // number vcols of coefficients is:
    // a = min(n,m)
    // b = max(n,m)
    //
    // vcols = (a+1)*(b+1) - a(a+1)/2 = (a+1)(1+b-a/2) = (a+1)(2+2b-a)/2 (in order to avoid rounding problems)
    //

    //Construct weighted Vandermonde matrix.
    //now X,Y,Z,W have the same type and size [nrOfPoints,1]
    int m = orderY;
    int n = orderX;

    int vcols = (std::min(m,n)+1)*(2+2*std::max(m,n)-std::min(m,n))/2;

    DEF_REALMAT(V)
    DEF_REALMAT(Vals)

    if (vcols > nrOfPoints)
    {
        retval += ito::RetVal(ito::retError,0,"too less points for determination of the polynomial fit");
    }
    else
    {
        ALLOC_REALMAT(V, nrOfPoints, vcols) //cv::Mat V(nrOfPoints, vcols, CV_32FC1);
        ALLOC_REALMAT(Vals, nrOfPoints, 1)  //cv::Mat Vals(nrOfPoints, 1, CV_32FC1);

        if (V == NULL)
        {
            retval += ito::RetVal(ito::retError,0,"error allocating memory for Vandermonde matrix V");
        }
        else if (Vals == NULL)
        {
            retval += ito::RetVal(ito::retError,0,"error allocating memory for right hand side of Vandermonde matrix");
        }
    }

    if (!retval.containsError())
    {

        //copy weights to first column //V[:,0] = W
        for (int i = 0; i < nrOfPoints; ++i)
        {
            REALMAT_REF(V,i,0) = W[i];
            REALMAT_REF(Vals,i,0) = Z[i]*W[i];
        }

        int majorColumn = 0;
        int currentColumn = 0;

        if (n <= m) // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
        {
            for (int i = 0; i <= n; ++i)
            {
                //this handles j = 0 (for i = 0 this is already done above)
                if (i > 0)
                {
                    currentColumn ++;
                    for (int r = 0; r < nrOfPoints; ++r)
                    {
                        REALMAT_REF(V,r,currentColumn) = X[r] * REALMAT_REF(V,r,majorColumn);
                    }
                    majorColumn = currentColumn;
                }

                for (int j = 1; j <= m-i; ++j)
                {
                    currentColumn ++;
                    for (int r = 0; r < nrOfPoints; ++r)
                    {
                        REALMAT_REF(V,r,currentColumn) = Y[r] * REALMAT_REF(V,r,currentColumn-1);
                    }
                }
            }
        }
        else // \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
        {
            for (int j = 0; j <= m; ++j)
            {
                //this handles i = 0 (for j = 0 this is already done above)
                if (j > 0)
                {
                    currentColumn ++;
                    for (int r = 0; r < nrOfPoints; ++r)
                    {
                        REALMAT_REF(V,r,currentColumn) = Y[r] * REALMAT_REF(V,r,majorColumn);
                    }
                    majorColumn = currentColumn;
                }

                for (int i = 1; i <= n-j; ++i)
                {
                    currentColumn ++;
                    for (int r = 0; r < nrOfPoints; ++r)
                    {
                        REALMAT_REF(V,r,currentColumn) = X[r] * REALMAT_REF(V,r,currentColumn-1);
                    }
                }
            }
        }



        //std::cout << "vandermonde aufbau: "  << (double)(cv::getTickCount() - timer)/cv::getTickFrequency() << "\n" << std::endl;
        //timer = cv::getTickCount();

#ifndef LAPACKE
        try
        {
            //solve least squares problem
            cv::Mat result;
            cv::solve( *V, *Vals, result, cv::DECOMP_QR ); //cv::DECOMP_SVD );

            coefficients.clear();
            coefficients.reserve(result.total());

            cv::MatConstIterator_<_Tp> it = result.begin<_Tp>(), it_end = result.end<_Tp>();
            for(; it != it_end; ++it)
            {
                coefficients.push_back( *it );
            }
        }
        catch(cv::Exception exc)
        {
            retval += ito::RetVal(ito::retError,0, (exc.err).c_str());
            return retval;
        }
#else
        //direct solving
        lapack_int info, lm, ln, lnrhs, llda, lldb, lmn, lwork;
        lm = nrOfPoints;
        ln = vcols;
        lnrhs = 1;
        llda = nrOfPoints;
        lldb = nrOfPoints;
        lmn = std::min(lm,ln);

        if (1)
        {

            //this is the faster implementation: first qr, then solve (see numpy linalg.qr)
            _Tp *tau = new _Tp[ lmn ];
            memset(tau, 0, sizeof(_Tp) * lmn);
//                lwork = 1;
            _Tp *work = new _Tp[1];
            work[0] = 0;

            //printFortranMatrix("V", V, lm, ln);

            //calculate optimal size of work data 'work'
            info = LAPACKE_FCT_NAME(geqrf_work)(LAPACK_COL_MAJOR, lm, ln, V, llda, tau, work, -1);
            lwork = static_cast<int>( std::abs( work[0] ));
            delete[] work;

            if (info != 0)
            {
                retval += ito::RetVal(ito::retError,0,"error when calculating optimal work for qr factorization");
            }
            else
            {
                //do qr decomposition
                work = new _Tp[lwork];
                memset(work,0,sizeof(_Tp)*lwork);

                info = LAPACKE_FCT_NAME(geqrf_work)(LAPACK_COL_MAJOR, lm, ln, V, llda, tau, work, lwork);
                delete[] work;

                if (info != 0)
                {
                    retval += ito::RetVal(ito::retError,0,"illegal value in input data (qr factorization)");
                }
                else
                {
                    //generate R (size min(M,N) x N)
                    _Tp *R = new _Tp[lmn * ln];
                    memset(R, 0, sizeof(_Tp) * ln * lmn);

                    for (lapack_int i=0; i < lmn; ++i)
                    {
                        memcpy( &(R[i * lmn]), &(V[i * lm]), sizeof(_Tp) * (i+1));
                    }

                    //printFortranMatrix("R", R, lmn, ln);


                    //now build orthonormal matrix q from V

                    //determine optimal lwork
                    lwork = 1;
                    work = new _Tp[lwork];
                    memset(work,0,sizeof(_Tp)*lwork);
                    info = LAPACKE_FCT_NAME(orgqr_work)(LAPACK_COL_MAJOR, lm, lmn, lmn, V, lm, tau, work, -1);
                    lwork = static_cast<int>(std::abs(work[0]));
                    delete[] work;

                    if (info != 0)
                    {
                        retval += ito::RetVal(ito::retError,0,"error when calculating optimal work for qr factorization");
                    }
                    else
                    {
                        //compute q (size: M x min(M,n))
                        work = new _Tp[lwork];
                        memset(work,0,sizeof(_Tp)*lwork);
                        info = LAPACKE_FCT_NAME(orgqr_work)(LAPACK_COL_MAJOR, lm, lmn, lmn, V, lm, tau, work, lwork);
                        delete[] work;

                        if (info != 0)
                        {
                            retval += ito::RetVal(ito::retError,0,"error when calculating optimal work for qr factorization");
                        }
                        else
                        {
                            //_Tp *Q = new _Tp[lmn * lm]; //lm x lmn
                            //memcpy(Q,V, sizeof(_Tp) * lmn * lm);
                            //printFortranMatrix("Q", Q, lm, lmn);


                            //now solve equation R*x = Q.T * Vals for x

                            //Q.T * Vals = B
                            // sizeof B [lmn x 1]
                            // Q are the first (lm * lmn) elements of V
                            _Tp *B = new _Tp[lmn];
                            memset(B, 0, sizeof(_Tp) * lmn);

                            for (int i = 0; i < lmn; ++i)
                            {
                                for (int j = 0; j < lm; ++j)
                                {
                                    //B[i] = Q[j,i] * Vals[j]
                                    B[i] += V[ i * lm + j ] * Vals[j]; //since the first lm*lmn elements of V are the first lm*lmn elements of Q, we don't need to create Q and can directly use V
                                }
                            }

                            /*delete[] Q;
                            Q = NULL;*/
                            //printFortranMatrix("Vals", Vals, lm, 1);
                            //printFortranMatrix("B", B, lmn, 1);

                            lapack_int *pivots = new lapack_int[lm];
                            memset(pivots,0,sizeof(lapack_int) * lm);
                            info = LAPACKE_FCT_NAME(gesv_work)(LAPACK_COL_MAJOR, lmn, 1, R, lmn, pivots, B, lm);

                            delete[] pivots;
                            pivots = NULL;

                            //std::cout << "qr-solver (lapack): "  << (double)(cv::getTickCount() - timer)/cv::getTickFrequency() << "\n" << std::endl;
                            //timer = cv::getTickCount();

                            if (info == 0)
                            {
                                coefficients.clear();
                                coefficients.reserve(vcols);

                                for(int i = 0; i < vcols; ++i)
                                {
                                    coefficients.push_back( B[i] );
                                }
                            }
                            else if (info < 0)
                            {
                                retval += ito::RetVal(ito::retError,0,"illegal value in input data");
                            }
                            else
                            {
                                retval += ito::RetVal(ito::retError,0,"Matrix has no full rank. Please provide more independent input data points for the given orders");
                            }
                            delete[] B;
                            B = NULL;
                        }
                    }

                    delete[] R;
                    R = NULL;

                }
            }

            delete[] tau;

        }
        else //direct solving (slower)
        {
            info = LAPACKE_FCT_NAME(gels)(LAPACK_COL_MAJOR, 'N', lm, ln, lnrhs, V, llda, Vals, lldb);

            //std::cout << "qr-solver (lapack): "  << (double)(cv::getTickCount() - timer)/cv::getTickFrequency() << "\n" << std::endl;
            //timer = cv::getTickCount();

            if (info == 0)
            {
                coefficients.clear();
                coefficients.reserve(vcols);

                for(int i = 0; i < vcols; ++i)
                {
                    coefficients.push_back( Vals[i] );
                }
            }
            else if (info < 0)
            {
                retval += ito::RetVal(ito::retError,0,"illegal value in input data");
            }
            else
            {
                retval += ito::RetVal(ito::retError,0,"Matrix has no full rank. Please provide more independent input data points for the given orders");
            }
        }
#endif
    }

    DELETE_REALMAT(V)
    DELETE_REALMAT(Vals)

    return retval;
}



//-------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::calcPolyval2DRegularGrid(ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> &coefficients)
{
    ito::RetVal retval;

    typedef MATTYPE _Tp;

    if (dataZ->getDims() == 0)
    {
        return retval;
    }
    else if (dataZ->getDims() != 2)
    {
        retval += ito::RetVal(ito::retError,0,"dataZ must have two dimensions");
    }
    else if (coefficients.size() <= 0)
    {
        retval += ito::RetVal(ito::retError,0,"you must indicate at least 1 coefficient");
    }
    else if (orderX < 0 || orderY < 0)
    {
        retval += ito::RetVal(ito::retError,0,"orderX and orderY must be positive");
    }
    else
    {
        //Polynomial is:
        //
        // if (order_x=n) <= (order_y=m):
        // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
        //
        // else:
        // f(x,y) = \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
        //
        // number vcols of coefficients is:
        // a = min(n,m)
        // b = max(n,m)
        //
        // vcols = (a+1)*(b+1) - a(a+1)/2 = (a+1)(1+b-a/2) = (a+1)(2+2b-a)/2 (in order to avoid rounding problems)
        //

        //Construct weighted Vandermonde matrix.
        //now X,Y,Z,W have the same type and size [nrOfPoints,1]
        int m = orderY;
        int n = orderX;

        int vcols = (std::min(m,n)+1)*(2+2*std::max(m,n)-std::min(m,n))/2;

        if ( vcols != coefficients.size())
        {
            retval += ito::RetVal::format(ito::retError,0,"The given orders (%i,%i) requires %i coefficients", n, m, vcols);
        }
        else
        {
            ito::DataObject result;
            dataZ->convertTo(result, TYPEID);
            cv::Mat *result_ = (cv::Mat*)result.get_mdata()[ result.seekMat(0) ];

            _Tp offsets[] = { (_Tp)dataZ->getAxisOffset(0), (_Tp)dataZ->getAxisOffset(1) };
            _Tp scales[] = { (_Tp)dataZ->getAxisScale(0), (_Tp)dataZ->getAxisScale(1) };

            _Tp *x = new _Tp[result_->cols];
            _Tp *y = new _Tp[result_->rows];

            for (int i = 0; i < dataZ->getSize(0); ++i)
            {
                y[i]  = ((float)i - offsets[0])*scales[0];
            }

            for (int i = 0; i < dataZ->getSize(1); ++i)
            {
                x[i]  = ((float)i - offsets[1])*scales[1];
            }

            const double *coeff = coefficients.data();
            int ordercolumn = 1;
            int lp = (int)coefficients.size();

            //in this implementation, the resulting matrix is handled row by row

            cv::Mat P;
            cv::Mat *Vrow = NULL;

            if (TYPEID == ito::tFloat32)
            {
                Vrow = new cv::Mat(result_->cols, lp, CV_32FC1);
                P = cv::Mat(lp, 1, CV_32FC1);
            }
            else
            {
                Vrow = new cv::Mat(result_->cols, lp, CV_64FC1);
                P = cv::Mat(lp, 1, CV_64FC1);
            }

            cv::MatIterator_<_Tp> it = P.begin<_Tp>(), it_end = P.end<_Tp>();
            int i = 0;
            for(; it != it_end; ++it)
            {
                *it = coefficients[i++];
            }

            int max_order = (std::sqrt((float)(1 + 8*lp)) - 3) / 2;

            //write 1.0 in the first column of Vrow
            Vrow->col(0) = 1.0;

            int majorColumn = 0;
            int currentColumn = 0;
            cv::Mat temp;
            int cols = dataZ->getSize(1);

            for (int row = 0; row < dataZ->getSize(0); ++row)
            {
                majorColumn = 0;
                currentColumn = 0;

                if (n <= m) // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
                {
                    for (int i = 0; i <= n; ++i)
                    {
                        //this handles j = 0 (for i = 0 this is already done above)
                        if (i > 0)
                        {
                            currentColumn ++;
                            for (int col = 0; col < cols; ++col)
                                CVMATREF_2DREAL(Vrow,col,currentColumn) = x[col] * CVMATREF_2DREAL(Vrow,col,majorColumn);
                            majorColumn = currentColumn;
                        }

                        for (int j = 1; j <= m-i; ++j)
                        {
                            currentColumn ++;
                            for (int col = 0; col < cols; ++col)
                                CVMATREF_2DREAL(Vrow,col,currentColumn) = y[row] * CVMATREF_2DREAL(Vrow,col,currentColumn-1);
                        }
                    }
                }
                else // \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
                {
                    for (int j = 0; j <= m; ++j)
                    {
                        //this handles i = 0 (for j = 0 this is already done above)
                        if (j > 0)
                        {
                            currentColumn ++;
                            for (int col = 0; col < cols; ++col)
                                CVMATREF_2DREAL(Vrow,col,currentColumn) = y[row] * CVMATREF_2DREAL(Vrow,col,majorColumn);
                            majorColumn = currentColumn;
                        }

                        for (int i = 1; i <= n-j; ++i)
                        {
                            currentColumn ++;
                            for (int col = 0; col < cols; ++col)
                                CVMATREF_2DREAL(Vrow,col,currentColumn) = x[col] * CVMATREF_2DREAL(Vrow,col,currentColumn-1);
                        }
                    }
                }

                temp = (*Vrow) * P;

                //here it is assumed that data lies continuously in temp
                memcpy( result_->ptr(row), temp.data, sizeof(_Tp) * result_->cols );
            }

            delete Vrow;

            delete[] x;
            delete[] y;

            *dataZ = result;
        }
    }

    return retval;
}



//-------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::calcPolyval2DSinglePoints(const ito::DataObject *dataX, const ito::DataObject *dataY, ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> &coefficients)
{
    ito::RetVal retval;

    typedef MATTYPE _Tp;

    ito::DataObject dataXFloat = ito::dObjHelper::squeezeConvertCheck2DDataObject(dataX, "dataX", ito::Range::all(), ito::Range::all(), retval, TYPEID, 0);
    ito::Range ySize(dataXFloat.getSize(0), dataXFloat.getSize(0));
    ito::Range xSize(dataXFloat.getSize(1), dataXFloat.getSize(1));
    ito::DataObject dataYFloat = ito::dObjHelper::squeezeConvertCheck2DDataObject(dataY, "dataY", ySize, xSize, retval, TYPEID, 0);

    if (coefficients.size() <= 0)
    {
        retval += ito::RetVal(ito::retError,0,"you must indicate at least 1 coefficient");
    }
    else if (orderX < 0 || orderY < 0)
    {
        retval += ito::RetVal(ito::retError,0,"orderX and orderY must be positive");
    }
    else
    {
        //make x and y flat:
        int total = dataXFloat.getSize(0) * dataXFloat.getSize(1);
        cv::Mat xFlatten = dataXFloat.getCvPlaneMat(0)->reshape(0, 1);
        cv::Mat yFlatten = dataYFloat.getCvPlaneMat(0)->reshape(0, 1);

        //Polynomial is:
        //
        // if (order_x=n) <= (order_y=m):
        // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
        //
        // else:
        // f(x,y) = \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
        //
        // number vcols of coefficients is:
        // a = min(n,m)
        // b = max(n,m)
        //
        // vcols = (a+1)*(b+1) - a(a+1)/2 = (a+1)(1+b-a/2) = (a+1)(2+2b-a)/2 (in order to avoid rounding problems)
        //

        //Construct weighted Vandermonde matrix.
        //now X,Y,Z,W have the same type and size [nrOfPoints,1]
        int m = orderY;
        int n = orderX;

        int vcols = (std::min(m,n)+1)*(2+2*std::max(m,n)-std::min(m,n))/2;

        if ( vcols != coefficients.size())
        {
            retval += ito::RetVal::format(ito::retError,0,"The given orders (%i,%i) requires %i coefficients", n, m, vcols);
        }
        else
        {
            cv::Mat result(1, total, TYPEID == ito::tFloat32 ? CV_32FC1 : CV_64FC1);

            int ordercolumn = 1;
            int lp = (int)coefficients.size();

            //in this implementation, the resulting matrix is handled row by row

            cv::Mat P;
            cv::Mat *Vrow = NULL;

            if (TYPEID == ito::tFloat32)
            {
                Vrow = new cv::Mat(total, lp, CV_32FC1);
                P = cv::Mat(lp, 1, CV_32FC1);
            }
            else
            {
                Vrow = new cv::Mat(total, lp, CV_64FC1);
                P = cv::Mat(lp, 1, CV_64FC1);
            }

            cv::MatIterator_<_Tp> it = P.begin<_Tp>(), it_end = P.end<_Tp>();
            int i = 0;
            for(; it != it_end; ++it)
            {
                *it = coefficients[i++];
            }

            int max_order = (std::sqrt((float)(1 + 8*lp)) - 3) / 2;

            //write 1.0 in the first column of Vrow
            Vrow->col(0) = 1.0;

            int majorColumn = 0;
            int currentColumn = 0;
            cv::Mat temp;

            const _Tp *xRowPtr = (_Tp*)xFlatten.ptr(0);
            const _Tp *yRowPtr = (_Tp*)yFlatten.ptr(0);
            int cols = dataXFloat.getSize(1);

            majorColumn = 0;
            currentColumn = 0;

            if (n <= m) // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
            {
                for (int i = 0; i <= n; ++i)
                {
                    //this handles j = 0 (for i = 0 this is already done above)
                    if (i > 0)
                    {
                        currentColumn ++;
                        for (int idx = 0; idx < total; ++idx)
                        {
                            CVMATREF_2DREAL(Vrow,idx,currentColumn) = xRowPtr[idx] * CVMATREF_2DREAL(Vrow,idx,majorColumn);
                        }
                        majorColumn = currentColumn;
                    }

                    for (int j = 1; j <= m-i; ++j)
                    {
                        currentColumn ++;
                        for (int idx = 0; idx < total; ++idx)
                        {
                            CVMATREF_2DREAL(Vrow,idx,currentColumn) = yRowPtr[idx] * CVMATREF_2DREAL(Vrow,idx,currentColumn-1);
                        }
                    }
                }
            }
            else // \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
            {
                for (int j = 0; j <= m; ++j)
                {
                    //this handles i = 0 (for j = 0 this is already done above)
                    if (j > 0)
                    {
                        currentColumn ++;
                        for (int idx = 0; idx < total; ++idx)
                        {
                            CVMATREF_2DREAL(Vrow,idx,currentColumn) = yRowPtr[idx] * CVMATREF_2DREAL(Vrow,idx,majorColumn);
                        }
                        majorColumn = currentColumn;
                    }

                    for (int i = 1; i <= n-j; ++i)
                    {
                        currentColumn ++;
                        for (int idx = 0; idx < total; ++idx)
                        {
                            CVMATREF_2DREAL(Vrow,idx,currentColumn) = xRowPtr[idx] * CVMATREF_2DREAL(Vrow,idx,currentColumn-1);
                        }
                    }
                }
            }

            temp = (*Vrow) * P;

            //here it is assumed that data lies continuously in temp
            memcpy( result.data, temp.data, sizeof(_Tp) * total );

            delete Vrow;

            int sizes[] = {dataXFloat.getSize(0), dataXFloat.getSize(1)};
            cv::Mat result2 = result.reshape(0, sizes[0]);
            *dataZ = ito::DataObject(2, sizes, TYPEID, &result2, 1);
        }
    }

    return retval;
}
