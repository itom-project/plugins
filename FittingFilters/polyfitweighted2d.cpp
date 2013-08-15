#include "fittingfilters.h"

#include <qvariant.h>
#include <qnumeric.h>

//#undef LAPACKE
#define FITWITHDOUBLEPRECISION 1 //1 if double precision, else 0 (float)

#if FITWITHDOUBLEPRECISION == 1 //double -> much more precise for big matrices
    typedef ito::float64 MATTYPE;
    #define TYPEID ito::tFloat64  
    #define LAPACKE_FCT_NAME(SUFFIX) LAPACKE_d ## SUFFIX
#else
    typedef ito::float32 MATTYPE;
    #define TYPEID ito::tFloat32 
    #define LAPACKE_FCT_NAME(SUFFIX) LAPACKE_s ## SUFFIX
#endif

#define CVMATREF_2DREAL(mat,m,n)  *( (MATTYPE*)(mat->data + mat->step[0]*(m) + mat->step[1]*(n)) ) 

#if defined LAPACKE
    #include "lapacke.h"
    
    #define CREATE_REALMAT(NAME, ROWS, COLS)  MATTYPE *NAME = new (std::nothrow) MATTYPE[(ROWS)*(COLS)]; size_t NAME ## __rows = (ROWS);   
    #define DELETE_REALMAT(NAME) delete[] NAME; NAME = NULL;
    #define REALMAT_REF(MAT,M,N)  MAT[(N) * MAT ##__rows + (M)]
#else
    #define CREATE_REALMAT(NAME, ROWS, COLS)  cv::Mat *NAME = new (std::nothrow) cv::Mat((ROWS), (COLS), CV_32FC1);
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
/*static*/ ito::RetVal FittingFilters::calcPolyfitWeighted2D(const ito::DataObject *dataZ, int orderX, int orderY, std::vector<double> &coefficients, double margin /*= 0.2*/, const ito::DataObject *weights /*= NULL*/)
{

    typedef MATTYPE _Tp;
    ito::RetVal retval;

    ito::DataObject *dataZFloat = apiCreateFromDataObject(dataZ, 2, TYPEID, NULL, &retval);

    if (retval.containsError()) return retval;

    size_t sizes[] = {dataZFloat->getSize(0), dataZFloat->getSize(0), dataZFloat->getSize(1), dataZFloat->getSize(1)};
    ito::DataObject *weightsFloat = weights ? apiCreateFromDataObject(weights,2,TYPEID,sizes,&retval) : NULL;

    if (orderX < 0 || orderY < 0)
    {
        retval += ito::RetVal(ito::retError,0,"orderX and orderY must be positive");
    }

    //int64 timer;

    if (!retval.containsError())
    {

        _Tp *Z = new _Tp[ dataZ->getTotal() ];
        _Tp *X = new _Tp[ dataZ->getTotal() ];
        _Tp *Y = new _Tp[ dataZ->getTotal() ];
        _Tp *W = new _Tp[ dataZ->getTotal() ];

        size_t nrOfPoints = 0;

        float offsets[] = { (float)dataZ->getAxisOffset(0), (float)dataZ->getAxisOffset(1) };
        float scales[] = { (float)dataZ->getAxisScale(0), (float)dataZ->getAxisScale(1) };
        _Tp *rowPtr;
        _Tp *rowPtrW;

        //timer = cv::getTickCount();

        for (size_t m = 0; m < dataZ->getSize(0); ++m)
        {
            rowPtr = (_Tp*)dataZFloat->rowPtr(0,m);
            rowPtrW = weightsFloat ? (_Tp*)weightsFloat->rowPtr(0,m) : NULL;

            for (size_t n = 0; n < dataZ->getSize(1); ++n)
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
        size_t m = orderY;
        size_t n = orderX;

        int vcols = (std::min(m,n)+1)*(2+2*std::max(m,n)-std::min(m,n))/2;

        CREATE_REALMAT(V, nrOfPoints, vcols) //cv::Mat V(nrOfPoints, vcols, CV_32FC1);
        CREATE_REALMAT(Vals, nrOfPoints, 1)  //cv::Mat Vals(nrOfPoints, 1, CV_32FC1); 

        if (V == NULL)
        {
            retval += ito::RetVal(ito::retError,0,"error allocating memory for Vandermonde matrix V");
        }
        else if (Vals == NULL)
        {
            retval += ito::RetVal(ito::retError,0,"error allocating memory for right hand side of Vandermonde matrix");
        }

        if (!retval.containsError())
        {

            //copy weights to first column //V[:,0] = W
            for (size_t i = 0; i < nrOfPoints; ++i)
            {
                REALMAT_REF(V,i,0) = W[i];
                REALMAT_REF(Vals,i,0) = Z[i]*W[i];
            }

            size_t majorColumn = 0;
            size_t currentColumn = 0;

            if (n <= m) // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
            {
                for (size_t i = 0; i <= n; ++i)
                {
                    //this handles j = 0 (for i = 0 this is already done above)
                    if (i > 0)
                    {
                        currentColumn ++;
                        for (size_t r = 0; r < nrOfPoints; ++r)
                        {
                            REALMAT_REF(V,r,currentColumn) = X[r] * REALMAT_REF(V,r,majorColumn);
                        }
                        majorColumn = currentColumn;
                    }

                    for (size_t j = 1; j <= m-i; ++j)
                    {
                        currentColumn ++;
                        for (size_t r = 0; r < nrOfPoints; ++r)
                        {
                            REALMAT_REF(V,r,currentColumn) = Y[r] * REALMAT_REF(V,r,currentColumn-1);
                        }
                    }
                }
            }
            else // \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
            {
                for (size_t j = 0; j <= m; ++j)
                {
                    //this handles i = 0 (for j = 0 this is already done above)
                    if (j > 0)
                    {
                        currentColumn ++;
                        for (size_t r = 0; r < nrOfPoints; ++r)
                        {
                            REALMAT_REF(V,r,currentColumn) = Y[r] * REALMAT_REF(V,r,majorColumn);
                        }
                        majorColumn = currentColumn;
                    }

                    for (size_t i = 1; i <= n-j; ++i)
                    {
                        currentColumn ++;
                        for (size_t r = 0; r < nrOfPoints; ++r)
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
                //std::cout << *V << "\n" << std::endl;
                //std::cout << *Vals << "\n" << std::endl;

                //solve least squares problem
                cv::Mat result;
                cv::solve( *V, *Vals, result, cv::DECOMP_QR ); //cv::DECOMP_SVD );

                //std::cout << *V << "\n" << std::endl;

                //std::cout << "qr-solver: "  << (double)(cv::getTickCount() - timer)/cv::getTickFrequency() << "\n" << std::endl;
                //timer = cv::getTickCount();

                coefficients.clear();
                coefficients.reserve(result.total());

                cv::MatConstIterator_<_Tp> it = result.begin<_Tp>(), it_end = result.end<_Tp>();
                for(; it != it_end; ++it)
                {
                    coefficients.push_back( *it );
                }

                //Eigen::MatrixXf V_e = Eigen::MatrixXf::Random(1000*1000,21);
                //Eigen::MatrixXf Vals_e = Eigen::MatrixXf::Random(1000*1000,1);
                ////Eigen::Map< Eigen::Matrix<ito::float32,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>, Eigen::Unaligned, Eigen::OuterStride<> > V_e((ito::float32*)V.data, V.rows, V.cols, Eigen::OuterStride<>(V.step[0]));
                ////Eigen::Map< Eigen::Matrix<ito::float32,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>, Eigen::Unaligned, Eigen::OuterStride<> > Vals_e((ito::float32*)Vals.data, Vals.rows, V.cols, Eigen::OuterStride<>(Vals.step[0]));
                ////Eigen::HouseholderQR<Eigen::MatrixXf> qr(V_e);
                ////Eigen::MatrixXf Q = qr.householderQ();
                //Eigen::JacobiSVD<Eigen::MatrixXf> svd(V_e, Eigen::ComputeThinU | Eigen::ComputeThinV);
                //std::cout << "A least-squares solution of m*x = rhs is:" << std::endl << svd.solve(Vals_e) << std::endl;
            
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
                lwork = 1;
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

        delete[] Z;
        delete[] X;
        delete[] Y;
        delete[] W;
    }

    delete dataZFloat;
    if(weightsFloat) delete weightsFloat;
    
    return retval;
}



//-------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FittingFilters::calcPolyval2D(ito::DataObject *dataZ, int orderX, int orderY, const std::vector<double> coefficients)
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
        size_t m = orderY;
        size_t n = orderX;

        int vcols = (std::min(m,n)+1)*(2+2*std::max(m,n)-std::min(m,n))/2;

        if ( vcols != coefficients.size())
        {
            retval += ito::RetVal::format(ito::retError,0,"The given orders (%i,%i) requires %i coefficients", n, m, vcols);
        }

        ito::DataObject result;
        dataZ->convertTo(result, TYPEID);
        cv::Mat *result_ = (cv::Mat*)result.get_mdata()[ result.seekMat(0) ];

        _Tp offsets[] = { (_Tp)dataZ->getAxisOffset(0), (_Tp)dataZ->getAxisOffset(1) };
        _Tp scales[] = { (_Tp)dataZ->getAxisScale(0), (_Tp)dataZ->getAxisScale(1) };
        
        _Tp *x = new _Tp[result_->cols];
        _Tp *y = new _Tp[result_->rows];

        for (size_t i = 0; i < dataZ->getSize(0); ++i)
        {
            y[i]  = ((float)i - offsets[0])*scales[0];
        }

        for (size_t i = 0; i < dataZ->getSize(1); ++i)
        {
            x[i]  = ((float)i - offsets[1])*scales[1];
        }

        const double *coeff = coefficients.data();
        int ordercolumn = 1;
        int lp = coefficients.size();

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

        size_t majorColumn = 0;
        size_t currentColumn = 0;
        cv::Mat temp;

        for (size_t row = 0; row < dataZ->getSize(0); ++row)
        {
            majorColumn = 0;
            currentColumn = 0;

            if (n <= m) // f(x,y) = \sum_{i=0}^n \sum_{j=0}^{m-i} p_{ij} x^i y^j -> coefficients contain p in the order of these sums
            {
                for (size_t i = 0; i <= n; ++i)
                {
                    //this handles j = 0 (for i = 0 this is already done above)
                    if (i > 0)
                    {
                        currentColumn ++;
                        for (size_t col = 0; col < dataZ->getSize(1); ++col)
                            CVMATREF_2DREAL(Vrow,col,currentColumn) = x[col] * CVMATREF_2DREAL(Vrow,col,majorColumn);
                        majorColumn = currentColumn;
                    }

                    for (size_t j = 1; j <= m-i; ++j)
                    {
                        currentColumn ++;
                        for (size_t col = 0; col < dataZ->getSize(1); ++col)
                            CVMATREF_2DREAL(Vrow,col,currentColumn) = y[row] * CVMATREF_2DREAL(Vrow,col,currentColumn-1);
                    }
                }
            }
            else // \sum_{j=0}^m \sum_{i=0}^{n-j} p_{ij} y^j x^i -> coefficients contain p in the order of these sums
            {
                for (size_t j = 0; j <= m; ++j)
                {
                    //this handles i = 0 (for j = 0 this is already done above)
                    if (j > 0)
                    {
                        currentColumn ++;
                        for (size_t col = 0; col < dataZ->getSize(1); ++col)
                            CVMATREF_2DREAL(Vrow,col,currentColumn) = y[row] * CVMATREF_2DREAL(Vrow,col,majorColumn);
                        majorColumn = currentColumn;
                    }

                    for (size_t i = 1; i <= n-j; ++i)
                    {
                        currentColumn ++;
                        for (size_t col = 0; col < dataZ->getSize(1); ++col)
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

    return retval;
}