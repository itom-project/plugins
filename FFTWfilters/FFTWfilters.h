/* ************************************************************************
    This file is part of fftw-plugin for ito's itom measurement software

    The fftw-plugin for itom is a wrapper for the FFTW package.
    The FFTW package was developed at MIT by Matteo Frigo and Steven G.
    Johnson. It was published under GNU General Public License and
    can be downloaded under http://www.fftw.org/.

    The fftw-plugin is a free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation, either version 3 of
    the License, or (at your option) any later version.

    the fftw-plugin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar. If not, see <http://www.gnu.org/licenses/>.
************************************************************************ */

/*! \file FFTWfilters.h
   \brief   This is the main file for the M++Filter library, which contains the interface definition.

   The algorithms in this dll are mostly copied from the filter.h and filter.cpp. The filters are grouped in different sub .cpp-files.

   \author ITO
   \date 02.2016
*/

#ifndef FFTWFILTERS_H
#define FFTWFILTERS_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include <qsharedpointer.h>

#ifdef USEOPENMP
#define USEOMP 1
#else
#define USEOMP 0
#endif

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FFTWFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the FFTWFilters class s. also \ref FFTWFilters
*/
class FFTWFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        FFTWFiltersInterface();
        ~FFTWFiltersInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

/*!
\class DataObjectAxisIterator
\brief This iterator is used for iterations along one axes (besides the last two axes)

nextPlaneTreeIndex always gives the plane number of dataObject::get_mdata() (relative to seekMat(0))
that is used for the next line along the given axis.

getPlaneStepIn/Out returns the number of planes one has to walk in order to get from one value in a line segment
to the next one.
*/
class DataObjectAxisIterator
{
public:
    DataObjectAxisIterator(const ito::DataObject *objIn, ito::DataObject *objOut, int axis); //only for dim > 2 and axis < dim - 1
    ~DataObjectAxisIterator();

    bool nextPlaneTreeIndex(int &planeIndexIn, int &planeIndexOut);
    int getPlaneStepIn() const;
    int getPlaneStepOut() const;

private:
    bool nextPlaneTreeIndex(char idx, int &planeIndex);

    const ito::DataObject *m_obj[2];
    int m_currentPlaneTreeIndex[2];
    int m_axis;
    int *m_sizes;
    int *m_currentSizes[2];
    int *m_steps[2];
    int m_controlIndex[2];
    int m_len;
    int m_firstPlaneIndex[2];
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FFTWFilters
*   @brief Algorithms used to process images and dataobjects with filters developed at ITO
*
*   In this class the algorithms used for the processing of measurement data are implemented.
*
*/
class FFTWFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        FFTWFilters();
        ~FFTWFilters();

    public:
        friend class FFTWFiltersInterface;

        static const QString fftw1dDOC;
        static const QString ifftw1dDOC;
        static const QString fftw2dDOC;
        static const QString ifftw2dDOC;
        static const QString fftshiftDOC;
        static const QString ifftshiftDOC;

        static ito::RetVal xfftw1dParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                   /**< Get the standard IO-Parameters for fftw filter */
        static ito::RetVal fftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */
        static ito::RetVal ifftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */

        static ito::RetVal xfftw2dParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal fftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */
        static ito::RetVal ifftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */

        static ito::RetVal xfftshiftParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal fftshift(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< do fftshift i.e. move zero order to center */
        static ito::RetVal ifftshift(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< do ifftshift i.e. move zero order to corner */

    private:
        typedef void(*tGetComplexLine)(const cv::Mat **mdata, int planeStep, int n, int row, int col, int rowStep, void* linedata);
        static tGetComplexLine fListGetComplexLineToComplex64[];
        static tGetComplexLine fListGetComplexLineToComplex128[];

        static ito::RetVal doFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward);
        template<typename _TpIn, typename _TpOut> static void getComplexLine(const cv::Mat **mdata, int planeStep, int n, int row, int col, int rowStep, void* linedata);
        template<typename _Tp> static void setComplexLine(cv::Mat **mdata, int planeStep, int n, int row, int col, int rowStep, const _Tp *linedata);

        static ito::RetVal doFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward);              /**< Calculate DFT by means of FFTW */

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // FFTWFilters_H
