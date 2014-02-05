/*************************************************************************
    This file is part of fftw-plugin for ito's itom measurement software

    The fftw-plugin for itom is a wrapper for the FFTW package.
    The FFTW package was developed at MIT by Matteo Frigo and Steven G. 
    Johnson. It was published unter GNU General Public License and 
    can be downloaded unter http://www.fftw.org/.

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
*************************************************************************/

/*! \file FFTWfilters.h
   \brief   This is the main file for the M++Filter libary, which contains the interface definition. 
   
   The algorithms in this dll are mostly copied from the filter.h and filter.cpp. The filters are grouped in different sub .cpp-files.

   \author ITO 
   \date 12.2011
   */


#ifndef FFTWFILTERS_H
#define FFTWFILTERS_H

#include "common/addInInterface.h"

#include "DataObject/dataobj.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FFTWFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the FFTWFilters class s. also \ref FFTWFilters
*/
class FFTWFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

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

//----------------------------------------------------------------------------------------------------------------------------------
/** @class FFTWFilters
*   @brief Algorithms used to process images and dataobjects with filters develped at ITO
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

        static ito::RetVal ParamsFFTW(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);                                    /**< Get the standard IO-Parameters for fftw filter */
        static ito::RetVal doFFTW(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward, const bool lineWise);              /**< Calculate DFT by means of FFTW */
        
        static const char * fftw1dDOC;
        static const char * ifftw1dDOC;
        static const char * fftw2dDOC;
        static const char * ifftw2dDOC;
        static ito::RetVal fftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */
        static ito::RetVal ifftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */
        static ito::RetVal fftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */
        static ito::RetVal ifftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);              /**< Calculate DFT by means of FFTW */

    private:

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // FFTWFilters_H
