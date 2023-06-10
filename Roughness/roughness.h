/* ********************************************************************
Plugin "Roughness" for itom software
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

#ifndef ROUGHNESS_H
#define ROUGHNESS_H

#include "common/addInInterface.h"

namespace cv
{
    template<typename _Tp> class Mat_;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** @class RoughnessInterface
*/
class RoughnessInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        RoughnessInterface();
        ~RoughnessInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class Roughness
*   @brief short description
*
*   longer description
*/
namespace cv
{
    class Mat; //forward declaration
}

class Roughness : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        Roughness() {}
        virtual ~Roughness() {}

    public:
        friend class RoughnessInterface;

        static const QString getGaussianFilterKernelDoc;
        static ito::RetVal getGaussianFilterKernel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal getGaussianFilterKernelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString calcRoughnessProfileDoc;
        static ito::RetVal calcRoughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal calcRoughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString evalRoughnessProfileDoc;
        static ito::RetVal evalRoughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal evalRoughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString roughnessProfileDoc;
        static ito::RetVal roughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal roughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString calcAbbottCurveDoc;
        static ito::RetVal calcAbbottCurve(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal calcAbbottCurveParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        struct RoughnessParams
        {
            enum SamplingLengthDivision
            {
                SplitInto5EqualSamples = 0, //split the measurement length (less the endeffects) into 5 samples. Warns if the sampling length of each sample varies from the cut-off-wavelength (tol. 5%)
                SplitIntoNSamplesWithCutOffWavelength = 1, //same as Use5SamplesWithCutOffWavelength but creates N samples and raises an error if N < 1.
                Use5SamplesWithCutOffWavelength = 2 //split the measurement length (less the endeffects) into 5 samples with the length of the cut-off-wavelength. If the measurement length is longer, the end is omitted. If it is too short, an error is raisen.
            };
            ito::float64 cutoff_wavelength; //in \mum
            ito::float64 spacing;
            SamplingLengthDivision sampling_length_mode;
            int height_discriminiation; //e.g. 10%
            int spacing_discriminiation; //e.g. 1%
            int RdcHigh; //0-100% as higher value for the R\deltac or W\deltac values
            int RdcLow; //0-100% as lower value for the R\deltac or W\deltac values
            bool RskRkuOld; //if true, Rsk and Rku will be determined using the old norm ISO 4287 (see ptb.de/rptb -> Help), else Rsk and Rku will be determined using the new version of ISO 4287 (default)
        };

    private:
        Q_DISABLE_COPY( Roughness )

        static ito::RetVal gauss_conv_non_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_kernel);
        static ito::RetVal gauss_dft_non_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_dft_kernel, int length_optimized = -1);
        static ito::RetVal gauss_conv_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_kernel);
        static std::vector<ito::float64> gen_gauss_convolution(const ito::float64 spacing, const ito::float64 cutoff, const ito::float64 cutoff_factor, int &nr_of_endeffect_pixels);
        static std::vector<ito::float64> gen_dft_gauss_convolution(const int length, const ito::float64 spacing, const ito::float64 cutoff, const ito::float64 cutoff_factor, int &nr_of_endeffect_pixels);

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ROUGHNESS_H
