/* ********************************************************************
Plugin "Roughness" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut für Technische Optik (ITO),
Universität Stuttgart, Germany

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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "roughness.h"


#include "pluginVersion.h"
#include "gitVersion.h"
#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include "common/numeric.h"

#include <QtCore/QtPlugin>
#include <qnumeric.h>
#include <qstring.h>
#include <numeric>

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal RoughnessInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Roughness)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal RoughnessInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(Roughness)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
RoughnessInterface::RoughnessInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("Roughness");

    m_description = QObject::tr("Algorithms for roughness evaluation");
    m_detaildescription = QObject::tr("This plugin contains algorithms for the 1D roughness evaluation. \n\
\n\
The contained algorithms are: \n\
\n\
* calcRoughnessProfile: divide the given profile row-by-row into roughness and waviness \n\
* evalRoughnessProfile: evaluate either the roughness or waviness based on various 1D roughness parameters (e.g. Rz, Ra...) \n\
* roughnessProfile: combination of the two algorithms above as well as a possible initial subtraction of a regression line \n\
* calcAbbottCurve: determination of the abbott curve based on the roughness or waviness \n\
\n\
Some algorithms the plugin 'fittingFilters' for a valid execution.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
RoughnessInterface::~RoughnessInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#define MAKELIST(FuncName, TypeDefName) static TypeDefName f##FuncName[] =   \
{                                                                       \
   FuncName<ito::int8>,                                                 \
   NULL,                                                                \
   FuncName<ito::int16>,                                                \
   NULL,                                                                \
   FuncName<ito::int32>,                                                \
   NULL,                                                                \
   FuncName<ito::float32>,                                              \
   FuncName<ito::float64>,                                              \
   NULL,                                                                \
   NULL,                                                                \
   NULL,                                                                \
};

//----------------------------------------------------------------------------------------------------------------------------------
typedef ito::RetVal (*tSurfaceParamBasedOnSamplingLength)(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples);
typedef ito::RetVal (*tSurfaceParamBasedOnMeasurementLength)(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z);
typedef ito::RetVal (*tAbbottCurveBasedOnMeasurementLength)(const uchar* data, const size_t byte_steps, const int length, ito::float64 *abbott_buffer, ito::DataObject *histogram);

//----------------------------------------------------------------------------------------------------------------------------------
//divides the given measurement length into sampling length according to the sampling_length_mode.
/*
* @param length [in] is the measurement length in \mu m
* @param params [in] are the general parameters for the current roughness determination
* @param num_samples [out] are the final number of samples
* @param sample_length [out] is the number of pixels in each sample
*/
ito::RetVal checkSamplingLength(const int length, const Roughness::RoughnessParams &params, int &num_samples, int &sample_length)
{
    ito::RetVal retval;
    double len_mu = length * params.spacing;

    if (params.sampling_length_mode == Roughness::RoughnessParams::SplitInto5EqualSamples)
    {
        double len_sample_mu = len_mu / 5.0;
        sample_length = std::floor(len_sample_mu / params.spacing);
        num_samples = 5;
        if (std::abs(len_sample_mu - params.cutoff_wavelength) > (0.05 * params.cutoff_wavelength))
        {
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("the chosen sampling length for five sections differs from the intended length (cut-off wavelength)").toLatin1().data());
        }
    }
    else if (params.sampling_length_mode == Roughness::RoughnessParams::SplitIntoNSamplesWithCutOffWavelength)
    {
        num_samples = std::floor(len_mu / params.cutoff_wavelength);
        sample_length = std::floor(0.5 + params.cutoff_wavelength / params.spacing);
        if ((sample_length * num_samples) > length)
        {
            sample_length--;
        }
    }
    else if (params.sampling_length_mode == Roughness::RoughnessParams::Use5SamplesWithCutOffWavelength)
    {
        num_samples = 5;
        sample_length = std::floor(0.5 + params.cutoff_wavelength / params.spacing);
        if ((sample_length * num_samples) > length)
        {
            sample_length--;
        }
        if (sample_length * num_samples > length)
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("measurement length is too short to be split into 5 samples with a length of the cut-off wavelength each").toLatin1().data());
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("wrong sampling_length_mode").toLatin1().data());
    }

    if (num_samples < 1)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("measurement length is too short").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rp and Wp for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZp(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = 0.0;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            ito::float64 current = 0.0;
            for (int i = 0; i < sample_length; ++i)
            {
                if (*values > current && ito::isFinite(*values))
                {
                    current = *values;
                }
                values += steps;
            }

            if (current > 0.0)
            {
                valids = true;
            }

            Z_mean += current;
            Z_min = std::min(Z_min, current);
            Z_max = std::max(Z_max, current);
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZp, tSurfaceParamBasedOnSamplingLength); //Rp, Wp

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rv and Wv for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZv(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = 0.0;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            ito::float64 current = 0.0;
            for (int i = 0; i < sample_length; ++i)
            {
                if (*values < current && ito::isFinite(*values))
                {
                    current = *values;
                }
                values += steps;
            }

            if (current < 0.0)
            {
                valids = true;
            }

            Z_mean += current;
            Z_min = std::min(Z_min, -current);
            Z_max = std::max(Z_max, -current);
        }

        Z_mean /= (ito::float64)num_samples;
        Z_mean = -Z_mean;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZv, tSurfaceParamBasedOnSamplingLength); //Rv, Wv

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rz and Wz for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZz(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = 0.0;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            ito::float64 current_lowest = 0.0;
            ito::float64 current_highest = 0.0;
            for (int i = 0; i < sample_length; ++i)
            {
                if (ito::isFinite(*values))
                {
                    if (*values > current_highest)
                    {
                        current_highest = *values;
                    }
                    if ((-*values) > current_lowest)
                    {
                        current_lowest = -*values;
                    }

                    valids = true;
                }

                values += steps;
            }

            Z_mean += (current_highest + current_lowest);
            Z_min = std::min(Z_min, current_highest + current_lowest);
            Z_max = std::max(Z_max, current_highest + current_lowest);
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZz, tSurfaceParamBasedOnSamplingLength); //Rz, Wz

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rt and Wt for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z [out] returns the result over the whole measurement length
*/
template<typename _Tp> ito::RetVal CalcZt(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z)
{
    ito::RetVal retval;
    const _Tp* values = (const _Tp*)data;
    size_t steps = byte_steps / sizeof(_Tp);
    ito::float64 current_lowest = 0.0;
    ito::float64 current_highest = 0.0;
    bool valids = false;

    for (int i = 0; i < length; ++i)
    {
        if (ito::isFinite(*values))
        {
            if (*values > current_highest)
            {
                current_highest = *values;
            }
            if ((-*values) > current_lowest)
            {
                current_lowest = -*values;
            }
            valids = true;
        }
        values += steps;
    }

    Z = valids ? current_lowest + current_highest : std::numeric_limits<ito::float64>::quiet_NaN();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZt, tSurfaceParamBasedOnMeasurementLength); //Rt, Wt, Pt

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Ra and Wa for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZa(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = 0.0;
        ito::float64 current;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            current = 0.0;
            ito::float64 current_highest = 0.0;
            for (int i = 0; i < sample_length; ++i)
            {
                if (ito::isFinite(*values))
                {
                    current += std::abs(*values);
                    valids = true;
                }
                values += steps;
            }

            current /= sample_length;
            Z_mean += current;
            Z_min = std::min(Z_min, current);
            Z_max = std::max(Z_max, current);
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZa, tSurfaceParamBasedOnSamplingLength); //Ra, Wa

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rq and Wq for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZq(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = 0.0;
        ito::float64 current;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            current = 0.0;
            for (int i = 0; i < sample_length; ++i)
            {
                if (ito::isFinite(*values))
                {
                    current += (*values * *values);
                    valids = true;
                }
                values += steps;
            }

            current = std::sqrt(current / sample_length);
            Z_mean += current;
            Z_min = std::min(Z_min, current);
            Z_max = std::max(Z_max, current);
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZq, tSurfaceParamBasedOnSamplingLength); //Rq, Wq

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rsk and Wsk for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZsk(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = -std::numeric_limits<ito::float64>::max();
        ito::float64 current = 0.0;
        bool valids = false;

        if (params.RskRkuOld)
        {
            //based on the old standard, every single Rsk,i for i in range(0, sample) uses Rq,i as quotient
            for (int sample = 0; sample < num_samples; ++sample)
            {
                ito::float64 temp;
                ito::float64 current_Rq = 0.0;
                ito::float64 current_Sk = 0.0;
                for (int i = 0; i < sample_length; ++i)
                {
                    if (ito::isFinite(*values))
                    {
                        temp = *values * *values;
                        current_Rq += temp;
                        current_Sk += (temp * *values);
                        valids = true;
                    }
                    values += steps;
                }

                current_Rq = std::sqrt(current_Rq / sample_length);
                if (current_Rq > 0)
                {
                    current = (current_Sk / sample_length) / (current_Rq*current_Rq*current_Rq);
                }
                else
                {
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("profile contains at least one sample length with only zero values -> Zq == 0 -> Zsk not determinable").toLatin1().data());
                }

                Z_mean += current;
                Z_min = std::min(Z_min, current);
                Z_max = std::max(Z_max, current);
            }

            Z_mean /= (ito::float64)num_samples;
        }
        else
        {
            //based on the new standard, every single Rsk,i for i in range(0, sample) uses the same overall Rq as quotient
            ito::float64 Rq_mean, Rq_min, Rq_max;
            retval += CalcZq<_Tp>(data, byte_steps, length, params, Rq_mean, Rq_min, Rq_max, num_samples);
            if (!retval.containsError()) {
                Rq_mean = (Rq_mean * Rq_mean * Rq_mean);
                if (Rq_mean <= 0) {
                    retval += ito::RetVal(ito::retError, 0,
                        QObject::tr("profile contains at least one sample length "\
                            "with only zero values -> Zq == 0 -> Zsk not determinable").toLatin1().data());
                }
            }
            if (!retval.containsError())
            {
                for (int sample = 0; sample < num_samples; ++sample)
                {
                    ito::float64 current_Sk = 0.0;
                    for (int i = 0; i < sample_length; ++i)
                    {
                        if (ito::isFinite(*values))
                        {
                            current_Sk += (*values * *values * *values);
                            valids = true;
                        }
                        values += steps;
                    }

                    current = (current_Sk / sample_length) / (Rq_mean);

                    Z_mean += current;
                    Z_min = std::min(Z_min, current);
                    Z_max = std::max(Z_max, current);
                }

                Z_mean /= (ito::float64)num_samples;
            }
        }

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }



    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZsk, tSurfaceParamBasedOnSamplingLength); //Rsk, Wsk

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rku and Wku for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZku(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = -std::numeric_limits<ito::float64>::max();
        ito::float64 current = 0.0;
        bool valids = false;

        if (params.RskRkuOld)
        {
            //based on the old standard, every single Rsk,i for i in range(0, sample) uses Rq,i as quotient
            for (int sample = 0; sample < num_samples; ++sample)
            {
                ito::float64 temp;
                ito::float64 current_Rq = 0.0;
                ito::float64 current_Sk = 0.0;
                for (int i = 0; i < sample_length; ++i)
                {
                    if (ito::isFinite(*values))
                    {
                        temp = *values * *values;
                        current_Rq += temp;
                        current_Sk += (temp * temp);
                        valids = true;
                    }
                    values += steps;
                }

                current_Rq = current_Rq / sample_length;
                if (current_Rq > 0)
                {
                    current = (current_Sk / sample_length) / (current_Rq * current_Rq);
                }
                else
                {
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("profile contains at least one sample length with only zero values -> Zq == 0 -> Zku not determinable").toLatin1().data());
                }

                Z_mean += current;
                Z_min = std::min(Z_min, current);
                Z_max = std::max(Z_max, current);
            }

            Z_mean /= (ito::float64)num_samples;
        }
        else
        {
            //based on the new standard, every single Rsk,i for i in range(0, sample) uses the same overall Rq as quotient
            ito::float64 Rq_mean, Rq_min, Rq_max;
            retval += CalcZq<_Tp>(data, byte_steps, length, params, Rq_mean, Rq_min, Rq_max, num_samples);

            if (!retval.containsError())
            {
                Rq_mean *= Rq_mean;
                Rq_mean *= Rq_mean;
                if (Rq_mean <= 0) {
                    retval += ito::RetVal(ito::retError, 0,
                        QObject::tr("profile contains at least one sample "\
                            "length with only zero values -> Zq == 0 -> Zku not determinable").toLatin1().data());
                }
            }
            if (!retval.containsError())
            {
                for (int sample = 0; sample < num_samples; ++sample)
                {
                    ito::float64 temp;
                    ito::float64 current_Sk = 0.0;
                    for (int i = 0; i < sample_length; ++i)
                    {
                        if (ito::isFinite(*values))
                        {
                            temp = *values * *values;
                            current_Sk += (temp * temp);
                            valids = true;
                        }
                        values += steps;
                    }

                    current = (current_Sk / sample_length) / (Rq_mean);

                    Z_mean += current;
                    Z_min = std::min(Z_min, current);
                    Z_max = std::max(Z_max, current);
                }

                Z_mean /= (ito::float64)num_samples;
            }
        }

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZku, tSurfaceParamBasedOnSamplingLength); //Rku, Wku

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rdq and Wdq for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZdq(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (sample_length < 7)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Rdq or Wdq require a sample length of at least 7 samples.").toLatin1().data());
    }

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = -std::numeric_limits<ito::float64>::max();
        ito::float64 current = 0.0;
        unsigned int bufferIdx = 0;
        ito::float64 temp;
        int m3 = -3*(int)steps;
        int m2 = -2*(int)steps;
        int m1 = -(int)steps;
        int p1 = steps;
        int p2 = 2*steps;
        int p3 = 3*steps;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            unsigned int counts = 0;
            current = 0.0;
            values += 3*steps; //move values 3 steps forward

            for (int i = 3; i < sample_length - 3; ++i)
            {
                temp = 1.0/(60.0 * params.spacing) * (values[p3] - 9*values[p2] + 45*values[p1] - 45*values[m1] + 9*values[m2] - values[m3]);
                if (ito::isFinite(temp))
                {
                    current += (temp * temp);
                    counts++;
                    valids = true;
                }

                values += steps;
            }

            if (counts > 0)
            {
                current = std::sqrt(current / counts);
                Z_mean += current;
                Z_min = std::min(Z_min, current);
                Z_max = std::max(Z_max, current);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("at least one sample does not contain at least 7 adjacent valid data points").toLatin1().data());
            }
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZdq, tSurfaceParamBasedOnSamplingLength); //Rdq, Wdq

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rda and Wda for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07,
    the division into the sampling length is given by DIN EN ISO 4288:1997

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z_mean [out] returns the mean value of the result over all sampling lengths
    @param Z_min [out] returns the minimum value of the result over all sampling lengths
    @param Z_max [out] returns the maximum value of the result over all sampling lengths
    @param num_samples [out] returns the number of samples (usually 5, the strategy depends on params)
*/
template<typename _Tp> ito::RetVal CalcZda(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z_mean, ito::float64 &Z_min, ito::float64 &Z_max, int &num_samples)
{
    int sample_length;
    ito::RetVal retval = checkSamplingLength(length, params, num_samples, sample_length);

    if (sample_length < 7)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Rda or Wda require a sample length of at least 7 samples.").toLatin1().data());
    }

    if (!retval.containsError())
    {
        const _Tp* values = (const _Tp*)data;
        size_t steps = byte_steps / sizeof(_Tp);
        Z_mean = 0.0;
        Z_min = std::numeric_limits<ito::float64>::max();
        Z_max = -std::numeric_limits<ito::float64>::max();
        ito::float64 current = 0.0;
        ito::float64 temp;
        int m3 = -3*(int)steps;
        int m2 = -2*(int)steps;
        int m1 = -(int)steps;
        int p1 = steps;
        int p2 = 2*steps;
        int p3 = 3*steps;
        bool valids = false;

        for (int sample = 0; sample < num_samples; ++sample)
        {
            unsigned int counts = 0;
            current = 0.0;
            values += 3*steps; //move values 3 steps forward

            for (int i = 3; i < sample_length - 3; ++i)
            {
                temp = 1.0/(60.0 * params.spacing) * (values[p3] - 9*values[p2] + 45*values[p1] - 45*values[m1] + 9*values[m2] - values[m3]);
                if (ito::isFinite(temp))
                {
                    current += std::abs(temp);
                    counts++;
                    valids = true;
                }

                values += steps;
            }

            if (counts > 0)
            {
                current = current / counts;
                Z_mean += current;
                Z_min = std::min(Z_min, current);
                Z_max = std::max(Z_max, current);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("at least one sample does not contain at least 7 adjacent valid data points").toLatin1().data());
            }
        }

        Z_mean /= (ito::float64)num_samples;

        if (!valids)
        {
            Z_mean = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_min = std::numeric_limits<ito::float64>::quiet_NaN();
            Z_max = std::numeric_limits<ito::float64>::quiet_NaN();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZda, tSurfaceParamBasedOnSamplingLength); //Rda, Wda

//----------------------------------------------------------------------------------------------------------------------------------
//Algorithm to calculate Rdc and Wdc for one single line of measurement data.
/*
    This algorithm is defined by DIN EN ISO 4287:2010-07

    Rdc and Wdc are evaluated on the abbott-curve based on the parameters RdcHigh or RdcLow

    @param data [in] is a pointer to the first value in the data set (will be cast to _Tp*)
    @param byte_steps [in] is the number of bytes between one value and the next value
    @param length [in] are the number of samples
    @param params [in] define the boundary conditions for the roughness determination
    @param Z [out] returns the result over the whole measurement length
*/
template<typename _Tp> ito::RetVal CalcZdc(const uchar* data, const size_t byte_steps, const int length, const Roughness::RoughnessParams &params, ito::float64 &Z)
{
    ito::RetVal retval;
    const _Tp* values = (const _Tp*)data;
    size_t steps = byte_steps / sizeof(_Tp);
    std::vector<_Tp> vec;
    int count = 0;
    bool valids = false;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        vec.resize(length);
        memcpy(vec.data(), data, length * sizeof(_Tp));
        count = length;
        valids = true;
    }
    else
    {
        vec.resize(length);

        for (int i = 0; i < length; ++i)
        {
            if (ito::isFinite(*values))
            {
                vec[count] = *values;
                count++;
                valids = true;
            }
            values += steps;
        }
        vec.resize(count);
    }

    int idxLow = qBound(0, (count * params.RdcLow) / 100, count - 1);
    int idxHigh = qBound(0, (count * params.RdcHigh) / 100, count - 1);
    std::nth_element(vec.begin(), vec.begin() + idxLow, vec.end());
    std::nth_element(vec.begin(), vec.begin() + idxHigh, vec.end());

    Z = vec[idxHigh] - vec[idxLow];

    if (!valids)
    {
        Z = std::numeric_limits<ito::float64>::quiet_NaN();
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcZdc, tSurfaceParamBasedOnMeasurementLength); //Rt, Wt, Pt

//----------------------------------------------------------------------------------------------------------------------------------
/*abbott_buffer must be initialized with 1001 elements!!
histogram_buffer can be NULL or must be a 1x1000 int32 data object.*/
template<typename _Tp> bool descsort(_Tp i, _Tp j) { return (j<i); }
template<typename _Tp> ito::RetVal CalcAbbott(const uchar* data, const size_t byte_steps, const int length, ito::float64 *abbott_buffer, ito::DataObject *histogram)
{
    ito::RetVal retval;
    const _Tp* values = (const _Tp*)data;
    size_t steps = byte_steps / sizeof(_Tp);
    std::vector<_Tp> vec;
    int count = 0;

    if (std::numeric_limits<_Tp>::is_exact)
    {
        vec.resize(length);
        memcpy(vec.data(), data, length * sizeof(_Tp));
        count = length;
    }
    else
    {
        vec.resize(length);

        for (int i = 0; i < length; ++i)
        {
            if (ito::isFinite(*values))
            {
                vec[count] = *values;
                count++;
            }
            values += steps;
        }
        vec.resize(count);
    }

    std::sort(vec.begin(), vec.end(), descsort<_Tp>);
    double abbott_step = (double)count / 1001.0;

    if (abbott_buffer)
    {
        memset(abbott_buffer, 0, 1001*sizeof(ito::float64));
        if (count > 0)
        {
            abbott_buffer[0] = vec[0];
            abbott_buffer[1000] = vec[count-1];
            int idx1, idx2;
            double dist;
            for (int i = 1; i < 1000; ++i)
            {
                idx1 = qBound(0, (int)std::floor(i * abbott_step), count-1);
                idx2 = qBound(0, (int)std::ceil(i * abbott_step), count-1);
                dist = (i * abbott_step - (double)idx1) / abbott_step;
                abbott_buffer[i] = vec[idx1] * (1-dist) + vec[idx2] * dist; //interpolation
            }
        }
    }

    if (histogram)
    {
        ito::int32* histogram_buffer = (ito::int32*)histogram->rowPtr(0,0);
        memset(histogram_buffer, 0, 1000*sizeof(ito::int32));
        if (count > 0)
        {
            _Tp maximum = vec[0];
            _Tp minimum = vec[count-1];
            double bucket_step = (double)(maximum - minimum) / 1000.0;
            int idx;

            for (size_t i = 0; i < vec.size(); ++i)
            {
                idx = std::min((int)std::floor((vec[i] - minimum) / bucket_step), 999);
                histogram_buffer[idx]++;
            }

            histogram->setAxisScale(1, bucket_step);
            histogram->setAxisOffset(1, -minimum / bucket_step);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MAKELIST(CalcAbbott, tAbbottCurveBasedOnMeasurementLength);

//---------------------------------------------------------------------------------------------------------------------------------------------
const QString Roughness::getGaussianFilterKernelDoc = QObject::tr("returns the gaussian filter kernel as used by calcRoughnessProfile, based on EN ISO 11562");

//---------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::getGaussianFilterKernelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("kernel", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("kernel data object that contains the kernel after the call.").toLatin1().data()));
    paramsMand->append(ito::Param("cutoff_wavelength", ito::ParamBase::Double | ito::ParamBase::In, 0.0001, 100000.0, 80.0, QObject::tr("cut-off wavelength in _m").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsMand->append(ito::Param("spacing", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 1.0, QObject::tr("spacing in _m between two adjacent pixels").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsOpt->append(ito::Param("cutoff_factor", ito::ParamBase::Double | ito::ParamBase::In, 0.5, 1.0, 0.6, QObject::tr("quality factor for the length of the filter kernel. The kernel is set to zero for indices > cutoff-wavelength * cutoff_factor. See ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::getGaussianFilterKernel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    double cutoff_wavelength = paramsMand->at(1).getVal<double>();
    double spacing = paramsMand->at(2).getVal<double>();
    double cutoff_factor = paramsOpt->at(0).getVal<double>();
    int nr_of_endeffect_pixels;

    std::vector<ito::float64> values = gen_gauss_convolution(spacing, cutoff_wavelength, cutoff_factor, nr_of_endeffect_pixels);
    if (values.size() == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("kernel has a size of 0. Invalid.").toLatin1().data());
    }

    size_t val_size = values.size();

    ito::DataObject kernel(1, val_size*2-1, ito::tFloat64);
    kernel.setAxisUnit(1, "µm");
    kernel.setAxisScale(1, spacing);
    kernel.setAxisOffset(1, values.size());

    ito::float64* val = (ito::float64*)kernel.rowPtr(0,0);

    for (size_t i = 0; i < val_size; ++i)
    {
        val[i] = values[val_size - i - 1];
    }

    for (size_t i = 1; i < val_size; ++i)
    {
        val[val_size + i - 1] = values[i];
    }

    *((*paramsMand)[0].getVal<ito::DataObject*>()) = kernel;

    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
const QString Roughness::calcRoughnessProfileDoc = QObject::tr("calculate the roughness and waviness profile based in a given real input data object. \n\
\n\
The roughness and waviness profile is determined row-by-row from the given input object that is filtered using \n\
one or two gaussian convolution operations. The gaussian filters are chosen such that a transmission of 50% is \n\
obtained at the given cut-off wavelength levels. \n\
\n\
The waviness is a lowpass of the input data cut at the cut-off wavelength Lc. \n\
The roughness is the obtained by the difference between input and waviness. Additionally a lowpass is applied \n\
at the cut-off wavelength Ls (if Ls > 0.0). \n\
\n\
You need to define if your profile is an open, non-periodic profiel or a closed, periodic profile. In the latter case\n\
the evaluation is only implemented by a convolution based algorithm, in the first case one can choose between the fast \n\
dft-implementation or the convolution-based implementation. If 'mode' is set to 'auto', lowpass filter operations with \n\
big wavelengths are evaluated using the dft-approach (Lc) while the high-frequency cut-off Ls is done using the convolution operation. \n\
\n\
In order to guarantee an efficient algorithm, the gaussian filter kernel will be cut after a certain length. This length can be \n\
controlled using the parameter 'cutoff_factor'. \n\
\n\
This filter is implemented based on DIN EN ISO 16610-21:2013.");

//---------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::calcRoughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("input", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, QObject::tr("real input object (1D or 2D). The roughness is determined row-by-row. The axis units must be set to 'mm', '_m' or 'nm'.").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsMand->append(ito::Param("roughness", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("roughness output object of the same size than the input object. Type is float64.").toLatin1().data()));
    paramsMand->append(ito::Param("waviness", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("waviness output object of the same size than the input object. Type is float64.").toLatin1().data()));
    paramsMand->append(ito::Param("Lc", ito::ParamBase::Double | ito::ParamBase::In, 10.0, 100000.0, 80.0, QObject::tr("cut-off wavelength in _m for the separation between the waviness and roughness").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsMand->append(ito::Param("Ls", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 25.0, 2.5, QObject::tr("short cut-off wavelength in _m for the separation between roughness and further high-frequency components (set 0.0 to omit this filtering)").replace("_", QLatin1String("\u00B5")).toLatin1().data()));

    param = ito::Param("mode", ito::ParamBase::String | ito::ParamBase::In, "auto", QObject::tr("mode of determination: convolution, dft or auto (auto-selection of both methods)").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "auto");
    sm->addItem("dft");
    sm->addItem("convolution");
    param.setMeta(sm, true);
    paramsOpt->append(param);

    paramsOpt->append(ito::Param("periodicity", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, QObject::tr("0: non periodic profile (default), 1: periodic profile (not implemented yet)").toLatin1().data()));
    paramsOpt->append(ito::Param("cutoff_factor", ito::ParamBase::Double | ito::ParamBase::In, 0.5, 1.0, 0.6, QObject::tr("quality factor for the convolution based calculation. The convolution kernel is set to zero for indices > cutoff-wavelength * cutoff_factor. See ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0").toLatin1().data()));

    paramsOut->append(ito::Param("nr_of_endeffect_pixels", ito::ParamBase::Int | ito::ParamBase::Out, 0, std::numeric_limits<int>::max(), 0, QObject::tr("nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::calcRoughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int nr_of_endeffect_pixels_lc = 0;
    int nr_of_endeffect_pixels_ls = 0;

    ito::float64 lc = paramsMand->at(3).getVal<double>();
    ito::float64 ls = paramsMand->at(4).getVal<double>();
    QString mode = paramsOpt->at(0).getVal<char*>();
    bool periodicity = paramsOpt->at(1).getVal<int>() > 0 ? true : false;
    ito::float64 cutoff_factor = paramsOpt->at(2).getVal<double>();

    ito::DataObject input = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "input", ito::Range::all(), ito::Range::all(), retval, ito::tFloat64, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (input.getTotal() < 1)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("empty input object given").toLatin1().data());
    }
    else
    {
        bool valid;
        std::string unit = input.getAxisUnit(1, valid);
        ito::float64 spacing = input.getAxisScale(1);

        if (!valid)
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("axis unit could not be read").toLatin1().data());
        }
        else if (unit == "mm")
        {
            spacing *= 1000.0; //convert to \mu m
        }
        else if (unit == QLatin1String("\u00B5m").latin1())
        {
        }
        else if (unit == "nm")
        {
            spacing /= 1000.0; //convert to \mu m
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("axis unit of input must be mm, _m or nm").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        }

        if (!retval.containsError())
        {
            ito::DataObject roughness(input.getSize(0), input.getSize(1), ito::tFloat64);
            ito::DataObject waviness(input.getSize(0), input.getSize(1), ito::tFloat64);
            for (int i = 0; i < 2; i++)
            {
                roughness.setAxisDescription(i, input.getAxisDescription(i, valid));
                roughness.setAxisUnit(i, input.getAxisUnit(i, valid));
                roughness.setAxisOffset(i, input.getAxisOffset(i));
                roughness.setAxisScale(i, input.getAxisScale(i));
                waviness.setAxisDescription(i, input.getAxisDescription(i, valid));
                waviness.setAxisUnit(i, input.getAxisUnit(i, valid));
                waviness.setAxisOffset(i, input.getAxisOffset(i));
                waviness.setAxisScale(i, input.getAxisScale(i));
            }

            roughness.setValueUnit(input.getValueUnit());
            roughness.setValueDescription("roughness");
            waviness.setValueUnit(input.getValueUnit());
            waviness.setValueDescription("waviness");

            const cv::Mat_<ito::float64>* input_plane = (const cv::Mat_<ito::float64>*)(input.getCvPlaneMat(0));
            cv::Mat_<ito::float64>* waviness_plane = (cv::Mat_<ito::float64>*)(waviness.getCvPlaneMat(0));
            cv::Mat_<ito::float64>* roughness_plane = (cv::Mat_<ito::float64>*)(roughness.getCvPlaneMat(0));

            if (mode == "convolution" || (mode == "auto" && periodicity))
            {
                if (periodicity) //closed profiles
                {
                    std::vector<ito::float64> kernel = gen_gauss_convolution(spacing, lc, cutoff_factor, nr_of_endeffect_pixels_lc);
                    retval += gauss_conv_periodic_rowwise(input_plane, waviness_plane, kernel);

                    //filter out high-frequency components from roughness
                    if (ls > std::numeric_limits<ito::float64>::epsilon())
                    {
                        ito::DataObject roughness_temp = input - waviness;
                        const cv::Mat_<ito::float64> *roughness_temp_plane = (const cv::Mat_<ito::float64>*)(roughness_temp.getCvPlaneMat(0));
                        kernel = gen_gauss_convolution(spacing, ls, cutoff_factor, nr_of_endeffect_pixels_ls);
                        retval += gauss_conv_periodic_rowwise(roughness_temp_plane, roughness_plane, kernel);
                    }
                    else
                    {
                        roughness = input - waviness;
                    }

                    nr_of_endeffect_pixels_ls = 0;
                    nr_of_endeffect_pixels_lc = 0;
                    *((*paramsMand)[1].getVal<ito::DataObject*>()) = roughness;
                    *((*paramsMand)[2].getVal<ito::DataObject*>()) = waviness;
                }
                else //open profiles (non periodicity)
                {

                    std::vector<ito::float64> kernel = gen_gauss_convolution(spacing, lc, cutoff_factor, nr_of_endeffect_pixels_lc);
                    retval += gauss_conv_non_periodic_rowwise(input_plane, waviness_plane, kernel);

                    //filter out high-frequency components from roughness
                    if (ls > std::numeric_limits<ito::float64>::epsilon())
                    {
                        ito::DataObject roughness_temp = input - waviness;
                        const cv::Mat_<ito::float64> *roughness_temp_plane = (const cv::Mat_<ito::float64>*)(roughness_temp.getCvPlaneMat(0));
                        kernel = gen_gauss_convolution(spacing, ls, cutoff_factor, nr_of_endeffect_pixels_ls);
                        retval += gauss_conv_non_periodic_rowwise(roughness_temp_plane, roughness_plane, kernel);
                    }
                    else
                    {
                        roughness = input - waviness;
                    }

                    *((*paramsMand)[1].getVal<ito::DataObject*>()) = roughness;
                    *((*paramsMand)[2].getVal<ito::DataObject*>()) = waviness;
                }
            }
            else if (mode == "dft" || (mode == "auto" && !periodicity))
            {
                if (periodicity) //closed profiles
                {
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("periodic, closed profiles cannot be evaluated using dft (not implemented yet)").toLatin1().data());
                }
                else //open profiles (non periodicity)
                {
                    int length = input.getSize(1); //default: row-wise
                    int m_space = std::ceil((lc * cutoff_factor) / spacing);

                    //get optimal length
                    int length_opt = cv::getOptimalDFTSize(((length + 2 * m_space) + 1) / 2) * 2;
                    if (length_opt < 0)
                    {
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("profile is too long. Frequency analysis cannot be executed").toLatin1().data());
                    }
                    else
                    {
                        std::vector<ito::float64> kernel = gen_dft_gauss_convolution(length_opt, spacing, lc, cutoff_factor, nr_of_endeffect_pixels_lc);
                        nr_of_endeffect_pixels_lc = m_space;
                        retval += gauss_dft_non_periodic_rowwise(input_plane, waviness_plane, kernel, length_opt);

                        //filter out high-frequency components from roughness
                        if (ls > std::numeric_limits<ito::float64>::epsilon())
                        {
                            ito::DataObject roughness_temp = input - waviness;
                            const cv::Mat_<ito::float64> *roughness_temp_plane = (const cv::Mat_<ito::float64>*)(roughness_temp.getCvPlaneMat(0));

                            //in auto mode use convolution for ls if filter kernel is small
                            if (mode == "auto" && std::ceil((ls * cutoff_factor)/spacing) < 25)
                            {
                                kernel = gen_gauss_convolution(spacing, ls, cutoff_factor, nr_of_endeffect_pixels_ls);
                                retval += gauss_conv_non_periodic_rowwise(roughness_temp_plane, roughness_plane, kernel);
                            }
                            else
                            {
                                m_space = std::ceil((ls * cutoff_factor) / spacing);
                                length_opt = cv::getOptimalDFTSize(((length + 2 * m_space) + 1) / 2) * 2;
                                kernel = gen_dft_gauss_convolution(length_opt, spacing, ls, cutoff_factor, nr_of_endeffect_pixels_ls);
                                nr_of_endeffect_pixels_ls = m_space;
                                retval += gauss_dft_non_periodic_rowwise(roughness_temp_plane, roughness_plane, kernel, length_opt);
                            }
                        }
                        else
                        {
                            roughness = input - waviness;
                        }

                        *((*paramsMand)[1].getVal<ito::DataObject*>()) = roughness;
                        *((*paramsMand)[2].getVal<ito::DataObject*>()) = waviness;
                    }
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("unsupported mode").toLatin1().data());
            }

            QString msg;
            msg = tr("Roughness filtered with L_c = %1 and L_s = %2, mode = %3").arg(lc).arg(ls).arg(mode);
            roughness.addToProtocol(std::string(msg.toLatin1().data()));
            roughness.setTag("lc", lc);
            roughness.setTag("ls", ls);

            msg = tr("Waviness filtered with L_c = %1 and L_s = %2, mode = %3").arg(lc).arg(ls).arg(mode);
            waviness.addToProtocol(std::string(msg.toLatin1().data()));
            waviness.setTag("lc", lc);
            waviness.setTag("ls", ls);
        }
    }

    (*paramsOut)[0].setVal<int>(nr_of_endeffect_pixels_lc + nr_of_endeffect_pixels_ls);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString Roughness::evalRoughnessProfileDoc = QObject::tr("Evaluates given roughness or waviness profiles for a specific roughness profile parameter. \n\
\n\
The given roughness data is evaluated line by line. If only one line is evaluated, 'result' contains: \n\
\n\
(mean-value-over-all-samples, min-value-over-all-samples, max-value-over-all-samples) \n\
\n\
The result values are always in _m. In case of Rt (or Wt), the evaluation is not separated to various samples, therefore mean, min and max contain the same values. \n\
\n\
If multiple lines are evaluated, the result contains the \n\
\n\
(mean-value-over-all-lines, min-value, max-value, std-dev), \n\
where min-value, max-value and std-dev are calculated over the mean-values of all lines. \n\
\n\
The evaluation is done based on DIN EN ISO 4287:2010, the separation of different sample lengths is based on DIN EN ISO 4288:1997. \n\
\n\
Possible roughness parameters are Rp, Rv, Rz, Rt, Ra, Rq, Rsk, Rku, Rdq, Rda, Rdc. If you pass the waviness profile instead of the roughness profile \n\
the parameters are then Wp, Wv, Wz, Wt, Wa, Wq, Wsk, Wku, Wdq, Wda, Wdc.").replace("_", QLatin1String("\u00B5"));

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Roughness::evalRoughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("roughness", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, QObject::tr("real input object (1D or 2D, no unsigned data types) - must be either the output argument 'roughness' or 'waviness' from filter 'calcRoughnessProfile'. The roughness is determined row-by-row. The axis units must be set to 'mm', '_m' or 'nm'.").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    param = ito::Param("param", ito::ParamBase::String | ito::ParamBase::In, "Ra", QObject::tr("roughness or waviness parameter to determine").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "Rp");
    sm->addItem("Rv");
    sm->addItem("Rz");
    sm->addItem("Rt");
    sm->addItem("Ra");
    sm->addItem("Rq");
    sm->addItem("Rsk");
    sm->addItem("Rku");
    sm->addItem("Rdq");
    sm->addItem("Rda");
    sm->addItem("Rdc");
    sm->addItem("Wp");
    sm->addItem("Wv");
    sm->addItem("Wz");
    sm->addItem("Wt");
    sm->addItem("Wa");
    sm->addItem("Wq");
    sm->addItem("Wsk");
    sm->addItem("Wku");
    sm->addItem("Wdq");
    sm->addItem("Wda");
    sm->addItem("Wdc");
    sm->addItem("Pp");
    sm->addItem("Pv");
    sm->addItem("Pz");
    sm->addItem("Pt");
    sm->addItem("Pa");
    sm->addItem("Pq");
    sm->addItem("Psk");
    sm->addItem("Pku");
    sm->addItem("Pdq");
    sm->addItem("Pda");
    sm->addItem("Pdc");
    param.setMeta(sm, true);
    paramsMand->append(param);

    paramsMand->append(ito::Param("Lc", ito::ParamBase::Double | ito::ParamBase::In, 10.0, 100000.0, 80.0, QObject::tr("cut-off wavelength in _m used for the separation between the waviness and roughness").replace("_", QLatin1String("\u00B5")).toLatin1().data()));

    paramsOpt->append(ito::Param("nr_of_endeffect_pixels", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, QObject::tr("nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)").toLatin1().data()));
    paramsOpt->append(ito::Param("sampling_length_mode", ito::ParamBase::Int | ito::ParamBase::In, RoughnessParams::SplitInto5EqualSamples, RoughnessParams::Use5SamplesWithCutOffWavelength, RoughnessParams::SplitInto5EqualSamples, QObject::tr("mode how to split the measurement length (ml) into different sampling lengths (sl): 0: split ml into five samples (warn if the sl does not correspond to Lc), 1: split ml into n samples whose length is Lc, 2: same as 1 but only use the first 5 sampling lengths.").toLatin1().data()));
    paramsOpt->append(ito::Param("result_detailed", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("returns a Nx3 float64 data object will the mean,min and max roughness value for each evaluated line (only interesting if input contains more than one line)").toLatin1().data()));

    int mr_range[] = {20, 80};
    param = ito::Param("mr_range", ito::ParamBase::IntArray | ito::ParamBase::In, 2, mr_range, QObject::tr("[min,max] range in percent for the parameter Rdc or Wdc (height difference between two levels of the Abbott-Firstone-Curve").toLatin1().data());
    param.setMeta(new ito::RangeMeta(0, 100, 1), true);
    paramsOpt->append (param);

    paramsOut->append(ito::Param("result", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, QObject::tr("resulting values. See docstring of filter.").toLatin1().data()));
    paramsOut->append(ito::Param("nr_of_samples", ito::ParamBase::Int | ito::ParamBase::Out, 1, 1000, 5, QObject::tr("Number of evaluated samples per line (5 is the desired value with respect to DIN EN ISO 4288), Rt is evaluated over the whole measurement range, therefore nr_of_samples is 1.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal Roughness::evalRoughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int endeffect_pixels = paramsOpt->at(0).getVal<int>();
    ito::DataObject input = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "roughness", ito::Range(1, INT_MAX), ito::Range(2*endeffect_pixels+1, INT_MAX), retval, -1, ito::tInt8, ito::tInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    QByteArray param = paramsMand->at(1).getVal<char*>();
    double lc = paramsMand->at(2).getVal<double>();
    int sampling_length_mode = paramsOpt->at(1).getVal<int>();
    ito::DataObject *result_detailed = paramsOpt->at(2).getVal<ito::DataObject*>();
    const int *mr_range = paramsOpt->at(3).getVal<int*>();

    if (!retval.containsError())
    {
        const cv::Mat *plane = input.getCvPlaneMat(0);
        int type = input.getType();
        tSurfaceParamBasedOnSamplingLength func_type1 = NULL; //fCalcZp[type];
        tSurfaceParamBasedOnMeasurementLength func_type2 = NULL;

        switch (param[1])
        {
        case 'p':
            func_type1 = fCalcZp[type];
            break;
        case 'v':
            func_type1 = fCalcZv[type];
            break;
        case 'z':
            func_type1 = fCalcZz[type];
            break;
        case 'a':
            func_type1 = fCalcZa[type];
            break;
        case 'q':
            func_type1 = fCalcZq[type];
            break;
        case 's':
            func_type1 = fCalcZsk[type];
            break;
        case 'k':
            func_type1 = fCalcZku[type];
            break;
        case 't':
            func_type2 = fCalcZt[type];
            break;
        case 'd': //Rdq or Rda
            if (param[2] == 'q')
                func_type1 = fCalcZdq[type];
            else if (param[2] == 'a')
                func_type1 = fCalcZda[type];
            else
                func_type2 = fCalcZdc[type];
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, QObject::tr("unknown param name").toLatin1().data());
        }

        bool valid;
        std::string unit = input.getAxisUnit(1, valid);
        ito::float64 spacing = input.getAxisScale(1);

        if (!valid)
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("axis unit could not be read").toLatin1().data());
        }
        else if (unit == "mm")
        {
            spacing *= 1000.0; //convert to \mu m
        }
        else if (unit == QLatin1String("\u00B5m").latin1())
        {
        }
        else if (unit == "nm")
        {
            spacing /= 1000.0; //convert to \mu m
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("axis unit of input must be mm, _m or nm").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        }

        unit = input.getValueUnit();
        ito::float64 resultScale = 1.0;

        if (param[1] != 's' && param[1] != 'k')
        {//Zsk and Zku are without unit
            if (unit == "mm")
            {
                resultScale *= 1000.0; //convert to \mu m
            }
            else if (unit == QLatin1String("\u00B5m").latin1())
            {
            }
            else if (unit == "nm")
            {
                resultScale /= 1000.0; //convert to \mu m
            }
            else
            {
                retval += ito::RetVal(ito::retWarning, 0, QObject::tr("value unit of input should be mm, _m or nm. Else the right scaling of the resulting values (in _m) cannot be guaranteed.").replace("_", QLatin1String("\u00B5")).toLatin1().data());
            }
        }


        if (!retval.containsError())
        {
            RoughnessParams params;
            params.height_discriminiation = 10;
            params.spacing_discriminiation = 1;
            params.RdcLow = mr_range[0];
            params.RdcHigh = mr_range[1];
            params.spacing = spacing;
            params.cutoff_wavelength = lc;
            params.sampling_length_mode = (RoughnessParams::SamplingLengthDivision)sampling_length_mode;
            params.RskRkuOld = false;
            cv::Mat plane_cropped;

            if (endeffect_pixels == 0)
            {
                plane_cropped = *plane;
            }
            else
            {
                plane_cropped = plane->colRange(endeffect_pixels, plane->cols - endeffect_pixels);
            }

            if (plane->rows == 1)
            {
                int num_samples_out;
                ito::float64 Z_out[3];

                if (func_type1)
                {
                    retval += func_type1(plane_cropped.data, plane_cropped.step[1], plane_cropped.cols, params, Z_out[0], Z_out[1], Z_out[2], num_samples_out);
                    Z_out[0] *= resultScale;
                    Z_out[1] *= resultScale;
                    Z_out[2] *= resultScale;
                }
                else
                {
                    retval += func_type2(plane_cropped.data, plane_cropped.step[1], plane_cropped.cols, params, Z_out[0]);
                    num_samples_out = 1;
                    Z_out[0] *= resultScale;
                    Z_out[1] = Z_out[2] = Z_out[0];
                }

                if (!retval.containsError())
                {
                    (*paramsOut)[0].setVal<double*>(Z_out, 3);
                    (*paramsOut)[1].setVal<int>(num_samples_out);

                    if (result_detailed)
                    {
                        ito::DataObject result_detailed_(1,3,ito::tFloat64);
                        memcpy(result_detailed_.rowPtr(0,0), Z_out, 3*sizeof(ito::float64));
                        *result_detailed = result_detailed_;
                        result_detailed->setTag("lc", lc);
                        bool dummy;
                        result_detailed->setTag("ls", input.getTag("ls", dummy));
                    }
                }
            }
            else
            {
                int num_samples_out = 1;
                std::vector<ito::float64> Z_mean_out;
                ito::float64 Z_min_out, Z_max_out;
                Z_mean_out.resize(plane_cropped.rows);

                ito::DataObject result_detailed_;
                ito::float64 *row_ptr;
                ito::float64 row_ptr_dummy[3];
                if (result_detailed)
                {
                    result_detailed_ = ito::DataObject(plane_cropped.rows,3,ito::tFloat64);
                }

                for (int i = 0; i < plane_cropped.rows; ++i)
                {
                    row_ptr = result_detailed ? (ito::float64*)result_detailed_.rowPtr(0,i) : row_ptr_dummy;
                    if (func_type1)
                    {
                        retval += func_type1(plane_cropped.data + i * plane_cropped.step[0], plane_cropped.step[1], plane_cropped.cols, params, Z_mean_out[i], Z_min_out, Z_max_out, num_samples_out);
                        row_ptr[0] = Z_mean_out[i] * resultScale;
                        row_ptr[1] = Z_min_out * resultScale;
                        row_ptr[2] = Z_max_out * resultScale;
                    }
                    else
                    {
                        retval += func_type2(plane_cropped.data + i * plane_cropped.step[0], plane_cropped.step[1], plane_cropped.cols, params, Z_mean_out[i]);
                        row_ptr[0] = Z_mean_out[i] * resultScale;
                        row_ptr[1] = std::numeric_limits<ito::float64>::quiet_NaN();
                        row_ptr[2] = std::numeric_limits<ito::float64>::quiet_NaN();
                    }
                }

                if (result_detailed)
                {
                    *result_detailed = result_detailed_;
                    result_detailed->setTag("lc", lc);
                    bool dummy;
                    result_detailed->setTag("ls", input.getTag("ls", dummy));
                }

                if (!retval.containsError())
                {
                    (*paramsOut)[1].setVal<int>(num_samples_out);
                    ito::float64 result[] = {0.0, 0.0, 0.0, 0.0};
                    for (size_t i = 0; i < Z_mean_out.size(); ++i)
                    {
                        result[0] += Z_mean_out[i];
                    }
                    if (Z_mean_out.size() > 0)
                    {
                        result[0] /= Z_mean_out.size();
                        result[0] *= resultScale;

                        for (size_t i = 0; i < Z_mean_out.size(); ++i)
                        {
                            result[3] += (Z_mean_out[i] - result[0])*(Z_mean_out[i] - result[0]);
                        }

                        result[3] = std::sqrt(result[3] / Z_mean_out.size()) * resultScale;
                    }
                    result[1] = *(std::min_element(Z_mean_out.begin(), Z_mean_out.end())) * resultScale;
                    result[2] = *(std::max_element(Z_mean_out.begin(), Z_mean_out.end())) * resultScale;
                    (*paramsOut)[0].setVal<double*>(result, 4);
                }
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString Roughness::roughnessProfileDoc = QObject::tr("Calculate a profile roughness parameter for each line in the given input object. \n\
\n\
The evaluation is done based on DIN EN ISO 4287:2010, the separation of different sample lengths is based on DIN EN ISO 4288:1997. \n\
\n\
This filter is a thre-step filter, based on the single filters 'subtract1DRegression' from the plugin 'FittingFilters' \n\
and 'calcRoughnessProfile' as well as 'evalRoughnessProfile'. The first is an optional filter to remove the form of the raw signal. \n\
The second filter splits the raw signal into the waviness and roughness signal (given by the cut-off wavelengths Lc and Ls). \n\
The last filter evaluates the given roughness parameter and returns the result. For more information see the description of the single filters.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::roughnessProfileParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("input", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, QObject::tr("real input object (1D or 2D). The roughness is determined row-by-row. The axis units must be set to 'mm', '_m' or 'nm'.").replace("_", QLatin1String("\u00B5")).toLatin1().data()));

    param = ito::Param("param", ito::ParamBase::String | ito::ParamBase::In, "Ra", QObject::tr("roughness parameter to determine").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "Rp");
    sm->addItem("Rv");
    sm->addItem("Rz");
    sm->addItem("Rt");
    sm->addItem("Ra");
    sm->addItem("Rq");
    sm->addItem("Rsk");
    sm->addItem("Rku");
    sm->addItem("Rdq");
    sm->addItem("Rda");
    sm->addItem("Rdc");
    sm->addItem("Wv");
    sm->addItem("Wz");
    sm->addItem("Wt");
    sm->addItem("Wa");
    sm->addItem("Wq");
    sm->addItem("Wsk");
    sm->addItem("Wku");
    sm->addItem("Wdq");
    sm->addItem("Wda");
    sm->addItem("Wdc");
    param.setMeta(sm, true);
    paramsMand->append(param);

    paramsMand->append(ito::Param("Lc", ito::ParamBase::Double | ito::ParamBase::In, 10.0, 100000.0, 80.0, QObject::tr("cut-off wavelength in _m for the separation between the waviness and roughness").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsMand->append(ito::Param("Ls", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 25.0, 2.5, QObject::tr("short cut-off wavelength in _m for the separation between roughness and further high-frequency components (set 0.0 to omit this filtering)").replace("_", QLatin1String("\u00B5")).toLatin1().data()));

    paramsOpt->append(ito::Param("remove_form", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 1, QObject::tr("0: no form is subtracted from input as first step, else: a polynomial form of given order (1: line) is fitted and removed as first step (requires filter plugin 'FittingFilters')").toLatin1().data()));
    paramsOpt->append(ito::Param("periodicity", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, QObject::tr("0: non periodic profile (default), 1: periodic profile (not implemented yet)").toLatin1().data()));
    paramsOpt->append(ito::Param("cutoff_factor", ito::ParamBase::Double | ito::ParamBase::In, 0.5, 1.0, 0.6, QObject::tr("quality factor for the convolution based calculation. The convolution kernel is set to zero for indices > cutoff-wavelength * cutoff_factor. See ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0").toLatin1().data()));
    paramsOpt->append(ito::Param("sampling_length_mode", ito::ParamBase::Int | ito::ParamBase::In, RoughnessParams::SplitInto5EqualSamples, RoughnessParams::SplitIntoNSamplesWithCutOffWavelength, RoughnessParams::SplitInto5EqualSamples, QObject::tr("mode how to split the measurement length (ml) into different sampling lengths (sl): 0: split ml into five samples (warn if the sl does not correspond to Lc), 1: split ml into n samples whose length is Lc, 2: same as 1 but only use the first 5 sampling lengths.").toLatin1().data()));

    int mr_range[] = {20, 80};
    param = ito::Param("mr_range", ito::ParamBase::IntArray | ito::ParamBase::In, 2, mr_range, QObject::tr("[min,max] range in percent for the parameter Rdc or Wdc (height difference between two levels of the Abbott-Firstone-Curve").toLatin1().data());
    param.setMeta(new ito::RangeMeta(0, 100, 1), true);
    paramsOpt->append (param);

    paramsOut->append(ito::Param("result", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, QObject::tr("resulting values. See docstring of filter.").toLatin1().data()));
    paramsOut->append(ito::Param("nr_of_samples", ito::ParamBase::Int | ito::ParamBase::Out, 1, 1000, 5, QObject::tr("Number of evaluated samples per line (5 is the desired value with respect to DIN EN ISO 4288), Rt is evaluated over the whole measurement range, therefore nr_of_samples is 1.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::roughnessProfile(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject *input = paramsMand->at(0).getVal<ito::DataObject*>();
    double lc = paramsMand->at(2).getVal<double>();
    double ls = paramsMand->at(3).getVal<double>();
    QByteArray param = paramsMand->at(1).getVal<char*>();
    int remove_form = paramsOpt->at(0).getVal<int>();
    int periodicity = paramsOpt->at(1).getVal<int>();
    double cutoff_factor = paramsOpt->at(2).getVal<double>();
    int sampling_length_mode = paramsOpt->at(3).getVal<int>();
    const int *mr_range = paramsOpt->at(4).getVal<int*>();

    QVector<ito::ParamBase> p_mand, p_opt, p_out;
    ito::DataObject roughness, waviness;
    int nr_of_endeffect_pixels = 0;

    if (remove_form > 0)
    {
        retval += apiFilterParamBase("subtract1DRegression", &p_mand, &p_opt, &p_out);
        if (!retval.containsError())
        {
            p_mand[0].setVal<ito::DataObject*>(input);
            p_mand[1].setVal<ito::DataObject*>(input);
            p_mand[2].setVal<int>(remove_form);
            p_opt[0].setVal<int>(1);
            retval += apiFilterCall("subtract1DRegression", &p_mand, &p_opt, &p_out);
        }
    }

    if (!retval.containsError())
    {
        retval += apiFilterParamBase("calcRoughnessProfile", &p_mand, &p_opt, &p_out);
        if (!retval.containsError())
        {
            p_mand[0].setVal<ito::DataObject*>(input);
            p_mand[1].setVal<ito::DataObject*>(&roughness);
            p_mand[2].setVal<ito::DataObject*>(&waviness);
            p_mand[3].setVal<double>(lc);
            p_mand[4].setVal<double>(ls);
            p_opt[0].setVal<const char*>("auto");
            p_opt[1].setVal<int>(periodicity);
            p_opt[2].setVal<double>(cutoff_factor);
            retval += apiFilterCall("calcRoughnessProfile", &p_mand, &p_opt, &p_out);
            nr_of_endeffect_pixels = p_out[0].getVal<int>();
        }
    }

    if (!retval.containsError())
    {
        retval += apiFilterParamBase("evalRoughnessProfile", &p_mand, &p_opt, &p_out);
        if (!retval.containsError())
        {
            p_mand[0].setVal<ito::DataObject*>(&roughness);
            p_mand[1].setVal<char*>(param.data());
            p_mand[2].setVal<double>(lc);
            p_opt[0].setVal<int>(nr_of_endeffect_pixels);
            p_opt[1].setVal<int>(sampling_length_mode);
            p_opt[3].setVal<int*>((int*)mr_range,2);
            retval += apiFilterCall("evalRoughnessProfile", &p_mand, &p_opt, &p_out);

            if (!retval.containsError())
            {
                if (p_out[0].getLen() > 0)
                {
                    (*paramsOut)[0].setVal<double*>(p_out[0].getVal<double*>(), p_out[0].getLen());
                }
                (*paramsOut)[1].setVal<int>(p_out[1].getVal<int>());
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString Roughness::calcAbbottCurveDoc = QObject::tr("This filter calculates the Abbott firestone curve (as well as an optional histogram) of \n\
a roughness or waviness profile, e.g. obtained from the filter 'calcRoughnessProfile'. \n\
\n\
The evaluation is done based on DIN EN ISO 4287:2010.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::calcAbbottCurveParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("roughness", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, QObject::tr("real input object (1D or 2D, no unsigned data types) - must be either the output argument 'roughness' or 'waviness' from filter 'calcRoughnessProfile'. The roughness is determined row-by-row. The axis units must be set to 'mm', '_m' or 'nm'.").replace("_", QLatin1String("\u00B5")).toLatin1().data()));
    paramsMand->append(ito::Param("abbott", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("abbott firestone curve with a horizontal step width of 1% (float64, size: Mx100 for M rows in the input object.").toLatin1().data()));

    paramsOpt->append(ito::Param("nr_of_endeffect_pixels", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, QObject::tr("nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)").toLatin1().data()));
    paramsOpt->append(ito::Param("histogram", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, QObject::tr("optional histogram with 100 buckets (int32, 1x100). Only the first line is evaluated.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::calcAbbottCurve(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int endeffect_pixels = paramsOpt->at(0).getVal<int>();
    ito::DataObject input = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "roughness", ito::Range(1, INT_MAX), ito::Range(2*endeffect_pixels+1, INT_MAX), retval, -1, ito::tInt8, ito::tInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);

    if (!retval.containsError())
    {
        const cv::Mat *plane = input.getCvPlaneMat(0);
        cv::Mat plane_cropped;

        if (endeffect_pixels == 0)
        {
            plane_cropped = *plane;
        }
        else
        {
            plane_cropped = plane->colRange(endeffect_pixels, plane->cols - endeffect_pixels);
        }

        int type = input.getType();

        ito::DataObject abbott_curve(plane_cropped.rows, 1001, ito::tFloat64);
        ito::DataObject histogram(1, 1000, ito::tInt32);
        ito::DataObject *h = paramsOpt->at(1).getVal<ito::DataObject*>();

        for (int r = 0; r < plane_cropped.rows; ++r)
        {
            retval += fCalcAbbott[type](plane_cropped.ptr(r), plane_cropped.step[1], plane_cropped.cols, (ito::float64*)abbott_curve.rowPtr(0,0), (r == 0 && h) ? &histogram : NULL);
        }

        abbott_curve.setAxisScale(1, 0.1);
        abbott_curve.setAxisDescription(1, "Mr");
        abbott_curve.setAxisUnit(1, "%");
        abbott_curve.setValueDescription("c");
        abbott_curve.setValueUnit(input.getValueUnit());

        histogram.setValueDescription("counts");
        histogram.setAxisUnit(1, input.getValueUnit());
        histogram.setAxisDescription(1, "z");

        *((*paramsMand)[1].getVal<ito::DataObject*>()) = abbott_curve;

        if (h)
        {
            *h = histogram;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** helper for gauss_conv_non_periodic_rowwise: generates the gaussian kernel (based on DIN EN ISO 16610-21:2013)
*
*   this method returns the kernel s(x) for x = 0 : spacing : ceil(cutoff_factor * cutoff)
*   The first half of the kernel is not calculated since it is symmetrical.
*   If the kernel is applied in gauss_conv_non_periodic_rowwise, the first and last (len(kernel)-1) values must be ignored since they do not
*   contain valid values.
*
*   In addition to the standard, the kernel is normalized. In the continuous domain the integral of the gaussian kernel
*   has to be 1.0 in order to guarantee the desired amplitude transfer factor of 1.0. This is usually achieved if the discrete values
*   of the discrete gaussian kernel are summed up. However if the sampling rate is very low (close to the cut-off frequency), the discrete
*   integral is over-estimated, which leads to bad results. Of course, one has to think if it is allowed to filter under-sampled profiles
*   with a short cut-off frequency, nevertheless, if this happens, it is not desired to get magnified results. Therefore, we decided
*   to normalize the gaussian profile.
*
*   @param [in] spacing is the spacing between two points in input_line (in \mu m, nm or similar length unit)
*   @param [in] cutoff is the cut-off wavelength lambda of the filter (must be in the same unit than spacing)
*   @param [in] cutoff factor for gaussian weighting function (see Lc in DIN EN ISO 16610-21), ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0
*   @param [out] nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)
*
*   @seealso gauss_conv_periodic_rowwise
*/
std::vector<ito::float64> Roughness::gen_gauss_convolution(const ito::float64 spacing, const ito::float64 cutoff, const ito::float64 cutoff_factor, int &nr_of_endeffect_pixels)
{
    ito::float64 alpha = 0.46971863934982566688; //sqrt(log(2)/pi)
    ito::float64 sqrt_pi = 1.7724538509055160273; //sqrt(pi)

    ito::float64 h1 = spacing / (alpha * cutoff);
    ito::float64 h3 = 0.0;
    ito::float64 h2 = sqrt_pi * h1;
    ito::float64 sum = 0.0;

    int m = std::ceil((cutoff * cutoff_factor) / spacing);

    std::vector<ito::float64> kernel;
    kernel.resize(m+1);

    /* k = 0: */
    //h3 = 0.0;
    kernel[0] = h1;
    sum += kernel[0];

    /* k > 0: */
    for (int k = 1; k <= m; ++k)
    {
        h3 = h2 * (double)k;
        kernel[k] = h1 * std::exp(-h3*h3);
        sum += 2 * kernel[k];
    }

    //normalize
    for (int k = 0; k <= m; ++k)
    {
        kernel[k] /= sum;
    }

    nr_of_endeffect_pixels = m;

    return kernel;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** helper for gauss_conv_non_periodic_rowwise: generates the fourier transform of the gaussian kernel (based on DIN EN ISO 16610-21:2013)
*
*   this method returns the kernel s(x) for x = 0 : spacing : ceil(cutoff_factor * cutoff)
*   The first half of the kernel is not calculated since it is symmetrical.
*   If the kernel is applied, the first and last (len(kernel)-1) values must be ignored since they do not
*   contain valid values.
*
*   @param [in] length is the desired length of the fourier transformed gaussian peak
*   @param [in] spacing is the spacing between two points in input_line (in \mu m, nm or similar length unit)
*   @param [in] cutoff is the cut-off wavelength lambda of the filter (must be in the same unit than spacing)
*   @param [in] cutoff factor for gaussian weighting function (see Lc in DIN EN ISO 16610-21), ISO 16610-21:2013 A.5: 0.5 for general purpose, high precision: 0.6, reference software: 1.0
*   @param [out] nr of pixels at both sides of the profile that contain wrong values to the final length of the convolution filter kernel. Skip these pixels in any evaluation (see DIN EN ISO 16610-21, 4.3; DIN EN ISO 16610-28 will show more information when it is released in a final version)
*
*   @seealso gauss_conv_periodic_rowwise
*/
std::vector<ito::float64> Roughness::gen_dft_gauss_convolution(const int length, const ito::float64 spacing, const ito::float64 cutoff, const ito::float64 cutoff_factor, int &nr_of_endeffect_pixels)
{
    ito::float64 alpha = 0.46971863934982566688; //sqrt(log(2)/pi)
    ito::float64 sqrt_pi = 1.7724538509055160273; //sqrt(pi)

    ito::float64 f = (alpha * cutoff) / (length * spacing);
    ito::float64 h2 = sqrt_pi * sqrt_pi * f * f;

    int m = std::ceil(cutoff_factor / (alpha * f));

    std::vector<ito::float64> kernel;
    kernel.resize(m+1);
    for (int k = 0; k <= m; ++k)
    {
        kernel[k] = std::exp(-h2 * (double)(k * k));
    }

    nr_of_endeffect_pixels = m;

    return kernel;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** discrete gaussian convolution for open, non-periodic profiles implemented using the dft from OpenCV
*
*   implemented based on "Die digitale Implementierung des Profilfilters nach DIN EN ISO 11562", Michael Krystek, Beuth Verlag, 1. Auflage, 2004
*
*   This filter applies a gaussian filtering of input data and returns the lowpass filtered values such that
*   the transmission at the cutoff wavelength is 50%.
*
*   See also: Michael Krystek, "A fast gauss filtering algorithm for roughness measurements", Precision Engineering 19, 1996
*
*   @param [in] input is a float64 cv::Mat where each line will be filtered independently
*   @param [in/out] output is the resulting float64 cv::Mat contain the resulting, lowpass filtered values (must be allocated with the same length than input and must not be the same than input)
*   @param [in] half_gauss_kernel is the pre-calculated gaussian kernel for non periodic profiles as given by DIN EN ISO 16610-21:2013, 4.1 (beginning with the center-value to the right end cut after a certain length)
*
*/
ito::RetVal Roughness::gauss_dft_non_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_dft_kernel, int length_optimized /*= -1*/)
{
    int m = half_gauss_dft_kernel.size() - 1;
    int length = input->cols;

    if (m < 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("given gauss kernel too short").toLatin1().data());
    }

    m = std::min(m, length/2);

    if (length == length_optimized) //input has optimal size
    {
        cv::Mat_<ito::float64> w(input->rows, input->cols, CV_64FC1);

        cv::dft(*input, w, cv::DFT_ROWS);

        ito::float64 *w_ptr;

        for (int r = 0; r < w.rows; ++r)
        {
            w_ptr = (ito::float64*)w.ptr(r);
            w_ptr[0] *= 1.0;

            for (int i = 1; i <= m; ++i)
            {
                w_ptr[i*2-1] *= half_gauss_dft_kernel[i]; //ReY0,1
                w_ptr[i*2]   *= half_gauss_dft_kernel[i]; //ImY0,1
            }

            memset(&(w_ptr[2*m+1]), 0, (w.cols - 2*m - 1) * sizeof(ito::float64));
        }

        cv::dft(w, *output, cv::DFT_ROWS | cv::DFT_REAL_OUTPUT | cv::DFT_INVERSE | cv::DFT_SCALE);
    }
    else //input must be rescaled
    {
        cv::Size optimized_size(length_optimized, input->rows);
        cv::Mat input_(optimized_size, input->type(), cv::Scalar::all(0));
        cv::Mat input_roi(input_, cv::Rect(0,0, input->cols, input->rows));
        input->copyTo(input_roi);
        cv::Mat w_(optimized_size, input->type());
        cv::Mat output_(optimized_size, output->type());

        cv::dft(input_, w_, cv::DFT_ROWS);

        ito::float64 *w_ptr;

        for (int r = 0; r < w_.rows; ++r)
        {
            w_ptr = (ito::float64*)w_.ptr(r);
            w_ptr[0] *= 1.0;

            for (int i = 1; i <= m; ++i)
            {
                w_ptr[i*2-1] *= half_gauss_dft_kernel[i]; //ReY0,1
                w_ptr[i*2]   *= half_gauss_dft_kernel[i]; //ImY0,1
            }

            memset(&(w_ptr[2*m+1]), 0, (w_.cols - 2*m - 1) * sizeof(ito::float64));
        }

        cv::dft(w_, output_, cv::DFT_ROWS | cv::DFT_REAL_OUTPUT | cv::DFT_INVERSE | cv::DFT_SCALE);
        output_(cv::Rect(0, 0, output->cols, output->rows)).copyTo(*output);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** discrete gaussian convolution for open, non-periodic profiles
*
*   implemented based on "Die digitale Implementierung des Profilfilters nach DIN EN ISO 11562", Michael Krystek, Beuth Verlag, 1. Auflage, 2004
*
*   This filter applies a gaussian filtering of input data and returns the lowpass filtered values such that
*   the transmission at the cutoff wavelength is 50%.
*
*   See also: Michael Krystek, "A fast gauss filtering algorithm for roughness measurements", Precision Engineering 19, 1996
*
*   @param [in] input is a float64 cv::Mat where each line will be filtered independently
*   @param [in/out] output is the resulting float64 cv::Mat contain the resulting, lowpass filtered values (must be allocated with the same length than input and must not be the same than input)
*   @param [in] half_gauss_kernel is the pre-calculated gaussian kernel for non periodic profiles as given by DIN EN ISO 16610-21:2013, 4.1 (beginning with the center-value to the right end cut after a certain length)
*
*/
ito::RetVal Roughness::gauss_conv_non_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_kernel)
{
    int m = half_gauss_kernel.size() - 1;
    int length = input->cols;
    const ito::float64 *input_ptr = NULL;
    ito::float64 *output_ptr = NULL;

    if (m < 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("given gauss kernel too short").toLatin1().data());
    }
    else if (length < 2 * m)
    {
        return ito::RetVal::format(ito::retError, 0, QObject::tr("the given data must have at least %i values").toLatin1().data(), 2*m);
    }

    for (int r = 0; r < input->rows; ++r)
    {
        input_ptr = (const ito::float64*)input->ptr(r);
        output_ptr = (ito::float64*)output->ptr(r);

        //first part (values out of range are assumed to be 0.0)
        for (int i = 0; i < m; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((input_ptr[i+k] + ((i >= k) ? input_ptr[i-k] : 0.0)) * half_gauss_kernel[k]);
            }
        }

        //middle part
        for (int i = m; i < length - m; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((input_ptr[i+k] + input_ptr[i-k]) * half_gauss_kernel[k]);
            }
        }

        //last part (values out of range are assumed to be 0.0)
        for (int i = length - m; i < length; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((((i+k) < length ? input_ptr[i+k] : 0.0) + input_ptr[i-k]) * half_gauss_kernel[k]);
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** discrete gaussian convolution for closed periodic profiles
*
*   implemented based on "Die digitale Implementierung des Profilfilters nach DIN EN ISO 11562", Michael Krystek, Beuth Verlag, 1. Auflage, 2004
*
*   This filter applies a gaussian filtering of input data and returns the lowpass filtered values such that
*   the transmission at the cutoff wavelength is 50%.
*
*   See also: Michael Krystek, "A fast gauss filtering algorithm for roughness measurements", Precision Engineering 19, 1996
*
*   @param [in] input is a float64 cv::Mat where each line will be filtered independently
*   @param [in/out] output is the resulting float64 cv::Mat contain the resulting, lowpass filtered values (must be allocated with the same length than input and must not be the same than input)
*   @param [in] half_gauss_kernel is the pre-calculated gaussian kernel for non periodic profiles as given by DIN EN ISO 16610-21:2013, 4.1 (beginning with the center-value to the right end cut after a certain length)
*
*/
ito::RetVal Roughness::gauss_conv_periodic_rowwise(const cv::Mat_<ito::float64> *input, cv::Mat_<ito::float64> *output, const std::vector<ito::float64> &half_gauss_kernel)
{
    int m = half_gauss_kernel.size() - 1;
    int length = input->cols;
    const ito::float64 *input_ptr = NULL;
    ito::float64 *output_ptr = NULL;

    if (m < 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("given gauss kernel too short").toLatin1().data());
    }
    else if (length < 2 * m)
    {
        return ito::RetVal::format(ito::retError, 0, QObject::tr("the given data must have at least %i values").toLatin1().data(), 2*m);
    }

    for (int r = 0; r < input->rows; ++r)
    {
        input_ptr = (const ito::float64*)input->ptr(r);
        output_ptr = (ito::float64*)output->ptr(r);

        //first part (values out of range are assumed to be 0.0)
        for (int i = 0; i < m; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((input_ptr[i+k] + ((i >= k) ? input_ptr[i-k] : input_ptr[length + i - k])) * half_gauss_kernel[k]);
            }
        }

        //middle part
        for (int i = m; i < length - m; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((input_ptr[i+k] + input_ptr[i-k]) * half_gauss_kernel[k]);
            }
        }

        //last part (values out of range are assumed to be 0.0)
        for (int i = length - m; i < length; ++i)
        {
            output_ptr[i] = input_ptr[i] * half_gauss_kernel[0];
            for (int k = 1; k <= m; ++k)
            {
                output_ptr[i] += ((((i+k) < length ? input_ptr[i+k] : input_ptr[i + k - length]) + input_ptr[i-k]) * half_gauss_kernel[k]);
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize filter functions within this addIn
*    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on initialization
*    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on initialization
*    @return                    retError in case of an error
*
*    Here are the filter functions defined that are available through this addIn.
*    These are:
*       - filterName    description for this filter
*
*   This plugin additionally makes available the following widgets, dialogs...:
*       - dialogName    description for this widget
*/
ito::RetVal Roughness::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval;

    m_filterList.insert("calcRoughnessProfile", new FilterDef(calcRoughnessProfile, calcRoughnessProfileParams, calcRoughnessProfileDoc));
    m_filterList.insert("evalRoughnessProfile", new FilterDef(evalRoughnessProfile, evalRoughnessProfileParams, evalRoughnessProfileDoc));
    m_filterList.insert("getGaussianFilterKernel", new FilterDef(getGaussianFilterKernel, getGaussianFilterKernelParams, getGaussianFilterKernelDoc));
    m_filterList.insert("roughnessProfile", new FilterDef(roughnessProfile, roughnessProfileParams, roughnessProfileDoc));
    m_filterList.insert("calcAbbottCurve", new FilterDef(calcAbbottCurve, calcAbbottCurveParams, calcAbbottCurveDoc));

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Roughness::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}
