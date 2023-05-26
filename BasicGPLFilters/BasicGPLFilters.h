/* ********************************************************************
    Plugin "BasicGPLFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public Licence as published by
    the Free Software Foundation; either version 3 of the Licence, or (at
    your option) any later version.

    This plugin contains code and algorithms inspired or copied from other open
    source projects under GNU General Public Licence, e.g.

        - GIMP 2.8 under GPL version 3.0 or higher

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */


/*! \file BasicGPLFilters.h
   \brief   This is the main header file for BasicGPLFilters libary, which contains the interface declarations.

   \author trwip
   \date 04.2014
*/

#ifndef BasicGPLFilters_H
#define BasicGPLFilters_H

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"

#include <qsharedpointer.h>

#define MAX_RADIUS 30
#define MAX_LIST_ELEMS (2 * MAX_RADIUS + 1)*(2 * MAX_RADIUS + 1)

/*!\struct PixelsList
   \brief Structure containing pointer to the original pixels within each luma bucket
   \autor  Based on code developed by Michael Sweet for GIMP, adaped to itom /c++ by W. Lyda
*/
typedef struct
{
  const ito::uint8*     elems[MAX_LIST_ELEMS];      /*! Original pixel pointers stored in a ringbuffer*/
  ito::int32            start;                      /*! First allowed pixel pointer*/
  ito::int32            count;                      /*! Number of current used pixel pointer*/
} PixelsList;


/*!\struct DespeckleHistogram
   \brief structure containing the filter kernel wrapped into an histogramm
          The filter uses this structure to create a ring buffered local histogramm within kernel-size.
          The image is median filtered based on this histogramm. The histogramm has 256 bins.
   \autor  Based on code developed by Michael Sweet for GIMP, adaped to itom /c++ by W. Lyda
*/
typedef struct
{
  ito::int32   elems[256]; /*! Number of pixels that fall into each luma bucket */
  PixelsList   origs[256]; /*! Original pixels stored as PixelList */
  ito::int32   xmin;       /*! first pixel coordinate in x for source rect*/
  ito::int32   ymin;       /*! first pixel coordinate in y for source rect*/
  ito::int32   xmax;       /*! last pixel coordinate in x for source rect*/
  ito::int32   ymax;       /*! last pixel coordinate in y for source rect*/
} DespeckleHistogram;


/*!\struct DespeckleHistogram
   \brief structure defining filter functions which have been static variables in orignal code
*/
typedef struct
{
    ito::int32 histLess; /*! Number of ignored pixel lower than the black limit*/
    ito::int32 histMore; /*! Number of ignored pixel higher than the white limit*/
    ito::int32 histRemain; /*! Number of pixels within the histogramm*/
    ito::int32 blackLevel; /*! balck (lower) limit as value between 0 and 255*/
    ito::int32 whiteLevel; /*! White (upper) limit as value between 0 and 255*/
    ito::int32 radiusMax;  /*! Maximum kernel size (more a rectangle) */
    bool adaptRadius;      /*! Toggle wether kernel may be adepted between 1 and radiusMax*/
} DespeckleSettings;

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicGPLFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the BasicGPLFilters class s. also \ref BasicGPLFilters
*/
class BasicGPLFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        BasicGPLFiltersInterface();
        ~BasicGPLFiltersInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicGPLFilters
*   @brief Algorithms used to process images and dataobjects with filters develped at ITO
*
*   In this class the algorithms used for the processing of measurement data are implemented.
*
*/
class BasicGPLFilters : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        BasicGPLFilters();
        ~BasicGPLFilters();

    public:
        friend class BasicGPLFiltersInterface;

        static const char* despeckle_adapted;

        static ito::RetVal despeckleAdapted(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut);
        static ito::RetVal despeckleAdaptedParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);


    private:
        template<typename _Tp> static void despeckleAdaptedFilterBlock(DespeckleSettings &filterSettings, const cv::Mat* cvMatIn, cv::Mat* cvMatOut);


        template<typename _Tp> static inline ito::uint8 pixel_luminance (const ito::uint8 *pixel)
        {
            return *pixel;
        }

        static inline void histogram_add (DespeckleHistogram &hist, const ito::uint8 &val, const ito::uint8 *orig)
        {
            hist.elems[val]++;
            // Add the element to the original value list
            const ito::int32 pos = hist.origs[val].start + hist.origs[val].count++;
            hist.origs[val].elems[pos >= MAX_LIST_ELEMS ? pos - MAX_LIST_ELEMS : pos] = orig;
        }

        static inline void histogram_remove (DespeckleHistogram &hist, const ito::uint8 &val)
        {
            hist.elems[val]--;

            // Delete the element from the original value list
            hist.origs[val].count--;
            hist.origs[val].start++;

            if (hist.origs[val].start >= MAX_LIST_ELEMS) hist.origs[val].start = 0;
        }

        static inline void histogram_clean (DespeckleHistogram &hist)
        {
            ito::int32 i;

            for (i = 0; i < 256; i++)
            {
                hist.elems[i] = 0;
                hist.origs[i].count = 0;
            }
        }

        static inline const ito::uint8 * histogram_get_median (ito::int32 count, DespeckleHistogram &hist, const ito::uint8 * defVal)
        {

            ito::int32 i;
            ito::int32 sum = 0;

            if (count < 1)
            {
                return defVal;
            }

            count = (count + 1) / 2;    // half count of total elements within the histogramm

            i = 0;  // the median possition within the histogramm
            while ((sum += hist.elems[i]) < count)
            {
                i++;
            }

            // Get a random element from the right bucket
            const ito::int32 pos = hist.origs[i].start + rand() % hist.origs[i].count;

            if (pos >= MAX_LIST_ELEMS) return hist.origs[i].elems[pos - MAX_LIST_ELEMS];
            else return hist.origs[i].elems[pos];
        }

        template<typename _Type> static inline void add_vals (
            DespeckleSettings &settings,
            DespeckleHistogram &hist,
            const ito::uint8 *src,
            const ito::int32 &width,
            const ito::int32 &xmin,
            const ito::int32 &ymin,
            const ito::int32 &xmax,
            const ito::int32 &ymax)
        {
            ito::int32 x;
            ito::int32 y;

            if (xmin > xmax)
                return;

            for (y = ymin; y <= ymax; y++)
            {
                for (x = xmin; x <= xmax; x++)
                {
                    //add_val<_Type>(hist, src, width, x, y);
                    const ito::int32 pos   = (x + (y * width)) * sizeof(_Type);
                    //const ito::int32 pos   = x * sizeof(_Type);
                    const ito::uint8 value = pixel_luminance<_Type>(src + pos);
                    if (value > settings.blackLevel && value < settings.whiteLevel)
                    {
                        histogram_add (hist, value, src + pos);
                        settings.histRemain++;
                    }
                    else
                    {
                        if (value <= settings.blackLevel)
                            settings.histLess++;

                        if (value >= settings.whiteLevel)
                            settings.histMore++;
                    }
                }
            }
        }

        template<typename _Type>  static inline void del_vals (
            DespeckleSettings &settings,
            DespeckleHistogram &hist,
            const ito::uint8 *src,
            const ito::int32 &width,
            const ito::int32 &xmin,
            const ito::int32 &ymin,
            const ito::int32 &xmax,
            const ito::int32 &ymax)
        {
            ito::int32 x;
            ito::int32 y;

            if (xmin > xmax)
                return;

            for (y = ymin; y <= ymax; y++)
            {
                for (x = xmin; x <= xmax; x++)
                {
                    //del_val<_Type>(hist, src, width, x, y);
                    const ito::int32 pos   = (x + (y * width)) * sizeof(_Type);
                    //const ito::int32 pos   = x * sizeof(_Type);
                    const ito::int32 value = pixel_luminance<_Type>(src + pos);

                    if (value > settings.blackLevel && value < settings.whiteLevel)
                    {
                        histogram_remove (hist, value);
                        settings.histRemain--;
                    }
                    else
                    {
                        if (value <= settings.blackLevel)
                            settings.histLess--;

                        if (value >= settings.whiteLevel)
                            settings.histMore--;
                    }
                }
            }
        }

        template<typename _Tp> static inline void update_histogram (
            DespeckleSettings &settings,
            DespeckleHistogram &hist,
            const ito::uint8 *src,
            const ito::int32 &width,
            const ito::int32 &xmin,
            const ito::int32 &ymin,
            const ito::int32 &xmax,
            const ito::int32 &ymax)
        {
            /* assuming that radious of the box can change no more than one
                pixel in each call */
            /* assuming that box is moving either right or down */

            del_vals<_Tp>(settings, hist, src, width, hist.xmin, hist.ymin, xmin - 1, hist.ymax);
            del_vals<_Tp>(settings, hist, src, width, xmin, hist.ymin, xmax, ymin - 1);
            del_vals<_Tp>(settings, hist, src, width, xmin, ymax + 1, xmax, hist.ymax);

            add_vals<_Tp>(settings, hist, src, width, hist.xmax + 1, ymin, xmax, ymax);
            add_vals<_Tp>(settings, hist, src, width, xmin, ymin, hist.xmax, hist.ymin - 1);
            add_vals<_Tp>(settings, hist, src, width, hist.xmin, hist.ymax + 1, hist.xmax, ymax);

            hist.xmin = xmin;
            hist.ymin = ymin;
            hist.xmax = xmax;
            hist.ymax = ymax;
        }

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

template<> /*static*/ inline ito::uint8 BasicGPLFilters::pixel_luminance<ito::Rgba32>(const ito::uint8 *pixel)
{
    return (ito::Rgba32::fromUnsignedLong((long)*pixel)).gray();
}

//----------------------------------------------------------------------------------------------------------------------------------
#endif // BasicGPLFilters_H
