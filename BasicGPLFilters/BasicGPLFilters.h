/* ********************************************************************
    Plugin "BasicGPLFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

/*! \file BasicGPLFilters.h
   \brief   This is the main file for the M++Filter libary, which contains the interface definition. 
   
   The algorithms in this dll are mostly copied from the filter.h and filter.cpp. The filters are grouped in different sub .cpp-files.

   \author ITO 
   \date 12.2011
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

typedef struct
{
  const ito::uint8   *elems[MAX_LIST_ELEMS];
  ito::int32    start;
  ito::int32    count;
} PixelsList;

typedef struct
{
  ito::int32   elems[256]; /* Number of pixels that fall into each luma bucket */
  PixelsList   origs[256]; /* Original pixels */
  ito::int32   xmin;
  ito::int32   ymin;
  ito::int32   xmax;
  ito::int32   ymax; /* Source rect */
} DespeckleHistogram;

//----------------------------------------------------------------------------------------------------------------------------------
/** @class BasicGPLFiltersInterface
*   @brief ITO developed filter functions for the itom
*
*   AddIn Interface for the BasicGPLFilters class s. also \ref BasicGPLFilters
*/
class BasicGPLFiltersInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5,0,0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
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


        static ito::int32 histLess;
        static ito::int32 histMore;
        static ito::int32 histRemain;
        static ito::int32 blackLevel;
        static ito::int32 whiteLevel;

    private:
        template<typename _Tp> static void despeckleAdaptedFilterBlock(const cv::Mat* cvMatIn, cv::Mat* cvMatOut, const ito::int32 radius, const bool adaptRadius);

        template<typename _Tp> static inline ito::uint8 pixel_luminance (const ito::uint8 *pixel)
        {
            return *pixel;
        }

        static inline void list_add_elem (PixelsList *list, const ito::uint8 *elem)
        {
            const ito::int32 pos = list->start + list->count++;
            list->elems[pos >= MAX_LIST_ELEMS ? pos - MAX_LIST_ELEMS : pos] = elem;
        }

        static inline void list_del_elem (PixelsList* list)
        {
            list->count--;
            list->start++;

            if (list->start >= MAX_LIST_ELEMS) list->start = 0;
        }

        static inline const ito::uint8 * list_get_random_elem (PixelsList *list)
        {
            const ito::int32 pos = list->start + rand() % list->count;

            if (pos >= MAX_LIST_ELEMS) return list->elems[pos - MAX_LIST_ELEMS];
            else return list->elems[pos];
        }

        static inline void histogram_add (DespeckleHistogram *hist, ito::uint8 val, const ito::uint8 *orig)
        {
            hist->elems[val]++;
            list_add_elem (&hist->origs[val], orig);
        }

        static inline void histogram_remove (DespeckleHistogram *hist, ito::uint8 val)
        {
            hist->elems[val]--;
            list_del_elem (&hist->origs[val]);
        }

        static inline void histogram_clean (DespeckleHistogram *hist)
        {
            ito::int32 i;

            for (i = 0; i < 256; i++)
            {
                hist->elems[i] = 0;
                hist->origs[i].count = 0;
            }
        }

        static inline const ito::uint8 * histogram_get_median (DespeckleHistogram *hist, const ito::uint8 * defVal)
        {
            ito::int32 count = BasicGPLFilters::histRemain;
            ito::int32 i;
            ito::int32 sum = 0;

            if (count < 1)
            {
                return defVal;
            }

            count = (count + 1) / 2;

            i = 0;
            while ((sum += hist->elems[i]) < count)
            {
                i++;
            }
            return list_get_random_elem (&hist->origs[i]);
        }

        template<typename _Tp> static inline void add_val (DespeckleHistogram *hist, const ito::uint8 *src, ito::int32 width, ito::int32 x, ito::int32 y)
        {
            const ito::int32 pos   = (x + (y * width)) * sizeof(_Tp);
            const ito::uint8 value = pixel_luminance<_Tp>(src + pos);

            if (value > blackLevel && value < whiteLevel)
            {
                histogram_add (hist, value, src + pos);
                histRemain++;
            }
            else
            {
                if (value <= blackLevel)
                    histLess++;

                if (value >= whiteLevel)
                    histMore++;
            }
        }

        template<typename _Tp> static inline void del_val (DespeckleHistogram *hist, const ito::uint8 *src, ito::int32 width, ito::int32 x, ito::int32 y)
        {
            const ito::int32 pos   = (x + (y * width)) * sizeof(_Tp);
            const ito::int32 value = pixel_luminance<_Tp>(src + pos);

            if (value > blackLevel && value < whiteLevel)
            {
                histogram_remove (hist, value);
                BasicGPLFilters::histRemain--;
            }
            else
            {
                if (value <= blackLevel)
                    BasicGPLFilters::histLess--;

                if (value >= whiteLevel)
                    BasicGPLFilters::histMore--;
            }
        }

        template<typename _Type> static inline void add_vals (DespeckleHistogram *hist, const ito::uint8 *src, ito::int32 width, ito::int32 xmin, ito::int32 ymin, ito::int32 xmax, ito::int32 ymax)
        {
            ito::int32 x;
            ito::int32 y;

            if (xmin > xmax)
                return;

            for (y = ymin; y <= ymax; y++)
            {
                for (x = xmin; x <= xmax; x++)
                {
                    add_val<_Type>(hist, src, width, x, y);
                }
            }
        }

        template<typename _Type>  static inline void del_vals (DespeckleHistogram *hist, const ito::uint8 *src, ito::int32 width, ito::int32 xmin, ito::int32 ymin, ito::int32 xmax, ito::int32 ymax)
        {
            ito::int32 x;
            ito::int32 y;

            if (xmin > xmax)
                return;

            for (y = ymin; y <= ymax; y++)
            {
                for (x = xmin; x <= xmax; x++)
                {
                    del_val<_Type>(hist, src, width, x, y);
                }
            }
        }

        template<typename _Tp> static inline void update_histogram (DespeckleHistogram *hist, const ito::uint8 *src, ito::int32 width, ito::int32 xmin, ito::int32 ymin, ito::int32 xmax, ito::int32 ymax)
        {
            /* assuming that radious of the box can change no more than one
                pixel in each call */
            /* assuming that box is moving either right or down */

            del_vals<_Tp>(hist, src, width, hist->xmin, hist->ymin, xmin - 1, hist->ymax);
            del_vals<_Tp>(hist, src, width, xmin, hist->ymin, xmax, ymin - 1);
            del_vals<_Tp>(hist, src, width, xmin, ymax + 1, xmax, hist->ymax);

            add_vals<_Tp>(hist, src, width, hist->xmax + 1, ymin, xmax, ymax);
            add_vals<_Tp>(hist, src, width, xmin, ymin, hist->xmax, hist->ymin - 1);
            add_vals<_Tp>(hist, src, width, hist->xmin, hist->ymax + 1, hist->xmax, ymax);

            hist->xmin = xmin;
            hist->ymin = ymin;
            hist->xmax = xmax;
            hist->ymax = ymax;
        }

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

template<> static inline ito::uint8 BasicGPLFilters::pixel_luminance<ito::Rgba32>(const ito::uint8 *pixel)
{
    return (ito::Rgba32::fromUnsignedLong((long)*pixel)).gray();
}

//----------------------------------------------------------------------------------------------------------------------------------
#endif // BasicGPLFilters_H
