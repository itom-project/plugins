/* ********************************************************************
    Plugin "OpenCV-Filter" for itom software
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include <math.h>
#include "OpenCVFilters.h"
#include "itomCvConversions.h"
#include "common/numeric.h"
#include "gitVersion.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"
#include "common/numeric.h"

//#ifdef _DEBUG
#ifdef USEOPENMP
    #define useomp 1
#else
    #define useomp 0
#endif

#include <QtCore/QtPlugin>
#include <qnumeric.h>

#define TIMEBENCHMARK 0
//#include "common/helperCommon.h"

int NTHREADS = 1;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(OpenCVFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(OpenCVFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersInterface::OpenCVFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("OpenCV-Filters");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This plugin provides wrappers for various OpenCV algorithms. These are for instance: \n\
\n\
* morphological filters (dilation, erosion) \n\
* image filtering (blur, median blur...) \n\
* 1d and 2d fft and ifft \n\
* histogram determination \n\
* feature detections (circles, chessboard corners...) \n\
\n\
This plugin not only requires access to the core library of OpenCV but also to further libraries like \
imgproc and calib3d.";
*/
    m_description = QObject::tr("Wrapped algorithms from OpenCV");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin provides wrappers for various OpenCV algorithms. These are for instance: \n\
\n\
* morphological filters (dilation, erosion) \n\
* image filtering (blur, median blur...) \n\
* 1d and 2d fft and ifft \n\
* histogram determination \n\
* feature detections (circles, chessboard corners...) \n\
\n\
This plugin not only requires access to the core library of OpenCV but also to further libraries like \
imgproc and calib3d. \n\
\n\
This plugin has been created at a time when OpenCV did not yet provide bindings for Python 3. \n\
From OpenCV 3 on, these bindings exist. Therefore, it is possible to access almost all OpenCV \n\
methods via the cv2 python package. The wrapped methods within this plugin can still be used; \n\
In addition to the cv2 methods, they can sometimes operate on multi-plane dataObjects, preserve \n\
the tags and meta information and save protocol data.");

    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    NTHREADS  = QThread::idealThreadCount();

 //    ito::tParam paramVal = ito::tParam("Number of Axis", ito::ParamBase::Int, 0, 10, 6, "Number of axis for this Motor");
//    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersInterface::~OpenCVFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFilters::OpenCVFilters() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFilters::~OpenCVFilters()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date
*/
ito::RetVal OpenCVFilters::stdParams2Objects(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("scrImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::checkInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output, bool * unequal)
{
    *unequal = false;

    if (!p_input)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!p_output)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (p_input->getDims() == 1)
    {
        if ((p_output->getSize(0) != p_input->getSize(0)) || (p_output->getType() != p_input->getType()))
        {
            *unequal = true;
        }
    }
    else if (p_input->getDims() == 2)
    {
        if ((p_output->getSize(0) != p_input->getSize(0)) || (p_output->getSize(1) != p_input->getSize(1)) || (p_output->getType() != p_input->getType()))
        {
            *unequal = true;
        }
    }
    else if (p_output->getDims() == 3)
    {
        if ((p_output->getSize(0) != p_input->getSize(0)) || (p_output->getSize(1) != p_input->getSize(1)) || (p_output->getSize(2) != p_input->getSize(2)) || (p_output->getType() != p_input->getType()))
        {
            *unequal = true;
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::makeInputOutputEqual(ito::DataObject * p_input, ito::DataObject * p_output)
{
    ito::RetVal ret = ito::retOk;
    bool check = false;

    ret = checkInputOutputEqual(p_input, p_output, &check);

    if (!ret.containsError() && check)    // okay image has to be changed and no error
    {
        if ((p_output == p_input))
        {
            ret = ito::RetVal(ito::retError, 0, tr("Error: pointer of input and output objects are equal").toLatin1().data());
        }
        if (p_input->getDims() == 1)
        {
            (*p_output) = ito::DataObject(p_input->getSize(0), p_input->getType());
        }
        else if (p_input->getDims() == 2)
        {
            (*p_output) = ito::DataObject(p_input->getSize(0), p_input->getSize(1), p_input->getType());
        }
        else if (p_output->getDims() == 3)
        {
            (*p_output) = ito::DataObject(p_input->getSize(0), p_input->getSize(1), p_input->getSize(2), p_input->getType());
        }
        else
            return ito::RetVal(ito::retError, 0, tr("Error: the check command is currently not implemented for more than 3 dims").toLatin1().data());
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvDilateErodeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceObj", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input data object of type uint8, uint16, int16, float32, float64").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationObj", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output image with the same type and size than input (inplace allowed)").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("element", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("structuring element used for the morpholocial operation (default: None, a 3x3 rectangular structuring element is used). Else: An uint8 data object where values > 0 are considered for the operation.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("anchor", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("position of the anchor within the element. If not given or if (-1,-1), the anchor is at the element center [default].").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("iterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 65000, 1, tr("number of times the morpholocial operation is applied [default: 1]").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("borderType", ito::ParamBase::String | ito::ParamBase::In, "CONSTANT", tr("This string defines how the filter should hande pixels at the border of the matrix. Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvDilateErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt,  QVector<ito::ParamBase> * /*paramsOut*/, bool erodeNotDilate)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (!dObjSrc || !dObjDst)
    {
        return ito::RetVal(ito::retError,0,tr("source and destination object must not be NULL").toLatin1().data());
    }

    int dims = dObjSrc->getDims();

    if (dims < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    // Check if input type is allowed or not
    retval += ito::dObjHelper::verifyDataObjectType(dObjSrc, "source data object", 5, ito::tUInt8, ito::tUInt16, ito::tInt16, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    //iterations
    int iterations = (*paramsOpt)[2].getVal<int>();

    //create structuring element
    cv::Mat cvElement = cv::Mat();

    if (paramsOpt->at(0).getVal<ito::DataObject*>())
    {
        ito::DataObject *element = apiCreateFromDataObject(paramsOpt->at(0).getVal<ito::DataObject*>(), 2, ito::tUInt8, NULL, &retval);
        if (element && !retval.containsError())
        {
            cvElement = cv::Mat( *((cv::Mat*)(element->get_mdata()[ element->seekMat(0)])));
            delete element;
        }
    }

    //border type
    QString borderTypeStr = paramsOpt->at(3).getVal<char*>() ? paramsOpt->at(3).getVal<char*>() : QString();
    int borderType;
    if (QString::compare(borderTypeStr, "CONSTANT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_CONSTANT;
    }
    else if (QString::compare(borderTypeStr, "REPLICATE", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REPLICATE;
    }
    else if (QString::compare(borderTypeStr, "REFLECT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT;
    }
    else if (QString::compare(borderTypeStr, "WRAP", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_WRAP;
    }
    else if (QString::compare(borderTypeStr, "REFLECT_101", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT_101;
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("border type %1 is unknown").arg(borderTypeStr.toLatin1().data()).toLatin1().data());
        return retval;
    }

    //anchor
    int anchorLen = paramsOpt->at(1).getLen();
    cv::Point anchor;

    if (paramsOpt->at(1).getLen() == 2)
    {
        anchor = cv::Point( paramsOpt->at(1).getVal<int*>()[0], paramsOpt->at(1).getVal<int*>()[1] );

        int m = dObjSrc->getSize( dims - 2 );
        int n = dObjSrc->getSize( dims - 1 );

        if (anchor.x < 0 || anchor.x >= n || anchor.y < 0 || anchor.y >= m)
        {
            retval += ito::RetVal(ito::retError, 0, tr("anchor must be in range [0,%1];[0,%2]").arg(m-1).arg(n-1).toLatin1().data());
            return retval;
        }
    }
    else if (paramsOpt->at(1).getLen() <= 0)
    {
        anchor = cv::Point(-1,-1);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("anchor must have either 2 values or none").toLatin1().data());
        return retval;
    }

    int planes = dObjSrc->calcNumMats();

    //dObjDst is either equal to dObjSrc or must have the same size and type than dObjSrc (if not is created such it fullfills these requirements)
    if (dObjDst != dObjSrc)
    {
        int dstDim = dObjDst->getDims();
        int dstType = dObjDst->getType();
        int *sizes = new int[dims];
        bool sizeFit = true;

        for (int i = 0; i < dims; ++i)
        {
            sizes[i] = dObjSrc->getSize(i);
            if (dstDim != dims || sizes[i] != dObjDst->getSize(i))
            {
                sizeFit = false;
            }
        }

        if (dstDim != dims || sizeFit == false || dstType != dObjSrc->getType())
        {
            (*dObjDst) = ito::DataObject(dims, sizes, dObjSrc->getType());
        }

        delete[] sizes;

        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut;

    if (erodeNotDilate)
    {
        for (int z = 0; z < planes; z++)
        {
            try
            {
                cvMatIn = ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                cv::erode(*cvMatIn, *cvMatOut, cvElement, anchor, iterations, borderType);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }
    else
    {
        for (int z = 0; z < planes; z++)
        {
            try
            {
                cvMatIn = ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                cv::dilate(*cvMatIn, *cvMatOut, cvElement, anchor, iterations, borderType);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }

    if (!retval.containsError())
    {

        QString msg;
        if (erodeNotDilate)
        {
            msg = tr("erosion with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6").arg(cvElement.rows).arg(cvElement.cols).arg(anchor.y).arg(anchor.x).arg(iterations).arg(borderTypeStr);
        }
        else
        {
            msg = tr("dilation with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6").arg(cvElement.rows).arg(cvElement.cols).arg(anchor.y).arg(anchor.x).arg(iterations).arg(borderTypeStr);
        }
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvDilateDoc = QObject::tr("Dilates every plane of a data object by using a specific structuring element. \n\
\n\
This filter applies the dialation method cvDilate of OpenCV to every plane in the source data object. The \
result is contained in the destination object. It can handle data objects of type uint8, uint16, int16, float32 and float64 only. \n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated. \n\
\n\
The dilation is executed using a structuring element which is (if not otherwise stated) a 3x3 kernel filled with ones. Else you can give \
an two-dimensional uint8 data object. Then, the function dilates the source image using the specified structuring element that determines \
the shape of a pixel neighborhood over which the maximum is taken: \n\
\n\
dst(x,y) = max_{(x',y'):element(x',y')!=0} src(x+x',y+y') \n\
\n\
Dilation can be applied several times (parameter 'iterations').");

ito::RetVal OpenCVFilters::cvDilate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return cvDilateErode(paramsMand, paramsOpt, paramsOut, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvErodeDoc = QObject::tr("Erodes every plane of a data object by using a specific structuring element. \n\
\n\
This filter applies the erosion method cvErode of OpenCV to every plane in the source data object. The \
result is contained in the destination object. It can handle data objects of type uint8, uint16, int16, float32 and float64 only. \n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated. \n\
\n\
The erosion is executed using a structuring element which is (if not otherwise stated) a 3x3 kernel filled with ones. Else you can give \
an two-dimensional uint8 data object. Then, the function dilates the source image using the specified structuring element that determines \
the shape of a pixel neighborhood over which the maximum is taken: \n\
\n\
dst(x,y) = min_{(x',y'):element(x',y')!=0} src(x+x',y+y') \n\
\n\
Erosion can be applied several times (parameter 'iterations').");

ito::RetVal OpenCVFilters::cvErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return cvDilateErode(paramsMand, paramsOpt, paramsOut, true);
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMorphologyExParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceObj", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input data object of type uint8, uint16, int16, float32, float64").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationObj", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output image with the same type and size than input (inplace allowed)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("operation", ito::ParamBase::Int | ito::ParamBase::In, 0, 7, 0, tr("This parameters defines the operation type, 0: Erode, 1: Dilate, 2: Open, 3: Close, 4: Gradient, 5: Tophat, 6: Blackhat, 7: Hit or miss").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("element", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("structuring element used for the morpholocial operation (default: None, a 3x3 rectangular structuring element is used). Else: An uint8 data object where values > 0 are considered for the operation.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("anchor", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("position of the anchor within the element. If not given or if (-1,-1), the anchor is at the element center [default].").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("iterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 65000, 1, tr("number of times the morpholocial operation is applied [default: 1]").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("borderType", ito::ParamBase::String | ito::ParamBase::In, "CONSTANT", tr("This string defines how the filter should hande pixels at the border of the matrix. Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMorphologyEx(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    int op = (*paramsMand)[2].getVal<int>();

    if (!dObjSrc || !dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("source and destination object must not be NULL").toLatin1().data());
    }

    int dims = dObjSrc->getDims();

    if (dims < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    // Check if input type is allowed or not
    retval += ito::dObjHelper::verifyDataObjectType(dObjSrc, "source data object", 5, ito::tUInt8, ito::tUInt16, ito::tInt16, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    //iterations
    int iterations = (*paramsOpt)[2].getVal<int>();

    //create structuring element
    cv::Mat cvElement = cv::Mat();

    if (paramsOpt->at(0).getVal<ito::DataObject*>())
    {
        ito::DataObject *element = apiCreateFromDataObject(paramsOpt->at(0).getVal<ito::DataObject*>(), 2, ito::tUInt8, NULL, &retval);
        if (element && !retval.containsError())
        {
            cvElement = cv::Mat(*((cv::Mat*)(element->get_mdata()[element->seekMat(0)])));
            delete element;
        }
    }

    //border type
    QString borderTypeStr = paramsOpt->at(3).getVal<char*>() ? paramsOpt->at(3).getVal<char*>() : QString();
    int borderType;
    if (QString::compare(borderTypeStr, "CONSTANT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_CONSTANT;
    }
    else if (QString::compare(borderTypeStr, "REPLICATE", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REPLICATE;
    }
    else if (QString::compare(borderTypeStr, "REFLECT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT;
    }
    else if (QString::compare(borderTypeStr, "WRAP", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_WRAP;
    }
    else if (QString::compare(borderTypeStr, "REFLECT_101", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT_101;
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("border type %1 is unknown").arg(borderTypeStr.toLatin1().data()).toLatin1().data());
        return retval;
    }

    //anchor
    int anchorLen = paramsOpt->at(1).getLen();
    cv::Point anchor;

    if (paramsOpt->at(1).getLen() == 2)
    {
        anchor = cv::Point(paramsOpt->at(1).getVal<int*>()[0], paramsOpt->at(1).getVal<int*>()[1]);

        int m = dObjSrc->getSize(dims - 2);
        int n = dObjSrc->getSize(dims - 1);

        if (anchor.x < 0 || anchor.x >= n || anchor.y < 0 || anchor.y >= m)
        {
            retval += ito::RetVal(ito::retError, 0, tr("anchor must be in range [0,%1];[0,%2]").arg(m - 1).arg(n - 1).toLatin1().data());
            return retval;
        }
    }
    else if (paramsOpt->at(1).getLen() <= 0)
    {
        anchor = cv::Point(-1, -1);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("anchor must have either 2 values or none").toLatin1().data());
        return retval;
    }

    int planes = dObjSrc->calcNumMats();

    //dObjDst is either equal to dObjSrc or must have the same size and type than dObjSrc (if not is created such it fullfills these requirements)
    if (dObjDst != dObjSrc)
    {
        int dstDim = dObjDst->getDims();
        int dstType = dObjDst->getType();
        int *sizes = new int[dims];
        bool sizeFit = true;

        for (int i = 0; i < dims; ++i)
        {
            sizes[i] = dObjSrc->getSize(i);
            if (dstDim != dims || sizes[i] != dObjDst->getSize(i))
            {
                sizeFit = false;
            }
        }

        if (dstDim != dims || sizeFit == false || dstType != dObjSrc->getType())
        {
            (*dObjDst) = ito::DataObject(dims, sizes, dObjSrc->getType());
        }

        delete[] sizes;

        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut;

    for (int z = 0; z < planes; z++)
    {
        try
        {
            cvMatIn = ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
            cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
            cv::morphologyEx(*cvMatIn, *cvMatOut, op, cvElement, anchor, iterations, borderType);
        }
        catch (cv::Exception &exc)
        {
            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
            break;
        }
    }
    

    if (!retval.containsError())
    {

        QString msg;
        msg = tr("morphologyEx with (y,x) kernel(%1, %2), anchor(%3, %4), %5 iterations, borderType %6").arg(cvElement.rows).arg(cvElement.cols).arg(anchor.y).arg(anchor.x).arg(iterations).arg(borderTypeStr);

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvMorphologyExDoc = QObject::tr("Erodes every plane of a data object by using a specific structuring element. \n\
\n\
Performs advanced morphological transformations.\
\
The function cv::morphologyEx can perform advanced morphological transformations using an erosion and dilation as basic operations.\
MORPH_ERODE \
\
\
Any of the operations can be done in - place.In case of multi - channel images, each channel is processed independently.).");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("All types except complex64 and complex128 are accepted").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty object handle. Image will be of src-type").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernelSizeX", ito::ParamBase::Int | ito::ParamBase::In, 1, 255, 3, tr("Kernelsize for x-axis").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("kernelSizeY", ito::ParamBase::Int | ito::ParamBase::In, 1, 255, 3, tr("Kernelsize for y-axis").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("anchor", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Position of the kernel anchor, see openCV-Help").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("borderType", ito::ParamBase::String | ito::ParamBase::In, "CONSTANT", tr("border mode used to extrapolate pixels outside of the image").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvBlurDoc = QObject::tr("Planewise median blur filter.\n\
\n\
This filter applies the method cv::blur to every plane in the source data object. The function smoothes the images by a simple mean-filter. The\
result is contained in the destination object. It can handle data objects of type uint8, uint16, int16, ito::tInt32, float32 and float64 only. \n\
\n\
The cv::blur interally calls the cv::boxfilter()-method.\n\
\n\
The itom-wrapping does not work inplace currently. A new dataObject is allocated.\n\
\n\
borderType: This string defines how the filter should hande pixels at the border of the matrix.\
Allowed is CONSTANT [default], REPLICATE, REFLECT, WRAP, REFLECT_101. In case of a constant border, only pixels inside of the element mask are considered (morphologyDefaultBorderValue)\
\n\
Warning: NaN-handling for floats not verified.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;
    cv::Size kernelsizes(3,3);
    cv::Point anchor(-1,-1);
//    bool checkequal = 0;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjImages, "sourceImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
    {
        return retval;
    }

    kernelsizes.width = (*paramsOpt)[0].getVal<int>();
    kernelsizes.height = (*paramsOpt)[1].getVal<int>();

    ito::DataObject *dObjAnchor = (ito::DataObject*)(*paramsOpt)[2].getVal<void*>();

    //border type
    QString borderTypeStr = paramsOpt->at(3).getVal<char*>() ? paramsOpt->at(3).getVal<char*>() : QString();
    int borderType;
    if (QString::compare(borderTypeStr, "CONSTANT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_CONSTANT;
    }
    else if (QString::compare(borderTypeStr, "REPLICATE", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REPLICATE;
    }
    else if (QString::compare(borderTypeStr, "REFLECT", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT;
    }
    else if (QString::compare(borderTypeStr, "WRAP", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_WRAP;
    }
    else if (QString::compare(borderTypeStr, "REFLECT_101", Qt::CaseInsensitive) == 0)
    {
        borderType = cv::BORDER_REFLECT_101;
    }
    else
    {
        retval += ito::RetVal::format(ito::retError,0,"border type %s is unknown", borderTypeStr.toLatin1().data());
        return retval;
    }

    if (dObjAnchor)
    {
        if (dObjAnchor->getSize(0) == ito::tInt8)
        {
            return ito::RetVal(ito::retError, 0, tr("Error: anchor should be 'int8'").toLatin1().data());
        }

        if ((dObjAnchor->getDims() == 2) && (dObjAnchor->getSize(1) == 2) && (dObjAnchor->getSize(0) == 1))
        {
            anchor.x = dObjAnchor->at<ito::int8>(0, 0);
            anchor.y = dObjAnchor->at<ito::int8>(0, 1);
        }
        else
            return ito::RetVal(ito::retError, 0, tr("Error: anchor has wrong size or number of dims").toLatin1().data());
    }

    int z_length = dObjImages->calcNumMats();

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut = new cv::Mat[z_length];
    ito::tDataType itomtype;
    ito::RetVal ret;

    for (int z = 0; z < z_length; z++)
    {
        try
        {
            cvMatIn = ((cv::Mat *)dObjImages->get_mdata()[dObjImages->seekMat(z)]);
            cv::blur(*cvMatIn, cvMatOut[z], kernelsizes, anchor, borderType);
        }
        catch (cv::Exception &exc)
        {
            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
            goto end;
        }
    }

    // Warning: if you copy this, this could cause a problem
    itomtype = ito::guessDataTypeFromCVMat(&(cvMatOut[0]), ret);
    if (!ret.containsError())
    {
        *dObjDst = ito::DataObject(dObjImages->getDims(), dObjImages->getSize(), itomtype, cvMatOut, z_length);

        dObjImages->copyAxisTagsTo(*dObjDst);
        dObjImages->copyTagMapTo(*dObjDst);

        QString msg;
        msg = tr("OpenCV blur-filter with (y,x) kernel(%1, %2), anchor(%3, %4), borderType %5").arg(kernelsizes.height).arg(kernelsizes.width).arg(anchor.y).arg(anchor.x).arg(borderType);
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("No compatible dataObject type found for given OpenCV matrix type.").toLatin1().data());
    }

end:
    delete[] cvMatOut;

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "Time: " << duration << "ms\n";
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvFFT2DDoc = QObject::tr("2D-dimentional fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.\
The result is a complex-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, false)-function.\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvIFFT2DDoc = QObject::tr("2D-dimentional inverse fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.\
The result is a real-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, false)-function.\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvFFT1DDoc = QObject::tr("1D-dimentional fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.\
The result is a complex-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, true)-function.\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvIFFT1DDoc = QObject::tr("1D-dimentional inverse fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.\
The result is a real-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, true)-function.\n\
");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFFTParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Input Object handle, must be a single plane").toLatin1().data());
        paramsMand->append(param);
        //param = ito::Param("destinationImage", ito::ParamBase::DObjPtr, NULL, tr("Output Object handle. Will be come complex-type").toLatin1().data());
        //paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
//    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    ito::dObjHelper::calcCVDFT(dObjImages, false, false, false);

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "Time: " << duration << "ms\n";
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
//    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    ito::dObjHelper::calcCVDFT(dObjImages, false, false, true);

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "Time: " << duration << "ms\n";
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvIFFT2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
//    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    ito::dObjHelper::calcCVDFT(dObjImages, true, true, false);

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "Time: " << duration << "ms\n";
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvIFFT1D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
//    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    ito::dObjHelper::calcCVDFT(dObjImages, true, false, true);

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "Time: " << duration << "ms\n";
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvMedianBlurDoc = QObject::tr("Planewise median blur filter.\n\
\n\
The function smoothes an image using the median filter with the kernel-size x kernel-size aperture. Each channel of a multi-channel image is processed independently. \
It can handle data objects of type uint8, uint16, int16, ito::tInt32, float32 and float64 only. \n\
\n\
The itom-wrapping does not work inplace currently. A new dataObject is allocated.\n\
\n\
Warning: NaN-handling for floats not verified.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMedianBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Image of type Integer or float32").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObject-hanlde. Destination is of source type").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernelSize", ito::ParamBase::Int | ito::ParamBase::In, 3, 255, 3, tr("Kernelsize in x/y").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMedianBlur(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
//    bool checkequal = 0;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjImages, "sourceImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    int kernelsize = (*paramsOpt)[0].getVal<int>();

    if (kernelsize % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel must be odd").toLatin1().data());
    }

    
        
#if (CV_VERSION_MAJOR < 3 || (CV_VERSION_MAJOR > 3 && CV_VERSION_MINOR < 3))
    if (kernelsize > 5 && dObjImages->getType() != ito::tUInt8)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernelsize > 3 and object is not uint8").toLatin1().data());
    }
#endif

    int z_length = dObjImages->calcNumMats();

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut = new cv::Mat[z_length];
    ito::tDataType itomtype;
    ito::RetVal ret;

    for (int z = 0; z < z_length; z++)
    {
        try
        {
            cvMatIn = ((cv::Mat *)dObjImages->get_mdata()[dObjImages->seekMat(z)]);
            cv::medianBlur(*cvMatIn, cvMatOut[z], kernelsize);
        }
        catch (cv::Exception &exc)
        {
            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
            goto end;
        }
    }
    // Warning: if you copy this, this could cause a problem
    itomtype = ito::guessDataTypeFromCVMat(&(cvMatOut[0]), ret);
    if (!ret.containsError())
    {
        *dObjDst = ito::DataObject(dObjImages->getDims(), dObjImages->getSize(), itomtype, cvMatOut, z_length);
        //(*dObjDst).setT(dObjImages->isT());

        dObjImages->copyAxisTagsTo(*dObjDst);
        dObjImages->copyTagMapTo(*dObjDst);

        // Add Protokoll
        QString msg;
        msg = tr("OpenCV medianblur-filter with kernel size = %1").arg(kernelsize);
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));

    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("No compatible dataObject type found for given OpenCV matrix type.").toLatin1().data());
    }

end:
    delete[] cvMatOut;
    return retval;
}

////----------------------------------------------------------------------------------------------------------------------------------
//static const char * cvCalcHistDoc = "Planewise histogram calculation";
////----------------------------------------------------------------------------------------------------------------------------------
//ito::RetVal OpenCVFilters::cvCalcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
//{
//    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
//    if (!retval.containsError())
//    {
//        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Image of type Integer or float").toLatin1().data());
//        paramsMand->append(param);
//        param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObject-handle. Will be source type later").toLatin1().data());
//        paramsMand->append(param);
//        param = ito::Param("Steps", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("Number of steps").toLatin1().data());
//        paramsOpt->append(param);
//    }
//    return retval;
//}
////----------------------------------------------------------------------------------------------------------------------------------
//ito::RetVal OpenCVFilters::cvCalcHist(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
//{
//    ito::RetVal retval = ito::retOk;
////    bool checkequal = 0;
//
//    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
//    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
//
//    return ito::RetVal(ito::retError, 0, tr("Error: not implemented yet").toLatin1().data());
//
//    if (!dObjImages)
//    {
//        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
//    }
//
//    if (!dObjDst)
//    {
//        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
//    }
//
//    if (dObjImages->getDims() < 2)
//    {
//        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
//    }
//
//    int hbins = (*paramsOpt)[0].getVal<int>();
//    double maxVal = 0;
//    double minVal = 0;
//    bool recalcMinMax = false;
//
//    switch(dObjImages->getType())
//    {
//        case ito::tInt8:
//            hbins = 256;
//            minVal = -128;
//            maxVal = 127;
//            break;
//        case ito::tUInt8:
//            hbins = 256;
//            minVal = 0;
//            maxVal = 255;
//            break;
//        case ito::tUInt16:
//        case ito::tInt16:
//        case ito::tInt32:
//        case ito::tFloat32:
//            if ( hbins < 1 )
//            {
//                hbins = 1024;
//            }
//            recalcMinMax = true;
//            break;
//        default:
//            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toLatin1().data());
//    }
//
//    int z_length = dObjImages->calcNumMats();
//
//    cv::Mat *cvMatIn;
//    cv::MatND *cvMatOut = new cv::Mat[z_length];
//
//    int histSize[] = {hbins};
//    int channels[] = {0};
//
//    float hranges[] = { (float)minVal, (float)maxVal };
//    int itomtype = 0;
//    for (int z = 0; z < z_length; z++)
//    {
//        cvMatIn = ((cv::Mat *)dObjImages->get_mdata()[dObjImages->seekMat(z)]);
//        if (recalcMinMax)
//        {
//            cv::minMaxLoc(*cvMatIn, &minVal, &maxVal, 0, 0);
//
//            if (qIsNaN(maxVal)||qIsInf(maxVal))
//            {
//                maxVal = std::numeric_limits<float>::max();
//            }
//            if (qIsNaN(minVal)||qIsInf(minVal))
//            {
//                maxVal = std::numeric_limits<float>::max() * -1;
//            }
//            if (minVal == maxVal)
//            {
//                minVal =  maxVal + 0.001;
//            }
//            hranges[0] = cv::saturate_cast<float>(minVal);
//            hranges[1] = cv::saturate_cast<float>(maxVal);
//        }
//
//        const float* ranges[] = {hranges};
//        try
//        {
//            calcHist( cvMatIn, 1, channels, cv::Mat(), // do not use mask
//                    cvMatOut[z], 1, histSize, ranges,
//                    true, // the histogram is uniform
//                    false);
//        }
//        catch (cv::Exception exc)
//        {
//            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
//            goto end;
//        }
//    }
//
//    // Warning: if you copy this, this could cause a problem
//    itomtype = ito::dObjHelper::cvType2itomType(cvMatOut[0].type());
//    if (itomtype > 0)
//    {
//        *dObjDst = ito::DataObject(dObjImages->getDims(), dObjImages->getSize(), itomtype, cvMatOut, z_length);
//        //(*dObjDst).setT(dObjImages->isT());
//    }
//    else
//    {
//        retval += ito::RetVal(ito::retError, 0, tr("Unknown or unexpected CV-Datatype recived.").toLatin1().data());
//    }
//
//end:
//    delete[] cvMatOut;
//    return retval;
//}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvFlipLeftRightDoc = QObject::tr("This filter flips the image left to right. \n\
\n\
This filter applies the flip method cvFlip of OpenCV with the flipCode > 0 to a 2D source data object. The \
result is contained in the destination object\n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated\n\
.");

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvFlipUpDownDoc = QObject::tr("This filter flips the image upside down. \n\
\n\
This filter applies the flip method cvFlip of OpenCV with the flipCode = 0 to a 2D source data object. The \
result is contained in the destination object.\n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated\n\
.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFlipLeftRight(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return cvFlip(paramsMand, paramsOpt, paramsOut, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFlipUpDown(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return cvFlip(paramsMand, paramsOpt, paramsOut, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFlip(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/, bool colsIfTrue)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::DataObject destTemp;

    bool overWrite = true;

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet, only 2D and 3D.").toLatin1().data());
    }

    if (dObjImages->getDims() == 0)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: input object must not be empty.").toLatin1().data());
    }

    int ysize = dObjImages->getSize(dObjImages->getDims() - 2);
    int xsize = dObjImages->getSize(dObjImages->getDims() - 1);
    int planes = 0;
    if (dObjImages->getDims() > 1)
    {
        planes = dObjImages->getSize(dObjImages->getDims() - 3);
    }

    if (!retval.containsError())
    {
        retval += ito::dObjHelper::verifyDataObjectType(dObjImages, "srcImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    //if (planes > 0)
    //{
    //    retval += ito::dObjHelper::verify3DDataObject(dObjImages, "srcImage", planes, planes, ysize, ysize, xsize, xsize,  1, dObjImages->getType());
    //}
    //else
    //{
    //    retval += ito::dObjHelper::verify2DDataObject(dObjImages, "srcImage", ysize, ysize, xsize, xsize,  1, dObjImages->getType());
    //}

    if (!retval.containsError())
    {
        if (dObjDst != dObjImages)
        {
            if (planes > 0)
            {
                ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjDst, "destImage", planes, planes, ysize, ysize, xsize, xsize,  1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    int sizes[3] = {planes, ysize, xsize};
                    destTemp = ito::DataObject(3, sizes, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }
            }
            else
            {
                ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjDst, "destImage", ysize, ysize, xsize, xsize,  1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    destTemp = ito::DataObject(ysize, xsize, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }            
            }
        }
        else
        {
            //destDataPhase = ito::DataObject( ysize, xsize, ito::tFloat64);
            destTemp = *dObjDst;
            overWrite = false;
        }
    }

    if (!retval.containsError())
    {
        int z_length = dObjImages->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjImages->get_mdata()[dObjImages->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[destTemp.seekMat(z)];
            try
            {
                cv::flip(*cvMatIn, *cvMatOut, colsIfTrue ? 1 : 0);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }

    if (!retval.containsError())
    {
        if (overWrite)
        {
            *dObjDst = destTemp;
        }

        if (dObjDst != dObjImages)
        {
            dObjImages->copyAxisTagsTo(*dObjDst);
            dObjImages->copyTagMapTo(*dObjDst);
        }

        QString msg = colsIfTrue ? tr("Flipped left/rigth with cvFlip-Filter") : tr("Flipped upside/down with cvFlip-Filter");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvRotP90Doc = QObject::tr("This filter rotates the image by 90° count clock wise. \n\
\n\
This filter applies the flip method cvFlip and the transpose method cvTranspose of OpenCV to rotate the object. The \
result is contained in the destination object\n\
\n\
It is allowed to let the filter work pseudo inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated.\n");

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvRotM90Doc = QObject::tr("This filter rotates the image by 90° clock wise. \n\
\n\
This filter applies the flip method cvFlip and the transpose method cvTranspose of OpenCV to rotate the object. The \
result is contained in the destination object\n\
\n\
It is allowed to let the filter work pseudo inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated.\n");


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvRotP90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return cvRotate(paramsMand, paramsOpt, paramsOut, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvRotM90(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return cvRotate(paramsMand, paramsOpt, paramsOut, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvRotate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/, bool rotClw)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::DataObject destTemp;

    bool overWrite = true;

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet, only 2D and 3D.").toLatin1().data());
    }

    if (dObjImages->getDims() == 0)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: input object must not be empty.").toLatin1().data());
    }

    int ysize = dObjImages->getSize(dObjImages->getDims() - 2);
    int xsize = dObjImages->getSize(dObjImages->getDims() - 1);
    int planes = 0;
    if (dObjImages->getDims() > 2)
    {
        planes = dObjImages->getSize(dObjImages->getDims() - 3);
    }

    if (!retval.containsError())
    {
        retval += ito::dObjHelper::verifyDataObjectType(dObjImages, "srcImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    if (!retval.containsError())
    {
        if (dObjDst != dObjImages)
        {
            if (planes > 0)
            {
                ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjDst, "destImage", planes, planes, xsize, xsize, ysize, ysize, 1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    int sizes[3] = {planes, xsize, ysize};
                    destTemp = ito::DataObject(3, sizes, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }
            }
            else
            {
                ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjDst, "destImage", xsize, xsize, ysize, ysize, 1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    destTemp = ito::DataObject(xsize, ysize, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }            
            }
        }
        else
        {
            if (planes > 0)
            {
                int sizes[3] = {planes, xsize, ysize};
                destTemp = ito::DataObject(3, sizes, dObjImages->getType());
            }
            else
            {
                destTemp = ito::DataObject(xsize, ysize, dObjImages->getType());          
            }
        }
    }

    if (!retval.containsError())
    {
        int z_length = dObjImages->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjImages->get_mdata()[dObjImages->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[destTemp.seekMat(z)];
            try
            {
                cv::transpose(*cvMatIn, *cvMatOut);
                cv::flip(*cvMatOut, *cvMatOut, rotClw ? 0 : 1);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }

    if (!retval.containsError())
    {
        dObjImages->copyAxisTagsTo(destTemp);
        dObjImages->copyTagMapTo(destTemp);

        bool check;
        destTemp.setAxisDescription(destTemp.getDims() - 2, dObjImages->getAxisDescription(dObjImages->getDims() - 1, check)); 
        destTemp.setAxisDescription(destTemp.getDims() - 1, dObjImages->getAxisDescription(dObjImages->getDims() - 2, check)); 

        destTemp.setAxisUnit(destTemp.getDims() - 2, dObjImages->getAxisUnit(dObjImages->getDims() - 1, check)); 
        destTemp.setAxisUnit(destTemp.getDims() - 1, dObjImages->getAxisUnit(dObjImages->getDims() - 2, check)); 

        destTemp.setAxisOffset(destTemp.getDims() - 2, dObjImages->getAxisOffset(dObjImages->getDims() - 1)); 
        destTemp.setAxisOffset(destTemp.getDims() - 1, dObjImages->getAxisOffset(dObjImages->getDims() - 2)); 

        destTemp.setAxisScale(destTemp.getDims() - 2, dObjImages->getAxisOffset(dObjImages->getDims() - 1)); 
        destTemp.setAxisScale(destTemp.getDims() - 1, dObjImages->getAxisOffset(dObjImages->getDims() - 2)); 

        if (overWrite)
        {
            *dObjDst = destTemp;
        }

        QString msg = rotClw ? tr("Rotated object by 90° clockwise with cvRotateM90-Filter") : tr("Rotated object by 90° counter clockwise with cvRotateP90-Filter");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvRot180Doc = QObject::tr("This filter rotates the image by 180°. \n\
\n\
This filter applies the flip method cvFlip from OpenCV horizontally and vertically to rotate the object. The \
result is contained in the destination object\n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated.\n");

ito::RetVal OpenCVFilters::cvRot180(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::DataObject destTemp;

    bool overWrite = true;

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet, only 2D and 3D.").toLatin1().data());
    }

    if (dObjImages->getDims() == 0)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: input object must not be empty.").toLatin1().data());
    }

    int ysize = dObjImages->getSize(dObjImages->getDims() - 2);
    int xsize = dObjImages->getSize(dObjImages->getDims() - 1);
    int planes = 0;
    if (dObjImages->getDims() > 1)
    {
        planes = dObjImages->getSize(dObjImages->getDims() - 3);
    }

    if (!retval.containsError())
    {
        retval += ito::dObjHelper::verifyDataObjectType(dObjImages, "srcImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    if (!retval.containsError())
    {
        if (dObjDst != dObjImages)
        {
            if (planes > 0)
            {
                ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjDst, "destImage", planes, planes, ysize, ysize, xsize, xsize,  1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    int sizes[3] = {planes, ysize, xsize};
                    destTemp = ito::DataObject(3, sizes, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }
            }
            else
            {
                ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjDst, "destImage", ysize, ysize, xsize, xsize,  1, dObjImages->getType());
                if (tRetval.containsError())
                {
                    destTemp = ito::DataObject(ysize, xsize, dObjImages->getType());
                }
                else
                {
                    destTemp = *dObjDst;
                    overWrite = false;
                }            
            }
        }
        else
        {
            //destDataPhase = ito::DataObject( ysize, xsize, ito::tFloat64);
            destTemp = *dObjDst;
            overWrite = false;
        }
    }

    if (!retval.containsError())
    {
        int z_length = dObjImages->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjImages->get_mdata()[dObjImages->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[destTemp.seekMat(z)];
            try
            {
                cv::flip(*cvMatIn, *cvMatOut, 1);
                cv::flip(*cvMatOut, *cvMatOut, 0);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }

    if (!retval.containsError())
    {
        if (overWrite)
        {
            *dObjDst = destTemp;
        }

        if (dObjDst != dObjImages)
        {
            dObjImages->copyAxisTagsTo(*dObjDst);
            dObjImages->copyTagMapTo(*dObjDst);
        }

        QString msg = tr("Rotated object by 180° using cvRotate180-Filter");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvRemoveSpikesDoc = QObject::tr("Set single spikes at measurement edges to a new value. \n\
\n\
This filter creates a binary mask for the input object. The value of mask(y,x) will be 1 if value of input(y,x) is within the specified range and is finite.\
The mask is eroded and than dilated by kernel size using openCV cv::erode and cv::dilate with a single iteration. \
In the last step the value of output(y,x) is set to newValue if mask(y,x) is 0.\n\
\n\
It is allowed to let the filter work inplace if you give the same source and destination data object, else the destination data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated and the input data is copied to the new object. \n\
");

ito::RetVal OpenCVFilters::cvRemoveSpikesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "32 or 64 bit floating point input image") );
    paramsMand->append( ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "32 or 64 bit floating point output image") );

    paramsOpt->append( ito::Param("kernelSize", ito::ParamBase::Int | ito::ParamBase::In, 5, new ito::IntMeta(3, 13), "N defines the N x N kernel") );
    paramsOpt->append( ito::Param("lowestValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, "Lowest value to consider as valid") );
    paramsOpt->append( ito::Param("highestValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 1.0, "Highest value to consider as valid") );
    paramsOpt->append( ito::Param("newValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::quiet_NaN(), "Replacement value for spike elements") );
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvRemoveSpikes(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError,0, tr("sourceObject must not be NULL").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError,0, tr("destinationObject must not be NULL").toLatin1().data());
    }

    int dims = dObjSrc->getDims();

    if (dims != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("sourceObject is not a matrix or image stack").toLatin1().data());
    }

    //kernelSize
    int kernel = (*paramsOpt)[0].getVal<int>();

    if (!(kernel % 2))
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel must be odd").toLatin1().data());
    }

    //anchor
    cv::Point anchor(-1, -1);

    // Check if input type is allowed or not
    retval += ito::dObjHelper::verify2DDataObject(dObjSrc, "sourceObject", kernel, std::numeric_limits<ito::uint32>::max(), kernel, std::numeric_limits<ito::uint32>::max(), 2, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
    {
        return retval;
    }

    //create structuring element
    cv::Mat cvElement = cv::Mat::ones(kernel, kernel, CV_8U);

    ito::float64 minClipVal = (*paramsOpt)[1].getVal<ito::float64>();
    ito::float64 maxClipVal = (*paramsOpt)[2].getVal<ito::float64>();
    ito::float64 newVal = (*paramsOpt)[3].getVal<ito::float64>();

    //dObjDst is either equal to dObjSrc or must have the same size and type than dObjSrc (if not is created such it fullfills these requirements)
    if (dObjDst != dObjSrc)
    {
        int dstDim = dObjDst->getDims();
        int dstType = dObjDst->getType();
        int *sizes = new int[dims];
        bool sizeFit = true;

        for (int i = 0; i < dims; ++i)
        {
            sizes[i] = dObjSrc->getSize(i);
            if (dstDim != dims || sizes[i] != dObjDst->getSize(i))
            {
                sizeFit = false;
            }
        }

        if (dstDim != dims || sizeFit == false || dstType != dObjSrc->getType())
        {
            (*dObjDst) = ito::DataObject(dims, sizes, dObjSrc->getType());
        }

        delete[] sizes;


        dObjSrc->copyTo(*dObjDst);
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    cv::Mat *cvMatIn = ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(0)]);
    cv::Mat *cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(0)]);

    cv::Mat cvTemp = cv::Mat::zeros(cvMatIn->rows, cvMatIn->cols, CV_8U);

    #if (useomp)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    ito::uint8 *tmpPtr = NULL;
    if (dObjSrc->getType() == ito::tFloat64)
    {
        ito::float64 *srcPtr = NULL;

        #if (useomp)
        #pragma omp for schedule(guided)
        #endif

        for(int y = 0; y < cvMatIn->rows; y++)
        {
            srcPtr = cvMatIn->ptr<ito::float64>(y);
            tmpPtr = cvTemp.ptr<ito::uint8>(y);
            for(int x = 0; x < cvMatIn->cols; x++)
            {
                if (ito::isFinite(srcPtr[x]) && srcPtr[x] > minClipVal && srcPtr[x] < maxClipVal)
                {
                    tmpPtr[x] = 1;
                }
            }
        }
    }
    else
    {
        ito::float32 minClipValf = cv::saturate_cast<ito::float32>(minClipVal);
        ito::float32 maxClipValf = cv::saturate_cast<ito::float32>(maxClipVal);

        ito::float32 *srcPtr = NULL;

        #if (useomp)
        #pragma omp for schedule(guided)
        #endif
        for(int y = 0; y < cvMatIn->rows; y++)
        {
            srcPtr = cvMatIn->ptr<ito::float32>(y);
            tmpPtr = cvTemp.ptr<ito::uint8>(y);
            for(int x = 0; x < cvMatIn->cols; x++)
            {
                if (ito::isFinite(srcPtr[x]) && srcPtr[x] > minClipValf && srcPtr[x] < maxClipValf)
                {
                    tmpPtr[x] = 1;
                }
            }
        }
    }

    #if (useomp)
    }
    #endif

    if (!retval.containsError())
    {
        try
        {
            cv::morphologyEx(cvTemp, cvTemp, cv::MORPH_OPEN, cvElement, anchor, 1, cv::BORDER_CONSTANT);
            //cv::erode(cvTemp, cvTemp, cvElement, anchor, 1, cv::BORDER_CONSTANT);
            //cv::dilate(cvTemp, cvTemp, cvElement, anchor, 1, cv::BORDER_CONSTANT);
        }
        catch (cv::Exception &exc)
        {
            retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
        }
    }

    if (!retval.containsError())
    {
        #if (useomp)
        #pragma omp parallel num_threads(NTHREADS)
        {
        #endif

        ito::uint8 *tmpPtr = NULL;
        if (dObjSrc->getType() == ito::tFloat64)
        {
            ito::float64 *dstPtr = NULL;
            #if (useomp)
            #pragma omp for schedule(guided)
            #endif

            for(int y = 0; y < cvMatIn->rows; y++)
            {
                dstPtr = cvMatOut->ptr<ito::float64>(y);
                tmpPtr = cvTemp.ptr<ito::uint8>(y);
                for(int x = 0; x < cvMatIn->cols; x++)
                {
                    if (tmpPtr[x] == 0)
                    {
                        dstPtr[x] = newVal;
                    }
                }
            }
        }
        else
        {
            ito::float32 newValf = cv::saturate_cast<ito::float32>(newVal);

            ito::float32 *dstPtr = NULL;

            #if (useomp)
            #pragma omp for schedule(guided)
            #endif
            for(int y = 0; y < cvMatIn->rows; y++)
            {
                dstPtr = cvMatOut->ptr<ito::float32>(y);
                tmpPtr = cvTemp.ptr<ito::uint8>(y);
                for(int x = 0; x < cvMatIn->cols; x++)
                {
                    if (tmpPtr[x] == 0)
                    {
                        dstPtr[x] = newValf;
                    }
                }
            }
        }

        #if (useomp)
        }
        #endif
    }

    if (!retval.containsError())
    {

        QString msg;
        msg = tr("Spike removal filter with kernel(%1, %1) and range ]%2, %3[").arg(kernel).arg(minClipVal).arg(maxClipVal);

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvSplitChannelsDoc = QObject::tr("Converts a rgba32 data object (with four channels blue, green, red, alpha) into \n\
an output data object of type 'uint8' and a shape that has one dimension more than the input object and the first dimension is equal to 4. \n\
The four color components are then distributed into the 4 planes of the first dimension. \n\
\n\
For instance a 4x5x3, rgba32 data objects leads to a 4x4x5x3 uint8 data object.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvSplitChannelsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("rgbaObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "rgba32 data object with any shape") );
    paramsMand->append( ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "uint8 data object with new shape [4,shape] where shape is the original shape. The inserted 4 dimensions represent the color components (b,g,r,alpha) of the source object.") );
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvSplitChannels(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;
    const ito::DataObject *rgbaObject = paramsMand->at(0).getVal<const ito::DataObject*>();

    if (!rgbaObject || rgbaObject->getType() != ito::tRGBA32)
    {
        retVal += ito::RetVal(ito::retError, 0, "rgbaObject must be of type 'rgba32'");
    }

    if (!retVal.containsError())
    {
        if (rgbaObject->getDims() > 0)
        {
            int dims = rgbaObject->getDims();
            int planes = rgbaObject->calcNumMats();
            int *newShape = new int[dims+1];
            newShape[0] = 4;
            for (int i = 0; i < dims; ++i)
            {
                newShape[i+1] = rgbaObject->getSize(i);
            }

            cv::Mat *newMats = new cv::Mat[4*planes];

            for (int i = 0; i < planes; ++i)
            {
                std::vector<cv::Mat> channels;
                channels.resize(4);
                cv::split(*(rgbaObject->getCvPlaneMat(i)), channels);
                newMats[i] = channels[0];
                newMats[i+planes] = channels[1];
                newMats[i+2*planes] = channels[2];
                newMats[i+3*planes] = channels[3];
            }

            ito::DataObject output(dims + 1, newShape, ito::tUInt8, newMats, 4*planes);
            *(paramsMand->at(1).getVal<ito::DataObject*>()) = output;

            delete[] newMats;
            delete[] newShape;
        }
        else
        {
            *(paramsMand->at(1).getVal<ito::DataObject*>()) = ito::DataObject();
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvMergeChannelsDoc = QObject::tr("Reduces a [4x...xMxN] or [3x...xMxN] uint8 data object to a [...xMxN] rgba32 data object where the \n\
first dimension is merged into the color type. If the first dimension is equal to 4, the planes are used for the blue, green, red and alpha \n\
component, in case of three, the alpha component is set to the optional alpha value.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMergeChannelsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append( ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "uint8 data object with any shape and at least three dimensions") );
    paramsMand->append( ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "rgba32 data object") );

    paramsOpt->append( ito::Param("alpha", ito::ParamBase::Int, 0, 255, 255, "if the first dimension of the inputObject is 3, this alpha value is used for all alpha components in the output object"));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMergeChannels(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;
    const ito::DataObject *inputObject = paramsMand->at(0).getVal<const ito::DataObject*>();

    if (!inputObject || inputObject->getType() != ito::tUInt8 || inputObject->getDims() < 3)
    {
        retVal += ito::RetVal(ito::retError, 0, "rgbaObject must be of type 'uint8' and have at least 3 dimensions.");
    }

    if (!retVal.containsError())
    {
        {
            int dims = inputObject->getDims();
            cv::Size planeSize = inputObject->getCvPlaneMat(0)->size();
            int *newShape = new int[dims-1];
            for (int i = 1; i < dims; ++i)
            {
                newShape[i-1] = inputObject->getSize(i);
            }

            int components = inputObject->getSize(0);
            int newPlanes = inputObject->calcNumMats() / components;

            cv::Mat *newMats = new cv::Mat[newPlanes];
            cv::Mat alpha;
            
            if (components < 4)
            {
                alpha.create(planeSize, CV_8UC1);
                alpha.setTo(paramsOpt->at(0).getVal<int>());
            }

            for (int i = 0; i < newPlanes; ++i)
            {
                cv::Mat output(planeSize, CV_8UC4);
                std::vector<cv::Mat> colors;
                colors.push_back( *(inputObject->getCvPlaneMat(i)) );
                colors.push_back( *(inputObject->getCvPlaneMat(i+newPlanes)) );
                colors.push_back( *(inputObject->getCvPlaneMat(i+2*newPlanes)) );

                if (components == 3)
                {
                    colors.push_back(alpha);
                }
                else
                {
                    colors.push_back(*(inputObject->getCvPlaneMat(i+3*newPlanes)) );
                }

                cv::merge(colors, output);
                output.convertTo(newMats[i], cv::DataType<ito::Rgba32>::type);
            }

            ito::DataObject output(dims - 1, newShape, ito::tRGBA32, newMats, newPlanes);
            *(paramsMand->at(1).getVal<ito::DataObject*>()) = output;

            delete[] newMats;
            delete[] newShape;
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvResizeDoc = QObject::tr("Resizes an image \n\
\n\
The function resize resizes the image 'inputObject' down to or up by the specific factors. \n\
\n\
To shrink an image, it will generally look best with CV_INTER_AREA interpolation, whereas to enlarge an image, \n\
it will generally look best with CV_INTER_CUBIC (slow) or CV_INTER_LINEAR (faster but still looks OK). \n\
The axisScale properties of the x- and y-axes of the outputObject are divided by fx and fy respectively, while the offset values are multiplied with fx and fy.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvResizeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input image (2D after an optional squeeze operation)") );
    paramsMand->append( ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output image, will have the same type than inputObject. Its size corresponds to the size of the input object multiplied with fx and fy respectively.") );

    paramsMand->append( ito::Param("fx", ito::ParamBase::Double, 1e-6, std::numeric_limits<double>::max(), 1.0, "scale factor along the horizontal axis."));
    paramsMand->append( ito::Param("fy", ito::ParamBase::Double, 1e-6, std::numeric_limits<double>::max(), 1.0, "scale factor along the vertical axis."));

    QString description = "Interpolation method. The following values are possible:\n\n";
    description += QString("INTER_NEAREST (%1)\n").arg(cv::INTER_NEAREST);
    description += QString("INTER_LINEAR (%1)\n").arg(cv::INTER_LINEAR);
    description += QString("INTER_AREA (%1)\n").arg(cv::INTER_AREA);
    description += QString("INTER_CUBIC (%1)\n").arg(cv::INTER_CUBIC);
    description += QString("INTER_LANCZOS4 (%1)\n").arg(cv::INTER_LANCZOS4 );
    paramsOpt->append( ito::Param("interpolation", ito::ParamBase::Int | ito::ParamBase::In, 0, cv::INTER_LANCZOS4, cv::INTER_LINEAR, description.toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvResize(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(),"source", ito::Range(1,INT_MAX), ito::Range(1,INT_MAX), retval, -1, 0);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, "destination is empty");
    }

    int interpolation = paramsOpt->at(0).getVal<int>();
    double fx = paramsMand->at(2).getVal<double>();
    double fy = paramsMand->at(3).getVal<double>();

    if (!retval.containsError())
    {
        cv::Mat dst;
        
        try
        {
            cv::resize(*(src.getCvPlaneMat(0)), dst, cv::Size(), fx, fy, interpolation);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }

        if (!retval.containsError())
        {
            ito::DataObject *dst = (*paramsMand)[1].getVal<ito::DataObject*>();
            dst->setAxisScale(0, src.getAxisScale(0) / fy);
            dst->setAxisScale(1, src.getAxisScale(1) / fx);
            dst->setAxisOffset(0, src.getAxisOffset(0) * fy);
            dst->setAxisOffset(1, src.getAxisOffset(1) * fx);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvBilateralFilterDoc = QObject::tr("Resizes an image \n\
\n\
The function resize resizes the image 'inputObject' down to or up by the specific factors. \n\
\n\
To shrink an image, it will generally look best with CV_INTER_AREA interpolation, whereas to enlarge an image, \n\
it will generally look best with CV_INTER_CUBIC (slow) or CV_INTER_LINEAR (faster but still looks OK). \n\
The axisScale properties of the x- and y-axes of the outputObject are divided by fx and fy respectively, while the offset values are multiplied with fx and fy.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvBilateralFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("inputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "input image (8-bit or floating-point, 1-Channel or 3-Channel)"));
    paramsMand->append(ito::Param("outputObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output image, will have the same type and size than inputObject."));

    paramsMand->append(ito::Param("diameter", ito::ParamBase::Int, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 1, "diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from sigmaSpace.."));
    paramsMand->append(ito::Param("sigmaColor", ito::ParamBase::Double, 1e-6, std::numeric_limits<double>::max(), 1.0, "Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger areas of semi-equal color."));
    paramsMand->append(ito::Param("sigmaSpace", ito::ParamBase::Double, 1e-6, std::numeric_limits<double>::max(), 1.0, "Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see sigmaColor ). When diameter>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, diameter is proportional to sigmaSpace.."));

    QString description = "border mode used to extrapolate pixels outside of the image. The following values are possible:\n\n";
    description += QString("BORDER_CONSTANT (%1) (iiiiii|abcdefgh|iiiiiii with some specified i)\n").arg(cv::BORDER_CONSTANT);
    description += QString("BORDER_REPLICATE (%1) (aaaaaa|abcdefgh|hhhhhhh)\n").arg(cv::BORDER_REPLICATE);
    description += QString("BORDER_REFLECT  (%1) (fedcba|abcdefgh|hgfedcb)\n").arg(cv::BORDER_REFLECT);
    description += QString("BORDER_WRAP (%1) (cdefgh|abcdefgh|abcdefg)\n").arg(cv::BORDER_WRAP);
    description += QString("BORDER_TRANSPARENT (%1) (gfedcb|abcdefgh|gfedcba)\n").arg(cv::BORDER_REFLECT_101);
    description += QString("BORDER_ISOLATED (%1) (do not look outside of ROI)\n").arg(cv::BORDER_DEFAULT);
    paramsOpt->append(ito::Param("borderType", ito::ParamBase::Int | ito::ParamBase::In, cv::BORDER_DEFAULT, cv::BORDER_CONSTANT, cv::BORDER_DEFAULT, description.toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvBilateralFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::DataObject dObj = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "inputObject", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 2, ito::tUInt8, ito::tFloat32);

    if (!paramsMand->at(1).getVal<ito::DataObject*>())
    {
        retval += ito::RetVal(ito::retError, 0, "outputObject is empty");
    }

    int diameter = paramsMand->at(2).getVal<int>();
    double sigmaColor = paramsMand->at(3).getVal<double>();
    double sigmaSpace = paramsMand->at(4).getVal<double>();
    int border = paramsOpt->at(0).getVal<int>();

    if (!retval.containsError())
    {
        cv::Mat input = *(dObj.getCvPlaneMat(0));

        int type = input.type();
        cv::Mat dst;

        try
        {
            cv::bilateralFilter(input, dst, diameter, sigmaColor, sigmaSpace, border);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            retval += itomcv::setOutputArrayToDataObject((*paramsMand)[1], &dst);
        }
    }

    return retval;
}
        
//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvCannyEdgeDoc = QObject::tr("Canny Edge detector using cv::DFT.\n\
\n\
It's just Canny's edge filter\n");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvCannyEdgeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Input Object handle, must be a single plane").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationImage", ito::ParamBase::DObjPtr, NULL, tr("Output Object handle. Will be come complex-type").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("lowThreshold", ito::ParamBase::Double, -1.0e10, 1.0e10, 2.0, tr("Low Threshold").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("highThresholdRatio", ito::ParamBase::Double, 0.0, 1.0e10, 3.0, tr("Ratio between High Threshold and Low Threshold, Canny's recommendation is three").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("kernelSize", ito::ParamBase::Int, 3, 300, 3, tr("Kernel size for Sobel filter, default is 3").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvCannyEdge(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjInp = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjOutp = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::float64  lowThres = (*paramsOpt)[0].getVal<ito::float64>();
    ito::float64  highThresRatio = (*paramsOpt)[1].getVal<ito::float64>();
    ito::int16 kernelSize = (*paramsOpt)[2].getVal<ito::float64>();

    ito::DataObject destTemp;

    bool overWrite = true;

    if (!dObjInp)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjOutp)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjInp->getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet, only 2D and 3D.").toLatin1().data());
    }

    int ysize = dObjInp->getSize(dObjInp->getDims() - 2);
    int xsize = dObjInp->getSize(dObjInp->getDims() - 1);
    int planes = 0;
    if (dObjInp->getDims() > 1)
    {
        planes = dObjInp->getSize(dObjInp->getDims() - 3);
    }

    if (!retval.containsError())
    {
        retval += ito::dObjHelper::verifyDataObjectType(dObjInp, "srcImage", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    }

    if (!retval.containsError())
    {
        if (dObjOutp != dObjInp)
        {
            if (planes > 0)
            {
                ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjOutp, "destImage", planes, planes, ysize, ysize, xsize, xsize, 1, dObjInp->getType());
                if (tRetval.containsError())
                {
                    int sizes[3] = { planes, ysize, xsize };
                    destTemp = ito::DataObject(3, sizes, dObjInp->getType());
                }
                else
                {
                    destTemp = *dObjOutp;
                    overWrite = false;
                }
            }
            else
            {
                ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjOutp, "destImage", ysize, ysize, xsize, xsize, 1, dObjInp->getType());
                if (tRetval.containsError())
                {
                    destTemp = ito::DataObject(ysize, xsize, dObjInp->getType());
                }
                else
                {
                    destTemp = *dObjOutp;
                    overWrite = false;
                }
            }
        }
        else
        {
            //destDataPhase = ito::DataObject( ysize, xsize, ito::tFloat64);
            destTemp = *dObjOutp;
            overWrite = false;
        }
    }

    if (!retval.containsError())
    {
        int z_length = dObjInp->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjInp->get_mdata()[dObjInp->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[destTemp.seekMat(z)];
            try
            {
                cv::Canny(*cvMatIn, *cvMatOut, lowThres, lowThres * highThresRatio, kernelSize);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
        }
    }

    if (!retval.containsError())
    {
        if (overWrite)
        {
            *dObjOutp = destTemp;
        }

        if (dObjOutp != dObjInp)
        {
            dObjInp->copyAxisTagsTo(*dObjOutp);
            dObjInp->copyTagMapTo(*dObjOutp);
        }

        QString msg = tr("Canny edge filter");
        dObjOutp->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString OpenCVFilters::cvCvtColorDoc = QObject::tr("Converts an image from one color space to another.\n\
In case of linear transformations, the range does not matter. But in case of a non-linear transformation,\n\
an input RGB image should be normalized to the proper value range to get the correct results, for example,\n\
for RGB -> L*u*v* transformation. For example, if you have a 32-bit floating-point image directly\n\
converted from an 8-bit image without any scaling, then it will have the 0..255 value range instead of 0..1\n\
assumed by the function. So, before calling cvtColor , you need first to scale the image down\n\
\n\
The parameter code defines the conversion:\n\
\n\
* RGB <-> GRAY ( CV_BGR2GRAY = 6, CV_RGB2GRAY = 7 , CV_GRAY2BGR = 8, CV_GRAY2RGB = 8)\n\
* RGB <-> CIE XYZ.Rec 709 with D65 white point ( CV_BGR2XYZ = 32, CV_RGB2XYZ = 33, CV_XYZ2BGR = 34, CV_XYZ2RGB = 35)\n\
* RGB <-> YCrCb JPEG (or YCC) ( CV_BGR2YCrCb = 36, CV_RGB2YCrCb = 37, CV_YCrCb2BGR = 38, CV_YCrCb2RGB = 39)\n\
* RGB <-> HSV ( CV_BGR2HSV = 40, CV_RGB2HSV = 41, CV_HSV2BGR = 54, CV_HSV2RGB = 55 )\n\
* RGB <-> HLS ( CV_BGR2HLS = 52, CV_RGB2HLS = 53, CV_HLS2BGR = 60, CV_HLS2RGB = 61)\n\
* RGB <-> CIE L*a*b* ( CV_BGR2Lab = 44, CV_RGB2Lab = 45, CV_Lab2BGR = 56, CV_Lab2RGB = 57)\n\
* RGB <-> CIE L*u*v* ( CV_BGR2Luv = 50, CV_RGB2Luv = 51, CV_Luv2BGR = 58, CV_Luv2RGB = 59)\n\
* Bayer <-> RGB ( CV_BayerBG2BGR = 46, CV_BayerGB2BGR = 47, CV_BayerRG2BGR = 48, CV_BayerGR2BGR = 49, ...\n\
    CV_BayerBG2RGB = 48, CV_BayerGB2RGB = 49, CV_BayerRG2RGB = 46, CV_BayerGR2RGB = 47)\n\
\n\
For more details see OpenCV documentation.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvCvtColorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Input Object handle, must be a single plane").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output Object handle. Will be come complex-type").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("code", ito::ParamBase::Int | ito::ParamBase::In, 0, 65535, 0, tr("Transformation code, see (OpenCV) documentation").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("dstChan", ito::ParamBase::Int | ito::ParamBase::In, 0, 5, 0, tr("number of color channels of destination image, for 0 the number of channels is derived from the transformation (default)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvCvtColor(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjInp = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjOutp = (*paramsMand)[1].getVal<ito::DataObject*>();

    ito::uint16 code = (*paramsOpt)[0].getVal<int>();
    ito::uint8  numChan = (*paramsOpt)[1].getVal<int>();

    ito::DataObject destTemp;

    bool overWrite = true;

    if (!dObjInp)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjOutp)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjInp->getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet, only 2D and 3D.").toLatin1().data());
    }

    int ysize = dObjInp->getSize(dObjInp->getDims() - 2);
    int xsize = dObjInp->getSize(dObjInp->getDims() - 1);
    int planes = 0;
    if (dObjInp->getDims() > 1)
    {
        planes = dObjInp->getSize(dObjInp->getDims() - 3);
    }

    if (!retval.containsError())
    {
        retval += ito::dObjHelper::verifyDataObjectType(dObjInp, "srcImage", 8, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64, ito::tRGBA32);
    }

    if (!retval.containsError())
    {
        if (dObjOutp != dObjInp)
        {
            if (planes > 0)
            {
                ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjOutp, "destImage", planes, planes, ysize, ysize, xsize, xsize, 1, dObjInp->getType());
                if (tRetval.containsError())
                {
                    int sizes[3] = { planes, ysize, xsize };
                    destTemp = ito::DataObject(3, sizes, dObjInp->getType());
                }
                else
                {
                    destTemp = *dObjOutp;
//                    overWrite = false;
                }
            }
            else
            {
                ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjOutp, "destImage", ysize, ysize, xsize, xsize, 1, dObjInp->getType());
                if (tRetval.containsError())
                {
                    destTemp = ito::DataObject(ysize, xsize, dObjInp->getType());
                }
                else
                {
                    destTemp = *dObjOutp;
//                    overWrite = false;
                }
            }
        }
        else
        {
            //destDataPhase = ito::DataObject( ysize, xsize, ito::tFloat64);
            destTemp = *dObjOutp;
//            overWrite = false;
        }
    }

    if (!retval.containsError())
    {
        int z_length = dObjInp->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;
        int outType, inType;
        int outChan, inChan;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjInp->get_mdata()[dObjInp->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[destTemp.seekMat(z)];
            try
            {
                cv::cvtColor(*cvMatIn, *cvMatOut, code, numChan);
            }
            catch (cv::Exception &exc)
            {
                retval += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
                break;
            }
            inType = cvMatIn->type();
            inChan = cvMatIn->channels();
            outType = cvMatOut->type();
            outChan = cvMatOut->channels();
        }

        // conversion changed data type, so we need to adapt the output dataObject
        if (inType != outType || inChan != outChan)
        {
            int *newSizes = (int*)calloc(destTemp.getDims(), sizeof(int));
            for (int ns = 0; ns < destTemp.getDims(); ns++)
            {
                newSizes[ns] = destTemp.getSize(ns);
            }
            // do we really need this?
            newSizes[destTemp.getDims() - 1] = newSizes[destTemp.getDims() - 1] * outChan;
            switch (outType)
            {
                case CV_8U:
                    destTemp = ito::DataObject(destTemp.getDims(), newSizes, ito::tUInt8, *destTemp.get_mdata(), destTemp.getNumPlanes());
                break;

                case CV_16U:
                    destTemp = ito::DataObject(destTemp.getDims(), newSizes, ito::tUInt16, *destTemp.get_mdata(), destTemp.getNumPlanes());
                break;

                case CV_32F:
                    destTemp = ito::DataObject(destTemp.getDims(), newSizes, ito::tFloat32, *destTemp.get_mdata(), destTemp.getNumPlanes());
                break;
            }
            free(newSizes);
        }
    }

    if (!retval.containsError())
    {
        if (overWrite)
        {
            *dObjOutp = destTemp;
        }

        if (dObjOutp != dObjInp)
        {
            dObjInp->copyAxisTagsTo(*dObjOutp);
            dObjInp->copyTagMapTo(*dObjOutp);
        }

        QString msg = tr("CvtColor conversion with code: %1").arg(code);
        dObjOutp->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString OpenCVFilters::cvThresholdDoc = QObject::tr("Applies a fixed-level threshold to each array element.. \n\
\n\
The function applies fixed-level thresholding to a multiple-channel array. \n\
The function is typically used to get a bi-level (binary) image out of a grayscale image (compare could be also used for this purpose)\n\
or for removing a noise, that is, filtering out pixels with too small or too large values. \n\
There are several types of thresholding supported by the function. They are determined by type parameter.\n\
\n\
Also, the special values THRESH_OTSU or THRESH_TRIANGLE may be combined with one of the above values. \n\
In these cases, the function determines the optimal threshold value using the Otsu's or Triangle algorithm and uses it instead of the specified thresh.\n\
\n\
Note: \n\
Currently, the Otsu's and Triangle methods are implemented only for 8-bit single-channel images.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append(ito::Param("source", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("source image").toLatin1().data()));
    paramsMand->append(ito::Param("destination", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("destination image. It hast the same size as map1 and the same type as src.").toLatin1().data()));

    paramsMand->append(ito::Param("threshold", ito::ParamBase::Double | ito::ParamBase::In, std::numeric_limits<double>::min(), std::numeric_limits<double>::max(), 0.0, tr("threshold value.").toLatin1().data()));
    paramsMand->append(ito::Param("maxValue", ito::ParamBase::Double | ito::ParamBase::In, std::numeric_limits<double>::min(), std::numeric_limits<double>::max(), 0.0, tr("maximum value to use with the THRESH_BINARY and THRESH_BINARY_INV thresholding types.").toLatin1().data()));

    QString description = tr("threshold type\n").toLatin1().data();
    description += QString("THRESH_BINARY (%1)\n").arg(cv::THRESH_BINARY);
    description += QString("THRESH_BINARY_INV (%1)\n").arg(cv::THRESH_BINARY_INV);
    description += QString("THRESH_TRUNC (%1)\n").arg(cv::THRESH_TRUNC);
    description += QString("THRESH_TOZERO (%1)\n").arg(cv::THRESH_TOZERO);
    description += QString("THRESH_TOZERO_INV (%1)\n").arg(cv::THRESH_TOZERO_INV);
    description += QString("THRESH_MASK (%1)\n").arg(cv::THRESH_MASK);
    description += QString("THRESH_OTSU (%1)\n").arg(cv::THRESH_OTSU);
    description += QString("THRESH_TRIANGLE (%1)").arg(cv::THRESH_TRIANGLE);
    paramsMand->append(ito::Param("type", ito::ParamBase::Int | ito::ParamBase::In, cv::THRESH_BINARY, cv::THRESH_TRIANGLE, cv::THRESH_BINARY, description.toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject src = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "source", ito::Range(1, INT_MAX), ito::Range(1, INT_MAX), retval, -1, 0);
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    double threshold = (*paramsMand)[2].getVal<double>();
    double maxVal = (*paramsMand)[3].getVal<double>();
    int type = (*paramsMand)[4].getVal<int>();


    if (src.getDims() > 3)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: input dataObject must be two-dimensional.").toLatin1().data());
    }


    if (!retval.containsError())
    {
        
        try
        {
            (*dObjDst) = ito::DataObject(src.getDims(), src.getSize(), src.getType());

            cv::Mat *cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(0)]);

            cv::threshold(*(src).getCvPlaneMat(0), *cvMatOut, threshold, maxVal, type);

        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str());
        }

        if (!retval.containsError())
        {
            dObjDst->copyAxisTagsTo(src);
            dObjDst->copyTagMapTo(src);

            QString msg = tr("Image has been threshold filter by a value of %1").arg(threshold);
            dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
        }

    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(OpenCVFilters::cvDilate, OpenCVFilters::cvDilateErodeParams, cvDilateDoc);
    m_filterList.insert("cvDilate", filter);

    filter = new FilterDef(OpenCVFilters::cvErode, OpenCVFilters::cvDilateErodeParams, cvErodeDoc);
    m_filterList.insert("cvErode", filter);

    filter = new FilterDef(OpenCVFilters::cvMorphologyEx, OpenCVFilters::cvMorphologyExParams, cvMorphologyExDoc);
    m_filterList.insert("cvMorphologyEx", filter);

    filter = new FilterDef(OpenCVFilters::cvMedianBlur, OpenCVFilters::cvMedianBlurParams, cvMedianBlurDoc);
    m_filterList.insert("cvMedianBlur", filter);

    filter = new FilterDef(OpenCVFilters::cvBlur, OpenCVFilters::cvBlurParams, cvBlurDoc);
    m_filterList.insert("cvBlur", filter);

    filter = new FilterDef(OpenCVFilters::cvFFT2D, OpenCVFilters::cvFFTParams, cvFFT2DDoc);
    m_filterList.insert("cvFFT2D", filter);

    filter = new FilterDef(OpenCVFilters::cvIFFT2D, OpenCVFilters::cvFFTParams, cvIFFT2DDoc);
    m_filterList.insert("cvIFFT2D", filter);

    filter = new FilterDef(OpenCVFilters::cvFFT1D, OpenCVFilters::cvFFTParams, cvFFT1DDoc);
    m_filterList.insert("cvFFT1D", filter);

    filter = new FilterDef(OpenCVFilters::cvIFFT1D, OpenCVFilters::cvFFTParams, cvIFFT1DDoc);
    m_filterList.insert("cvIFFT1D", filter);

    filter = new FilterDef(OpenCVFilters::cvRemoveSpikes, OpenCVFilters::cvRemoveSpikesParams, cvRemoveSpikesDoc);
    m_filterList.insert("cvRemoveSpikes", filter);

    filter = new FilterDef(OpenCVFilters::cvSplitChannels, OpenCVFilters::cvSplitChannelsParams, cvSplitChannelsDoc);
    m_filterList.insert("cvSplitChannels", filter);

    filter = new FilterDef(OpenCVFilters::cvMergeChannels, OpenCVFilters::cvMergeChannelsParams, cvMergeChannelsDoc);
    m_filterList.insert("cvMergeChannels", filter);

    filter = new FilterDef(OpenCVFilters::cvResize, OpenCVFilters::cvResizeParams, cvResizeDoc);
    m_filterList.insert("cvResize", filter);

    filter = new FilterDef(OpenCVFilters::cvBilateralFilter, OpenCVFilters::cvBilateralFilterParams, cvBilateralFilterDoc);
    m_filterList.insert("cvBilateralFilter", filter);

    /*filter = new FilterDef(OpenCVFilters::cvCalcHist, OpenCVFilters::cvCalcHistParams, cvCalcHistDoc);
    m_filterList.insert("cvCalcHistogram", filter);*/

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    filter = new FilterDef(OpenCVFilters::cvFindCircles, OpenCVFilters::cvFindCirclesParams, cvFindCirclesDoc);
    m_filterList.insert("cvFindCircles", filter);

    filter = new FilterDef(OpenCVFilters::cvFindChessboardCorners, OpenCVFilters::cvFindChessboardCornersParams, cvFindChessboardCornersDoc);
    m_filterList.insert("cvFindChessboardCorners", filter);

    filter = new FilterDef(OpenCVFilters::cvCornerSubPix, OpenCVFilters::cvCornerSubPixParams, cvCornerSubPixDoc);
    m_filterList.insert("cvCornerSubPix", filter);

    filter = new FilterDef(OpenCVFilters::cvCalibrateCamera, OpenCVFilters::cvCalibrateCameraParams, cvCalibrateCameraDoc);
    m_filterList.insert("cvCalibrateCamera", filter);

    filter = new FilterDef(OpenCVFilters::cvDrawChessboardCorners, OpenCVFilters::cvDrawChessboardCornersParams, cvDrawChessboardCornersDoc);
    m_filterList.insert("cvDrawChessboardCorners", filter);

    filter = new FilterDef(OpenCVFilters::cvEstimateAffine3D, OpenCVFilters::cvEstimateAffine3DParams, cvEstimateAffine3DDoc);
    m_filterList.insert("cvEstimateAffine3DParams", filter);

    filter = new FilterDef(OpenCVFilters::cvUndistort, OpenCVFilters::cvUndistortParams, cvUndistortDoc);
    m_filterList.insert("cvUndistort", filter);

    filter = new FilterDef(OpenCVFilters::cvUndistortPoints, OpenCVFilters::cvUndistortPointsParams, cvUndistortPointsDoc);
    m_filterList.insert("cvUndistortPoints", filter);

    filter = new FilterDef(OpenCVFilters::cvInitUndistortRectifyMap, OpenCVFilters::cvInitUndistortRectifyMapParams, cvInitUndistortRectifyMapDoc);
    m_filterList.insert("cvInitUndistortRectifyMap", filter);

    filter = new FilterDef(OpenCVFilters::cvRemap, OpenCVFilters::cvRemapParams, cvRemapDoc);
    m_filterList.insert("cvRemap", filter);

    filter = new FilterDef(OpenCVFilters::cvFindHomography, OpenCVFilters::cvFindHomographyParams, cvFindHomographyDoc);
    m_filterList.insert("cvFindHomography", filter);

    filter = new FilterDef(OpenCVFilters::cvFindFundamentalMat, OpenCVFilters::cvFindFundamentalMatParams, cvFindFundamentalMatDoc);
    m_filterList.insert("cvFindFundamentalMat", filter);

    filter = new FilterDef(OpenCVFilters::cvComputeCorrespondEpilines, OpenCVFilters::cvComputeCorrespondEpilinesParams, cvComputeCorrespondEpilinesDoc);
    m_filterList.insert("cvComputeCorrespondEpilines", filter);

    filter = new FilterDef(OpenCVFilters::cvFlannBasedMatcher, OpenCVFilters::cvFlannBasedMatcherParams, cvFlannBasedMatcherDoc);
    m_filterList.insert("cvFlannBasedMatcher", filter);

    filter = new FilterDef(OpenCVFilters::cvDrawKeypoints, OpenCVFilters::cvDrawKeypointsParams, cvDrawKeypointsDoc);
    m_filterList.insert("cvDrawKeypoints", filter);

    filter = new FilterDef(OpenCVFilters::cvDrawMatcher, OpenCVFilters::cvDrawMatcherParams, cvDrawMatcherDoc);
    m_filterList.insert("cvDrawMatcher", filter);

    filter = new FilterDef(OpenCVFilters::cvWarpPerspective, OpenCVFilters::cvWarpPerspectiveParams, cvWarpPerspectiveDoc);
    m_filterList.insert("cvWarpPerspective", filter);

    filter = new FilterDef(OpenCVFilters::cvProjectPoints, OpenCVFilters::cvProjectPointsParams, cvProjectPointsDoc);
    m_filterList.insert("cvProjectPoints", filter);

    filter = new FilterDef(OpenCVFilters::cvThreshold, OpenCVFilters::cvThresholdParams, cvThresholdDoc);
    m_filterList.insert("cvThreshold", filter);

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    filter = new FilterDef(OpenCVFilters::cvFlipUpDown, OpenCVFilters::stdParams2Objects, cvFlipUpDownDoc);
    m_filterList.insert("cvFlipUpDown", filter);

    filter = new FilterDef(OpenCVFilters::cvFlipLeftRight, OpenCVFilters::stdParams2Objects, cvFlipLeftRightDoc);
    m_filterList.insert("cvFlipLeftRight", filter);

    filter = new FilterDef(OpenCVFilters::cvRotP90, OpenCVFilters::stdParams2Objects, cvRotP90Doc);
    m_filterList.insert("cvRotateP90", filter);

    filter = new FilterDef(OpenCVFilters::cvRotM90, OpenCVFilters::stdParams2Objects, cvRotM90Doc);
    m_filterList.insert("cvRotateM90", filter);

    filter = new FilterDef(OpenCVFilters::cvRot180, OpenCVFilters::stdParams2Objects, cvRot180Doc);
    m_filterList.insert("cvRotate180", filter);

    filter = new FilterDef(OpenCVFilters::cvCannyEdge, OpenCVFilters::cvCannyEdgeParams, cvCannyEdgeDoc);
    m_filterList.insert("cvCannyEdge", filter);

    filter = new FilterDef(OpenCVFilters::cvCvtColor, OpenCVFilters::cvCvtColorParams, cvCvtColorDoc);
    m_filterList.insert("cvCvtColor", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------