/* ********************************************************************
    Plugin "OpenCV-Filter" for itom software
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include <math.h>
#include "OpenCVFilters.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"

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
    OpenCVFilters* newInst = new OpenCVFilters();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);
    QList<QString> keyList = newInst->m_filterList.keys();
    for (int i = 0; i < newInst->m_filterList.size(); i++)
    {
        newInst->m_filterList[keyList[i]]->m_pBasePlugin = this;
    }

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((OpenCVFilters *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVFiltersInterface::OpenCVFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("OpenCV-Filters");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin provides wrappers for various OpenCV algorithms. These are for instance: \n\
- morphological filters (dilation, erosion) \n\
- image filtering (blur, median blur...) \n\
- 1d and 2d fft and ifft \n\
- histogram determination \n\
- feature detections (circles, chessboard corners...) \n\
\n\
This plugin not only requires access to the core library of OpenCV but also to further libraries like \
imgproc and calib3d.";

    m_description = QObject::tr("Wrapped algorithms from OpenCV");
    m_detaildescription = QObject::tr(docstring);
    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr("N.A.");

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
#if QT_VERSION < 0x05000
    Q_EXPORT_PLUGIN2(OpenCVFilter, OpenCVFiltersInterface)
#endif

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
    if(!retval.containsError())
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
    if(!retval.containsError())
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
    if(retval.containsError())
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
        retval += ito::RetVal::format(ito::retError,0,"border type %s is unknown", borderTypeStr.toLatin1().data());
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
            retval += ito::RetVal::format(ito::retError,0,"anchor must be in range [0,%i];[0,%i]", m-1, n-1);
            return retval;
        }
    }
    else if(paramsOpt->at(1).getLen() <= 0)
    {
        anchor = cv::Point(-1,-1);
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"anchor must have either 2 values or none");
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
const char* OpenCVFilters::cvDilateDoc = "Dilates every plane of a data object by using a specific structuring element. \n\
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
Dilation can be applied several times (parameter 'iterations').";
ito::RetVal OpenCVFilters::cvDilate(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return cvDilateErode(paramsMand, paramsOpt, paramsOut, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvErodeDoc = "Erodes every plane of a data object by using a specific structuring element. \n\
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
Erosion can be applied several times (parameter 'iterations').";
ito::RetVal OpenCVFilters::cvErode(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return cvDilateErode(paramsMand, paramsOpt, paramsOut, true);
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
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
const char* OpenCVFilters::cvBlurDoc = "Planewise median blur filter.\n\
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
Warning: NaN-handling for floats not verified.";
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
    if(retval.containsError())
        return retval;

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
    int itomtype = 0;

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
    itomtype = ito::dObjHelper::cvType2itomType(cvMatOut[0].type());
    if (itomtype > 0)
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
        retval += ito::RetVal(ito::retError, 0, tr("Unknown or unexpected CV-Datatype recived.").toLatin1().data());
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
#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvFindCirclesDoc = "Finds circles in a grayscale image using the Hough transform.\n\
\n\
This filter is a wrapper for the openCV-function cv::HoughCircles.\
he function finds circles in a grayscale image using a modification of the Hough transform.\
Based on this filter, circles are identified and located.\
The result is dataObject with rows = (n-Circles) and cols = (x,y,r).\n\
";
ito::RetVal OpenCVFilters::cvFindCirclesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        // Mandatory Parameters
        ito::Param param = ito::Param("Image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Must be 8bit").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("Circles", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("").toLatin1().data());
        paramsMand->append(param);

        // Optional Parameters
        param = ito::Param("dp", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 100.0, 1.0, tr("dp: Inverse ratio of the accumulator resolution to the image resolution.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("Min Distance", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100000.0, 20.0, tr("Minimum center distance of the circles.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("threshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 255.0, 200.0, tr("The higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("accumulator threshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 255.0, 100.0, tr("The accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("Min Radius [px]", ito::ParamBase::Int | ito::ParamBase::In, 1, 2048, 0, tr("Min Radius in x/y").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("Max Radius [px]", ito::ParamBase::Int | ito::ParamBase::In, 1, 2048, 0, tr("Max Radius in x/y").toLatin1().data());
        paramsOpt->append(param);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFindCircles(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
#if TIMEBENCHMARK
    int64 teststart = cv::getTickCount();
#endif

    ito::RetVal retval = ito::retOk;

    // Initialize pointers to the input and output dataObjects
    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    // Check if source dataObject exists
    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    // Check if destination dataObject exists
    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    // Check if source dataObject is an image (2D)
    if (dObjImages->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not an image").toLatin1().data());
    }

    double dp = (*paramsOpt)[0].getVal<double>();
    double MinDist = (*paramsOpt)[1].getVal<double>();
    double Threshold = (*paramsOpt)[2].getVal<double>();
    double AccThreshold = (*paramsOpt)[3].getVal<double>();
    int MinRadius = (*paramsOpt)[4].getVal<int>();
    int MaxRadius = (*paramsOpt)[5].getVal<int>();

    // Copy input image to a cv Mat
    cv::Mat *cvplaneIn = NULL;
    cvplaneIn = (cv::Mat_<unsigned char> *)(dObjImages->get_mdata())[0];

    // Declare the output vector to hold the circle coordinates and radii
    cv::vector<cv::Vec3f> circles;

    /*    void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
        dp – Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
        param1 – First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
        param2 – Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.*/
    cv::HoughCircles(*cvplaneIn, circles, CV_HOUGH_GRADIENT, dp, MinDist, Threshold, AccThreshold, MinRadius, MaxRadius);


    // Copy the circles into the output dataObject
    if(circles.size() > 0)
    {
        int sizes[2] = {(int)circles.size(), 3};
        *dObjDst = ito::DataObject(2, sizes, ito::tFloat32);
        ito::float32 *rowPtr = NULL;

        for( size_t i = 0; i < circles.size(); i++ )
        {
            //    This can be used to draw the circles directly into the input image
            /*    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                circle( *cvplaneIn, center, radius, 70, 3, 8, 0 );*/

            rowPtr = (ito::float32*)dObjDst->rowPtr(0, i);
            rowPtr[0] = circles[i][0];
            rowPtr[1] = circles[i][1];
            rowPtr[2] = circles[i][2];
        }
    }

#if TIMEBENCHMARK
    int64 testend = cv::getTickCount() - teststart;
    double duration = (double)testend / cv::getTickFrequency();
    std::cout << "cvFindCircles: " << circles.size() << " circles found in " << duration << "ms\n";
#endif

    return retval;
}
#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvFFT2DDoc = "2D-dimentional fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.\
The result is a complex-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, false)-function.\n\
";
//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvIFFT2DDoc = "2D-dimentional inverse fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given 2D-dataObject. The FFT is calculated planewise.\
The result is a real-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, false)-function.\n\
";
//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvFFT1DDoc = "1D-dimentional fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.\
The result is a complex-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, false, false, true)-function.\n\
";
//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvIFFT1DDoc = "1D-dimentional inverse fourier-transformation using cv::DFT.\n\
\n\
This filter tries to perform an inplace FFT for a given line or 2D-dataObject. The FFT is calculated linewise.\
The result is a real-dataObject. The axis-scales and units are invertes and modified.\n\
\n\
This filter internally calls the ito::dObjHelper::calcCVDFT(dObjImages, true, true, true)-function.\n\
";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvFFTParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input Object handle, must be a single plane").toLatin1().data());
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
const char* OpenCVFilters::cvMedianBlurDoc = "Planewise median blur filter.\n\
\n\
The function smoothes an image using the median filter with the kernel-size x kernel-size aperture. Each channel of a multi-channel image is processed independently. \
It can handle data objects of type uint8, uint16, int16, ito::tInt32, float32 and float64 only. \n\
\n\
The itom-wrapping does not work inplace currently. A new dataObject is allocated.\n\
\n\
Warning: NaN-handling for floats not verified.";
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::cvMedianBlurParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
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
    if(retval.containsError())
        return retval;

    int kernelsize = (*paramsOpt)[0].getVal<int>();

    if (kernelsize % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel must be odd").toLatin1().data());
    }

    if (kernelsize > 5 && dObjImages->getType() != ito::tUInt8)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernelsize > 3 and object is not uint8").toLatin1().data());
    }

    int z_length = dObjImages->calcNumMats();

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut = new cv::Mat[z_length];
    int itomtype = 0;

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
    itomtype = ito::dObjHelper::cvType2itomType(cvMatOut[0].type());
    if (itomtype > 0)
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
        retval += ito::RetVal(ito::retError, 0, tr("Unknown or unexpected CV-Datatype received.").toLatin1().data());
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
//    if(!retval.containsError())
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

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
//------------------------------------------------------------------------------------------------------------------------------
/*static*/ const char* OpenCVFilters::cvFindChessboardCornersDoc = "Finds the positions of internal corners of the chessboard.\n\
\n\
This filter is a wrapper for the cv::method cv::findChessboardCorners. \
\n\
The openCV-function attempts to determine whether the input image is a view of the chessboard pattern and locate the internal chessboard corners. \
The function returns a non-zero value if all of the corners are found and they are placed in a certain order (row by row, left to right in every row). \
Otherwise, if the function fails to find all the corners or reorder them, \
it returns 0. For example, a regular chessboard has 8 x 8 squares and 7 x 7 internal corners, that is, points where the black squares touch each other. \
The detected coordinates are approximate, and to determine their positions more accurately, the function calls cornerSubPix(). \n\
\n\
Remark 1: This function gives only a rough estimation of the positions. For a higher resolutions, you should use\
the function cornerSubPix() with different parameters if returned coordinates are not accurate enough.\
This function is wrapped to itom by the filter 'cvCornerSubPix'.\n\
\n\
Remark 2: The outer frame of the dataObject / the image should not be white but have approximately the same gray value than the bright field.\n\
\n\
Remark 3: The bright fields should be free of darker dirt or dust and you should apply a corse shading correction to improve the results. \n\
";
ito::RetVal OpenCVFilters::cvFindChessboardCornersParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "8bit grayscale input image") );
    paramsMand->append( ito::Param("pointsPerRow", ito::ParamBase::Int | ito::ParamBase::In, 10, new ito::IntMeta(1, 10000), "number of inner corners per chessboard row") );
    paramsMand->append( ito::Param("pointsPerColumn", ito::ParamBase::Int | ito::ParamBase::In, 10, new ito::IntMeta(1, 10000), "number of inner corners per chessboard column") );
    paramsMand->append( ito::Param("corners", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "output: float32-dataObject, [2 x n] with the coordinates of n detected corner points") );

    int allflags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK;
    paramsOpt->append( ito::Param("flags", ito::ParamBase::Int | ito::ParamBase::In, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE, new ito::IntMeta(0, allflags), "additional flags (OR-combination), see openCV-docu") );

    paramsOut->append( ito::Param("result", ito::ParamBase::Int | ito::ParamBase::Out, 0, new ito::IntMeta(0,1), "0: detection failed, 1: detection has been successful") );

    return retval;
}

ito::RetVal OpenCVFilters::cvFindChessboardCorners(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    const ito::DataObject *input = (const ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    int pointsPerRow = (*paramsMand)[1].getVal<int>();
    int pointsPerCol = (*paramsMand)[2].getVal<int>();
    ito::DataObject *corners = (ito::DataObject*)(*paramsMand)[3].getVal<void*>();
    int flags = (*paramsOpt)[0].getVal<int>();

    int result = 0;

    if(corners == NULL || input == NULL)
    {
        retval += ito::RetVal(ito::retError,0,"parameters 'dataObject' or 'corners' must not be NULL");
    }

    if(!retval.containsError())
    {
        cv::Mat *input_;
        input_ = (cv::Mat*)input->get_mdata()[ input->seekMat(0) ];
        std::vector<cv::Point2f> corners_;

        try
        {
            result = cv::findChessboardCorners(*input_, cv::Size(pointsPerCol, pointsPerRow), corners_, flags);
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
            result = 0;
        }

        if(result)
        {
            *corners = ito::DataObject(2, (int)corners_.size(), ito::tFloat32);
            ito::float32 *rowPtr1 = (ito::float32*)(corners->rowPtr(0,0));
            ito::float32 *rowPtr2 = (ito::float32*)(corners->rowPtr(0,1));

            for(size_t i = 0 ; i < corners_.size() ; i++)
            {
                rowPtr1[i] = corners_[i].x;
                rowPtr2[i] = corners_[i].y;
            }
        }
    }

    (*paramsOut)[0].setVal<int>(result);
    return retval;
}


/*static*/ const char *OpenCVFilters::cvCornerSubPixDoc = "Refines the corner locations e.g. from cvFindChessboardCorners.\n\
\n\
This filter is a wrapper for the cv::method cv::cornerSubPix. Check the openCV-doku for more details\n\
";
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal OpenCVFilters::cvCornerSubPixParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "8bit grayscale input image") );
    paramsMand->append( ito::Param("corners", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "initial coordinates of the input corners and refined coordinates provided for output") );
    paramsMand->append( ito::Param("winSize", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "Half of the side length of the search window. Example: (5,5) leads to a (11x11) search window") );

    int zeroZone[] = {-1,-1};
    param = ito::Param("zeroZone", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, "Half of the size of the dead region in the middle of the search zone over which the summation is not done. (-1,-1) indicates that there is no such a size");
    param.setVal<int*>(zeroZone);
    paramsOpt->append (param);

    paramsOpt->append( ito::Param("maxCount", ito::ParamBase::Int | ito::ParamBase::In, 200, new ito::IntMeta(1,100000), "position refinement stops after this maximum number of iterations") );
    paramsOpt->append( ito::Param("epsilon", ito::ParamBase::Double | ito::ParamBase::In, 0.05, new ito::DoubleMeta(0.0, 10.0), "position refinement stops when the corner position moves by less than this value") );

    return retval;
}

/*static*/ ito::RetVal OpenCVFilters::cvCornerSubPix(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    ito::DataObject *image = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *rawCorners = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (image == NULL || rawCorners == NULL)
    {
        retval += ito::RetVal(ito::retError,0, "Parameters 'image' and 'corners' may not be NULL");
    }

    int limits[] = {2,2,0,1000000};
    ito::DataObject *corners = apiCreateFromDataObject(rawCorners,2,ito::tFloat32,limits,&retval);

    cv::Size winSize;
    int *winSizeVals = paramsMand->at(2).getVal<int*>();
    int winSizeValsNum = paramsMand->at(2).getLen();

    if (winSizeValsNum == 1)
    {
        winSize = cv::Size(winSizeVals[0], winSizeVals[0]);
    }
    else if (winSizeValsNum == 2)
    {
        winSize = cv::Size(winSizeVals[0], winSizeVals[1]);
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"The parameter 'winSize' must have 1 or 2 values");
    }

    cv::Size zeroZone(-1,-1);
    int *zeroZoneVals = paramsOpt->at(0).getVal<int*>();
    int zeroZoneValsNum = paramsOpt->at(0).getLen();

    if (zeroZoneValsNum == 1)
    {
        zeroZone = cv::Size(zeroZoneVals[0], zeroZoneVals[0]);
    }
    else if (zeroZoneValsNum == 2)
    {
        zeroZone = cv::Size(zeroZoneVals[0], zeroZoneVals[1]);
    }
    else if (zeroZoneValsNum > 2)
    {
        retval += ito::RetVal(ito::retError,0,"The parameter 'zeroZone' must have between 0 and 2 values");
    }

    cv::TermCriteria criteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, paramsOpt->at(1).getVal<int>(), paramsOpt->at(2).getVal<double>());

    if (!retval.containsError())
    {
        const cv::Mat *image_;
        image_ = (const cv::Mat*)image->get_mdata()[ image->seekMat(0) ];

        std::vector<cv::Point2f> cornersVec;
        ito::float32 *x = (ito::float32*)rawCorners->rowPtr(0,0);
        ito::float32 *y = (ito::float32*)rawCorners->rowPtr(0,1);

        for (int i = 0; i < corners->getSize(1); ++i)
        {
            cornersVec.push_back( cv::Point2f(x[i],y[i]) );
        }

        try
        {
            cv::cornerSubPix(*image_, cornersVec, winSize, zeroZone, criteria);

            for (int i = 0; i < corners->getSize(1); ++i)
            {
                x[i] = cornersVec[i].x;
                y[i] = cornersVec[i].y;
            }

            *rawCorners = *corners;
        }
        catch (cv::Exception exc)
        {
            retval += ito::RetVal::format(ito::retError, 0, "%s", exc.err.c_str() );
        }

    }

    delete corners;
    corners = NULL;

    return retval;

}

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)
//----------------------------------------------------------------------------------------------------------------------------------
const char* OpenCVFilters::cvFlipLeftRightDoc = "This filter flips the image left to right. \n\
\n\
This filter applies the flip method cvFlip of OpenCV with the flipCode > 0 to a 2D source data object. The \
result is contained in the destination object\n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated\n\
.";
const char* OpenCVFilters::cvFlipUpDownDoc = "This filter flips the image upside down. \n\
\n\
This filter applies the flip method cvFlip of OpenCV with the flipCode = 0 to a 2D source data object. The \
result is contained in the destination object.\n\
\n\
It is allowed to let the filter work inplace if you give the same input than destination data object, else the output data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated\n\
.";
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

    if(dObjImages->getDims() > 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: nDim-stacks not supported yet").toLatin1().data());
    }

    int ysize = dObjImages->getSize(0);
    int xsize = dObjImages->getSize(1);

    retval += ito::dObjHelper::verify2DDataObject(dObjImages, "srcImage", ysize, ysize, xsize, xsize,  7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);

    if(!retval.containsError())
    {
        if(dObjDst != dObjImages)
        {
            ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjDst, "destImage", ysize, ysize, xsize, xsize,  1, dObjImages->getType());
            if(tRetval.containsError())
            {
                destTemp = ito::DataObject(ysize, xsize, dObjImages->getType());
            }
            else
            {
                destTemp = *dObjDst;
                overWrite = false;
            }
        }
        else
        {
            //destDataPhase = ito::DataObject( ysize, xsize, ito::tFloat64);
            destTemp = *dObjDst;
            overWrite = false;
        }
    }

    if(!retval.containsError())
    {
        int z_length = dObjImages->calcNumMats();

        cv::Mat *cvMatIn = NULL;
        cv::Mat *cvMatOut = NULL;

        for (int z = 0; z < z_length; z++)
        {
            cvMatIn = (cv::Mat*)dObjImages->get_mdata()[dObjImages->seekMat(z)];
            cvMatOut = (cv::Mat*)destTemp.get_mdata()[dObjImages->seekMat(z)];
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

    if(!retval.containsError())
    {
        if(overWrite)
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
const char* OpenCVFilters::cvRemoveSpikesDoc = "Set single spikes at measurement edges to a new value. \n\
\n\
This filter creates a binary mask for the input object. The value of mask(y,x) will be 1 if value of input(y,x) is within the specified range and is finite.\
The mask is eroded and than dilated by kernel size using openCV cv::erode and cv::dilate with a single iteration. \
In the last step the value of output(y,x) is set to newValue if mask(y,x) is 0.\n\
\n\
It is allowed to let the filter work inplace if you give the same source and destination data object, else the destination data object is verified \
if it fits to the size and type of the source data object and if not a new one is allocated and the input data is copied to the new object. \n\
";
ito::RetVal OpenCVFilters::cvRemoveSpikesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    paramsMand->append( ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, "32 or 64 bit floating point input image") );
    paramsMand->append( ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "32 or 64 bit floating point output image") );

    paramsOpt->append( ito::Param("kernelSize", ito::ParamBase::Int | ito::ParamBase::In, 5, new ito::IntMeta(3, 13), "N defines the N x N kernel") );
    paramsOpt->append( ito::Param("lowestValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 0.0, "Lowest value to consider as valid") );
    paramsOpt->append( ito::Param("highestValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), 1.0, "Highest value to consider as valid") );
    paramsOpt->append( ito::Param("newValue", ito::ParamBase::Double | ito::ParamBase::In, -std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::max(), std::numeric_limits<ito::float64>::quiet_NaN(), "Replacement value for spike elements") );
    return retval;
}

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
    if(retval.containsError())
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
    if(dObjSrc->getType() == ito::tFloat64)
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
                if(ito::dObjHelper::isFinite(srcPtr[x]) && srcPtr[x] > minClipVal && srcPtr[x] < maxClipVal)
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
                if(ito::dObjHelper::isFinite(srcPtr[x]) && srcPtr[x] > minClipValf && srcPtr[x] < maxClipValf)
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
        if(dObjSrc->getType() == ito::tFloat64)
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
                    if(tmpPtr[x] == 0)
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
                    if(tmpPtr[x] == 0)
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
ito::RetVal OpenCVFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(OpenCVFilters::cvDilate, OpenCVFilters::cvDilateErodeParams, tr(cvDilateDoc));
    m_filterList.insert("cvDilate", filter);

    filter = new FilterDef(OpenCVFilters::cvErode, OpenCVFilters::cvDilateErodeParams, tr(cvErodeDoc));
    m_filterList.insert("cvErode", filter);

    filter = new FilterDef(OpenCVFilters::cvMedianBlur, OpenCVFilters::cvMedianBlurParams, tr(cvMedianBlurDoc));
    m_filterList.insert("cvMedianBlur", filter);

    filter = new FilterDef(OpenCVFilters::cvBlur, OpenCVFilters::cvBlurParams, tr(cvBlurDoc));
    m_filterList.insert("cvBlur", filter);

    filter = new FilterDef(OpenCVFilters::cvFFT2D, OpenCVFilters::cvFFTParams, tr(cvFFT2DDoc));
    m_filterList.insert("cvFFT2D", filter);

    filter = new FilterDef(OpenCVFilters::cvIFFT2D, OpenCVFilters::cvFFTParams, tr(cvIFFT2DDoc));
    m_filterList.insert("cvIFFT2D", filter);

    filter = new FilterDef(OpenCVFilters::cvFFT1D, OpenCVFilters::cvFFTParams, tr(cvFFT1DDoc));
    m_filterList.insert("cvFFT1D", filter);

    filter = new FilterDef(OpenCVFilters::cvIFFT1D, OpenCVFilters::cvFFTParams, tr(cvIFFT1DDoc));
    m_filterList.insert("cvIFFT1D", filter);

    filter = new FilterDef(OpenCVFilters::cvRemoveSpikes, OpenCVFilters::cvRemoveSpikesParams, tr(cvRemoveSpikesDoc));
    m_filterList.insert("cvRemoveSpikes", filter);



    /*filter = new FilterDef(OpenCVFilters::cvCalcHist, OpenCVFilters::cvCalcHistParams, tr(cvCalcHistDoc));
    m_filterList.insert("cvCalcHistogram", filter);*/

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    filter = new FilterDef(OpenCVFilters::cvFindCircles, OpenCVFilters::cvFindCirclesParams, tr(cvFindCirclesDoc));
    m_filterList.insert("cvFindCircles", filter);

    filter = new FilterDef(OpenCVFilters::cvFindChessboardCorners, OpenCVFilters::cvFindChessboardCornersParams, tr(cvFindChessboardCornersDoc));
    m_filterList.insert("cvFindChessboardCorners", filter);

    filter = new FilterDef(OpenCVFilters::cvCornerSubPix, OpenCVFilters::cvCornerSubPixParams, tr(cvCornerSubPixDoc));
    m_filterList.insert("cvCornerSubPix", filter);

#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    filter = new FilterDef(OpenCVFilters::cvFlipUpDown, OpenCVFilters::stdParams2Objects, tr(cvFlipUpDownDoc));
    m_filterList.insert("cvFlipUpDown", filter);

    filter = new FilterDef(OpenCVFilters::cvFlipLeftRight, OpenCVFilters::stdParams2Objects, tr(cvFlipLeftRightDoc));
    m_filterList.insert("cvFlipLeftRight", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}