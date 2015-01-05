/* ********************************************************************
    Plugin "demoAlgorithms" for itom software
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

#include "demoAlgorithms.h"

#include "common/helperActuator.h"    //! This include is for the actuator-helper class used in demoMoveActuator
#include "common/helperGrabber.h"     //! This include is for the camera-helper class used in demoSnapImage & demoSnapMovie

#include "DataObject/dataobj.h"
#include "pluginVersion.h"
#include <QtCore/QtPlugin>


//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithmsInterface::DemoAlgorithmsInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DemoAlgorithms");

    m_description = QObject::tr("DemoAlgorithms to show a plugin developer how to write plugins in c++");
    m_detaildescription = QObject::tr("The DemoAlgorithms-DLL contains some basic filter function to show a plugin developer how to use a motor or programm an own plugin widget");
    m_author            = "Wolfram Lyda, ITO, University Stuttgart";
    m_license           = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("Fill in about dialog content");        
    
}

//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithmsInterface::~DemoAlgorithmsInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DemoAlgorithmsInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DemoAlgorithms)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DemoAlgorithmsInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DemoAlgorithms)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(demoalgorithmsinterface, DemoAlgorithmsInterface)
#endif


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithms::DemoAlgorithms() : AddInAlgo()
{
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
ito::RetVal DemoAlgorithms::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;
    //AlgoWidgetDef *widget = NULL;

    //----------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------------------------------User-Defined-Content-----------------------------------------------------
    filter = new FilterDef(DemoAlgorithms::demoMoveActuator, DemoAlgorithms::demoMoveActuatorParams, "Demo algorithm (I) for plugin-developers - actuator communication. Moves selected axes of an actuator.", ito::AddInAlgo::catNone, ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoMoveActuator", filter);
    filter = new FilterDef(DemoAlgorithms::demoSnapImage, DemoAlgorithms::demoSnapImageParams, "Demo algorithm (II) for plugin-developers - camera communication. Snaps a single image.", ito::AddInAlgo::catNone, ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoSnapImage", filter);
    filter = new FilterDef(DemoAlgorithms::demoSnapMovie, DemoAlgorithms::demoSnapMovieParams, "Demo algorithm (III) for plugin-developers - camera communication. Snaps a number of images to a stack.", ito::AddInAlgo::catNone, ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoSnapMovie", filter);
    filter = new FilterDef(DemoAlgorithms::demoTestActuator, DemoAlgorithms::demoTestActuatorParams, "Demo algorithm (IV) for plugin-developers - actuator communication. Moves first axis of an actuator several time to test the actuator performance.", ito::AddInAlgo::catNone, ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoTestActuator", filter);

    //---------------------------------------------------------End-User-Defined-Content-------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DemoAlgorithms::close(ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
/** short description for this filter
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    longer description for this filter
*/
/*
ito::RetVal DemoAlgorithms::filterMethod(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    //double a = 2.0;
    //int b =1;
    //QString str="hallo";
    //outVals->append(a);
    //outVals->append(b);
    //outVals->append(str);

    return retval;
}
*/

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling the corresponding filter
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*        - describe the mandatory parameters here (list)
*
*   opt. Params:
*       - describe the optional parameter here (list)
*/
/*ito::RetVal DemoAlgorithms::filterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::tParam param;

    if (!paramsMand)
    {
        retval = ito::RetVal(ito::retError, 0, "uninitialized vector for mandatory parameters!");
    }
    else if (!paramsOpt)
    {
        retval = ito::RetVal(ito::retError, 0, "uninitialized vector for optional parameters!");
    }
    else
    {
        //mandatory
        param = ito::Param("dataObject", ito::ParamBase::DObjPtr, NULL, "description"); 
        // Optional comment: if the param is a dataObject and it IS NOT modified but only input please use ito::ParamBase::DObjPtr | ito::ParamBase::In to the type for automatic documentaton
        // Optional comment: if the param is a dataObject and it IS modifiedif input + output add ito::ParamBase::DObjPtr | ito::ParamBase::In | | ito::ParamBase::Out for automatic documentaton
        paramsMand->append(param);
        param = ito::Param("doubleValue", ito::ParamBase::Double, 0.0, 65535.0, 10.0, "double value between 0.0 and 65535.0, default: 10.0");
        paramsMand->append(param);

        //optional
        //param = ito::Param("integerValue", ito::ParamBase::Int, 0, 65535, 65535, "integer value beween 0 and 65535, default: 65535");
        //paramsOpt->append(param);
    }

    return retval;
}
*/

//----------------------------------------------------------------------------------------------------------------------------------
/** short description for this widget
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    longer description for this widget
*/
QWidget* DemoAlgorithms::dialog(QVector<ito::Param> * /*paramsMand*/, QVector<ito::Param> * /*paramsOpt*/, ito::RetVal &retValue)
{
    retValue += ito::retOk;

    //example:
    //DialogDemoAlgorithms *dialog = new DialogDemoAlgorithms(NULL);
    //return qobject_cast<QWidget*>(dialog);

    //please delete the following line if you really want to provide the widget
    return NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling dialog
*    @param [in]    paramsMand    mandatory parameters for calling dialog
*    @param [in]    paramsOpt    optional parameters for calling dialog
*
*    mand. Params:
*        - describe the mandatory parameters here (list)
*
*   opt. Params:
*       - describe the optional parameter here (list)
*/
ito::RetVal DemoAlgorithms::dialogParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        //mandatory
        /*param = ito::Param("dataObject", ito::ParamBase::DObjPtr, NULL, "description");
        paramsMand->append(param);
        param = ito::Param("doubleValue", ito::ParamBase::Double, 0.0, 65535.0, 10.0, "double value between 0.0 and 65535.0, default: 10.0");
        paramsMand->append(param);*/

        //optional
        /*param = ito::Param("integerValue", ito::ParamBase::Int, 0, 65535, 65535, "integer value beween 0 and 65535, default: 65535");
        paramsOpt->append(param);*/
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------User-Defined-Content-----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling demoMoveActuator copied form the template "filterParams" and modified
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*        - Motor     --> Handle to the Motorstage
*       - axis1     --> Number of the first axis to move
*       - pos1      --> New position of the 1. axis
*       - axis2     --> Number of the second axis to move
*       - pos2      --> New Position of axis 2
*
*   opt. Params:
*       - speed     --> Algorithm will change speed to this value (default = 1.0)
*/
ito::RetVal DemoAlgorithms::demoMoveActuatorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("Motor", ito::ParamBase::HWRef, NULL, "Handle to the Motorstage");
        paramsMand->append(param);
        param = ito::Param("axis1", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0, "Number of the first axis to move");
        paramsMand->append(param);
        param = ito::Param("pos1", ito::ParamBase::Double, std::numeric_limits<double>::max() * -1, std::numeric_limits<double>::max(), 0.0, "New position of the 1. axis");
        paramsMand->append(param);
        param = ito::Param("axis2", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0, "Number of the second axis to move");
        paramsMand->append(param);
        param = ito::Param("pos2", ito::ParamBase::Double, std::numeric_limits<double>::max() * -1, std::numeric_limits<double>::max(), 0.0, "New Position of axis 2");
        paramsMand->append(param);
        param = ito::Param("speed", ito::ParamBase::Double,std::numeric_limits<double>::max() * -1, std::numeric_limits<double>::max(),  1.0, "Algorithm will change speed to this value");
        paramsOpt->append(param);
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to move an actuator in an algorithm
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    This algotihmes uses the ito::threadActuator-class defined in "common/helperActuator.h" to move the axis of a actuator living in another thread.
*
*   \author Wolfram Lyda
*   \date 04.2012
*   \sa helperActuator.h, ito::threadActuator
*/
ito::RetVal DemoAlgorithms::demoMoveActuator(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::threadActuator myStage(paramsMand, 0);

    int axis1 = (int)(*paramsMand)[1].getVal<int>();
    double pos1 = (double)(*paramsMand)[2].getVal<double>();
    int axis2 = (int)(*paramsMand)[3].getVal<int>();
    double pos2 = (double)(*paramsMand)[4].getVal<double>();

    double speed = (double)(*paramsOpt)[0].getVal<double>();

    retval += myStage.setPosAbs(axis1, pos1);
    retval += myStage.getPos(axis1, pos1);
    ito::ParamBase setSpeed("speed", ito::ParamBase::Double, speed);
    ito::Param speedParam("speed");
    retval += myStage.setParam(setSpeed);

    retval += myStage.getParam(speedParam);

    retval += myStage.setPosAbs(axis2, pos2, 0);
    retval += myStage.waitForSemaphore();
    retval += myStage.getPos(axis2, pos2);

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling demoSnapImage copied form the template "filterParams" and modified
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*        - myCamera  --> Handle to the Camera
*       - Image     --> Empty object, will contain 2D image later
*
*   opt. Params: NONE

*/
ito::RetVal DemoAlgorithms::demoSnapImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("camera", ito::ParamBase::HWRef, NULL, "Handle to the Camera");
        paramsMand->append(param);
        param = ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "Empty object, will contain 2D image later");
        paramsMand->append(param);

        paramsOpt->clear();
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to capture a single image from a camera with an algorithm
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    This algotihmes uses the ito::threadCamera-class defined in "common/helperGrabber.h" to get a deep copy of a camera image. The camera is living in another thread.
*
*   \author Wolfram Lyda
*   \date 04.2012
*   \sa helperGrabber.h, ito::threadCamera
*/
ito::RetVal DemoAlgorithms::demoSnapImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::threadCamera myCamera(paramsMand, 0);

    ito::DataObject *image = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>());
    if(image == NULL)
    {
        return ito::RetVal(ito::retError, 0, "Iamge handle empty");
    }

    ito::DataObject dObj;   // create an mepty object

    // Now get a shallow copy for a single frame
    retval += myCamera.startDevice();
    if(retval.containsError())
    {
        return retval;  // end and return error of failed
    }

    retval += myCamera.acquire(0);

    if(!retval.containsError())     // Only try to getVal if retval is retOK ir retWarning
    {
        retval += myCamera.getVal(dObj);
        dObj.copyTo(*image);    // Make a deepcopy of the data, otherwise the content of image will be overwritten during the next snap
    }

    retval += myCamera.stopDevice();

    if(!retval.containsError())
    {
        QString msg = tr("Recorded via demoSnapImage-filter");
        image->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling demoSnapMovie copied form the template "filterParams" and modified
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*        - myCamera  --> Handle to the Camera
*       - Movie     --> Empty object, will contain 3D image stack later
*       - images    --> Number of images to aquire
*
*   opt. Params: NONE

*/
ito::RetVal DemoAlgorithms::demoSnapMovieParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("camera", ito::ParamBase::HWRef, NULL, "Handle to the Camera");
        paramsMand->append(param);
        param = ito::Param("Movie", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "Empty resulting data object. Contains 3d data object with acquired data after call.");
        paramsMand->append(param);
        param = ito::Param("images", ito::ParamBase::Int, 0, 1000000 , 1, "Number of images to aquire");
        paramsMand->append(param);
        paramsOpt->clear();
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to capture a set of images from a camera with an algorithm
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    This algotihmes uses the ito::threadCamera-class defined in "common/helperGrabber.h" to get a set of camera image. The camera is living in another thread.
*   The image buffer is allocated first and filled by direct deep-copies from the grabber using the copyVal-function.
*
*   \author Wolfram Lyda
*   \date 04.2012
*   \sa helperGrabber.h, ito::threadCamera
*/
ito::RetVal DemoAlgorithms::demoSnapMovie(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::threadCamera myCamera(paramsMand, 0);

    ito::DataObject *movie = static_cast<ito::DataObject*>( (*paramsMand)[1].getVal<void*>());
    if(movie == NULL)
    {
        return ito::RetVal(ito::retError, 0, "Movie handle empty");
    }

    // This is for benchmarking the camera speed
//    double starttime = (double)(cv::getTickCount())/cv::getTickFrequency();
    double middletime = 0.0;
    double endtime = 0.0;
    int images = (int)(*paramsMand)[2].getVal<int>();

    int bpp = 0, sizeX = 0, sizeY = 0;
    ito::DataObject dObj;

    retval += myCamera.getImageParams(bpp, sizeX, sizeY);
    if(retval.containsError())
    {
        return retval;
    }

    int sizes[2] = {sizeY, sizeX};

    bpp = ceil(bpp / 8.0) * 8;  // round the bitdepth to 8, 16 ... bit

    switch(bpp)
    {
        case 8:
            *movie = ito::DataObject(images, sizes[0], sizes[1], ito::tUInt8);
            break;
        case 16:
            *movie = ito::DataObject(images, sizes[0], sizes[1], ito::tUInt16);
            break;
        case 64:
            *movie = ito::DataObject(images, sizes[0], sizes[1], ito::tFloat64);
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, "Grabber bit depth not supported");
        break;
    }

    retval += myCamera.startDevice();
    middletime = (double)(cv::getTickCount())/cv::getTickFrequency();
    if(!retval.containsError()) // If no error after startDevice -> start capturing
    {
        for(int i = 0; i < images; i++)
        {
            dObj = ito::DataObject(2, sizes, (*movie).getType(), (cv::Mat*)((*movie).get_mdata())[i], 1);   // set a ROI of movie handled by dObj

            retval += myCamera.acquire(0);
            if(retval.containsError())  // Stop of error occures no
            {
                break;
            }

            retval += myCamera.copyVal(dObj);    // Direct DeepCopy the image from grabber to the roi of movie (dObj)
            if(retval.containsError())  // Stop of error occures no
            {
                break;
            }
        }
    }

    endtime = (double)(cv::getTickCount())/cv::getTickFrequency();

    retval += myCamera.stopDevice(); // Stop of error occures no
    std::cout << "\nAcquire: overall\t" << endtime-middletime << "\n";

    if(!retval.containsError())
    {
        QString msg = tr("Recorded via demoSnapMovie-filter in [s]: ");
        msg.append(QString::number(endtime-middletime));
        movie->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling demoTestActuator copied form the template "filterParams" and modified
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*        - Motor     --> Handle to the Motorstage
*       - axis1     --> Number of the first axis to move
*       - pos1      --> New position of the 1. axis
*       - axis2     --> Number of the second axis to move
*       - pos2      --> New Position of axis 2
*
*   opt. Params:
*       - speed     --> Algorithm will change speed to this value (default = 1.0)
*/
ito::RetVal DemoAlgorithms::demoTestActuatorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("Motor", ito::ParamBase::HWRef, NULL, "Handle to the Motorstage");
        paramsMand->append(param);
        param = ito::Param("deltaX", ito::ParamBase::Double, std::numeric_limits<double>::max() * -1, std::numeric_limits<double>::max(), 0.0, "Delta for the current axis.");
        paramsMand->append(param);
        param = ito::Param("iterations", ito::ParamBase::Int, 1, std::numeric_limits<int>::max(), 1000, "Number of iterations to move.");
        paramsMand->append(param);

        param = ito::Param("absRelFlag", ito::ParamBase::Int, 0, 1, 0, "Toggle between abs (0) and relative (1) movements");
        paramsOpt->append(param);
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to move an actuator in an algorithm
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    This algotihmes uses the ito::threadActuator-class defined in "common/helperActuator.h" to move the axis of a actuator living in another thread.
*
*   \author Wolfram Lyda
*   \date 04.2012
*   \sa helperActuator.h, ito::threadActuator
*/
ito::RetVal DemoAlgorithms::demoTestActuator(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::threadActuator myStage(paramsMand, 0);
    double deltaX = (double)(*paramsMand)[1].getVal<double>();
    int iter = (int)(*paramsMand)[2].getVal<int>();
    int mode = (int)(*paramsOpt)[0].getVal<int>();
    double startPos = 0.0;
    double starttime = 0.0;
    double endtime = 0.0;

    retval += myStage.getPos(0, startPos);



    if(!retval.containsError())
    {
        // Get current time
        starttime = (double)(cv::getTickCount())/cv::getTickFrequency();
        int i = 0;
        for(i = 0; i < iter; i ++)
        {
            if(mode) retval += myStage.setPosRel(0, deltaX);
            else retval += myStage.setPosAbs(0, deltaX * i + startPos );
            if(retval.containsError()) break;
        }
        // Get current time
        endtime = (double)(cv::getTickCount())/cv::getTickFrequency();

        std::cout << "\nMovement: Duration\t" << endtime-starttime << "s, Frequency " << i / (endtime-starttime) << "hz\n";
    }
    
    


    return retval;
}