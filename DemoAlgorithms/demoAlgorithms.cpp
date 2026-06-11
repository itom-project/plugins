/* ********************************************************************
    Plugin "demoAlgorithms" for itom software
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

#include "demoAlgorithms.h"

#include "algoCancelWidget.h"

#include "common/pluginThreadCtrl.h" //! This include is for the actuator-helper class used in demoMoveActuator, demoSnapImage and demoSnapMovie

#include "DataObject/dataobj.h"
#include "gitVersion.h"
#include "pluginVersion.h"
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>


//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithmsInterface::DemoAlgorithmsInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DemoAlgorithms");

    m_description =
        QObject::tr("DemoAlgorithms to show a plugin developer how to write plugins in c++");
    m_detaildescription =
        QObject::tr("The DemoAlgorithms-DLL contains some basic filter function to show a plugin "
                    "developer how to use a motor or program an own plugin widget");
    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithmsInterface::~DemoAlgorithmsInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DemoAlgorithmsInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(DemoAlgorithms)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DemoAlgorithmsInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(DemoAlgorithms)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
DemoAlgorithms::DemoAlgorithms() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize filter functions within this addIn
 *    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on
 * initialization
 *    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on
 * initialization
 *    @return                    retError in case of an error
 *
 *    Here are the filter functions defined that are available through this addIn.
 *    These are:
 *       - filterName    description for this filter
 *
 *   This plugin additionally makes available the following widgets, dialogs...:
 *       - dialogName    description for this widget
 */
ito::RetVal DemoAlgorithms::init(
    QVector<ito::ParamBase>* /*paramsMand*/,
    QVector<ito::ParamBase>* /*paramsOpt*/,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;
    FilterDef* filter = NULL;
    AlgoWidgetDef* widget = NULL;

    //----------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------------------------------User-Defined-Content-----------------------------------------------------
    filter = new FilterDef(
        DemoAlgorithms::demoMoveActuator,
        DemoAlgorithms::demoMoveActuatorParams,
        tr("Demo algorithm (I) for plugin-developers - actuator communication. Moves selected axes "
           "of an actuator."),
        ito::AddInAlgo::catNone,
        ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoMoveActuator", filter);
    filter = new FilterDef(
        DemoAlgorithms::demoSnapImage,
        DemoAlgorithms::demoSnapImageParams,
        tr("Demo algorithm (II) for plugin-developers - camera communication. Snaps a single "
           "image."),
        ito::AddInAlgo::catNone,
        ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoSnapImage", filter);
    filter = new FilterDef(
        DemoAlgorithms::demoSnapMovie,
        DemoAlgorithms::demoSnapMovieParams,
        tr("Demo algorithm (III) for plugin-developers - camera communication. Snaps a number of "
           "images to a stack."),
        ito::AddInAlgo::catNone,
        ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoSnapMovie", filter);
    filter = new FilterDef(
        DemoAlgorithms::demoTestActuator,
        DemoAlgorithms::demoTestActuatorParams,
        tr("Demo algorithm (IV) for plugin-developers - actuator communication. Moves first axis "
           "of an actuator several time to test the actuator performance."),
        ito::AddInAlgo::catNone,
        ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("demoTestActuator", filter);
    filter = new FilterDefExt(
        DemoAlgorithms::demoCancellationFunction,
        DemoAlgorithms::demoCancellationFunctionParams,
        tr("Demo algorithm, that can be cancelled and that delivers status updates."),
        ito::AddInAlgo::catNone,
        ito::AddInAlgo::iNotSpecified,
        QString(),
        true,
        true);
    m_filterList.insert("demoCancellationFunction", filter);

    widget = new AlgoWidgetDef(
        DemoAlgorithms::demoWidget, DemoAlgorithms::demoWidgetParams, tr("Demo widget"));
    m_algoWidgetList.insert("demoWidget", widget);

    widget = new AlgoWidgetDef(
        DemoAlgorithms::demoAlgoCancelWidget,
        DemoAlgorithms::demoAlgoCancelWidgetParams,
        tr("Demo widget to show how to observe a long-running algorithm and cancel its execution"));
    m_algoWidgetList.insert("demoCancellationFunctionWidget", widget);

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
ito::RetVal DemoAlgorithms::close(ItomSharedSemaphore* waitCond)
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
ito::RetVal DemoAlgorithms::filterMethod(QVector<ito::ParamBase> *paramsMand,
QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
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
/*ito::RetVal DemoAlgorithms::filterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param>
*paramsOpt, QVector<ito::Param> *paramsOut)
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
        // Optional comment: if the param is a dataObject and it IS NOT modified but only input
please use ito::ParamBase::DObjPtr | ito::ParamBase::In to the type for automatic documentation
        // Optional comment: if the param is a dataObject and it IS modifiedif input + output add
ito::ParamBase::DObjPtr | ito::ParamBase::In | | ito::ParamBase::Out for automatic documentation
        paramsMand->append(param);
        param = ito::Param("doubleValue", ito::ParamBase::Double, 0.0, 65535.0, 10.0, "double value
between 0.0 and 65535.0, default: 10.0"); paramsMand->append(param);

        //optional
        //param = ito::Param("integerValue", ito::ParamBase::Int, 0, 65535, 65535, "integer value
between 0 and 65535, default: 65535");
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
QWidget* DemoAlgorithms::dialog(
    QVector<ito::Param>* /*paramsMand*/, QVector<ito::Param>* /*paramsOpt*/, ito::RetVal& retValue)
{
    retValue += ito::retOk;

    // example:
    // DialogDemoAlgorithms *dialog = new DialogDemoAlgorithms(NULL);
    // return qobject_cast<QWidget*>(dialog);

    // please delete the following line if you really want to provide the widget
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
ito::RetVal DemoAlgorithms::dialogParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        // mandatory
        /*param = ito::Param("dataObject", ito::ParamBase::DObjPtr, NULL, "description");
        paramsMand->append(param);
        param = ito::Param("doubleValue", ito::ParamBase::Double, 0.0, 65535.0, 10.0, "double value
        between 0.0 and 65535.0, default: 10.0"); paramsMand->append(param);*/

        // optional
        /*param = ito::Param("integerValue", ito::ParamBase::Int, 0, 65535, 65535, "integer value
        between 0 and 65535, default: 65535"); paramsOpt->append(param);*/
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
ito::RetVal DemoAlgorithms::demoMoveActuatorParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "motor", ito::ParamBase::HWRef, NULL, tr("Handle to the Motorstage").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "axis1",
            ito::ParamBase::Int,
            0,
            std::numeric_limits<int>::max(),
            0,
            tr("Number of the first axis to move").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "pos1",
            ito::ParamBase::Double,
            std::numeric_limits<double>::max() * -1,
            std::numeric_limits<double>::max(),
            0.0,
            tr("New position of the 1. axis").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "axis2",
            ito::ParamBase::Int,
            0,
            std::numeric_limits<int>::max(),
            0,
            tr("Number of the second axis to move").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "pos2",
            ito::ParamBase::Double,
            std::numeric_limits<double>::max() * -1,
            std::numeric_limits<double>::max(),
            0.0,
            tr("New Position of axis 2").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "speed",
            ito::ParamBase::Double,
            std::numeric_limits<double>::max() * -1,
            std::numeric_limits<double>::max(),
            1.0,
            tr("Algorithm will change speed to this value").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to move an actuator in an algorithm
 *    @param [in]    paramsMand    mandatory parameters
 *    @param [in]    paramsOpt    optional parameters
 *
 *    This algotihmes uses the ito::threadActuator-class defined in "common/helperActuator.h" to
 * move the axis of a actuator living in another thread.
 *
 *   \author Wolfram Lyda
 *   \date 04.2012
 *   \sa helperActuator.h, ito::threadActuator
 */
ito::RetVal DemoAlgorithms::demoMoveActuator(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::ActuatorThreadCtrl myStage(paramsMand->at(0), NULL);

    int axis1 = paramsMand->at(1).getVal<int>();
    double pos1 = paramsMand->at(2).getVal<double>();
    int axis2 = paramsMand->at(3).getVal<int>();
    double pos2 = paramsMand->at(4).getVal<double>();

    double speed = paramsOpt->at(0).getVal<double>();

    retval += myStage.setPosAbs(axis1, pos1);
    retval += myStage.getPos(axis1, pos1);
    ito::ParamBase setSpeed("speed", ito::ParamBase::Double, speed);
    ito::Param speedParam("speed");
    retval += myStage.setParam(setSpeed);

    retval += myStage.getParam(speedParam);

    retval += myStage.setPosAbs(axis2, pos2, 0);
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
ito::RetVal DemoAlgorithms::demoSnapImageParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "camera", ito::ParamBase::HWRef, NULL, tr("Handle to the Camera").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "image",
            ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
            NULL,
            tr("Empty object, will contain 2D image later").toLatin1().data());
        paramsMand->append(param);

        paramsOpt->clear();
        param = ito::Param(
            "simulateWork",
            ito::ParamBase::Int | ito::ParamBase::In,
            0,
            1,
            0,
            tr("If 1, some calculations are simulated during the wait for the camera thread in "
               "order to show a time efficient approach.")
                .toLatin1()
                .data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to capture a single image from a camera with an algorithm
 *    @param [in]    paramsMand    mandatory parameters
 *    @param [in]    paramsOpt    optional parameters
 *
 *    This algotihmes uses the ito::threadCamera-class defined in "common/helperGrabber.h" to get a
 * deep copy of a camera image. The camera is living in another thread.
 *
 *   \author Wolfram Lyda
 *   \date 04.2012
 *   \sa helperGrabber.h, ito::threadCamera
 */
ito::RetVal DemoAlgorithms::demoSnapImage(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataIOThreadCtrl myCamera(paramsMand->at(0), NULL);
    bool simulateWork = paramsOpt->at(0).getVal<int>() > 0;

    ito::DataObject* image = static_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());
    if (image == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Image handle empty").toLatin1().data());
    }

    ito::DataObject dObj; // create an mepty object

    // Now get a shallow copy for a single frame
    retval += myCamera.startDevice();
    if (retval.containsError())
    {
        return retval; // end and return error of failed
    }

    if (!simulateWork)
    {
        retval += myCamera.acquire(0, 5000);

        if (!retval.containsError()) // Only try to getVal if retval is retOK or retWarning (e.g. no
                                     // timeout while acquisition)
        {
            retval += myCamera.getVal(dObj, 5000);
        }

        if (!retval.containsError()) // Only try to copy the image if retval is retOK or retWarning
                                     // (e.g. no timeout while getVal)
        {
            dObj.copyTo(*image); // Make a deepcopy of the data, otherwise the content of image will
                                 // be overwritten during the next snap
        }
    }
    else
    {
        retval += myCamera.acquire(0, 0); // immediately return after the acquisition...
        double v;
        for (int i = 0; i < 1e6; ++i)
        {
            v = std::sin(2.54 * i);
        }

        retval +=
            myCamera.waitForSemaphore(5000); // wait until the acquisition has been done in order to
                                             // get synchronized with the camera thread

        if (!retval.containsError()) // Only try to getVal if retval is retOK or retWarning (e.g. no
                                     // timeout while acquisition)
        {
            retval +=
                myCamera.getVal(dObj, 0); // get the image and do not wait for it to be ready...
            for (int i = 0; i < 1e6; ++i) // do some intense work...
            {
                v = std::sin(2.54 * i);
            }
            retval += myCamera.waitForSemaphore(
                5000); // wait until the image has been obtained. If an error occurs here, don't
                       // read dObj since it is not in a valid state.
        }

        if (!retval.containsError()) // Only try to copy the image if retval is retOK or retWarning
                                     // (e.g. no timeout while getVal)
        {
            dObj.copyTo(*image); // Make a deepcopy of the data, otherwise the content of image will
                                 // be overwritten during the next snap
        }
    }

    retval +=
        myCamera.stopDevice(); // you have to call any method of the camera thread after
                               // timeout-based calls. If one of the methods above run into a
                               // timeout and you passed e.g. a dataObject to the camera thread that
                               // has been created in this method, you need to wait until the camera
                               // thread is not using this dataObject any more before it is deleted
                               // upon finishing this method! If stopDevice is finished, no other
                               // method of the camera thread is still running!

    if (!retval.containsError())
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
*       - images    --> Number of images to acquire
*
*   opt. Params: NONE

*/
ito::RetVal DemoAlgorithms::demoSnapMovieParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);

    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "camera", ito::ParamBase::HWRef, NULL, tr("Handle to the Camera").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "movie",
            ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out,
            NULL,
            tr("Empty resulting data object. Contains 3d data object with acquired data after "
               "call.")
                .toLatin1()
                .data());
        paramsMand->append(param);
        param = ito::Param(
            "images",
            ito::ParamBase::Int,
            0,
            1000000,
            1,
            tr("Number of images to acquire").toLatin1().data());
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
 *    This algotihmes uses the ito::threadCamera-class defined in "common/helperGrabber.h" to get a
 * set of camera image. The camera is living in another thread. The image buffer is allocated first
 * and filled by direct deep-copies from the grabber using the copyVal-function.
 *
 *   \author Wolfram Lyda
 *   \date 04.2012
 *   \sa helperGrabber.h, ito::threadCamera
 */
ito::RetVal DemoAlgorithms::demoSnapMovie(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* /*paramsOpt*/,
    QVector<ito::ParamBase>* /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataIOThreadCtrl myCamera(paramsMand->at(0), NULL);

    ito::DataObject* movie = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (movie == nullptr)
    {
        return ito::RetVal(ito::retError, 0, tr("Movie handle empty").toLatin1().data());
    }

    // This is for benchmarking the camera speed
    //    double starttime = (double)(cv::getTickCount())/cv::getTickFrequency();
    double middletime = 0.0;
    double endtime = 0.0;
    int images = (int)(*paramsMand)[2].getVal<int>();

    int bpp = 0, sizeX = 0, sizeY = 0;
    ito::DataObject dObj;

    retval += myCamera.getImageParams(bpp, sizeX, sizeY);
    if (retval.containsError())
    {
        return retval;
    }

    int sizes[2] = {sizeY, sizeX};

    bpp = ceil(bpp / 8.0) * 8; // round the bitdepth to 8, 16 ... bit

    switch (bpp)
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
        retval +=
            ito::RetVal(ito::retError, 0, tr("Grabber bit depth not supported").toLatin1().data());
        break;
    }

    retval += myCamera.startDevice();
    middletime = (double)(cv::getTickCount()) / cv::getTickFrequency();
    if (!retval.containsError()) // If no error after startDevice -> start capturing
    {
        for (int i = 0; i < images; i++)
        {
            dObj = ito::DataObject(
                2,
                sizes,
                (*movie).getType(),
                (cv::Mat*)((*movie).get_mdata())[i],
                1); // set a ROI of movie handled by dObj

            retval += myCamera.acquire(0);
            if (retval.containsError()) // Stop of error occurs no
            {
                break;
            }

            retval += myCamera.copyVal(
                dObj); // Direct DeepCopy the image from grabber to the roi of movie (dObj)
            if (retval.containsError()) // Stop of error occurs no
            {
                break;
            }
        }
    }

    endtime = (double)(cv::getTickCount()) / cv::getTickFrequency();

    retval += myCamera.stopDevice(); // Stop of error occurs no
    std::cout << "\nAcquire: overall\t" << endtime - middletime << "\n";

    if (!retval.containsError())
    {
        QString msg = tr("Recorded via demoSnapMovie-filter in [s]: ");
        msg.append(QString::number(endtime - middletime));
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
ito::RetVal DemoAlgorithms::demoTestActuatorParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param(
            "motor", ito::ParamBase::HWRef, NULL, tr("Handle to the Motorstage").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "deltaX",
            ito::ParamBase::Double,
            std::numeric_limits<double>::max() * -1,
            std::numeric_limits<double>::max(),
            0.0,
            tr("Delta for the current axis.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param(
            "iterations",
            ito::ParamBase::Int,
            1,
            std::numeric_limits<int>::max(),
            1000,
            tr("Number of iterations to move.").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param(
            "absRelFlag",
            ito::ParamBase::Int,
            0,
            1,
            0,
            tr("Toggle between abs (0) and relative (1) movements").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** Demo-filter to show how to move an actuator in an algorithm
 *    @param [in]    paramsMand    mandatory parameters
 *    @param [in]    paramsOpt    optional parameters
 *
 *    This algotihmes uses the ito::threadActuator-class defined in "common/helperActuator.h" to
 * move the axis of a actuator living in another thread.
 *
 *   \author Wolfram Lyda
 *   \date 04.2012
 *   \sa helperActuator.h, ito::threadActuator
 */
ito::RetVal DemoAlgorithms::demoTestActuator(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::ActuatorThreadCtrl myStage(paramsMand->at(0), NULL);
    double deltaX = (double)(*paramsMand)[1].getVal<double>();
    int iter = (int)(*paramsMand)[2].getVal<int>();
    int mode = (int)(*paramsOpt)[0].getVal<int>();
    double startPos = 0.0;
    double starttime = 0.0;
    double endtime = 0.0;

    retval += myStage.getPos(0, startPos);

    if (!retval.containsError())
    {
        // Get current time
        starttime = (double)(cv::getTickCount()) / cv::getTickFrequency();
        int i = 0;
        for (i = 0; i < iter; i++)
        {
            if (mode)
                retval += myStage.setPosRel(0, deltaX);
            else
                retval += myStage.setPosAbs(0, deltaX * i + startPos);
            if (retval.containsError())
                break;
        }
        // Get current time
        endtime = (double)(cv::getTickCount()) / cv::getTickFrequency();

        std::cout << "\nMovement: Duration\t" << endtime - starttime << "s, Frequency "
                  << i / (endtime - starttime) << "hz\n";
    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DemoAlgorithms::demoWidgetParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    return retval;
}

/*static*/ QWidget* DemoAlgorithms::demoWidget(
    QVector<ito::ParamBase>* paramsMand, QVector<ito::ParamBase>* paramsOpt, ito::RetVal& retValue)
{
    QWidget* widget = new QWidget();
    return widget;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DemoAlgorithms::demoAlgoCancelWidgetParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    return retval;
}

/*static*/ QWidget* DemoAlgorithms::demoAlgoCancelWidget(
    QVector<ito::ParamBase>* paramsMand, QVector<ito::ParamBase>* paramsOpt, ito::RetVal& retValue)
{
    AlgoCancelWidget* widget = new AlgoCancelWidget(demoCancellationFunction, NULL);
    return widget;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DemoAlgorithms::demoCancellationFunctionParams(
    QVector<ito::Param>* paramsMand, QVector<ito::Param>* paramsOpt, QVector<ito::Param>* paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
    }

    return retval;
}

/*static*/ ito::RetVal DemoAlgorithms::demoCancellationFunction(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    QVector<ito::ParamBase>* paramsOut,
    QSharedPointer<ito::FunctionCancellationAndObserver> observer)
{
    // let the algorithm run for 10 seconds and reports the progress.
    // this algorithm can be cancelled

    ito::RetVal retVal;

    QElapsedTimer timer;
    timer.start();
    qint64 nextProgressReport = 1000; // every second

    if (observer)
    {
        observer->setProgressValue(observer->progressMinimum());
        observer->setProgressText("Start of algorithm");
    }

    while (timer.elapsed() < 10000)
    {
        if (observer)
        {
            if (timer.elapsed() >= nextProgressReport)
            {
                // always pass the value between the given minimum / maximum of the observer
                int value = observer->progressMinimum() +
                    timer.elapsed() * (observer->progressMaximum() - observer->progressMinimum()) /
                        10000;
                observer->setProgressValue(value);
                observer->setProgressText(
                    QString("This algorithm run %1 from 10.0 seconds").arg(timer.elapsed() / 1000));
                nextProgressReport += 1000;
            }

            if (observer->isCancelled())
            {
                retVal += ito::RetVal(ito::retError, 0, "algorithm cancelled");
                break;
            }
        }

        QThread::msleep(100);
    }

    if (!retVal.containsError())
    {
        observer->setProgressValue(observer->progressMaximum());
        observer->setProgressText(QString("This algorithm run 10.0 from 10.0 seconds"));
    }

    return retVal;
}
