/* ********************************************************************
Plugin "NITWidySWIR" for itom software
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

#include "NITWidySWIR.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include "dockWidgetNITWidySWIR.h"

/**** NITLibrary ****/
//#include "NITException.h"
//#include "NITCatalog.h"
#include "NITManager.h"
#include "NITDevice.h"

/**** NITLibrary::NITToolBox ****/
//#include "NITPlayer.h"
//#include "NITAutomaticGainControl.h"

#include "opencv2/opencv.hpp"

using namespace NITLibrary;
using namespace std;
using namespace cv;

QList<int> NITWidySWIR::m_initList;

//----------------------------------------------------------------------------------------------------------------------------------
NITWidySWIRInterface::NITWidySWIRInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("NITWidySWIR");

    m_description = QObject::tr("Plugin for cameras from NEW IMAGING TECHNOLOGIES (NIT).");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
        "This dataIO grabber plugin runs with the NITLIBRARY API from the company NEW IMAGING TECHNOLOGIES (NIT). \n\
        This plugin has been tested using the Widy SWIR 640U-S camera in USB2 version and the SDK version 1.5. \n\
        In this itom plugin a bad pixel correction is not implemented, because for optical measurement applications it is not necessary. \n\
        \n\
        **WARNING**: Some parameters do no work with the current tested camera!\n\
        	NITLibrary defines the parameter **\"offset, gain, histogram threshold, pixel clock\"** as changable, but with the current version is does not! \n\
            	The parameter **pixel_clock** is set to *Readonly*, because the camera acquisition crash. This problem should be tested with the next NITLibrary version.\n";
    m_detaildescription = QObject::tr(docstring);

    m_author = "J. Krauter, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LPGL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("printManual", ito::ParamBase::Int, 0, 1, 0, tr("If printManual is set to 1, the parameters of the camera are printed into the itom shelf.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("printParameterValues", ito::ParamBase::Int, 0, 1, 0, tr("If printParameterValues is set to 1, all available values of the parameters are printed into the itom shelf.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("deviceNumber", ito::ParamBase::Int, 0, 99, -1, tr("Number of the device, which you want to connect.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

}

//----------------------------------------------------------------------------------------------------------------------------------
NITWidySWIRInterface::~NITWidySWIRInterface()
{
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIRInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(NITWidySWIR)
        return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIRInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(NITWidySWIR)
        return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
NITWidySWIR::NITWidySWIR() : AddInGrabber(),
m_grabbingStatus(GrabIdle),
m_camManager(NITManager::getInstance()),
m_camDevice(NULL),
m_frameObserver(NULL),
m_isgrabbing(false),
m_pixelCorrection(false),
m_minBPP(0),
m_maxBPP(0)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::In, "NITWidySWIR", tr("Name of plugin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("model_id", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::In, "unknown", tr("Model ID of connected device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("firmware_version", ito::ParamBase::Double | ito::ParamBase::Readonly | ito::ParamBase::In, 0.00, 100.00, 0.00, tr("Firmware version of connectecd device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 0, 9999, 0, tr("Serial number of connected device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = { 0, 0, 0, 0 };
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("Region of Interest ROI [x0, y0, width, height].").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 2048), ito::RangeMeta(0, 2048));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 640, tr("Width of ROI (x-direction).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 512, tr("Height of ROI (y-direction).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 32, 14, tr("Bits per pixel bpp (8,14,32).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, 0.1, 25.6, 0.01, tr("Integrationtime of connected device (0.1..25.6) in [s].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Gain of camera (0..7.5) in [no unit].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("framerate", ito::ParamBase::Int | ito::ParamBase::In, 0, 1000, 60, tr("Framerate of image acquisition (0..1000) in [fps].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("pixel_clock", ito::ParamBase::Double | ito::ParamBase::In, 12.5, 80.00, 25.00, tr("Pixel clock of device (12.5, 16.66, 20, 25, 33.33, 40, 50, 66.66, 80) in [MHz].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("histogram_threshold", ito::ParamBase::Double | ito::ParamBase::In, 0.00, 100.00, 25.00, tr("Histogram threshold of image acquisition (0.025, 0.1, 0.4, 1.6) in [%].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("trigger_mode", ito::ParamBase::String | ito::ParamBase::In, "unknown", tr("Trigger Mode of connected device (Disabled, Input, Output). Use Disabled for software trigger.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("shutter_mode", ito::ParamBase::String | ito::ParamBase::In, "unknown", tr("Shutter Mode of connected device (Global Shutter, Rolling).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::In, 0.00, 100.00, 00.00, tr("Offset of image acquisition (0..100) in [%].").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("enable_pixel_correction", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("enables pixel correction by using NIT NUC files. By changing the integration_time the NUC file is changed, too.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("nucFilePath", ito::ParamBase::String | ito::ParamBase::In, "D:\\NUC", tr("Path of the NUC pixel correction files.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    dockWidgetNITWidySWIR *dw = new dockWidgetNITWidySWIR(this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
NITWidySWIR::~NITWidySWIR()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ParamMapIterator it;

    bool printManual = (paramsOpt->at(0).getVal<int>()) ? 1 : 0;
    bool printParameterValues = (paramsOpt->at(1).getVal<int>()) ? 1 : 0;
    int givenDeviceNumber = paramsOpt->at(2).getVal<int>();
    m_instanceInitialized = false;
    m_deviceCount = 0;

    try //open Device
    {
        m_deviceCount = m_camManager.deviceCount();

        if (m_deviceCount != 0 && m_initList.length() == 0) // no camera initialized and some available
        {
            if (givenDeviceNumber != -1)// use given deviceNumber
            {
                m_deviceNumber = givenDeviceNumber;
            }
            else // use deviceNumber 0
            {
                m_deviceNumber = 0;
            }

            m_camDevice = m_camManager.openDevice(m_deviceNumber);
            m_initList.append(m_deviceNumber);
            m_instanceInitialized = true;
        }
        else if (m_deviceCount > 1 && m_initList.length() >= 1) // one camera connected and more are available
        {
            if (givenDeviceNumber != -1)
            {
                m_camDevice = m_camManager.openDevice(givenDeviceNumber);
                m_deviceNumber = givenDeviceNumber;
                m_initList.append(m_deviceNumber);
                m_instanceInitialized = true;
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("One device is already connected. For a new instance, you must define a deviceNumber.").toLatin1().data());
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Camera ist already initialized.").toLatin1().data());
        }

    }
    catch (NITException& e)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("NITLibrary Exception: %1").arg(e.what()).toLatin1().data());
    }

    if (!retValue.containsError())// print Manual, print parameter values
    {
        try
        {
            if (printManual) //paramsOpt->at(0) is printManual. Manual with parameters will be printed in the itom shelf. 
            {
                cout << "Manual: " << m_camDevice->manual() << "\n" << endl;
            }

            if (printParameterValues) //paramsOpt->at(0) is printParameterValues. Parameters with all avaiable values will be printed in the itom shelf. 
            {
                for (NITParamIterator paramIt = m_camDevice->begin(); paramIt != m_camDevice->end(); ++paramIt)
                {
                    const NITParam& param = **paramIt;
                    int i = 0;

                    cout << "---------------------------\n" << endl;
                    cout << "Name :          " << param.name() << "\n" << endl;
                    cout << "Is Innert :     " << ((param.isInert()) ? "true" : "false") << "\n" << endl;
                    cout << "------Possible Values------" << "\n" << endl;
                    for (NITParam::ValueIterator valueIt = param.begin(); valueIt != param.end(); ++valueIt)
                    {
                        cout << (*valueIt) << "\t";
                        if (++i % 5 == 0)
                        {
                            cout << "\n" << endl;
                        }

                    }
                    cout << "\n---------------------------\n" << endl;
                }
            }
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("NITLibrary Exception: %1").arg(e.what()).toLatin1().data());
        }
    }

    if (!retValue.containsError()) // initialize parameters
    {
        try
        {
            const NITDeviceDescriptor& deviceDescriptor = m_camManager.getDeviceDescriptor(m_deviceNumber);

            m_params["model_id"].setVal<char*>(&deviceDescriptor.modelId()[0u]);
            m_identifier = m_params["model_id"].getVal<char*>();

            m_params["firmware_version"].setVal<double>(deviceDescriptor.firmwareVersion());

            m_params["serial_number"].setVal<int>(deviceDescriptor.serialNumber());

            string deviceString = m_camDevice->paramStrValueOf("Trigger Mode");
            m_params["trigger_mode"].setVal<char*>(strcpy(new char[deviceString.length() + 1], deviceString.c_str()));

            deviceString = m_camDevice->paramStrValueOf("Mode");
            m_params["shutter_mode"].setVal<char*>(strcpy(new char[deviceString.length() + 1], deviceString.c_str()));

            m_params["framerate"].setMeta(new ito::IntMeta(m_camDevice->minFps(true), m_camDevice->maxFps(true), 1), true);
            m_params["framerate"].setVal<int>(m_camDevice->fps(true));
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("NITLibrary Exception: %1").arg(e.what()).toLatin1().data());
        }
    }

    if (!retValue.containsError())// define min and max values of parameters
    {
        try
        {
            for (NITParamIterator paramIt = m_camDevice->begin(); paramIt != m_camDevice->end(); ++paramIt) //Iteration through all parameter. 
            {
                const NITParam& param = **paramIt;

                if (param.name() == "Exposure Time")
                {
                    it = m_params.find("integration_time");

                    if (param.isInert())
                    {
                        m_params["integration_time"].setFlags(ito::ParamBase::Readonly);
                    }

                    double integrationtime = (double)atof(m_camDevice->paramStrValueOf("Exposure Time").c_str()) / 1000;

                    NITParam::ValueIterator valueIt = param.begin();
                    string minstringValue = (*valueIt);

                    string maxstringValue = (*valueIt);
                    do
                    {
                        maxstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    double minintegrationtime = (double)atof(minstringValue.c_str()) / 1000;
                    double maxintegrationtime = (double)atof(maxstringValue.c_str()) / 1000;

                    it->setMeta(new ito::DoubleMeta(minintegrationtime, maxintegrationtime, 0.1), true);
                    it->setVal<double>(integrationtime);
                }
                else if (param.name() == "Gain")
                {
                    it = m_params.find("gain");

                    if (param.isInert())
                    {
                        m_params["gain"].setFlags(ito::ParamBase::Readonly);
                    }
                    //m_params["integration_time"].setFlags(ito::ParamBase::Readonly);// delete if parameter is changable with different NITLibrary version > 1.5

                    double gain = (double)atof(m_camDevice->paramStrValueOf("Gain").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string minstringValue = (*valueIt);

                    string maxstringValue = (*valueIt);
                    do
                    {
                        maxstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    double mingain = (double)atof(minstringValue.c_str());
                    double maxgain = (double)atof(maxstringValue.c_str());

                    it->setMeta(new ito::DoubleMeta(mingain, maxgain, 0.05), true);
                    it->setVal<double>(gain);
                }
                else if (param.name() == "Pixel Depth")
                {
                    it = m_params.find("bpp");

                    if (param.isInert())
                    {
                        m_params["bpp"].setFlags(ito::ParamBase::Readonly);
                    }

                    int bpp = (int)atof(m_camDevice->paramStrValueOf("Pixel Depth").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string maxstringValue = (*valueIt);

                    string minstringValue = (*valueIt);
                    do
                    {
                        minstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    m_minBPP = (int)atof(minstringValue.c_str());
                    m_maxBPP = (int)atof(maxstringValue.c_str());

                    it->setMeta(new ito::IntMeta(m_minBPP, m_maxBPP, 1), true);
                    it->setVal<int>(bpp);
                }
                else if (param.name() == "Pixel Clock")
                {
                    it = m_params.find("pixel_clock");

                    if (param.isInert())
                    {
                        m_params["pixel_clock"].setFlags(ito::ParamBase::Readonly);
                    }
                    m_params["pixel_clock"].setFlags(ito::ParamBase::Readonly);// delete if parameter is changable with different NITLibrary version > 1.5

                    double clock = (double)atof(m_camDevice->paramStrValueOf("Pixel Clock").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string minstringValue = (*valueIt);

                    string maxstringValue = (*valueIt);
                    do
                    {
                        maxstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    double minclock = (double)atof(minstringValue.c_str());
                    double maxclock = (double)atof(maxstringValue.c_str());

                    it->setMeta(new ito::DoubleMeta(minclock, maxclock, 0.01), true);
                    it->setVal<double>(clock);
                }
                else if (param.name() == "Number of Column")
                {
                    it = m_params.find("sizex");

                    if (param.isInert())
                    {
                        m_params["roi"].setFlags(ito::ParamBase::Readonly);
                    }

                    int sizex = (int)atof(m_camDevice->paramStrValueOf("Number of Column").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string minstringValue = (*valueIt);

                    string maxstringValue = (*valueIt);
                    do
                    {
                        maxstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    int minsizex = (int)atof(minstringValue.c_str());
                    int maxsizex = (int)atof(maxstringValue.c_str());

                    valueIt = param.begin();
                    string valueFirst = (*valueIt);
                    ++valueIt;
                    string valueSecond = (*valueIt);

                    int stepsize = (int)atof(valueSecond.c_str()) - (int)atof(valueFirst.c_str());

                    it->setMeta(new ito::IntMeta(0, maxsizex, stepsize), true);
                    it->setVal<int>(sizex);
                }
                else if (param.name() == "Number of Line")
                {
                    it = m_params.find("sizey");

                    if (param.isInert())
                    {
                        m_params["roi"].setFlags(ito::ParamBase::Readonly);
                    }

                    int sizey = (int)atof(m_camDevice->paramStrValueOf("Number of Line").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string minstringValue = (*valueIt);

                    string maxstringValue = (*valueIt);
                    do
                    {
                        maxstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    valueIt = param.begin();
                    string valueFirst = (*valueIt);
                    ++valueIt;
                    string valueSecond = (*valueIt);

                    int stepsize = (int)atof(valueSecond.c_str()) - (int)atof(valueFirst.c_str());

                    int minsizey = (int)atof(minstringValue.c_str());
                    int maxsizey = (int)atof(maxstringValue.c_str());

                    it->setMeta(new ito::IntMeta(0, maxsizey, stepsize), true);
                    it->setVal<int>(sizey);
                }
                else if (param.name() == "Offset")
                {
                    it = m_params.find("offset");

                    if (param.isInert())
                    {
                        m_params["offset"].setFlags(ito::ParamBase::Readonly);
                    }
                    //m_params["offset"].setFlags(ito::ParamBase::Readonly);// delete if parameter is changable with different NITLibrary version > 1.5

                    double offset = (double)atof(m_camDevice->paramStrValueOf("Offset").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string maxstringValue = (*valueIt);

                    string minstringValue = (*valueIt);
                    do
                    {
                        minstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    double minoffset = (double)atof(minstringValue.c_str());
                    double maxoffset = (double)atof(maxstringValue.c_str());

                    it->setMeta(new ito::DoubleMeta(minoffset, maxoffset, 0.01), true);
                    it->setVal<double>(offset);
                }
                else if (param.name() == "Histogram Threshold")
                {
                    it = m_params.find("histogram_threshold");

                    if (param.isInert())
                    {
                        m_params["histogram_threshold"].setFlags(ito::ParamBase::Readonly);
                    }
                    //m_params["histogram_threshold"].setFlags(ito::ParamBase::Readonly);// delete if parameter is changable with different NITLibrary version > 1.5

                    double offset = (double)atof(m_camDevice->paramStrValueOf("Histogram Threshold").c_str());

                    NITParam::ValueIterator valueIt = param.begin();
                    string maxstringValue = (*valueIt);

                    string minstringValue = (*valueIt);
                    do
                    {
                        minstringValue = (*valueIt);
                        ++valueIt;
                    } while (valueIt != param.end());

                    double minoffset = (double)atof(minstringValue.c_str());
                    double maxoffset = (double)atof(maxstringValue.c_str());

                    it->setMeta(new ito::DoubleMeta(minoffset, maxoffset, 0.005), true);
                    it->setVal<double>(offset);
                }
            }


            ito::IntMeta *sizexMeta = static_cast<ito::IntMeta*>(m_params["sizex"].getMeta());
            ito::IntMeta *sizeyMeta = static_cast<ito::IntMeta*>(m_params["sizey"].getMeta());

            ito::RangeMeta widthMeta(sizexMeta->getMin(), sizexMeta->getMax(), sizexMeta->getStepSize());
            ito::RangeMeta heightMeta(sizeyMeta->getMin(), sizeyMeta->getMax(), sizeyMeta->getStepSize());

            m_params["roi"].setMeta(new ito::RectMeta(widthMeta, heightMeta), true);

            int roi[] = { 0, 0, sizexMeta->getMax(), sizeyMeta->getMax() };
            m_params["roi"].setVal<int*>(roi, 4);

            m_params["roi"].setFlags(ito::ParamBase::Readonly); // delete if parameter is changable with different NITLibrary version > 1.5

            retValue += checkData(); //update m_data
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("NITLibrary Exception: %1").arg(e.what()).toLatin1().data());
        }
    }

    if (!retValue.containsError()) // load NIT NUC pixel correction file
    {
        QString filePath = m_params["nucFilePath"].getVal<char *>();
        m_nucFilePath = filePath;
        retValue += loadNUCFile(m_nucFilePath, m_params["integration_time"].getVal<double>());
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_instanceInitialized)
    {

        if (!m_initList.isEmpty()) //del cam
        {
            if (m_isgrabbing)
            {
                retValue += stopDevice(NULL);
            }

            m_camDevice = NULL;
            m_initList.removeAt(m_initList.indexOf(m_deviceNumber));

        }
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
        *val = it.value();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        retValue += stopDevice(NULL);
        try
        {
            if (QString::compare(key, "framerate", Qt::CaseInsensitive) == 0)
            {
                m_camDevice->setFps((unsigned int)val->getVal<int>(), true);
                it->setVal<int>(m_camDevice->fps(true));
            }
            else if (QString::compare(key, "bpp", Qt::CaseInsensitive) == 0)
            {

                if (val->getVal<int>() == 8)
                {
                    m_camDevice->setParamValueOf("Pixel Depth", "8bits");
                }
                else if (val->getVal<int>() == 14)
                {
                    m_camDevice->setParamValueOf("Pixel Depth", "14bits");
                }

                it->setVal<int>(atoi(m_camDevice->paramStrValueOf("Pixel Depth").c_str()));
            }
            else if (QString::compare(key, "pixel_clock", Qt::CaseInsensitive) == 0)
            {
                double value = val->getVal<double>();
                const char* stringValue;

                if (12.5 <= value && value < 16.66)
                {
                    stringValue = "12.5MHz";
                }
                else if (16.66 <= value && value < 20.00)
                {
                    stringValue = "16.66MHz";
                }
                else if (20.00 <= value && value < 25.00)
                {
                    stringValue = "20MHz";
                }
                else if (25.00 <= value && value < 33.33)
                {
                    stringValue = "25MHz";
                }
                else if (33.33 <= value && value < 40.00)
                {
                    stringValue = "33.33MHz";
                }
                else if (40.00 <= value && value < 50.00)
                {
                    stringValue = "40MHz";
                }
                else if (50.00 <= value && value < 66.66)
                {
                    stringValue = "50MHz";
                }
                else if (66.66 <= value && value < 80.00)
                {
                    stringValue = "66.66MHz";
                }
                else if (80.00 <= value)
                {
                    stringValue = "80MHz";
                }
                else
                {
                    cout << "pixel clock value out of range\n" << endl;
                }

                m_camDevice->setParamValueOf("Pixel Clock", stringValue);
                it->setVal<int>(atoi(m_camDevice->paramStrValueOf("Pixel Clock").c_str()));
            }
            else if (QString::compare(key, "integration_time", Qt::CaseInsensitive) == 0)
            {
                m_camDevice->setParamValueOf("Exposure Time", (to_string((val->getVal<double>() * 1000)) + QString(QLatin1String("\u00B5")).toStdString() + "s"));
                it->setVal<double>((double)atof(m_camDevice->paramStrValueOf("Exposure Time").c_str()) / 1000);
                retValue += loadNUCFile(m_nucFilePath, m_params["integration_time"].getVal<double>());
            }
            else if (QString::compare(key, "gain", Qt::CaseInsensitive) == 0)
            {
                m_camDevice->setParamValueOf("Gain", to_string((val->getVal<double>())));
                it->setVal<double>((double)atof(m_camDevice->paramStrValueOf("Gain").c_str()));
            }
            else if (QString::compare(key, "offset", Qt::CaseInsensitive) == 0)
            {
                m_camDevice->setParamValueOf("Offset", (to_string(val->getVal<double>()) + "% full scale"));
                it->setVal<double>((double)atof(m_camDevice->paramStrValueOf("Offset").c_str()));
            }
            else if (QString::compare(key, "histogram_threshold", Qt::CaseInsensitive) == 0)
            {
                double value = val->getVal<double>();
                const char* stringValue;

                if (0.025 <= value && value < 0.1)
                {
                    stringValue = "0.025%";
                }
                else if (0.1 <= value && value < 0.4)
                {
                    stringValue = "0.1%";
                }
                else if (0.4 <= value && value < 1.6)
                {
                    stringValue = "0.4%";
                }
                else if (1.6 <= value)
                {
                    stringValue = "1.6%";
                }

                m_camDevice->setParamValueOf("Histogram Threshold", stringValue);

                string stringVal = m_camDevice->paramStrValueOf("Histogram Threshold");

                if (stringVal == "0.025%")
                {
                    value = 0.025;
                }
                else if (stringVal == "0.1%")
                {
                    value = 0.1;
                }
                else if (stringVal == "0.4%")
                {
                    value = 0.4;
                }
                else if (stringVal == "1.6%")
                {
                    value = 1.6;
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("wrong Histogram value returned: %1").arg(QString::fromStdString(stringVal)).toLatin1().data());
                }

                it->setVal<double>(value);
            }
            else if (QString::compare(key, "roi", Qt::CaseInsensitive) == 0)
            {
                int *roi = val->getVal<int*>();

                m_camDevice->setParamValueOf("First Column", roi[0]);
                m_camDevice->setParamValueOf("Number of Column", roi[2]);
                m_camDevice->setParamValueOf("First Line", roi[1]);
                m_camDevice->setParamValueOf("Number of Line", roi[3]);

                roi[0] = (int)atof(m_camDevice->paramStrValueOf("First Column").c_str());
                roi[2] = (int)atof(m_camDevice->paramStrValueOf("Number of Column").c_str());
                roi[1] = (int)atof(m_camDevice->paramStrValueOf("First Line").c_str());
                roi[3] = (int)atof(m_camDevice->paramStrValueOf("Number of Line").c_str());

                m_params["roi"].setVal<int*>(roi, 4);
                m_params["sizex"].setVal<int>(roi[2]);
                m_params["sizey"].setVal<int>(roi[3]);

            }
            else if (QString::compare(key, "trigger_mode", Qt::CaseInsensitive) == 0)
            {
                if (QString::compare(val->getVal<char*>(), "Disabled", Qt::CaseInsensitive) == 0)
                {
                    m_camDevice->setParamValueOf("Trigger Mode", "Disabled");
                }
                else if (QString::compare(val->getVal<char*>(), "Input", Qt::CaseInsensitive) == 0)
                {
                    m_camDevice->setParamValueOf("Trigger Mode", "Input");
                }
                else if (QString::compare(val->getVal<char*>(), "Output", Qt::CaseInsensitive) == 0)
                {
                    m_camDevice->setParamValueOf("Trigger Mode", "Output");
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Given Trigger Mode is not defined.").toLatin1().data());
                }

                string trigger = m_camDevice->paramStrValueOf("Trigger Mode");
                m_params["trigger_mode"].setVal<char*>(strcpy(new char[trigger.length() + 1], trigger.c_str()));

            }
            else if (QString::compare(key, "shutter_mode", Qt::CaseInsensitive) == 0)
            {
                if (QString::compare(val->getVal<char*>(), "Global Shutter", Qt::CaseInsensitive) == 0)
                {
                    m_camDevice->setParamValueOf("Mode", "Global Shutter");
                }
                else if (QString::compare(val->getVal<char*>(), "Rolling", Qt::CaseInsensitive) == 0)
                {
                    m_camDevice->setParamValueOf("Mode", "Rolling");
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Given Shutter Mode is not defined.").toLatin1().data());
                }

                string shutter = m_camDevice->paramStrValueOf("Mode");
                m_params["shutter_mode"].setVal<char*>(strcpy(new char[shutter.length() + 1], shutter.c_str()));
            }
            else if (QString::compare(key, "enable_pixel_correction", Qt::CaseInsensitive) == 0)
            {
                m_pixelCorrection = bool(val->getVal<int>());
                m_params["enable_pixel_correction"].setVal<int>(m_pixelCorrection);

                if (bool(val->getVal<int>())) //pixel correction activated
                {
                    m_camDevice->setParamValueOf("Pixel Depth", "14bits");
                    m_params["bpp"].setMeta(new ito::IntMeta(64, 64, 1), true);
                    m_params["bpp"].setVal<int>(64);
                }
                else // pixel correction deactivated
                {
                    int bpp = atoi(m_camDevice->paramStrValueOf("Pixel Depth").c_str());
                    m_params["bpp"].setMeta(new ito::IntMeta(m_minBPP, m_maxBPP, 1), true);
                    m_params["bpp"].setVal<int>(bpp);
                }
            }
            else if (QString::compare(key, "nucFilePath", Qt::CaseInsensitive) == 0)
            {
                m_nucFilePath = QString(val->getVal<char *>());
                retValue += loadNUCFile(m_nucFilePath, m_params["integration_time"].getVal<double>());
                m_params["nucFilePath"].setVal<char *>((m_nucFilePath.toLatin1().data()));

            }
            else
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("NITLibrary Exception: %1").arg(e.what()).toLatin1().data());
        }
        retValue += startDevice(NULL);
        retValue += checkData(); //update m_data
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);


    if (grabberStartedCount() == 0)
    {
        try
        {
            m_frameObserver = new NITWidySWIRObserver(this);
            m_frameObserver->connectTo(*m_camDevice);

            /**** Start Capture!!! ****/
            m_camDevice->start();						// Start Capture
            m_isgrabbing = true;
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("%1").arg(e.what()).toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        incGrabberStarted();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() == 0)
    {
        try
        {
            m_camDevice->stop();
            if (m_frameObserver)
            {
                m_frameObserver->disconnect();
                DELETE_AND_SET_NULL(m_frameObserver);
            }
            m_isgrabbing = false;
        }
        catch (NITException& e)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("%1").arg(e.what()).toLatin1().data());
        }
    }

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_mutex.lock();
        m_grabbingStatus = GrabAcquisitionRunning;
        //wait until the next image has been received
        GrabStatus grabbingStatus = m_grabbingStatus;
        m_mutex.unlock();

        while (grabbingStatus != GrabFrameReceived)
        {
            QCoreApplication::processEvents();

            m_mutex.lock();
            grabbingStatus = m_grabbingStatus;
            m_mutex.unlock();
        }

    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal NITWidySWIR::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    //ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);

    //const int bufferWidth = m_params["sizex"].getVal<int>();
    //const int bufferHeight = m_params["sizey"].getVal<int>();

    m_mutex.lock();

    if (m_grabbingStatus == GrabIdle)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject && hasListeners)
        {
            retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            retValue += checkData(externalDataObject); //update external object or m_data
        }

        if (externalDataObject)
        {
            m_data.deepCopyPartial(*externalDataObject);
        }

        m_grabbingStatus = GrabIdle;
    }

    m_mutex.unlock();

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void NITWidySWIR::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal NITWidySWIR::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new dialogNITWidySWIR(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
void NITWidySWIRObserver::onNewFrame(const NITLibrary::NITFrame& frame, const NITLibrary::NITFrameInfo& info)
{
    m_pCamera->m_mutex.lock();
    if (m_pCamera->m_grabbingStatus == NITWidySWIR::GrabAcquisitionRunning)
    {
        if (m_pCamera->m_pixelCorrection)//acquisition with pixel correction
        {
            //1. convert frame to datatype of m_NUCDark
            cv::Mat frame_;
            frame.convertTo(frame_, CV_64FC1);

            //make sure that the result of the OpenCV calculation has exactly the same type and size than one plane in m_pCamera->m_data
            *(m_pCamera->m_data.getCvPlaneMat(0)) = (frame_ - m_pCamera->m_NUCDark).mul(m_pCamera->m_GainMap);

        }
        else //acquisition without pixel correction
        {
            if (frame.type() == CV_8U)
            {
                m_pCamera->m_data.copyFromData2D<ito::uint8>(frame.data, frame.cols, frame.rows);
            }
            else if (frame.type() == CV_16U)
            {
                m_pCamera->m_data.copyFromData2D<ito::uint16>((ito::uint16*)frame.data, frame.cols, frame.rows);
            }
            else
            {
                cout << "invalid frame format encountered.\n" << endl;
            }
        }


        m_pCamera->m_grabbingStatus = NITWidySWIR::GrabFrameReceived;
    }
    m_pCamera->m_mutex.unlock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NITWidySWIR::loadNUCFile(const QString &filePath, double integrationTime)
{
    ito::RetVal retValue(ito::retOk);

    integrationTime = integrationTime * 1000;
    integrationTime = (int)integrationTime;

    QString nucFile("NUCFactory_" + QString::number(integrationTime) + "us.yml");
    string absPath = QString("%1\\%2").arg(filePath).arg(nucFile).toLatin1().data();

    try
    {
        FileStorage nucFile(absPath, FileStorage::READ);
        if (nucFile.isOpened())
        {
            nucFile["NUCDark"] >> m_NUCDark;
            nucFile["GainMap"] >> m_GainMap;

            Mat nuc;
            m_NUCDark.convertTo(nuc, CV_64FC1);
            m_NUCDark = nuc;

            Mat gain;
            m_GainMap.convertTo(gain, CV_64FC1);
            m_GainMap = gain;

            nucFile.release();
            //cout << "loaded NUC file name: " << absPath << " for the interation time of: " << integrationTime << "mikroseconds \n" << endl;
        }
        else
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Can not load YAML file. File does not exist.").toLatin1().data());
        }

    }
    catch (NITException& e)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Can not load YAML file. CV Exception: %1").arg(e.what()).toLatin1().data());
    }



    return retValue;
}