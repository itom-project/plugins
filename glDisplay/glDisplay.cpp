/* ********************************************************************
    Plugin "GLDisplay" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "glDisplay.h"

#include "common/helperCommon.h"

#include <string.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qdesktopwidget.h>
#include <qapplication.h>
#include <qsurfaceformat.h>

#ifndef WIN32
    #include <unistd.h>
#endif

#include "pluginVersion.h"
#include "gitVersion.h"

Q_DECLARE_METATYPE(QVector<unsigned char>)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplayInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(GLDisplay)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplayInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(GLDisplay)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** addIn interface constructor
*/
GLDisplayInterface::GLDisplayInterface()
{
    ito::Param paramVal;
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("GLDisplay");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This plugin displays a frameless window that displays one or multiple arrays using OpenGL technology. \n\
Each array is then created as texture, where the horizontal and vertical wrap property can be chosen individually. \n\
OpenGL allows a fast switch between all created textures. \n\
\n\
Per default, the window is either displayed in a second screen (fullscreen) or if only one screen is available - \n\
placed as small window in the top left corner of the main screen. Otherwise chose an appropriate x0, y0, xsize and ysize \n\
parameter at initialization. \n\
\n\
In order to assign textures, use the exec-function 'addTextures' and pass a 2d or 3d dataObject, where in the latter case \n\
every plane of the 3d dataObject is registered as single texture. Create the tags 'wrapT', 'wrapS', 'MinFilter' or 'MagFilter' \n\
to control the repeatability of every texture or its interpolation method. \n\
\n\
Allowed values for these tags are: \n\
\n\
* 'MagFilter' (interpolation used when the texture is smaller than the window size): 'GL_NEAREST' (default) or 'GL_LINEAR' \n\
* 'MinFilter' (interpolation used when the texture is bigger than the window size): 'GL_NEAREST' (default), 'GL_LINEAR', 'GL_LINEAR_MIPMAP_NEAREST', 'GL_NEAREST_MIPMAP_NEAREST', 'GL_LINEAR_MIPMAP_LINEAR' \n\
* 'wrapT' (vertical scaling mode): 'GL_REPEAT' (default), 'SCALED', 'GL_MIRRORED_REPEAT, 'GL_CLAMP_TO_EDGE' \n\
* 'wrapS' (horizontal scaling mode): 'GL_REPEAT' (default), 'SCALED', 'GL_MIRRORED_REPEAT', 'GL_CLAMP_TO_EDGE'";
*/
    m_description = tr("Frameless window to display images using OpenGL");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr("This plugin displays a frameless window that displays one or multiple arrays using OpenGL technology. \n\
Each array is then created as texture, where the horizontal and vertical wrap property can be chosen individually. \n\
OpenGL allows a fast switch between all created textures. \n\
\n\
Per default, the window is either displayed in a second screen (fullscreen) or if only one screen is available - \n\
placed as small window in the top left corner of the main screen. Otherwise chose an appropriate x0, y0, xsize and ysize \n\
parameter at initialization. \n\
\n\
In order to assign textures, use the exec-function 'addTextures' and pass a 2d or 3d dataObject, where in the latter case \n\
every plane of the 3d dataObject is registered as single texture. Create the tags 'wrapT', 'wrapS', 'MinFilter' or 'MagFilter' \n\
to control the repeatability of every texture or its interpolation method. \n\
\n\
Allowed values for these tags are: \n\
\n\
* 'MagFilter' (interpolation used when the texture is smaller than the window size): 'GL_NEAREST' (default) or 'GL_LINEAR' \n\
* 'MinFilter' (interpolation used when the texture is bigger than the window size): 'GL_NEAREST' (default), 'GL_LINEAR', 'GL_LINEAR_MIPMAP_NEAREST', 'GL_NEAREST_MIPMAP_NEAREST', 'GL_LINEAR_MIPMAP_LINEAR' \n\
* 'wrapT' (vertical scaling mode): 'GL_REPEAT' (default), 'SCALED', 'GL_MIRRORED_REPEAT, 'GL_CLAMP_TO_EDGE' \n\
* 'wrapS' (horizontal scaling mode): 'GL_REPEAT' (default), 'SCALED', 'GL_MIRRORED_REPEAT', 'GL_CLAMP_TO_EDGE'");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("LGPL");
    m_aboutThis = tr(GITVERSION);
    m_callInitInNewThread = false;

    paramVal = ito::Param("x0", ito::ParamBase::Int, -4096, 4096, 0, tr("x0 position of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, -4096, 4096, 0, tr("y0 position of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("xsize", ito::ParamBase::Int, 0, 4096, 0, tr("width of window, if 0 (default) the window is positioned in the second screen or gets a default width of 100px if no second screen is available.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("ysize", ito::ParamBase::Int, 0, 4096, 0, tr("height of window, if 0 (default) the window is positioned in the second screen or gets a default height of 100px if no second screen is available.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("lut", ito::ParamBase::CharArray, nullptr, tr("Lookup table for a gamma correction with 256 values. If given, the gamma correction will be enabled (default: off) and the projected values are then modified with lut[value].").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
GLDisplayInterface::~GLDisplayInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
/** constructor of the GLDisplay class
*
*    the openGL window is opened here, based on a qgl widget.
*/
GLDisplay::GLDisplay() :
    m_pWindow(nullptr)
{
    QDesktopWidget *qdesk = QApplication::desktop();
    int scount = qdesk->screenCount();
    int maxwidth = 0, maxheight = 0, maxx0 = -4096, minx0 = 4096, maxy0 = -4096, miny0 = 4096;
    int defx0 = 0, defy0 = 0, defwidth = 3, defheight = 3;
    QRect geometry;

    for (int num = 0; num < scount; num++)
    {
        geometry = qdesk->screenGeometry(num);

        if (geometry.width() > maxwidth)
        {
            maxwidth = geometry.width();
        }

        if (geometry.height() > maxheight)
        {
            maxheight = geometry.height();
        }

        if (geometry.x() + geometry.width() > maxx0)
        {
            maxx0 = geometry.x() + geometry.width();
        }

        if (geometry.y() + geometry.height() > maxy0)
        {
            maxy0 = geometry.y() + geometry.height();
        }

        if (geometry.x() < minx0)
        {
            minx0 = geometry.x();
        }

        if (geometry.y() < miny0)
        {
            miny0 = geometry.y();
        }
    }

    if (scount > 1)
    {
        int prjscreen = 0;

        while (prjscreen == qdesk->primaryScreen())
        {
            prjscreen++;
        }

        geometry = qdesk->screenGeometry(prjscreen);
        defheight = geometry.height();
        defwidth = geometry.width();
        defx0 = geometry.x();
        defy0 = geometry.y();
    }
    else
    {
//        defheight = maxheight / 2;
//        defwidth = maxwidth / 2;
        defheight = 100;
        defwidth = 100;
        defx0 = geometry.x() + maxwidth - defwidth;
        defy0 = geometry.y();
    }

    qRegisterMetaType<QVector<unsigned char> >("QVector<unsigned char>&");
    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>() << ito::Param("meanGrayValues", ito::ParamBase::DoubleArray | ito::ParamBase::In, nullptr, tr("mean grey values from intensity calibration").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("calcLut", pMand, pOpt, pOut, tr("Calculate lookup-table for the calibration between projected grayvalue and the registered camera intensity (maps 256 gray-values to its respective mean ccd values, see parameter 'lut')"));

    pMand = QVector<ito::Param>() << ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("absolute filename of the file where the grabbing image should be saved").toLatin1().data());
    registerExecFunc("grabFramebuffer", pMand, pOpt, pOut, tr("grab the current OpenGL frame as image and saves it to the given filename. The image format is guessed from the suffix of the filename (default QImage formats supported)"));

    pMand = QVector<ito::Param>() << ito::Param("textures", ito::ParamBase::DObjPtr | ito::ParamBase::In, nullptr, tr("two or three dimensional data object with the texture(s) to add to the stack of textures.").toLatin1().data());
    registerExecFunc("addTextures", pMand, pOpt, pOut, tr("method to add further textures"));

    pMand << ito::Param("firstTextureIndex", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("first index (zero-based) of given texture that is replaced. If a 3D data object is given, the following textures are replaced, too.").toLatin1().data());
    registerExecFunc("editTextures", pMand, pOpt, pOut, tr("method to edit existing textures and replace them by a new data object"));

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register exec functions done

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "GLDisplay", tr("name of the plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("color", ito::ParamBase::Int, 0, 3, 3, tr("0: Red, 1: Green, 2: Blue, 3: White").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, minx0, maxx0, defx0, tr("x0 position of display window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("y0", ito::ParamBase::Int, miny0, maxy0, defy0, tr("y0 position of display window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("xsize", ito::ParamBase::Int, 3, maxwidth, defwidth, tr("width of window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("ysize", ito::ParamBase::Int, 3, maxheight, defheight, tr("height of window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("currentIdx", ito::ParamBase::Int, 0, 0, 0, tr("Index of currently displayed image [0..)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numImages", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 0, 0, tr("Number of different images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gamma", ito::ParamBase::Int, 0, 1, 0, tr("0: disable gamma correction, 1: enable gamma correction; default disable (see also 'lut')").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lut", ito::ParamBase::CharArray, nullptr, tr("Lookup table for a gamma correction with 256 values. The gamma correction itself is en-/disabled via parameter 'gamma'. If enabled, the value to display is modified by lut[value]. Per default the lut is a 1:1 relation.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //initialize m_lut with default values (1:1 relation)
    char lut[256];
    for (int i = 0; i < 256; ++i) lut[i] = i;
    m_params["lut"].setVal<char*>(lut, 256);

    constructionResult = ito::retOk;

    //set QSurfaceFormat
    QSurfaceFormat format;

    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setStereo(false);
    format.setSwapInterval(1);

    if (format.swapInterval() != -1)
    {
        format.setSwapInterval(0);
    }

    format.setVersion(2, 0);
    format.setProfile(QSurfaceFormat::CoreProfile);

    if (!constructionResult.containsError())
    {
        m_pWindow = new GLWindow(nullptr, Qt::Window|Qt::MSWindowsOwnDC|Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        m_pWindow->setFormat(format);
        
        m_pWindow->setCursor(Qt::BlankCursor);
        m_pWindow->setWindowTitle("GLDisplay");
        m_pWindow->move(defx0, defy0);
        m_pWindow->resize(defwidth, defheight);

        m_pWindow->show();

        if (hasGuiSupport())
        {
            //now create dock widget for this plugin
            DockWidgetGLDisplay *DispWinWid = new DockWidgetGLDisplay(this);
            Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
            QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
            createDockWidget(QString(m_params["name"].getVal<const char*>()), features, areas, DispWinWid);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GLDisplay::~GLDisplay()
{
    if (m_pWindow)
    {
        m_pWindow->shutdown();
        m_pWindow->deleteLater();
        //delete(m_pWindow);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplay::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (QString::compare(key, "currentIdx", Qt::CaseInsensitive) == 0)
        {
            //it->setVal<int>(m_pWindow->getOrientationClearedCurImg());
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "numImages", Qt::CaseInsensitive) == 0)
        {
            //it->setVal<int>(m_pWindow->getNumGrayImages());
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "lut", Qt::CaseInsensitive) == 0)
        {
            if (hasIndex == false) //return the entire lut array
            {
                *val = it.value();
            }
            else
            {
                ito::Param itemParam;
                retValue += apiGetItemFromParamArray(*it, index, itemParam); //extracts item from array and checks if index is out of bound...

                if (!retValue.containsError())
                {
                    *val = itemParam;
                }
            }
        }
        else
        {
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
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
ito::RetVal GLDisplay::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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

        //readonly are 'name', 'numgraybits', 
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
        if (QString::compare(key, "lut", Qt::CaseInsensitive) == 0)
        {
            if (!hasIndex)
            {
                if ((*val).getLen() != 256)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("lut has wrong size, 256 values required!").toLatin1().data());
                }
                else
                {
                    QVector<unsigned char> lutVals(256);
                    memcpy(lutVals.data(), val->getVal<char*>(), 256 * sizeof(unsigned char));
                    QMetaObject::invokeMethod(m_pWindow, "setLUT", Qt::BlockingQueuedConnection, Q_ARG(QVector<unsigned char>&, lutVals));
                    it->setVal<char*>(val->getVal<char*>(), 256);
                }
            }
            else
            {
                int lutLen = it->getLen();

                if (index < 0 || index >= lutLen)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("Currently the lut only has %i values, therefore the index must be in the range [0,%i]").toLatin1().data(), lutLen, lutLen-1);
                }
                else
                {
                    //change one single entry
                    QVector<unsigned char> lutVals(lutLen);
                    unsigned char *gwptr = (unsigned char*)it->getVal<char*>(); //stored values in m_params
                    gwptr[index] = (unsigned char)val->getVal<char>();
                    memcpy(lutVals.data(), gwptr, sizeof(unsigned char) * lutLen);
                    QMetaObject::invokeMethod(m_pWindow, "setLUT", Qt::BlockingQueuedConnection, Q_ARG(QVector<unsigned char>&, lutVals));
                }
            }
        }
        else if (QString::compare(key, "gamma", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "enableGammaCorrection", Qt::BlockingQueuedConnection, Q_ARG(bool, val->getVal<int>() > 0));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "color", Qt::CaseInsensitive) == 0)
        {
            QColor color;
            switch (val->getVal<int>())
            {
            case 0:
                color = Qt::red;
                break;
            case 1:
                color = Qt::green;
                break;
            case 2:
                color = Qt::blue;
                break;
            default:
                color = Qt::white;
                break;
            }
            QMetaObject::invokeMethod(m_pWindow, "setColor", Qt::BlockingQueuedConnection, Q_ARG(QColor, color));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "currentIdx", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "setCurrentTexture", Qt::BlockingQueuedConnection, Q_ARG(int, val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "x0", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "setPos", Qt::BlockingQueuedConnection, Q_ARG(int, val->getVal<int>()), Q_ARG(int,  m_params["y0"].getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "y0", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "setPos", Qt::BlockingQueuedConnection, Q_ARG(int, m_params["x0"].getVal<int>()), Q_ARG(int,  val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "xsize", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "setSize", Qt::BlockingQueuedConnection, Q_ARG(int, val->getVal<int>()), Q_ARG(int,  m_params["ysize"].getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "ysize", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(m_pWindow, "setSize", Qt::BlockingQueuedConnection, Q_ARG(int, m_params["xsize"].getVal<int>()), Q_ARG(int,  val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
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
ito::RetVal GLDisplay::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = constructionResult;

    if (!retval.containsError())
    {
        // mandatory and optional parameters
        if (paramsMand == nullptr || paramsOpt == nullptr)
        {
            retval += ito::RetVal(ito::retError, 0, tr("mandatory or optional parameters vector not initialized!!").toLatin1().data());
        }

        if (m_pWindow)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(m_pWindow, "getErrors", Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
            if (locker->wait(2000))
            {
                retval += locker->returnValue;
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("timeout getting initialization status of OpenGL display").toLatin1().data());
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("OpenGL display window is not available").toLatin1().data());
        }
    }

    if (!retval.containsError())
    {
        setIdentifier(QString::number(getID()));

        int x0, y0, xsize, ysize;
        ito::IntMeta *roiMeta;

        if (paramsOpt->at(2).getVal<int>() > 0 && paramsOpt->at(3).getVal<int>() > 0)
        {
            roiMeta = (ito::IntMeta*)(m_params["x0"].getMeta());
            x0 = paramsOpt->at(0).getVal<int>();
            x0 = qBound<int>(roiMeta->getMin(), x0, roiMeta->getMax());

            roiMeta = (ito::IntMeta*)(m_params["y0"].getMeta());
            y0 = paramsOpt->at(1).getVal<int>();
            y0 = qBound<int>(roiMeta->getMin(), y0, roiMeta->getMax());

            roiMeta = (ito::IntMeta*)(m_params["xsize"].getMeta());
            xsize = paramsOpt->at(2).getVal<int>();
            xsize = qBound<int>(roiMeta->getMin(), xsize, roiMeta->getMax());

            roiMeta = (ito::IntMeta*)(m_params["ysize"].getMeta());
            ysize = paramsOpt->at(3).getVal<int>();
            ysize = qBound<int>(roiMeta->getMin(), ysize, roiMeta->getMax());

            m_params["x0"].setVal<int>(x0);
            m_params["y0"].setVal<int>(y0);
            m_params["xsize"].setVal<int>(xsize);
            m_params["ysize"].setVal<int>(ysize);
            
            QMetaObject::invokeMethod(m_pWindow, "setPosAndSize", Q_ARG(int, x0), Q_ARG(int, y0), Q_ARG(int, xsize), Q_ARG(int, ysize));
        }

        int lutLen = paramsOpt->at(4).getLen();

        if (lutLen > 0)
        {
            if (lutLen != 256)
            {
                retval += ito::RetVal(ito::retError, 0, tr("You need to pass a lut with 256 values").toLatin1().data());
            }
            else
            {
                QVector<unsigned char> lutVals(256);
                memcpy(lutVals.data(), paramsOpt->at(4).getVal<char*>(), 256 * sizeof(unsigned char));
                m_params["lut"].setVal<char*>(paramsOpt->at(4).getVal<char*>(), 256);
                QMetaObject::invokeMethod(m_pWindow, "setLUT", Q_ARG(QVector<unsigned char>&, lutVals));
                m_params["gamma"].setVal<int>(1);
                QMetaObject::invokeMethod(m_pWindow, "enableGammaCorrection", Q_ARG(bool, true));
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplay::close(ItomSharedSemaphore *waitCond)
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
void GLDisplay::numberOfImagesChanged(int numImg, int numGray, int numCos)
{
    m_params["phaseshift"].setVal<int>(numCos);
    m_params["numgraybits"].setVal<int>(numGray);
    static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(numImg);
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal GLDisplay::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogGLDisplay(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplay::interpolateLUT(QVector<double> &grayvalues, QVector<unsigned char> &lut)
{
    ito::RetVal retval = ito::retOk;

    if (grayvalues.size() < 64)
    {
        retval = ito::RetVal(ito::retError, 0, tr("insufficient gray values").toLatin1().data());
    }

    double minval = 10000;
    double maxval = -1;

    //calc min and max grayvalue, which has been given to this method as parameter
    for (int gval = 0; gval < grayvalues.size(); gval++)
    {
        if (grayvalues[gval] < minval)
        {
            minval = grayvalues[gval];
        }
        if (grayvalues[gval] > maxval)
        {
            maxval = grayvalues[gval];
        }
    }

    //normGVals is vector where the contrast of each gray value is stored (normalized gray value: 0<=value<=1)
    QVector<double> normGVals;
    normGVals.resize(grayvalues.size());
    for (int gval = 0; gval < grayvalues.size(); gval++)
    {
        normGVals[gval] = (grayvalues[gval] - minval) / (maxval - minval);
    }

    lut.resize(256);
    double fak = 255.0 / (double)normGVals.size();
    for (int gval = 0; gval < 256; gval++)
    {
        double val = gval / 255.0;
        for (int n = 1; n < normGVals.size(); n++)
        {
            if ((normGVals[n - 1] <= val) && (normGVals[n] >= val))
            {
                lut[gval] = floor((n + (val - normGVals[n - 1]) / (normGVals[n] - normGVals[n - 1])) * fak + 0.5);
            }
        }
    }
    normGVals.clear();
    lut[0] = 0;
    lut[255] = 255;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLDisplay::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = nullptr;
    ito::ParamBase *param2 = nullptr;
    ito::ParamBase *param3 = nullptr;
    QVector<QPair<int, QByteArray> > lastError;

    if (funcName == "calcLut")
    {
        param1 = ito::getParamByName(&(*paramsMand), "meanGrayValues", &retValue);

        if (!retValue.containsError())
        {
            QVector<double> grayVals;
            QVector<unsigned char> lutVals;
            int len = param1->getLen();
            double *value = param1->getVal<double *>();
            lutVals.resize(256);
            grayVals.reserve(len);
            for (int n = 0; n < len; n++)
            {
                grayVals.append(value[n]);
            }
            retValue += interpolateLUT(grayVals, lutVals);
            if (!retValue.containsError())
            {
                QMetaObject::invokeMethod(m_pWindow, "setLUT", Qt::BlockingQueuedConnection, Q_ARG(QVector<unsigned char>&, lutVals));
                retValue += m_params["lut"].setVal<char*>((char*)lutVals.data(),256);
                emit parametersChanged(m_params);
            }
        }
    }
    else if (funcName == "addTextures")
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        const ito::DataObject *dObj = paramsMand->at(0).getVal<const ito::DataObject*>();
        QSharedPointer<int> nrOfTextures(new int);
        QMetaObject::invokeMethod(m_pWindow, "addOrEditTextures", Q_ARG(ito::DataObject, *dObj), Q_ARG(QSharedPointer<int>, nrOfTextures), Q_ARG(int, -1), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        if (locker.getSemaphore()->wait(5000))
        {
            retValue += locker.getSemaphore()->returnValue;
            m_params["numImages"].setVal<int>(*nrOfTextures);
            ((ito::IntMeta*)m_params["numImages"].getMeta())->setMin(*nrOfTextures);
            ((ito::IntMeta*)m_params["numImages"].getMeta())->setMax(*nrOfTextures);
            ((ito::IntMeta*)m_params["currentIdx"].getMeta())->setMax(*nrOfTextures-1);
            emit parametersChanged(m_params);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout while adding textures").toLatin1().data());
        }
    }
    else if (funcName == "editTextures")
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        const ito::DataObject *dObj = paramsMand->at(0).getVal<const ito::DataObject*>();
        int index = paramsMand->at(1).getVal<int>();
        QSharedPointer<int> nrOfTextures(new int);
        QMetaObject::invokeMethod(m_pWindow, "addOrEditTextures", Q_ARG(ito::DataObject, *dObj), Q_ARG(QSharedPointer<int>, nrOfTextures), Q_ARG(int, index), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        if (locker.getSemaphore()->wait(5000))
        {
            retValue += locker.getSemaphore()->returnValue;
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout while editing textures").toLatin1().data());
        }
    }
    else if (funcName == "grabFramebuffer")
    {
        QString filename = paramsMand->at(0).getVal<char*>();

        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pWindow, "grabFramebuffer", Q_ARG(QString, filename), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        if (locker.getSemaphore()->wait(5000))
        {
            retValue += locker.getSemaphore()->returnValue;
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout while grabbing current OpenGL frame").toLatin1().data());
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("function name '%s' does not exist").toLatin1().data(), funcName.toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = nullptr;
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void GLDisplay::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
