/* ********************************************************************
    Plugin "dispWindow" for itom software
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

#include "dispWindow.h"

#include "common/helperCommon.h"

#include <QtCore/QtPlugin>
#include <qdesktopwidget.h>
#include <qstringlist.h>
#include <string.h>

#ifndef WIN32
#include <unistd.h>
#endif

#include "gitVersion.h"
#include "pluginVersion.h"

Q_DECLARE_METATYPE(QVector<unsigned char>)

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
ito::RetVal DispWindowInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(DispWindow)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindowInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(DispWindow)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
/** addIn interface constructor
 *
 *    The DispWindow plugin provides a window for displaying cosine fringes and graycode images. The
 * window is topmost, frameless and uses openGL for the actual painting. The avaiable parameters
 * are:
 *        - x0, y0: window position
 *        - xsize, ysize: window size
 *        - period: cosine period in pixels (must be divideable by two and the number of phase
 * shifts
 *        - phaseshift: the number of phase shifts
 */
DispWindowInterface::DispWindowInterface()
{
    ito::Param paramVal;
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("DispWindow");

    // for the docstring, please don't set any spaces at the beginning of the line.
    /*    char docstring[] = \
    "This plugin opens a borderless window at a given position and displays horizontal or vertical
    cosine fringes including \
    various graycode fringes (for unwrapping). The visualization is done with the help of OpenGL and
    the open source library GLEW. \n\
    \n\
    For building this plugin, download (the binaries) of glew from http://glew.sourceforge.net/ and
    set the variable GLEW_DIR in CMake \
    to the corresponding folder. The necessary library will finally be copied to the lib-folder of
    itom such that an entry in the \
    environment variable path is not necessary. Please make sure, that you use always the same
    version of glew for all plugins that \ require this library.";
    */
    m_description = tr("Window for SLM/LCD-Applications");
    //    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
        "This plugin opens a borderless window at a given position and displays horizontal or vertical cosine fringes including \
various graycode fringes (for unwrapping). The visualization is done with the help of OpenGL and the open source library GLEW. \n\
\n\
For building this plugin, download (the binaries) of glew from http://glew.sourceforge.net/ and set the variable GLEW_DIR in CMake \
to the corresponding folder. The necessary library will finally be copied to the lib-folder of itom such that an entry in the \
environment variable path is not necessary. Please make sure, that you use always the same version of glew for all plugins that \
require this library.");

    m_author = "C. Kohler, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("LGPL");
    m_aboutThis = tr(GITVERSION);

    paramVal = ito::Param(
        "x0", ito::ParamBase::Int, -4096, 4096, 0, tr("x0 position of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "y0", ito::ParamBase::Int, -4096, 4096, 0, tr("y0 position of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "xsize", ito::ParamBase::Int, 3, 4096, 3, tr("height of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "ysize", ito::ParamBase::Int, 3, 4096, 3, tr("width of window").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "period", ito::ParamBase::Int, 12, nullptr, tr("cosine period in pixel").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(4, 4096, 2), true);
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "phaseshift",
        ito::ParamBase::Int,
        3,
        8,
        4,
        tr("number of total phase shifts").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param(
        "lut",
        ito::ParamBase::CharArray,
        nullptr,
        tr("Lookup table for a gamma correction with 256 values. If given, the gamma correction "
           "will be enabled (default: off) and the projected values are then modified with "
           "lut[value].")
            .toLatin1()
            .data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param(
        "dObj",
        ito::ParamBase::DObjPtr | ito::ParamBase::In,
        nullptr,
        tr("DataObject with pixel values to display").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//-------------------------------------------------------------------------------------
DispWindowInterface::~DispWindowInterface()
{
}

//-------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------
/** constructor of the DispWindow class
 *
 *    the openGL window is opened here, based on a qgl widget.
 */
DispWindow::DispWindow() : m_pWindow(nullptr)
{
    QDesktopWidget* qdesk = QApplication::desktop();
    int scount = qdesk->screenCount();
    //    int sprim = qdesk->primaryScreen();
    int maxwidth = 0, maxheight = 0, maxx0 = -4096, minx0 = 4096, maxy0 = -4096, miny0 = 4096;
    int defx0 = 0, defy0 = 0, defwidth = 3, defheight = 3;
    QRect geometry;
    for (int num = 0; num < scount; num++)
    {
        geometry = qdesk->screenGeometry(num);
        if (geometry.width() > maxwidth)
            maxwidth = geometry.width();
        if (geometry.height() > maxheight)
            maxheight = geometry.height();
        if (geometry.x() + geometry.width() > maxx0)
            maxx0 = geometry.x() + geometry.width();
        if (geometry.y() + geometry.height() > maxy0)
            maxy0 = geometry.y() + geometry.height();
        if (geometry.x() < minx0)
            minx0 = geometry.x();
        if (geometry.y() < miny0)
            miny0 = geometry.y();
    }
    if (scount > 1)
    {
        int prjscreen = 0;
        while (prjscreen == qdesk->primaryScreen())
            prjscreen++;
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

    qRegisterMetaType<QMap<QString, ito::Param>>(
        "QMap<QString, ito::Param>"); // To enable the programm to transmit parameters via signals -
                                      // slot connections
    qRegisterMetaType<QVector<unsigned char>>("QVector<unsigned char>&");

    // register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>()
        << ito::Param("meanGrayValues",
                      ito::ParamBase::DoubleArray | ito::ParamBase::In,
                      nullptr,
                      tr("mean grey values from intensity calibration").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc(
        "calcLut",
        pMand,
        pOpt,
        pOut,
        tr("Calculate lookup-table for the calibration between projected grayvalue and the "
           "registered camera intensity (maps 256 gray-values to its respective mean ccd values, "
           "see parameter 'lut')"));

    pMand = QVector<ito::Param>() << ito::Param(
                "filename",
                ito::ParamBase::String | ito::ParamBase::In,
                "",
                tr("absolute filename of the file where the grabbing image should be saved")
                    .toLatin1()
                    .data());
    registerExecFunc(
        "grabFramebuffer",
        pMand,
        pOpt,
        pOut,
        tr("grab the current OpenGL frame as image and saves it to the given filename. The image "
           "format is guessed from the suffix of the filename (default QImage formats supported)"));

    pMand = QVector<ito::Param>() << ito::Param(
                "grayValue",
                ito::ParamBase::Int | ito::ParamBase::In,
                0,
                255,
                0,
                tr("unique gray value. Depending on the projected color, all color channels are "
                   "reduced by this value [0..255]")
                    .toLatin1()
                    .data());
    registerExecFunc(
        "projectGrayValue",
        pMand,
        pOpt,
        pOut,
        tr("projects an image where all pixels have the same gray-level. This is used for the "
           "determination of the gamma correction")
            .toLatin1()
            .data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // register exec functions done

    ito::Param paramVal(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "DispWindow",
        tr("name of the plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "period",
        ito::ParamBase::Int,
        12,
        new ito::IntMeta(4, 2048, 2),
        tr("Cosine period in pixel. This must be a multiple of 2 and the number of 'phaseshift'.")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(4, 4096, 2), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "phaseshift",
        ito::ParamBase::Int,
        3,
        8,
        4,
        tr("Count of phase shifts. If this value is changed and the 'period' does not fit to the "
           "new value, the 'period' is adapted to the next possible value.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "color",
        ito::ParamBase::Int,
        0,
        3,
        3,
        tr("0: Red, 1: Green, 2: Blue, 3: White").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "orientation",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("0: vertical, 1: horizontal; default: vertical").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "x0",
        ito::ParamBase::Int,
        minx0,
        maxx0,
        defx0,
        tr("x0 position of display window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "y0",
        ito::ParamBase::Int,
        miny0,
        maxy0,
        defy0,
        tr("y0 position of display window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "xsize",
        ito::ParamBase::Int,
        3,
        maxwidth,
        defwidth,
        tr("width of window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "ysize",
        ito::ParamBase::Int,
        3,
        maxheight,
        defheight,
        tr("height of window [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "numimg",
        ito::ParamBase::Int,
        -1,
        100,
        0,
        tr("Number of current image (phase images, dark image, flatfield image, graycode images)")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "numgraybits",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        80,
        0,
        tr("Number of different images: Phaseshift + GrayCode + 2").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "gamma",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("0: disable gamma correction, 1: enable gamma correction; default disable (see also "
           "'lut')")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "lut",
        ito::ParamBase::CharArray,
        nullptr,
        tr("Lookup table for a gamma correction with 256 values. The gamma correction itself is "
           "en-/disabled via parameter 'gamma'. If enabled, the value to display is modified by "
           "lut[value]. Per default the lut is a 1:1 relation.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "dObj",
        ito::ParamBase::DObjPtr,
        nullptr,
        tr("DataObject with pixel values to display.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // initialize m_lut with default values (1:1 relation)
    char lut[256];
    for (int i = 0; i < 256; ++i)
        lut[i] = i;
    m_params["lut"].setVal<char*>(lut, 256);

    constructionResult = ito::retOk;

    // set QGLFormat
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

    format.setVersion(3, 0);
    format.setProfile(QSurfaceFormat::CoreProfile);

    qDebug() << format.majorVersion();
    qDebug() << format.minorVersion();

    if (!constructionResult.containsError())
    {
        m_pWindow = new PrjWindow(
            m_params,
            nullptr,
            Qt::Window | Qt::MSWindowsOwnDC | Qt::FramelessWindowHint |
                Qt::WindowStaysOnTopHint); // 0, 0, Qt::Window|Qt::MSWindowsOwnDC);
                                           // //Qt::Window|Qt::MSWindowsOwnDC|Qt::ScrollBarAlwaysOff

        if (m_pWindow == nullptr)
        {
            return;
        }

        m_pWindow->setFormat(format);

        m_pWindow->setCursor(Qt::BlankCursor);
        m_pWindow->setWindowTitle("DispWindow");
        m_pWindow->setPos(defx0, defy0);
        m_pWindow->resize(12, 12);
        m_pWindow->show();

        connect(
            m_pWindow,
            SIGNAL(numberOfImagesChanged(int, int, int)),
            this,
            SLOT(numberOfImagesChanged(int, int, int)));

        if (hasGuiSupport())
        {
            // now create dock widget for this plugin
            DockWidgetDispWindow* DispWinWid = new DockWidgetDispWindow(this);
            Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
            QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
                QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
            createDockWidget(
                QString(m_params["name"].getVal<char*>()), features, areas, DispWinWid);
        }
    }
}

//-------------------------------------------------------------------------------------
DispWindow::~DispWindow()
{
    if (m_pWindow)
    {
        delete (m_pWindow);
    }

    m_params.clear();
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (QString::compare(key, "orientation", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getOrientation());
            // finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "numimg", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getOrientationClearedCurImg());
            // finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "phaseshift", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getPhaseShift());
            // finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "numgraybits", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getNumGrayImages());
            // finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "lut", Qt::CaseInsensitive) == 0)
        {
            if (hasIndex == false) // return the entire lut array
            {
                *val = it.value();
            }
            else
            {
                ito::Param itemParam;
                retValue += apiGetItemFromParamArray(
                    *it,
                    index,
                    itemParam); // extracts item from array and checks if index is out of bound...

                if (!retValue.containsError())
                {
                    *val = itemParam;
                }
            }
        }
        else
        {
            // finally, save the desired value in the argument val (this is a shared pointer!)
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

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);

        // readonly are 'name', 'numgraybits',
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        //  value in m_params and whether the new type fits to the requirements of any possible
        //  meta structure.
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
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("lut has wrong size, 256 values required!").toLatin1().data());
                }
                else
                {
                    QVector<unsigned char> lutVals(256);
                    memcpy(lutVals.data(), val->getVal<char*>(), 256 * sizeof(unsigned char));
                    QMetaObject::invokeMethod(
                        m_pWindow,
                        "setLUT",
                        Qt::BlockingQueuedConnection,
                        Q_ARG(QVector<unsigned char>&, lutVals));
                    it->setVal<char*>(val->getVal<char*>(), 256);
                }
            }
            else
            {
                int lutLen = it->getLen();

                if (index < 0 || index >= lutLen)
                {
                    retValue += ito::RetVal::format(
                        ito::retError,
                        0,
                        tr("Currently the lut only has %i values, therefore the index must be in "
                           "the range [0,%i]")
                            .toLatin1()
                            .data(),
                        lutLen,
                        lutLen - 1);
                }
                else
                {
                    // change one single entry
                    QVector<unsigned char> lutVals(lutLen);
                    unsigned char* gwptr =
                        (unsigned char*)it->getVal<char*>(); // stored values in m_params
                    gwptr[index] = (unsigned char)val->getVal<char>();
                    memcpy(lutVals.data(), gwptr, sizeof(unsigned char) * lutLen);
                    QMetaObject::invokeMethod(
                        m_pWindow,
                        "setLUT",
                        Qt::BlockingQueuedConnection,
                        Q_ARG(QVector<unsigned char>&, lutVals));
                }
            }
        }
        else if (QString::compare(key, "gamma", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "enableGammaCorrection",
                Qt::BlockingQueuedConnection,
                Q_ARG(bool, val->getVal<int>() > 0));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "color", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "setColor",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "numimg", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "showImageNum",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "x0", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "setPos",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, val->getVal<int>()),
                Q_ARG(int, m_params["y0"].getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "y0", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "setPos",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, m_params["x0"].getVal<int>()),
                Q_ARG(int, val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "xsize", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "setSize",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, val->getVal<int>()),
                Q_ARG(int, m_params["ysize"].getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "ysize", Qt::CaseInsensitive) == 0)
        {
            QMetaObject::invokeMethod(
                m_pWindow,
                "setSize",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, m_params["xsize"].getVal<int>()),
                Q_ARG(int, val->getVal<int>()));
            it->copyValueFrom(&(*val));
        }
        else if (QString::compare(key, "orientation", Qt::CaseInsensitive) == 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(
                m_pWindow,
                "configProjection",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, m_params["period"].getVal<int>()),
                Q_ARG(int, m_params["phaseshift"].getVal<int>()),
                Q_ARG(int, val->getVal<int>()),
                Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            retValue += locker.getSemaphore()->returnValue;

            m_params["numgraybits"].setVal<int>(
                m_pWindow->getNumGrayImages()); // set dependend parameter
            static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())
                ->setMax(m_pWindow->getNumImages() - 1);

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
        }
        else if (QString::compare(key, "phaseshift", Qt::CaseInsensitive) == 0)
        {
            int period = m_params["period"].getVal<int>();
            // period must dividable by the number of shifts
            div_t divisor = div(period, val->getVal<int>());
            if (divisor.rem != 0)
            {
                period = 2 * val->getVal<int>() *
                    qRound((float)period / (float)(2 * val->getVal<int>()));

                retValue += ito::RetVal::format(
                    ito::retWarning,
                    0,
                    "The period of the cosine fringes (%i px) must be dividable by the number of "
                    "phaseshifts (%i). The period has been corrected to %i.",
                    m_params["period"].getVal<int>(),
                    val->getVal<int>(),
                    period);
                m_params["period"].setVal<int>(period);
            }

            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(
                m_pWindow,
                "configProjection",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, m_params["period"].getVal<int>()),
                Q_ARG(int, val->getVal<int>()),
                Q_ARG(int, m_params["orientation"].getVal<int>()),
                Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            retValue += locker.getSemaphore()->returnValue;

            m_params["numgraybits"].setVal<int>(
                m_pWindow->getNumGrayImages()); // set dependend parameter
            static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())
                ->setMax(m_pWindow->getNumImages() - 1);

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
        }
        else if (QString::compare(key, "period", Qt::CaseInsensitive) == 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(
                m_pWindow,
                "configProjection",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, val->getVal<int>()),
                Q_ARG(int, m_params["phaseshift"].getVal<int>()),
                Q_ARG(int, m_params["orientation"].getVal<int>()),
                Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            retValue += locker.getSemaphore()->returnValue;

            m_params["numgraybits"].setVal<int>(
                m_pWindow->getNumGrayImages()); // set dependend parameter
            static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())
                ->setMax(m_pWindow->getNumImages() - 1);

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
        }
        else if (QString::compare(key, "dObj", Qt::CaseInsensitive) == 0)
        {
            m_params["dObj"].setVal<ito::DataObject*>(val->getVal<ito::DataObject*>());
            m_params["numimg"].setVal<int>(-1);
            QMetaObject::invokeMethod(
                m_pWindow, "showImageNum", Qt::BlockingQueuedConnection, Q_ARG(int, -1));
            QMetaObject::invokeMethod(
                m_pWindow,
                "setDObj",
                Qt::BlockingQueuedConnection,
                Q_ARG(ito::DataObject*, val->getVal<ito::DataObject*>()));
            // it->copyValueFrom(&(*val));
        }

        // one last thing, if the value of numimg is now greater than its maximum, reset the value
        // to the maximum and set the value
        if (m_params["numimg"].getVal<int>() > m_params["numimg"].getMax())
        {
            m_params["numimg"].setVal<int>(m_params["numimg"].getMax());
            QMetaObject::invokeMethod(
                m_pWindow,
                "showImageNum",
                Qt::BlockingQueuedConnection,
                Q_ARG(int, m_params["numimg"].getVal<int>()));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(
            m_params); // send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ItomSharedSemaphoreLocker lockerTemp(new ItomSharedSemaphore());
    ito::RetVal retval = constructionResult;

    // mandatory and optional parameters
    if (paramsMand == nullptr || paramsOpt == nullptr)
    {
        retval += ito::RetVal(
            ito::retError,
            0,
            tr("mandatory or optional parameters vector not initialized!!").toLatin1().data());
    }

    if (!retval.containsError())
    {
        int x0, y0, xsize, ysize;
        if ((x0 = (*paramsOpt)[0].getVal<int>()) != 0)
        {
            m_params["x0"].setVal<int>(x0);
        }
        else
        {
            x0 = m_params["x0"].getVal<int>();
        }

        if ((y0 = (*paramsOpt)[1].getVal<int>()) != 0)
        {
            m_params["y0"].setVal<int>(y0);
        }
        else
        {
            y0 = m_params["y0"].getVal<int>();
        }

        if ((xsize = (*paramsOpt)[2].getVal<int>()) != 3)
        {
            m_params["xsize"].setVal<int>(xsize);
        }
        else
        {
            xsize = m_params["xsize"].getVal<int>();
        }
        if ((ysize = (*paramsOpt)[3].getVal<int>()) != 3)
        {
            m_params["ysize"].setVal<int>(ysize);
        }
        else
        {
            ysize = m_params["ysize"].getVal<int>();
        }

        if ((*paramsOpt)[4].getVal<int>() != 12)
        {
            m_params["period"].setVal<int>((*paramsOpt)[4].getVal<int>());
        }

        if ((*paramsOpt)[5].getVal<int>() != 4)
        {
            m_params["phaseshift"].setVal<int>((*paramsOpt)[5].getVal<int>());
        }

        m_pWindow->enableInit();

        QMetaObject::invokeMethod(
            m_pWindow,
            "configProjectionFull",
            Q_ARG(int, x0),
            Q_ARG(int, xsize),
            Q_ARG(int, y0),
            Q_ARG(int, ysize),
            Q_ARG(int, m_params["period"].getVal<int>()),
            Q_ARG(int, m_params["phaseshift"].getVal<int>()),
            Q_ARG(int, m_params["orientation"].getVal<int>()));

        int lutLen = paramsOpt->at(6).getLen();
        if (lutLen > 0)
        {
            if (lutLen != 256)
            {
                retval += ito::RetVal(ito::retError, 0, "you need to pass a lut with 256 values");
            }
            else
            {
                QVector<unsigned char> lutVals(256);
                memcpy(
                    lutVals.data(), paramsOpt->at(6).getVal<char*>(), 256 * sizeof(unsigned char));
                m_params["lut"].setVal<char*>(paramsOpt->at(6).getVal<char*>(), 256);
                QMetaObject::invokeMethod(
                    m_pWindow, "setLUT", Q_ARG(QVector<unsigned char>&, lutVals));
                m_params["gamma"].setVal<int>(1);
                QMetaObject::invokeMethod(m_pWindow, "enableGammaCorrection", Q_ARG(bool, true));
            }
        }

        setIdentifier(QString::number(getID()));
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    setInitialized(true); // init method has been finished (independent on retval)
    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QMetaObject::invokeMethod(m_pWindow, "shutDown");

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::getVal(void* data, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    int width, height;

    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(data);

    cv::Mat_<ito::uint8>* cvMat = nullptr;
    ito::uint8* dptr = nullptr;
    int phaShift;
    int img;
    unsigned char** imgSrc = nullptr;

    if (dObj->getDims() > 2)
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong z-size").toLatin1().data());
        goto end;
    }

    if ((width = m_params["xsize"].getVal<int>()) != (int)dObj->getSize(dObj->getDims() - 1))
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong x-size").toLatin1().data());
        goto end;
    }

    if ((height = m_params["ysize"].getVal<int>()) != (int)dObj->getSize(dObj->getDims() - 2))
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong y-size").toLatin1().data());
        goto end;
    }

    if (dObj->getType() != ito::tUInt8)
    {
        retval =
            ito::RetVal(ito::retError, 0, tr("wrong data type (uint8 required)").toLatin1().data());
        goto end;
    }

    cvMat = ((cv::Mat_<ito::uint8>*)dObj->get_mdata()[dObj->seekMat(0)]);
    phaShift = m_params["phaseshift"].getVal<int>();
    img = m_pWindow->getCurImg();

    if (m_params["orientation"].getVal<int>() == 0)
    {
        if (img < phaShift)
        {
            imgSrc = m_pWindow->getCosPtrVert();
            if ((imgSrc == nullptr) || (imgSrc[img] == nullptr))
            {
                retval = ito::RetVal(
                    ito::retError, 0, tr("cosine image uninitialized").toLatin1().data());
                goto end;
            }
            if (cvMat->isContinuous())
            {
                dptr = cvMat->ptr(0);
                memcpy(dptr, imgSrc[img], width * height);
            }
            else
            {
                for (int y = 0; y < height; y++)
                {
                    dptr = cvMat->ptr(y);
                    memcpy(dptr, &imgSrc[img][width * y], width);
                }
            }
        }
        else if (img < phaShift + m_pWindow->getGrayBitsVert() + 2)
        {
            imgSrc = m_pWindow->getGrayPtrVert();
            if ((imgSrc == nullptr) || (imgSrc[img - phaShift] == nullptr))
            {
                retval = ito::RetVal(
                    ito::retError, 0, tr("cosine image uninitialized").toLatin1().data());
                goto end;
            }
            if (cvMat->isContinuous())
            {
                dptr = cvMat->ptr(0);
                memcpy(dptr, imgSrc[img - phaShift], width * height);
            }
            else
            {
                for (int y = 0; y < height; y++)
                {
                    dptr = cvMat->ptr(y);
                    memcpy(dptr, &imgSrc[img - phaShift][width * y], width);
                }
            }
        }
        else
        {
            retval = ito::RetVal(
                ito::retError, 0, tr("wrong image number - internal error").toLatin1().data());
            goto end;
        }
    }
    else
    {
        if (img < phaShift)
        {
            imgSrc = m_pWindow->getCosPtrHoriz();
            if ((imgSrc == nullptr) || (imgSrc[img] == nullptr))
            {
                retval = ito::RetVal(
                    ito::retError, 0, tr("cosine image uninitialized").toLatin1().data());
                goto end;
            }
            if (cvMat->isContinuous())
            {
                dptr = cvMat->ptr(0);
                memcpy(dptr, imgSrc[img], width * height);
            }
            else
            {
                for (int y = 0; y < height; y++)
                {
                    dptr = cvMat->ptr(y);
                    memcpy(dptr, &imgSrc[img][width * y], width);
                }
            }
        }
        else if (img < phaShift + m_pWindow->getGrayBitsHoriz() + 2)
        {
            imgSrc = m_pWindow->getGrayPtrHoriz();
            if ((imgSrc == nullptr) || (imgSrc[img - phaShift] == nullptr))
            {
                retval = ito::RetVal(
                    ito::retError, 0, tr("cosine image uninitialized").toLatin1().data());
                goto end;
            }
            if (cvMat->isContinuous())
            {
                dptr = cvMat->ptr(0);
                memcpy(dptr, imgSrc[img - phaShift], width * height);
            }
            else
            {
                for (int y = 0; y < height; y++)
                {
                    dptr = cvMat->ptr(y);
                    memcpy(dptr, &imgSrc[img - phaShift][width * y], width);
                }
            }
        }
        else
        {
            retval = ito::RetVal(
                ito::retError, 0, tr("wrong image number - internal error").toLatin1().data());
            goto end;
        }
    }

end:
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::setVal(const void* /*data*/, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    if (waitCond)
    {
        waitCond->returnValue = ito::retOk;
        waitCond->release();
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------
void DispWindow::numberOfImagesChanged(int numImg, int numGray, int numCos)
{
    m_params["phaseshift"].setVal<int>(numCos);
    m_params["numgraybits"].setVal<int>(numGray);
    static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(numImg - 1);
}

//-------------------------------------------------------------------------------------
const ito::RetVal DispWindow::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogDispWindow(this, m_pWindow));
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::interpolateLUT(QVector<double>& grayvalues, QVector<unsigned char>& lut)
{
    ito::RetVal retval = ito::retOk;

    if (grayvalues.size() < 64)
    {
        retval = ito::RetVal(ito::retError, 0, tr("insufficient gray values").toLatin1().data());
    }

    double minval = 10000;
    double maxval = -1;

    // calc min and max grayvalue, which has been given to this method as parameter
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

    // normGVals is vector where the contrast of each gray value is stored (normalized gray value:
    // 0<=value<=1)
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
                lut[gval] = floor(
                    (n + (val - normGVals[n - 1]) / (normGVals[n] - normGVals[n - 1])) * fak + 0.5);
            }
        }
    }
    normGVals.clear();
    lut[0] = 0;
    lut[255] = 255;

    return retval;
}

//-------------------------------------------------------------------------------------
ito::RetVal DispWindow::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase* param1 = nullptr;
    ito::ParamBase* param2 = nullptr;
    ito::ParamBase* param3 = nullptr;
    QVector<QPair<int, QByteArray>> lastError;

    if (funcName == "calcLut")
    {
        param1 = ito::getParamByName(&(*paramsMand), "meanGrayValues", &retValue);

        if (!retValue.containsError())
        {
            QVector<double> grayVals;
            QVector<unsigned char> lutVals;
            int len = param1->getLen();
            double* value = param1->getVal<double*>();
            lutVals.resize(256);
            grayVals.reserve(len);
            for (int n = 0; n < len; n++)
            {
                grayVals.append(value[n]);
            }
            retValue += interpolateLUT(grayVals, lutVals);
            if (!retValue.containsError())
            {
                QMetaObject::invokeMethod(
                    m_pWindow,
                    "setLUT",
                    Qt::BlockingQueuedConnection,
                    Q_ARG(QVector<unsigned char>&, lutVals));
                retValue += m_params["lut"].setVal<char*>((char*)lutVals.data(), 256);
            }
        }
    }
    else if (funcName == "grabFramebuffer")
    {
        QString filename = paramsMand->at(0).getVal<char*>();

        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(
            m_pWindow,
            "grabFramebuffer",
            Q_ARG(QString, filename),
            Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        if (locker.getSemaphore()->wait(5000))
        {
            retValue += locker.getSemaphore()->returnValue;
        }
        else
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("timeout while grabbing current OpenGL frame").toLatin1().data());
        }
    }
    else if (funcName == "projectGrayValue")
    {
        m_params["numimg"].setVal<int>(-1); // set dependent parameter
        int grayValue = paramsMand->at(0).getVal<int>();

        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(
            m_pWindow,
            "setGammaPrj",
            Q_ARG(int, grayValue),
            Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        if (locker.getSemaphore()->wait(5000))
        {
            retValue += locker.getSemaphore()->returnValue;
        }
        else
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("timeout while projecting a single-color image.").toLatin1().data());
        }
    }
    else
    {
        retValue += ito::RetVal::format(
            ito::retError,
            0,
            tr("function name '%s' does not exist").toLatin1().data(),
            funcName.toLatin1().data());
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

//-------------------------------------------------------------------------------------
void DispWindow::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            connect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                getDockWidget()->widget(),
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                getDockWidget()->widget(),
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//-------------------------------------------------------------------------------------
