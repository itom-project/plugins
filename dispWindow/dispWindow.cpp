#include "dispWindow.h"

#include "common/helperCommon.h"

#include <string.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#if linux
    #include <unistd.h>
#endif

#include "pluginVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindowInterface::getAddInInst(ito::AddInBase **addInInst)
{
   *addInInst = (ito::AddInBase *) new DispWindow();
   ((DispWindow *)*addInInst)->setBasePlugin(this);
   m_InstList.append(*addInInst);

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindowInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
        delete ((DispWindow *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** addIn interface constructor
*	
*	The DispWindow plugin provides a window for displaying cosine fringes and graycode images. The window is
*	topmost, frameless and uses openGL for the actual painting. The avaiable parameters are:
*		- x0, y0: window position
*		- xsize, ysize: window size
*		- period: cosine period in pixels (must be divideable by two and the number of phase shifts
*		- phaseshift: the number of phase shifts
*/
DispWindowInterface::DispWindowInterface()
{
    ito::Param paramVal;
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("DispWindow");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugin opens a borderless window at a given position and displays horizontal or vertical cosine fringes including \
various graycode fringes (for unwrapping). The visualization is done with the help of OpenGL and the open source library GLEW. \n\
\n\
For building this plugin, download (the binaries) of glew from http://glew.sourceforge.net/ and set the variable GLEW_DIR in CMake \
to the corresponding folder. The necessary library will finally be copied to the lib-folder of itom such that an entry in the \
environment variable path is not necessary. Please make sure, that you use always the same version of glew for all plugins that \
require this library.";

    m_description = tr("Window for SLM/LCD-Applications");
    m_detaildescription = QObject::tr(docstring);
    m_author = "C. Kohler, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("LGPL");
    m_aboutThis = tr("N.A.");

    paramVal = ito::Param("x0", ito::ParamBase::Int, -4096, 4096, 0, tr("x0 position of window").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, -4096, 4096, 0, tr("y0 position of window").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("xsize", ito::ParamBase::Int, 3, 4096, 3, tr("height of window").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("ysize", ito::ParamBase::Int, 3, 4096, 3, tr("width of window").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("period", ito::ParamBase::Int, 3, 2048, 12, tr("cosine period in pixel").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("phaseshift", ito::ParamBase::Int, 3, 8, 4, tr("number of total phase shifts").toAscii().data());
    m_initParamsOpt.append(paramVal);

    unsigned char * lutVals = (unsigned char*)malloc(256 * sizeof(unsigned char));
    for (int n = 0; n < 256; n++)
    {
        lutVals[n] = n;
    }
    paramVal = ito::Param("lut", ito::ParamBase::CharArray, 256, reinterpret_cast<char*>(lutVals), tr("Lookup table").toAscii().data());
    m_initParamsOpt.append(paramVal);
    free(lutVals);
}

//----------------------------------------------------------------------------------------------------------------------------------
DispWindowInterface::~DispWindowInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(DispWindowInterface, DispWindowInterface)

//----------------------------------------------------------------------------------------------------------------------------------
/** constructor of the DispWindow class
*
*	the openGL window is opened here, based on a qgl widget.
*/
DispWindow::DispWindow()
{
    QDesktopWidget *qdesk = QApplication::desktop();
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


    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");	// To enable the programm to transmit parameters via signals - slot connections

    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>() << ito::Param("meanGrayValues", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("mean grey values from intensity calibration").toAscii().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("calcLut", pMand, pOpt, pOut, tr("Calculate lookup-table for the calibration between projected grayvalue and the registered camera intensity (maps 256 gray-values to its respective mean ccd values, see parameter 'lut')"));

	pMand = QVector<ito::Param>() << ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("absolute filename of the file where the grabbing image should be saved").toAscii().data());
	registerExecFunc("grabFramebuffer", pMand, pOpt, pOut, tr("grab the current OpenGL frame as image and saves it to the given filename. The image format is guessed from the suffix of the filename (default QImage formats supported)"));

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register exec functions done

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "DispWindow", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("period", ito::ParamBase::Int, 3, 2048, 12, tr("Cosine period").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("phaseshift", ito::ParamBase::Int, 3, 8, 4, tr("Count of phase shifts").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("color", ito::ParamBase::Int, 0, 3, 3, tr("0: Red, 1: Green, 2: Blue, 3: White").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("orientation", ito::ParamBase::Int, 0, 1, 0, tr("0: vertical, 1: horizontal; default: vertical").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, minx0, maxx0, defx0, tr("x0 position of display window [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("y0", ito::ParamBase::Int, miny0, maxy0, defy0, tr("y0 position of display window [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("xsize", ito::ParamBase::Int, 3, maxwidth, defwidth, tr("width of window [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("ysize", ito::ParamBase::Int, 3, maxheight, defheight, tr("height of window [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numimg", ito::ParamBase::Int, -1, 100, 0, tr("Number of current image (phase images, dark image, bright image, graycode images)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numgraybits", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 80, 0, tr("Number of different images: Phaseshift + GrayCode + 2").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gamma", ito::ParamBase::Int, 0, 1, 0, tr("0: disable, 1: enable; default disable").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gammaCol", ito::ParamBase::Int, 0, 255, 127, NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("lut", ito::ParamBase::CharArray, NULL, tr("Lookup table").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //set QGLFormat
    QGLFormat::OpenGLVersionFlags glVer = QGLFormat::openGLVersionFlags();
    QGLFormat fmt;

    fmt.setDoubleBuffer(0);
    fmt.setOverlay(0);
    if (fmt.swapInterval() != -1)
    {
        fmt.setSwapInterval(0);
    }

    fmt.setProfile(QGLFormat::CoreProfile);

    //if (m_glVer >= 65536)
    //{
    //    fmt.setVersion(4, 0);
    //}
    //else if (m_glVer >= 32768)
    //{
    //    fmt.setVersion(3, 3);
    //}
    //else if (m_glVer >= 16384)
    //{
    //    fmt.setVersion(3, 2);
    //}
    //else
    if (glVer >= QGLFormat::OpenGL_Version_3_1)
    {
        fmt.setVersion(3, 1);
    }
    else if (glVer >= QGLFormat::OpenGL_Version_3_0)
    {
        fmt.setVersion(3, 0);
    }
    else if (glVer >= QGLFormat::OpenGL_Version_2_1)
    {
        fmt.setVersion(2, 1);
    }
    else if (glVer >= QGLFormat::OpenGL_Version_2_0)
    {
        fmt.setVersion(2, 0);
    }

    fmt.setDepth(0);

    m_pWindow = new PrjWindow(m_params, fmt, NULL, NULL, Qt::Window|Qt::MSWindowsOwnDC|Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);//0, 0, Qt::Window|Qt::MSWindowsOwnDC); //Qt::Window|Qt::MSWindowsOwnDC|Qt::ScrollBarAlwaysOff
    if (m_pWindow == NULL)
    {
        return;
    }

    m_pWindow->setCursor(Qt::BlankCursor);
    m_pWindow->setWindowTitle("DispWindow");
    m_pWindow->setPos(defx0, defy0);
//    m_pWindow->resize(defwidth, defheight);
    m_pWindow->resize(12, 12);
    m_pWindow->show();

    bool testCon = connect(m_pWindow, SIGNAL(numberOfImagesChanged(int, int, int)), this, SLOT(numberOfImagesChanged(int, int, int)));

 //   m_pWindow->makeCurrent();
 //   const GLubyte * version = glGetString(GL_VERSION);
 //   //version = glGetString(GL_EXTENSIONS);
 //   //std::cerr << version << "\n";

    //glClear(GL_COLOR_BUFFER_BIT);	//clear screen buffer
    //glClearColor(0.0f, 0.0f, 1.0f, 0.0f);	//black background

 //   m_pWindow->doneCurrent();

//now create dock widget for this plugin
    DockWidgetDispWindow *DispWinWid = new DockWidgetDispWindow("HoloEye",m_pWindow,this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, DispWinWid);

}

//----------------------------------------------------------------------------------------------------------------------------------
DispWindow::~DispWindow()
{
    if (m_pWindow)
    {
        delete(m_pWindow);
    }
    m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindow::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        if (QString::compare(key, "orientation", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getOrientation());
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "numimg", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getOrientationClearedCurImg());
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "phaseshift", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getPhaseShift());
            //finally, save the desired value in the argument val (this is a shared pointer!)
            *val = it.value();
        }
        else if (QString::compare(key, "numgraybits", Qt::CaseInsensitive) == 0)
        {
            it->setVal<int>(m_pWindow->getNumGrayImages());
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
ito::RetVal DispWindow::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);

        //readonly are 'name', 'numgraybits', 
    }

    if(!retValue.containsError())
    {
        //here the new parameter is checked whether it's type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if(!retValue.containsError())
    {
        if (QString::compare(key, "lut", Qt::CaseInsensitive) == 0)
        {
            if (!hasIndex)
            {
                if ((*val).getLen() != 256)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("lut has wrong size, 256 values required!").toAscii().data());
                }
                else
                {
                    QVector<unsigned char> lutVals;
                    unsigned char *gwptr = (unsigned char*)val->getVal<char*>();
                    lutVals.reserve(256);
                    for (int n = 0; n < 256; n++)
                    {
                        lutVals.append(gwptr[n]);
                    }
                    m_pWindow->setLUT(&lutVals);
                    it->setVal<char *>(val->getVal<char*>(), 256);
                }
            }
            else
            {
                int lutLen = it->getLen();

                if (index < 0 || index >= lutLen)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, "Currently the lut only has %i values, therefore the index must be in the range [0,%i]", lutLen, lutLen-1);
                }
                else
                {
                    //change one single entry
                    QVector<unsigned char> lutVals(lutLen);
                    unsigned char *gwptr = (unsigned char*)it->getVal<char*>(); //stored values in m_params
                    gwptr[index] = (unsigned char)val->getVal<char>();
                    memcpy( lutVals.data(), gwptr, sizeof(unsigned char) * lutLen );
                    m_pWindow->setLUT(&lutVals);
                }
            }
        }
        else if (QString::compare(key, "gammaCol", Qt::CaseInsensitive) == 0)
        {
            it->copyValueFrom( &(*val) );
            m_params["numimg"].setVal<int>(-1); //set dependent parameter
            m_pWindow->setGammaPrj( val->getVal<int>() );
        }
        else
        {
            //in an earlier version, at first, it has been checked whether the new value is the same than the old one. If so, the setting has been aborted.

            it->copyValueFrom( &(*val) );

            if (QString::compare(key, "color", Qt::CaseInsensitive) == 0)
            {
                m_pWindow->setColor(it->getVal<int>());
            }
            else if (QString::compare(key, "orientation", Qt::CaseInsensitive) == 0)
            {
                QMetaObject::invokeMethod(m_pWindow, "setOrientation", Qt::BlockingQueuedConnection, Q_ARG(int, it->getVal<int>()));
                m_params["numgraybits"].setVal<int>(m_pWindow->getNumGrayImages()); //set dependent parameter
                static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(m_pWindow->getNumImages());
            }
            else if (QString::compare(key, "numimg", Qt::CaseInsensitive) == 0)
            {
                m_pWindow->showImageNum(it->getVal<int>());
            }
            else if (QString::compare(key, "x0", Qt::CaseInsensitive) == 0)
            {
                QMetaObject::invokeMethod(m_pWindow, "setPos", Qt::BlockingQueuedConnection, Q_ARG(int, it->getVal<int>()), Q_ARG(int,  m_params["y0"].getVal<int>()));
            }
            else if (QString::compare(key, "y0", Qt::CaseInsensitive) == 0)
            {
                QMetaObject::invokeMethod(m_pWindow, "setPos", Qt::BlockingQueuedConnection, Q_ARG(int, m_params["x0"].getVal<int>()), Q_ARG(int,  it->getVal<int>()));
            }
            else if (QString::compare(key, "xsize", Qt::CaseInsensitive) == 0)
            {
                QMetaObject::invokeMethod(m_pWindow, "setSize", Qt::BlockingQueuedConnection, Q_ARG(int, it->getVal<int>()), Q_ARG(int,  m_params["ysize"].getVal<int>()));
            }
            else if (QString::compare(key, "ysize", Qt::CaseInsensitive) == 0)
            {
                QMetaObject::invokeMethod(m_pWindow, "setSize", Qt::BlockingQueuedConnection, Q_ARG(int, m_params["xsize"].getVal<int>()), Q_ARG(int,  it->getVal<int>()));
            }
            else if (QString::compare(key, "phaseshift", Qt::CaseInsensitive) == 0 || QString::compare(key, "period", Qt::CaseInsensitive) == 0 || QString::compare(key, "orientation", Qt::CaseInsensitive) == 0)
            {
                ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
                QMetaObject::invokeMethod(m_pWindow, "configProjection", 
                                            Qt::BlockingQueuedConnection, 
                                            Q_ARG(int, m_params["period"].getVal<int>()), 
                                            Q_ARG(int, m_params["phaseshift"].getVal<int>()), 
                                            Q_ARG(int, m_params["orientation"].getVal<int>()), 
                                            Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

                retValue += locker.getSemaphore()->returnValue;

                m_params["numgraybits"].setVal<int>(m_pWindow->getNumGrayImages()); //set dependend parameter
                static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(m_pWindow->getNumImages());
            }
        }
    }

    if(!retValue.containsError())
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
ito::RetVal DispWindow::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ItomSharedSemaphoreLocker lockerTemp(new ItomSharedSemaphore());
    ito::RetVal retval = ito::retOk;

    // mandatory and optional parameters
    if (paramsMand == NULL || paramsOpt == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("mandatory or optional parameters vector not initialized!!").toAscii().data());
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
        //QMetaObject::invokeMethod(m_pWindow, "setSize", Qt::BlockingQueuedConnection, Q_ARG(int, xsize), Q_ARG(int, ysize));

        if ((*paramsOpt)[4].getVal<int>() != 12)
        {
            m_params["period"].setVal<int>((*paramsOpt)[4].getVal<int>());
        }

        if ((*paramsOpt)[5].getVal<int>() != 4)
        {
            m_params["phaseshift"].setVal<int>((*paramsOpt)[5].getVal<int>());
        }

        m_pWindow->enableInit();

        QMetaObject::invokeMethod(m_pWindow, "configProjectionFull", 
                                  Q_ARG(int, x0), Q_ARG(int, xsize),
                                  Q_ARG(int, y0), Q_ARG(int, ysize),
                                  Q_ARG(int, m_params["period"].getVal<int>()), 
                                  Q_ARG(int, m_params["phaseshift"].getVal<int>()), 
                                  Q_ARG(int, m_params["orientation"].getVal<int>()));
        //locker.getSemaphore()->wait(-1);
        //retval += lockerTemp.getSemaphore()->returnValue;
    }
    /*
    if (!retval.containsError())
    {
        m_params["numgraybits"].setVal<int>(m_pWindow->getNumGrayImages());
        static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(m_pWindow->getNumImages());
    }
    */
    /*
    m_pWindow->makeCurrent();
    if (retval != ito::retError)
    {
        retval += m_pWindow->cosineInit();
    }
    if (retval != ito::retError)
    {
        retval += m_pWindow->graycodeInit();
    }

    m_pWindow->doneCurrent();
    */

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindow::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QMetaObject::invokeMethod(m_pWindow, "shotDown");


    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindow::getVal(void *data, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval  = ito::retOk;
    int width, height;

    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(data);

    cv::Mat_<ito::uint8> *cvMat = NULL;
    ito::uint8 *dptr = NULL;
    int phaShift;
    int img;
    unsigned char **imgSrc = NULL;

    if (dObj->getDims() > 2)
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong z-size").toAscii().data());
        goto end;
    }
    if ((width = m_params["xsize"].getVal<int>()) != (int)dObj->getSize(dObj->getDims() - 1))
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong x-size").toAscii().data());
        goto end;
    }
    if ((height = m_params["ysize"].getVal<int>()) != (int)dObj->getSize(dObj->getDims() - 2))
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong y-size").toAscii().data());
        goto end;
    }
    if (dObj->getType() != ito::tUInt8)
    {
        retval = ito::RetVal(ito::retError, 0, tr("wrong data type (uint8) required").toAscii().data());
        goto end;
    }

    cvMat = ((cv::Mat_<ito::uint8> *)dObj->get_mdata()[dObj->seekMat(0)]);
    phaShift = m_params["phaseshift"].getVal<int>();
    img = m_pWindow->getCurImg();

    if (m_params["orientation"].getVal<int>() == 0)
    {
        if (img < phaShift)
        {
            imgSrc = m_pWindow->getCosPtrVert();
            if ((imgSrc == NULL) || (imgSrc[img] == NULL))
            {
                retval = ito::RetVal(ito::retError, 0, tr("cosine image uninitialized").toAscii().data());
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
            if ((imgSrc == NULL) || (imgSrc[img - phaShift] == NULL))
            {
                retval = ito::RetVal(ito::retError, 0, tr("cosine image uninitialized").toAscii().data());
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
            retval = ito::RetVal(ito::retError, 0, tr("wrong image number - internal error").toAscii().data());
            goto end;
        }
    }
    else
    {
        if (img < phaShift)
        {
            imgSrc = m_pWindow->getCosPtrHoriz();
            if ((imgSrc == NULL) || (imgSrc[img] == NULL))
            {
                retval = ito::RetVal(ito::retError, 0, tr("cosine image uninitialized").toAscii().data());
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
            if ((imgSrc == NULL) || (imgSrc[img - phaShift] == NULL))
            {
                retval = ito::RetVal(ito::retError, 0, tr("cosine image uninitialized").toAscii().data());
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
            retval = ito::RetVal(ito::retError, 0, tr("wrong image number - internal error").toAscii().data());
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindow::setVal(const void * /*data*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    if (waitCond)
    {
        waitCond->returnValue = ito::retOk;
        waitCond->release();
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
void DispWindow::numberOfImagesChanged(int numImg, int numGray, int numCos)
{
    m_params["phaseshift"].setVal<int>(numCos);
    m_params["numgraybits"].setVal<int>(numGray);
    static_cast<ito::IntMeta*>(m_params["numimg"].getMeta())->setMax(numImg);
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal DispWindow::showConfDialog(void)
{
    dialogDispWindow *confDialog = new dialogDispWindow((void*)this);
    confDialog->setVals(&m_params, m_pWindow->getNumImages());
    if (confDialog->exec())
    {
        confDialog->getVals(&m_params);
        emit parametersChanged(m_params);
    }
    delete confDialog;

    QSharedPointer<ito::Param> temp(new ito::Param(m_params["numimg"]));
    this->setParam(temp);

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DispWindow::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;
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
            retValue += m_pWindow->calcLUT(&grayVals, &lutVals);
            if (retValue != ito::retError)
            {
                retValue += m_pWindow->setLUT(&lutVals);
                retValue += m_params["lut"].setVal<char*>((char*)lutVals.data(),256);
                //retValue += paramIt->copyValueFrom(&(*val)); //store given gray-vals in parameter "calclut"
            }
        }
    }
	else if (funcName == "grabFramebuffer")
	{
		QString filename = paramsMand->at(0).getVal<char*>();

		ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pWindow, "grabFramebuffer", Q_ARG(QString, filename), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

		if(locker.getSemaphore()->wait(5000))
		{
			retValue += locker.getSemaphore()->returnValue;
		}
		else
		{
			retValue += ito::RetVal(ito::retError,0,"timeout while grabbing current OpenGL frame");
		}
	}
    else
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("function name '%s' does not exist").toAscii().data(), funcName.toAscii().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void DispWindow::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(valuesChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(valuesChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
