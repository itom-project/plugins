/* ********************************************************************
    Plugin "dispWindow" for itom software
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

#ifndef PROJWINDOW_H
#define PROJWINDOW_H

#define NOMINMAX        // we need this define to remove min max macros from M$ includes, otherwise we get problems within params.h
#include <QtOpenGL/qgl.h>
#if QT_VERSION >= 0x050000
    #include <qglfunctions.h>
	#include <qopenglvertexarrayobject.h>
#endif
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

//----------------------------------------------------------------------------------------------------------------------------------
class PrjWindow : public QGLWidget
{
    Q_OBJECT

    public:
        PrjWindow(const QMap<QString, ito::Param> &params, const QGLFormat &format, QWidget *parent = 0, const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);
        ~PrjWindow();

        ito::RetVal calcLUT(QVector<double> *grayvalues, QVector<unsigned char> *lut);
        ito::RetVal setLUT(QVector<unsigned char> *lut);

        int getNumImages(void) const;
        int getOrientation(void) const {return m_orientation;};
        int getPhaseShift(void) const  {return m_phaShift;};
        int getNumGrayImages(void) const;
        int getCurImg(void) const { return m_imgNum; }
        int getOrientationClearedCurImg(void) const;
        int getGrayBitsVert(void) const { return m_grayBitsVert; }
        int getGrayBitsHoriz(void) const { return m_grayBitsHoriz; }
        unsigned char ** getCosPtrVert(void) const { return m_cosImgsVert; };
        unsigned char ** getCosPtrHoriz(void) const { return m_cosImgsHoriz; };
        unsigned char ** getGrayPtrVert(void) const { return m_grayImgsVert; };
        unsigned char ** getGrayPtrHoriz(void) const { return m_grayImgsHoriz; };

        enum InitState
        {
            unInit = 0,
            paramsValid  = 1,
            cosIsInit = 2,
            grayIsInit = 4,
            idleState = paramsValid | cosIsInit | grayIsInit,
            initFail = 128
        };
	
	protected:
		
        ito::RetVal setGamma(const int gamma);
        ito::RetVal setGammaPrj(const int gammaCol);

    private:
        QGLFormat::OpenGLVersionFlags m_glVer;
        int m_isInit;
        int m_color;
        int m_grayBitsVert;
        int m_grayBitsHoriz;
        int m_phaShift;
        int m_period;
        int m_orientation;
        int m_gamma;
        int m_imgNum;
        int m_direction;
        int m_gammaCol;
        GLuint m_texture[38];
        GLuint m_lutTex;
        unsigned char **m_cosImgsVert;
        unsigned char **m_cosImgsHoriz;
        unsigned char **m_grayImgsVert;
        unsigned char **m_grayImgsHoriz;
        QVector<unsigned char> m_lut;
#if QT_VERSION >= 0x050000
        QGLFunctions *m_glf;
		QOpenGLVertexArrayObject *m_vao;
#else
        // just a dummy pointer so we don't need to adapt the rest of the code
        char *m_glf;
		char *m_vao;
#endif
        void paintGL();
        void initializeGL();
        void resizeGL(int width, int height);
//        void paintEvent(QPaintEvent *pevent);
//        void resizeEvent(QResizeEvent *pevent);

        GLuint ProgramName;
        GLuint ArrayBufferName;
        GLuint ElementBufferName;
        GLint UniformMVP;
        GLint UniformTexture;
        GLint UniformLut;
        GLint UniformGamma;
        GLint UniformColor;
        
        // Init Function
        ito::RetVal cosineInit();
        ito::RetVal graycodeInit();
        ito::RetVal setupProjection();
        int initOGL3(const int glVer, GLuint &ProgramName, GLint &UniformMVP, GLint &UniformLut, GLint &UniformGamma,
            GLint &UniformTexture, GLint &UniformColor, GLuint &ArrayBufferName, GLuint &ElementBufferName);
        int initOGL2(const int width, const int height);

        //ito::RetVal setPeriod(const int period);
        //ito::RetVal setPhaseShift(const int phaseshift);
        //ito::RetVal setOrientation(const int orient);

        // Deleter Function
        ito::RetVal cosineExit();
        ito::RetVal graycodeExit();

    signals:
        void numberOfImagesChanged(int numImg, int numGray, int numCos); /** Signal to sent new max image counts to the parent-Plugin*/

    public slots:
        ito::RetVal setSize(int sizex, int sizey, bool reCalcGL = true);
        void setPos(int xpos, int ypos);
		ito::RetVal setColor(const int col);

        ito::RetVal shutDown(ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal configProjection(int period, int phaseShift, int orient, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal configProjectionFull(int xpos, int sizex, int ypos, int sizey, int period, int phaseShift, int orient, ItomSharedSemaphore *waitCond = NULL);

        //ito::RetVal setOrientation(const int orient);

        void enableInit() { if (!(m_isInit & paramsValid)) m_isInit |= paramsValid; };
        void disableInit() { m_isInit &= ~paramsValid; };

        ito::RetVal showFirstImg(ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal showNextImg(ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal showFirstGrayImg(ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal showFirstCosImg(ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal showImageNum(const int num);

        ito::RetVal grabFramebuffer(const QString &filename, ItomSharedSemaphore *waitCond = NULL);

    private slots:

};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PROJWINDOW_H
