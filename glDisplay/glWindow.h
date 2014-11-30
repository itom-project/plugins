/* ********************************************************************
    Plugin "GLDisplay" for itom software
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

#ifndef GLWINDOW_H
#define GLWINDOW_H

#define NOMINMAX        // we need this define to remove min max macros from M$ includes, otherwise we get problems within params.h


#include <QGLWidget>
#include <qvector.h>

#if QT_VERSION >= 0x050000
    #include <qopenglfunctions.h> //be careful: see https://bugreports.qt-project.org/browse/QTBUG-27408 or http://stackoverflow.com/questions/11845230/glgenbuffers-crashes-in-release-build
    #include <qopenglvertexarrayobject.h>
    #include <qopenglshaderprogram.h>
    #include <qopenglbuffer.h>
#if  _DEBUG
	#include <qopengldebug.h>
#endif
#else
    //#include <qglfunctions.h>  //be careful: see https://bugreports.qt-project.org/browse/QTBUG-27408 or http://stackoverflow.com/questions/11845230/glgenbuffers-crashes-in-release-build
    #include <qglshaderprogram.h>
    #include <qglfunctions.h>
    
#endif

#include "DataObject/dataobj.h"

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

//----------------------------------------------------------------------------------------------------------------------------------
class GLWindow : public QGLWidget
{
    Q_OBJECT

public:
    GLWindow(const QGLFormat &format, QWidget *parent = 0, const QGLWidget *shareWidget = 0, Qt::WindowFlags f = 0);
    ~GLWindow();

protected:

    struct TextureItem
    {
        GLuint texture;
        GLint textureMinFilter;
        GLint textureMagFilter;
        GLint textureWrapS;
        GLint textureWrapT;
        int width;
        int height;
    };

    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();

    ito::RetVal checkGLError();

private:
#if QT_VERSION >= 0x050000
    QOpenGLShaderProgram shaderProgram;
    QOpenGLFunctions *m_glf;
#if _DEBUG
    QOpenGLDebugLogger *m_pLogger;
#else
    void *m_pLogger; //dummy
    typedef int QOpenGLDebugMessage; //dummy, necessary since slot below cannot be commented in RELEASE (moc'er will not accept it)
#endif
    QOpenGLBuffer m_vertexBuffer;
    QOpenGLBuffer m_textureBuffer;
    QOpenGLVertexArrayObject *m_vao;
#else
    QGLShaderProgram shaderProgram;
    void *m_vao; //dummy
    void *m_pLogger; //dummy
#endif
    QVector<QVector3D> m_vertices;
    QVector<QVector2D> m_textureCoordinates;

    QVector<ito::DataObject> m_objects;
    QVector<TextureItem> m_textures;
    int m_currentTexture;
    bool m_init;

    ito::RetVal m_glErrors;

    

public slots:
    ito::RetVal getErrors(ItomSharedSemaphore *waitCond = NULL);
    ito::RetVal shutdown();
    ito::RetVal addTextures(const ito::DataObject &textures, QSharedPointer<int> nrOfTotalTextures, ItomSharedSemaphore *waitCond = NULL);
    ito::RetVal setColor(const QColor &color);
    ito::RetVal setClearColor(const QColor &color);
    ito::RetVal grabFramebuffer(const QString &filename, ItomSharedSemaphore *waitCond = NULL);
    ito::RetVal setCurrentTexture(const int index);
    ito::RetVal setPos(const int &x, const int &y);
    ito::RetVal setSize(const int &width, const int &height);
    ito::RetVal enableGammaCorrection(bool enabled); //en/disables gamma correction based on the lut values (per default, the lut values are a 1:1 relation)
    void setLUT(QVector<unsigned char> &lut); //transfers the lut values for possible gamma correction to the opengl buffer

#if QT_VERSION >= 0x050100 //do not anything to this #if line, since the moc'er cannot read this
	void onMessageLogged( QOpenGLDebugMessage message );
#endif
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // GLWINDOW_H
