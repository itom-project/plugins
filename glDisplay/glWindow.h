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

#ifndef GLWINDOW_H
#define GLWINDOW_H

#define NOMINMAX        // we need this define to remove min max macros from M$ includes, otherwise we get problems within params.h

#include <QtGlobal>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    #include <QtOpenGLWidgets/qopenglwidget.h>
#else
    #include <qopenglwidget.h>
#endif
#include <qvector.h>

#include <qopenglfunctions.h> //be careful: see https://bugreports.qt-project.org/browse/QTBUG-27408 or http://stackoverflow.com/questions/11845230/glgenbuffers-crashes-in-release-build
#include <qopenglvertexarrayobject.h>
#include <qopenglshaderprogram.h>
#include <qopenglbuffer.h>

#if  _DEBUG
    #include <qopengldebug.h>
#endif


#include "DataObject/dataobj.h"

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

//----------------------------------------------------------------------------------------------------------------------------------
class GLWindow : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWindow(QWidget *parent = 0, const Qt::WindowFlags f = Qt::Widget);
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

    QOpenGLShaderProgram shaderProgram;
    QOpenGLFunctions *m_glf;
#if _DEBUG
    QOpenGLDebugLogger *m_pLogger;
#else
    typedef int QOpenGLDebugMessage; //dummy, necessary since slot below cannot be commented in RELEASE (moc'er will not accept it)
#endif
    QOpenGLBuffer m_vertexBuffer;
    QOpenGLBuffer m_textureBuffer;
    QOpenGLVertexArrayObject *m_vao;
    QVector<QVector3D> m_vertices;
    QVector<QVector2D> m_textureCoordinates;

    QVector<ito::DataObject> m_objects;
    QVector<TextureItem> m_textures;
    int m_currentTexture;
    bool m_init;
    bool m_gammaCorrection;

    ito::RetVal m_glErrors;

public slots:
    ito::RetVal getErrors(ItomSharedSemaphore *waitCond = nullptr);
    ito::RetVal shutdown(ItomSharedSemaphore *waitCond = nullptr);
    ito::RetVal addOrEditTextures(const ito::DataObject &textures, QSharedPointer<int> nrOfTotalTextures, int firstTextureIndex = -1, ItomSharedSemaphore *waitCond = nullptr);
    ito::RetVal setColor(const QColor &color);
    ito::RetVal setClearColor(const QColor &color);
    ito::RetVal grabFramebuffer(const QString &filename, ItomSharedSemaphore *waitCond = nullptr);
    ito::RetVal setCurrentTexture(const int index);
    ito::RetVal setPos(const int &x, const int &y);
    ito::RetVal setSize(const int &width, const int &height);
    ito::RetVal setPosAndSize(int x, int y, int width, int height);
    ito::RetVal enableGammaCorrection(bool enabled); //en/disables gamma correction based on the lut values (per default, the lut values are a 1:1 relation)
    void setLUT(QVector<unsigned char> &lut); //transfers the lut values for possible gamma correction to the opengl buffer
    void onMessageLogged( QOpenGLDebugMessage message );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // GLWINDOW_H
