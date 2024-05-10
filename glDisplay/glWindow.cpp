/* ********************************************************************
    Plugin "GLDisplay" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut für Technische Optik (ITO),
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
#ifndef WIN32
    #include <unistd.h>
#endif

#include <qevent.h>
#include <qstring.h>
#include <qstringlist.h>
#include <iostream>
#include <qfileinfo.h>
#include <qdir.h>
#include <qdebug.h>

#include "common/retVal.h"

#include "glWindow.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"

//----------------------------------------------------------------------------------------------------------------------------------

//CAREFUL: With NVIDIA drivers >~ 347.xx, no command at all may stay before the #version directive (even no line break or spaces).
//         else, it will lead to the C0204 error (version directive must be first statement and must not be repeated)

//! fragment and vertex shaders for gl v2 and gl v3
//! the fragment shader multiplies input vertices with the transformation matrix MVP, the
//! fragment shader calculates the texture pixel (and color) for each pixel. In addition a
//! gamma correction can be applied using a simple lookup vector (lutarr)
const char *VERTEX_SHADER = "#version 130\n\
\
uniform mat4 MVP; \
\
in vec4 vertex; \
in vec2 textureCoordinate; \
out vec2 TexCoord;              \
\
void main(void) \
{ \
    gl_Position = MVP * vertex; \
    TexCoord = textureCoordinate; \
}";

const char *FRAGMENT_SHADER = "#version 130\n\
\
uniform sampler2D textureObject;    \
uniform vec4 color;                 \
uniform int gamma;                  \
in vec2 TexCoord;                   \
uniform vec3 lutarr[256];           \
\
out vec4 fragColor; \
\
void main(void) \
{ \
    if (gamma == 0) \
    {               \
        fragColor = color * texture(textureObject, TexCoord); \
    }               \
    else            \
    {               \
        int col = int(texture(textureObject, TexCoord).r * 255.0);  \
        fragColor = color * vec4(lutarr[col], 1.0);      \
    }           \
}";

    //texture2d is deprecated since shader language 1.3 (version 130), use texture instead




//----------------------------------------------------------------------------------------------------------------------------------
GLWindow::GLWindow(QWidget *parent, Qt::WindowFlags f)
    : QOpenGLWidget(parent, f), m_gammaCorrection(false),
    m_init(false),
#if _DEBUG
    m_pLogger(nullptr),
#endif
    m_glf(nullptr)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
GLWindow::~GLWindow()
{
#if _DEBUG
    if (m_pLogger)
    {
        delete m_pLogger;
        m_pLogger = nullptr;
    }
#endif

    m_glf = nullptr;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GLWindow::initializeGL()
{
    ito::RetVal retval;

    // Set up the rendering context, load shaders and other resources, etc.:
    m_glf = QOpenGLContext::currentContext()->functions();

    if (m_glf)
    {
        m_glf->initializeOpenGLFunctions();
    }

    m_glf->glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);

    // Make sure that textures are enabled.
    // I read that ATI cards need this before MipMapping.
    //glEnable(GL_TEXTURE_2D);

    //glActiveTexture(0); //https://bugreports.qt-project.org/browse/QTBUG-27408

    m_glf->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    if (shaderProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, VERTEX_SHADER) == false)
    {
        QString log = shaderProgram.log();
        qDebug() << "error adding vertex shader: " << log;
        retval += ito::RetVal::format(ito::retError, 0, "error adding vertex shader: %s", log.toLatin1().data());
    }
    if (shaderProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, FRAGMENT_SHADER) == false)
    {
        QString log = shaderProgram.log();
        qDebug() << "error adding fragment shader: " << log;
        retval += ito::RetVal::format(ito::retError, 0, "error adding fragment shader: %s", log.toLatin1().data());
    }

    m_vao = new QOpenGLVertexArrayObject(this);
    m_vao->create();
    m_vao->bind();

    if (shaderProgram.link() == false)
    {
        QString log = shaderProgram.log();
        qDebug() << "error linking shader program: " << log;
        retval += ito::RetVal::format(ito::retError, 0, "error linking shader program shader: %s", log.toLatin1().data());
    }

    QMatrix4x4 unityMatrix;
    unityMatrix.setToIdentity();

    //!> setting up initial gamma lut with linear response for rgb
    GLfloat templut[256][3];
    for (GLint col = 0; col < 256; col++)
    {
        templut[col][0] = col / 255.0;
        templut[col][1] = col / 255.0;
        templut[col][2] = col / 255.0;
    }

    m_vertices <<  QVector3D(-1, -1, 0) << QVector3D(1, -1, 0) << QVector3D(-1, 1, 0) << QVector3D(1, 1, 0);
    m_textureCoordinates << QVector2D(0,1) << QVector2D(1,1) << QVector2D(0,0) << QVector2D(1,0);

    if (shaderProgram.bind())
    {
        shaderProgram.setUniformValue("MVP", unityMatrix);
        shaderProgram.setUniformValue("textureObject", 0);
        shaderProgram.setUniformValue("gamma", 0);
        m_gammaCorrection = false;
        shaderProgram.setUniformValueArray("lutarr", &templut[0][0], 256, 3);
        shaderProgram.setUniformValue("color", QColor(Qt::white));
        shaderProgram.setUniformValue("gamma", 0);

        if (m_vertexBuffer.create())
        {
            m_vertexBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
            m_vertexBuffer.bind(); //use it now
            m_vertexBuffer.allocate( m_vertices.constData(), m_vertices.size() * 3 * sizeof(qreal) );
            shaderProgram.enableAttributeArray("vertex");
            shaderProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3); //load m_vertexBuffer to Shader variable 'vertex'
            QString log = shaderProgram.log();
            checkGLError();
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "OpenGL implementation does not support buffers or no OpenGL context available (error creating buffer)");
        }

        if (m_textureBuffer.create())
        {
            m_textureBuffer.setUsagePattern( QOpenGLBuffer::DynamicDraw );
            m_textureBuffer.bind(); //use it now
            m_textureBuffer.allocate( m_textureCoordinates.constData(), m_textureCoordinates.size() * 3 * sizeof(qreal) );
            shaderProgram.enableAttributeArray("textureCoordinate");
            shaderProgram.setAttributeBuffer("textureCoordinate", GL_FLOAT, 0, 3); //load m_textureBuffer to Shader variable 'vertex'
            QString log = shaderProgram.log();
            checkGLError();
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "OpenGL implementation does not support buffers or no OpenGL context available (error creating buffer)");
        }

        shaderProgram.release();
    }
    else
    {
        QString log = shaderProgram.log();
        qDebug() << "error binding shader program in initializeGL: " << log;
        retval += ito::RetVal::format(ito::retError, 0, "error binding shader program in initializeGL: %s", log.toLatin1().data());
    }



    if (!retval.containsError())
    {
        m_init = true;
    }

    m_glErrors += retval;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::shutdown(ItomSharedSemaphore *waitCond /*= nullptr*/)
{
    m_vertexBuffer.destroy();

    shaderProgram.removeAllShaders();
    shaderProgram.release();

    m_init = false;

    if (waitCond)
    {
        waitCond->release();
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GLWindow::resizeGL(int width, int height)
{
    if (height == 0)
    {
        height = 1;
    }

    glViewport(0, 0, width, height);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GLWindow::paintGL()
{
    QSize size = this->size();

    if (shaderProgram.bind() == false)
    {
        QString log = shaderProgram.log();
        qDebug() << "error binding shader program in paintGL: " << log;
        m_glErrors += ito::RetVal::format(ito::retError, 0, "error binding shader program in paintGL: %s", log.toLatin1().data());
        return;
    }

    if (m_textures.size() > 0)
    {
        m_glf->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_currentTexture = qBound(0, m_currentTexture, m_textures.size() - 1);
        const TextureItem &item = m_textures[m_currentTexture];

        //glActiveTexture(GL_TEXTURE0); //https://bugreports.qt-project.org/browse/QTBUG-27408
        m_glf->glBindTexture(GL_TEXTURE_2D, item.texture);
        m_glErrors += checkGLError();

        if (item.textureWrapS == 0)
        {
            m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); //scaled in reality, set by texture coordinates
        }
        else
        {
            m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, item.textureWrapS);
        }

        if (item.textureWrapT == 0)
        {
            m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); //scaled in reality, set by texture coordinates
        }
        else
        {
            m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, item.textureWrapT);
        }

        m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, item.textureMinFilter);
        m_glf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, item.textureMagFilter);

        m_glf->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        //glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
        checkGLError();

        qreal scaleX = (qreal)size.width() / (qreal)item.width;

        if (item.textureWrapS == 0)
        {
            scaleX = 1.0;
        }
        qreal scaleY = (qreal)size.height() / (qreal)item.height;

        if (item.textureWrapT == 0)
        {
            scaleY = 1.0;
        }

        m_textureCoordinates[0].setY(scaleY);
        m_textureCoordinates[1].setY(scaleY);
        m_textureCoordinates[3].setX(scaleX);
        m_textureCoordinates[1].setX(scaleX);

        m_vao->bind();
        //m_glErrors += checkGLError();
        shaderProgram.enableAttributeArray("vertex");
        m_textureBuffer.bind(); //use it now
        m_textureBuffer.allocate( m_textureCoordinates.constData(), m_textureCoordinates.size() * 3 * sizeof(qreal) );
        shaderProgram.enableAttributeArray("textureCoordinate");
        shaderProgram.setAttributeBuffer("textureCoordinate", GL_FLOAT, 0, 2); //load m_textureBuffer to Shader variable 'vertex'
        m_glErrors += checkGLError();

        m_glf->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        shaderProgram.disableAttributeArray("vertex");
        shaderProgram.disableAttributeArray("textureCoordinate");

        m_glf->glBindTexture(GL_TEXTURE_2D, 0);
        m_glErrors += checkGLError(); //opengl error is normal!

        //glActiveTexture(0); //https://bugreports.qt-project.org/browse/QTBUG-27408 or http://stackoverflow.com/questions/11845230/glgenbuffers-crashes-in-release-build
    }

    shaderProgram.release();
}

//----------------------------------------------------------------------------------------------------------------------------------
//firstTextureIndex == -1: new texture(s) is/are appended to the stack
//else: the given texture(s) replace the one stored at the given index position and following positions if a 3D data object is given as textures.
ito::RetVal GLWindow::addOrEditTextures(const ito::DataObject &textures, QSharedPointer<int> nrOfTotalTextures, int firstTextureIndex /*= -1*/, ItomSharedSemaphore *waitCond /*= nullptr*/)
{
    makeCurrent();

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    m_objects.append(textures);

    int width, height, nrOfItems;
    const cv::Mat *plane = nullptr;
    ito::uint32 *data = nullptr;
    const ito::uint8 *cvLinePtr;
    const ito::Rgba32 *cvLinePtrRgba;
    bool valid;
    ito::DataObjectTagType tag;
    ito::ByteArray tag_;
    bool uint8NotRgba = true;

    ito::DataObject texturesUint8;

    if (textures.getType() == ito::tRGBA32)
    {
        if (m_gammaCorrection)
        {
            retval += ito::RetVal(ito::retWarning, 0, "a rgba32 dataObject will not be properly displayed if gamma correction is True (only red-channel is used for lookup in gamma correction LUT).");
        }

        texturesUint8 = textures;
        uint8NotRgba = false;

        if (texturesUint8.getDims() == 2)
        {
            nrOfItems = 1;
            width = texturesUint8.getSize(1);
            height = texturesUint8.getSize(0);
        }
        else if (texturesUint8.getDims() == 3)
        {
            nrOfItems = texturesUint8.getSize(0);
            width = texturesUint8.getSize(2);
            height = texturesUint8.getSize(1);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "invalid texture data object");
        }
    }
    else
    {
        retval += textures.convertTo(texturesUint8, ito::tUInt8);

        if (texturesUint8.getDims() == 2)
        {
            nrOfItems = 1;
            width = texturesUint8.getSize(1);
            height = texturesUint8.getSize(0);
        }
        else if (texturesUint8.getDims() == 3)
        {
            nrOfItems = texturesUint8.getSize(0);
            width = texturesUint8.getSize(2);
            height = texturesUint8.getSize(1);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "conversion to 8bit object failed");
        }
    }

    if (nrOfItems > 0 && firstTextureIndex >= 0 && m_textures.size() < (nrOfItems + firstTextureIndex))
    {
        retval += ito::RetVal(ito::retError, 0, "texture cannot be edited since it does not exist. Use 'addTextures' to append a texture first.");
    }

    if (!retval.containsError())
    {
        m_glf->glClearColor(0.0f, 0.0f, 0.0f, 0.0f);  //black background
        m_glf->glClear(GL_COLOR_BUFFER_BIT);          //clear screen buffer

        data = new ito::uint32[width*height];


        for (int i = 0; i < nrOfItems; ++i)
        {
            plane = texturesUint8.getCvPlaneMat(i);

            if (uint8NotRgba)
            {
                for (int r = 0; r < height; ++r)
                {
                    cvLinePtr = plane->ptr(r);

                    for (int c = 0; c < width; ++c)
                    {
                        //a, b, g, r
                        data[r*width+c] = qRgba(cvLinePtr[c], cvLinePtr[c], cvLinePtr[c], 255);
                    }
                }
            }
            else
            {
                for (int r = 0; r < height; ++r)
                {
                    cvLinePtrRgba = (ito::Rgba32*)(plane->ptr(r));

                    for (int c = 0; c < width; ++c)
                    {
                        //a, b, g, r
                        data[r*width+c] = qRgba(cvLinePtrRgba[c].b, cvLinePtrRgba[c].g, cvLinePtrRgba[c].r, 255);
                    }
                }
            }

            TextureItem item;

            if (firstTextureIndex == -1) //add texture(s)
            {
                m_glf->glGenTextures(1, &(item.texture));
                this->checkGLError();
            }
            else //edit the given texture
            {
                item = m_textures[firstTextureIndex + i];
            }

            m_glf->glBindTexture(GL_TEXTURE_2D, item.texture);
            this->checkGLError();


            m_glf->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            this->checkGLError();

            qDebug() << glIsTexture(item.texture);
            m_glf->glBindTexture(GL_TEXTURE_2D, 0);

            item.textureMagFilter = GL_NEAREST;
            item.textureMinFilter = GL_NEAREST;
            item.textureWrapS = 0; //GL_REPEAT;
            item.textureWrapT = 0; //GL_REPEAT;
            item.width = width;
            item.height = height;

            tag = textures.getTag("MagFilter", valid);
            if (valid)
            {
                tag_ = tag.getVal_ToString();
                if (tag_ == "GL_NEAREST")
                {
                    item.textureMagFilter = GL_NEAREST;
                }
                else if (tag_ == "GL_LINEAR")
                {
                    item.textureMagFilter = GL_LINEAR;
                }
                else
                {
                    retval += ito::RetVal(ito::retWarning, 0, "invalid value of tag 'MagFilter'");
                }
            }

            tag = textures.getTag("MinFilter", valid);
            if (valid)
            {
                tag_ = tag.getVal_ToString();
                if (tag_ == "GL_NEAREST")
                {
                    item.textureMinFilter = GL_NEAREST;
                }
                else if (tag_ == "GL_LINEAR")
                {
                    item.textureMinFilter = GL_LINEAR;
                }
                else if (tag_ == "GL_LINEAR_MIPMAP_NEAREST")
                {
                    item.textureMinFilter = GL_LINEAR_MIPMAP_NEAREST;
                }
                else if (tag_ == "GL_NEAREST_MIPMAP_NEAREST")
                {
                    item.textureMinFilter = GL_NEAREST_MIPMAP_NEAREST;
                }
                else if (tag_ == "GL_LINEAR_MIPMAP_LINEAR")
                {
                    item.textureMinFilter = GL_LINEAR_MIPMAP_LINEAR;
                }
                else
                {
                    retval += ito::RetVal(ito::retWarning, 0, "invalid value of tag 'MinFilter'");
                }
            }

            tag = textures.getTag("wrapS", valid);
            if (valid)
            {
                tag_ = tag.getVal_ToString();
                if (tag_ == "GL_CLAMP_TO_EDGE")
                {
                    item.textureWrapS = GL_CLAMP_TO_EDGE;
                }
                else if (tag_ == "GL_REPEAT")
                {
                    item.textureWrapS = GL_REPEAT;
                }
                else if (tag_ == "GL_MIRRORED_REPEAT")
                {
                    item.textureWrapS = GL_MIRRORED_REPEAT;
                }
                else if (tag_ == "SCALED")
                {
                    item.textureWrapS = 0;
                }
                else
                {
                    retval += ito::RetVal(ito::retWarning, 0, "invalid value of tag 'wrapS'");
                }
            }

            tag = textures.getTag("wrapT", valid);
            if (valid)
            {
                tag_ = tag.getVal_ToString();
                if (tag_ == "GL_CLAMP_TO_EDGE")
                {
                    item.textureWrapT = GL_CLAMP_TO_EDGE;
                }
                else if (tag_ == "GL_REPEAT")
                {
                    item.textureWrapT = GL_REPEAT;
                }
                else if (tag_ == "GL_MIRRORED_REPEAT")
                {
                    item.textureWrapT = GL_MIRRORED_REPEAT;
                }
                else if (tag_ == "SCALED")
                {
                    item.textureWrapT = 0;
                }
                else
                {
                    retval += ito::RetVal(ito::retWarning, 0, "invalid value of tag 'wrapT'");
                }
            }

            if (firstTextureIndex == -1) //add texture(s)
            {
                m_textures.append(item);
            }
        }

        delete[] data;
        data = nullptr;
    }

    *nrOfTotalTextures = m_textures.size();

    doneCurrent();

    update();

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::grabFramebuffer(const QString &filename, ItomSharedSemaphore *waitCond /*= nullptr*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    QFileInfo finfo(filename);
    QDir filepath(finfo.canonicalPath());

    if (filepath.exists() == false)
    {
        retval += ito::RetVal::format(ito::retError,0,"folder '%s' does not exist", finfo.canonicalPath().toLatin1().data());
    }
    else
    {
        update();
        QImage shot = QOpenGLWidget::grabFramebuffer();
        bool ok = shot.save(filepath.absoluteFilePath( finfo.fileName() ) );

        if (!ok)
        {
            retval += ito::RetVal(ito::retError,0,"error while saving grabbed framebuffer");
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setColor(const QColor &color)
{
    ito::RetVal retval;
    makeCurrent();
    shaderProgram.bind();
    shaderProgram.setUniformValue("color", color);
    shaderProgram.release();
    doneCurrent();
    update();
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setClearColor(const QColor &color)
{
    ito::RetVal retval;
    makeCurrent();
    m_glf->glClearColor(color.redF(), color.greenF(), color.blueF(), color.alphaF()); //  qglClearColor(color);
    doneCurrent();
    update();
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::checkGLError()
{
    GLenum ret = m_glf->glGetError();
    ito::RetVal retval;

    switch (ret)
    {
        case GL_NO_ERROR:
            return ito::retOk;
        case GL_INVALID_ENUM:
            retval += ito::RetVal(ito::retError, ret, "An unacceptable value is specified for an enumerated argument. The offending command is ignored and has no other side effect than to set the error flag.");
            break;
        case GL_INVALID_VALUE:
            retval += ito::RetVal(ito::retError, ret, "A numeric argument is out of range. The offending command is ignored and has no other side effect than to set the error flag.");
            break;
        case GL_INVALID_OPERATION:
            retval += ito::RetVal(ito::retError, ret, "The specified operation is not allowed in the current state. The offending command is ignored and has no other side effect than to set the error flag.");
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            retval += ito::RetVal(ito::retError, ret, "The framebuffer object is not complete. The offending command is ignored and has no other side effect than to set the error flag.");
            break;
        case GL_OUT_OF_MEMORY:
            retval += ito::RetVal(ito::retError, ret, "There is not enough memory left to execute the command. The state of the GL is undefined, except for the state of the error flags, after this error is recorded.");
            break;
        case GL_STACK_UNDERFLOW:
            retval += ito::RetVal(ito::retError, ret, "An attempt has been made to perform an operation that would cause an internal stack to underflow.");
            break;
        case GL_STACK_OVERFLOW:
            retval += ito::RetVal(ito::retError, ret, "An attempt has been made to perform an operation that would cause an internal stack to overflow.");
            break;
        default:
            retval += ito::RetVal(ito::retError, ret, "an unknown opengl error occurred");
            break;
    }

    qDebug() << "OpenGL Error: " << ret << " - " << retval.errorMessage();

    return retval;
}


//---------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setCurrentTexture(const int index)
{
    makeCurrent();
    m_currentTexture = qBound(0, index, m_textures.size()-1);
    doneCurrent();
    update();
    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setPos(const int &x, const int &y)
{
    move(x, y);
    repaint();
    return ito::retOk;
}

//---------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setSize(const int &width, const int &height)
{
    ito::RetVal retval;

    resize(width, height);

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::setPosAndSize(int x, int y, int width, int height)
{
    return setPos(x, y) + setSize(width, height);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::enableGammaCorrection(bool enabled)
{
    ito::RetVal retval;
    makeCurrent();
    shaderProgram.bind();
    shaderProgram.setUniformValue("gamma", enabled ? 1 : 0);
    shaderProgram.release();
    doneCurrent();
    update();
    m_gammaCorrection = enabled;
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GLWindow::setLUT(QVector<unsigned char> &lut)
{
    makeCurrent();

    //!> setting up initial gamma lut with linear response for rgb
    GLfloat templut[256][3];
    for (int col = 0; col < 256; col++)
    {
        templut[col][0] = lut[col] / 255.0;
        templut[col][1] = lut[col] / 255.0;
        templut[col][2] = lut[col] / 255.0;
    }

    shaderProgram.bind();
    shaderProgram.setUniformValueArray("lutarr", &templut[0][0], 256, 3);
    shaderProgram.release();
    doneCurrent();

    update();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GLWindow::getErrors(ItomSharedSemaphore *waitCond/* = nullptr*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal r = m_glErrors;
    m_glErrors = ito::retOk;

    if (waitCond)
    {
        waitCond->returnValue = r;
        waitCond->release();
    }

    return r;
}

#if QT_VERSION >= 0x050100
//----------------------------------------------------------------------------------------------------------------------------------
void GLWindow::onMessageLogged( QOpenGLDebugMessage message )
{
    qDebug() << message;
}
#endif
