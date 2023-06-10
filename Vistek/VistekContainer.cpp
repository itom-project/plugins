/* ********************************************************************
    Plugin "Vistek" for itom software
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

#include "VistekContainer.h"

#include <qdir.h>
#include <qcoreapplication.h>

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class VistekContainer
    \brief Helper-Class for the Vistek.

    This class manages the camera container, which is the same for all instances of Vistek.
*/

//----------------------------------------------------------------------------------------------------------------------------------

/*static*/ VistekContainer * VistekContainer::getInstance(void)
{
    static VistekContainerSingleton w;
    if (VistekContainer::m_pVistekContainer == NULL)
    {
        #pragma omp critical
        {
            if (VistekContainer::m_pVistekContainer == NULL)
            {
                VistekContainer::m_pVistekContainer = new VistekContainer();
            }
        }
    }
    return VistekContainer::m_pVistekContainer;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Initializes the camera container, if it is not initialized
/*!
    If the camera container is not initialized, this is done here. Additionally the SDK dll for the Vistek will be loaded and version-checked.
    The connected cameras are dicovered. Some of their parameters are read from the cams.
*/
ito::RetVal VistekContainer::initCameraContainer()
{
    ito::RetVal retval;
    SVGigE_RETURN svsReturn;

    QMutexLocker locker(&m_mutex);

    if(m_initialized == false)
    {
        if( m_camClient != SVGigE_NO_CLIENT) // Check if container client handle already exists
        {
            std::cout << "Handle already existing." << std::endl;
            return ito::retOk;
        }
        else
        {
            // Get the path to the plugin directory
            QDir dllDir = QCoreApplication::applicationDirPath();
            if( !dllDir.cd("plugins/Vistek") )
            {
                dllDir.cdUp();
                dllDir.cd("plugins/Vistek");
            }
            QString dllDir2 = QDir::cleanPath(dllDir.filePath(""));

            // Add the plugin path to the path environment variable
            char *oldpath = getenv("path");
            char *newpath = (char*)malloc(strlen(oldpath) + dllDir2.size() + 10);
            newpath[0] = 0;
            strcat(newpath, "path=");
            strcat(newpath, dllDir2.toLatin1().data());
            strcat(newpath, ";");
            strcat(newpath, oldpath);
            _putenv(newpath);
            free(newpath);

            // Force the loading of the dll
            isLoadedGigEDLL();

            // Check if the version of the dll is the same as the version of the sdk files
            SVGigE_VERSION expectedVersion;
            expectedVersion.MajorVersion = SVGigE_VERSION_MAJOR;
            expectedVersion.MinorVersion = SVGigE_VERSION_MINOR;
            expectedVersion.DriverVersion = SVGigE_VERSION_DRIVER;
            expectedVersion.BuildVersion = SVGigE_VERSION_BUILD;
            SVGigE_VERSION dllVersion;

            svsReturn = isVersionCompliantDLL(&dllVersion, &expectedVersion);

            if(svsReturn == SVGigE_DLL_VERSION_MISMATCH)
            {
                QString gotVersion = QString("%1.%2.%3.%4").arg( dllVersion.MajorVersion).arg(dllVersion.MinorVersion).arg(dllVersion.DriverVersion).arg(dllVersion.BuildVersion);
                QString expectedVersion = QString("%1.%2.%3.%4").arg( dllVersion.MajorVersion).arg(dllVersion.MinorVersion).arg(dllVersion.DriverVersion).arg(dllVersion.BuildVersion);
#ifdef _WIN64
                return ito::RetVal(ito::retError, 0, tr("SVS Vistek: dll version mismatch, got: %1, expected: %2 (64bit).").arg(gotVersion).arg(expectedVersion).toLatin1().data());
#else
                return ito::RetVal(ito::retError, 0, tr("SVS Vistek: dll version mismatch, got: %1, expected: %2.").arg(gotVersion).arg(expectedVersion).toLatin1().data());
#endif
            }
            else
            {
                retval += checkError("Check Vistek DLL", svsReturn);
            }

            if (!retval.containsError())
            {
                // Create container Handle
                std::cout << "Trying to connect via FilterDriver ... " << std::endl;
                m_camClient = CameraContainer_create(SVGigETL_TypeFilter);
                if(m_camClient == SVGigE_NO_CLIENT)
                {
                    std::cout << "connecting via FilterDriver failed.\n" << std::endl;
                    std::cout << "Trying to connect via winsock ... " << std::endl;
                    m_camClient = CameraContainer_create(SVGigETL_TypeWinsock);
                    if(m_camClient == SVGigE_NO_CLIENT)
                    {
                        return ito::RetVal(ito::retError, 0, tr("Connecting via winsock failed.").toLatin1().data());
                    }
                }
                std::cout << "done!\n" << std::endl;
            }
        }

        if (!retval.containsError())
        {
            // Discover cameras
            std::cout << "Trying to discover cameras ... " << std::endl;
            svsReturn = CameraContainer_discovery(m_camClient);
            retval += checkError("Camera discovery failed", svsReturn);

            if (!retval.containsError())
            {
                // Get the number of connected cams
                int CamCounter = 0;
                Camera_handle TempCam;
                std::cout << "Counting cameras ... " << std::endl;
                int numberOfCameras = CameraContainer_getNumberOfCameras(m_camClient);
                if( 0 == numberOfCameras )
                {
                    return ito::RetVal(ito::retError, 0, tr("No cameras detected.").toLatin1().data());
                }
                std::cout << numberOfCameras << " camera(s) detected!\n" << std::endl;

                // Read all the relevant info from each cam
                std::cout << "Fetching camera data:\n" << std::endl;
                m_cameras.clear();
                for (CamCounter = 0; CamCounter < numberOfCameras; CamCounter++)
                {
                    TempCam = CameraContainer_getCamera(m_camClient, CamCounter);
                    VistekCam cam;
                    int TempSize = 0;

                    cam.camModel =            Camera_getModelName(TempCam);
                    cam.camSerialNo =        Camera_getSerialNumber(TempCam);
                    cam.camVersion =        Camera_getDeviceVersion(TempCam);
                    cam.camIP =                Camera_getIPAddress(TempCam);
                    cam.camManufacturer =    Camera_getManufacturerName(TempCam);
                    // Doesn't work here. Maybe a Camera_openConnection(m_cam, 30) is needed before...
                    /*Camera_getSizeX(TempCam, &TempSize);
                    cam.sensorPixelsX = TempSize;
                    Camera_getSizeY(TempCam, &TempSize);
                    cam.sensorPixelsY = TempSize;*/
                    cam.started = false;

                    std::cout << "Camera SN: " << cam.camSerialNo.toLatin1().data() << " found at IP " << cam.camIP.toLatin1().data() << "\n" <<std::endl;
                    Camera_closeConnection(TempCam);
                    m_cameras << cam;
                }

                std::cout << "done!\n" << std::endl;
            }
        }

        m_initialized = true;
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor for VistekContainer
/*!
    Protected, can only be called by getInstance
    \sa getInstance
*/
VistekContainer::VistekContainer(void) :
    m_initialized(false),
    m_camClient(SVGigE_NO_CLIENT)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor for VistekContainer
/*!
    \sa VistekContainer
*/
VistekContainer::~VistekContainer(void)
{
    if (m_camClient)
    {
        CameraContainer_delete(m_camClient);
        m_camClient = SVGigE_NO_CLIENT;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Error conversion between SVGigE_RETURN and ito::RetVal
/*!
    Returns retOk if returnCode is SVGigE_SUCCESS, else retError with appropriate error message
    \return ito::RetVal
*/
ito::RetVal VistekContainer::checkError(const char *prependStr, SVGigE_RETURN returnCode)
{
    ito::RetVal retval;
    if (returnCode != SVGigE_SUCCESS)
    {
        const char *str = prependStr;
        if (prependStr == NULL)
        {
            str = "";
        }

        const char *msg = Error_getMessage(returnCode);
        if (msg)
        {
            retval += ito::RetVal::format(ito::retError,returnCode, "%s: Vistek DLL error %i '%s' occurred", str, returnCode, msg);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError,returnCode, "%s: unknown Vistek DLL error %i occurred", str, returnCode);
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Tries to find the next free camera.
/*!
    If the no camera is marked as free -1 is returned as camera number.
    \return int CameraNumber
*/
const int VistekContainer::getNextFreeCam()
{
    QMutexLocker locker(&m_mutex);
    for(int i=0;i<m_cameras.size();i++)
    {
        if(m_cameras[i].started == false)
        {
            std::cout << "Camera " << i << " selected (first free camera)\n" <<std::endl;
            m_cameras[i].started = true;
            return i;
        }
    }
    std::cout << "No free camera found\n" <<std::endl;
    return -1;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! Tries to find the camera requested by the SN string.
/*!
    If the requested camera is not found -1 is returned as camera number.
    \param [in] camSerialNo QString containing the serial number of the desired camera.
    \return int CameraNumber
*/
const int VistekContainer::getCameraBySN(const QString &camSerialNo)
{
    QMutexLocker locker(&m_mutex);
    for(int i=0;i<m_cameras.size();i++)
    {
        if(m_cameras[i].camSerialNo == camSerialNo)
        {
            if(m_cameras[i].started == false)
            {
                std::cout << "Camera " << i << " selected by SN: " << camSerialNo.toLatin1().data() << "\n" <<std::endl;
                m_cameras[i].started = true;
                return i;
            }
            else
            {
                std::cout << "Camera " << i << " selected by SN: " << camSerialNo.toLatin1().data() << " is busy, looking for alternatives...\n" <<std::endl;
                return -1;
            }
        }
    }
    std::cout << "SN: " << camSerialNo.toLatin1().data() << " not found\n" <<std::endl;
    return -1;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Resets the started flag of camera CamNo.
/*!
    If the requested camera is not found -1 is returned as camera number.
    \param [in] CamNo is the number of the camera to mark as free.
*/
void VistekContainer::freeCameraStatus(const int CamNo)
{
    QMutexLocker locker(&m_mutex);
    if (0 <= CamNo && CamNo < m_cameras.size())
        m_cameras[CamNo].started = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Getter function that returns the camera container handle.
/*!
    \return CameraContainerClient_handle
*/
CameraContainerClient_handle VistekContainer::getCameraContainerHandle()
{
    QMutexLocker locker(&m_mutex);
    return m_camClient;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Getter function that returns the camera info struct of camera CamNo.
/*!
    \param [in] CamNo is the number of the camera.
    \return VistekCam
*/
VistekCam VistekContainer::getCamInfo(int CamNo)
{
    return m_cameras[CamNo];
}
