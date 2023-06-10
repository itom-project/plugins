/* ********************************************************************
    Plugin "PCLTools" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
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

#include "pclTools.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#define EIGEN_QT_SUPPORT
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//before we defined #define EIGEN2_SUPPORT, which also set the #define above.
//However, EIGEN2_SUPPORT leads to errors using newer Eigen libraries (Eigen2 support has been removed there)
//The "I know sparse module is not stable yet" define is only set to also compile with older Eigen libraries (e.g. 3.0.5)

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"

#include "xyzFormat.h"

#include "DataObject/dataObjectFuncs.h"
#include "common/apiFunctionsInc.h"

//#include <pcl/surface/reconstruction.h>
#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#if PCL_VERSION_COMPARE(>,1,5,1)
    #include <pcl/io/ply_io.h>

#endif

#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include "random_sample_corrected.h" //corrected version for errornous version of random_sample filter in pcl 1.6.0
#include <pcl/common/pca.h>

#if PCL_VERSION_COMPARE(>, 1, 7, 0) && PCL_VERSION_COMPARE(<, 1, 10, 0)
    #include <pcl/recognition/auxiliary.h>
    #include <pcl/recognition/ransac_based/trimmed_icp.h>
#elif PCL_VERSION_COMPARE(>=, 1, 10, 0) && PCL_VERSION_COMPARE(<, 1, 11, 0)
    #include <pcl/recognition/auxiliary.h>
    #include <pcl/recognition/trimmed_icp.h>
#elif PCL_VERSION_COMPARE(>=, 1, 11, 0)
    #include <pcl/recognition/trimmed_icp.h>
    #include <pcl/common/common.h>
#endif


#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/organized_fast_mesh.h>

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    #include <pcl/surface/simplification_remove_unused_vertices.h>
    #include <pcl/surface/poisson.h>
    #include <pcl/surface/marching_cubes_hoppe.h>
    #include <pcl/surface/marching_cubes_rbf.h>
#endif

#include "vtkImageData.h"
#include "vtkXMLImageDataWriter.h"

#include <pcl/io/impl/pcd_io.hpp>

#include "pcl/filters/impl/cylinder_clipper3D.hpp"

#include <QtCore/QtPlugin>
#include <qstring.h>
#include <qdir.h>
#include <qfileinfo.h>
#include <qfile.h>
#include <qvariant.h>
#include <qtextstream.h>
#include <qthread.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/principal_curvatures.hpp>

int PclTools::nthreads = 2;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclToolsInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PclTools)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclToolsInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(PclTools)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
PclToolsInterface::PclToolsInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("PclTools");

/*    char docstring[] = \
"This plugin contains methods for filtering, transforming, saving and loading \n\
point clouds and polygon meshes. Most methods are wrappers for functions provided \n\
by the open source project PointCloudLibrary (pcl). The function calls are usually \n\
implemented for the cloud types supported by the itom classes itom.pointCloud \n\
and itom.polygonMesh (XYZ,XYZI,XYZRGBA,XYZNormals...). \n\
\n\
This library uses also methods from the current pcl version 1.7.1, however also \n\
compiles with older versions. In this case some methods are not compiled. \n\
\n\
This plugin also covers the methods for loading and saving point clouds and polygon \n\
meshes to common formats like pcd, ply, stl, obj... Once the plugin is loaded \n\
itom in general is also able to load and save such structures using the methods provided \n\
by this plugin.";
*/
    m_description = tr("Filters and methods for pointClouds and polygonMeshes");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr("This plugin contains methods for filtering, transforming, saving and loading \n\
point clouds and polygon meshes. Most methods are wrappers for functions provided \n\
by the open source project PointCloudLibrary (pcl). The function calls are usually \n\
implemented for the cloud types supported by the itom classes itom.pointCloud \n\
and itom.polygonMesh (XYZ,XYZI,XYZRGBA,XYZNormals...). \n\
\n\
This library uses also methods from the current pcl version 1.7.1, however also \n\
compiles with older versions. In this case some methods are not compiled. \n\
\n\
This plugin also covers the methods for loading and saving point clouds and polygon \n\
meshes to common formats like pcd, ply, stl, obj... Once the plugin is loaded \n\
itom in general is also able to load and save such structures using the methods provided \n\
by this plugin.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    PclTools::nthreads  = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
PclToolsInterface::~PclToolsInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
PclTools::PclTools() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//ItomDoc_STRVAR(savePointCloud_doc, "saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii)");
    const QString PclTools::savePointCloudDOC = QObject::tr("save an itom.pointCloud object to a file\n\
\n\
The supported file formats are: \n\
\n\
* pcd (point cloud data file format from the point cloud library, binary or ascii mode possible depending on optional parameter 'mode') \n\
* ply (polygon file format or stanford triangle format, binary or ascii mode possible depending on optional parameter 'mode') \n\
* vtk (VTK point cloud format) \n\
* xyz (ascii text format where each line contains a whitespace separated list of the X, Y and Z coordinate of each point. The decimal sign is a dot, the real number precision is 6.)");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::savePointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("point cloud to save").toLatin1().data()));
    paramsMand->append(ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("complete filename (type is either read by suffix of filename or by parameter 'type'").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("mode", ito::ParamBase::String, "b", tr("mode (b=binary (default) or t=ascii, for type 'pcd' and 'ply' only)").toLatin1().data()));
    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('pcd' [default],'ply','vtk','xyz')").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::savePointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPointCloud *pointCloud = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();  //Input object
    QString filename = QString::fromLatin1((*paramsMand)[1].getVal<char*>());
    std::string filename_ = (*paramsMand)[1].getVal<const char*>(); //directly load the std::string from the given char* instead of extracting it from the latin1-str, since encoding errors can occur in case of special characters
    QString mode = (*paramsOpt)[0].getVal<char*>();
    QString type = (*paramsOpt)[1].getVal<char*>();
    bool binary_mode = true;
    int ret = 1;

    //check filename
    QFileInfo finfo(filename);
    if (type == "")
    {
        type = finfo.suffix().toLower();
        filename = finfo.absoluteFilePath();
    }

    //check type
    type = type.toLower();
    if (type != "pcd" && type != "ply" && type != "vtk" && type != "xyz")
    {
        retval += ito::RetVal(ito::retWarning, 0, tr("type was set to 'pcd', since 'pcd','ply','xyz' or 'vtk' expected").toLatin1().data());
        type = "pcd";
    }

    //check mode
    mode = mode.toLower();
    if (type == "pcd" || type == "ply")
    {
        if (mode != "b" && mode != "t")
        {
            retval += ito::RetVal(ito::retWarning, 0, tr("mode is set to 'b' since for type 'pcd' or 'ply' mode is expected to be 't' (ascii) or 'b' (binary).").toLatin1().data());
            mode = "b";
        }
        binary_mode = (mode == "b");
    }

    try
    {
        //check point cloud
        if (pointCloud->getType() == ito::pclInvalid)
        {
            retval += ito::RetVal(ito::retError, 0, tr("can not save invalid point cloud").toLatin1().data());
        }
        else if (type == "xyz")
        {
            QFile data(filename);
            if (data.open(QFile::WriteOnly | QFile::Truncate))
            {
                QTextStream out(&data);
                switch(pointCloud->getType())
                {
                case ito::pclXYZ:
                    retval += writeXYZ<pcl::PointXYZ>(*pointCloud->toPointXYZ(), out);
                    break;
                case ito::pclXYZI:
                    retval += writeXYZ<pcl::PointXYZI>(*pointCloud->toPointXYZI(), out);
                    break;
                case ito::pclXYZRGBA:
                    retval += writeXYZ<pcl::PointXYZRGBA>(*pointCloud->toPointXYZRGBA(), out);
                    break;
                case ito::pclXYZNormal:
                    retval += writeXYZ<pcl::PointNormal>(*pointCloud->toPointXYZNormal(), out);
                    break;
                case ito::pclXYZINormal:
                    retval += writeXYZ<pcl::PointXYZINormal>(*pointCloud->toPointXYZINormal(), out);
                    break;
                case ito::pclXYZRGBNormal:
                    retval += writeXYZ<pcl::PointXYZRGBNormal>(*pointCloud->toPointXYZRGBNormal(), out);
                    break;
                }
                data.close();
            }
        }
        else if (type == "pcd")
        {
            switch(pointCloud->getType())
            {
            case ito::pclXYZ:
                ret = pcl::io::savePCDFile<pcl::PointXYZ>(filename_, *pointCloud->toPointXYZ(), binary_mode);
                break;
            case ito::pclXYZI:
                ret = pcl::io::savePCDFile<pcl::PointXYZI>(filename_, *pointCloud->toPointXYZI(), binary_mode);
                break;
            case ito::pclXYZRGBA:
                ret = pcl::io::savePCDFile<pcl::PointXYZRGBA>(filename_, *pointCloud->toPointXYZRGBA(), binary_mode);
                break;
            case ito::pclXYZNormal:
                ret = pcl::io::savePCDFile<pcl::PointNormal>(filename_, *pointCloud->toPointXYZNormal(), binary_mode);
                break;
            case ito::pclXYZINormal:
                ret = pcl::io::savePCDFile<pcl::PointXYZINormal>(filename_, *pointCloud->toPointXYZINormal(), binary_mode);
                break;
            case ito::pclXYZRGBNormal:
                ret = pcl::io::savePCDFile<pcl::PointXYZRGBNormal>(filename_, *pointCloud->toPointXYZRGBNormal(), binary_mode);
                break;
            }
        }
        else if (type == "ply")
        {
#if PCL_VERSION_COMPARE(>, 1, 5, 1)
            switch(pointCloud->getType())
            {
            case ito::pclXYZ:
                ret = pcl::io::savePLYFile<pcl::PointXYZ>(filename_, *pointCloud->toPointXYZ(), binary_mode);
                break;
            case ito::pclXYZI:
                ret = pcl::io::savePLYFile<pcl::PointXYZI>(filename_, *pointCloud->toPointXYZI(), binary_mode);
                break;
            case ito::pclXYZRGBA:
                ret = pcl::io::savePLYFile<pcl::PointXYZRGBA>(filename_, *pointCloud->toPointXYZRGBA(), binary_mode);
                break;
            case ito::pclXYZNormal:
                ret = pcl::io::savePLYFile<pcl::PointNormal>(filename_, *pointCloud->toPointXYZNormal(), binary_mode);
                break;
            case ito::pclXYZINormal:
                ret = pcl::io::savePLYFile<pcl::PointXYZINormal>(filename_, *pointCloud->toPointXYZINormal(), binary_mode);
                break;
            case ito::pclXYZRGBNormal:
                ret = pcl::io::savePLYFile<pcl::PointXYZRGBNormal>(filename_, *pointCloud->toPointXYZRGBNormal(), binary_mode);
                break;
            }
    #else
            retval += ito::RetVal(ito::retError, 0, tr("ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower").toLatin1().data());
    #endif
        }
        else if (type == "vtk")
        {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::PCLPointCloud2 pc2;
            switch(pointCloud->getType())
            {
            case ito::pclXYZ:
                pcl::toPCLPointCloud2<pcl::PointXYZ>(*pointCloud->toPointXYZ(), pc2);
                break;
            case ito::pclXYZI:
                pcl::toPCLPointCloud2<pcl::PointXYZI>(*pointCloud->toPointXYZI(), pc2);
                break;
            case ito::pclXYZRGBA:
                pcl::toPCLPointCloud2<pcl::PointXYZRGBA>(*pointCloud->toPointXYZRGBA(), pc2);
                break;
            case ito::pclXYZNormal:
                pcl::toPCLPointCloud2<pcl::PointNormal>(*pointCloud->toPointXYZNormal(), pc2);
                break;
            case ito::pclXYZINormal:
                pcl::toPCLPointCloud2<pcl::PointXYZINormal>(*pointCloud->toPointXYZINormal(), pc2);
                break;
            case ito::pclXYZRGBNormal:
                pcl::toPCLPointCloud2<pcl::PointXYZRGBNormal>(*pointCloud->toPointXYZRGBNormal(), pc2);
                break;
            }
#else
            sensor_msgs::PointCloud2 pc2;
            switch(pointCloud->getType())
            {
            case ito::pclXYZ:
                pcl::toROSMsg<pcl::PointXYZ>(*pointCloud->toPointXYZ(), pc2);
                break;
            case ito::pclXYZI:
                pcl::toROSMsg<pcl::PointXYZI>(*pointCloud->toPointXYZI(), pc2);
                break;
            case ito::pclXYZRGBA:
                pcl::toROSMsg<pcl::PointXYZRGBA>(*pointCloud->toPointXYZRGBA(), pc2);
                break;
            case ito::pclXYZNormal:
                pcl::toROSMsg<pcl::PointNormal>(*pointCloud->toPointXYZNormal(), pc2);
                break;
            case ito::pclXYZINormal:
                pcl::toROSMsg<pcl::PointXYZINormal>(*pointCloud->toPointXYZINormal(), pc2);
                break;
            case ito::pclXYZRGBNormal:
                pcl::toROSMsg<pcl::PointXYZRGBNormal>(*pointCloud->toPointXYZRGBNormal(), pc2);
                break;
            }
#endif

            ret = pcl::io::saveVTKFile(filename_,pc2);
        }
    }
    catch(pcl::IOException exc)
    {
        std::string tmp = exc.detailedMessage(); //detailedMessage() is const char* in newer versions of PCL and std::string in older. Therefore this hack...
        retval += ito::RetVal::format(ito::retError, 0, tr("pointCloud could not be saved: %s").toLatin1().data(), tmp.data());
        ret = 1;
    }

    if (ret < 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("error while saving point cloud (internal error of method in point cloud library").toLatin1().data());
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::loadPointCloudDOC = QObject::tr("This filter loads point cloud data files to an itom.pointCloud object\n\
\n\
The following file formats are supported:\n\
\n\
* pcd (point cloud data file format provided from the point cloud library \n\
* ply (polygon file format also known under stanford triangle format - be careful ply can also contain polygon mesh data which is loaded using 'loadPolygonMesh'). Both ascii and binary formats are supported. \n\
* vtk (point cloud format from the vtk library\n\
* xyz (a whitespace separated ascii text file where each line contains the x, y and z coordinate of a point, e.g.: 2.546 -4.345 0.001) \n\
\n\
Usually the file format is automatically detected by the suffix of the filename. However it is also possible to indicate the \n\
type by the optional string parameter 'type'.");

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("pointCloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("loaded pointcloud").toLatin1().data()));
    paramsMand->append(ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("complete filename (type is read by suffix)").toLatin1().data()));

    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('xyz', 'pcd','ply','auto' [default, check suffix of filename])").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPointCloud *pointCloud = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    QString filename = QString::fromLatin1((*paramsMand)[1].getVal<const char*>());
    std::string filename_ = (*paramsMand)[1].getVal<const char*>(); //directly load the std::string from the given char* instead of extracting it from the latin1-str, since encoding errors can occur in case of special characters
    QString typeString = (*paramsOpt)[0].getVal<char*>();

    QString type;
    int ret = 1;

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    pcl::PCLPointCloud2 pc2;
#else
    sensor_msgs::PointCloud2 pc2;
#endif
    ito::PCLPointCloud pc;

    //check filename
    QFileInfo finfo(filename);

    if (typeString == "" || typeString == "auto")
    {
        type = finfo.suffix().toLower();
    }
    else
    {
        type = typeString;
    }

    QString filenameCanonical = finfo.canonicalFilePath();

    if (filenameCanonical == "")
    {
        QByteArray ba = filename.toLatin1();
        char *name = ba.data();
        if (name)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("filename '%s' does not exist.").toLatin1().data(), name);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("empty filename not allowed.").toLatin1().data(), name);
        }
    }
    else if (type == "xyz")
    {
        QFile data(filename);
        if (data.open(QFile::ReadOnly))
        {
            QTextStream in(&data);
            pc = ito::PCLPointCloud(ito::pclXYZ);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclpc = pc.toPointXYZ();
            retval += readXYZ<pcl::PointXYZ>(in, *(pclpc.get()));
            data.close();
        }
    }
    else if (type == "pcd")
    {
        ret = pcl::io::loadPCDFile(filename_, pc2);
    }
    else if (type == "ply")
    {
#if PCL_VERSION_COMPARE(>, 1, 5, 1)
        //be careful: ply files can be everything: therefore check if they are build by pcl:
        QFile file(QString::fromStdString(filename_));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("file '%s' could not be opened.").toLatin1().data(), filename_.data());
        }
        else
        {
            QByteArray startContent;
            int i = 0;
            while (!file.atEnd() && (++i) < 30)
            {
                startContent += file.readLine();
            }

            file.close();

            if (startContent.contains("ply") && startContent.contains("format") && \
                startContent.contains("element vertex") && !(startContent.contains("element face")))
            {
                ret = pcl::io::loadPLYFile(filename_, pc2);
            }
            else
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("file '%s' does not contain valid point cloud data.").toLatin1().data(), filename_.data());
            }
        }
#else
        retval += ito::RetVal(ito::retError, 0, tr("ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower").toLatin1().data());
#endif
    }
    /*else if (type == "vtk")
    {
        retval += ito::RetVal(ito::retError, 0, tr("vtk file format cannot be loaded (not supported)").toLatin1().data());
    }*/
    else
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("unsupported format '%s'").toLatin1().data(), type.toLatin1().data());
    }

    if (retval == ito::retOk && type != "xyz")
    {
        try
        {
            if (pcl::getFieldIndex(pc2, "x") >= 0 && pcl::getFieldIndex(pc2, "y") >= 0 && pcl::getFieldIndex(pc2, "z") >= 0)
            {
                bool rgb = (pcl::getFieldIndex(pc2, "rgb") >= 0);
                bool rgba = (pcl::getFieldIndex(pc2, "rgba") >= 0);

                //hack, since ply-files sometimes call normal_i ni. rename it now. (maybe this is fixed in pcl 1.6)
                int idx = pcl::getFieldIndex(pc2, "nx");
                if (idx >= 0)
                {
                    pc2.fields[idx].name = "normal_x";
                }
                idx = pcl::getFieldIndex(pc2, "ny");
                if (idx >= 0)
                {
                    pc2.fields[idx].name = "normal_y";
                }
                idx = pcl::getFieldIndex(pc2, "nz");
                if (idx >= 0)
                {
                    pc2.fields[idx].name = "normal_z";
                }

                bool normal = (pcl::getFieldIndex(pc2, "normal_x") >= 0 && pcl::getFieldIndex(pc2,"normal_y") >= 0 && pcl::getFieldIndex(pc2,"normal_z") >= 0 && pcl::getFieldIndex(pc2,"curvature") >= 0);
                bool intensity = (pcl::getFieldIndex(pc2, "intensity") >= 0);

                if (rgb)
                {
                    if (normal)
                    {
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZRGBNormal>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else
                    {
                        pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, tempCloud);
#else
                        pcl::fromROSMsg(pc2, tempCloud);
#endif
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZRGBA>()));
                        ito::pclHelper::PointCloudXYZRGBtoXYZRGBA(tempCloud, *cloud);
                        pc = ito::PCLPointCloud(cloud);
                    }
                }
                else if (rgba)
                {
                    if (normal)
                    {
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZRGBNormal>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else
                    {
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZRGBA>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                }
                else //no rgb
                {
                    if (normal && !intensity)
                    {
                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud((new pcl::PointCloud<pcl::PointNormal>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else if (normal && intensity)
                    {
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZINormal>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else if (!normal && intensity)
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZI>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else if (!normal && !intensity)
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud((new pcl::PointCloud<pcl::PointXYZ>()));
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
                        pcl::fromPCLPointCloud2(pc2, *cloud);
#else
                        pcl::fromROSMsg(pc2, *cloud);
#endif
                        pc = ito::PCLPointCloud(cloud);
                    }
                    else
                    {
                        retval += ito::RetVal(ito::retError, 0, tr("The loaded point cloud has an uncompatible format.").toLatin1().data());
                        pc = ito::PCLPointCloud(ito::pclInvalid);
                    }
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("The loaded point cloud does not contain x,y,z-fields.").toLatin1().data());
                pc = ito::PCLPointCloud(ito::pclInvalid);
            }
        }
        catch(pcl::InvalidConversionException exp)
        {
            const char* msg = string_to_char(exp.detailedMessage());
            retval += ito::RetVal::format(ito::retError, 0, tr("Error while loading the point cloud. Message: %s").toLatin1().data(), msg);
            pc = ito::PCLPointCloud(ito::pclInvalid);
        }
    }

    if (!retval.containsError())
    {
        *pointCloud = pc;
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString PclTools::saveVTKImageDataDOC = QObject::tr("saves a 2D or 3D uint8 or uint16 data object to a VTK imageData volume image\n\
\n\
This file format allows displaying volume data from the given 3D data object for instance using ParaView.");

//------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::saveVTKImageDataParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("dataObject", ito::ParamBase::DObjPtr, NULL, tr("data object to save (two or three dimensional, uint8 or uint16)").toLatin1().data()));
    paramsMand->append(ito::Param("filename", ito::ParamBase::String, "", tr("complete filename, ending .vti will be appended if not available").toLatin1().data()));

    paramsOpt->clear();
    ito::Param opt1("mode", ito::ParamBase::String, "b", tr("mode (b=binary (default) or t=ascii)").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "b");
    sm->addItem("t");
    opt1.setMeta(sm, true);
    paramsOpt->append(opt1);

    paramsOpt->append(ito::Param("scalarFieldName", ito::ParamBase::String, "scalars", tr("name of scalar field, e.g. 'scalars' (zero values will be transparent), 'ImageScalars' (zero values will be displayed)...").toLatin1().data()));
    paramsOpt->append(ito::Param("scalarThreshold", ito::ParamBase::Int, 0, std::numeric_limits<ito::uint16>::max(), 0, tr("values <= threshold will be set to 0 (transparent values for scalar field name 'scalars')").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::saveVTKImageData(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int sizeLimits[] = {1, 10000, 1, 10000, 1, 10000};
    const ito::DataObject *input = paramsMand->at(0).getVal<const ito::DataObject*>();
    ito::DataObject *dobj = NULL;

    if (input->getDims() == 3)
    {
        switch (input->getType())
        {
        case ito::tUInt8:
            dobj = apiCreateFromDataObject(input, 3, ito::tUInt8, sizeLimits, &retval);
            break;
        case ito::tUInt16:
            dobj = apiCreateFromDataObject(input, 3, ito::tUInt16, sizeLimits, &retval);
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, tr("dataObject must be uint8 or uint16").toLatin1().data());
            break;
        }
    }
    else if (input->getDims() == 2)
    {
        switch (input->getType())
        {
        case ito::tUInt8:
            dobj = apiCreateFromDataObject(input, 2, ito::tUInt8, sizeLimits, &retval);
            break;
        case ito::tUInt16:
            dobj = apiCreateFromDataObject(input, 2, ito::tUInt16, sizeLimits, &retval);
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, tr("dataObject must be uint8 or uint16").toLatin1().data());
            break;
        }
    }

    QString filename = QString::fromLatin1(paramsMand->at(1).getVal<char*>());
    QByteArray scalarName = paramsOpt->at(1).getVal<char*>();
    ito::uint16 t = paramsOpt->at(2).getVal<int>();

    if (filename.isEmpty())
    {
        retval += ito::RetVal(ito::retError, 0, tr("filename is empty").toLatin1().data());
    }
    else
    {
        QFileInfo fi(filename);
        if (fi.suffix().compare("vti", Qt::CaseInsensitive) != 0)
        {
            filename += ".vti";
            fi = QFileInfo(filename);
        }

        filename = fi.absoluteFilePath().toLatin1();
    }

    if (!retval.containsError())
    {
        vtkSmartPointer<vtkImageData> structuredPoints = vtkSmartPointer<vtkImageData>::New();

        int xDim, yDim;

        if (dobj->getDims() == 3)
        {
            structuredPoints->SetDimensions(dobj->getSize(2), dobj->getSize(1), dobj->getSize(0));
            structuredPoints->SetSpacing(dobj->getAxisScale(2), dobj->getAxisScale(1), dobj->getAxisScale(0));
            structuredPoints->SetOrigin(dobj->getAxisOffset(2), dobj->getAxisOffset(1), dobj->getAxisOffset(0));
            xDim = 2;
            yDim = 1;
        }
        else if (dobj->getDims() == 2)
        {
            structuredPoints->SetDimensions(dobj->getSize(1), dobj->getSize(0), 1);
            structuredPoints->SetSpacing(dobj->getAxisScale(1), dobj->getAxisScale(0), 1.0);
            structuredPoints->SetOrigin(dobj->getAxisOffset(1), dobj->getAxisOffset(0), 0.0);
            xDim = 1;
            yDim = 0;
        }

#if VTK_MAJOR_VERSION <= 5
        structuredPoints->SetNumberOfScalarComponents(1);
        switch (input->getType())
        {
        case ito::tUInt8:
            structuredPoints->SetScalarTypeToUnsignedChar();
            break;
        case ito::tUInt16:
            structuredPoints->SetScalarTypeToUnsignedShort();
            break;
        }
#else
        switch (input->getType())
        {
        case ito::tUInt8:
            structuredPoints->AllocateScalars(VTK_UNSIGNED_CHAR,1);
            break;
        case ito::tUInt16:
            structuredPoints->AllocateScalars(VTK_UNSIGNED_SHORT,1);
            break;
        }
#endif

        switch (input->getType())
        {
        case ito::tUInt8:
            {
                const ito::uint8 *ptr;
                ito::uint8 *scalar;
                for (int z = 0; z < dobj->getNumPlanes(); ++z)
                {
                    for (int y = 0; y < dobj->getSize(yDim); ++y)
                    {
                        ptr = (const ito::uint8*)(dobj->rowPtr(z, y));
                        for (int x = 0; x < dobj->getSize(xDim); ++x)
                        {
                            scalar = static_cast<ito::uint8*>(structuredPoints->GetScalarPointer(x,y,z));
                            scalar[0] = (ptr[x] <= t) ? 0 : ptr[x];
                        }
                    }
                }
            }
            break;
        case ito::tUInt16:
            {
                const ito::uint16 *ptr;
                ito::uint16 *scalar;
                for (int z = 0; z < dobj->getNumPlanes(); ++z)
                {
                    for (int y = 0; y < dobj->getSize(yDim); ++y)
                    {
                        ptr = (const ito::uint16*)(dobj->rowPtr(z, y));
                        for (int x = 0; x < dobj->getSize(xDim); ++x)
                        {
                            scalar = static_cast<ito::uint16*>(structuredPoints->GetScalarPointer(x,y,z));
                            scalar[0] = (ptr[x] <= t) ? 0 : ptr[x];
                        }
                    }
                }
            }
            break;
        }

        structuredPoints->GetPointData()->GetScalars()->SetName(scalarName.data());

        vtkSmartPointer<vtkXMLImageDataWriter> writer = vtkSmartPointer<vtkXMLImageDataWriter>::New();
        if (paramsOpt->at(0).getVal<char*>()[0] == 'b')
        {
            writer->SetDataModeToBinary();
        }
        else
        {
            writer->SetDataModeToAscii();
        }
        writer->SetFileName(filename.toLatin1().data());

#if (VTK_MAJOR_VERSION == 5)
        writer->SetInput (structuredPoints);
#else
        writer->SetInputData (structuredPoints);
#endif
        writer->Write();

        delete dobj;
        dobj = NULL;
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::savePolygonMeshDOC = QObject::tr("save an itom.polygonMesh object to a file\n\
\n\
The following file formats are currently supported: \n\
\n\
* obj (wavefront obj file format) \n\
* ply (polygon file format or stanford triangle format - binary file format only) \n\
* vtk (VTK file format) \n\
* stl (Stereolithography file format) \n\
\n\
Usually the format is guessed from the suffix of the given file name. Else use the optional parameter 'type' to indicate the desired file format.");

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::savePolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("polygonMesh", ito::ParamBase::PolygonMeshPtr, NULL, tr("polygon mesh to save").toLatin1().data()));
    paramsMand->append(ito::Param("filename", ito::ParamBase::String, "", tr("complete filename (type is either read by suffix of filename or by parameter 'type')").toLatin1().data()));

    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('obj' [default],'ply','vtk','stl')").toLatin1().data()));
    paramsOpt->append(ito::Param("binary", ito::ParamBase::Int, 0, 1, 1, tr("If 1 (default), the file is written as binary file, else ascii. If type is 'obj', the file is always an ascii file. (This option is only considered for PCL > 1.8.0).").toLatin1().data()));
    paramsOpt->append(ito::Param("precision", ito::ParamBase::Int, 0, 16, 5, tr("Precision (default: 5), only valid for 'obj'-file types.").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::savePolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPolygonMesh *polygonMesh = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<char*>();  //Input object
    QString filename = QString::fromLatin1((*paramsMand)[1].getVal<char*>());
    std::string filename_ = (*paramsMand)[1].getVal<const char*>(); //directly load the std::string from the given char* instead of extracting it from the latin1-str, since encoding errors can occur in case of special characters
    QString type = (*paramsOpt)[0].getVal<char*>();
    bool binary_mode = (paramsOpt->at(1).getVal<int>() > 0) ? true : false;
    unsigned int precision = static_cast<unsigned int>(paramsOpt->at(2).getVal<int>());
    int ret = 1;

    //check filename
    QFileInfo finfo(filename);
    if (type == "")
    {
        type = finfo.suffix().toLower();
        filename = finfo.absoluteFilePath();
    }

    //check type
    type = type.toLower();
    if (type != "obj" && type != "ply" && type != "vtk" && type != "stl")
    {
        retval += ito::RetVal(ito::retWarning, 0, tr("type was set to 'obj', since 'obj','ply', 'stl' or 'vtk' expected").toLatin1().data());
        type = "obj";
    }

    if (polygonMesh->valid() == false)
    {
        retval += ito::RetVal(ito::retError, 0, tr("invalid polygon mesh cannot be saved").toLatin1().data());
    }

    if (!retval.containsError())
    {
    //check point cloud
        if (type == "obj")
        {
            ret = pcl::io::saveOBJFile(filename_, *(polygonMesh->polygonMesh()), precision);
        }
        else if (type == "stl")
        {
#if PCL_VERSION_COMPARE(>,1,8,0)
            ret = pcl::io::savePolygonFileSTL(filename_, *(polygonMesh->polygonMesh()), binary_mode);
#else
            ret = pcl::io::savePolygonFileSTL(filename_, *(polygonMesh->polygonMesh()));
#endif
        }
        else if (type == "ply")
        {
#if PCL_VERSION_COMPARE(>,1,8,0)
            ret = pcl::io::savePolygonFilePLY(filename_, *(polygonMesh->polygonMesh()), binary_mode);
#else
            ret = pcl::io::savePolygonFilePLY(filename_, *(polygonMesh->polygonMesh()));
#endif
        }
        else if (type == "vtk")
        {
#if PCL_VERSION_COMPARE(>,1,8,0)
            ret = pcl::io::savePolygonFileVTK(filename_, *(polygonMesh->polygonMesh()), binary_mode);
#else
            ret = pcl::io::savePolygonFileVTK(filename_, *(polygonMesh->polygonMesh()));
#endif
        }

#if PCL_VERSION_COMPARE(>=,1,7,0)
        if (ret < 0)
        {
            retval += ito::RetVal(ito::retError, 0, tr("error while saving polygon mesh (internal error of method in point cloud library").toLatin1().data());
        }
#else
    //uncommented since huge polygon meshes result in a buffer overflow of int.
    // bug in PCL version 1.6 or below. see: http://dev.pointclouds.org/issues/974, fixed in current trunk leading to any version bigger than the binary of 1.6
    /**/
#endif
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::loadPolygonMeshDOC = QObject::tr("This filter loads polygon mesh data files to an itom.polygonMesh object\n\
\n\
The following file formats are supported:\n\
\n\
* ply (polygon file format also known under stanford triangle format - be careful ply can also contain point cloud data only which is loaded using 'loadPointCloud'). Both ascii and binary formats are supported. \n\
* vtk (point cloud format from the vtk library\n\
* obj (wavefront OBJ file format) \n\
* stl (Stereolithography file format)");

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->append(ito::Param("polygonMesh", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("loaded polygon mesh").toLatin1().data()));
    paramsMand->append(ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("complete filename (type is read by suffix").toLatin1().data()));

    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('obj','vtk','stl','ply', 'auto' [default, check suffix of filename])").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPolygonMesh* mesh = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<char*>();
    QString filename = QString::fromLatin1((*paramsMand)[1].getVal<const char*>());
    std::string filename_ = (*paramsMand)[1].getVal<const char*>(); //directly load the std::string from the given char* instead of extracting it from the latin1-str, since encoding errors can occur in case of special characters
    QString type = (*paramsOpt)[0].getVal<char*>();

//    bool binary_mode = true;
    int ret = 1;
    pcl::PolygonMesh::Ptr polyMesh(new pcl::PolygonMesh());

    //check filename
    QFileInfo finfo(filename);
    if (type == "auto" || type == "")
    {
        type = finfo.suffix().toLower();
    }

    if (finfo.exists() == false)
    {
        char *name = (*paramsMand)[1].getVal<char*>();
        retval += ito::RetVal::format(ito::retError, 0, tr("filename '%s' does not exist").toLatin1().data(), name);
    }
    else if (filename == "")
    {
        retval += ito::RetVal(ito::retError, 0, tr("Filename is empty").toLatin1().data());
    }
    else if (type == "obj")
    {
        ret = pcl::io::loadPolygonFileOBJ(filename_, *polyMesh);
    }
    else if (type == "vtk")
    {
        ret = pcl::io::loadPolygonFileVTK(filename_, *polyMesh);
    }
    else if (type == "stl")
    {
        ret = pcl::io::loadPolygonFileSTL(filename_, *polyMesh);
    }
    else if (type == "ply")
    {
        //be careful: ply files can be everything: therefore check if they are a polygon mesh structure built by VTK
        QFile file(QString::fromStdString(filename_));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("file '%s' could not be opened.").toLatin1().data(), filename_.data());
        }
        else
        {
            QByteArray startContent;
            int i = 0;
            while (!file.atEnd() && (++i) < 30)
            {
                startContent += file.readLine();
            }

            file.close();

            if (startContent.contains("ply") && startContent.contains("format") && \
                startContent.contains("element vertex") && startContent.contains("element face"))
            {
                ret = pcl::io::loadPolygonFilePLY(filename_, *polyMesh);
            }
            else
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("file '%s' does not contain valid polygon mesh data.").toLatin1().data(), filename_.data());
            }
        }
    }
    else
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("unsupported format '%s'").toLatin1().data(), type.toLatin1().data());
    }

    if (ret < 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("error while loading polygon mesh").toLatin1().data());
    }

    if (retval == ito::retOk)
    {
        ito::PCLPolygonMesh polygonMesh(polyMesh);

        /*QVariant output = QVariant::fromValue(polygonMesh);
        outVals->append(output);*/
        *mesh = ito::PCLPolygonMesh(polyMesh);
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::transformAffineDOC = QObject::tr("Applies an affine coordinate transform to the input pointCloud \n\
\n\
The transformed point cloud is saved in 'pointCloudOut' (inplace possible). The transformation matrix has to be a 4x4 homogeneous transformation matrix \
given by a 4x4 real dataObject (uint8, int8, uint16, int16, uint32, int32 or float32 allowed). Every point P_in in the input cloud \
is transformed by P_out = transform * P_in. Independent on the type of the transformation matrix, the matrix multiplication is done with float32 precision.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::transformAffineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("The affine transform is applied to this point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Resulting, transformed point cloud (inplace possible)").toLatin1().data()));
    paramsMand->append(ito::Param("transform", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("4x4 homogeneous transformation matrix (uint8, int8, uint16, int16, uint32, int32, float32, float64)").toLatin1().data()));

    paramsOpt->clear();
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::transformAffine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    //read params from mandatory and optional params

    const ito::PCLPointCloud *pclIn = (*paramsMand)[0].getVal<const ito::PCLPointCloud*>();
    ito::PCLPointCloud *pclOut = (*paramsMand)[1].getVal<ito::PCLPointCloud*>();
    const ito::DataObject *transform = (*paramsMand)[2].getVal<const ito::DataObject*>();

    if (pclIn == NULL || pclOut == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
        return retval;
    }

    Eigen::Affine3f trafo;
    retval += ito::pclHelper::dataObj4x4ToEigenAffine3f(transform, trafo);

    if (!retval.containsError())
    {
        if (pclIn != pclOut)
        {
            *pclOut = ito::PCLPointCloud(pclIn->getType());
        }

        switch(pclIn->getType())
        {
        case ito::pclXYZ:
            {
                const pcl::PointCloud<pcl::PointXYZ> *cloudIn = pclIn->toPointXYZ().get();
                pcl::PointCloud<pcl::PointXYZ> *cloudOut = pclOut->toPointXYZ().get();
                pcl::transformPointCloud<pcl::PointXYZ>(*cloudIn, *cloudOut, trafo);
                break;
            }
        case ito::pclXYZI:
            {
                const pcl::PointCloud<pcl::PointXYZI> *cloudIn = pclIn->toPointXYZI().get();
                pcl::PointCloud<pcl::PointXYZI> *cloudOut = pclOut->toPointXYZI().get();
                pcl::transformPointCloud<pcl::PointXYZI>(*cloudIn, *cloudOut, trafo);
                break;
            }
        case ito::pclXYZRGBA:
            {
                const pcl::PointCloud<pcl::PointXYZRGBA> *cloudIn = pclIn->toPointXYZRGBA().get();
                pcl::PointCloud<pcl::PointXYZRGBA> *cloudOut = pclOut->toPointXYZRGBA().get();
                pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloudIn, *cloudOut, trafo);
                break;
            }
        case ito::pclXYZNormal:
            {
                const pcl::PointCloud<pcl::PointNormal> *cloudIn = pclIn->toPointXYZNormal().get();
                pcl::PointCloud<pcl::PointNormal> *cloudOut = pclOut->toPointXYZNormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
        case ito::pclXYZINormal:
            {
                const pcl::PointCloud<pcl::PointXYZINormal> *cloudIn = pclIn->toPointXYZINormal().get();
                pcl::PointCloud<pcl::PointXYZINormal> *cloudOut = pclOut->toPointXYZINormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointXYZINormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
        case ito::pclXYZRGBNormal:
            {
                const pcl::PointCloud<pcl::PointXYZRGBNormal> *cloudIn = pclIn->toPointXYZRGBNormal().get();
                pcl::PointCloud<pcl::PointXYZRGBNormal> *cloudOut = pclOut->toPointXYZRGBNormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointXYZRGBNormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
        default:
            break;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString PclTools::transformAffineMeshDOC = QObject::tr("Applies an affine coordinate transform to all points of the input mesh. \n\
\n\
The transformed points are saved in 'meshOut' (inplace possible). The transformation matrix has to be a 4x4 homogeneous transformation matrix \
given by a 4x4 real dataObject (uint8, int8, uint16, int16, uint32, int32 or float32 allowed). Every point P_in in the input mesh \
is transformed by P_out = transform * P_in. Independent on the type of the transformation matrix, the matrix multiplication is done with float32 precision.");

/*static*/ ito::RetVal PclTools::transformAffineMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("meshIn", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("The affine transform is applied to the points in this polygonal mesh").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Resulting, transformed polygon mesh (inplace possible)").toLatin1().data()));
    paramsMand->append(ito::Param("transform", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("4x4 homogeneous transformation matrix (uint8, int8, uint16, int16, uint32, int32, float32, float64)").toLatin1().data()));

    paramsOpt->clear();
    return retval;
}

/*static*/ ito::RetVal PclTools::transformAffineMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    //read params from mandatory and optional params

    const ito::PCLPolygonMesh *meshIn = (*paramsMand)[0].getVal<const ito::PCLPolygonMesh*>();
    ito::PCLPolygonMesh *meshOut = (*paramsMand)[1].getVal<ito::PCLPolygonMesh*>();
    const ito::DataObject *transform = (*paramsMand)[2].getVal<const ito::DataObject*>();

    if (meshIn == NULL || meshOut == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("mesh must not be NULL").toLatin1().data());
        return retval;
    }
    else if (meshIn->polygonMesh().get() == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("input mesh is empty").toLatin1().data());
    }

    ito::tPCLPointType t = ito::pclHelper::guessPointType(meshIn->polygonMesh()->cloud);
    if (t == ito::pclInvalid)
    {
        return ito::RetVal(ito::retError, 0, tr("cloud type of 'meshIn' is invalid.").toLatin1().data());
    }

    Eigen::Affine3f trafo;
    retval += ito::pclHelper::dataObj4x4ToEigenAffine3f(transform, trafo);

    ito::PCLPointCloud cloud(t);

    if (!retval.containsError())
    {
        retval += ito::pclHelper::pointCloud2ToPCLPointCloud(meshIn->polygonMesh()->cloud, &cloud);
    }

    if (!retval.containsError())
    {
        if (meshIn != meshOut)
        {
            ito::PCLPointCloud dummyCloud(t);
            dummyCloud.push_back(ito::PCLPoint(t));
            *meshOut = ito::PCLPolygonMesh(dummyCloud, meshIn->polygonMesh()->polygons); //constructing a mesh from an empty point cloud crashes in PCL 1.8.0
        }

        if (!retval.containsError())
        {
            switch (cloud.getType())
            {
            case ito::pclXYZ:
            {
                const pcl::PointCloud<pcl::PointXYZ> *cloudIn = cloud.toPointXYZ().get();
                pcl::PointCloud<pcl::PointXYZ> *cloudOut = cloud.toPointXYZ().get();
                pcl::transformPointCloud<pcl::PointXYZ>(*cloudIn, *cloudOut, trafo);
                break;
            }
            case ito::pclXYZI:
            {
                const pcl::PointCloud<pcl::PointXYZI> *cloudIn = cloud.toPointXYZI().get();
                pcl::PointCloud<pcl::PointXYZI> *cloudOut = cloud.toPointXYZI().get();
                pcl::transformPointCloud<pcl::PointXYZI>(*cloudIn, *cloudOut, trafo);
                break;
            }
            case ito::pclXYZRGBA:
            {
                const pcl::PointCloud<pcl::PointXYZRGBA> *cloudIn = cloud.toPointXYZRGBA().get();
                pcl::PointCloud<pcl::PointXYZRGBA> *cloudOut = cloud.toPointXYZRGBA().get();
                pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloudIn, *cloudOut, trafo);
                break;
            }
            case ito::pclXYZNormal:
            {
                const pcl::PointCloud<pcl::PointNormal> *cloudIn = cloud.toPointXYZNormal().get();
                pcl::PointCloud<pcl::PointNormal> *cloudOut = cloud.toPointXYZNormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
            case ito::pclXYZINormal:
            {
                const pcl::PointCloud<pcl::PointXYZINormal> *cloudIn = cloud.toPointXYZINormal().get();
                pcl::PointCloud<pcl::PointXYZINormal> *cloudOut = cloud.toPointXYZINormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointXYZINormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
            case ito::pclXYZRGBNormal:
            {
                const pcl::PointCloud<pcl::PointXYZRGBNormal> *cloudIn = cloud.toPointXYZRGBNormal().get();
                pcl::PointCloud<pcl::PointXYZRGBNormal> *cloudOut = cloud.toPointXYZRGBNormal().get();
                pcl::transformPointCloudWithNormals<pcl::PointXYZRGBNormal>(*cloudIn, *cloudOut, trafo);
                break;
            }
            default:
                break;
            }

            retval += ito::pclHelper::pclPointCloudToPointCloud2(cloud, meshOut->polygonMesh()->cloud);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclProjectOnModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Input point cloud with normal values").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with distances").toLatin1().data()));
    paramsMand->append(ito::Param("modelType", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Model type according to enum pcl::SacModel").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("point on cylinder symmetrie axis").toLatin1().data()));
    paramsOpt->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("symmetrie axis of cylinder").toLatin1().data()));
    paramsOpt->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::In, (double)(-std::numeric_limits<float>::max()), (double)(std::numeric_limits<float>::max()), 0.0, tr("cylinder radius").toLatin1().data()));

    paramsOut->clear();

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclEstimateNormalsDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclEstimateNormalsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid point cloud whose normals should be estimated").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with estimated normals. The type corresponds to the normal-enhanced type of the input cloud.").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("kSearch", ito::ParamBase::Int | ito::ParamBase::In, 3, 10000, 50, tr("the number of k nearest neighbors to use for the feature estimation [default: 50]").toLatin1().data()));
    paramsOpt->append(ito::Param("viewPoint", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("optional camera view point (if given it must have three entries [x,y,z], [default: not used])").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclEstimateNormals(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    //read params from mandatory and optional params

    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    int kSearch = (*paramsOpt)[0].getVal<int>();
    double *viewPoint = (*paramsOpt)[1].getVal<double*>();

    double noViewPoint[3] = {0.0, 0.0, 1.0};

    if (pclIn == NULL || pclOut == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    bool areTheSame = false;

    if (pclIn == pclOut)
    {
        // Houston we have a problem
        areTheSame = true;
    }

    if (viewPoint && (*paramsOpt)[1].getLen() != 3)
    {
        retval += ito::RetVal(ito::retError, 0, tr("view point must be empty or a double list with three elements.").toLatin1().data());
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    int nr_threads_ = QThread::idealThreadCount();
    unsigned int nr_threads =  std::max(1, nr_threads_ - 1); //let one thread for the rest

    if (!retval.containsError())
    {
        switch(pclIn->getType())
        {
        case ito::pclInvalid:
            retval += ito::RetVal(ito::retError, 0, tr("input point cloud must be valid").toLatin1().data());
            break;
        case ito::pclXYZ:
            {

                pcl::PointCloud<pcl::PointXYZ> *srcTempPCL = NULL;
                if (areTheSame)
                {
                    srcTempPCL = new pcl::PointCloud<pcl::PointXYZ>();
                    pcl::copyPointCloud(*(pclIn->toPointXYZ()), *srcTempPCL);
                }
                else
                {
                    srcTempPCL = pclIn->toPointXYZ().get();
                }

                *pclOut = ito::PCLPointCloud(ito::pclXYZNormal);

                pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

                ne.setSearchMethod(tree);
                ne.setInputCloud(srcTempPCL->makeShared());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);

                pcl::concatenateFields(*(srcTempPCL), *normals, *(pclOut->toPointXYZNormal()));

                if (areTheSame)
                {
                    delete srcTempPCL;
                }
            }
            break;
        case ito::pclXYZI:
            {
                pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());

                pcl::PointCloud<pcl::PointXYZI> *srcTempPCL = NULL;
                if (areTheSame)
                {
                    srcTempPCL = new pcl::PointCloud<pcl::PointXYZI>();
                    pcl::copyPointCloud(*(pclIn->toPointXYZI()), *srcTempPCL);
                }
                else
                {
                    srcTempPCL = pclIn->toPointXYZI().get();
                }

                *pclOut = ito::PCLPointCloud(ito::pclXYZINormal);

                ne.setSearchMethod(tree);
                ne.setInputCloud(srcTempPCL->makeShared());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);


                pcl::concatenateFields(*(srcTempPCL), *normals, *(pclOut->toPointXYZINormal()));

                if (areTheSame)
                {
                    delete srcTempPCL;
                }
            }
            break;
        case ito::pclXYZRGBA:
            {

                //the alpha value of rgba will be omitted
                pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
                pcl::copyPointCloud(*(pclIn->toPointXYZRGBA()), rgbCloud);

                *pclOut = ito::PCLPointCloud(ito::pclXYZRGBNormal);

                pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

                ne.setSearchMethod(tree);
                ne.setInputCloud(rgbCloud.makeShared());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);

                retval += ito::RetVal(ito::retWarning, 0, tr("the alpha values of the input point cloud cannot be copied to the output point cloud [not supported]").toLatin1().data());
                pcl::concatenateFields(rgbCloud, *normals, *(pclOut->toPointXYZRGBNormal()));
            }
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, tr("type of input point cloud not supported (no normal based input types allowed)").toLatin1().data());
            break;
        }
    }

    return retval;
}

////------------------------------------------------------------------------------------------------------------------------------
//const QString PclTools::pclEstimateMaxCurvatureDOC = QObject::tr("estimates the curvature of a given point cloud with normal vectors based on nearest neighbours. \n\
//\n\
//The nearest neighbours are determined by a flann based kd-tree search that can be parametrized by the number of nearest neighbours (kSearch) \n\
//and / or the maximum distance to nearest neighbours (searchRadius). Both values can be considered, too. The maximum curvature is set to the\n\
//curvature value of the input 'pointCloudNormalIn'.");
//
////----------------------------------------------------------------------------------------------------------------------------------
///*static*/ ito::RetVal PclTools::pclEstimateMaxCurvatureParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
//{
//    ito::Param param;
//    ito::RetVal retval = ito::retOk;
//    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
//    if (retval.containsError())
//    {
//        return retval;
//    }
//
//    paramsMand->clear();
//    paramsMand->append(ito::Param("pointCloudNormalIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Valid point cloud with normals. The curvature value of this point cloud is set to the maximum curvature value determined by this filter.").toLatin1().data()));
//
//    paramsOpt->clear();
//    paramsOpt->append(ito::Param("kSearch", ito::ParamBase::Int | ito::ParamBase::In, 0, 10000, 7, tr("the number of k nearest neighbours to use for the feature estimation [default: 7, if 0: not set]").toLatin1().data()));
//    paramsOpt->append(ito::Param("searchRadius", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 0.0, tr("search radius for nearest neighbours (if 0.0, this value is not set)").toLatin1().data()));
//    return retval;
//}
//
//template<typename PointT> ito::RetVal _pclEstimateMaxCurvature(typename pcl::PointCloud<PointT>::Ptr cloud, int kSearch, double searchRadius)
//{
//    pcl::PointCloud<pcl::PrincipalCurvatures> curvatures;
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    pcl::PrincipalCurvaturesEstimation<PointT, PointT, pcl::PrincipalCurvatures> curvatureEstimator;
//    curvatureEstimator.setSearchMethod(tree);
//    curvatureEstimator.setInputCloud(cloud);
//    curvatureEstimator.setInputNormals(cloud);
//    if (searchRadius > 0)
//    {
//        curvatureEstimator.setRadiusSearch(searchRadius);
//    }
//    else if (kSearch > 0)
//    {
//        curvatureEstimator.setKSearch(kSearch);
//    }
//    curvatureEstimator.compute(curvatures);
//    for (int i = 0; i < curvatures.size(); ++i)
//    {
//        cloud->points[i].curvature = curvatures[i].pc2; //max curvature value
//    }
//    return ito::retOk;
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
///*static*/ ito::RetVal PclTools::pclEstimateMaxCurvature(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
//{
//    ito::RetVal retval = ito::retOk;
//
//    //read params from mandatory and optional params
//
//    ito::PCLPointCloud *pclIn = (*paramsMand)[0].getVal<ito::PCLPointCloud*>();
//
//    int kSearch = paramsOpt->at(0).getVal<int>();
//    double searchRadius = paramsOpt->at(1).getVal<double>();
//
//    if ((kSearch == 0 && searchRadius == 0.0) || (kSearch > 0 && searchRadius > 0.0))
//    {
//        retval += ito::RetVal(ito::retError, 0, tr("you need to indicate either the parameter 'kSearch' or 'searchRadius'").toLatin1().data());
//    }
//
//    if (pclIn == NULL)
//    {
//        retval += ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
//    }
//
//    pcl::PointCloud<pcl::PrincipalCurvatures> curvatures;
//
//    if (!retval.containsError())
//    {
//        switch(pclIn->getType())
//        {
//        case ito::pclInvalid:
//            retval += ito::RetVal(ito::retError, 0, tr("input point cloud must be valid").toLatin1().data());
//            break;
//        case ito::pclXYZNormal:
//            {
//                _pclEstimateMaxCurvature<pcl::PointNormal>(pclIn->toPointXYZNormal(), kSearch, searchRadius);
//            }
//            break;
//        case ito::pclXYZINormal:
//            {
//                _pclEstimateMaxCurvature<pcl::PointXYZINormal>(pclIn->toPointXYZINormal(), kSearch, searchRadius);
//            }
//            break;
//        case ito::pclXYZRGBNormal:
//            {
//                _pclEstimateMaxCurvature<pcl::PointXYZRGBNormal>(pclIn->toPointXYZRGBNormal(), kSearch, searchRadius);
//            }
//            break;
//
//        default:
//            retval += ito::RetVal(ito::retError, 0, tr("type of input point cloud not supported (only normal based input types allowed.)").toLatin1().data());
//            break;
//        }
//    }
//
//    return retval;
//}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclRemoveNaNDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclRemoveNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclRemoveNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    //real inplace is possible
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    if (pclIn != pclOut)
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    std::vector<int> indices;

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ:
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*(ptr), *(pclOut->toPointXYZ()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    case ito::pclXYZI:
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointXYZI>(*(ptr), *(pclOut->toPointXYZI()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    case ito::pclXYZRGBA:
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointXYZRGBA>(*(ptr), *(pclOut->toPointXYZRGBA()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    case ito::pclXYZNormal:
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointNormal>(*(ptr), *(pclOut->toPointXYZNormal()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    case ito::pclXYZINormal:
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*(ptr), *(pclOut->toPointXYZINormal()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    case ito::pclXYZRGBNormal:
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            ptr->is_dense = false; //if it is true, no nan will be removed, after procedure
            pcl::removeNaNFromPointCloud<pcl::PointXYZRGBNormal>(*(ptr), *(pclOut->toPointXYZRGBNormal()), indices);
            if (indices.size() == ptr->points.size())
            {
                ptr->is_dense = true;
            }
        }
        break;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclPassThroughDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPassThroughParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));
    paramsMand->append(ito::Param("fieldName", ito::ParamBase::String | ito::ParamBase::In, "z", tr("valid field name to filter 'x','y',z','intensity'...").toLatin1().data()));

    paramsOpt->append(ito::Param("minValue", ito::ParamBase::Double | ito::ParamBase::In, -FLT_MAX, FLT_MAX, -FLT_MAX, tr("minimum value (default: -FLT_MAX)").toLatin1().data()));
    paramsOpt->append(ito::Param("maxValue", ito::ParamBase::Double | ito::ParamBase::In, -FLT_MAX, FLT_MAX, FLT_MAX, tr("maximum value (default: FLT_MAX)").toLatin1().data()));
    paramsOpt->append(ito::Param("negative", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1: values inside of range will be deleted, else 0 [default]").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPassThrough(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    std::string fieldName = (*paramsMand)[2].getVal<char*>();
    ito::float32 minValue = (*paramsOpt)[0].getVal<double>();
    ito::float32 maxValue = (*paramsOpt)[1].getVal<double>();
    int negative = (*paramsOpt)[2].getVal<int>();

    QString fieldNames = QString::fromStdString(pclIn->getFieldsList());
    QStringList fields = fieldNames.split(" ");

    if (fields.contains(QString::fromStdString(fieldName)) == false)
    {
        return ito::RetVal::format(ito::retError, 0, tr("field with name '%s' does not exist in given point cloud").toLatin1().data(), fieldName.data());
    }

    if (pclIn == pclOut)
    {
        pclOut = new ito::PCLPointCloud(pclIn->getType());
        inplace = true;
    }
    else
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ:
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::PassThrough<pcl::PointXYZ> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZ()));
        }
        break;
    case ito::pclXYZI:
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::PassThrough<pcl::PointXYZI> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZI()));
        }
        break;
    case ito::pclXYZRGBA:
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::PassThrough<pcl::PointXYZRGBA> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZRGBA()));
        }
        break;
    case ito::pclXYZNormal:
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::PassThrough<pcl::PointNormal> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZNormal()));
        }
        break;
    case ito::pclXYZINormal:
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::PassThrough<pcl::PointXYZINormal> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZINormal()));
        }
        break;
    case ito::pclXYZRGBNormal:
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::PassThrough<pcl::PointXYZRGBNormal> ptfilter (false);
            ptfilter.setInputCloud(ptr);
            ptfilter.setFilterFieldName(fieldName);
            ptfilter.setFilterLimits(minValue, maxValue);
            if (negative > 0)
            {
                ptfilter.setNegative(true);
            }
            ptfilter.filter(*(pclOut->toPointXYZRGBNormal()));
        }

        break;
    }

    if (inplace)
    {
        (*pclIn) = (*pclOut); //here: pclOut is a new, temporary point cloud, pclIn is the given argument pclIn AND pclOut!
        delete pclOut;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclCropBoxDOC = QObject::tr("pclCropBox is a filter that allows the user to filter all the data inside of a given box.\n\
\n\
Indicate the minimum and maximum values in x,y and z direction for the box and optionally tranlate and rotate the box to \n\
adjust its position and orientation. The rotation vector are the euler angles rx, ry and rz.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclCropBoxParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));

    double mins[] = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
    param = ito::Param("minValue", ito::ParamBase::DoubleArray | ito::ParamBase::In, 3, mins, tr("minimum values (x,y,z) (default: FLT_MIN)").toLatin1().data());
    ito::DoubleArrayMeta *dam = new ito::DoubleArrayMeta(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0, 3, 3);
    param.setMeta(dam, true);
    paramsOpt->append(param);

    double maxs[] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    param = ito::Param("maxValue", ito::ParamBase::DoubleArray | ito::ParamBase::In, 3, maxs, tr("maximum values (x,y,z) (default: FLT_MAX)").toLatin1().data());
    dam = new ito::DoubleArrayMeta(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0, 3, 3);
    param.setMeta(dam, true);
    paramsOpt->append(param);

    double zeros[] = {0.0, 0.0, 0.0};
    param = ito::Param("translation", ito::ParamBase::DoubleArray | ito::ParamBase::In, 3, zeros, tr("translation of box (dx,dy,dz) (default: zero values)").toLatin1().data());
    dam = new ito::DoubleArrayMeta(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0, 3, 3);
    param.setMeta(dam, true);
    paramsOpt->append(param);

    param = ito::Param("rotation", ito::ParamBase::DoubleArray | ito::ParamBase::In, 3, zeros, tr("euler rotation angles (in rad) of box (rx,ry,rz) (default: zero values)").toLatin1().data());
    dam = new ito::DoubleArrayMeta(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0, 3, 3);
    param.setMeta(dam, true);
    paramsOpt->append(param);

    paramsOpt->append(ito::Param("negative", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1: values inside of range will be deleted, else 0 [default]").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclCropBox(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    const double *mins = paramsOpt->at(0).getVal<double*>();
    const double *maxs = paramsOpt->at(1).getVal<double*>();
    const double *rot = paramsOpt->at(3).getVal<double*>();
    const double *trans = paramsOpt->at(2).getVal<double*>();

    Eigen::Vector4f min_pt(mins[0], mins[1], mins[2], 0.0);
    Eigen::Vector4f max_pt(maxs[0], maxs[1], maxs[2], 0.0);
    Eigen::Vector3f rotation(rot[0], rot[1], rot[2]);
    Eigen::Vector3f translation(trans[0], trans[1], trans[2]);

    int negative = (*paramsOpt)[4].getVal<int>();

    if (pclIn == pclOut)
    {
        pclOut = new ito::PCLPointCloud(pclIn->getType());
        inplace = true;
    }
    else
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ:
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointXYZ> cbfilter (false);
#else
            pcl::CropBox<pcl::PointXYZ> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZ()));
        }
        break;
    case ito::pclXYZI:
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointXYZI> cbfilter (false);
#else
            pcl::CropBox<pcl::PointXYZI> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZI()));
        }
        break;
    case ito::pclXYZRGBA:
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointXYZRGBA> cbfilter (false);
#else
            pcl::CropBox<pcl::PointXYZRGBA> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZRGBA()));
        }
        break;
    case ito::pclXYZNormal:
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointNormal> cbfilter (false);
#else
            pcl::CropBox<pcl::PointNormal> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZNormal()));
        }
        break;
    case ito::pclXYZINormal:
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointXYZINormal> cbfilter (false);
#else
            pcl::CropBox<pcl::PointXYZINormal> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZINormal()));
        }
        break;
    case ito::pclXYZRGBNormal:
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            pcl::CropBox<pcl::PointXYZRGBNormal> cbfilter (false);
#else
            pcl::CropBox<pcl::PointXYZRGBNormal> cbfilter;
#endif
            cbfilter.setInputCloud(ptr);
            cbfilter.setMin(min_pt);
            cbfilter.setMax(max_pt);
            cbfilter.setTranslation(translation);
            cbfilter.setRotation(rotation);
            if (negative > 0)
            {
                cbfilter.setNegative(true);
            }
            cbfilter.filter(*(pclOut->toPointXYZRGBNormal()));
        }

        break;
    }

    if (inplace)
    {
        (*pclIn) = (*pclOut); //here: pclOut is a new, temporary point cloud, pclIn is the given argument pclIn AND pclOut!
        delete pclOut;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclVoxelGridDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclVoxelGridParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));
    paramsMand->append(ito::Param("leafSize", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("voxel grid leaf size [lx,ly,lz] (3 elements necessary)").toLatin1().data()));

    paramsOpt->append(ito::Param("downsampleAllData", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("1: downsample all fields, 0: only xyz [default]").toLatin1().data()));
    paramsOpt->append(ito::Param("fieldName", ito::ParamBase::String | ito::ParamBase::In, "", tr("field name, whose value is filtered between 'fieldMin' and 'fieldMax' or '' if no field filtering should be applied [default]").toLatin1().data()));
    paramsOpt->append(ito::Param("fieldMin", ito::ParamBase::Double | ito::ParamBase::In, FLT_MIN, FLT_MAX, FLT_MIN, tr("minimum field filtering value (default: FLT_MIN)").toLatin1().data()));
    paramsOpt->append(ito::Param("fieldMax", ito::ParamBase::Double | ito::ParamBase::In, FLT_MIN, FLT_MAX, FLT_MAX, tr("maximum field filtering value (default: FLT_MAX)").toLatin1().data()));
    paramsOpt->append(ito::Param("negative", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("0 [default]: values inside of field range will be deleted, else 1").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclVoxelGrid(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    ito::ParamBase leafSizeP = (*paramsMand)[2];
    if (leafSizeP.getLen() != 3)
    {
        return ito::RetVal(ito::retError, 0, tr("leafSize must be a vector with three elements").toLatin1().data());
    }
    double *leafSize = leafSizeP.getVal<double*>();

    int downsampleAllData = (*paramsOpt)[0].getVal<int>();
    std::string fieldName = (*paramsOpt)[1].getVal<char*>();
    ito::float32 minValue = (*paramsOpt)[2].getVal<double>();
    ito::float32 maxValue = (*paramsOpt)[3].getVal<double>();
    int negative = (*paramsOpt)[4].getVal<int>();

    if (fieldName != "")
    {
        QString fieldNames = QString::fromStdString(pclIn->getFieldsList());
        QStringList fields = fieldNames.split(" ");

        if (fields.contains(QString::fromStdString(fieldName)) == false)
        {
            return ito::RetVal::format(ito::retError, 0, tr("field with name '%s' does not exist in given point cloud").toLatin1().data(), fieldName.data());
        }
    }

    if (pclIn == pclOut)
    {
        pclOut = new ito::PCLPointCloud(pclIn->getType());
        inplace = true;
    }
    else
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::VoxelGrid<pcl::PointXYZ> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZ()));
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::VoxelGrid<pcl::PointXYZI> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZI()));
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::VoxelGrid<pcl::PointXYZRGBA> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZRGBA()));
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::VoxelGrid<pcl::PointNormal> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZNormal()));
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::VoxelGrid<pcl::PointXYZINormal> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZINormal()));
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxelsor;
            voxelsor.setInputCloud(ptr);
            voxelsor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
            if (fieldName != "")
            {
                voxelsor.setFilterFieldName(fieldName);
                voxelsor.setFilterLimits(minValue, maxValue);
                voxelsor.setFilterLimitsNegative(negative > 0);
            }

            voxelsor.filter(*(pclOut->toPointXYZRGBNormal()));
        }

        break;
    }

    if (inplace)
    {
        (*pclIn) = (*pclOut); //here: pclOut is a new, temporary point cloud, pclIn is the given argument pclIn AND pclOut!
        delete pclOut;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclStatisticalOutlierRemovalDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclStatisticalOutlierRemovalParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));
    paramsMand->append(ito::Param("meanK", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 8, tr("number of nearest neighbors to use for mean distance estimation").toLatin1().data()));
    paramsMand->append(ito::Param("stdDevMulThresh", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 1.0, tr("standard deviation multiplier for the distance threshold calculation").toLatin1().data()));

    paramsOpt->append(ito::Param("negative", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("0: the regular filter conditions are applied [default], 1: the inverted conditions are applied").toLatin1().data()));
    paramsOpt->append(ito::Param("keepOrganized", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("filtered points should be kept and set to NaN [1] or removed (potentially breaking organized structure) [default: 0]").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclStatisticalOutlierRemoval(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    if (pclIn == pclOut)
    {
        pclOut = new ito::PCLPointCloud(pclIn->getType());
        inplace = true;
    }
    else
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    int meanK = (*paramsMand)[2].getVal<int>();
    double stdDevMulThres = (*paramsMand)[3].getVal<double>();
    int negative = (*paramsOpt)[0].getVal<int>();
    int keepOrganized = (*paramsOpt)[1].getVal<int>();

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZ()));
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZI()));
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZRGBA()));
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::StatisticalOutlierRemoval<pcl::PointNormal> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZNormal()));
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZINormal()));
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sorfilter  (false);
            sorfilter.setInputCloud(ptr);
            sorfilter.setMeanK(meanK);
            sorfilter.setStddevMulThresh(stdDevMulThres);
            sorfilter.setNegative(negative > 0);
            sorfilter.setKeepOrganized(keepOrganized > 0);

            sorfilter.filter(*(pclOut->toPointXYZRGBNormal()));
        }

        break;
    }

    if (inplace)
    {
        (*pclIn) = (*pclOut); //here: pclOut is a new, temporary point cloud, pclIn is the given argument pclIn AND pclOut!
        delete pclOut;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclRandomSampleDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclRandomSampleParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud with removed NaN values").toLatin1().data()));
    paramsMand->append(ito::Param("nrOfPoints", ito::ParamBase::Int | ito::ParamBase::In, 1, std::numeric_limits<int>::max(), 10000, tr("number of randomly picked points.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclRandomSample(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    const ito::PCLPointCloud *pclIn = (*paramsMand)[0].getVal<ito::PCLPointCloud*>();
    ito::PCLPointCloud *pclOut = (*paramsMand)[1].getVal<ito::PCLPointCloud*>();

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    int nrOfPoints = (*paramsMand)[2].getVal<int>();

    if (pclIn == pclOut)
    {
        pclIn = new ito::PCLPointCloud(pclIn->copy()); //deep copy
        inplace = true;
    }
    else
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            //pcl::RandomSample<pcl::PointXYZ> randsample;
            RandomSampleCorrected<pcl::PointXYZ> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZ()));
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            //pcl::RandomSample<pcl::PointXYZI> randsample;
            RandomSampleCorrected<pcl::PointXYZI> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZI()));
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            //pcl::RandomSample<pcl::PointXYZRGBA> randsample;
            RandomSampleCorrected<pcl::PointXYZRGBA> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZRGBA()));
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            //pcl::RandomSample<pcl::PointNormal> randsample;
            RandomSampleCorrected<pcl::PointNormal> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZNormal()));
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            //pcl::RandomSample<pcl::PointXYZINormal> randsample;
            RandomSampleCorrected<pcl::PointXYZINormal> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZINormal()));
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            //pcl::RandomSample<pcl::PointXYZRGBNormal> randsample;
            RandomSampleCorrected<pcl::PointXYZRGBNormal> randsample;
            randsample.setInputCloud(ptr);
            randsample.setSample(nrOfPoints);
            randsample.setSeed((unsigned int)cv::getCPUTickCount());
            randsample.filter(*(pclOut->toPointXYZRGBNormal()));
        }

        break;
    }

    if (inplace)
    {
        DELETE_AND_SET_NULL(pclIn);
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclGetMinMax3DDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetMinMax3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));

    paramsOut->append(ito::Param("minXYZ", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("minimum (x,y,z) values").toLatin1().data()));
    paramsOut->append(ito::Param("maxXYZ", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("maximum (x,y,z) values").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetMinMax3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();

    double mins[] = {std::numeric_limits<double>::signaling_NaN() ,std::numeric_limits<double>::signaling_NaN() ,std::numeric_limits<double>::signaling_NaN() };
    double maxs[] = {std::numeric_limits<double>::signaling_NaN() ,std::numeric_limits<double>::signaling_NaN() ,std::numeric_limits<double>::signaling_NaN() };

    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL)
    {
        (*paramsOut)[0].setVal<double*>(mins,3);
        (*paramsOut)[1].setVal<double*>(maxs,3);
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::PointXYZ minP, maxP;
            pcl::getMinMax3D<pcl::PointXYZ>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::PointXYZI minP, maxP;
            pcl::getMinMax3D<pcl::PointXYZI>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::PointXYZRGBA minP, maxP;
            pcl::getMinMax3D<pcl::PointXYZRGBA>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::PointNormal minP, maxP;
            pcl::getMinMax3D<pcl::PointNormal>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::PointXYZINormal minP, maxP;
            pcl::getMinMax3D<pcl::PointXYZINormal>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::PointXYZRGBNormal minP, maxP;
            pcl::getMinMax3D<pcl::PointXYZRGBNormal>(*ptr, minP, maxP);
            mins[0] = minP.x;
            mins[1] = minP.y;
            mins[2] = minP.z;
            maxs[0] = maxP.x;
            maxs[1] = maxP.y;
            maxs[2] = maxP.z;
        }

        break;
    }

    (*paramsOut)[0].setVal<double*>(mins, 3);
    (*paramsOut)[1].setVal<double*>(maxs, 3);

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclGetPercentageThresholdDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetPercentageThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("fieldName", ito::ParamBase::String | ito::ParamBase::In, "", tr("field name, whose values are used for the determination of the threshold value.").toLatin1().data()));
    paramsMand->append(ito::Param("percentage", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 100.0, 50.0, tr("percentage value [0.0,100.0]").toLatin1().data()));

    paramsOpt->append(ito::Param("sortedValues", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("provide a dataObject if you want to access the sorted values of 'fieldName'. The output is then a float32 data object of size [1 x n], where n is the number of valid values in the specific field.").toLatin1().data()));

    paramsOut->append(ito::Param("threshold", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("threshold value (NaN if point cloud was empty or invalid)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetPercentageThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dataObject = (ito::DataObject*)(*paramsOpt)[0].getVal<void*>();
    double percentage = (*paramsMand)[2].getVal<double>() / 100.0;

    //default
    (*paramsOut)[0].setVal<double>(std::numeric_limits<double>::signaling_NaN());

    if (pclIn == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    std::string fieldName = (*paramsMand)[1].getVal<char*>();
    // Attempt to get the field name's index
#if PCL_VERSION_COMPARE(>=,1,7,0)
    std::vector<pcl::PCLPointField> fields;
#else
    std::vector<sensor_msgs::PointField> fields;
#endif
    int distance_idx = 0;

    cv::Mat_<float> values(1, pclIn->size());
    float* cvData = reinterpret_cast<float*>(values.data);
    int idx = 0;
    const uint8_t* pt_data = NULL;
    uint32_t offset;

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx >= 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&(cvData[idx]) , pt_data + offset, sizeof(float));
                    if (std::isfinite(cvData[idx])) idx++;
                }
            }
        }
        break;
    }

    if (distance_idx == -1)
    {
        if (dataObject)
        {
            *dataObject = ito::DataObject(0,ito::tFloat32);
        }
        return ito::RetVal(ito::retError, 0, tr("Unable to find field name in point type.").toLatin1().data());
    }
    else if (distance_idx > 0 && idx > 0)
    {
        cv::Mat_<float> values_valid(values, cv::Range::all(), cv::Range(0,idx));
        cv::sort(values_valid, values_valid, cv::SORT_EVERY_ROW | cv::SORT_ASCENDING);

        double thresholdIdx = percentage * idx;
        int thresholdIdxInt = cvRound(thresholdIdx);
        if (thresholdIdxInt < 0) thresholdIdxInt = 0;
        if (thresholdIdxInt >= idx) thresholdIdxInt = idx-1;

        (*paramsOut)[0].setVal<double>(values_valid.at<float>(0,thresholdIdxInt));

        if (dataObject)
        {
            const int sizes[] = {1,idx};
            *dataObject = ito::DataObject(2, sizes, ito::tFloat32, &values_valid, 1);
        }
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclGetHistogramDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetHistogramParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("fieldName", ito::ParamBase::String | ito::ParamBase::In, "", tr("field name, whose values are used for the calculation of the histogram").toLatin1().data()));
    paramsMand->append(ito::Param("minValue", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<float>::max(), 4095.0, tr("lower boundary of the uniformly distributed histogram").toLatin1().data()));
    paramsMand->append(ito::Param("maxValue", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<float>::max(), 4095.0, tr("upper boundary of the uniformly distributed histogram").toLatin1().data()));
    paramsMand->append(ito::Param("steps", ito::ParamBase::Int | ito::ParamBase::In, 1, std::numeric_limits<int>::max(), 4096, tr("number of discrete fields in the histogram").toLatin1().data()));
    paramsMand->append(ito::Param("histogram", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("The histogram is a int32 data object with size [1 x steps].").toLatin1().data()));

    paramsOpt->append(ito::Param("uniformDistribution", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("provide a dataObject if you want to access the uniform percentage distribution. The resulting data object is of type float32 and has the size [1 x 100]. The value at index j gives the histogram value, where j% of the values lies below that value.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetHistogram(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    if (pclIn == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }
    std::string fieldName = (*paramsMand)[1].getVal<char*>();
    float minValue = cv::saturate_cast<float>((*paramsMand)[2].getVal<double>());
    float maxValue = cv::saturate_cast<float>((*paramsMand)[3].getVal<double>());
    int steps = (*paramsMand)[4].getVal<int>();
    float stepSize = (1 + maxValue - minValue) / (float)steps;

    ito::DataObject *histogram = (ito::DataObject*)(*paramsMand)[5].getVal<void*>();
    ito::int32* histo_ptr = NULL;
    if (histogram == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("histogram must not be NULL").toLatin1().data());
    }
    else
    {
        *histogram = ito::DataObject(1,steps,ito::tInt32);
        histo_ptr = reinterpret_cast<ito::int32*>(histogram->rowPtr(0,0));
        memset(histo_ptr,0,steps * sizeof(ito::int32));
    }

    ito::DataObject *uniformDistribution = (ito::DataObject*)(*paramsOpt)[0].getVal<void*>();
    float *ud_ptr = NULL;
    if (uniformDistribution)
    {
        *uniformDistribution = ito::DataObject(1,100,ito::tFloat32);
        ud_ptr = reinterpret_cast<float*>(uniformDistribution->rowPtr(0,0));
        memset(ud_ptr,0,100 * sizeof(ito::float32));
    }

    // Attempt to get the field name's index
#if PCL_VERSION_COMPARE(>=,1,7,0)
    std::vector<pcl::PCLPointField> fields;
#else
    std::vector<sensor_msgs::PointField> fields;
#endif
    int distance_idx = 0;

    int idx = 0;
    const uint8_t* pt_data = NULL;
    uint32_t offset;
    float val;

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            distance_idx = pcl::getFieldIndex(*ptr, fieldName, fields);

            if (distance_idx > 0)
            {
                offset = fields[distance_idx].offset;
                for (int i = 0; i < ptr->size(); i++)
                {
                    pt_data = reinterpret_cast<const uint8_t*>(&ptr->points[i]);
                    memcpy(&val , pt_data + offset, sizeof(float));
                    if (std::isfinite(val))
                    {
                        idx += fillBucket(histo_ptr, &val, &minValue, &stepSize, steps);
                    }
                }
            }
        }
        break;
    }

    if (distance_idx == -1)
    {
        return ito::RetVal(ito::retError, 0, tr("Unable to find field name in point type.").toLatin1().data());
    }
    else if (distance_idx > 0)
    {
        if (ud_ptr)
        {
            int cumSum = 0;
            int percentageSumStep = idx / 100;
            int nextStep = percentageSumStep;

            if (percentageSumStep != 0)
            {
                int j = 0;
                //fill uniform distribution array
                for (int i = 0; i < steps; i++)
                {
                    cumSum += histo_ptr[i];

                    while(nextStep <= cumSum)
                    {
                        ud_ptr[j] = minValue + i * stepSize;
                        j++;
                        nextStep += percentageSumStep;
                    }
                }

                for (int j2 = j; j2<100;j2++)
                {
                    ud_ptr[j2] = maxValue;
                }
            }
        }
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclCylinderClipper3DDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclCylinderClipper3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output point cloud").toLatin1().data()));

    paramsMand->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("point on axis of cylinder").toLatin1().data()));
    paramsMand->append(ito::Param("orientation", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("orientation vector of cylinder").toLatin1().data()));
    paramsMand->append(ito::Param("radius", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("array with [min,max] radius").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclCylinderClipper3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    bool inplace = false; //no real inplace is possible

    if (pclIn == NULL || pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    double *point = (*paramsMand)[2].getVal<double*>();
    if ((*paramsMand)[2].getLen() != 3)
    {
        return ito::RetVal(ito::retError, 0, tr("point on axis must have 3 elements").toLatin1().data());
    }

    double *orientation = (*paramsMand)[3].getVal<double*>();
    if ((*paramsMand)[3].getLen() != 3)
    {
        return ito::RetVal(ito::retError, 0, tr("orientation vector of cylinder must have 3 elements").toLatin1().data());
    }

    Eigen::Vector3f pt_on_axis(point[0], point[1], point[2]);
    Eigen::Vector3f axis_direction(orientation[0], orientation[1], orientation[2]);

    double *radius = (*paramsMand)[4].getVal<double*>();
    if ((*paramsMand)[4].getLen() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("radius must contain two values").toLatin1().data());
    }

    std::vector<int > clipped;

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::CylinderClipper3D<pcl::PointXYZ> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::CylinderClipper3D<pcl::PointXYZI> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::CylinderClipper3D<pcl::PointXYZRGBA> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::CylinderClipper3D<pcl::PointNormal> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::CylinderClipper3D<pcl::PointXYZINormal> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::CylinderClipper3D<pcl::PointXYZRGBNormal> clipper(pt_on_axis, axis_direction);
            clipper.setMinMaxRadius(radius[0], radius[1]);
            clipper.clipPointCloud3D(*ptr, clipped, std::vector<int>());
        }

        break;
    }

    *pclOut = ito::PCLPointCloud(*pclIn, clipped);

    return ito::retOk;
#else
    return ito::RetVal(ito::retError, 0, tr("CylinderClipper3D not supported in PCL version < 1.7.0").toLatin1().data());
#endif
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclPCADOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPCAParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid input point cloud").toLatin1().data()));

    paramsOpt->append(ito::Param("eigenVectors", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("results in 3x3 float32 data object with three eigen-vectors").toLatin1().data()));

    paramsOut->append(ito::Param("mean", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("mean value").toLatin1().data()));
    paramsOut->append(ito::Param("eigenValues", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("eigen values").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPCA(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *ev = (ito::DataObject*)(*paramsOpt)[0].getVal<void*>();

    if (pclIn == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    Eigen::Vector4f mean;
    Eigen::Vector4d meanD;
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    Eigen::Vector3d eigenValuesD;
    //Eigen::MatrixXf coefficients;

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr = pclIn->toPointXYZ();
            pcl::PCA<pcl::PointXYZ> pca; //(*ptr);
            pca.setInputCloud(ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = pclIn->toPointXYZI();
            pcl::PCA<pcl::PointXYZI> pca; //(*ptr);
            pca.setInputCloud(ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr = pclIn->toPointXYZRGBA();
            pcl::PCA<pcl::PointXYZRGBA> pca; //(*ptr);
            pca.setInputCloud(ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr ptr = pclIn->toPointXYZNormal();
            pcl::PCA<pcl::PointNormal> pca; //(*ptr);
            pca.setInputCloud(ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr = pclIn->toPointXYZINormal();
            pcl::PCA<pcl::PointXYZINormal> pca; //(*ptr);
            pca.setInputCloud(ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ptr = pclIn->toPointXYZRGBNormal();
            pcl::PCA<pcl::PointXYZRGBNormal> pca; //(*ptr);
            pca.setInputCloud(ptr);
            //pcl::PCA<pcl::PointXYZRGBNormal> pca(*ptr);
            mean = pca.getMean();
            eigenVectors = pca.getEigenVectors();
            eigenValues = pca.getEigenValues();
        }

        break;
    }

    meanD = mean.cast<double>();
    (*paramsOut)[0].setVal<double*>(meanD.data(), 4);

    eigenValuesD = eigenValues.cast<double>();
    (*paramsOut)[1].setVal<double*>(eigenValuesD.data(), 3);

    if (ev)
    {
        ito::pclHelper::eigenMatrixToDataObj<float,3,3>(eigenVectors, *ev);
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------

const QString PclTools::pclTrimmedICPDOC = QObject::tr("");

ito::RetVal PclTools::pclTrimmedICPParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudTarget", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid target point cloud of type XYZ").toLatin1().data()));

    paramsMand->append(ito::Param("pointCloudSource", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Point cloud of same type than target cloud. This cloud is registered to the target.").toLatin1().data()));

    paramsMand->append(ito::Param("numSourcePointsToUse", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 100, tr("gives the number of closest source points taken into account for registration. By closest source points we mean the source points closest to the target. These points are computed anew at each iteration.").toLatin1().data()));
    paramsMand->append(ito::Param("transform", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("    is the estimated rigid transform. IMPORTANT: this matrix is also taken as the initial guess for the alignment. If there is no guess, set the matrix to identity!").toLatin1().data()));

    return retval;
}

ito::RetVal PclTools::pclTrimmedICP(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    ito::RetVal retval;
    const ito::PCLPointCloud *pclTarget = (*paramsMand)[0].getVal<ito::PCLPointCloud*>();
    const ito::PCLPointCloud *pclSource = (*paramsMand)[1].getVal<ito::PCLPointCloud*>();
    int pointsToUse = paramsMand->at(2).getVal<int>();
    ito::DataObject *transform = (*paramsMand)[3].getVal<ito::DataObject*>();
    Eigen::Matrix<float, 4, 4> transform_;

    if (!pclTarget || !pclSource || !transform)
    {
        retval += ito::RetVal(ito::retError, 0, "at least one mandatory argument is invalid or NULL");
    }
    else if (pclTarget->getType() != pclSource->getType())
    {
        retval += ito::RetVal(ito::retError, 0, "target and source cloud must have the same type pointXYZ");
    }
    else if (pclTarget->getType() != ito::pclXYZ)
    {
        retval += ito::RetVal(ito::retError, 0, "target and source cloud must have the same type pointXYZ");
    }


    if (!retval.containsError())
    {
        retval += ito::pclHelper::dataObjToEigenMatrix<float, 4, 4>(*transform, transform_);
    }

    if (!retval.containsError())
    {
        pointsToUse = std::min((uint32_t)pointsToUse, pclSource->height() * pclSource->width());
        switch(pclTarget->getType())
        {
        case ito::pclInvalid:
            break;
        case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
            {
                pcl::recognition::TrimmedICP<pcl::PointXYZ, float> icp;
                icp.init(pclTarget->toPointXYZ());
                icp.align(*(pclSource->toPointXYZ()), pointsToUse, transform_);
            }
            break;
        }
    }

    if (!retval.containsError())
    {
        ito::pclHelper::eigenMatrixToDataObj<float, 4, 4>(transform_, *transform);
    }

    return retval;
#else
    return ito::RetVal(ito::retError, 0, "PCL >= 1.7.0 is required for trimmed ICP");
#endif
}


//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclPolygonMeshFromIndicesDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPolygonMeshFromIndicesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("meshIn", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("Valid polygon mesh").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output polygon mesh").toLatin1().data()));
    paramsMand->append(ito::Param("indices", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("vector with indices of polygons that will be copied into output mesh").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclPolygonMeshFromIndices(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPolygonMesh *meshIn = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPolygonMesh *meshOut = (ito::PCLPolygonMesh*)(*paramsMand)[1].getVal<void*>();
    int *idx = (int*)(*paramsMand)[2].getVal<int*>();
    int idxSize = (*paramsMand)[2].getLen();

    bool inplace = false; //no real inplace is possible

    if (meshIn == NULL || meshIn->polygonMesh().get() == NULL || meshOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("polygon mesh must not be NULL").toLatin1().data());
    }

    pcl::PolygonMesh *mIn = meshIn->polygonMesh().get();

    pcl::PolygonMesh::Ptr polyMesh(new pcl::PolygonMesh());
    pcl::PolygonMesh *mOut = polyMesh.get();

    //prepare bucket for points
    int points = mIn->cloud.width * mIn->cloud.height;
    ito::uint8 *buckets = new ito::uint8[ points ];
    memset(buckets,0, sizeof(ito::uint8) * points);

    //prepare lut that points the old point index to the new point index in the reduced point cloud
    ito::uint32 *lut = new ito::uint32[points];

    mOut->header = mIn->header; //copy header to output
    std::vector<pcl::Vertices> &outPolygons = mOut->polygons;
    std::vector<pcl::Vertices> &inPolygons = mIn->polygons;
    int verticeIdx;
    int maxVertice = inPolygons.size();
    pcl::Vertices v;
    int counter = 0;

    //copy vertices
    outPolygons.clear();

    for (int i = 0; i < idxSize; i++)
    {
        verticeIdx = idx[i];
        if (verticeIdx < 0 || verticeIdx >= maxVertice)
        {
            delete[] buckets;
            return ito::RetVal::format(ito::retError, 0, tr("vertice index [%i] is out of range").toLatin1().data(), verticeIdx);
        }

        v = inPolygons[verticeIdx]; //deep copy (this is good, since the vertice-indices might be changed)

        for (int j = 0; j < v.vertices.size(); j++)
        {
            ++buckets[ v.vertices[j] ];
            if (buckets[ v.vertices[j] ] == 1) //this point is used for the first time
            {
                lut[ v.vertices[j] ] = counter; //change index of point to its index in output cloud
                v.vertices[j] = counter;
                counter++;
            }
            else
            {
                v.vertices[j] = lut[ v.vertices[j] ];
            }
        }

        outPolygons.push_back(v);
    }

    //copy points
    mOut->cloud.fields = mIn->cloud.fields;
    mOut->cloud.header = mIn->cloud.header;
    mOut->cloud.height = 1;
    mOut->cloud.is_bigendian = mIn->cloud.is_bigendian;
    mOut->cloud.is_dense = false;
    mOut->cloud.point_step = mIn->cloud.point_step;
    uint32_t pointStep = mIn->cloud.point_step;

#if PCL_VERSION_COMPARE(>=,1,11,0)
    typedef std::uint8_t pcluint8_t;
#else
    typedef pcl::uint8_t pcluint8_t;
#endif


    std::vector<pcluint8_t> &data = mOut->cloud.data;
    pcluint8_t *ptrIn = &(mIn->cloud.data[0]);

    data.resize(counter * pointStep);

    if (idxSize > 0)
    {
        pcluint8_t *ptrOut = &(data[0]);

        for (int i = 0; i < points; i++)
        {
            if (buckets[i] > 0)
            {
                memcpy(ptrOut + pointStep * lut[i] * sizeof(pcluint8_t), ptrIn + pointStep * i * sizeof(pcluint8_t), pointStep * sizeof(pcluint8_t));
            }
        }

        mOut->cloud.width = counter;
    }

    delete[] buckets;
    buckets = NULL;

    delete[] lut;
    lut = NULL;

    *meshOut = polyMesh;

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclMeshTriangulationDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclMeshTriangulationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("meshIn", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("Valid polygon mesh").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output polygon mesh").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclMeshTriangulation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPolygonMesh *meshIn = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPolygonMesh *meshOut = (ito::PCLPolygonMesh*)(*paramsMand)[1].getVal<void*>();

    bool inplace = false; //no real inplace is possible

    if (meshIn == NULL || meshIn->polygonMesh().get() == NULL || meshOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("polygon mesh must not be NULL").toLatin1().data());
    }

    pcl::PolygonMesh *mIn = meshIn->polygonMesh().get();

    pcl::PolygonMesh::Ptr polyMesh(new pcl::PolygonMesh());

    pcl::EarClipping earClipper;
    earClipper.setInputMesh(meshIn->polygonMesh());
    earClipper.process(*polyMesh);

    *meshOut = polyMesh;

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclSampleToDataObjectDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclSampleToDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPointCloud *pointCloud = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *disperityMap = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    ito::DataObject *intensityMap = (ito::DataObject*)(*paramsOpt)[0].getVal<void*>();
    ito::DataObject *deviationMap = (ito::DataObject*)(*paramsOpt)[1].getVal<void*>();

    int width = (*paramsMand)[2].getVal<int>();
    int height = (*paramsMand)[3].getVal<int>();

    ito::RetVal retval = ito::retOk;

    bool getDisperity = true;
    bool getIntensity = intensityMap != NULL;
    bool getDeviation = deviationMap != NULL;

    if (!pointCloud->isOrganized() && !pointCloud->is_dense())
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must be organized and dense").toLatin1().data());
    }

    if (pointCloud == NULL || pointCloud->width() < 1 || pointCloud->height() < 1)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud pointer or content must not be NULL").toLatin1().data());
    }

    if (pointCloud->width() * pointCloud->height() != height * width)
    {
        return ito::RetVal(ito::retError, 0, tr("number of element of pointcloud must be identical to width*height").toLatin1().data());
    }

    if (disperityMap == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("disperity output must not be NULL").toLatin1().data());
    }

    if (intensityMap == disperityMap)
    {
        return ito::RetVal(ito::retError, 0, tr("intensity output dataObject must differ from disperity dataObject").toLatin1().data());
    }

    if ((deviationMap == disperityMap || deviationMap == intensityMap) && deviationMap != NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("curvature output dataObject must differ from disperity dataObject").toLatin1().data());
    }

    if (getDisperity)
    {
        *disperityMap = ito::DataObject(height, width, ito::tFloat32);
    }

    cv::Mat* dispMat = (cv::Mat*)(disperityMap->get_mdata()[disperityMap->seekMat(0)]);
    cv::Mat* intMat = NULL;
    cv::Mat* devMat = NULL;

    if (getIntensity)
    {
        if (pointCloud->getType() == ito::pclXYZI  || pointCloud->getType() == ito::pclXYZINormal)
        {
            *intensityMap = ito::DataObject(height, width, ito::tFloat32);
        }
        /*
        else if (pointCloud->getType() == ito::pclXYZRGBA || pointCloud->getType() == ito::pclXYZRGBNormal)
        {
            *deviationMap = ito::DataObject(height, width, ito::tRGBA32);
        }
        */
        else
        {
            return ito::RetVal(ito::retError, 0, tr("point cloud must be of type a with an intensity or rgba vector to extract intensity").toLatin1().data());
        }
        intMat = (cv::Mat*)(intensityMap->get_mdata()[intensityMap->seekMat(0)]);
    }

    if (getDeviation)
    {
        if (pointCloud->getType() == ito::pclXYZNormal  || pointCloud->getType() == ito::pclXYZINormal || pointCloud->getType() == ito::pclXYZRGBNormal)
        {
            *deviationMap = ito::DataObject(height, width, ito::tFloat32);
        }

        else
        {
            return ito::RetVal(ito::retError, 0, tr("point cloud must be of type a with defined normals to extract intensity").toLatin1().data());
        }
        devMat = (cv::Mat*)(deviationMap->get_mdata()[deviationMap->seekMat(0)]);
    }

    switch(pointCloud->getType())
    {
    default:
        retval += ito::RetVal(ito::retError, 0, tr("point cloud type not supported be given").toLatin1().data());
        break;
    case ito::pclInvalid:
        retval += ito::RetVal(ito::retError, 0, tr("a valid organized point cloud must be given").toLatin1().data());
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclSrc = pointCloud->toPointXYZ();


            //#if (USEOMP)
            //#pragma omp for schedule(guided)
            //#endif

            ito::float32* rowPtr = dispMat->ptr<ito::float32>(0);

            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif
            for (int np = 0; np < pointCloud->size(); np++)
            {
                rowPtr[np] = pclSrc->at(np).z;
            }

        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pclSrc = pointCloud->toPointXYZI();

            #if (USEOMP)
            #pragma omp parallel num_threads(nthreads)
            {
            #endif

            ito::float32* rowPtr = dispMat->ptr<ito::float32>(0);

            if (getIntensity)
            {
                ito::float32* intPtr = intMat->ptr<ito::float32>(0);

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                    intPtr[np] = pclSrc->at(np).intensity;
                }
            }
            else
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                }
            }

            #if (USEOMP)
            }
            #endif
        }
        break;
    /*
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {

        }
        break;
    */
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr pclSrc = pointCloud->toPointXYZNormal();

            #if (USEOMP)
            #pragma omp parallel num_threads(nthreads)
            {
            #endif
            ito::float32* rowPtr = dispMat->ptr<ito::float32>(0);

            if (getDeviation)
            {
                ito::float32* devPtr = devMat->ptr<ito::float32>(0);

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                    devPtr[np] = pclSrc->at(np).curvature;
                }
            }
            else
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                }
            }
            #if (USEOMP)
            }
            #endif
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclSrc = pointCloud->toPointXYZINormal();

            #if (USEOMP)
            #pragma omp parallel num_threads(nthreads)
            {
            #endif
            ito::float32* rowPtr = dispMat->ptr<ito::float32>(0);

            if (getIntensity && getDeviation)
            {
                ito::float32* intPtr = intMat->ptr<ito::float32>(0);
                ito::float32* devPtr = devMat->ptr<ito::float32>(0);

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                    intPtr[np] = pclSrc->at(np).intensity;
                    devPtr[np] = pclSrc->at(np).curvature;
                }
            }
            else if (getIntensity)
            {
                ito::float32* intPtr = intMat->ptr<ito::float32>(0);

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                    intPtr[np] = pclSrc->at(np).intensity;
                }
            }
            else if (getDeviation)
            {
                ito::float32* devPtr = devMat->ptr<ito::float32>(0);

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                    devPtr[np] = pclSrc->at(np).curvature;
                }
            }
            else
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pointCloud->size(); np++)
                {
                    rowPtr[np] = pclSrc->at(np).z;
                }
            }
            #if (USEOMP)
            }
            #endif
        }
        break;
    /*
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {

        }

        break;
    */
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclSampleToDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("organizedCloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid, organized point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("disperityMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output dataObject with z-Values").toLatin1().data()));
    paramsMand->append(ito::Param("width", ito::ParamBase::Int | ito::ParamBase::In , 1, std::numeric_limits<int>::max(), 1280, tr("Output dataObject with z-Values").toLatin1().data()));
    paramsMand->append(ito::Param("height", ito::ParamBase::Int | ito::ParamBase::In , 1, std::numeric_limits<int>::max(), 1024, tr("Output dataObject with z-Values").toLatin1().data()));
    paramsOpt->append(ito::Param("intensityMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output dataObject with intensity-Values").toLatin1().data()));
    paramsOpt->append(ito::Param("curvatureMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output dataObject with curvature-Values").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclOrganizedFastMeshDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclOrganizedFastMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("organizedCloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid, organized point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output polygonal mesh").toLatin1().data()));

    paramsOpt->clear();
    ito::Param triangleType("triangulationType", ito::ParamBase::String | ito::ParamBase::In, "TRIANGLE_RIGHT_CUT", tr("'TRIANGLE_RIGHT_CUT': _always_ cuts a quad from top left to bottom right (default), 'TRIANGLE_LEFT_CUT': _always_ cuts a quad from top right to bottom left, 'TRIANGLE_ADAPTIVE_CUT': cuts where possible and prefers larger differences in z direction, 'QUAD_MESH': create a simple quad mesh").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);
    sm->addItem("TRIANGLE_RIGHT_CUT");
    sm->addItem("TRIANGLE_LEFT_CUT");
    sm->addItem("TRIANGLE_ADAPTIVE_CUT");
    sm->addItem("QUAD_MESH");
    triangleType.setMeta(sm,true);
    paramsOpt->append(triangleType);

    paramsOpt->append(ito::Param("trianglePixelSizeRows", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 1, tr("Set the edge length (in pixels) used for iterating over rows when constructing the fixed mesh. Default: 1, neighboring pixels are connected").toLatin1().data()));
    paramsOpt->append(ito::Param("trianglePixelSizeColumns", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 1, tr("Set the edge length (in pixels) used for iterating over columns when constructing the fixed mesh. Default: 1, neighboring pixels are connected").toLatin1().data()));
    paramsOpt->append(ito::Param("storeShadowFaces", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Store shadowed faces or not (default: 1, yes).").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclOrganizedFastMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPointCloud *pointCloud = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPolygonMesh *polygonMesh = (ito::PCLPolygonMesh*)(*paramsMand)[1].getVal<void*>();

    int psRows = paramsOpt->at(1).getVal<int>();
    int psCols = paramsOpt->at(2).getVal<int>();
    bool storeShadowFaces = paramsOpt->at(3).getVal<int>() > 0;
    QString typeStr = paramsOpt->at(0).getVal<char*>();

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    if (psRows != 1 || psCols != 1)
    {
        return ito::RetVal(ito::retError, 0, tr("with a PCL < 1.7.0 trianglePixelSizeRows and trianglePixelSizeColumns must be 1").toLatin1().data());
    }
#endif

    pcl::OrganizedFastMesh<pcl::PointXYZ>::TriangulationType type;

    if (QString::compare(typeStr, "TRIANGLE_RIGHT_CUT") == 0)
    {
        type = pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT;
    }
    else if (QString::compare(typeStr, "TRIANGLE_LEFT_CUT") == 0)
    {
        type = pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_LEFT_CUT;
    }
    else if (QString::compare(typeStr, "TRIANGLE_ADAPTIVE_CUT") == 0)
    {
        type = pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT;
    }
    else if (QString::compare(typeStr, "QUAD_MESH") == 0)
    {
        type = pcl::OrganizedFastMesh<pcl::PointXYZ>::QUAD_MESH;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("wrong triangulationType parameter.").toLatin1().data());
    }

    if (polygonMesh->valid() == false)
    {
        *polygonMesh = ito::PCLPolygonMesh(pcl::PolygonMesh::Ptr(new pcl::PolygonMesh()));
    }



    if (pointCloud == NULL || polygonMesh == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("the parameters organizedCloud and meshOut must not be NULL").toLatin1().data());
    }

    if (pointCloud->getType() != ito::pclInvalid && pointCloud->isOrganized() == false)
    {
        return ito::RetVal(ito::retError, 0, tr("the given point cloud must be organized. The height property of an organized point cloud is bigger than one.").toLatin1().data());
    }

    switch(pointCloud->getType())
    {
    case ito::pclInvalid:
        retval += ito::RetVal(ito::retError, 0, tr("a valid organized point cloud must be given").toLatin1().data());
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZ> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZConst());
            fastMesh.setTriangulationType(type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZI> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZIConst());
            fastMesh.setTriangulationType((pcl::OrganizedFastMesh<pcl::PointXYZI>::TriangulationType)type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZRGBA> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZRGBAConst());
            fastMesh.setTriangulationType((pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TriangulationType)type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointNormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZNormalConst());
            fastMesh.setTriangulationType((pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType)type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZINormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZINormalConst());
            fastMesh.setTriangulationType((pcl::OrganizedFastMesh<pcl::PointXYZINormal>::TriangulationType)type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
 #if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZRGBNormalConst());
            fastMesh.setTriangulationType((pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::TriangulationType)type);
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct(*(polygonMesh->polygonMesh()));
        }

        break;
    }

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclSimplifyMeshDOC = QObject::tr("\n\
\n\
\n\
\n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclSimplifyMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("meshIn", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("Valid, organized point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output polygonal mesh").toLatin1().data()));

    paramsOpt->clear();
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of deleted elements").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclSimplifyMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    retval += ito::RetVal(ito::retError, 0, tr("Only tested / implemented for version 1.7.0").toLatin1().data());
#else
    ito::PCLPolygonMesh *meshIn = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPolygonMesh *meshOut = (ito::PCLPolygonMesh*)(*paramsMand)[1].getVal<void*>();

    if (meshIn == NULL || meshOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("the parameters meshIn and meshOut must not be NULL.").toLatin1().data());
    }

    if (meshIn->valid() == false)
    {
        return ito::RetVal(ito::retError, 0, tr("the input mesh must be valid.").toLatin1().data());
    }

    if (meshOut->valid() == false)
    {
        *meshOut = ito::PCLPolygonMesh(pcl::PolygonMesh::Ptr(new pcl::PolygonMesh()));
    }

    std::vector<int> deletedOnes;

    pcl::surface::SimplificationRemoveUnusedVertices cleaner;
    cleaner.simplify(*(meshIn->polygonMesh()), *(meshOut->polygonMesh()), deletedOnes);

    paramsOut->data()[0].setVal<int>(deletedOnes.size());
#endif
    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclPoissonDOC = QObject::tr("Uses pcl::Poisson-filter to reduce a mesh / estimate the surface of an object.");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclPoissonParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("cloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, nullptr, tr("Input point cloud with normal vector information.").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, nullptr, tr("Output polygonal mesh").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("treeDepth", ito::ParamBase::Int | ito::ParamBase::In , 1, 100, 8, tr("Maximum depth of the octTree to reconstruct. Be careful: High values might require a lot of memory and processing time.").toLatin1().data()));
    paramsOpt->append(ito::Param("minTreeDepth", ito::ParamBase::Int | ito::ParamBase::In, 1, 100, 5, tr("The minimum depth.").toLatin1().data()));
    paramsOpt->append(ito::Param("isoDivide", ito::ParamBase::Int | ito::ParamBase::In, 1, 100, 8,
        tr("Set the depth at which a block iso-surface extractor should be used to extract the iso-surface.\n\
This parameter must be >= minTreeDepth. \n\
\n\
Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. \n\
(In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)").toLatin1().data()));

    paramsOpt->append(ito::Param("solverDivide", ito::ParamBase::Int | ito::ParamBase::In, 1, 100, 8,
        tr("Get the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation.\n\
This parameter must be >= minTreeDepth. \n\
\n\
Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. \n\
(In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)").toLatin1().data()));
    //paramsOpt->append(ito::Param("threads", ito::ParamBase::Int | ito::ParamBase::In, 0, 32, 1, tr("The number of threads to use for computation. 0: use the maximum available number of threads.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclPoisson(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    retval += ito::RetVal(ito::retError, 0, tr("Only tested / implemented for PCL >= 1.7.0").toLatin1().data());

#else
    const ito::PCLPointCloud *cloudIn = (*paramsMand)[0].getVal<const ito::PCLPointCloud*>();
    ito::PCLPolygonMesh *meshOut = (*paramsMand)[1].getVal<ito::PCLPolygonMesh*>();

    int maxDepth = (*paramsOpt)[0].getVal<int>();
    int minDepth = paramsOpt->at(1).getVal<int>();
    int isoDivide = paramsOpt->at(2).getVal<int>();
    int solverDivide = paramsOpt->at(3).getVal<int>();
    //int threads = paramsOpt->at(4).getVal<int>();
    int degree = -1;

    if (isoDivide < minDepth)
    {
        return ito::RetVal(ito::retError, 0, tr("isoDivide must be >= minDepth").toLatin1().data());
    }

    if (solverDivide < minDepth)
    {
        return ito::RetVal(ito::retError, 0, tr("solverDivide must be >= minDepth").toLatin1().data());
    }

    if (cloudIn == NULL || meshOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("the parameters meshIn and meshOut must not be NULL.").toLatin1().data());
    }

    if (cloudIn->getType() == ito::pclInvalid || cloudIn->empty())
    {
        return ito::RetVal(ito::retError, 0, tr("the input point cloud must be valid.").toLatin1().data());
    }

    *meshOut = ito::PCLPolygonMesh(pcl::PolygonMesh::Ptr(new pcl::PolygonMesh()));

    switch(cloudIn->getType())
    {
        default:
            return ito::RetVal(ito::retError, 0, tr("The type of the input point cloud must be XYZNormal, XYZINormal or XYZRGBNormal.").toLatin1().data());
        case ito::pclInvalid:
            return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not defined or point cloud invalid").toLatin1().data());
        case ito::pclXYZNormal:
        {
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth(maxDepth);
            poisson.setMinDepth(minDepth);
            //poisson.setThreads(threads);
            poisson.setSolverDivide(solverDivide);
            poisson.setIsoDivide(isoDivide);
            poisson.setInputCloud(cloudIn->toPointXYZNormal());
            if (degree > 0) poisson.setDegree(degree);
            poisson.reconstruct(*(meshOut->polygonMesh()));
        }
        break;
        case ito::pclXYZINormal:
        {
            pcl::Poisson<pcl::PointXYZINormal> poisson;
            poisson.setDepth(maxDepth);
            poisson.setMinDepth(minDepth);
            //poisson.setThreads(threads);
            poisson.setSolverDivide(solverDivide);
            poisson.setIsoDivide(isoDivide);
            poisson.setInputCloud(cloudIn->toPointXYZINormal());
            if (degree > 0) poisson.setDegree(degree);
            poisson.reconstruct(*(meshOut->polygonMesh()));
        }
        break;
        case ito::pclXYZRGBNormal:
        {
            pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
            poisson.setDepth(maxDepth);
            poisson.setMinDepth(minDepth);
            //poisson.setThreads(threads);
            poisson.setSolverDivide(solverDivide);
            poisson.setIsoDivide(isoDivide);
            poisson.setInputCloud(cloudIn->toPointXYZRGBNormal());
            if (degree > 0) poisson.setDegree(degree);
            poisson.reconstruct(*(meshOut->polygonMesh()));
        }
        break;
    }
#endif
    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclMarchingCubesDOC = QObject::tr(
"The marching cubes surface reconstruction algorithm. \n\
\n\
There are two algorithms implemented: \n\
\n\
1. MarchingCubesHoppe: \n\
    using a signed distance function based on the distance \n\
    from tangent planes, proposed by Hoppe et. al.in: \n\
    \n\
    Hoppe H., DeRose T., Duchamp T., MC - Donald J., Stuetzle W., \n\
    \"Surface reconstruction from unorganized points\", SIGGRAPH '92 \n\
\n\
2. MarchingCubesRBF: \n\
    The marching cubes surface reconstruction algorithm, using a signed distance function based on radial \n\
    basis functions.Partially based on: \n\
    \n\
    Carr J.C., Beatson R.K., Cherrie J.B., Mitchell T.J., Fright W.R., McCallum B.C. and Evans T.R., \n\
    \"Reconstruction and representation of 3D objects with radial basis functions\" \n\
    SIGGRAPH '01");

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclMarchingCubesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("cloud", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, nullptr, tr("Input point cloud with normal vector information.").toLatin1().data()));
    paramsMand->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output polygonal mesh").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("algorithmType", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("0: MarchingCubesHoppe, 1: MarchingCubesRBF").toLatin1().data()));
    paramsOpt->append(ito::Param("isoLevel", ito::ParamBase::Double | ito::ParamBase::In, -100000.0, 100000.0, 0.0, tr("the iso level of the surface to be extracted.").toLatin1().data()));

    int gridResolution[] = { 32,32,32 };
    auto pGridRes = ito::Param("gridResolution", ito::ParamBase::IntArray | ito::ParamBase::In, 3, gridResolution, tr("The grid resolution in x, y, and z (default: 32 each)").toLatin1().data());
    pGridRes.setMeta(new ito::IntArrayMeta(INT_MIN, INT_MAX, 1, 3, 3, 1), true);
    paramsOpt->append(pGridRes);

    paramsOpt->append(ito::Param("percentageExtendGrid", ito::ParamBase::Double | ito::ParamBase::In, -100000.0, 100000.0, 0.0,
        tr("parameter that defines how much free space should be left inside the grid between the bounding box of the point cloud and the grid limits, as a percentage of the bounding box.").toLatin1().data()));

    paramsOpt->append(ito::Param("distIgnore", ito::ParamBase::Double | ito::ParamBase::In, -1.0, 100000.0, -1.0,
        tr("Method that sets the distance for ignoring voxels which are far from point cloud. \n\
If the distance is negative, then the distance functions would be calculated in all voxels; \n\
otherwise, only voxels with distance lower than dist_ignore would be involved in marching cube. \n\
Default value is - 1.0. Set to negative if all voxels are to be involved. \n\
Only used for algorithmType = MarchingCubesHoppe (0).").toLatin1().data()));

    paramsOpt->append(ito::Param("offSurfaceEpsilon", ito::ParamBase::Double | ito::ParamBase::In, -100000.0, 100000.0, 0.1,
        tr("Set the off - surface points displacement value. Only used for algorithmType = MarchingCubesRBF (1)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::pclMarchingCubes(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    retval += ito::RetVal(ito::retError, 0, tr("Only tested / implemented for PCL >= 1.7.0").toLatin1().data());

#else
    auto cloudIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<const ito::PCLPointCloud*>();
    auto meshOut = (ito::PCLPolygonMesh*)(*paramsMand)[1].getVal<ito::PCLPolygonMesh*>();

    bool useHoppeNotRBF = (paramsOpt->at(0).getVal<int>() == 0);
    double isoLevel = paramsOpt->at(1).getVal<double>();
    const int* gridRes = paramsOpt->at(2).getVal<const int*>();
    double percentageExtendGrid = (*paramsOpt)[3].getVal<double>();
    double distIgnore = (*paramsOpt)[4].getVal<double>();
    double offSurfaceEpsilon = (*paramsOpt)[5].getVal<double>();

    if (cloudIn == NULL || meshOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("the parameters cloud and meshOut must not be nullptr.").toLatin1().data());
    }

    if (cloudIn->getType() == ito::pclInvalid || cloudIn->empty())
    {
        return ito::RetVal(ito::retError, 0, tr("the input point cloud must be valid.").toLatin1().data());
    }

    *meshOut = ito::PCLPolygonMesh(pcl::PolygonMesh::Ptr(new pcl::PolygonMesh()));

    switch (cloudIn->getType())
    {
    default:
        return ito::RetVal(ito::retError, 0, tr("The type of the input point cloud must be XYZNormal, XYZINormal or XYZRGBNormal.").toLatin1().data());
    case ito::pclInvalid:
        return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not defined or point cloud invalid").toLatin1().data());
    case ito::pclXYZNormal:
    {
        pcl::MarchingCubes<pcl::PointNormal> *mc;

        if (useHoppeNotRBF)
        {
            mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>(distIgnore, percentageExtendGrid, isoLevel);

        }
        else
        {
            mc = new pcl::MarchingCubesRBF<pcl::PointNormal>(offSurfaceEpsilon, percentageExtendGrid, isoLevel);
        }

        mc->setGridResolution(gridRes[0], gridRes[1], gridRes[2]);
        mc->setInputCloud(cloudIn->toPointXYZNormal());
        mc->reconstruct(*(meshOut->polygonMesh()));
        delete mc;
    }
    break;
    case ito::pclXYZINormal:
    {
        pcl::MarchingCubes<pcl::PointXYZINormal> *mc;

        if (useHoppeNotRBF)
        {
            mc = new pcl::MarchingCubesHoppe<pcl::PointXYZINormal>(distIgnore, percentageExtendGrid, isoLevel);

        }
        else
        {
            mc = new pcl::MarchingCubesRBF<pcl::PointXYZINormal>(offSurfaceEpsilon, percentageExtendGrid, isoLevel);
        }

        mc->setGridResolution(gridRes[0], gridRes[1], gridRes[2]);
        mc->setInputCloud(cloudIn->toPointXYZINormal());
        mc->reconstruct(*(meshOut->polygonMesh()));
        delete mc;
    }
    break;
    case ito::pclXYZRGBNormal:
    {
        pcl::MarchingCubes<pcl::PointXYZRGBNormal> *mc;

        if (useHoppeNotRBF)
        {
            mc = new pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal>(distIgnore, percentageExtendGrid, isoLevel);

        }
        else
        {
            mc = new pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal>(offSurfaceEpsilon, percentageExtendGrid, isoLevel);
        }

        mc->setGridResolution(gridRes[0], gridRes[1], gridRes[2]);
        mc->setInputCloud(cloudIn->toPointXYZRGBNormal());
        mc->reconstruct(*(meshOut->polygonMesh()));
        delete mc;
    }
    break;
    }
#endif
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString PclTools::pclGetNormalsAtCogFromMeshDOC = QObject::tr("calculates a point cloud with normal information which contains the normal at each triangle of the given \n\
polygonal mesh centered at the center of gravity of the triangle. Use indices to filter only certain triangles.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetNormalsAtCogFromMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("mesh", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("Valid polygonal mesh").toLatin1().data()));
    paramsMand->append(ito::Param("cloudWithNormals", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("output point cloud with the same point type than contained in the mesh but including normal vectors.").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("indices", ito::ParamBase::IntArray | ito::ParamBase::In , NULL, tr("Optional list with indices that should be considered.").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclGetNormalsAtCogFromMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    const ito::PCLPolygonMesh *mesh = paramsMand->at(0).getVal<const ito::PCLPolygonMesh*>();
    ito::PCLPointCloud *cloud = paramsMand->at(1).getVal<ito::PCLPointCloud*>();
    const int *indices = paramsOpt->at(0).getVal<int*>();
    int indices_len = paramsOpt->at(0).getLen();

    if (indices_len > 0)
    {
        std::vector<int> idx;
        idx.resize(indices_len);
        memcpy(idx.data(), indices, indices_len *sizeof(int));
        return ito::pclHelper::normalsAtCogFromPolygonMesh(*mesh, *cloud, idx);
    }
    else
    {
        return ito::pclHelper::normalsAtCogFromPolygonMesh(*mesh, *cloud);
    }
}

// ---------------------------------------------------------------------- DO NOT ADD FILTER BELOW THIS!!! -----------------------------------------------------------
/** initialize filter functions within this addIn
*    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on initialization
*    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on initialization
*    @return                    retError in case of an error
*
*    Here are the filter functions defined that are available through this addIn.
*    These are:
*       - filterName    description for this filter
*
*   This plugin additionally makes available the following widgets, dialogs...:
*       - dialogName    description for this widget
*/
ito::RetVal PclTools::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;
    AlgoWidgetDef *widget = NULL;

    //specify filters here, example:
    filter = new FilterDef(PclTools::savePointCloud, PclTools::savePointCloudParams, tr("saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii), xyz(ascii))"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWritePointCloud, tr("Point Cloud (*.pcd *.ply *.vtk *.xyz)"));
    m_filterList.insert("savePointCloud", filter);

    filter = new FilterDef(PclTools::loadPointCloud, PclTools::loadPointCloudParams, tr("loads pointCloud from hard drive and returns it (format pcd(binary or ascii), ply(binary or ascii))"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadPointCloud, tr("Point Cloud (*.pcd *.ply *.xyz)"));
    m_filterList.insert("loadPointCloud", filter);

    filter = new FilterDef(PclTools::savePolygonMesh, PclTools::savePolygonMeshParams, tr("saves polygonMesh to hard drive (format obj[default], ply, vtk, stl)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWritePolygonMesh, tr("Polygon Mesh (*.obj *.ply *.vtk *.stl)"));
    m_filterList.insert("savePolygonMesh", filter);

    filter = new FilterDef(PclTools::loadPolygonMesh, PclTools::loadPolygonMeshParams, tr("loads polygonMesh from hard drive and returns it (format obj[default], ply, vtk, stl)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadPolygonMesh, tr("Polygon Mesh (*.obj *.ply *.vtk *.stl)"));
    m_filterList.insert("loadPolygonMesh", filter);

    filter = new FilterDef(PclTools::saveVTKImageData, PclTools::saveVTKImageDataParams, saveVTKImageDataDOC, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("VTK Image Data File (*.vti)"));
    m_filterList.insert("saveVTKImageData", filter);

    filter = new FilterDef(PclTools::transformAffine, PclTools::transformAffineParams, transformAffineDOC);
    m_filterList.insert("pclTransformAffine", filter);

    filter = new FilterDef(PclTools::transformAffineMesh, PclTools::transformAffineMeshParams, transformAffineMeshDOC);
    m_filterList.insert("meshTransformAffine", filter);

    filter = new FilterDef(PclTools::pclFitModel, PclTools::pclFitModelParams, pclFitModelDOC);
    m_filterList.insert("pclFitModel", filter);

    filter = new FilterDef(PclTools::pclFitModelDObj, PclTools::pclFitModelDObjParams, pclFitModelDObjDOC);
    m_filterList.insert("pclFitModelDObj", filter);

    filter = new FilterDef(PclTools::pclFitCylinder, PclTools::pclFitCylinderParams, pclFitCylinderDOC);
    m_filterList.insert("pclFitCylinder", filter);

    filter = new FilterDef(PclTools::pclFitSphere, PclTools::pclFitSphereParams, pclFitSphereDOC);
    m_filterList.insert("pclFitSphere", filter);

    filter = new FilterDef(PclTools::pclFitCircle2D, PclTools::pclFitCircle2DParams, pclFitCircle2DDOC);
    m_filterList.insert("pclFitCircle2D", filter);

    filter = new FilterDef(PclTools::pclFitCircle3D, PclTools::pclFitCircle3DParams, pclFitCircle3DDOC);
    m_filterList.insert("pclFitCircle3D", filter);

    filter = new FilterDef(PclTools::pclFitLine, PclTools::pclFitLineParams, pclFitLineDOC);
    m_filterList.insert("pclFitLine", filter);

    filter = new FilterDef(PclTools::pclFitPlane, PclTools::pclFitPlaneParams, pclFitPlaneDOC);
    m_filterList.insert("pclFitPlane", filter);

    filter = new FilterDef(PclTools::pclFitCone, PclTools::pclFitConeParams, pclFitConeDOC);
    m_filterList.insert("pclFitCone", filter);

    filter = new FilterDef(PclTools::pclDistanceToModel, PclTools::pclDistanceToModelParams, pclDistanceToModelDOC);
    m_filterList.insert("pclDistanceToModel", filter);

    filter = new FilterDef(PclTools::pclDistanceToModelDObj, PclTools::pclDistanceToModelDObjParams, pclDistanceToModelDObjDOC);
    m_filterList.insert("pclDistanceToModelDObj", filter);

    filter = new FilterDef(PclTools::pclProjectOnModel, PclTools::pclProjectOnModelParams, tr("Projects points onto a given model."));
    m_filterList.insert("pclProjectOnModel", filter);

    filter = new FilterDef(PclTools::pclEstimateNormals, PclTools::pclEstimateNormalsParams, tr("estimates normal vectors to the given input point cloud and returns the normal-enhanced representation of the input point cloud"));
    m_filterList.insert("pclEstimateNormals", filter);

    /*filter = new FilterDef(PclTools::pclEstimateMaxCurvature, PclTools::pclEstimateMaxCurvatureParams, pclEstimateMaxCurvatureDOC);
    m_filterList.insert("pclEstimateMaxCurvature", filter);*/

    filter = new FilterDef(PclTools::pclRemoveNaN, PclTools::pclRemoveNaNParams, tr("removes NaN values from input point cloud (input and output can be the same)."));
    m_filterList.insert("pclRemoveNaN", filter);

    filter = new FilterDef(PclTools::pclPassThrough, PclTools::pclPassThroughParams, tr("filters a point cloud by giving boundary values to a specific dimension (outside or inside of this field)."));
    m_filterList.insert("pclPassThrough", filter);

    filter = new FilterDef(PclTools::pclCropBox, PclTools::pclCropBoxParams, pclCropBoxDOC);
    m_filterList.insert("pclCropBox", filter);

    filter = new FilterDef(PclTools::pclVoxelGrid, PclTools::pclVoxelGridParams, tr("downsamples a point cloud using a voxelized gripd approach."));
    m_filterList.insert("pclVoxelGrid", filter);

    filter = new FilterDef(PclTools::pclStatisticalOutlierRemoval, PclTools::pclStatisticalOutlierRemovalParams, tr("uses point neighborhood statistics to filter outlier data."));
    m_filterList.insert("pclStatisticalOutlierRemoval", filter);

    filter = new FilterDef(PclTools::pclRandomSample, PclTools::pclRandomSampleParams, tr("randomly reduces the point cloud to a given number of points."));
    m_filterList.insert("pclRandomSample", filter);

    filter = new FilterDef(PclTools::pclGetMinMax3D, PclTools::pclGetMinMax3DParams, tr("get the minimum and maximum values on each of the 3 (x-y-z) dimensions in a given pointcloud."));
    m_filterList.insert("pclGetMinMax3D", filter);

    filter = new FilterDef(PclTools::pclGetPercentageThreshold, PclTools::pclGetPercentageThresholdParams, tr("returns the threshold value at the percentage value in the sorted values of the specific field."));
    m_filterList.insert("pclGetPercentageThreshold", filter);

    filter = new FilterDef(PclTools::pclGetHistogram, PclTools::pclGetHistogramParams, tr("returns the histogram of the specific field."));
    m_filterList.insert("pclGetHistogram", filter);

    filter = new FilterDef(PclTools::pclCylinderClipper3D, PclTools::pclCylinderClipper3DParams, tr("clips points from the point cloud that lie outside a given cylindrical tube."));
    m_filterList.insert("pclCylinderClipper3D", filter);

    filter = new FilterDef(PclTools::pclPCA, PclTools::pclPCAParams, tr("determines PCA of point Cloud."));
    m_filterList.insert("pclPCA", filter);

    filter = new FilterDef(PclTools::pclPolygonMeshFromIndices, PclTools::pclPolygonMeshFromIndicesParams, tr("get sub-mesh from given mesh by the indices of the polygons."));
    m_filterList.insert("pclPolygonMeshFromIndices", filter);

    filter = new FilterDef(PclTools::pclMeshTriangulation, PclTools::pclMeshTriangulationParams, tr("calculates polygon mesh with triangles only. This is based on the ear-clipping algorithm, the complexity is n^3 and it does not handle holes."));
    m_filterList.insert("pclMeshTriangulation", filter);

    filter = new FilterDef(PclTools::pclOrganizedFastMesh, PclTools::pclOrganizedFastMeshParams, tr("creates a triangle based, polygonial mesh from an organized point cloud. The triangles are always spanned between neighboured points of the organized cloud."));
    m_filterList.insert("pclOrganizedFastMesh", filter);

    filter = new FilterDef(PclTools::pclSimplifyMesh, PclTools::pclSimplifyMeshParams, tr("Used SimplificationRemoveUnusedVertices from the PCL to simplify a pcl mesh."));
    m_filterList.insert("pclSimplifyMesh", filter);

    filter = new FilterDef(PclTools::pclPoisson, PclTools::pclPoissonParams, pclPoissonDOC);
    m_filterList.insert("pclSurfaceByPoisson", filter);

    filter = new FilterDef(PclTools::pclMarchingCubes, PclTools::pclMarchingCubesParams, pclMarchingCubesDOC);
    m_filterList.insert("pclSurfaceByMarchingCubes", filter);

    filter = new FilterDef(PclTools::pclSampleToDataObject, PclTools::pclSampleToDataObjectParams, tr("Uses copy an organized and dense pointcloud to an dataObject."));
    m_filterList.insert("pclSampleToDataObject", filter);

    filter = new FilterDef(PclTools::pclGetNormalsAtCogFromMesh, PclTools::pclGetNormalsAtCogFromMeshParams, pclGetNormalsAtCogFromMeshDOC);
    m_filterList.insert("pclGetNormalsAtCogFromMesh", filter);

    filter = new FilterDef(PclTools::pclTrimmedICP, PclTools::pclTrimmedICPParams, pclTrimmedICPDOC);
    m_filterList.insert("pclTrimmedICP", filter);

#if PCLHASSURFACENURBS
    filter = new FilterDef(PclTools::pclFitTrimmedBSpline, PclTools::pclFitTrimmedBSplineParams, pclFitTrimmedBSplineDOC);
    m_filterList.insert("pclFitTrimmedBSpline", filter);
#endif

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

// ---------------------------------------------------------------------- DO NOT ADD FILTER BELOW THIS!!! -----------------------------------------------------------
ito::RetVal PclTools::close(ItomSharedSemaphore *waitCond)
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

// ---------------------------------------------------------------------- DO NOT ADD FILTER BELOW THIS!!! -----------------------------------------------------------
