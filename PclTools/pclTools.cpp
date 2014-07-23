/* ********************************************************************
    Plugin "PCLTools" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut für Technische Optik (ITO),
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "pclTools.h"
#include "pluginVersion.h"
#define EIGEN_QT_SUPPORT

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"

#include "xyzFormat.h"

#include "DataObject/dataObjectFuncs.h"

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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include "random_sample_corrected.h" //corrected version for errornous version of random_sample filter in pcl 1.6.0
#include <pcl/common/pca.h>

#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp>

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

int NTHREADS = 2;

//PCL_INSTANTIATE_RandomSampleCorrected(pcl::PointXYZ)
    //template class pcl::RandomSampleCorrected<T>;


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
    
    char docstring[] = \
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

    m_description = tr("Filters and methods for pointClouds and polygonMeshes");
    m_detaildescription = QObject::tr(docstring);
    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");    
    
    NTHREADS  = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
PclToolsInterface::~PclToolsInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
	Q_EXPORT_PLUGIN2(PclTools, PclToolsInterface)
#endif


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
PclTools::PclTools() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//ItomDoc_STRVAR(savePointCloud_doc, "saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii)");
const char* PclTools::savePointCloudDOC = "\n\
\n\
\n\
\n\
\n";

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
    QString filename = (*paramsMand)[1].getVal<char*>();
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

    std::string filename_ = filename.toStdString();

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
        retval += ito::RetVal::format(ito::retError, 0, tr("pointCloud could not be saved: %s").toLatin1().data(), exc.detailedMessage().data());
        ret = 1;
    }

    if (ret < 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("error while saving point cloud (internal error of method in point cloud library").toLatin1().data());
    }

    return retval;
}
//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::loadPointCloudDOC = "\n\
\n\
\n\
\n\
\n";

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

    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('pcd','ply','vtk','auto' [default, check suffix of filename])").toLatin1().data()));
    
    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    QString filename = (*paramsMand)[1].getVal<char*>();
    QString typeString = (*paramsOpt)[0].getVal<char*>();
    ito::PCLPointCloud *pointCloud = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    QString type;
//    bool binary_mode = true;
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
    std::string filename_ = filenameCanonical.toStdString();

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
            while (!file.atEnd() && (++i) < 10)
            {
                startContent += file.readLine();
            }

            file.close();

            if (startContent.contains("comment PCL generated") && startContent.contains("element vertex"))
            {
                ret = pcl::io::loadPLYFile(filename_, pc2);
            }
            else
            {
                retval += ito::RetVal::format(ito::retError,0,"file '%s' does not contain point cloud data.", filename_.data());
            }
        }        
#else
        retval += ito::RetVal(ito::retError, 0, tr("ply-support is not compiled in this version (since this is not supported in PCL1.5.1 or lower").toLatin1().data());
#endif
    }
    else if (type == "vtk")
    {
        retval += ito::RetVal(ito::retError, 0, tr("vtk file format cannot be loaded (not supported)").toLatin1().data());
    }
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
            const char* msg = exp.detailedMessage().data();
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
const char* PclTools::savePolygonMeshDOC = "\n\
\n\
\n\
\n\
\n";
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
    paramsMand->append(ito::Param("filename", ito::ParamBase::String, "", tr("complete filename (type is either read by suffix of filename or by parameter 'type'").toLatin1().data()));

    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('obj' [default],'ply','vtk','stl')").toLatin1().data()));

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::savePolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::PCLPolygonMesh *polygonMesh = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<char*>();  //Input object
    QString filename = (*paramsMand)[1].getVal<char*>();
    QString type = (*paramsOpt)[0].getVal<char*>();
//    bool binary_mode = true;
    int ret = 1;

    //check filename
    QFileInfo finfo(filename);
    if (type == "")
    {
        type = finfo.suffix().toLower();
        filename = finfo.absoluteFilePath();
    }

    std::string filename_ = filename.toStdString();

    //check type
    type = type.toLower();
    if (type != "obj" && type != "ply" && type != "vtk" && type != "stl")
    {
        retval += ito::RetVal(ito::retWarning, 0, tr("type was set to 'obj', since 'obj','ply', 'stl' or 'vtk' expected").toLatin1().data());
        type = "obj";
    }

    //check point cloud
    if (type == "obj")
    {
        ret = pcl::io::saveOBJFile(filename_, *(polygonMesh->polygonMesh()));
    }
    else if (type == "stl")
    {
        ret = pcl::io::savePolygonFileSTL(filename_, *(polygonMesh->polygonMesh()));
    }
    else if (type == "ply")
    {
        ret = pcl::io::savePolygonFilePLY(filename_, *(polygonMesh->polygonMesh()));
    }
    else if (type == "vtk")
    {
        ret = pcl::io::savePolygonFileVTK(filename_, *(polygonMesh->polygonMesh()));
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

    return retval;
}
//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::loadPolygonMeshDOC = "\n\
\n\
\n\
\n\
\n";
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
    paramsMand->append(ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", "complete filename (type is read by suffix"));
    
    paramsOpt->append(ito::Param("type", ito::ParamBase::String, "", tr("type ('obj','vtk','stl','ply', 'auto' [default, check suffix of filename])").toLatin1().data()));
    
    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PclTools::loadPolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    QString filename = (*paramsMand)[1].getVal<char*>();
    ito::PCLPolygonMesh* mesh = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<char*>();
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

    std::string filename_ = finfo.canonicalFilePath().toStdString();

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
            while (!file.atEnd() && (++i)<10)
            {
                startContent += file.readLine();
            }

            file.close();

            if (startContent.contains("vtkPolyData points and polygons"))
            {
                ret = pcl::io::loadPolygonFilePLY(filename_, *polyMesh);
            }
            else
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("file '%s' does not contain polygon mesh data.").toLatin1().data(), filename_.data());
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

////create mesh
//        pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> fastMesh;
//        pcl::PolygonMesh::Ptr polygonMeshPtr(new pcl::PolygonMesh());
//        fastMesh.setInputCloud(cloud);
//        fastMesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::TRIANGLE_RIGHT_CUT);
//        fastMesh.storeShadowedFaces(true);
//        fastMesh.reconstruct(*polygonMeshPtr);

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::transformAffineDOC = "\n\
\n\
\n\
\n\
\n";

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
    paramsMand->append(ito::Param("pointCloudOut", ito::ParamBase::PointCloudPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("The affine transform is applied to this point cloud").toLatin1().data()));
    paramsMand->append(ito::Param("transform", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("4x4 homogeneous transformation matrix").toLatin1().data()));

    paramsOpt->clear();
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::transformAffine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    //read params from mandatory and optional params
    
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)(*paramsMand)[0].getVal<void*>();
    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)(*paramsMand)[1].getVal<void*>();
    const ito::DataObject *transform = (const ito::DataObject*)(*paramsMand)[2].getVal<void*>();

    if (pclIn == NULL || pclOut == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    Eigen::Affine3f trafo;
    retval += ito::pclHelper::dataObj4x4ToEigenAffine3f(transform, trafo);

    if (pclIn != pclOut)
    {
        *pclOut = ito::PCLPointCloud(pclIn->getType());
    }
    
    if (!retval.containsError())
    {
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
/*static*/ ito::RetVal PclTools::pclFitModelGeneric(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, int fitType)
{
    ito::RetVal retval = ito::retOk;

    QVector<ito::ParamBase> mands = (*paramsMand);
    QVector<ito::ParamBase> opts = (*paramsOpt);
    QVector<ito::ParamBase> outs = (*paramsOut);

    //read params from mandatory and optional params
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)mands[0].getVal<void*>();
    if (pclIn == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
    }

    if (mands[1].getLen() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("radiusLimits must contain of 2 entries").toLatin1().data());
    }

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    return ito::RetVal(ito::retError, 0, tr("pclFitCylinder not implemented for PCL 1.6.1 or lower").toLatin1().data());
#else

    double *radiusLimits = NULL;
    double normalDistanceWeight = 0.0;
    int maxIterations = 0;
    double distanceThreshold = 0.0;
    bool optimizeCoefficients = false;
    double probability = 0.0;

    bool doGenericOutPut = false;

    switch(fitType)
    {
        default:
            return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
        case pcl::SACMODEL_CYLINDER:
        case pcl::SACMODEL_SPHERE:
            doGenericOutPut = false;
            radiusLimits = mands[1].getVal<double*>();
            normalDistanceWeight = opts[0].getVal<double>();
            maxIterations = opts[1].getVal<int>();
            distanceThreshold = opts[2].getVal<double>();
            optimizeCoefficients = (opts[3].getVal<int>() > 0);
            probability = opts[4].getVal<double>();
        break;

        case -1: //  This is for generic fitting
            doGenericOutPut = true;
            fitType = mands[1].getVal<int>();

            switch(fitType)
            {
                default:
                    return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
                case pcl::SACMODEL_CYLINDER:
                case pcl::SACMODEL_SPHERE:
                    radiusLimits = opts[0].getVal<double*>();
                    normalDistanceWeight = opts[1].getVal<double>();
                    maxIterations = opts[2].getVal<int>();
                    distanceThreshold = opts[3].getVal<double>();
                    optimizeCoefficients = (opts[4].getVal<int>() > 0);
                    probability = opts[5].getVal<double>();
                break;
            }

            break;

    }

    pcl::ModelCoefficients::Ptr fitCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr fitInliers (new pcl::PointIndices);

    switch(pclIn->getType())
    {
    case ito::pclInvalid:
        return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not allowed").toLatin1().data());
    //case ito::pclXYZ: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
    //    {
    //        pcl::SACSegmentation<pcl::PointXYZ> seg;

    //        // Create the segmentation object for cylinder segmentation and set all the parameters
    //        seg.setOptimizeCoefficients (optimizeCoefficients);
    //        seg.setModelType (pcl::SACMODEL_CYLINDER);
    //        seg.setMethodType (pcl::SAC_RANSAC);
    //        seg.setMaxIterations (maxIterations);
    //        seg.setDistanceThreshold (distanceThreshold);
    //        seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
    //        seg.setInputCloud (pclIn->toPointXYZ());

    //        // Obtain the cylinder inliers and coefficients
    //        seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //    }
    //    break;
    case ito::pclXYZNormal:
        {
            pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg(true); 

            // Create the segmentation object for cylinder / spherical segmentation and set all the parameters
            seg.setOptimizeCoefficients (optimizeCoefficients);
            seg.setModelType (fitType);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (normalDistanceWeight);
            seg.setMaxIterations (maxIterations);
            seg.setDistanceThreshold (distanceThreshold);
            seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
            seg.setInputCloud (pclIn->toPointXYZNormal());
            seg.setInputNormals (pclIn->toPointXYZNormal());
            seg.setProbability (probability);

            // Obtain the cylinder inliers and coefficients
            seg.segment (*fitInliers, *fitCoefficients);
        }
        break;
    case ito::pclXYZINormal:
        {
            pcl::SACSegmentationFromNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> seg(true); 

            // Create the segmentation object for cylinder / spherical segmentation and set all the parameters
            seg.setOptimizeCoefficients (optimizeCoefficients);
            seg.setModelType (fitType);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (normalDistanceWeight);
            seg.setMaxIterations (maxIterations);
            seg.setDistanceThreshold (distanceThreshold);
            seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
            seg.setInputCloud (pclIn->toPointXYZINormal());
            seg.setInputNormals (pclIn->toPointXYZINormal());
            seg.setProbability (probability);

            // Obtain the cylinder inliers and coefficients
            seg.segment (*fitInliers, *fitCoefficients);
        }
        break;
    case ito::pclXYZRGBNormal:
        {
            pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg(true); 

            // Create the segmentation object for cylinder / spherical segmentation and set all the parameters
            seg.setOptimizeCoefficients (optimizeCoefficients);
            seg.setModelType (fitType);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (normalDistanceWeight);
            seg.setMaxIterations (maxIterations);
            seg.setDistanceThreshold (distanceThreshold);
            seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
            seg.setInputCloud (pclIn->toPointXYZRGBNormal());
            seg.setInputNormals (pclIn->toPointXYZRGBNormal());
            seg.setProbability (probability);

            // Obtain the cylinder inliers and coefficients
            seg.segment (*fitInliers, *fitCoefficients);           
            
        }
        break;
    default:
        return ito::RetVal(ito::retError, 0, tr("point cloud must have normal vectors defined.").toLatin1().data());
    }

    if (fitInliers->indices.size() == 0)
    {
        return ito::RetVal(ito::retError, 0, tr("no model could be fit to given point cloud").toLatin1().data());
    }

    if(doGenericOutPut)
    {
        int numOfCoeffs = fitCoefficients->values.size();
        double *result = (double*)calloc(numOfCoeffs, sizeof(double));
        if(!result)
        {
            retval += ito::RetVal(ito::retError, 0, (tr("Could not alloced result vector").arg(QString::number(fitType))).toLatin1().data());
        }
        else
        {
            for(int i = 0; i < numOfCoeffs; i++)
            {
                result[i] = fitCoefficients->values[i];
            }
            paramsOut->data()[0].setVal<double*>(result, numOfCoeffs); // Positions

            free(result);
            result = NULL;
        }
        
    }
    else
    {
        switch(fitType)
        {
            default:
                return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
            case pcl::SACMODEL_CYLINDER:
            {
                double points[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };
                double vec[] = { fitCoefficients->values[3], fitCoefficients->values[4], fitCoefficients->values[5] };

                paramsOut->data()[0].setVal<double*>(points, 3);
                paramsOut->data()[1].setVal<double*>(vec, 3);
                paramsOut->data()[2].setVal<double>(fitCoefficients->values[6]); //radius
                paramsOut->data()[3].setVal<int>(fitInliers->indices.size());
            
                break;
            }
            case pcl::SACMODEL_SPHERE:
            {
                // sphereCoefficients are centerX, centerY, centerZ, radius

                double points[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };

                paramsOut->data()[0].setVal<double*>(points, 3); // Positions
                paramsOut->data()[1].setVal<double>(fitCoefficients->values[3]); //radius
                paramsOut->data()[2].setVal<int>(fitInliers->indices.size());
                break;
            }
        }    
    }
    return retval;
#endif  
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclFitModelDOC = "\n\
\n\
\n\
\n\
\n";
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsMand->append(ito::Param("modelType", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, pcl::SACMODEL_PLANE, tr("Model type according to enum pcl::SacModel").toLatin1().data()));
    paramsOpt->clear();
    paramsOpt->append(ito::Param("radiusLimits", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("radius limits [min, max]").toLatin1().data()));
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("coeffizientsModel", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("Vector with the model coeffizients according to model definition.").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, -1);
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclFitCylinderDOC = "\n\
\n\
\n\
\n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCylinderParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsMand->append(ito::Param("radiusLimits", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("radius limits [min, max]").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting point on axis of symmetrie of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting axis of symmetrie of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting fitted radius of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCylinder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_CYLINDER);
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclFitSphereDOC = "\n\
\n\
\n\
\n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitSphereParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsMand->append(ito::Param("radiusLimits", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("radius limits [min, max]").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting center point of spehre").toLatin1().data()));
    paramsOut->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting fitted radius of sphere").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitSphere(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_SPHERE);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclDistanceToModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsOpt->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::In, 0.0, (double)(std::numeric_limits<float>::max()), 0.0, tr("cylinder radius").toLatin1().data()));

    paramsOut->clear();

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclDistanceToModelDOC = "\n\
\n\
\n\
\n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ double PclTools::pointToLineDist(const float inPt[3], const float modelCoefficients[7])
{
    Eigen::Vector4f linePt  (modelCoefficients[0], modelCoefficients[1], modelCoefficients[2], 0);
    Eigen::Vector4f lineDir (modelCoefficients[3], modelCoefficients[4], modelCoefficients[5], 0);
    Eigen::Vector4f pt (inPt[0], inPt[1], inPt[2], 0);
    return sqrt(pcl::sqrPointToLineDistance (pt, linePt, lineDir));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclDistanceToModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    QVector<ito::ParamBase> mands = (*paramsMand);
    QVector<ito::ParamBase> opts = (*paramsOpt);
    QVector<ito::ParamBase> outs = (*paramsOut);

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    return ito::RetVal(ito::retError, 0, tr("pclDistanceToModel not implemented for PCL 1.6.1 or lower").toLatin1().data());
#else
    //read params from mandatory and optional params
    ito::PCLPointCloud *pclIn = (ito::PCLPointCloud*)mands[0].getVal<void*>();
    if (pclIn == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("input point cloud must not be NULL").toLatin1().data());
    }

    ito::PCLPointCloud *pclOut = (ito::PCLPointCloud*)mands[1].getVal<void*>();
    if (pclOut == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("output point cloud must not be NULL").toLatin1().data());
    }

    if(pclOut == pclIn)
    {
        if((pclIn->getType() != ito::pclXYZINormal) && (pclIn->getType() != ito::pclXYZNormal) && (pclIn->getType() != ito::pclXYZRGBNormal))
            return ito::RetVal(ito::retError, 0, tr("Inplace operation only supported for pclXYZNormal, pclXYZIRGBNormal or pclXYZINormal").toLatin1().data());
    
    }

    int modelType = mands[2].getVal<int>();
    int distanceType = 0;

    Eigen::Vector4f linePt;
    linePt[3] = 0.0f;

    Eigen::Vector4f lineDir;
    lineDir[3] = 0.0f;



    float radius = cv::saturate_cast<float>(opts[2].getVal<double>());

    switch(modelType)
    {
        default:
            return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(modelType))).toLatin1().data());
        case pcl::SACMODEL_CYLINDER:
        {
            distanceType = 1;
            double* value = NULL;

            if(opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Cylinder model must have 7 parameters, [x,y,z], [dx, dy, dz] and r. [x,y,z] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                linePt[0] = cv::saturate_cast<float>(value[0]);
                linePt[1] = cv::saturate_cast<float>(value[1]);
                linePt[2] = cv::saturate_cast<float>(value[2]);
            }

            if(opts[1].getLen() < 3 || (value = (double*)(opts[1].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Cylinder model must have [x,y,z], [dx, dy, dz] and r. [dx,dy,dz] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                lineDir[0] = cv::saturate_cast<float>(value[0]);
                lineDir[1] = cv::saturate_cast<float>(value[1]);
                lineDir[2] = cv::saturate_cast<float>(value[2]);
            }
        }
        break;
        case pcl::SACMODEL_SPHERE:
        {
            distanceType = 3;
            double* value = NULL;

            if(opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Spherical model must have [x,y,z] and r. [x,y,z] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                linePt[0] = cv::saturate_cast<float>(value[0]);
                linePt[1] = cv::saturate_cast<float>(value[1]);
                linePt[2] = cv::saturate_cast<float>(value[2]);
            }

        }
        break;
    }

    float floatNAN = std::numeric_limits<float>::quiet_NaN();

    //#if (USEOMP)
    //#pragma omp parallel num_threads(NTHREADS)
    //{
    //#endif  
    
    Eigen::Vector4f curPt;
    curPt[3] = 0.0f;

    switch(pclIn->getType())
    {
        case ito::pclInvalid:
            return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not allowed").toLatin1().data());
        case ito::pclXYZ: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclSrc = pclIn->toPointXYZ();
                
                *pclOut = ito::PCLPointCloud(ito::pclXYZNormal);
                pclOut->resize(pclSrc->size());

                pcl::PointCloud<pcl::PointNormal>::Ptr pclDists = pclOut->toPointXYZNormal();


                if(distanceType == 0)
                {

                    //#if (USEOMP)
                    //#pragma omp for schedule(guided)
                    //#endif     
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                    
                }
            }
            break;
        case ito::pclXYZNormal:
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr pclSrc = pclIn->toPointXYZNormal();
                
                *pclOut = *pclIn;
                pcl::PointCloud<pcl::PointNormal>::Ptr pclDists = pclOut->toPointXYZNormal();

                if(distanceType == 0)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {                       
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {   
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {    
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                    
                }
            }
            break;
        case ito::pclXYZI:
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pclSrc = pclIn->toPointXYZI();

                *pclOut = ito::PCLPointCloud(ito::pclXYZINormal);
                pclOut->resize(pclSrc->size());

                pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclDists = pclOut->toPointXYZINormal();

                if(distanceType == 0)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).intensity = pclSrc->at(np).intensity;
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).intensity = pclSrc->at(np).intensity;
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).intensity = pclSrc->at(np).intensity;
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).intensity = pclSrc->at(np).intensity;
                    }                    
                }
            }
            break;
        case ito::pclXYZRGBA:
            {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclSrc = pclIn->toPointXYZRGBA();

                *pclOut = ito::PCLPointCloud(ito::pclXYZRGBNormal);
                pclOut->resize(pclSrc->size());

                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclDists = pclOut->toPointXYZRGBNormal();

                if(distanceType == 0)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).rgba = pclSrc->at(np).rgba;
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).rgba = pclSrc->at(np).rgba;
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).rgba = pclSrc->at(np).rgba;
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {
                        memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                        memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                        pclDists->at(np).rgba = pclSrc->at(np).rgba;
                    }                    
                }
            }
            break;
        case ito::pclXYZINormal:
            {
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclSrc = pclIn->toPointXYZINormal();

                *pclOut = *pclIn;
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclDists = pclOut->toPointXYZINormal();

                if(distanceType == 0)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                    
                }
            }
            break;
        case ito::pclXYZRGBNormal:
            {
                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclSrc = pclIn->toPointXYZRGBNormal();
                
                *pclOut = *pclIn;

                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclDists = pclOut->toPointXYZRGBNormal();
                
                if(distanceType == 0)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                
                }
                else if(distanceType == 1)
                {
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 2)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                     
                }
                else if(distanceType == 3)
                {
                    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);
                    for (int np = 0; np < pclOut->size(); np++)
                    {     
                        if(ito::dObjHelper::isFinite<float>(pclDists->at(np).z))
                        {
                            pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center) - radius;
                        }
                        else
                        {
                            pclDists->at(np).curvature = floatNAN;
                        }
                    }                    
                }
            }
            break;
        default:
            return ito::RetVal(ito::retError, 0, tr("point cloud must have normal vectors defined.").toLatin1().data());
    }

    //#if (USEOMP)
    //}
    //#endif

    return retval;
#endif  
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclEstimateNormalsDOC = "\n\
\n\
\n\
\n\
\n";

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

    if (pclIn == NULL || pclOut == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("point cloud must not be NULL").toLatin1().data());
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
                *pclOut = ito::PCLPointCloud(ito::pclXYZNormal);

                pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

                ne.setSearchMethod(tree);
                ne.setInputCloud(pclIn->toPointXYZ());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);

                pcl::concatenateFields(*(pclIn->toPointXYZ()), *normals, *(pclOut->toPointXYZNormal()));
            }
            break;
        case ito::pclXYZI:
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZINormal);

                pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());

                ne.setSearchMethod(tree);
                ne.setInputCloud(pclIn->toPointXYZI());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);

                pcl::concatenateFields(*(pclIn->toPointXYZI()), *normals, *(pclOut->toPointXYZINormal()));
            }
            break;
        case ito::pclXYZRGBA:
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZRGBNormal);

                pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne(nr_threads);
                pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

                ne.setSearchMethod(tree);
                ne.setInputCloud(pclIn->toPointXYZRGBA());
                if (kSearch > 0)
                {
                    ne.setKSearch(kSearch);
                }
                if (viewPoint)
                {
                    ne.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
                }
                ne.compute(*normals);

                //the alpha value of rgba will be omitted
                pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
                pcl::copyPointCloud(*(pclIn->toPointXYZRGBA()), rgbCloud);
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

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclRemoveNaNDOC = "\n\
\n\
\n\
\n\
\n";

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
        //pcl::removeNaNFromPointCloud<pcl::PointXYZI>(*(pclIn->toPointXYZI()), *(pclOut->toPointXYZI()), indices);
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
        //pcl::removeNaNFromPointCloud<pcl::PointXYZRGBA>(*(pclIn->toPointXYZRGBA()), *(pclOut->toPointXYZRGBA()), indices);
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
        //pcl::removeNaNFromPointCloud<pcl::PointNormal>(*(pclIn->toPointXYZNormal()), *(pclOut->toPointXYZNormal()), indices);
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
        //pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*(pclIn->toPointXYZINormal()), *(pclOut->toPointXYZINormal()), indices);
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
        //pcl::removeNaNFromPointCloud<pcl::PointXYZRGBNormal>(*(pclIn->toPointXYZRGBNormal()), *(pclOut->toPointXYZRGBNormal()), indices);
        break;
    }

    return ito::retOk;
}

//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclPassThroughDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclVoxelGridDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclStatisticalOutlierRemovalDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclRandomSampleDOC = "\n\
\n\
\n\
\n\
\n";

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
    paramsMand->append(ito::Param("nrOfPoints", ito::ParamBase::Int | ito::ParamBase::In, 1, 10000000, 10000, tr("number of randomly picked points").toLatin1().data()));
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclRandomSample(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
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
    
    int nrOfPoints = (*paramsMand)[2].getVal<int>();

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
            randsample.setSeed(0.0);
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
            randsample.setSeed(0.0);
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
            randsample.setSeed(0.0);
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
            randsample.setSeed(0.0);
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
            randsample.setSeed(0.0);
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
            randsample.setSeed(0.0);
            randsample.filter(*(pclOut->toPointXYZRGBNormal()));
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
const char* PclTools::pclGetMinMax3DDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclGetPercentageThresholdDOC = "\n\
\n\
\n\
\n\
\n";

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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
                    if (pcl_isfinite(cvData[idx])) idx++;
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
        cv::sort(values_valid,values_valid,CV_SORT_EVERY_ROW | CV_SORT_ASCENDING);

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
const char* PclTools::pclGetHistogramDOC = "\n\
\n\
\n\
\n\
\n";

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
                    if (pcl_isfinite(val))
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
                    if (pcl_isfinite(val))
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
                    if (pcl_isfinite(val))
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
                    if (pcl_isfinite(val))
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
                    if (pcl_isfinite(val))
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
                    if (pcl_isfinite(val))
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
const char* PclTools::pclCylinderClipper3DDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclPCADOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclPolygonMeshFromIndicesDOC = "\n\
\n\
\n\
\n\
\n";

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

    std::vector<pcl::uint8_t> &data = mOut->cloud.data;
    pcl::uint8_t *ptrIn = &(mIn->cloud.data[0]);

    data.resize(counter * pointStep);

    if (idxSize > 0)
    {
        pcl::uint8_t *ptrOut = &(data[0]);

        for (int i = 0; i < points; i++)
        {
            if (buckets[i] > 0)
            {
                memcpy(ptrOut + pointStep * lut[i] * sizeof(pcl::uint8_t), ptrIn + pointStep * i * sizeof(pcl::uint8_t), pointStep * sizeof(pcl::uint8_t));
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
const char* PclTools::pclMeshTriangulationDOC = "\n\
\n\
\n\
\n\
\n";

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
const char* PclTools::pclSampleToDataObjectDOC = "\n\
\n\
\n\
\n\
\n";

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclSampleToDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::PCLPolygonMesh *meshIn = (ito::PCLPolygonMesh*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *disperityMap = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    ito::DataObject *intensityMap = (ito::DataObject*)(*paramsOpt)[0].getVal<void*>();

    if (meshIn == NULL || meshIn->polygonMesh().get() == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("point cloud pointer or content must not be NULL").toLatin1().data());
    }

    if (disperityMap == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("disperity output must not be NULL").toLatin1().data());
    }

    if (intensityMap == disperityMap)
    {
        return ito::RetVal(ito::retError, 0, tr("intensity output dataObject must differ from disperity dataObject").toLatin1().data());
    }

    return ito::retOk;
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
    paramsMand->append(ito::Param("meshIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Valid point cloud of type XYZ or XYZI").toLatin1().data()));
    paramsMand->append(ito::Param("disperityMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Outpot dataObject with z-Values").toLatin1().data()));
    paramsOpt->append( ito::Param("intensityMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Outpot dataObject with intensity-Values").toLatin1().data()));
    return retval;
}


//------------------------------------------------------------------------------------------------------------------------------
const char* PclTools::pclOrganizedFastMeshDOC = "\n\
\n\
\n\
\n\
\n";

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
		return ito::RetVal(ito::retError, 0, "with a PCL < 1.7.0 trianglePixelSizeRows and trianglePixelSizeColumns must be 1");
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
        return ito::RetVal(ito::retError, 0, "wrong triangulationType parameter.");
    }

    if (polygonMesh->valid() == false)
    {
        *polygonMesh = ito::PCLPolygonMesh(pcl::PolygonMesh::Ptr(new pcl::PolygonMesh()));
    }

    

    if (pointCloud == NULL || polygonMesh == NULL)
    {
        return ito::RetVal(ito::retError, 0, "the parameters organizedCloud and meshOut must not be NULL");
    }

    if (pointCloud->getType() != ito::pclInvalid && pointCloud->isOrganized() == false)
    {
        return ito::RetVal(ito::retError, 0, "the given point cloud must be organized. The height property of an organized point cloud is bigger than one.");
    }

    switch(pointCloud->getType())
    {
    case ito::pclInvalid:
        retval += ito::RetVal(ito::retError, 0, "a valid organized point cloud must be given");
        break;
    case ito::pclXYZ: //pcl::PointXYZ, toPointXYZ
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZ> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZConst());
            fastMesh.setTriangulationType( type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }
        break;
    case ito::pclXYZI: //pcl::PointXYZI, toPointXYZI
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZI> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZIConst());
            fastMesh.setTriangulationType( (pcl::OrganizedFastMesh<pcl::PointXYZI>::TriangulationType)type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }
        break;
    case ito::pclXYZRGBA: //pcl::PointXYZRGBA, toPointXYZRGBA
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZRGBA> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZRGBAConst());
            fastMesh.setTriangulationType( (pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TriangulationType)type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }
        break;
    case ito::pclXYZNormal: //pcl::PointNormal, toPointXYZNormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointNormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZNormalConst());
            fastMesh.setTriangulationType( (pcl::OrganizedFastMesh<pcl::PointNormal>::TriangulationType)type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }
        break;
    case ito::pclXYZINormal: //pcl::PointXYZINormal, toPointXYZINormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZINormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZINormalConst());
            fastMesh.setTriangulationType( (pcl::OrganizedFastMesh<pcl::PointXYZINormal>::TriangulationType)type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
 #if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }
        break;
    case ito::pclXYZRGBNormal: //pcl::PointXYZRGBNormal, toPointXYZRGBNormal
        {
            //create mesh
            pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> fastMesh;
            fastMesh.setInputCloud(pointCloud->toPointXYZRGBNormalConst());
            fastMesh.setTriangulationType( (pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal>::TriangulationType)type );
            fastMesh.storeShadowedFaces(storeShadowFaces);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
            fastMesh.setTrianglePixelSizeRows(psRows);
            fastMesh.setTrianglePixelSizeColumns(psCols);
#endif
            fastMesh.reconstruct( *(polygonMesh->polygonMesh()) );
        }

        break;
    }

    return retval;
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
    filter = new FilterDef(PclTools::savePointCloud, PclTools::savePointCloudParams, tr("saves pointCloud to hard drive (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii), xyz(ascii)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWritePointCloud, tr("Point Cloud (*.pcd *.ply *.vtk *.xyz)"));
    m_filterList.insert("savePointCloud", filter);

    filter = new FilterDef(PclTools::loadPointCloud, PclTools::loadPointCloudParams, tr("loads pointCloud from hard drive and returns it (format pcd(binary or ascii), ply(binary or ascii), vtk(ascii)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadPointCloud, tr("Point Cloud (*.pcd *.ply *.vtk *.xyz)"));
    m_filterList.insert("loadPointCloud", filter);

    filter = new FilterDef(PclTools::savePolygonMesh, PclTools::savePolygonMeshParams, tr("saves polygonMesh to hard drive (format obj[default], ply, vtk, stl)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWritePolygonMesh, tr("Polygon Mesh (*.obj *.ply *.vtk *.stl)"));
    m_filterList.insert("savePolygonMesh", filter);

    filter = new FilterDef(PclTools::loadPolygonMesh, PclTools::loadPolygonMeshParams, tr("loads polygonMesh from hard drive and returns it (format obj[default], ply, vtk, stl)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadPolygonMesh, tr("Polygon Mesh (*.obj *.ply *.vtk *.stl)"));
    m_filterList.insert("loadPolygonMesh", filter);

    filter = new FilterDef(PclTools::transformAffine, PclTools::transformAffineParams, tr("transforms a point cloud with a given homogeneous transformation matrix (4x4 data object)"));
    m_filterList.insert("pclTransformAffine", filter);

    filter = new FilterDef(PclTools::pclFitModel, PclTools::pclFitModelParams, tr("fits a model of type pcl::SAC_MODEL to the given input point cloud using a RANSAC based fit (must have normals defined).\nInternally wrapped to pclFitModelGeneric.\nSee http://docs.pointclouds.org/1.7.0/group__sample__consensus.html for detailes"));
    m_filterList.insert("pclFitModel", filter);

    filter = new FilterDef(PclTools::pclFitCylinder, PclTools::pclFitCylinderParams, tr("fits a cylindrical model to the given input point cloud using a RANSAC based fit (must have normals defined). Internally wrapped to pclFitModelGeneric but with adapted output."));
    m_filterList.insert("pclFitCylinder", filter);

    filter = new FilterDef(PclTools::pclFitSphere, PclTools::pclFitSphereParams, tr("fits a spherical model to the given input point cloud using a RANSAC based fit (must have normals defined). Internally wrapped to pclFitModelGeneric but with adapted output."));
    m_filterList.insert("pclFitSphere", filter);

    filter = new FilterDef(PclTools::pclDistanceToModel, PclTools::pclDistanceToModelParams, tr("Calculates the distances of points of a point cloud to a given model."));
    m_filterList.insert("pclDistanceToModel", filter);

    filter = new FilterDef(PclTools::pclEstimateNormals, PclTools::pclEstimateNormalsParams, tr("estimates normal vectors to the given input point cloud and returns the normal-enhanced representation of the input point cloud"));
    m_filterList.insert("pclEstimateNormals", filter);
    
    filter = new FilterDef(PclTools::pclRemoveNaN, PclTools::pclRemoveNaNParams, tr("removes NaN values from input point cloud (input and output can be the same)."));
    m_filterList.insert("pclRemoveNaN", filter);

    filter = new FilterDef(PclTools::pclPassThrough, PclTools::pclPassThroughParams, tr("filters a point cloud by giving boundary values to a specific dimension (outside or inside of this field)."));
    m_filterList.insert("pclPassThrough", filter);

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