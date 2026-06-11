/* ********************************************************************
    Plugin "PCLTools" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut für Technische Optik (ITO),
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

#include "pclTools.h"
#define EIGEN_QT_SUPPORT

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"
#include "DataObject/dataObjectFuncs.h"


#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>

//all other impl includes are in the pclModelFitGeneric...Impl.cpp files in order to prevent too big obj files during compilation
//#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
//#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
//#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
//#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
//#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
//#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
//#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
//#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
//#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclProjectOnModelDOC = QObject::tr("\n\
\n\
\n\
Possible types are: \n\
--------------------\n\
SACMODEL_PLANE = 0, \n\
SACMODEL_LINE = 1, \n\
SACMODEL_CIRCLE2D = 2, \n\
SACMODEL_CIRCLE3D = 3, \n\
SACMODEL_SPHERE = 4, \n\
SACMODEL_CYLINDER = 5, \n\
SACMODEL_CONE = 6, \n\
SACMODEL_TORUS = 7, \n\
SACMODEL_PARALLEL_LINE = 8, \n\
SACMODEL_PERPENDICULAR_PLANE = 9, \n\
SACMODEL_PARALLEL_LINES = 10, \n\
SACMODEL_NORMAL_PLANE = 11, \n\
SACMODEL_NORMAL_SPHERE = 12, \n\
SACMODEL_REGISTRATION = 13, \n\
SACMODEL_REGISTRATION_2D = 14, \n\
SACMODEL_PARALLEL_PLANE = 15, \n\
SACMODEL_NORMAL_PARALLEL_PLANE = 16, \n\
SACMODEL_STICK = 17 \n\
\n");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclProjectOnModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
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

    int modelType = mands[2].getVal<int>();

    Eigen::Vector4f linePt;
    linePt[3] = 0.0f;

    Eigen::Vector4f lineDir;
    lineDir[3] = 0.0f;

    float radius = cv::saturate_cast<float>(opts[2].getVal<double>());
    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    switch(modelType)
    {
        default:
        case pcl::SACMODEL_LINE:
        case pcl::SACMODEL_CIRCLE2D:
        case pcl::SACMODEL_CIRCLE3D:
        {
            return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(modelType))).toLatin1().data());
        }

        case pcl::SACMODEL_PLANE:
        {
            double* value = NULL;
            coefficients->values.resize (4);

            if (opts[1].getLen() < 3 || (value = (double*)(opts[1].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Plane model must have [nx,ny,nz] and d. [nx,ny,nz] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                coefficients->values[0] = cv::saturate_cast<float>(value[0]);
                coefficients->values[1] = cv::saturate_cast<float>(value[1]);
                coefficients->values[2] = cv::saturate_cast<float>(value[2]);
            }
            coefficients->values[3] = opts[2].getVal<double>();
        }
        break;

        case pcl::SACMODEL_SPHERE:
        {
            double* value = NULL;
            coefficients->values.resize (4);

            if (opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Spherical model must have [x,y,z] and r. [x,y,z] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                coefficients->values[0] = cv::saturate_cast<float>(value[0]);
                coefficients->values[1] = cv::saturate_cast<float>(value[1]);
                coefficients->values[2] = cv::saturate_cast<float>(value[2]);
            }
            coefficients->values[3] = opts[2].getVal<double>();
        }
        break;

        case pcl::SACMODEL_CYLINDER:
        {
            double* value = NULL;
            coefficients->values.resize (7);

            if (opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Cylinder model must have 7 parameters, [x,y,z], [dx, dy, dz] and r. [x,y,z] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                coefficients->values[0] = cv::saturate_cast<float>(value[0]);
                coefficients->values[1] = cv::saturate_cast<float>(value[1]);
                coefficients->values[2] = cv::saturate_cast<float>(value[2]);
            }

            if (opts[1].getLen() < 3 || (value = (double*)(opts[1].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Cylinder model must have [x,y,z], [dx, dy, dz] and r. [dx,dy,dz] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                coefficients->values[3] = cv::saturate_cast<float>(value[0]);
                coefficients->values[4] = cv::saturate_cast<float>(value[1]);
                coefficients->values[5] = cv::saturate_cast<float>(value[2]);
            }

            coefficients->values[6] = opts[2].getVal<double>();
        }
        break;

        case pcl::SACMODEL_CONE:
        case pcl::SACMODEL_TORUS:
        case pcl::SACMODEL_PARALLEL_LINE:
        case pcl::SACMODEL_PERPENDICULAR_PLANE:
        case pcl::SACMODEL_PARALLEL_LINES:
        case pcl::SACMODEL_NORMAL_PLANE:
        case pcl::SACMODEL_NORMAL_SPHERE:
        case pcl::SACMODEL_REGISTRATION:
        case pcl::SACMODEL_REGISTRATION_2D:
        case pcl::SACMODEL_PARALLEL_PLANE:
        case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
        case pcl::SACMODEL_STICK:
        {
            return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(modelType))).toLatin1().data());
        }
    }

    float floatNAN = std::numeric_limits<float>::quiet_NaN();

    bool overwriteInput = false; //real inplace was possible
//    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);

    switch(pclIn->getType())
    {
        default:
        case ito::pclInvalid:
            return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not defined or point cloud invalid").toLatin1().data());
        case ito::pclXYZ:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZ);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZ);
                pclOut->resize(pclIn->size());
            }
        }
        break;
        case ito::pclXYZI:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZI);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZI);
                pclOut->resize(pclIn->size());
            }
        }
        break;
        case ito::pclXYZRGBA:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZRGBA);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZRGBA);
                pclOut->resize(pclIn->size());
            }
        }
        break;
        case ito::pclXYZNormal:
        case ito::pclXYZINormal:
        case ito::pclXYZRGBNormal:
        {
            if (pclIn == pclOut)
            {
                //*pclOut = *pclIn;
                //Do nothing
            }
            else
            {
                *pclOut = ito::PCLPointCloud(*pclIn);
            }
        }
        break;
    }

    #if (USEOMP)
    #pragma omp parallel num_threads(nthreads)
    {
    #endif

    Eigen::Vector4f curPt;
    curPt[3] = 0.0f;

    switch(pclIn->getType())
    {
        default:
        case ito::pclInvalid:
            retval += ito::RetVal(ito::retError, 0, tr("invalid point cloud type or type not allowed").toLatin1().data());
        break;

        case ito::pclXYZ: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclSrc = pclIn->toPointXYZ();
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclDists = pclOut->toPointXYZ();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;

        case ito::pclXYZNormal:
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr pclSrc = pclIn->toPointXYZNormal();
            pcl::PointCloud<pcl::PointNormal>::Ptr pclDists = pclOut->toPointXYZNormal();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointNormal> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;

        case ito::pclXYZI:
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pclSrc = pclIn->toPointXYZI();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pclDists = pclOut->toPointXYZI();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZI> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;

        case ito::pclXYZRGBA:
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclSrc = pclIn->toPointXYZRGBA();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclDists = pclOut->toPointXYZRGBA();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;

        case ito::pclXYZINormal:
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclSrc = pclIn->toPointXYZINormal();
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclDists = pclOut->toPointXYZINormal();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZINormal> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;

        case ito::pclXYZRGBNormal:
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclSrc = pclIn->toPointXYZRGBNormal();
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclDists = pclOut->toPointXYZRGBNormal();

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZRGBNormal> proj;
            proj.setModelType(modelType);
            proj.setInputCloud(pclSrc);
            proj.setModelCoefficients(coefficients);
            proj.filter(*pclDists);
        }
        break;
    }

    #if (USEOMP)
    }
    #endif

    if (!retval.containsError() && overwriteInput)
    {
        (*pclIn) = (*pclOut); //here: pclOut is a new, temporary point cloud, pclIn is the given argument pclIn AND pclOut!
        delete pclOut;
    }

    return retval;
#endif
}
