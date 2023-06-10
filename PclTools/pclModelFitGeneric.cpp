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

#include "pclTools.h"
#include "pluginVersion.h"
#define EIGEN_QT_SUPPORT

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"

#include "DataObject/dataObjectFuncs.h"

#include <pcl/pcl_config.h>

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
    #include <pcl/sample_consensus/method_types.h>
    #include <pcl/sample_consensus/model_types.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/segmentation/impl/sac_segmentation.hpp>
    /*#include <pcl/sample_consensus/impl/lmeds.hpp>
    #include <pcl/sample_consensus/impl/ransac.hpp>
    #include <pcl/sample_consensus/impl/rransac.hpp>
    #include <pcl/sample_consensus/impl/msac.hpp>
    #include <pcl/sample_consensus/impl/rmsac.hpp>
    #include <pcl/sample_consensus/impl/mlesac.hpp>
    #include <pcl/sample_consensus/impl/prosac.hpp>
    #include <pcl/sample_consensus/impl/sac_model_stick.hpp>
    #include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
    #include <pcl/sample_consensus/impl/sac_model_cone.hpp>
    #include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
    #include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
    #include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
    #include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
    #include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>*/
#endif

#include <qstring.h>
#include <qvariant.h>



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

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
    return ito::RetVal(ito::retError, 0, tr("pclFitCylinder not implemented for PCL 1.6.1 or lower").toLatin1().data());
#else
    double *angleLimits = NULL;
    double *radiusLimits = NULL;
    double *normalAxis = NULL;
    double orientationAngle = M_PI;

    double normalDistanceWeight = 0.0;
    int maxIterations = 0;
    double distanceThreshold = 0.0;
    bool optimizeCoefficients = false;
    double probability = 0.0;

    bool doGenericOutPut = false;

    bool useRadius = false;
    bool useAxis = false;
    bool useOpenAngle = false;

    switch(fitType)
    {


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
        default:
            retval += ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
            break;
        case pcl::SACMODEL_CONE:
            doGenericOutPut = false;
            useOpenAngle = true;

            angleLimits = mands[1].getVal<double*>();
            if(angleLimits == NULL || mands[1].getLen() != 2)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Angle limit must have 2 entries").toLatin1().data());
            }

#if CONE_LIMIT_NORMAL
            normalAxis = opts[0].getVal<double*>();
            if(normalAxis == NULL || opts[0].getLen() < 1)
            {
                useAxis = false;
            }
            else if(opts[0].getLen() < 3)
            {
                useAxis = false;
                retval += ito::RetVal(ito::retError, 0, tr("(normal-)axis vector must have at 3 entries").toLatin1().data());
            }
            else
            {
                useAxis = true;
            }
            orientationAngle = opts[1].getVal<double>();
            normalDistanceWeight = opts[2].getVal<double>();
            maxIterations = opts[3].getVal<int>();
            distanceThreshold = opts[4].getVal<double>();
            optimizeCoefficients = (opts[5].getVal<int>() > 0);
            probability = opts[6].getVal<double>();
#else
            useAxis = false;
            normalDistanceWeight = opts[0].getVal<double>();
            maxIterations = opts[1].getVal<int>();
            distanceThreshold = opts[2].getVal<double>();
            optimizeCoefficients = (opts[3].getVal<int>() > 0);
            probability = opts[4].getVal<double>();
#endif

            useRadius = false;
            break;
        case pcl::SACMODEL_PLANE:
        case pcl::SACMODEL_LINE:
            doGenericOutPut = false;

#if PLANE_LIMIT_NORMAL
            normalAxis = opts[0].getVal<double*>();
            if(normalAxis == NULL || opts[0].getLen() < 1)
            {
                useAxis = false;
            }
            else if(opts[0].getLen() < 3)
            {
                useAxis = false;
                retval += ito::RetVal(ito::retError, 0, tr("(normal-)axis vector must have at 3 entries").toLatin1().data());
            }
            else
            {
                useAxis = true;
            }
            orientationAngle = opts[1].getVal<double>();
            normalDistanceWeight = opts[2].getVal<double>();
            maxIterations = opts[3].getVal<int>();
            distanceThreshold = opts[4].getVal<double>();
            optimizeCoefficients = (opts[5].getVal<int>() > 0);
            probability = opts[6].getVal<double>();
#else
            useAxis = false;
            normalDistanceWeight = opts[0].getVal<double>();
            maxIterations = opts[1].getVal<int>();
            distanceThreshold = opts[2].getVal<double>();
            optimizeCoefficients = (opts[3].getVal<int>() > 0);
            probability = opts[4].getVal<double>();
#endif

            useRadius = false;
            break;
#if CIRCLE3D_LIMIT_NORMAL
        case pcl::SACMODEL_CIRCLE3D:
            doGenericOutPut = false;
            radiusLimits = mands[1].getVal<double*>();
            if(radiusLimits == NULL || mands[1].getLen() != 2)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Radius limit must have 2 entries").toLatin1().data());
            }

            normalAxis = opts[0].getVal<double*>();
            if(normalAxis == NULL || opts[0].getLen() < 1)
            {
                useAxis = false;
            }
            else if(opts[0].getLen() != 3)
            {
                useAxis = false;
                retval += ito::RetVal(ito::retError, 0, tr("(normal-)axis vector must have 3 entries").toLatin1().data());
            }
            else
            {
                useAxis = true;
            }
            orientationAngle = opts[1].getVal<double>();

            normalDistanceWeight = opts[2].getVal<double>();
            maxIterations = opts[3].getVal<int>();
            distanceThreshold = opts[4].getVal<double>();
            optimizeCoefficients = (opts[5].getVal<int>() > 0);
            probability = opts[6].getVal<double>();
            useRadius = true;
        break;
#else
        case pcl::SACMODEL_CIRCLE3D:
#endif // CIRCLE3D_LIMIT_NORMAL --> in theory circle 3D should accept a limit on the normal vector but this constrain has no effect!
        case pcl::SACMODEL_CIRCLE2D:
        case pcl::SACMODEL_CYLINDER:
        case pcl::SACMODEL_SPHERE:
            doGenericOutPut = false;
            radiusLimits = mands[1].getVal<double*>();
            if(radiusLimits == NULL || mands[1].getLen() != 2)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Radius limit must have 2 entries").toLatin1().data());
            }

            normalDistanceWeight = opts[0].getVal<double>();
            maxIterations = opts[1].getVal<int>();
            distanceThreshold = opts[2].getVal<double>();
            optimizeCoefficients = (opts[3].getVal<int>() > 0);
            probability = opts[4].getVal<double>();
            useRadius = true;
        break;

        case -1: //  This is for generic fitting
            doGenericOutPut = true;
            fitType = mands[1].getVal<int>();

            switch(fitType)
            {
                default:
                    retval += ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
                    break;

                case pcl::SACMODEL_PLANE:
                case pcl::SACMODEL_LINE:
#if PLANE_LIMIT_NORMAL
                    normalAxis = opts[1].getVal<double*>();
                    if(normalAxis == NULL || opts[1].getLen() < 1)
                    {
                        useAxis = false;
                    }
                    else if(opts[1].getLen() != 3)
                    {
                        useAxis = false;
                        retval += ito::RetVal(ito::retError, 0, tr("(normal-)axis vector must have 3 entries").toLatin1().data());
                    }
                    else
                    {
                        useAxis = true;
                    }
                    orientationAngle = opts[2].getVal<double>();
#else
                    useAxis = false;
#endif
                    normalDistanceWeight = opts[3].getVal<double>();
                    maxIterations = opts[4].getVal<int>();
                    distanceThreshold = opts[5].getVal<double>();
                    optimizeCoefficients = (opts[6].getVal<int>() > 0);
                    probability = opts[7].getVal<double>();
                    useRadius = false;
                    break;

                case pcl::SACMODEL_CIRCLE3D:
#if CIRCLE3D_LIMIT_NORMAL
                    normalAxis = opts[1].getVal<double*>();
                    if(normalAxis == NULL || opts[1].getLen() < 1)
                    {
                        useAxis = false;
                    }
                    else if(opts[1].getLen() != 3)
                    {
                        useAxis = false;
                        retval += ito::RetVal(ito::retError, 0, tr("(normal-)axis vector must have 3 entries").toLatin1().data());
                    }
                    else
                    {
                        useAxis = true;
                    }
                    orientationAngle = opts[2].getVal<double>();
#endif
                case pcl::SACMODEL_CIRCLE2D:
                case pcl::SACMODEL_SPHERE:
                case pcl::SACMODEL_CYLINDER:
                    radiusLimits = opts[0].getVal<double*>();
                    if(radiusLimits == NULL || opts[0].getLen() != 2)
                    {
                        retval += ito::RetVal(ito::retError, 0, tr("Radius limit must have 2 entries").toLatin1().data());
                    }

                    normalDistanceWeight = opts[3].getVal<double>();
                    maxIterations = opts[4].getVal<int>();
                    distanceThreshold = opts[5].getVal<double>();
                    optimizeCoefficients = (opts[6].getVal<int>() > 0);
                    probability = opts[7].getVal<double>();
                    useRadius = true;
                break;
            }

            break;

    }

    pcl::ModelCoefficients::Ptr fitCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr fitInliers (new pcl::PointIndices);

    if(!retval.containsError())
    {

        switch(pclIn->getType())
        {
        case ito::pclInvalid:
            retval += ito::RetVal(ito::retError, 0, tr("invalid point cloud type not allowed").toLatin1().data());
            break;
        case ito::pclXYZ: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
            {

                if(!checkFitWithOutNormals(fitType))
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Can not fit the supposed type to object without normals defined.").toLatin1().data());
                }
                else
                {
                    pcl::SACSegmentation<pcl::PointXYZ> seg;

                    // Create the segmentation object for cylinder segmentation and set all the parameters
                    seg.setOptimizeCoefficients (optimizeCoefficients);
                    seg.setModelType (fitType);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setMaxIterations (maxIterations);
                    seg.setDistanceThreshold (distanceThreshold);

                    if(useRadius)
                        seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));

                    if(useAxis)
                    {
                        seg.setEpsAngle(orientationAngle);
                        Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                        seg.setAxis(normalV);
                    }

                    seg.setInputCloud (pclIn->toPointXYZ());
                    seg.setProbability (probability);

                    // Obtain the cylinder inliers and coefficients
                    seg.segment (*fitInliers, *fitCoefficients);
                }
            }
            break;
        case ito::pclXYZI: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
            {

                if(!checkFitWithOutNormals(fitType))
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Can not fit the supposed type to object without normals defined.").toLatin1().data());
                }
                else
                {

                    pcl::SACSegmentation<pcl::PointXYZI> seg;

                    // Create the segmentation object for cylinder segmentation and set all the parameters
                    seg.setOptimizeCoefficients (optimizeCoefficients);
                    seg.setModelType (fitType);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setMaxIterations (maxIterations);
                    seg.setDistanceThreshold (distanceThreshold);

                    if(useRadius)
                        seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));

                    if(useAxis)
                    {
                        seg.setEpsAngle(orientationAngle);
                        Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                        seg.setAxis(normalV);
                    }

                    seg.setInputCloud (pclIn->toPointXYZI());
                    seg.setProbability (probability);
                    // Obtain the cylinder inliers and coefficients
                    seg.segment (*fitInliers, *fitCoefficients);
                }
            }
            break;
        case ito::pclXYZRGBA: //does not work, SACSegmentation do not support SACMODEL_CYLINDER
            {

                if(!checkFitWithOutNormals(fitType))
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Can not fit the supposed type to object without normals defined.").toLatin1().data());
                }
                else
                {

                    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

                    // Create the segmentation object for cylinder segmentation and set all the parameters

                    seg.setOptimizeCoefficients (optimizeCoefficients);
                    seg.setModelType (fitType);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setMaxIterations (maxIterations);
                    seg.setDistanceThreshold (distanceThreshold);

                    if(useRadius)
                        seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));

                    if(useAxis)
                    {
                        seg.setEpsAngle(orientationAngle);
                        Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                        seg.setAxis(normalV);
                    }

                    seg.setInputCloud (pclIn->toPointXYZRGBA());
                    seg.setProbability (probability);
                    // Obtain the cylinder inliers and coefficients
                    seg.segment (*fitInliers, *fitCoefficients);
                }
            }
            break;
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

                if(useRadius)
                    seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
                if(useAxis)
                {
                    seg.setEpsAngle(orientationAngle);
                    Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                    seg.setAxis(normalV);
                }
                if(useOpenAngle)
                {
                    seg.setMinMaxOpeningAngle(angleLimits[0], angleLimits[1]);
                }
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

                if(useRadius)
                    seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
                if(useAxis)
                {
                    seg.setEpsAngle(orientationAngle);
                    Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                    seg.setAxis(normalV);
                }
                if(useOpenAngle)
                {
                    seg.setMinMaxOpeningAngle(angleLimits[0], angleLimits[1]);
                }
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

                if(useRadius)
                    seg.setRadiusLimits (std::min(radiusLimits[0], radiusLimits[1]), std::max(radiusLimits[0], radiusLimits[1]));
                if(useAxis)
                {
                    seg.setEpsAngle(orientationAngle);
                    Eigen::Vector3f normalV(normalAxis[0], normalAxis[1], normalAxis[2]);
                    seg.setAxis(normalV);
                }
                if(useOpenAngle)
                {
                    seg.setMinMaxOpeningAngle(angleLimits[0], angleLimits[1]);
                }
                seg.setInputCloud (pclIn->toPointXYZRGBNormal());
                seg.setInputNormals (pclIn->toPointXYZRGBNormal());
                seg.setProbability (probability);

                // Obtain the cylinder inliers and coefficients
                seg.segment (*fitInliers, *fitCoefficients);

            }
            break;
        default:
            retval += ito::RetVal(ito::retError, 0, tr("point cloud must have normal vectors defined.").toLatin1().data());
            break;
        }
    }

    if (!retval.containsError() && fitInliers->indices.size() == 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("no model could be fit to given point cloud").toLatin1().data());
    }

    if(!retval.containsError())
    {
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
                paramsOut->data()[1].setVal<int>(fitInliers->indices.size());
                free(result);
                result = NULL;
            }

        }
        else
        {
            switch(fitType)
            {
                default:
                    retval += ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(fitType))).toLatin1().data());
                    break;
                case pcl::SACMODEL_PLANE:
                {
                    double vec[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };

                    paramsOut->data()[0].setVal<double*>(vec, 3);
                    paramsOut->data()[1].setVal<double>(fitCoefficients->values[3]);
                    paramsOut->data()[2].setVal<int>(fitInliers->indices.size());

                    break;
                }
                case pcl::SACMODEL_LINE:
                {
                    double points[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };
                    double vec[] = { fitCoefficients->values[3], fitCoefficients->values[4], fitCoefficients->values[5] };

                    paramsOut->data()[0].setVal<double*>(points, 3);
                    paramsOut->data()[1].setVal<double*>(vec, 3);
                    paramsOut->data()[2].setVal<int>(fitInliers->indices.size());

                    break;
                }
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
                case pcl::SACMODEL_CIRCLE2D:
                {
                    // sphereCoefficients are centerX, centerY, centerZ, radius

                    double points[] = { fitCoefficients->values[0], fitCoefficients->values[1]};

                    paramsOut->data()[0].setVal<double*>(points, 2); // Positions
                    paramsOut->data()[1].setVal<double>(fitCoefficients->values[2]); //radius
                    paramsOut->data()[2].setVal<int>(fitInliers->indices.size());
                    break;
                }
                case pcl::SACMODEL_CIRCLE3D:
                {
                    // sphereCoefficients are centerX, centerY, centerZ, radius

                    double points[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };
                    double normal[] = { fitCoefficients->values[4], fitCoefficients->values[5], fitCoefficients->values[6] };
                    paramsOut->data()[0].setVal<double*>(points, 3); // Positions
                    paramsOut->data()[1].setVal<double*>(normal, 3); // Normal
                    paramsOut->data()[2].setVal<double>(fitCoefficients->values[3]); //radius

                    paramsOut->data()[3].setVal<int>(fitInliers->indices.size());
                    break;
                }
                case pcl::SACMODEL_CONE:
                {
                    double points[] = { fitCoefficients->values[0], fitCoefficients->values[1], fitCoefficients->values[2] };
                    double vec[] = { fitCoefficients->values[3], fitCoefficients->values[4], fitCoefficients->values[5] };

                    paramsOut->data()[0].setVal<double*>(points, 3);
                    paramsOut->data()[1].setVal<double*>(vec, 3);
                    paramsOut->data()[2].setVal<double>(fitCoefficients->values[6]); //opening angle
                    paramsOut->data()[3].setVal<int>(fitInliers->indices.size());

                    break;
                }
            }
        }
    }
    return retval;
#endif
}
