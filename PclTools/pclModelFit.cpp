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
#define EIGEN_QT_SUPPORT

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "common/numeric.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"

#include "DataObject/dataObjectFuncs.h"

#include <pcl/pcl_config.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/distances.h>

#include <qstring.h>
#include <qvariant.h>

//------------------------------------------------------------------------------------------------------------------------------
bool PclTools::checkFitWithOutNormals(const int &fitObj)
{
    switch(fitObj)
    {
        case pcl::SACMODEL_PLANE:
        case pcl::SACMODEL_LINE:
        //case pcl::SACMODEL_STICK:
        case pcl::SACMODEL_CIRCLE2D:
        case pcl::SACMODEL_CIRCLE3D:
        case pcl::SACMODEL_SPHERE:
        //case pcl::SACMODEL_PARALLEL_LINE:
        //case pcl::SACMODEL_PERPENDICULAR_PLANE:
        //case pcl::SACMODEL_PARALLEL_PLANE:
            return true;
        default:
            return false;
    }
}

//------------------------------------------------------------------------------------------------------------------------------
bool PclTools::checkFitNormals(const int &fitObj)
{
    switch(fitObj)
    {
        case pcl::SACMODEL_PLANE:
        case pcl::SACMODEL_LINE:
        //case pcl::SACMODEL_STICK:
        case pcl::SACMODEL_CIRCLE2D:
        case pcl::SACMODEL_CIRCLE3D:
        case pcl::SACMODEL_SPHERE:
        case pcl::SACMODEL_CYLINDER:
        case pcl::SACMODEL_CONE:
        //case pcl::SACMODEL_PARALLEL_LINE:
        //case pcl::SACMODEL_PERPENDICULAR_PLANE:
        //case pcl::SACMODEL_PARALLEL_PLANE:
            return true;
        default:
            return false;
    }
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitModelDOC = QObject::tr("fits a geometric model to the given input point cloud using a RANSAC based approach. \n\
\n\
The method used for this fit is from the sample consensus module of point cloud library. \n\
(See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html). \n\
\n\
The following models are available: \n\
\n\
Plane (0). Hessian Normal Form (n_vec * pt + d = 0). Coefficients: \n\
\n\
* n_x \n\
* n_y \n\
* n_z \n\
* d \n\
\n\
Line (1). Output is the line vector (v) and one point (p) on the line. Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
\n\
Circle 2D (2). Output is the center point (p) of the circle and its radius (r) - (Fit in X, Y direction only). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* r \n\
\n\
Circle 3D (3). Output is the normal vector (v), the center point (p) of the circle and the circle radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* r \n\
* v_x \n\
* v_y \n\
* v_z \n\
\n\
Sphere (4). Output is the center point (p) and the radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* r \n\
\n\
Cylinder (5)*. Output is the orientation vector (v), one point (p) on the line and the cylinder radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
* r \n\
\n\
Cone (6)*. Output is the orientation vector (v), the tip point (p) and the opening angle in rad. Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
* angle \n\
\n\
Models with * need an input cloud where normal vectors are defined.");

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
    double limits[] = { 0.0, std::numeric_limits<double>::max()};
    paramsOpt->last().setVal<double*>(limits, 2);

    paramsOpt->append(ito::Param("axis", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("(normal-)axis to fit to [x, y, z]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxAngle", ito::ParamBase::Double | ito::ParamBase::In, 0.0, M_PI, M_PI, tr("maximum divergence between (normal-)axis and model orientation in radiant").toLatin1().data()));
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: A nonlinear optimization over all 7 parameters is applied (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("coefficientsModel", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("Vector with the model coefficients according to model definition.").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, -1);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitModelDObjDOC = QObject::tr("fits a geometric model to the given input data object using a RANSAC based approach. \n\
\n\
The input data object is transformed to a point cloud where the values are the Z coordinates, the X and Y coordinates are \n\
calculated using a meshgrid based on the axis scales and offsets. \n\
\n\
The method used for this fit is from the sample consensus module of point cloud library. \n\
(See http://docs.pointclouds.org/1.7.0/group__sample__consensus.html). \n\
\n\
The following models are available: \n\
\n\
Plane (0). Hessian Normal Form (n_vec * pt + d = 0). Coefficients: \n\
\n\
* n_x \n\
* n_y \n\
* n_z \n\
* d \n\
\n\
Line (1). Output is the line vector (v) and one point (p) on the line. Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
\n\
Circle 2D (2). Output is the center point (p) of the circle and its radius (r) - (Fit in X, Y direction only). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* r \n\
\n\
Circle 3D (3). Output is the normal vector (v), the center point (p) of the circle and the circle radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* r \n\
* v_x \n\
* v_y \n\
* v_z \n\
\n\
Sphere (4). Output is the center point (p) and the radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* r \n\
\n\
Cylinder (5). Output is the orientation vector (v), one point (p) on the line and the cylinder radius (r). Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
* r \n\
\n\
Cone (6). Output is the orientation vector (v), the tip point (p) and the opening angle in rad. Coefficients: \n\
\n\
* p_x \n\
* p_y \n\
* p_z \n\
* v_x \n\
* v_y \n\
* v_z \n\
* angle");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitModelDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = pclFitModelParams(paramsMand,paramsOpt,paramsOut);
/*
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut);
*/
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->first() = ito::Param("dataObjIn", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input data object").toLatin1().data());
    paramsMand->append(ito::Param("randomSamples", ito::ParamBase::Int | ito::ParamBase::In, 256, 65356, 65356, tr("Number of random samples. If this number is in the range [256,65355], randomly selected values from the data object are taken into account for the fit, else all values are used for the ransac fit").toLatin1().data()));

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitModelDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    if (ito::ITOM_API_FUNCS == NULL || apiFilterCall == NULL || apiFilterParam == NULL || apiFilterParamBase == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("The api-functions are not defined. Init of filter failed").toLatin1().data());
    }

    QVector<ito::ParamBase> tmpParamsMand;
    tmpParamsMand.clear();

    ito::RetVal retval = ito::retOk;

    ito::DataObject *inputObjct = (ito::DataObject*)((*paramsMand)[0].getVal<void*>());
    if (inputObjct == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Input DataObject must not be NULL").toLatin1().data());
    }

    ito::PCLPointCloud tmpCloud;

    ito::PCLPointCloud *tmpCloudFilter = NULL;

    bool deleteOnClose = false;

    try
    {
        ito::pclHelper::pointCloudFromDisparity(inputObjct, tmpCloud, true);
    }
    catch(std::bad_alloc exc)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not allocated new pointcloud").toLatin1().data());
    }

    if ((*paramsMand)[1].getVal<int>() != pcl::SACMODEL_SPHERE)
    {
        QVector<ito::ParamBase> estNormMand;
        QVector<ito::ParamBase> estNormOpt;
        QVector<ito::ParamBase> estNormOut;

        retval += apiFilterParamBase("pclEstimateNormals", &estNormMand, &estNormOpt, &estNormOut);
        if (!retval.containsError())
        {
            estNormMand[0].setVal<void*>(&tmpCloud);
            estNormMand[1].setVal<void*>(&tmpCloud);
            retval += pclEstimateNormals(&estNormMand, &estNormOpt, &estNormOut);
        }
    }

    int nrOfPoints = (*paramsMand)[2].getVal<int>();

    if (!retval.containsError())
    {
        if (nrOfPoints < 65356)
        {
            tmpCloudFilter = new ito::PCLPointCloud();

            QVector<ito::ParamBase> estRndMand;
            QVector<ito::ParamBase> estRndOpt;
            QVector<ito::ParamBase> estRndOut;

            retval += apiFilterParamBase("pclRandomSample", &estRndMand, &estRndOpt, &estRndOut);
            if (!retval.containsError())
            {
                estRndMand[0].setVal<void*>(&tmpCloud);
                estRndMand[1].setVal<void*>(tmpCloudFilter);
                estRndMand[2].setVal<int>(nrOfPoints);
                retval += pclRandomSample(&estRndMand, &estRndOpt, &estRndOut);
            }
            deleteOnClose = true;

        }
        else
        {
            tmpCloudFilter = &tmpCloud;
        }
    }

    if (!retval.containsError())
    {
        tmpParamsMand.append(ito::ParamBase("pointCloudIn", ito::ParamBase::PointCloudPtr));
        tmpParamsMand[0].setVal<void*>(tmpCloudFilter);
        tmpParamsMand.append(ito::ParamBase("modelType", ito::ParamBase::Int, (*paramsMand)[1].getVal<int>()));
        retval += pclFitModelGeneric(&tmpParamsMand, paramsOpt, paramsOut, -1);
    }

    if (tmpCloudFilter && deleteOnClose) delete tmpCloudFilter;

    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitCylinderDOC = QObject::tr("fits a cylindrical model to the given input point cloud using a RANSAC based approach (must have normals defined).");

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
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("point on axis of symmetry of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("axis of symmetry of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("fitted radius of cylinder").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCylinder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_CYLINDER);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitSphereDOC = QObject::tr("fits a spherical model to the given input point cloud using a RANSAC based approach");

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
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over al 7 parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
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

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitCircle2DDOC = QObject::tr("fits a planar circle model to the given input point cloud using a RANSAC based approach");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCircle2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting center point (xy) of circle").toLatin1().data()));
    paramsOut->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting fitted radius of circle").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCircle2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_CIRCLE2D);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitCircle3DDOC = QObject::tr("fits a 3D-circle model to the given input point cloud using a RANSAC based approach");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCircle3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
#if CIRCLE3D_LIMIT_NORMAL
    paramsOpt->append(ito::Param("axis", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("(normal-)axis to fit to [x, y, z]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxAngle", ito::ParamBase::Double | ito::ParamBase::In, 0.0, M_PI, M_PI, tr("maximum divergence between (normal-)axis and model oriantation in radiant").toLatin1().data()));
#endif
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting center point of the circle").toLatin1().data()));
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting normal vector").toLatin1().data()));
    paramsOut->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting fitted radius of the circle").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCircle3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_CIRCLE3D);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitPlaneDOC = QObject::tr("fits a plane model to the given input point cloud using a RANSAC based approach");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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

    paramsOpt->clear();
#if PLANE_LIMIT_NORMAL
    paramsOpt->append(ito::Param("axis", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("(normal-)axis to fit to [x, y, z]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxAngle", ito::ParamBase::Double | ito::ParamBase::In, 0.0, M_PI, M_PI, tr("maximum divergence between (normal-)axis and model oriantation in radiant").toLatin1().data()));
#endif
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting normal vector").toLatin1().data()));
    paramsOut->append(ito::Param("value", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting last value of Hesse Form").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_PLANE);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitLineDOC = QObject::tr("fits a line model to the given input point cloud using a RANSAC based approach");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitLineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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

    paramsOpt->clear();
#if PLANE_LIMIT_NORMAL
    paramsOpt->append(ito::Param("axis", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("axis to fit to [x, y, z]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxAngle", ito::ParamBase::Double | ito::ParamBase::In, 0.0, M_PI, M_PI, tr("maximum divergence between (normal-)axis and model oriantation in radiant").toLatin1().data()));
#endif
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting point on the line").toLatin1().data()));
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting oriantation vector").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitLine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_LINE);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitConeDOC = QObject::tr("fits a conical model to the given input point cloud using a RANSAC based approach (must have normals defined)");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitConeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
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
    paramsMand->append(ito::Param("angularLimits", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("opening angle limits in radiant [min, max]").toLatin1().data()));

    paramsOpt->clear();
#if CONE_LIMIT_NORMAL
    paramsOpt->append(ito::Param("axis", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("axis to fit to [x, y, z]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxAngle", ito::ParamBase::Double | ito::ParamBase::In, 0.0, M_PI, M_PI, tr("maximum divergence between (normal-)axis and model oriantation in radiant").toLatin1().data()));
#endif
    paramsOpt->append(ito::Param("normalDistanceWeight", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.1, tr("Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal [default: 0.1]").toLatin1().data()));
    paramsOpt->append(ito::Param("maxIterations", ito::ParamBase::Int | ito::ParamBase::In, 1, 1000000, 10000, tr("maximum number of RANSAC iterations [default: 10000]").toLatin1().data()));
    paramsOpt->append(ito::Param("distanceThreshold", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1000000.0, 0.05, tr("distanceThreshold of pcl [default: 0.05]").toLatin1().data()));
    paramsOpt->append(ito::Param("optimizeParameters", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 1: nonlinear optimization over all parameters is run (Careful: radius may exceed the given boundaries and then the resulting, considered indices become empty.)").toLatin1().data()));
    paramsOpt->append(ito::Param("probability", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.99, tr("the probability of choosing at least one sample free from outliers. [default: 0.99]").toLatin1().data()));

    paramsOut->clear();
    paramsOut->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting point on the line").toLatin1().data()));
    paramsOut->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::Out, NULL, tr("resulting oriantation vector").toLatin1().data()));
    paramsOut->append(ito::Param("openingAgle", ito::ParamBase::Double | ito::ParamBase::Out, NULL, tr("resulting opening angle in radiant").toLatin1().data()));
    paramsOut->append(ito::Param("inliers", ito::ParamBase::Int | ito::ParamBase::Out, NULL, tr("number of points considered after filtering outliers (due to RANSAC principle)").toLatin1().data()));

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitCone(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return pclFitModelGeneric(paramsMand, paramsOpt, paramsOut, pcl::SACMODEL_CONE);
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclDistanceToModelDOC = QObject::tr("Calculates the distances of points of a point cloud to a given model. \n\
\n\
**Possible types are:** \n\
\n\
SACMODEL_SPHERE = 4, \n\
SACMODEL_CYLINDER = 5, \n\
\n\
**Not supported yet:** \n\
\n\
SACMODEL_PLANE = 0, \n\
SACMODEL_LINE = 1, \n\
SACMODEL_CIRCLE2D = 2, \n\
SACMODEL_CIRCLE3D = 3, \n\
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
    paramsMand->append(ito::Param("modelType", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Model type according to enum pcl::SacModel (sphere: 4, cylinder: 5)").toLatin1().data()));

    paramsOpt->clear();
    paramsOpt->append(ito::Param("point", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("point on cylinder symmetrie axis").toLatin1().data()));
    paramsOpt->append(ito::Param("orientationVector", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("symmetrie axis of cylinder").toLatin1().data()));
    paramsOpt->append(ito::Param("radius", ito::ParamBase::Double | ito::ParamBase::In, 0.0, (double)(std::numeric_limits<float>::max()), 0.0, tr("cylinder radius").toLatin1().data()));

    paramsOut->clear();

    return retval;
}

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
        case pcl::SACMODEL_PLANE:
        case pcl::SACMODEL_LINE:
        case pcl::SACMODEL_CIRCLE2D:
        case pcl::SACMODEL_CIRCLE3D:
        {
            return ito::RetVal(ito::retError, 0, (tr("Fit of model type %1 not supported").arg(QString::number(modelType))).toLatin1().data());
        }
        case pcl::SACMODEL_SPHERE:
        {
            distanceType = 3;
            double* value = NULL;

            if (opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
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
        case pcl::SACMODEL_CYLINDER:
        {
            distanceType = 1;
            double* value = NULL;

            if (opts[0].getLen() < 3 || (value = (double*)(opts[0].getVal<void*>())) == NULL)
            {
                return ito::RetVal(ito::retError, 0, (tr("Cylinder model must have 7 parameters, [x,y,z], [dx, dy, dz] and r. [x,y,z] was not defined correctly.").arg(QString::number(modelType))).toLatin1().data());
            }
            else
            {
                linePt[0] = cv::saturate_cast<float>(value[0]);
                linePt[1] = cv::saturate_cast<float>(value[1]);
                linePt[2] = cv::saturate_cast<float>(value[2]);
            }

            if (opts[1].getLen() < 3 || (value = (double*)(opts[1].getVal<void*>())) == NULL)
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
    pcl::PointXYZ center(linePt[0], linePt[1], linePt[2]);

    switch(pclIn->getType())
    {
        default:
        case ito::pclInvalid:
            return ito::RetVal(ito::retError, 0, tr("invalid point cloud type not defined or point cloud invalid").toLatin1().data());
        case ito::pclXYZ:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZNormal);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZNormal);
                pclOut->resize(pclIn->size());
            }
        }
        break;
        case ito::pclXYZI:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZINormal);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZINormal);
                pclOut->resize(pclIn->size());
            }
        }
        break;
        case ito::pclXYZRGBA:
        {
            if (pclIn == pclOut)
            {
                pclOut = new ito::PCLPointCloud(ito::pclXYZRGBNormal);
                pclOut->resize(pclIn->size());
                overwriteInput = true;
            }
            else
            {
                *pclOut = ito::PCLPointCloud(ito::pclXYZRGBNormal);
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
            pcl::PointCloud<pcl::PointNormal>::Ptr pclDists = pclOut->toPointXYZNormal();

            if (distanceType == 0)
            {

                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            pcl::PointCloud<pcl::PointNormal>::Ptr pclDists = pclOut->toPointXYZNormal();

            if (distanceType == 0)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclDists = pclOut->toPointXYZINormal();

            if (distanceType == 0)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclDists = pclOut->toPointXYZRGBNormal();

            if (distanceType == 0)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    memcpy(pclDists->at(np).data, pclSrc->at(np).data, sizeof(float) * 4);
                    memset(pclDists->at(np).normal, 0, sizeof(float) * 4);

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclDists = pclOut->toPointXYZINormal();

            if (distanceType == 0)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {

                    if (ito::isFinite<float>(pclDists->at(np).z))
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
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclDists = pclOut->toPointXYZRGBNormal();

            if (distanceType == 0)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir));
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 1)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    curPt = Eigen::Vector4f(pclSrc->at(np).data[0], pclSrc->at(np).data[1], pclSrc->at(np).data[2], 0);
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = sqrt(pcl::sqrPointToLineDistance (curPt, linePt, lineDir)) - radius;
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 2)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    if (ito::isFinite<float>(pclDists->at(np).z))
                    {
                        pclDists->at(np).curvature = pcl::euclideanDistance(pclSrc->at(np), center);
                    }
                    else
                    {
                        pclDists->at(np).curvature = floatNAN;
                    }
                }
            }
            else if (distanceType == 3)
            {
                #if (USEOMP)
                #pragma omp for schedule(guided)
                #endif
                for (int np = 0; np < pclOut->size(); np++)
                {
                    if (ito::isFinite<float>(pclDists->at(np).z))
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

//--------------------------------------------------------------------------------------------------------
/*static*/ const QString PclTools::pclDistanceToModelDObjDOC = QObject::tr("calculates the distance from points in a given data object to a model.");

//--------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclDistanceToModelDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand,paramsOpt,paramsOut).containsError();
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("inObj", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input data object (real type)").toLatin1().data()));
    paramsMand->append(ito::Param("distanceObj", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output distance object (inplace allowed)").toLatin1().data()));
    paramsMand->append(ito::Param("modelType", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Model type according to enum pcl::SacModel (sphere: 4, cylinder: 5)").toLatin1().data()));
    paramsMand->append(ito::Param("coefficients", ito::ParamBase::DoubleArray | ito::ParamBase::In, NULL, tr("model coefficients (sphere: p_x, p_y, p_z, r; cylinder: p_x, p_y, p_z, v_x, v_y, v_z, r)").toLatin1().data()));

    paramsOpt->clear();
    paramsOut->clear();

    return retval;
}

//--------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal distanceToModelDObjHelper(const ito::DataObject *in, ito::DataObject *out, const int modelType, const double *coefficients)
{
    ito::RetVal retval;
    const cv::Mat *inMat = NULL;
    cv::Mat *outMat = NULL;
    const _Tp *rowIn = NULL;
    _Tp *rowOut = NULL;
    int dims = in->getDims();
    bool isInsideImage;
    double diff1, diff2, diff3;
    Eigen::Vector4d line_dir;
    Eigen::Vector4d line_pt(coefficients[0], coefficients[1], coefficients[2], 0);
    Eigen::Vector4d pt(0.0, 0.0, 0.0, 0.0);
    double line_dir_norm;

    switch (modelType)
    {
    case pcl::SACMODEL_CYLINDER:
        line_dir = Eigen::Vector4d(coefficients[4], coefficients[5], coefficients[6], 0);
        line_dir_norm = line_dir.squaredNorm();
        break;
    }

    for (int p = 0; p < in->calcNumMats(); ++p)
    {
        inMat = in->getCvPlaneMat(p);
        outMat = out->getCvPlaneMat(p);

        for (int m = 0; m < inMat->rows; ++m)
        {
            rowIn = inMat->ptr<_Tp>(m);
            rowOut = outMat->ptr<_Tp>(m);
            pt[1] = in->getPixToPhys(dims - 2, m, isInsideImage);

            for (int n = 0; n < inMat->cols; ++n)
            {
                pt[0] = in->getPixToPhys(dims - 1, n, isInsideImage);
                pt[2] = cv::saturate_cast<double>(rowIn[n]);
                switch (modelType)
                {
                case pcl::SACMODEL_SPHERE:
                    diff1 = pt.x() - coefficients[0];
                    diff2 = pt.y() - coefficients[1];
                    diff3 = pt.z() - coefficients[2];
                    rowOut[n] = cv::saturate_cast<_Tp>(std::sqrt(diff1*diff1 + diff2*diff2 + diff3*diff3) - coefficients[3]);
                    break;
                case pcl::SACMODEL_CYLINDER:
                    rowOut[n] = cv::saturate_cast<_Tp>(std::sqrt((line_dir.cross3 (line_pt - pt)).squaredNorm () / line_dir_norm) - coefficients[6]);
                    break;
                }
            }
        }
    }

    return retval;
}

//--------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclDistanceToModelDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    int modelType = paramsMand->at(2).getVal<int>();
    double *coeffs = paramsMand->at(3).getVal<double*>();
    int coeffsLen = paramsMand->at(3).getLen();
    const ito::DataObject *inObj = paramsMand->at(0).getVal<const ito::DataObject*>();
    ito::DataObject *outObj = paramsMand->at(1).getVal<ito::DataObject*>();
    ito::DataObject outObjTemp;
    bool newOutObj = false;

    //check modelType and coefficients
    switch (modelType)
    {
    case pcl::SACMODEL_SPHERE:
        if (coeffsLen != 4)
        {
            retval += ito::RetVal(ito::retError, 0, tr("four coefficients are required for a sphere model (p_x, p_y, p_z, r)").toLatin1().data());
        }
        break;
    case pcl::SACMODEL_CYLINDER:
        if (coeffsLen != 7)
        {
            retval += ito::RetVal(ito::retError, 0, tr("seven coefficients are required for a sphere model (p_x, p_y, p_z, v_x, v_y, v_z, r)").toLatin1().data());
        }
        break;
    default:
        retval += ito::RetVal::format(ito::retError, 0, tr("modelType %i not supported.").toLatin1().data(), modelType);
        break;
    }

    //check input data object
    retval += ito::dObjHelper::verifyDataObjectType(inObj, "inObj", 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);

    if (!retval.containsError())
    {
        if (inObj == outObj) //inplace
        {
            outObjTemp = *outObj;
        }
        else
        {
            if (inObj->getType() != outObj->getType() || inObj->getSize() != outObj->getSize())
            {
                newOutObj = true;
                outObjTemp = ito::DataObject(inObj->getDims(), inObj->getSize(), inObj->getType(), inObj->getContinuous());
                inObj->copyAxisTagsTo(outObjTemp);
            }
            else
            {
                outObjTemp = *outObj;
            }
        }

        switch (inObj->getType())
        {
        case ito::tUInt8:
            retval += distanceToModelDObjHelper<ito::uint8>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tInt8:
            retval += distanceToModelDObjHelper<ito::int8>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tUInt16:
            retval += distanceToModelDObjHelper<ito::uint16>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tInt16:
            retval += distanceToModelDObjHelper<ito::int16>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tUInt32:
            retval += distanceToModelDObjHelper<ito::uint32>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tInt32:
            retval += distanceToModelDObjHelper<ito::int32>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tFloat32:
            retval += distanceToModelDObjHelper<ito::float32>(inObj, &outObjTemp, modelType, coeffs);
            break;
        case ito::tFloat64:
            retval += distanceToModelDObjHelper<ito::float64>(inObj, &outObjTemp, modelType, coeffs);
            break;
        }

        if (!retval.containsError() && newOutObj)
        {
            *outObj = outObjTemp;
        }
    }

    return retval;
}
