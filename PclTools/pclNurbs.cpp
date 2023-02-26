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

#if PCLHASSURFACENURBS

#include "pclTools.h"
#define EIGEN_QT_SUPPORT

#include "DataObject/dataobj.h"
#include "common/helperCommon.h"
#include "PointCloud/pclStructures.h"
#include "PointCloud/pclFunctions.h"
#include "PointCloud/impl/pclFunctionsImpl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

template <typename Point> void 
pclPointCloud2Vector3d(const pcl::PointCloud<Point>* cloud, pcl::on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        const Point &p = cloud->at(i);
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
}

template<typename Point> ito::RetVal pclFitTrimmedBSplineHelper(const pcl::PointCloud<Point>* cloud, QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    if (cloud == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, "no cloud given");
    }
    else
    {
        ito::PCLPolygonMesh *meshParam = paramsOpt->at(0).getVal<ito::PCLPolygonMesh*>();
        unsigned int order = paramsOpt->at(1).getVal<int>();
        unsigned int refinement = paramsOpt->at(2).getVal<int>();
        unsigned int iterations = paramsOpt->at(3).getVal<int>();
        unsigned int mesh_resolution = paramsOpt->at(4).getVal<int>();
        QByteArray filename = paramsOpt->at(5).getVal<char*>();
        const double *surfaceFitParams = paramsOpt->at(6).getVal<double*>(); //always 4 elements due to meta contraints
        const double *curveFitParams = paramsOpt->at(7).getVal<double*>(); //always 4 elements due to meta contraints

        pcl::on_nurbs::NurbsDataSurface data;
        pclPointCloud2Vector3d(cloud, data.interior);

        // ############################################################################
        // fit B-spline surface

        // parameters

        pcl::on_nurbs::FittingSurface::Parameter params;
        params.interior_smoothness = surfaceFitParams[0]; // 0.2;
        params.interior_weight = surfaceFitParams[1]; // 1.0;
        params.boundary_smoothness = surfaceFitParams[2]; // 0.2;
        params.boundary_weight = surfaceFitParams[3]; // 0.0;

        // initialize
        ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
        pcl::on_nurbs::FittingSurface fit(&data, nurbs);
        //  fit.setQuiet (false); // enable/disable debug output

        // mesh for visualization
        pcl::PolygonMesh mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::Vertices> mesh_vertices;
        //pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);

        // surface refinement
        for (unsigned i = 0; i < refinement; i++)
        {
            fit.refine(0);
            fit.refine(1);
            fit.assemble(params);
            fit.solve();
            //pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        }

        // surface fitting with final refinement level
        for (unsigned i = 0; i < iterations; i++)
        {
            fit.assemble(params);
            fit.solve();
            //pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        }

        // ############################################################################
        // fit B-spline curve

        // parameters
        pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
        curve_params.addCPsAccuracy = curveFitParams[0]; //5e-2;
        curve_params.addCPsIteration = qRound(curveFitParams[1]); //3;
        curve_params.maxCPs = qRound(curveFitParams[2]); //200;
        curve_params.accuracy = curveFitParams[3]; //1e-3;
        curve_params.iterations = qRound(curveFitParams[4]); //100;

        curve_params.param.closest_point_resolution = qRound(curveFitParams[5]); //0;
        curve_params.param.closest_point_weight = curveFitParams[6]; //1.0;
        curve_params.param.closest_point_sigma2 = curveFitParams[7]; //0.1;
        curve_params.param.interior_sigma2 = curveFitParams[8]; //0.00001;
        curve_params.param.smooth_concavity = curveFitParams[9]; //1.0;
        curve_params.param.smoothness = curveFitParams[10]; //1.0;

        // initialisation (circular)
        pcl::on_nurbs::NurbsDataCurve2d curve_data;
        curve_data.interior = data.interior_param;
        curve_data.interior_weight_function.push_back(true);
        ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

        // curve fitting
        pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
        // curve_fit.setQuiet (false); // enable/disable debug output
        curve_fit.fitting(curve_params);

        // ############################################################################
        // triangulation of trimmed surface
        if (meshParam)
        {
            pcl::PolygonMesh::Ptr sharedMesh(new pcl::PolygonMesh);
            pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, *sharedMesh, mesh_resolution);
            //pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, *sharedMesh, mesh_resolution);
            *meshParam = ito::PCLPolygonMesh(sharedMesh);
        }
        
        // save trimmed B-spline surface
        if (fit.m_nurbs.IsValid() && filename != "")
        {
            ONX_Model model;
            ONX_Model_Object& surf = model.m_object_table.AppendNew();
            surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
            surf.m_bDeleteObject = true;
            surf.m_attributes.m_layer_index = 1;
            surf.m_attributes.m_name = "surface";

            ONX_Model_Object& curv = model.m_object_table.AppendNew();
            curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
            curv.m_bDeleteObject = true;
            curv.m_attributes.m_layer_index = 2;
            curv.m_attributes.m_name = "trimming curve";

            if (!model.Write(filename.data()))
            {
                retval += ito::RetVal::format(ito::retWarning, 0, "Error writing the model to file '%s'.", filename.data());
            }
        }
        else if (filename != "")
        {
            retval += ito::RetVal(ito::retWarning, 0, "B-spline could not be saved since the fitted nurbs is not valid.");
        }


    }
    return retval;
}

//------------------------------------------------------------------------------------------------------------------------------
const QString PclTools::pclFitTrimmedBSplineDOC = QObject::tr("This filter fits a trimmed B-spline to a point cloud. \n\
This filter is mainly derived from the example at http://pointclouds.org/documentation/tutorials/bspline_fitting.php. \n\
\n\
After the fit, you can either obtain the result as a polygonMesh that is discretized from the resulting B-spline or \n\
the fitted B-spline can be saved in the OpenNURBS format (3dm) to the harddrive.");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitTrimmedBSplineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += ito::checkParamVectors(paramsMand, paramsOpt, paramsOut);
    if (retval.containsError())
    {
        return retval;
    }

    paramsMand->clear();
    paramsMand->append(ito::Param("pointCloudIn", ito::ParamBase::PointCloudPtr | ito::ParamBase::In, NULL, tr("Input point cloud with normal values").toLatin1().data()));
    
    paramsOpt->clear();
    paramsOpt->append(ito::Param("meshOut", ito::ParamBase::PolygonMeshPtr | ito::ParamBase::In, NULL, tr("fitted mesh").toLatin1().data()));
    paramsOpt->append(ito::Param("order", ito::ParamBase::Int | ito::ParamBase::In, 1, 100, 3, tr("polynomial order of the B-spline surface").toLatin1().data()));
    paramsOpt->append(ito::Param("refinement", ito::ParamBase::Int | ito::ParamBase::In, 0, 100, 1, tr("number of refinement iterations, where for each iteration control-points are inserted, approximately doubling the control points in each parametric direction of the B-spline surface").toLatin1().data()));
    paramsOpt->append(ito::Param("iterations", ito::ParamBase::Int | ito::ParamBase::In, 0, 100, 1, tr("number of iterations that are performed after refinement is completed").toLatin1().data()));
    paramsOpt->append(ito::Param("meshResolution", ito::ParamBase::Int | ito::ParamBase::In, 1, 10000, 256, tr("the number of vertices in each parametric direction, used for triangulation of the B-spline surface").toLatin1().data()));
    paramsOpt->append(ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("filename of a *.3dm OpenNURBS file. If given, the B-spline surface and curve are saved as two layers in the file").toLatin1().data()));

    double surfaceFitParams[] = {0.2, 1.0, 0.2, 0.0};
    ito::Param p("surfaceFitParams", ito::ParamBase::DoubleArray | ito::ParamBase::In, 4, surfaceFitParams, tr("parameters for the B-spline surface fitting (interior_smoothness, interior_weight, boundary_smoothness, boundary_weight)").toLatin1().data());
    ito::DoubleArrayMeta dam(0.0, std::numeric_limits<double>::max(), 0.0, 4, 4);
    p.setMeta(&dam, false);
    paramsOpt->append(p);

    double curveFitParams[] = { 5e-2, 3.0, 200.0, 1e-3, 100.0, 0.0, 1.0, 0.1, 0.00001, 1.0, 1.0 };
    p = ito::Param("CurveFitParams", ito::ParamBase::DoubleArray | ito::ParamBase::In, 11, curveFitParams, tr("parameters for the B-spline curve fitting (addCPsAccuracy, addCPsIteration (int), maxCPs (int), accuracy, iterations (int), closest_point_resolution (int), closest_point_weight, closest_point_sigma2, interior_sigma2, smooth_concavity, smoothness)").toLatin1().data());
    dam = ito::DoubleArrayMeta(0.0, std::numeric_limits<double>::max(), 0.0, 11, 11);
    p.setMeta(&dam, false);
    paramsOpt->append(p);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal PclTools::pclFitTrimmedBSpline(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    // ############################################################################
    // load point cloud
    const ito::PCLPointCloud *cloud = paramsMand->at(0).getVal<ito::PCLPointCloud*>();

    switch (cloud->getType())
    {
    case ito::pclXYZ:
        return pclFitTrimmedBSplineHelper<pcl::PointXYZ>(cloud->toPointXYZConst().get(), paramsMand, paramsOpt, paramsOut);
    case ito::pclXYZI:
        return pclFitTrimmedBSplineHelper<pcl::PointXYZI>(cloud->toPointXYZIConst().get(), paramsMand, paramsOpt, paramsOut);
    case ito::pclXYZNormal:
        return pclFitTrimmedBSplineHelper<pcl::PointNormal>(cloud->toPointXYZNormalConst().get(), paramsMand, paramsOpt, paramsOut);
    case ito::pclXYZINormal:
        return pclFitTrimmedBSplineHelper<pcl::PointXYZINormal>(cloud->toPointXYZINormalConst().get(), paramsMand, paramsOpt, paramsOut);
    case ito::pclXYZRGBA:
        return pclFitTrimmedBSplineHelper<pcl::PointXYZRGBA>(cloud->toPointXYZRGBAConst().get(), paramsMand, paramsOpt, paramsOut);
    case ito::pclXYZRGBNormal:
        return pclFitTrimmedBSplineHelper<pcl::PointXYZRGBNormal>(cloud->toPointXYZRGBNormalConst().get(), paramsMand, paramsOpt, paramsOut);
    default:
        return ito::RetVal(ito::retError, 0, "unsupported type of the input point cloud");
    } 
}

#endif