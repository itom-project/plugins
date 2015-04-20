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

#ifndef PCLTOOLS_H
#define PCLTOOLS_H

#include "common/addInInterface.h"

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

#define CIRCLE3D_LIMIT_NORMAL 0
#define PLANE_LIMIT_NORMAL 0
#define CONE_LIMIT_NORMAL 0

//----------------------------------------------------------------------------------------------------------------------------------
/** @class PclToolsInterface
*   @brief short description
*
*   AddIn Interface for the PclTools class s. also \ref PclTools
*/
class PclToolsInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        PclToolsInterface();
        ~PclToolsInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class PclTools
*   @brief short description
*
*   longer description
*/
class PclTools : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        PclTools();
        ~PclTools() {}

    public:
        friend class PclToolsInterface;

        static const char *savePointCloudDOC;
        static ito::RetVal savePointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *loadPointCloudDOC;
        static ito::RetVal loadPointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadPointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *savePolygonMeshDOC;
        static ito::RetVal savePolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *saveVTKImageDataDOC;
        static ito::RetVal saveVTKImageData(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveVTKImageDataParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *loadPolygonMeshDOC;
        static ito::RetVal loadPolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadPolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *transformAffineDOC;
        static ito::RetVal transformAffine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal transformAffineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclDistanceToModelDOC;
        static ito::RetVal pclDistanceToModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclDistanceToModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclDistanceToModelDObjDOC;
        static ito::RetVal pclDistanceToModelDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclDistanceToModelDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclProjectOnModelDOC;
        static ito::RetVal pclProjectOnModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclProjectOnModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclFitModelGeneric(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, int fitType);
        
        static const char *pclFitModelDOC;
        static ito::RetVal pclFitModel(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitModelParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitModelDObjDOC;
        static ito::RetVal pclFitModelDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitModelDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitCylinderDOC;
        static ito::RetVal pclFitCylinder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitCylinderParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitSphereDOC;
        static ito::RetVal pclFitSphere(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitSphereParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
              
        static const char *pclFitCircle2DDOC;
        static ito::RetVal pclFitCircle2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitCircle2DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitCircle3DDOC;
        static ito::RetVal pclFitCircle3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitCircle3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitLineDOC;
        static ito::RetVal pclFitLine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitLineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclFitPlaneDOC;
        static ito::RetVal pclFitPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitPlaneParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);  

        static const char *pclFitConeDOC;
        static ito::RetVal pclFitCone(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitConeParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);  

        static const char *pclEstimateNormalsDOC;
        static ito::RetVal pclEstimateNormals(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclEstimateNormalsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclRemoveNaNDOC;
        static ito::RetVal pclRemoveNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclRemoveNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclPassThroughDOC;
        static ito::RetVal pclPassThrough(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPassThroughParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclCropBoxDOC;
        static ito::RetVal pclCropBox(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclCropBoxParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclVoxelGridDOC;
        static ito::RetVal pclVoxelGrid(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclVoxelGridParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclStatisticalOutlierRemovalDOC;
        static ito::RetVal pclStatisticalOutlierRemoval(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclStatisticalOutlierRemovalParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclRandomSampleDOC;
        static ito::RetVal pclRandomSample(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclRandomSampleParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclGetMinMax3DDOC;
        static ito::RetVal pclGetMinMax3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetMinMax3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclGetPercentageThresholdDOC;
        static ito::RetVal pclGetPercentageThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetPercentageThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclGetHistogramDOC;
        static ito::RetVal pclGetHistogram(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetHistogramParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclCylinderClipper3DDOC;
        static ito::RetVal pclCylinderClipper3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclCylinderClipper3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclPCADOC;
        static ito::RetVal pclPCA(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPCAParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclPolygonMeshFromIndicesDOC;
        static ito::RetVal pclPolygonMeshFromIndices(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPolygonMeshFromIndicesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclMeshTriangulationDOC;
        static ito::RetVal pclMeshTriangulation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclMeshTriangulationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclSampleToDataObjectDOC;
        static ito::RetVal pclSampleToDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclSampleToDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclOrganizedFastMeshDOC;
        static ito::RetVal pclOrganizedFastMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclOrganizedFastMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclSimplifyMeshDOC;
        static ito::RetVal pclSimplifyMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclSimplifyMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const char *pclPoissonDOC;
        static ito::RetVal pclPoisson(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPoissonParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    protected:
        //static int savePolygonFileSTLB(const std::string &file_name, const pcl::PolygonMesh& mesh);
		static int nthreads;

    private:
        Q_DISABLE_COPY( PclTools )

        static inline size_t fillBucket(ito::int32 *histogram, const float *value, const float *minVal, const float *stepSize, const int &len)
        {
            float v = *value - *minVal;
            if(v >= 0)
            {
                int idx = int(v / (*stepSize));
                if(idx < len) 
                {
                    histogram[idx]++;
                    return 1;
                }
            }
            return 0;
        }

        static double pointToLineDist(const float pt[3], const float modelCoefficients[7]);
        static bool checkFitWithOutNormals(const int &fitObj);
        static bool checkFitNormals(const int &fitObj);
    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};


//----------------------------------------------------------------------------------------------------------------------------------

#endif // PCLTOOLS_H
