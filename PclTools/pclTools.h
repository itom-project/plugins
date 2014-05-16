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


//----------------------------------------------------------------------------------------------------------------------------------
/** @class PclToolsInterface
*   @brief short description
*
*   AddIn Interface for the PclTools class s. also \ref PclTools
*/
class PclToolsInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
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
        
        static ito::RetVal savePointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal loadPointCloud(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadPointCloudParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal savePolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal loadPolygonMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadPolygonMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal transformAffine(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal transformAffineParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclFitCylinder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclFitCylinderParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclEstimateNormals(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclEstimateNormalsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclRemoveNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclRemoveNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclPassThrough(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPassThroughParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclVoxelGrid(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclVoxelGridParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclStatisticalOutlierRemoval(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclStatisticalOutlierRemovalParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclRandomSample(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclRandomSampleParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclGetMinMax3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetMinMax3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclGetPercentageThreshold(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetPercentageThresholdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclGetHistogram(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclGetHistogramParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclCylinderClipper3D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclCylinderClipper3DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclPCA(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPCAParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclPolygonMeshFromIndices(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclPolygonMeshFromIndicesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclMeshTriangulation(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclMeshTriangulationParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclSampleToDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclSampleToDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal pclOrganizedFastMesh(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal pclOrganizedFastMeshParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    protected:
        //static int savePolygonFileSTLB(const std::string &file_name, const pcl::PolygonMesh& mesh);

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

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};


//----------------------------------------------------------------------------------------------------------------------------------

#endif // PCLTOOLS_H
