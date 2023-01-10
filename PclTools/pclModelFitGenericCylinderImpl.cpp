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
*/

#include <pcl/point_types.h>

//these includes are not inside of another cpp where they are really needed (due to big-object compiler problems)
//therefore they are split into this single file, however it is necessary to implement the necessary classes for
//every point type, that is used.

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)

#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>

PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointNormal, struct pcl::PointNormal)
PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointNormal, struct pcl::Normal)
PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointXYZINormal, struct pcl::PointXYZINormal)
PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointXYZINormal, struct pcl::Normal)
PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointXYZRGBNormal, struct pcl::PointXYZRGBNormal)
PCL_INSTANTIATE_SampleConsensusModelCylinder(struct pcl::PointXYZRGBNormal, struct pcl::Normal)

#endif