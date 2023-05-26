/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_CYLINDER_CLIPPER3D_H_
#define PCL_CYLINDER_CLIPPER3D_H_
//temporary, replace by #include "clipper3D.h" if this file becomes part of pcl
#include "pcl/filters/clipper3D.h"

namespace pcl
{
  /**
    * \author Marc Gronle <gronle@ito.uni-stuttgart.de>
    * \brief Implementation of a cylinder clipper in 3D.
    * \ingroup filters
    */
  template<typename PointT>
  class CylinderClipper3D : public Clipper3D<PointT>
  {
    public:

#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
      typedef pcl::shared_ptr< CylinderClipper3D<PointT> > Ptr;
      typedef pcl::shared_ptr< const CylinderClipper3D<PointT> > ConstPtr;
#else
      typedef boost::shared_ptr< CylinderClipper3D<PointT> > Ptr;
      typedef boost::shared_ptr< const CylinderClipper3D<PointT> > ConstPtr;
#endif


      /**
        * \author Marc Gronle <gronle@ito.uni-stuttgart.de>
        * \brief Constructor taking an affine transformation matrix, which allows also shearing of the clipping area
        * \param[in] transformation the 3x3 affine transformation matrix that is used to describe the unit cube
        */
      CylinderClipper3D (const Eigen::Affine3f& transformation);

      /**
        * \brief creates a BoxClipper object with a scaled box in general pose
        * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
        * \param[in] translation the position of the box center
        * \param[in] box_size the size of the box for each dimension
        */
      CylinderClipper3D (const Eigen::Vector3f& pt_on_axis, const Eigen::Vector3f& axis_direction);

      /**
        * \brief Set the affine transformation
        * \param[in] transformation
        */
      void setTransformation (const Eigen::Affine3f& transformation);

      /**
        * \brief sets the box in general pose given by the orientation position and size
        * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
        * \param[in] translation the position of the box center
        * \param[in] box_size the size of the box for each dimension
        */
      void setTransformation (const Eigen::Vector3f& pt_on_axis, const Eigen::Vector3f& axis_direction);

      void setMinMaxRadius( const float minRadius, const float maxRadius );

      /**
        * \brief virtual destructor
        */
      virtual ~CylinderClipper3D () throw ();

      virtual bool
      clipPoint3D (const PointT& point) const;

      virtual bool
      clipLineSegment3D (PointT& from, PointT& to) const;
#if PCL_VERSION_COMPARE(<, 1, 7, 0)
      virtual void
      clipPlanarPolygon3D (std::vector<PointT>& polygon) const;

      virtual void
      clipPlanarPolygon3D (const std::vector<PointT>& polygon, std::vector<PointT>& clipped_polygon) const;

#else
      virtual void
      clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon) const;

      virtual void
      clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const;
#endif
      virtual void
      clipPointCloud3D (const pcl::PointCloud<PointT> &cloud_in, std::vector<int>& clipped, const std::vector<int>& indices = std::vector<int> ()) const;


      virtual Clipper3D<PointT>*
      clone () const;

    protected:
      float getDistance (const PointT& point) const;
      void transformPoint (const PointT& pointIn, PointT& pointOut) const;
    private:
      /**
        * \brief the affine transformation that is applied before clipping is done on the unit cube.
        */
      Eigen::Affine3f transformation_; //mode is affine, 4x4 matrix, ColumnMajor (default)
      float minRadius_;
      float maxRadius_;
      float minRadiusSquare_;
      float maxRadiusSquare_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/cylinder_clipper3D.hpp>
#endif

#endif // PCL_CylinderClipper3D_CLIPPER3D_H_
