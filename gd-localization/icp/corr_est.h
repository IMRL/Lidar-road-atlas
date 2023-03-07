/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <string>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

// #include "flann_multi.h"

namespace chunkmap
{
  namespace registration
  {
    /** \brief @b CorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimation : public pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
        // using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_cloud_updated_;
        // using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
      public:
        int __counter = 0;

        using Ptr = pcl::shared_ptr<CorrespondenceEstimation<PointSource, PointTarget, Scalar> >;
        using ConstPtr = pcl::shared_ptr<const CorrespondenceEstimation<PointSource, PointTarget, Scalar> >;

        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
        using pcl::PCLBase<PointSource>::deinitCompute;

        using KdTree = pcl::search::KdTree<PointTarget>;
        // using KdTree = pcl::search::KdTree<PointTarget, pcl::KdTreeFLANNMulti<PointTarget>>;
        using KdTreePtr = typename KdTree::Ptr;
        // KdTreePtr tree_;

        using PointCloudSource = pcl::PointCloud<PointSource>;
        using PointCloudSourcePtr = typename PointCloudSource::Ptr;
        using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

        using PointCloudTarget = pcl::PointCloud<PointTarget>;
        using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
        using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

        using PointRepresentationConstPtr = typename KdTree::PointRepresentationConstPtr;

        // void setInputTarget(const PointCloudTargetConstPtr& cloud)
        // {
        //     if (cloud->points.empty ())
        //     {
        //         PCL_ERROR ("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
        //         return;
        //     }
        //     target_ = cloud;

        //     // Set the internal point representation of choice
        //     if (point_representation_)
        //         tree_->setPointRepresentation (point_representation_);

        //     target_cloud_updated_ = true;
        // }

        // bool initCompute()
        // {
        //     if (!target_) {
        //         PCL_ERROR("[pcl::registration::%s::compute] No input target dataset was given!\n",
        //                 getClassName().c_str());
        //         return (false);
        //     }
            
        //     // Only update target kd-tree if a new target cloud was set
        //     if (target_cloud_updated_ && !force_no_recompute_) {
        //         // If the target indices have been given via setIndicesTarget
        //         if (target_indices_)
        //         tree_->setInputCloud(target_, target_indices_);
        //         else
        //         tree_->setInputCloud(target_);
            
        //         target_cloud_updated_ = false;
        //     }
            
        //     return (pcl::PCLBase<PointSource>::initCompute());
        // }

        /** \brief Empty constructor. */
        CorrespondenceEstimation () 
        {
          // tree_.reset(new KdTree());
          corr_name_  = "CorrespondenceEstimation";
        }
      
        /** \brief Empty destructor */
        ~CorrespondenceEstimation () {}

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ()) override;

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ()) override;

        
        /** \brief Clone and cast to CorrespondenceEstimationBase */
        typename pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::Ptr
        clone () const override
        {
          Ptr copy (new CorrespondenceEstimation<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }
     };
  }
}

#include "corr_est.hpp"
