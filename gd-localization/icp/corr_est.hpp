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
#ifndef CHUNKMAP_CORR_EST_H_
#define CHUNKMAP_CORR_EST_H_

#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include "../gd/mutli_execute.h"

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
chunkmap::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size ());

  std::vector<int> index (1);
  std::vector<float> distance (1);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;
  
  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (pcl::isSamePointType<PointSource, PointTarget> ())
  {
    // Iterate over the input set of source indices
    auto time_start = std::chrono::steady_clock::now();
    __counter += 1;
#if 1
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
#else
    std::vector<std::function<void()>> tasks;
    const auto *treePtr = tree_.get();
    auto action = [](pcl::Correspondence *corr, const PointSource *p, int id, double max_dist_sqr, const KdTree *const treePtr) {
        std::vector<int> index (1);
        std::vector<float> distance (1);
        treePtr->nearestKSearch (*p, 1, index, distance);
        if (distance[0] > max_dist_sqr)
            distance[0] = -1;
        // pcl::Correspondence corr;
        corr->index_query = id;
        corr->index_match = index[0];
        corr->distance = distance[0];
        // return corr;
    };
    std::vector<pcl::Correspondence> result(indices_->size());
    int tempId = 0;
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    // for(int i=0;i<indices_->size();i++)
    {
        tasks.push_back(std::bind(action, &result[tempId++], &input_->points[*idx], *idx, max_dist_sqr, treePtr));
    }
    multi_execute<2>(tasks);
    for (auto &corr : result)
    {
        if(corr.distance >= 0)
        {
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
#endif
    auto time_stop = std::chrono::steady_clock::now();
    std::cout << "corr est time: "<< std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start).count() << std::endl;
  }
  else
  {
    PointTarget pt;
    
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      // Copy the source data to a target PointTarget format so we can search in the tree
      copyPoint (input_->points[*idx], pt);

      tree_->nearestKSearch (pt, 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
chunkmap::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  // WRAN: useless
  // std::cout << "determineReciprocalCorrespondences" << std::endl;
  if (!initCompute ())
    return;

  // setup tree for reciprocal search
  // Set the internal point representation of choice
  if (!initComputeReciprocal())
    return;
  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size());
  std::vector<int> index (1);
  std::vector<float> distance (1);
  std::vector<int> index_reciprocal (1);
  std::vector<float> distance_reciprocal (1);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;
  int target_idx = 0;

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (pcl::isSamePointType<PointSource, PointTarget> ())
  {
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      target_idx = index[0];

      tree_reciprocal_->nearestKSearch (target_->points[target_idx], 1, index_reciprocal, distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  else
  {
    PointTarget pt_src;
    PointSource pt_tgt;
   
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      // Copy the source data to a target PointTarget format so we can search in the tree
      copyPoint (input_->points[*idx], pt_src);

      tree_->nearestKSearch (pt_src, 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      target_idx = index[0];

      // Copy the target data to a target PointSource format so we can search in the tree_reciprocal
      copyPoint (target_->points[target_idx], pt_tgt);

      tree_reciprocal_->nearestKSearch (pt_tgt, 1, index_reciprocal, distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

//#define PCL_INSTANTIATE_CorrespondenceEstimation(T,U) template class PCL_EXPORTS pcl::registration::CorrespondenceEstimation<T,U>;

#endif /* CHUNKMAP_CORR_EST_H_ */