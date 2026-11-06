// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#ifndef PCL_MULTI_VOXEL_GRID_COVARIANCE_IMPL_OMP_H_
#define PCL_MULTI_VOXEL_GRID_COVARIANCE_IMPL_OMP_H_

#include <pcl/common/common.h>
#include <pcl/filters/boost.h>
#include "multigrid_pclomp/multi_voxel_grid_covariance_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void pclomp::MultiVoxelGridCovariance<PointT>::applyFilter (
  const PointCloudConstPtr &input, const std::string &grid_id, LeafDict &leaves) const
{
  // Has the input dataset been set already?
  if (!input)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    return;
  }

  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<PointT> (*input, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if((dx*dy*dz) > std::numeric_limits<int32_t>::max())
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    return;
  }

  // Compute the minimum and maximum bounding box values
  BoundingBox bbox;
  bbox.min[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
  bbox.max[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
  bbox.min[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
  bbox.max[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
  bbox.min[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
  bbox.max[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  Eigen::Vector4i div_b = bbox.max - bbox.min + Eigen::Vector4i::Ones ();
  div_b[3] = 0;

  // Clear the leaves
  leaves.clear ();
  // leaves.reserve(8192);

  // Set up the division multiplier
  bbox.div_mul = Eigen::Vector4i (1, div_b[0], div_b[0] * div_b[1], 0);

  int centroid_size = 4;

  // First pass: go over all points and insert them into the right leaf
  for (size_t cp = 0; cp < input->points.size (); ++cp)
  {
    if (!input->is_dense)
      // Check if the point is invalid
      if (!std::isfinite (input->points[cp].x) || !std::isfinite (input->points[cp].y) || !std::isfinite (input->points[cp].z))
        continue;

    LeafID leaf_id = getLeafID(grid_id, input->points[cp], bbox);
    Leaf& leaf = leaves[leaf_id];
    updateLeaf(input->points[cp], centroid_size, leaf);
  }

  // Second pass: go over all leaves and compute centroids and covariance matrices

  // Eigen values and vectors calculated to prevent near singular matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
  Eigen::Vector3d pt_sum;

  // Eigen values less than a threshold of max eigen value are inflated to a set fraction of the max eigen value.
  std::vector<LeafID> leaf_ids_to_remove;
  for (auto it = leaves.begin (); it != leaves.end (); ++it)
  {
    // Normalize the centroid
    Leaf& leaf = it->second;

    // Normalize the centroid
    leaf.centroid /= static_cast<float> (leaf.nr_points);
    // Point sum used for single pass covariance calculation
    pt_sum = leaf.mean_;
    // Normalize mean
    leaf.mean_ /= leaf.nr_points;

    // If the voxel contains sufficient points, its covariance is calculated and is added to the voxel centroids and voxel_grid_info.voxel_centroids clouds.
    // Points with less than the minimum points will have a can not be accurately approximated using a normal distribution.
    if (leaf.nr_points >= min_points_per_voxel_)
    {
      computeLeafParams (pt_sum, eigensolver, leaf);
    }
    else
    {
      leaf_ids_to_remove.push_back(it->first);
    }
  }

  // Remove leaves that do not have sufficient points
  for (const LeafID & leaf_id: leaf_ids_to_remove)
  {
    leaves.erase(leaf_id);
  }
}

template<typename PointT>
void pclomp::MultiVoxelGridCovariance<PointT>::updateVoxelCentroids (
  const Leaf &leaf, PointCloud &voxel_centroids) const
{
  voxel_centroids.push_back (PointT ());
  voxel_centroids.points.back ().x = leaf.centroid[0];
  voxel_centroids.points.back ().y = leaf.centroid[1];
  voxel_centroids.points.back ().z = leaf.centroid[2];
}

template<typename PointT>
typename pclomp::MultiVoxelGridCovariance<PointT>::LeafID pclomp::MultiVoxelGridCovariance<PointT>::getLeafID (
  const std::string &grid_id, const PointT &point, const BoundingBox &bbox) const
{
  int ijk0 = static_cast<int> (floor (point.x * inverse_leaf_size_[0]) - static_cast<float> (bbox.min[0]));
  int ijk1 = static_cast<int> (floor (point.y * inverse_leaf_size_[1]) - static_cast<float> (bbox.min[1]));
  int ijk2 = static_cast<int> (floor (point.z * inverse_leaf_size_[2]) - static_cast<float> (bbox.min[2]));
  int idx = ijk0 * bbox.div_mul[0] + ijk1 * bbox.div_mul[1] + ijk2 * bbox.div_mul[2];
  LeafID leaf_id;
  leaf_id.parent_grid_id = grid_id;
  leaf_id.leaf_index = idx;
  return leaf_id;
}

template<typename PointT>
void pclomp::MultiVoxelGridCovariance<PointT>::updateLeaf (
  const PointT &point, const int &centroid_size, Leaf &leaf) const
{
  if (leaf.nr_points == 0)
  {
    leaf.centroid.resize (centroid_size);
    leaf.centroid.setZero ();
  }

  Eigen::Vector3d pt3d (point.x, point.y, point.z);
  // Accumulate point sum for centroid calculation
  leaf.mean_ += pt3d;
  // Accumulate x*xT for single pass covariance calculation
  leaf.cov_ += pt3d * pt3d.transpose ();

  Eigen::Vector4f pt (point.x, point.y, point.z, 0);
  leaf.centroid.template head<4> () += pt;
  ++leaf.nr_points;
}

template<typename PointT>
void pclomp::MultiVoxelGridCovariance<PointT>::computeLeafParams (
  const Eigen::Vector3d &pt_sum,
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> &eigensolver,
  Leaf &leaf) const
{
  // Single pass covariance calculation
  leaf.cov_ = (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose ())) / 
    leaf.nr_points + leaf.mean_ * leaf.mean_.transpose ();
  leaf.cov_ *= (leaf.nr_points - 1.0) / leaf.nr_points;

  //Normalize Eigen Val such that max no more than 100x min.
  eigensolver.compute (leaf.cov_);
  Eigen::Matrix3d eigen_val = eigensolver.eigenvalues ().asDiagonal ();
  leaf.evecs_ = eigensolver.eigenvectors ();

  if (eigen_val (0, 0) < 0 || eigen_val (1, 1) < 0 || eigen_val (2, 2) <= 0)
  {
    leaf.nr_points = -1;
    return;
  }

  // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
  double min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val (2, 2);
  if (eigen_val (0, 0) < min_covar_eigvalue)
  {
    eigen_val (0, 0) = min_covar_eigvalue;

    if (eigen_val (1, 1) < min_covar_eigvalue)
    {
      eigen_val (1, 1) = min_covar_eigvalue;
    }

    leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse ();
  }
  leaf.evals_ = eigen_val.diagonal ();

  leaf.icov_ = leaf.cov_.inverse ();
  if (leaf.icov_.maxCoeff () == std::numeric_limits<float>::infinity ( )
      || leaf.icov_.minCoeff () == -std::numeric_limits<float>::infinity ( ) )
  {
    leaf.nr_points = -1;
  }
}

#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl::VoxelGridCovariance<T>;

#endif    // PCL_MULTI_VOXEL_GRID_COVARIANCE_IMPL_H_
