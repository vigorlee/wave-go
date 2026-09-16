#include <multigrid_pclomp/multi_voxel_grid_covariance_omp.h>
#include <multigrid_pclomp/multi_voxel_grid_covariance_omp_impl.hpp>

template class pclomp::MultiVoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::MultiVoxelGridCovariance<pcl::PointXYZI>;
