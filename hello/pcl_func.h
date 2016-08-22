#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
//#include <pcl/features/intensity_spin.h>

#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "mlpack_func.h"

using namespace pcl;

inline bool path_exist(const std::string& name);

PointCloud<PointXYZ>::Ptr loadCloud(std::string file_name);

arma::mat convert_to_mat(PointCloud<FPFHSignature33>::Ptr descriptors);

void downsample(PointCloud<PointXYZ>::Ptr &points, float leaf_size,
				PointCloud<PointXYZ>::Ptr &downsampled_out);

PointCloud<Normal>::Ptr ParseNormal(PointCloud<PointNormal>::Ptr pointNormals);

void compute_surface_normals(PointCloud<PointXYZ>::Ptr points, float radius, PointCloud<PointNormal>::Ptr &normals_out);

void detect_keypoints(PointCloud<PointNormal>::Ptr points, 
					  float min_scale, int nr_octaves, int nr_scales_per_octave, 
					  float min_contrast, PointCloud<PointWithScale>::Ptr &keypoints_out);

void compute_PFH_features(PointCloud<PointXYZ>::Ptr points, PointCloud<Normal>::Ptr normals,
						  float radius, PointCloud<PFHSignature125>::Ptr &descriptors_out, PointCloud<PointWithScale>::Ptr keypoints=NULL);
						  
void compute_FPFH_features(PointCloud<PointXYZ>::Ptr points, PointCloud<Normal>::Ptr normals, 
						   float radius, PointCloud<FPFHSignature33>::Ptr &descriptors_out, PointCloud<PointWithScale>::Ptr keypoints=NULL);

void FPFH_est(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals);

void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
								   pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                                   std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);

// Points, Downsample Points, Normals
void visualize_normals (const PointCloud<PointXYZ>::Ptr points,
                        const PointCloud<PointXYZ>::Ptr normal_points,
                        const PointCloud<Normal>::Ptr normals);
						
void visualize_keypoints (const PointCloud<PointXYZ>::Ptr points,
                          const PointCloud<PointWithScale>::Ptr keypoints);
						  
void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores);
								
PointCloud<PointXYZ>::Ptr RANSAC(PointCloud<PointXYZ>::Ptr points);