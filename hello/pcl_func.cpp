#include "pcl_func.h"

inline bool path_exist (const std::string& name) {
    //int len;
    //char pBuf[32];
    //GetModuleFileName(NULL, pBuf, len);
    //PCL_ERROR(pBuf);

    std::ifstream f(name.c_str());
    return f.good();
}

PointCloud<PointXYZ>::Ptr loadCloud(std::string file_name)
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    
    clock_t start, finish;
    start = clock();
    
    if (io::loadPCDFile<PointXYZ> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        if (!path_exist(file_name))
            PCL_ERROR ("File not existed \n");
    }
    
    PCL_ERROR ("Time: %lf \n", float(clock()-start)/CLOCKS_PER_SEC);
    
    return cloud;
}

void downsample(PointCloud<PointXYZ>::Ptr &points, float leaf_size,
                                PointCloud<PointXYZ>::Ptr &downsampled_out)
{
    VoxelGrid<PointXYZ> vox_grid;
    vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud (points);
    vox_grid.filter (*downsampled_out);
    std::cout << "Downsample: " << points->size() << " V.S. " <<  downsampled_out->size() << std::endl;
}

PointCloud<Normal>::Ptr ParseNormal(PointCloud<PointNormal>::Ptr pointNormals)
{
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    
    for(size_t i = 0; i < pointNormals->points.size(); ++i)
    {
        Normal normal = Normal(pointNormals->points[i].normal_x, 
                               pointNormals->points[i].normal_y, 
                               pointNormals->points[i].normal_z);
        
        normal.curvature = pointNormals->points[i].curvature;
        normals->push_back(normal);
    }
    
    return normals;
}

void compute_surface_normals(PointCloud<PointXYZ>::Ptr points, float radius, PointCloud<PointNormal>::Ptr &normals_out)
{
    NormalEstimation<PointXYZ, PointNormal> ne;
    
    ne.setSearchMethod(search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    
    ne.setRadiusSearch(radius);
    ne.setInputCloud(points);
    ne.setSearchSurface(points);
    ne.compute(*normals_out);
    
    for (size_t i = 0; i < normals_out->points.size(); i++)
    {
        normals_out->points[i].x = points->points[i].x;
        normals_out->points[i].y = points->points[i].y;
        normals_out->points[i].z = points->points[i].z;
    }
    
    std::cout << "Normals: " << normals_out->points.size() << std::endl;
    
    for (int i = 0; i < normals_out->points.size(); i++)
    {
        if (!isFinite<PointNormal>(normals_out->points[i]))
        {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }
    //io::savePCDFileASCII("normal.pcd", *normals_out);
}

void detect_keypoints(PointCloud<PointNormal>::Ptr points, 
                      float min_scale, int nr_octaves, int nr_scales_per_octave, 
                      float min_contrast, PointCloud<PointWithScale>::Ptr &keypoints_out)
{
    SIFTKeypoint<PointNormal, PointWithScale> sift;
    sift.setSearchMethod(search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
    
    // Set the detection parameters
    sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    
    sift.setInputCloud(points);
    //sift.setSearchSurface(cloud_normals);
    
    sift.compute(*keypoints_out);
    
    std::cout << "Keypoint: " << points->size() << " V.S " << keypoints_out->size() << std::endl;
}

void compute_PFH_features(PointCloud<PointXYZ>::Ptr points, PointCloud<Normal>::Ptr normals,
                          float radius, PointCloud<PFHSignature125>::Ptr &descriptors_out, PointCloud<PointWithScale>::Ptr keypoints)
{
    PFHEstimation<PointXYZ, Normal, PFHSignature125> pfh_est;
    //sift.setSearchMethod(search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
    pfh_est.setSearchMethod(search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    
    pfh_est.setRadiusSearch (radius);
    
    // Use all of the points for analyzing the local structure of the cloud
    pfh_est.setSearchSurface (points);
    pfh_est.setInputNormals (normals);
    
    if (keypoints == NULL)
    {
        pfh_est.setInputCloud(points);
    }
    else
    {
        // Only compute features at the keypoin
        PointCloud<PointXYZ>::Ptr keypoints_xyz (new PointCloud<PointXYZ>);
        copyPointCloud (*keypoints, *keypoints_xyz);
        pfh_est.setInputCloud (keypoints_xyz);
    }
    
    pfh_est.compute (*descriptors_out);
}

void compute_FPFH_features(PointCloud<PointXYZ>::Ptr points, PointCloud<Normal>::Ptr normals,
                           float radius, PointCloud<FPFHSignature33>::Ptr &descriptors_out, PointCloud<PointWithScale>::Ptr keypoints)
{
    FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
    //sift.setSearchMethod(search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
    fpfh_est.setSearchMethod(search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    
    fpfh_est.setRadiusSearch (radius);
    
    // Use all of the points for analyzing the local structure of the cloud
    fpfh_est.setSearchSurface (points);
    fpfh_est.setInputNormals (normals);
    
    if (keypoints == NULL)
    {
        fpfh_est.setInputCloud (points);        
    }
    else
    {
        // Only compute features at the keypoin
        PointCloud<PointXYZ>::Ptr keypoints_xyz (new PointCloud<PointXYZ>);
        copyPointCloud (*keypoints, *keypoints_xyz);
        fpfh_est.setInputCloud (keypoints_xyz);
    }
    
    fpfh_est.compute (*descriptors_out);
}

void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                                   std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
    // Resize the output vector
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());
    
    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);
    
    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }
}

void visualize_normals (const PointCloud<PointXYZ>::Ptr points,
                        const PointCloud<PointXYZ>::Ptr normal_points,
                        const PointCloud<Normal>::Ptr normals)                      
{
    // Add the points and normals to the vizualizer
    visualization::PCLVisualizer viz;
    viz.addPointCloud (points, "points");
    viz.addPointCloud (normal_points, "normal_points");
    
    viz.addPointCloudNormals<PointXYZ, Normal> (normal_points, normals, 1, 0.1, "normals");
    
    // Give control over to the visualizer
    viz.spin ();
}

void visualize_keypoints (const PointCloud<PointXYZ>::Ptr points,
                          const PointCloud<PointWithScale>::Ptr keypoints)
{
    // Add the points to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points, "points");
    
    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size (); ++i)
    {
        // Get the point data
        const pcl::PointWithScale & p = keypoints->points[i];
        
        // Pick the radius of the sphere *
        float r = 2 * p.scale;
        // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
        //   radius of 2*p.scale is a good illustration of the extent of the keypoint
        
        // Generate a unique string for each sphere
        std::stringstream ss ("keypoint");
        ss << i;
        
        // Add a sphere at the keypoint
        viz.addSphere (p, r, 1.0, 0.0, 0.0, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}

void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores)
{
    // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
    // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
    
    // Create some new point clouds to hold our transformed data
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);
    
    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);
    
    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);
    
    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points_left, "points_left");
    viz.addPointCloud (points_right, "points_right");
    
    // Compute the median correspondence score
    std::vector<float> temp (correspondence_scores);
    std::sort (temp.begin (), temp.end ());
    float median_score = temp[temp.size ()/2];
    
    // Draw lines between the best corresponding points
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
        if (correspondence_scores[i] > median_score)
        {
            continue; // Don't draw weak correspondences
        }
        
        // Get the pair of points
        const pcl::PointWithScale & p_left = keypoints_left->points[i];
        const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];
        
        // Generate a random (bright) color
        double r = (rand() % 100);
        double g = (rand() % 100);
        double b = (rand() % 100);
        double max_channel = std::max (r, std::max (g, b));
        r /= max_channel;
        g /= max_channel;
        b /= max_channel;
        
        // Generate a unique string for each line
        std::stringstream ss ("line");
        ss << i;
        
        // Draw the line
        viz.addLine (p_left, p_right, r, g, b, ss.str ());
    }
    
    // Give control over to the visualizer
    viz.spin ();
}
