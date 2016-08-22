#include <stdio.h>
#include <iostream>

#include <pcl_func.h>
#include <mlpack_func.h>

clock_t start = clock();

void PAUSE(){
    std::cout << "Press enter to leave" << std::endl;
    fgetc(stdin);
}

void record(std::string name)
{
    std::cout << name << ", Cost: " << float(clock()-start)/CLOCKS_PER_SEC << std::endl;
    start = clock();
}

void temp2(std::string file_name1, std::string file_name2)
{
    // Create some new point clouds to hold our data
    PointCloud<PointXYZ>::Ptr points1 (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr downsampled1 (new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr pointNormals1 (new PointCloud<PointNormal>);
    PointCloud<Normal>::Ptr normals1 (new PointCloud<Normal>);
    PointCloud<PointWithScale>::Ptr keypoints1 (new PointCloud<PointWithScale>);
    PointCloud<PFHSignature125>::Ptr descriptors1 (new PointCloud<PFHSignature125>);
    
    PointCloud<PointXYZ>::Ptr points2 (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr downsampled2 (new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr pointNormals2 (new PointCloud<PointNormal>);
    PointCloud<Normal>::Ptr normals2 (new PointCloud<Normal>);
    PointCloud<PointWithScale>::Ptr keypoints2 (new PointCloud<PointWithScale>);
    PointCloud<PFHSignature125>::Ptr descriptors2 (new PointCloud<PFHSignature125>);
    
    // Load the pair of point clouds
    points1 = loadCloud(file_name1);
    points2 = loadCloud(file_name2);
    
    // Downsample the cloud
    const float voxel_grid_leaf_size = 0.1;
    downsample (points1, voxel_grid_leaf_size, downsampled1);
    downsample (points2, voxel_grid_leaf_size, downsampled2);
    
    // Compute surface normals
    const float normal_radius = 0.1;
    compute_surface_normals (downsampled1, normal_radius, pointNormals1);
    compute_surface_normals (downsampled2, normal_radius, pointNormals2);
    normals1 = ParseNormal(pointNormals1);
    normals2 = ParseNormal(pointNormals2);
    
    // Compute keypoints
    const float min_scale = 0.03;
    const int nr_octaves = 3;
    const int nr_octaves_per_scale = 4;
    const float min_contrast = 0.010f;
    detect_keypoints (pointNormals1, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints1);
    detect_keypoints (pointNormals2, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints2);
    
    // Compute PFH features
    const float feature_radius = 0.6f;
    compute_PFH_features (downsampled1, normals1, feature_radius, descriptors1, keypoints1);
    compute_PFH_features (downsampled2, normals2, feature_radius, descriptors2, keypoints2);
    
    // Find feature correspondences
    std::vector<int> correspondences;
    std::vector<float> correspondence_scores;
    find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
    
    // Print out ( number of keypoints / number of points )
    std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
              << "out of " << downsampled1->size () << " total points." << std::endl;
    std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
              << "out of " << downsampled2->size () << " total points." << std::endl;
    
    // Visualize the two point clouds and their feature correspondences
    visualize_correspondences (points1, keypoints1, points2, keypoints2, correspondences, correspondence_scores);
}

void view_keypoints(std::string file_name){
    PointCloud<PointXYZ>::Ptr points = loadCloud(file_name);
    record("Load file");
    
    PointCloud<PointXYZ>::Ptr downsampled (new PointCloud<PointXYZ>);
    const float voxel_grid_leaf_size = 0.06;
    downsample (points, voxel_grid_leaf_size, downsampled);
    record("Down sampling");
    //downsampled = points;
    
    PointCloud<PointNormal>::Ptr pointNormals (new PointCloud<PointNormal>);
    const float normal_radius = 0.1;
    compute_surface_normals(downsampled, normal_radius, pointNormals);
    record("Compute normals");
    
    PointCloud<Normal>::Ptr normals = ParseNormal(pointNormals);
    record("Parse normals");
    
    PointCloud<PointWithScale>::Ptr keypoints (new PointCloud<PointWithScale>);
    detect_keypoints(pointNormals, 0.03f, 3, 4, 0.010f, keypoints);
    record("Compute keypoints");
    
    //visualize_normals(points, downsampled, normals);
    visualize_keypoints(points, keypoints);
}

arma::mat raw_descr_mat (PointCloud<PointXYZ>::Ptr points){
    PointCloud<PointXYZ>::Ptr RANSAC_points = RANSAC(points);
    
    PointCloud<PointNormal>::Ptr pointNormals (new PointCloud<PointNormal>);
    const float normal_radius = 0.1;
    compute_surface_normals(RANSAC_points, normal_radius, pointNormals);
    PointCloud<Normal>::Ptr normals = ParseNormal(pointNormals);
    record("Compute normals");
    
    PointCloud<PointWithScale>::Ptr keypoints (new PointCloud<PointWithScale>);
    //detect_keypoints(pointNormals, 0.03f, 3, 4, 0.010f, keypoints);
    detect_keypoints(pointNormals, 0.04f, 6, 4, 0.005f, keypoints);
    //visualize_keypoints(points, keypoints);
    record("Compute keypoints");
    
    PointCloud<FPFHSignature33>::Ptr FPFH_features (new PointCloud<FPFHSignature33>);
    compute_FPFH_features(RANSAC_points, normals, 1, FPFH_features, keypoints);
    record("Compute FPFH features");
    
    /*
    PointCloud<PFHSignature125>::Ptr PFH_features (new PointCloud<PFHSignature125>);
    compute_PFH_features(RANSAC_points, normals, 1, PFH_features, keypoints);
    record("Compute PFH features");
    */
        
    return convert_to_mat(FPFH_features);
}

void gen_basis(){
    std::string base_path = "/home/johnny/Documents/Test/hello/Debug/sample/all/crop_filter/";
    //base_path = "/home/johnny/Documents/Test/hello/Debug/sample/all/origin/";
    std::vector<std::string> files = { "台幣1元 反面.pcd", "台幣1元 正面.pcd", "台幣10元 反面.pcd", "台幣10元 正面.pcd",
                                       "台幣50元 反面.pcd", "台幣50元 正面.pcd", "外國幣-1 反面.pcd", "外國幣-1 正面.pcd",
                                       "外國幣- 3反面.pcd", "外國幣- 3正面.pcd", "外國幣- 4反面.pcd", "外國幣- 4正面.pcd" };
    
    arma::mat data;
    //data.load("data_matrix.mat");
    //data.raw_print();
    
    for (int i = 0; i < files.size(); i++){
        std::cout << files[i] << std::endl;
        
        PointCloud<PointXYZ>::Ptr points = loadCloud(base_path+files[i]);
        
        // TODO: Use more accurate method
        if (i == 0)
            data = raw_descr_mat(points);
        else
            data = join_rows(data, raw_descr_mat(points));
    }
    
    data.save("data_matrix.mat");
    
    // Cluster using the Manhattan distance, 100 iterations maximum, saving only
    // the centroids.
    arma::mat centroids; // Cluster centroids.
    kmeans::KMeans<metric::ManhattanDistance> k(100);
    k.Cluster(data, 300, centroids); // 6 clusters.
    
    // With specified inital clusters
    //arma::Row<size_t> assignments; // Cluster assignments.
    //k.Cluster(data, 3, assignments, centroids); // 3 clusters.
    
    centroids.save("centroids.mat");
}

int main(int argc, char **argv)
{       
    //temp2("/home/johnny/Documents/Test/hello/Debug/sample/all/crop_filter/台幣50元 正面.pcd", 
    //          "/home/johnny/Documents/Test/hello/Debug/sample/all/crop_filter/台幣50元 反面_offset.pcd");
    
    std::string base_path = "/home/johnny/Documents/Test/hello/Debug/sample/all/crop_filter/";
    std::vector<std::string> files = { "台幣10元 反面.pcd", "台幣10元 正面.pcd", "台幣1元 反面.pcd", "台幣1元 正面.pcd"
                                       "台幣50元 反面.pcd", "台幣50元 正面.pcd" };
    
    gen_basis();
    
    PAUSE();
    return 0;
}
