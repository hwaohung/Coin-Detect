#include "mlpack_func.h"


void nn_example(){
	// Load the data from data.csv (hard-coded).  Use CLI for simple command-line
	// parameter handling.
	arma::mat data;
	data::Load("data.csv", data, true);
	// Use templates to specify that we want a NeighborSearch object which uses
	// the Manhattan distance.
	KNN a(data);
	// The matrices we will store output in.
	arma::Mat<size_t> neighbors;
	arma::mat distances;
	a.Search(5, neighbors, distances);

	for (size_t i = 0; i < neighbors.n_elem; ++i)
	{
		std::cout << "Nearest neighbor of point " << i << " is point "
			<< neighbors[i] << " and the distance is " << distances[i] << ".\n";
	}
}

void run_kmeans(){
	arma::mat data; // Dataset we want to run K-Means on.
	data.set_size(10, 100);
	arma::mat centroids; // Cluster centroids.

	for (int r = 0; r < 100; r++){
		for (int c = 0; c < 10; c++){
			data(r, c) = ((rand() % 100) +1) / 100.0;
		}
	}

	// Cluster using the Manhattan distance, 100 iterations maximum, saving only
	// the centroids.
	kmeans::KMeans<metric::ManhattanDistance> k(100);
	k.Cluster(data, 6, centroids); // 6 clusters.
	
	// With specified inital clusters
	//arma::Row<size_t> assignments; // Cluster assignments.
	//k.Cluster(data, 3, assignments, centroids); // 3 clusters.
	
	std::cout << centroids.size() << "\n";
	std::cout << size(centroids) << "\n";
	
	std::cout << centroids.n_cols << "\n";
	std::cout << centroids.n_rows << "\n";
}