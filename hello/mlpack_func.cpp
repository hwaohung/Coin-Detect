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