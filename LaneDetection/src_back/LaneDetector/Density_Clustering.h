#ifndef _DENSITY_CLUSTERING_H_
#define _DENSITY_CLUSTERING_H_

#include <vector>

class Density_Clustering
{
protected:
	typedef struct Center
	{
		float *Center_Vectors;
		float radius;
		float max_density;
		float min_density;
		float point_num;
		int vector_ID;
		int center_ID;
	};
	typedef struct Center_Rank
	{
		int ID;
		float point_num;
	};
	static int Compare(const void *a, const void *b) 
	{
		return (*(Center *)b).point_num - (*(Center *)a).point_num;
	}
public:	
	float *Vectors;
	int center_num;
	int dim;
	int vector_num;		
	
	float density_threshold;
	float density_radius;
	float minimum_dist_threshold;
	int expect_cluster_num;
	int top_center_num;
	std::vector<int> Clusters;
	std::vector<float> Cluster_Densities;
	std::vector<float> Cluster_Minimum_Dists;

	Density_Clustering( float *_Vectors, int _dim, int _vector_num, int _center_num, float _density_radius, float _density_threshold, float _minimum_dist_threshold, int _expect_cluster_num, int _top_center_num );
	long Initialize();
	long Process();
private:	
	long Center_Generation();
	long Density_Calculation();
	long Density_Calculation( int center_threshold );
	long Vector_Search( int vector_ID, int center_ID );
	long Minimum_Dist_Calculation();
	long Density_Search( int vector_ID, int center_ID );
	long Cluster_Generation();	
	long Single_Cluster();
	Center *Centers;
	float *Center_Vector_Mem;
	int initial_sign;
	int *Vector_CenterID;
	float density_radius_square;
	float *Densities;
	float *Minimum_Dist;
};


#endif