#include "Density_Clustering.h"
#include <math.h>
#include <algorithm>
#include <stdlib.h>

Density_Clustering::Density_Clustering( float *_Vectors, int _dim, int _vector_num, int _center_num, float _density_radius, float _density_threshold, float _minimum_dist_threshold, int _expect_cluster_num, int _top_center_num )
{
	Vectors = _Vectors;
	dim = _dim;
	vector_num = _vector_num;
	center_num = _center_num;
	density_radius = _density_radius;
	density_threshold = _density_threshold;
	minimum_dist_threshold = _minimum_dist_threshold;
	expect_cluster_num = _expect_cluster_num;
	top_center_num = _top_center_num;

	this->Initialize();
}

long Density_Clustering::Initialize()
{
	if ( Vectors == NULL || dim <= 0 || vector_num <= 0 || center_num <= 0 || vector_num < center_num )
	{
		initial_sign = -1;
		return -1;
	}

	if ( top_center_num >= center_num )
		top_center_num = -1;

	density_radius_square = density_radius * density_radius; 
	initial_sign = 1;
	return 1;
}

long Density_Clustering::Center_Generation()
{
	if (initial_sign < 0)
		return -1;

	if ( ( Centers = (Center *) calloc ( center_num, sizeof(Center) ) ) == NULL )
		return -1;

	if ( ( Center_Vector_Mem = (float *) calloc ( center_num * dim, sizeof(float) ) ) == NULL )
	{
		free( Centers );
		return -1;
	}

	int mem_iter = 0;
	for (int cen_iter = 0; cen_iter < center_num; cen_iter++ )
	{
		int rand_iter = 0;
		int vector_ID = 0;

		do
		{
			vector_ID = rand() % vector_num;
			for (rand_iter = 0; rand_iter < cen_iter; rand_iter++ )
			{
				if ( vector_ID == Centers[rand_iter].vector_ID )
					break;
			}
		}while ( rand_iter < cen_iter );
				
		Centers[cen_iter].center_ID = cen_iter;
		Centers[cen_iter].vector_ID = vector_ID;
		Centers[cen_iter].Center_Vectors = Center_Vector_Mem + mem_iter;
		Centers[cen_iter].radius = 0;
		Centers[cen_iter].point_num = 0;
		Centers[cen_iter].max_density = -1;
		Centers[cen_iter].min_density = vector_num + 1;
		for (int dim_iter = 0; dim_iter < dim; dim_iter++)
			Centers[cen_iter].Center_Vectors[dim_iter] = Vectors[ vector_ID * dim + dim_iter ];
		mem_iter += dim;
	}

	//for (int i=0; i<center_num; i++)
	//{
	//	printf("%d %d\n", Centers[i].center_ID, Centers[i].vector_ID);
	//}

	if ( ( Vector_CenterID = (int *) calloc ( vector_num, sizeof(int) ) ) == NULL )
	//if ((Vector_CenterID = new int[vector_num]) == NULL)
	{
		free( Centers );
		free( Center_Vector_Mem );
		return -1;
	}

	int total_num = vector_num * dim;
	for ( int vec_iter = 0; vec_iter < total_num; vec_iter += dim )
	{
		float NN_dist = 0;
		int cen_ID = 0;
		for (int dim_iter = 0; dim_iter < dim; dim_iter++)
		{
			float dist_diff = Centers[0].Center_Vectors[dim_iter] - Vectors[vec_iter+dim_iter];
			NN_dist +=  dist_diff * dist_diff;
		}
		
		for (int cen_iter = 1; cen_iter < center_num; cen_iter++)
		{
			float dist = 0;
			for (int dim_iter = 0; dim_iter < dim; dim_iter++)
			{
				float dist_diff = Centers[cen_iter].Center_Vectors[dim_iter] - Vectors[vec_iter+dim_iter];
				dist +=  dist_diff * dist_diff;
				if (dist > NN_dist)
					break;
			}

			if ( dist < NN_dist )
			{
				NN_dist = dist;
				cen_ID = cen_iter;
			}
		}

		Centers[cen_ID].point_num++;
		if ( Centers[cen_ID].radius < NN_dist )
			Centers[cen_ID].radius = NN_dist;
		
		Vector_CenterID[ vec_iter / dim ] = cen_ID;
	}

	for (int cen_iter = 0; cen_iter < center_num; cen_iter++)
		Centers[cen_iter].radius = sqrt( Centers[cen_iter].radius );

	qsort( Centers, center_num, sizeof(Center), Compare );
	std::vector<int> Center_IDs( center_num );
	int match_ID=0;
	for (int cen_iter = 0; cen_iter<center_num; cen_iter++)
	{		
		for (match_ID=0; match_ID<center_num; match_ID++)
		{
			if ( Centers[match_ID].center_ID == cen_iter )
				break;
		}
		Center_IDs[cen_iter] = match_ID;
	}

	for (int vec_iter = 0; vec_iter < vector_num; vec_iter++)
		Vector_CenterID[ vec_iter ] = Center_IDs[ Vector_CenterID[ vec_iter ] ];


	//for (int i=0; i<vector_num; i++)
	//{
	//	printf("%d\n",Vector_CenterID[i]);
	//}

	return 1;
}

long Density_Clustering::Vector_Search( int vector_ID, int center_ID )
{
	int val_ID = vector_ID * dim;
	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		if ( Vector_CenterID[vec_iter] == center_ID )
		{
			int val_iter = vec_iter * dim;
			float dist = 0;
			for (int dim_iter = 0; dim_iter < dim; dim_iter++)
			{
				float dist_diff = Vectors[val_iter+dim_iter] - Vectors[val_ID+dim_iter];
				dist +=  dist_diff * dist_diff;
				if ( dist > density_radius_square )
					break;
			}
			if ( dist < density_radius_square )
				Densities[vector_ID]++;
		}
	}

	return 1;
}

long Density_Clustering::Density_Calculation()
{
	if (initial_sign < 0)
		return -1;

	if ( ( Densities = (float *) calloc ( vector_num, sizeof(float) ) ) == NULL )
		return -1;

	int val_iter = 0;
	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		Densities[vec_iter] = 0;
		for (int cen_iter = 0; cen_iter < center_num; cen_iter++)
		{
			float dist = 0;			
			for (int dim_iter = 0; dim_iter < dim; dim_iter++)
			{
				float dist_diff = Centers[cen_iter].Center_Vectors[dim_iter] - Vectors[val_iter+dim_iter];
				dist +=  dist_diff * dist_diff;
			}
			dist = sqrt(dist);

			if ( density_radius < Centers[cen_iter].radius )
			{
				if ( dist < Centers[cen_iter].radius + density_radius )
					Vector_Search( vec_iter, cen_iter );
			}
			else
			{
				if ( dist < density_radius - Centers[cen_iter].radius )
					Densities[vec_iter] += Centers[cen_iter].point_num;
				else if ( dist < density_radius + Centers[cen_iter].radius )
					Vector_Search( vec_iter, cen_iter );
			}
		}

		val_iter += dim;
	}

	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		if ( Densities[vec_iter] > Centers[ Vector_CenterID[vec_iter] ].max_density )
			Centers[ Vector_CenterID[vec_iter] ].max_density = Densities[vec_iter];
		if ( Densities[vec_iter] < Centers[ Vector_CenterID[vec_iter] ].min_density )
			Centers[ Vector_CenterID[vec_iter] ].min_density = Densities[vec_iter];
	}
	return 1;
}

long Density_Clustering::Density_Calculation( int center_threshold )
{
	if (initial_sign < 0)
		return -1;

	if ( ( Densities = (float *) calloc ( vector_num, sizeof(float) ) ) == NULL )
		return -1;

	int val_iter = 0;
	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		Densities[vec_iter] = 0;
		if ( Vector_CenterID[vec_iter] < center_threshold )
		{
			for (int cen_iter = 0; cen_iter < center_num; cen_iter++)
			{
				float dist = 0;			
				for (int dim_iter = 0; dim_iter < dim; dim_iter++)
				{
					float dist_diff = Centers[cen_iter].Center_Vectors[dim_iter] - Vectors[val_iter+dim_iter];
					dist +=  dist_diff * dist_diff;
				}
				dist = sqrt(dist);

				if ( density_radius < Centers[cen_iter].radius )
				{
					if ( dist < Centers[cen_iter].radius + density_radius )
						Vector_Search( vec_iter, cen_iter );
				}
				else
				{
					if ( dist < density_radius - Centers[cen_iter].radius )
						Densities[vec_iter] += Centers[cen_iter].point_num;
					else if ( dist < density_radius + Centers[cen_iter].radius )
						Vector_Search( vec_iter, cen_iter );
				}
			}
		}

		val_iter += dim;
	}

	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		if ( Densities[vec_iter] > Centers[ Vector_CenterID[vec_iter] ].max_density )
			Centers[ Vector_CenterID[vec_iter] ].max_density = Densities[vec_iter];
		if ( Densities[vec_iter] < Centers[ Vector_CenterID[vec_iter] ].min_density )
			Centers[ Vector_CenterID[vec_iter] ].min_density = Densities[vec_iter];
	}
	return 1;
}

long Density_Clustering::Density_Search( int vector_ID, int center_ID )
{
	int val_ID = vector_ID * dim;
	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		if ( Vector_CenterID[vec_iter] == center_ID && Densities[vector_ID] < Densities[vec_iter] )
		{
			int val_iter = vec_iter * dim;
			float dist = 0;
			for (int dim_iter = 0; dim_iter < dim; dim_iter++)
			{
				float dist_diff = Vectors[val_iter+dim_iter] - Vectors[val_ID+dim_iter];
				dist +=  dist_diff * dist_diff;
			}

			if ( dist < Minimum_Dist[vector_ID] )
				Minimum_Dist[vector_ID] = dist;
		}
	}

	return 1;
}

long Density_Clustering::Minimum_Dist_Calculation()
{
	if (initial_sign < 0)
		return -1;

	if ( ( Minimum_Dist = (float *) calloc ( vector_num, sizeof(float) ) ) == NULL )
		return -1;

	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		Minimum_Dist[vec_iter] = 0;
		if ( Densities[vec_iter] > density_threshold )
		{
			Minimum_Dist[vec_iter] = FLT_MAX;
			for (int cen_iter = 0; cen_iter < center_num; cen_iter++)
			{
				if ( Densities[vec_iter] < Centers[cen_iter].max_density )
					Density_Search( vec_iter, cen_iter );
			}
			Minimum_Dist[vec_iter] = sqrt( Minimum_Dist[vec_iter] );
		}
	}

	return 1;
}

long Density_Clustering::Cluster_Generation()
{
	Clusters.resize(0);	

	return 1;
}

long Density_Clustering::Single_Cluster()
{
	Clusters.resize(0);	
	Clusters.push_back(0);
	Cluster_Minimum_Dists.resize(0);
	Cluster_Densities.resize(0);
	Cluster_Densities.push_back( Densities[0] );
	
	for ( int vec_iter = 0; vec_iter < vector_num; vec_iter ++ )
	{
		if ( Densities[ Clusters[0] ] < Densities[vec_iter] )
		{
			Clusters[0] = vec_iter;
			Cluster_Densities[0] = Densities[vec_iter];
		}
	}

	return 1;
}

long Density_Clustering::Process()
{
	if (initial_sign < 0)
		return -1;

	if ( expect_cluster_num <= 1 )
	{
		if ( top_center_num > 0 )
		{
			if ( this->Center_Generation() < 0 )
				return -1;
			if ( this->Density_Calculation( top_center_num ) < 0 )
			{
				free( Center_Vector_Mem );
				free( Centers );
				free( Vector_CenterID );
				return -1;
			}
			Single_Cluster();
			free( Center_Vector_Mem );
			free( Centers );
			free( Vector_CenterID );
			free( Densities );
		}
		else
		{
			if ( this->Center_Generation() < 0 )
				return -1;
			if ( this->Density_Calculation() < 0 )
			{
				free( Center_Vector_Mem );
				free( Centers );
				free( Vector_CenterID );
				return -1;
			}
			Single_Cluster();
			free( Center_Vector_Mem );
			free( Centers );
			free( Vector_CenterID );
			free( Densities );
		}
	}
	else
	{

	}

	return 1;
}