#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <getopt.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/gicp.h>
struct option long_opts [] =
  {
    {"voxel_leaf_size", required_argument, 0, 'a'},
    {"icp_epsilon", required_argument, 0, 'b'},
    {"icp_max_distance", required_argument, 0, 'c'},
    {"icp_max_ite", required_argument, 0, 'd'},
    {0, 0, 0, 0}
  };

float voxel_leaf_size;
float icp_epsilon;
float icp_max_distance;
int icp_max_ite;

int decode_switches (int argc, char ** argv);


int
main (int argc, char ** argv)
{
  int opt_ind = decode_switches (argc, argv);
  printf ("%s\n", argv[opt_ind]);
  return 0;
}

int decode_switches (int argc, char ** argv)
{
  int opt_ind = 0;
  while ( (opt_ind = getopt_long (argc, argv, "a:b:c:d:", long_opts, NULL)) != -1)
    {
      switch (opt_ind)
	{
	case 'a':
	  voxel_leaf_size = atof (optarg);
	  printf ("voxel_leaf_size  = %f\n", voxel_leaf_size);
	  break;
	case 'b':
	  icp_epsilon = atof (optarg);
	  printf ("icp_epsilon = %f\n", icp_epsilon);
	  break;
	case 'c':
	  icp_max_distance = atof (optarg);
	  printf ("icp_max_distance = %f\n", icp_max_distance);
	  break;
	case 'd':
	  icp_max_ite = atoi (optarg);
	  printf ("icp_max_ite = %d\n", icp_max_ite);
	  break;
	case '?':
	  break;
	default:
	  break;
	}
    }
  return optind;
}
