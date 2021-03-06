#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <getopt.h>
#include <string.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/PointIndices.h>

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud <PointXYZ> CloudXYZ;
typedef pcl::PointCloud <PointRGB> CloudRGB;
typedef pcl::PointCloud <PointNormal> CloudNormal;
typedef CloudXYZ::Ptr CloudXYZPtr;
typedef CloudRGB::Ptr CloudRGBPtr;
typedef CloudNormal::Ptr CloudNormalPtr;

//enum ICPTYPE = {GENERIALIZED_ICP = 0, DEFAULT_ICP = 1, NORMAL_ICP = 2};
struct option long_opts [] =
  {
    {"voxel_leaf_size", required_argument, 0, 'a'},
    {"icp_epsilon", required_argument, 0, 'b'},
    {"icp_max_distance", required_argument, 0, 'c'},
    {"icp_max_ite", required_argument, 0, 'd'},
    {"icp_type", required_argument, 0, 'e'},
    {0, 0, 0, 0}
  };
const float voxel_leaf_size = 0.05;
const float icp_epsilon = 1e-16;
const float icp_max_distance = 0.1;
const int icp_max_ite = 2000;
//ICPTYPE icp_type;
int decode_switches (int argc, char ** argv);
Eigen::Matrix4f align (CloudRGBPtr source,
		       CloudRGBPtr target,
		       double epsilon,
		       double max_ite,
		       double max_distance,
		       Eigen::Matrix4f init_transform);

void keyboard_event (vtkObject* caller, long unsigned int eventId, void *client_data, void* call_data);

int
main (int argc, char ** argv)
{
  //confirm to merge point cloud to global data
  bool merge_confirm = false;
  /*----------load data--------------------*/
  //all input point clouds will be stored here
  std::vector <CloudRGBPtr> cloud_data;

  //read argument, opt_ind is the index of point cloud file in argv
  int opt_ind = decode_switches (argc, argv);

  //check for valid number of point clouds
  if ((argc - opt_ind) < 2)
    {
      fprintf (stderr, "please input more than 1 point cloud\n");
      exit (1);
    }
  
  //read point cloud
  //temp indices for remove NAN point;
  std::vector <int> indices;
  for (int i = opt_ind; i<argc; i++)
    {
      CloudRGBPtr temp (new CloudRGB);;
      pcl::io::loadPCDFile (argv[i], *temp);
      
      //remove NAN point 
      pcl::removeNaNFromPointCloud(*temp,*temp, indices);

      cloud_data.push_back (temp);
      printf ("%zu\n",cloud_data[i-opt_ind]->points.size ());
    }
  printf (" read %zu clouds \n", cloud_data.size ());
  /*----------end load data----------------*/

  
  /*------set up visualizer ---------*/
  //set up an visualizer with 3 viewport
  //2 view ports for display init and result
  //1 view port for total result
  pcl::visualization::PCLVisualizer::Ptr visualizer (new pcl::visualization::PCLVisualizer(argc, argv, "bla bla bla"));
  int init_viewport = 0;
  int match_viewport = 1;
  int total_viewport = 2;
  visualizer->createViewPort (0, 0.5, 0.5, 1, init_viewport);
  visualizer->createViewPort (0.5, 0.5, 1, 1, match_viewport);
  visualizer->createViewPort (0, 0, 1, 0.5, total_viewport);
  visualizer->addCoordinateSystem (0.4, "init_coord", init_viewport);
  visualizer->addCoordinateSystem (0.4, "match_coord", match_viewport);
  visualizer->addCoordinateSystem (0.4, "total_coord", total_viewport);

  //custom the keypress event
  vtkSmartPointer<vtkInteractorStyleSwitch> style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
  style->SetCurrentStyleToTrackballCamera ();

  visualizer->setupInteractor(visualizer->getRenderWindow()->GetInteractor(), visualizer->getRenderWindow(), style);

  //custom keyboard event to confirm merge to total point cloud
  vtkSmartPointer<vtkCallbackCommand> custom_key_call_back =
    vtkSmartPointer<vtkCallbackCommand>::New ();
  custom_key_call_back->SetCallback (keyboard_event);
  custom_key_call_back->SetClientData (&merge_confirm);
  visualizer->getRenderWindow ()->GetInteractor ()->AddObserver (vtkCommand::KeyPressEvent, custom_key_call_back);
  
  /*-----------end set up visualizer----*/
  
  
  /*-----------registration-------------*/
  CloudRGBPtr source (new CloudRGB); 
  CloudRGBPtr target (new CloudRGB);
  CloudNormalPtr source_normal (new CloudNormal);
  CloudNormalPtr target_normal (new CloudNormal);
  //store result transformation from i+1 to i, all will transform to coordinate of first point cloud
  CloudRGBPtr local_result (new CloudRGB);
  CloudRGBPtr final_cloud (new CloudRGB);
  std::vector <Eigen::Matrix4f> transformation;
  transformation.resize (cloud_data.size ()-1);
  bool restart;

  /*       preprocessing        */
  //esitmate normal
  std::vector<CloudNormalPtr> cloud_normals;
  cloud_normals.resize (cloud_data.size ());
  pcl::NormalEstimation<PointRGB, PointNormal> ne;
  pcl::search::KdTree<PointRGB>::Ptr kd_tree (new pcl::search::KdTree<PointRGB> ());
  ne.setSearchMethod (kd_tree);
  ne.setRadiusSearch (0.03);
  for (int i = 0; i<cloud_data.size (); i++)
    {
      printf ("estimate normal cloud %d\n", i);
      CloudNormalPtr temp ( new CloudNormal);
      ne.setInputCloud (cloud_data[i]);
      ne.compute(*temp);
      cloud_normals[i] = temp;
    }
  //setup filter
  std::vector<CloudRGBPtr> cloud_sampled;
  cloud_sampled.resize (cloud_data.size ());
  pcl::NormalSpaceSampling<PointRGB, PointNormal> filter;
  filter.setSample(2000);
  filter.setBins (5, 5, 5);
  //filter point cloud
  for (int i = 0; i < cloud_data.size (); i++)
    {
      printf ("filter cloud %d\n", i);
      CloudRGBPtr temp ( new CloudRGB);
      filter.setInputCloud (cloud_data[i]);
      filter.setNormals (cloud_normals[i]);
      filter.filter (*temp);
      cloud_sampled[i] = temp;
    }
  /*        end preprocessing       */
  
  pcl::VoxelGrid<PointRGB> final_filter;
  final_filter.setLeafSize (0.005, 0.005, 0.005);
  *final_cloud = *cloud_data[0];
  
  pcl::PointIndices filtered_indices;

  for (int i = 0; i<cloud_data.size () - 1; i++)
    {
      restart = true;
      while (restart)
	{
	  printf ("register point cloud %d %d\n", i, i+1);
	  
	  /*------------init by user------------*/
	  
	  //remove current display point clouds in viewport init
	  visualizer->removePointCloud ("source");
	  visualizer->removePointCloud ("target");

	  //add current point cloud for display
	  pcl::visualization::PointCloudColorHandlerCustom<PointRGB> target_color (cloud_data[i], 0, 255, 0); 
	  pcl::visualization::PointCloudColorHandlerCustom<PointRGB> source_color (cloud_data[i+1], 255, 0, 0); 
	
	  visualizer->addPointCloud (cloud_data[i], target_color, "target", init_viewport);
	  visualizer->addPointCloud (cloud_data[i+1], source_color, "source", init_viewport);
	  //allow user move point cloud
	  visualizer->getCloudActorMap ()->at ("source").actor->PickableOn ();
	  visualizer->getCloudActorMap ()->at ("target").actor->PickableOff ();
	  visualizer->spin ();

	  // Get the transformation from viewer after init by user
	  pcl::visualization::CloudActorMapPtr cloud_actor_map = visualizer->getCloudActorMap ();
	  vtkSmartPointer<vtkMatrix4x4> init_transform = cloud_actor_map->at("source").viewpoint_transformation_;

	  Eigen::Matrix4f init_transform_matrix = Eigen::Matrix4f::Identity ();
	  pcl::visualization::PCLVisualizer::convertToEigenMatrix (init_transform, init_transform_matrix);
	  pcl::copyPointCloud (*cloud_sampled[i+1], *source);
	  pcl::copyPointCloud (*cloud_sampled[i], *target);
	  //align using icp
	  pcl::transformPointCloud (*source, *source, init_transform_matrix);
	  transformation[i] = align (source,
				     target,
				     icp_epsilon,
				     icp_max_ite,
				     icp_max_distance,
				     Eigen::Matrix4f::Identity ());
	  std::cout << transformation[i] << "\n";
	  pcl::copyPointCloud (*cloud_data[i+1], *source);
	  pcl::copyPointCloud (*cloud_data[i], *target);
	  pcl::transformPointCloud (*source, *source, init_transform_matrix);
	  //pcl::transformPointCloud (*source, *source, transformation[i]);
	  pcl::transformPointCloud (*source, *source, transformation[i]);
	  //pcl::visualization::PointCloudColorHandlerCustom<PointRGB> local_target_color (source, 0, 255, 0); 
	  //pcl::visualization::PointCloudColorHandlerCustom<PointRGB> local_source_color (target, 255, 0, 0);
	  visualizer->removePointCloud ("local_source");
	  visualizer->removePointCloud ("local_target");
	  visualizer->addPointCloud (source, "local_source", match_viewport);
	  visualizer->addPointCloud (target, "local_target", match_viewport);
	  visualizer->spin ();
	  /*--------end align with icp---------*/
	  transformation[i] = transformation[i] * init_transform_matrix;
	  
	  if (merge_confirm)
	    {
	      restart = false;
	      CloudRGBPtr temp (new CloudRGB);
	      for (int j = i-1; j>= 0; j--)
		{
		  pcl::transformPointCloud (*source, *source, transformation[j]);
		}
	      *final_cloud += *source;
	      printf ("final before filter %zu\n", final_cloud->points.size ());
	      final_filter.setInputCloud (final_cloud);
	      final_filter.filter (*temp);
	      *final_cloud = *temp;
	      printf ("final after filter %zu\n", final_cloud->points.size ());
	    }
	  visualizer->removePointCloud ("final_result");
	  visualizer->addPointCloud (final_cloud, "final_result", total_viewport);
	  visualizer->spin();
	}
    }
  pcl::io::savePCDFileASCII ("final.pcd", *final_cloud);
  /*-----------End registration------------*/
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
	  //	  voxel_leaf_size = atof (optarg);
	  printf ("voxel_leaf_size  = %f\n", voxel_leaf_size);
	  break;
	case 'b':
	  //icp_epsilon = atof (optarg);
	  printf ("icp_epsilon = %f\n", icp_epsilon);
	  break;
	case 'c':
	  //icp_max_distance = atof (optarg);
	  printf ("icp_max_distance = %f\n", icp_max_distance);
	  break;
	case 'd':
	  //icp_max_ite = atoi (optarg);
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

Eigen::Matrix4f
align (CloudRGBPtr source,
       CloudRGBPtr target,
       double epsilon,
       double max_ite,
       double max_distance,
       Eigen::Matrix4f init_transform)
{
  printf ("aligning point cloud\n");
  //pcl::registration::CorrespondenceRejectorOneToOne::Ptr oto_rejector (new pcl::registration::CorrespondenceRejectorOneToOne);
  Eigen::Matrix4f result_transformation;
  pcl::GeneralizedIterativeClosestPoint< PointRGB, PointRGB > icp;
  //icp.addCorrespondenceRejector (oto_rejector);
  icp.setMaximumIterations (max_ite);
  icp.setMaxCorrespondenceDistance (max_distance);
  icp.setTransformationEpsilon (epsilon);
  icp.setInputSource (source);
  icp.setInputTarget (target);
  
  CloudRGB temp;
  icp.align (temp, init_transform);
  return icp.getFinalTransformation ();
}

void keyboard_event (vtkObject* caller, long unsigned int eventId, void *client_data, void* call_data)
{
  vtkRenderWindowInteractor *ren = static_cast<vtkRenderWindowInteractor*>(caller);
  bool *merge_confirm = (bool *)client_data;

  if (strcmp (ren->GetKeySym(), "y") == 0)
    {
      printf ("key y pressed\n");
      *merge_confirm = true;
    }
  if (strcmp (ren->GetKeySym(), "n") == 0)
    {
      printf ("key n pressed\n");
      *merge_confirm = false;
    }
  printf ("keyboard callback with %s\n", ren->GetKeySym());
}
