#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include <math.h>
//#include "local_gicp.h"
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

const float voxel_leaf_size = 0.05;
const float icp_epsilon = 1e-16;
const float icp_max_distance = 0.1;
const int icp_max_ite = 1;
const int MAXITE = 20;
float compute_error (CloudRGBPtr target, CloudRGBPtr source, Eigen::Matrix4f init);
int
main (int argc, char ** argv)
{ 
  /*------set up visualizer ---------*/
  //set up an visualizer with 3 viewport
  //2 view ports for display init and result
  //1 view port for total result
  pcl::visualization::PCLVisualizer::Ptr visualizer (new pcl::visualization::PCLVisualizer(argc, argv, "bla bla bla"));
  int init_viewport = 0;
  int match_viewport = 1;
  visualizer->createViewPort (0, 0, 0.5, 1, init_viewport);
  visualizer->createViewPort (0.5, 0, 1, 1, match_viewport);
  
  visualizer->addCoordinateSystem (0.4, "init_coord", init_viewport);
  visualizer->addCoordinateSystem (0.4, "match_coord", match_viewport);


  //custom the keypress event
  vtkSmartPointer<vtkInteractorStyleSwitch> style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
  style->SetCurrentStyleToTrackballCamera ();

  visualizer->setupInteractor(visualizer->getRenderWindow()->GetInteractor(), visualizer->getRenderWindow(), style);
  
  /*-----------end set up visualizer----*/
  
  
  /*-----------registration-------------*/
  CloudRGBPtr source (new CloudRGB); 
  CloudRGBPtr target (new CloudRGB);
  pcl::io::loadPCDFile (argv[1], *target);
  pcl::io::loadPCDFile (argv[2], *source);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*target, *target, indices);
  pcl::removeNaNFromPointCloud (*source, *source, indices);
  printf ("target size = %zu; source size = %zu\n", target->points.size (), source->points.size ());
  CloudNormalPtr source_normal (new CloudNormal);
  CloudNormalPtr target_normal (new CloudNormal);

  /*       preprocessing        */
  //esitmate normal

  pcl::NormalEstimation<PointRGB, PointNormal> ne;
  pcl::search::KdTree<PointRGB>::Ptr kd_tree (new pcl::search::KdTree<PointRGB> ());
  ne.setSearchMethod (kd_tree);
  ne.setRadiusSearch (0.03);

  ne.setInputCloud (source);
  ne.compute(*source_normal);
  //  *target_normal = *source_normal;
  ne.setInputCloud (target);
  ne.compute(*target_normal);
  //setup filter
  CloudRGBPtr source_sampled (new CloudRGB);
  CloudRGBPtr target_sampled (new CloudRGB);

  pcl::NormalSpaceSampling<PointRGB, PointNormal> filter;
  filter.setSample(2000);
  filter.setBins (5, 5, 5);
  //filter point cloud  
  CloudRGBPtr temp ( new CloudRGB);
  filter.setInputCloud (source);
  filter.setNormals (source_normal);
  filter.filter (*source_sampled);
  //*target_sampled = *source_sampled;
  filter.setInputCloud (target);
  filter.setNormals (target_normal);
  filter.filter (*target_sampled);

  /*        end preprocessing       */
  //init the cloud
  pcl::visualization::PointCloudColorHandlerCustom<PointRGB> target_color (target, 0, 255, 0); 
  pcl::visualization::PointCloudColorHandlerCustom<PointRGB> source_color (source, 255, 0, 0); 
	  
  visualizer->addPointCloud (target, target_color, "target", init_viewport);
  visualizer->addPointCloud (source, source_color, "source", init_viewport);
  //allow user move point cloud
  visualizer->getCloudActorMap ()->at ("source").actor->PickableOn ();
  visualizer->getCloudActorMap ()->at ("target").actor->PickableOff ();
  visualizer->spin ();

  pcl::visualization::CloudActorMapPtr cloud_actor_map = visualizer->getCloudActorMap ();
  vtkSmartPointer<vtkMatrix4x4> init_transform = cloud_actor_map->at("source").viewpoint_transformation_;
  Eigen::Matrix4f init_transform_matrix;
  pcl::visualization::PCLVisualizer::convertToEigenMatrix (init_transform, init_transform_matrix);
  
  //compute error
  std::vector <float> error;
  error.resize (MAXITE + 1);
  error[0] = compute_error (target, source, init_transform_matrix);
  Eigen::Matrix4f temp_transform = init_transform_matrix;

  pcl::GeneralizedIterativeClosestPoint< PointRGB, PointRGB > icp;
  //icp.addCorrespondenceRejector (oto_rejector);
  icp.setMaximumIterations (icp_max_ite);
  icp.setMaxCorrespondenceDistance (icp_max_distance);
  icp.setTransformationEpsilon (icp_epsilon);
  icp.setInputSource (source_sampled);
  icp.setInputTarget (target_sampled);
  for (int i = 1; i<MAXITE; i++)
    {
      icp.align (*source_sampled, temp_transform);
      temp_transform = icp.getFinalTransformation ();
      error[i] = compute_error (target, source, temp_transform);
    }

 
  FILE *fout;
  fout = fopen ("error_result", "w");
  for (int i = 0; i<=MAXITE; i++)
    fprintf (fout, "%f\n", error[i]);
  /*-----------End registration------------*/
  pcl::transformPointCloud (*source, *source, temp_transform);
  visualizer->addPointCloud(target, "match_target", match_viewport);
  visualizer->addPointCloud(source, "match_source", match_viewport);
  visualizer->spin ();
  return 0;
}

float compute_error (CloudRGBPtr target, CloudRGBPtr source, Eigen::Matrix4f transformation)
{
  float temp = 0;
  CloudRGBPtr temp_cloud (new CloudRGB);
  *temp_cloud = *source;
  pcl::transformPointCloud (*temp_cloud, *temp_cloud, transformation);
  for (int i = 0; i<temp_cloud->points.size (); i++)
    {
      temp +=  ( pow(temp_cloud->points[i].x - target->points[i].x, 2) + pow (temp_cloud->points[i].y - target->points[i].y, 2) + pow (temp_cloud->points[i].z - target->points[i].z, 2) );
    }
  return sqrt(temp/temp_cloud->points.size ());
}
