#pragma once
#include <RobotRaconteur.h>
#include "robotraconteur_generated.h"
#include <stdio.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/centroid.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <boost/enable_shared_from_this.hpp>
//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/uniform_sampling.h>
//#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/ia_ransac.h>
//#include <Eigen/Core>
//#include <pcl/point_cloud.h>
#include "Template.h"
#include <chrono>



using namespace std;
using namespace std::chrono;
using namespace boost;
using namespace RobotRaconteur;
//using namespace ::sensors::kinect2;

class PCL_RR: public boost::enable_shared_from_this<PCL_RR>
{
	sensors::kinect2::KinectPtr kinect_interface;
public:
	PCL_RR(string data_address);
	void StartStreaming();
	void StopStreaming();
	void DisplayPointCloud();
	void get_kinect_data();
	void get_image();
	pcl::PointCloud<pcl::PointXYZ>::Ptr FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	vector<pcl::PointIndices> EuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	
	//void LoadFeatureModels();
	void FindFaces(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointXYZ FindCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud); 

	void LoadTemplates(std::vector<std::string> templates, std::string template_group);
	pcl::PointCloud<pcl::PointXYZ>::Ptr TemplateAlign(std::string template_group);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	sensors::kinect2::ImagePtr image;
	pcl::ModelCoefficients::Ptr conveyor_coefficients;
	//transform point cloud function
	//moment of inertia functions
	//std::vector<FeatureCloud> object_templates;
	TemplateAlignment template_align;

private:
	bool streaming;
	bool displaying;
	TimerPtr streaming_timer;
	
	

};

extern boost::recursive_mutex global_lock;