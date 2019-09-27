#pragma once

#include <pcl/pcl_macros.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
class FeatureCloud
{
public:
	// A bit of shorthand
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	FeatureCloud() :
		search_method_xyz_(new SearchMethod),
		normal_radius_(0.02f),
		feature_radius_(0.02f)
	{}

	~FeatureCloud() {}

	// Process the given cloud
	void setInputCloud(PointCloud::Ptr xyz);
	

	// Load and process the cloud in the given PCD file
	void loadInputCloud(const std::string &pcd_file);
	

	// Get a pointer to the cloud 3D points
	PointCloud::Ptr getPointCloud() const;
	

	// Get a pointer to the cloud of 3D surface normals
	SurfaceNormals::Ptr getSurfaceNormals() const;
	

	// Get a pointer to the cloud of feature descriptors
	LocalFeatures::Ptr getLocalFeatures() const;
	

protected:
	// Compute the surface normals and local features
	void processInput();
	

	// Compute the surface normals
	void computeSurfaceNormals();
	

	// Compute the local feature descriptors
	void computeLocalFeatures();
	

private:
	// Point cloud data
	PointCloud::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

	// Parameters
	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment
{
public:
	std::map<std::string, std::vector<FeatureCloud>> templates_;
	// A struct for storing alignment results
	struct Result
	{
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		//PCL_MAKE_ALIGNED_OPERATOR_NEW
	};

	TemplateAlignment() :
		min_sample_distance_(0.05f),
		max_correspondence_distance_(0.01f*0.01f),
		nr_iterations_(500)
	{
		// Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
		sac_ia_.setMinSampleDistance(min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
		sac_ia_.setMaximumIterations(nr_iterations_);
	}

	~TemplateAlignment() {}

	// Set the given cloud as the target to which the templates will be aligned
	void setTargetCloud(FeatureCloud &target_cloud);
	// Add the given cloud to the list of template clouds
	void addTemplateCloud(FeatureCloud &template_cloud, std::string template_group);
	
	// Align the given template cloud to the target specified by setTargetCloud ()
	void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result);
	

	// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
	void alignAllInGroup(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results, std::string template_group);

	// Align all of template clouds to the target cloud to find the one with best alignment score
	int findBestAlignmentInGroup(TemplateAlignment::Result &result, std::string template_group);

private:
	// A list of template clouds and the target to which they will be aligned; could be generalized in the future, for now group into gripper templates and object templates
	

	FeatureCloud target_;

	// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_;
	float max_correspondence_distance_;
	int nr_iterations_;
};