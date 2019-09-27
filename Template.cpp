#include "Template.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

void FeatureCloud::setInputCloud(PointCloud::Ptr xyz) {
	xyz_ = xyz;
	processInput();
}

void FeatureCloud::loadInputCloud(const std::string &pcd_file)
{
	xyz_ = PointCloud::Ptr(new PointCloud);
	pcl::io::loadPCDFile(pcd_file, *xyz_);
	processInput();
}
PointCloud::Ptr FeatureCloud::getPointCloud() const{
	return (xyz_);
}
SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals() const {
	return (normals_);
}
LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const
{
	return (features_);
}
void FeatureCloud::processInput() {
		computeSurfaceNormals();
		computeLocalFeatures();
}

void FeatureCloud::computeSurfaceNormals() {
	normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud(xyz_);
	norm_est.setSearchMethod(search_method_xyz_);
	norm_est.setRadiusSearch(normal_radius_);
	norm_est.compute(*normals_);
}

void FeatureCloud::computeLocalFeatures() {
	features_ = LocalFeatures::Ptr(new LocalFeatures);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(xyz_);
	fpfh_est.setInputNormals(normals_);
	fpfh_est.setSearchMethod(search_method_xyz_);
	fpfh_est.setRadiusSearch(feature_radius_);
	fpfh_est.compute(*features_);
}

void TemplateAlignment::setTargetCloud(FeatureCloud &target_cloud)
{
	target_ = target_cloud;
	sac_ia_.setInputTarget(target_cloud.getPointCloud());
	sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
}


void TemplateAlignment::addTemplateCloud(FeatureCloud &template_cloud, std::string template_group)
{
	templates_[template_group].push_back(template_cloud);
}


void TemplateAlignment::align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
{
	sac_ia_.setInputCloud(template_cloud.getPointCloud());
	sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

	pcl::PointCloud<pcl::PointXYZ> registration_output;
	sac_ia_.align(registration_output);

	result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
	result.final_transformation = sac_ia_.getFinalTransformation();
}

void TemplateAlignment::alignAllInGroup(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results, std::string template_group)
{
	std::vector<FeatureCloud> group = templates_[template_group];
	results.resize(group.size());
	for (size_t i = 0; i < group.size(); ++i)
	{
		align(group[i], results[i]);
	}
}

int TemplateAlignment::findBestAlignmentInGroup(TemplateAlignment::Result &result, std::string template_group)
{
	// Align all of the templates to the target cloud
	std::vector<Result, Eigen::aligned_allocator<Result> > results;
	alignAllInGroup(results, template_group);

	// Find the template with the best (lowest) fitness score
	float lowest_score = std::numeric_limits<float>::infinity();
	int best_template = 0;
	for (size_t i = 0; i < results.size(); ++i)
	{
		const Result &r = results[i];
		if (r.fitness_score < lowest_score)
		{
			lowest_score = r.fitness_score;
			best_template = (int)i;
		}
	}

	// Output the best alignment
	result = results[best_template];
	return (best_template);
}
