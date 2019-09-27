
#include "PCLRR.h"
#include <windows.h>



PCL_RR::PCL_RR(string data_address)
{
	
	this->kinect_interface = rr_cast<sensors::kinect2::Kinect>(RobotRaconteurNode::s()->ConnectService(string("tcp://localhost:8888/sensors.kinect2/Kinect2")));
	std::cout << "initialized" << std::endl;
	//sensors::kinect2::KinectMultiSourcePtr sensors_enabled(new sensors::kinect2::KinectMultiSource);
	sensors::kinect2::KinectMultiSourcePtr sensors_enabled = this->kinect_interface->SensorsEnabled();
	sensors_enabled->Depth = 1;
	sensors_enabled->Infrared = 1;
	std::cout << "initialized" << std::endl;
	uint8_t result = this->kinect_interface->EnableSensors(sensors_enabled);
	if (result == 0) {
		this->kinect_interface->DisableSensors();
		this->kinect_interface->EnableSensors(sensors_enabled);
		std::cout << "restarting" << std::endl;
	}
	std::cout << unsigned(result);
	Sleep(1000);
	//frameptr = kinect_interface->getPointCloud();
	
	
}

void PCL_RR::StartStreaming()
{
	boost::recursive_mutex::scoped_lock lock(global_lock);
	if (streaming_timer) throw InvalidOperationException("Already streaming");
	RR_WEAK_PTR<PCL_RR> weak_this = shared_from_this();
	streaming_timer = RobotRaconteurNode::s()->CreateTimer(boost::posix_time::milliseconds(100),
		[weak_this](TimerEvent ev)
	{
		auto shared_this = weak_this.lock();
		if (!shared_this) return;
		shared_this->get_kinect_data();
	}
	);
	streaming_timer->Start();
}

void PCL_RR::get_kinect_data()
{
	
	sensors::kinect2::PointCloudPtr frameptr = this->kinect_interface->getPointCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//cloud.is_dense(FALSE);
	cloud->width = 512;
	cloud->height = 424;
	
	cloud->is_dense = FALSE;
	//BOOST_FOREACH(frameptr& value, v) {
	
	pcl::PointXYZ point;
	for (int i = 0; i < frameptr->points->size(); i++ ) {
		
		point.x = (*frameptr->points)[i].s.x;
		point.y = (*frameptr->points)[i].s.y;
		point.z = (*frameptr->points)[i].s.z;
		
		cloud->points.push_back(point);
		

	}

	this->cloud = cloud;
}

void PCL_RR::get_image()
{
	sensors::kinect2::ImagePtr pic = this->kinect_interface->getCurrentColorImage();
	this->image = pic;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_RR::FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	
	pass.filter(*cloud_filtered);
	
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.3, 0.3);
	pass.filter(*cloud_filtered);

	//this->cloud = cloud_filtered;
	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_RR::SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr postsegmentedcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	this->conveyor_coefficients = coefficients;
	std::cout << *(this->conveyor_coefficients) << std::endl;
	pcl::ExtractIndices<pcl::PointXYZ> eifilter(true); // Initializing with true will allow us to extract the removed indices
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.setNegative(true);
	eifilter.filter(*postsegmentedcloud);

	// The indices_rem array indexes all points of cloud_in that are not indexed by indices_in
	
	//this->cloud = postsegmentedcloud;
	return postsegmentedcloud;
}

vector<pcl::PointIndices> PCL_RR::EuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr postsegmentedcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud);
	//extract.setIndices(cluster_indices);
	//extract.setNegative(false);
	//extract.filter(postsegmentedcloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
	pcl::PointIndices it = cluster_indices[0];
		for (std::vector<int>::const_iterator pit = it.indices.begin(); pit != it.indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

	//}
	

	this->cloud = cloud_cluster;
	return cluster_indices;
}

double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
/*
void PCL_RR::UseGeometricConsistencyModels(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model)
{
	
	float model_ss_(0.01f);
	float scene_ss_(0.03f);
	
	float descr_rad_(0.02f);
	float cg_size_(0.01f);
	float cg_thresh_(5.0f);
	/*
	//float resolution = static_cast<float> (computeCloudResolution(cloud));
	//if (resolution != 0.0f)
	{
		model_ss_ *= resolution;
		scene_ss_ *= resolution;
		
		descr_rad_ *= resolution;
		cg_size_ *= resolution;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);


	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(cloud);
	norm_est.compute(*scene_normals);

	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
	descr_est.setRadiusSearch(descr_rad_);
	
	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(cloud);
	descr_est.compute(*scene_descriptors);
	
	//
	//  Find Model-Scene Correspondences with KdTree
	//
	

	pcl::KdTreeFLANN<pcl::SHOT352> match_search;
	match_search.setInputCloud(model_descriptors);


	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
	gc_clusterer.setGCSize(cg_size_);
	gc_clusterer.setGCThreshold(cg_thresh_);

	gc_clusterer.setInputCloud(model_keypoints);
	gc_clusterer.setSceneCloud(scene_keypoints);
	gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

	//gc_clusterer.cluster (clustered_corrs);
	gc_clusterer.recognize(rototranslations, clustered_corrs);
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
}
*/
void PCL_RR::FindFaces(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	//seg.segment(*inliers, *coefficients);
}

pcl::PointXYZ PCL_RR::FindCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr)
{
	return pcl::PointXYZ();
}

void PCL_RR::LoadTemplates(std::vector<std::string> templates, std::string template_group)
{
	printf("Loading templates");
	FeatureCloud template_cloud;
	for (vector<string>::const_iterator i = templates.begin(); i != templates.end(); ++i) {
		template_cloud.loadInputCloud(*i);
		this->template_align.addTemplateCloud(template_cloud,template_group);
	}
	printf("Finished Loading templates");
	
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_RR::TemplateAlign(std::string template_group)
{
	printf("attempting to align clouds");
	
	const float voxel_grid_size = 0.005f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud(this->cloud);
	vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
	//vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
	vox_grid.filter(*tempCloud);
	//voxel filter takes about 1128 microseconds
	
	auto start2 = high_resolution_clock::now();
	//this->cloud = tempCloud;
	FeatureCloud target_cloud;
	target_cloud.setInputCloud(tempCloud);
	
	
	this->template_align.setTargetCloud(target_cloud);
	TemplateAlignment::Result best_alignment;
	int best_index = this->template_align.findBestAlignmentInGroup(best_alignment,template_group);
	const FeatureCloud &best_template = this->template_align.templates_[template_group][best_index];

	Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
	// Print the alignment fitness score (values less than 0.00002 are good)
	auto stop2 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(stop2 - start2);
	std::cout << duration2.count();
	printf("Best fitness score: %f\n", best_alignment.fitness_score);
	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
	return transformed_cloud;
	
}



void PCL_RR::DisplayPointCloud() {

	pcl::visualization::CloudViewer cloud_viewer("World Viewer");
	while (!cloud_viewer.wasStopped()){
		cloud_viewer.showCloud(this->cloud);
	}
}



void import_cloud() {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud_cluster_0.pcd", *cloud) == -1) //* load the file
	{

		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		

	}
	

	// Convert to the templated PointCloud
	
	pcl::visualization::CloudViewer cloud_viewer("World Viewer");
	while (!cloud_viewer.wasStopped())
	{
		cloud_viewer.showCloud(cloud);
	}

}
void combine_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	*(cloud) += *(cloud2);

}



int main(int argc, char* argv[])
{
	ClientNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES);
	
	std::string dummy;
	std::string gripper = "gripper";
	std::vector<std::string> gripper_templates{"gripper_1.pcd","gripper_2.pcd","gripper_3.pcd","gripper_4.pcd"};
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
	PCL_RR inter =PCL_RR(" ");
	//pcl::visualization::CloudViewer cloud_viewer("World Viewer");
	inter.LoadTemplates(gripper_templates, gripper);
	//if (pcl::io::loadPCDFile("cloud_cluster_0.pcd", *model) < 0)
	//{
	//std::cout << "Error loading model cloud." << std::endl;
		
	//return (-1);
	//}
	
	//std::vector<FeatureCloud> object_templates;
	
	//for item in list of pointclouds to look for
	


	//options for speedup: threading, limited use because already searching for multiple objects
	//Can PCL use GPU?
	//change templates to have less points, factor of complexity reduction, potentially affects accuracy
	//a
	
	int j = 1;
	//Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//while (true) {
	
	
		
	//auto start = high_resolution_clock::now();
	inter.get_kinect_data();
	inter.cloud = inter.FilterCloud(inter.cloud);
	inter.cloud = inter.SegmentPlane(inter.cloud);
	auto start = high_resolution_clock::now();
	//inter.UseGeometricConsistencyModels(inter.cloud, model);
	//around 8000 microsecond time
	inter.EuclidianClusterExtraction(inter.cloud);
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);

	std::cout << duration.count() << endl;
	//model=inter.TemplateAlign(gripper);
	//auto stop = high_resolution_clock::now();
	//auto duration = duration_cast<microseconds>(stop - start);
	//std::cout << duration.count();
	inter.DisplayPointCloud();
	//cloud_viewer.showCloud(inter.cloud);
		

		//while (!cloud_viewer.wasStopped())
		//{
		//cloud_viewer.showCloud(inter.cloud);
		//}
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		std::cout << "finished" << std::endl;
		//writer.write<pcl::PointXYZ>(ss.str(), *(inter.cloud), false);
		//import_cloud();
		
		std::getline(std::cin, dummy);
		j++;
	
	//}
}
boost::recursive_mutex global_lock;