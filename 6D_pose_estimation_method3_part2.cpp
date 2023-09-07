#include <iostream>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <pcl/io/ply_io.h>

#include "icp_simple.hpp"


void loadPLYFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_name, cloud) == -1)
    {
        PCL_ERROR("Failed to load PLY file.");
        return;
    }

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);
}

void loadFile(const char* file_name, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) == -1)
    {
        PCL_ERROR("Failed to load point cloud.");
        return;
    }

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(cloud, cloud, index);
}


Eigen::Vector4f computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    return centroid;
}

void moveToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector4f& centroid) {
    for (pcl::PointXYZ &point : cloud->points) {
        point.x -= centroid[0];
        point.y -= centroid[1];
        point.z -= centroid[2];
    }
}
Eigen::Vector4f centroid_source; 
Eigen::Vector4f centroid_target; 

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f T;  

    {
		std::string Entire_scene = "D:\\6D_Pose_Estimation_Method3\\entire_scene.pcd";
		std::string target_path = "D:\\6D_Pose_Estimation_Method3\\masked_object.pcd";
    	std::string source_path = "D:\\CPP_Grasp_Synthesis\\models\\obj_000005.ply";
  
        loadPLYFile(source_path.c_str(), *source);
        loadFile(target_path.c_str(), *cloud_target);
		loadFile(Entire_scene.c_str(), *raw_cloud);


		// Convert D435i's point cloud (source) from mm to meters 
		float conversion_factor = 0.001;  // Conversion factor from mm to m

		for (pcl::PointXYZ &point : source->points) {
			point.x *= conversion_factor;
			point.y *= conversion_factor;
			point.z *= conversion_factor;
		}

		// Compute centroids
		centroid_source = computeCentroid(source);
		centroid_target = computeCentroid(cloud_target);

		// Move point clouds to origin
		moveToOrigin(source, centroid_source);
		moveToOrigin(cloud_target, centroid_target);

		// Compute normals for both point clouds
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(0.03);

		ne.setInputCloud(source);
		ne.compute(*source_normals);

		ne.setInputCloud(cloud_target);
		ne.compute(*target_normals);
		std::cout<< "Calculating VFH for Source and Target point clouds for Rough Alignment"<<endl;
		
		// Compute VFH descriptors for both point clouds
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		pcl::PointCloud<pcl::VFHSignature308>::Ptr source_vfh (new pcl::PointCloud<pcl::VFHSignature308>);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr target_vfh (new pcl::PointCloud<pcl::VFHSignature308>);
		vfh.setInputCloud(source);
		vfh.setInputNormals(source_normals);
		vfh.setSearchMethod(tree);
		vfh.compute(*source_vfh);

		vfh.setInputCloud(cloud_target);
		vfh.setInputNormals(target_normals);
		vfh.compute(*target_vfh);

		// Match the VFH descriptors
		pcl::KdTreeFLANN<pcl::VFHSignature308> match_search;
		match_search.setInputCloud(target_vfh);
		std::vector<int> match_idx(1);
		std::vector<float> match_dist(1);
		match_search.nearestKSearch(*source_vfh, 0, 1, match_idx, match_dist);

		// Use ICP to refine the pose estimation
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(source);
		icp.setInputTarget(cloud_target);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);

		Eigen::Matrix4f transformation = icp.getFinalTransformation();
		Eigen::Matrix4f inverse_transformation = transformation.inverse();


		// Create a copy of the source point cloud
		pcl::transformPointCloud(*source, *cloud_source, inverse_transformation);


    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans (new pcl::PointCloud<pcl::PointXYZ>());

	{
		Eigen::MatrixXf source_matrix = cloud_source->getMatrixXfMap(3,4,0).transpose();
		Eigen::MatrixXf target_matrix = cloud_target->getMatrixXfMap(3,4,0).transpose();

		int max_iteration = 500;
		float tolenrance = 0.00001;

		// call icp
		ICP_OUT icp_result = icp(source_matrix.cast<double>(), target_matrix.cast<double>(), max_iteration, tolenrance);

		int iter = icp_result.iter;
		T = icp_result.trans.cast<float>();
		std::vector<float> distances = icp_result.distances;

		Eigen::MatrixXf source_trans_matrix = source_matrix;

		int row = source_matrix.rows();

		// Validate matrix dimensions and indices
		std::cout << "source_matrix size: " << source_matrix.rows() << "x" << source_matrix.cols() << std::endl;

		Eigen::MatrixXf source_trans4d = Eigen::MatrixXf::Ones(4, row);

		for (int i = 0; i < row; i++)
		{
			// std::cout << "Current i: " << i << std::endl; // Debug current index
			source_trans4d.block<3,1>(0,i) = source_matrix.block<1,3>(i,0).transpose();
		}

		source_trans4d = T * source_trans4d;

		for (int i = 0; i < row; i++)
		{
			source_trans_matrix.block<1,3>(i,0) = source_trans4d.block<3,1>(0,i).transpose();
		}

		pcl::PointCloud<pcl::PointXYZ> temp_cloud;
		temp_cloud.width = row;
		temp_cloud.height = 1;
		temp_cloud.points.resize(row);

		for (size_t n = 0; n < row; n++) 
		{
			temp_cloud[n].x = source_trans_matrix(n,0);
			temp_cloud[n].y = source_trans_matrix(n,1);
			temp_cloud[n].z = source_trans_matrix(n,2);	
		}

		cloud_source_trans = temp_cloud.makeShared();

		 // Adjust the transformed source cloud by adding the centroid of the target cloud
    	for (pcl::PointXYZ &point : cloud_source_trans->points) {
        point.x += centroid_target[0];
        point.y += centroid_target[1];
        point.z += centroid_target[2];
    	}

		for (pcl::PointXYZ &point : cloud_target->points) {
		point.x += centroid_target[0];
		point.y += centroid_target[1];
		point.z += centroid_target[2];
		}
		
	}

	{ // visualization
		boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(255, 255, 255);

		// // red
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source, 0, 255, 0);
		// viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source");
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");

		// green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(raw_cloud, 0, 255, 0);
		viewer->addPointCloud<pcl::PointXYZ>(raw_cloud, source_color, "original");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original");


		// blue
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 0, 255);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_target, target_color, "target");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target");

		// red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color(cloud_source_trans, 255, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_source_trans, source_trans_color, "source trans");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans");

		// Add a coordinate system at the centroid of source point cloud
        Eigen::Vector4f centroid_source;
        pcl::compute3DCentroid(*source, centroid_source);
        Eigen::Affine3f transform_source = Eigen::Affine3f::Identity();
        transform_source.translation() << centroid_source[0], centroid_source[1], centroid_source[2];
        // viewer->addCoordinateSystem(0.1, transform_source, "coordinate system source", 0);
        
        // Add a coordinate system at the centroid of target point cloud
        Eigen::Vector4f centroid_target;
        pcl::compute3DCentroid(*cloud_target, centroid_target);
        Eigen::Affine3f transform_target = Eigen::Affine3f::Identity();
        transform_target.translation() << centroid_target[0], centroid_target[1], centroid_target[2];
        viewer->addCoordinateSystem(0.1, transform_target, "coordinate system target", 0);
        
        // Add a coordinate system at the centroid of source to visualize the orientation change
        Eigen::Affine3f transform_orientation_change = Eigen::Affine3f(T); // Convert T to an Affine transformation
        transform_orientation_change.translation() << centroid_source[0], centroid_source[1], centroid_source[2]; // Use the original centroid of the source cloud
        // viewer->addCoordinateSystem(0.1, transform_orientation_change, "coordinate system orientation change", 0);

        viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
        viewer->resetCamera();
        viewer->spin();
	}

	return 0;
}
