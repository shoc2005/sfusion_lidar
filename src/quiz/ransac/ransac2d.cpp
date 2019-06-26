/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
		
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	// For max iterations 
	unsigned long int total_count = cloud->points.size();
	if (total_count == 1) {
		cout << "Can't run Ransac due to size of the point cloud is less than 2 points!" << endl;
		return inliersResult;
	}
	
	double A = 0.0;
	double B = 0.0;
	double C = 0.0;
	int a = 0;
	int b = 0;

	unsigned long int dist_best = 0;
	
	while (maxIterations--) {

		/* pick random two points from the point cloud */
		a = (int)rand() % total_count;
		b = (int)rand() % total_count;

		/* estimate line equation of standard form for the two points */
		A = cloud->points[a].y - cloud->points[b].y * 100;
		B = cloud->points[b].x - cloud->points[a].x * 100;
		C = (cloud->points[a].x * cloud->points[b].y) - (cloud->points[b].x * cloud->points[a].y);		

		/* iterate from all points and estimate distance */
		std::unordered_set<int> trial_set; // the temporary set within the Iteration

		float dist = 0.0;
		for(int i = 0; i < total_count; i++) {
			double x = cloud->points[i].x;
			double y = cloud->points[i].y;
			//double z = cloud->points[i].z;
			dist = abs(A * x + B * y + C) / sqrt(pow(A, 2.0) + pow(B, 2.0));
			if (dist <= distanceTol)   {
				trial_set.insert(i);
			}
		}
		/* compare with best fit results */
		if (trial_set.size() > dist_best) {
			dist_best = trial_set.size();
			inliersResult.clear();

			/* copy indices to the resulting set */
			std::unordered_set<int>::iterator it = trial_set.begin();
			while (it != trial_set.end()) {
				inliersResult.insert(*it);
				it++;
			}
		}


	}

	
	return inliersResult;

}

int main (int argc, char *argv[])
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	std::unordered_set<int> inliers; 

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	if (argc == 3) {
		int maxIter = atoi(argv[1]);
		float tol = atof(argv[2]);
		inliers = Ransac(cloud, maxIter, tol);
	} else {
		inliers = Ransac(cloud, 50, 0.5);
	}

	cout << "Size of inliers is: " << inliers.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
