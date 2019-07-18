/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

	ProcessPointClouds<pcl::PointXYZI> *pointProc = new (ProcessPointClouds<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProc->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  	float radius = 6.0;
  	pcl::PointCloud<pcl::PointXYZI>::Ptr filtCloud = pointProc->FilterCloud(inputCloud, 0.2f ,
                                                                            Eigen::Vector4f (-radius, -radius, -radius, 1), 
                                                                 			Eigen::Vector4f (radius, radius, radius, 1));
	//renderPointCloud(viewer,inputCloud,"inputCloud");
  	renderPointCloud(viewer, filtCloud, "inputCloud");
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = 0;//true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
	Lidar *lidar = new Lidar(cars, 0); 

    // TODO:: Create point processor
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud = lidar->scan(); 
	//renderRays(viewer, cars[0].position + Vect3(0, 0, 2.2), pcloud);
	renderPointCloud(viewer, pcloud, "Scene", Color(1, 1, 1));

	ProcessPointClouds<pcl::PointXYZ> *pproc = new (ProcessPointClouds<pcl::PointXYZ>);

	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, 
			  pcl::PointCloud<pcl::PointXYZ>::Ptr> seg_results = pproc->SegmentPlane(pcloud, 100, 0.2);

	renderPointCloud(viewer, seg_results.first, "Obstacles", Color(80,0,80));
	renderPointCloud(viewer, seg_results.second, "Road", Color(0,100,100));
	
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pproc->Clustering(seg_results.first, 1.0, 3, 30);
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster :cloudClusters)
    {
        cout << "Cluster size";
        pproc->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id), colors[cluster_id]);
		Box box = pproc->BoundingBox(cluster);
		renderBox(viewer,box,cluster_id);
        ++cluster_id;
    }    
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = Side;// XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
	cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
