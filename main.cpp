// Standard
#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <windows.h>
#include <direct.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>


using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

// Global Variables
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

	// Normals to Texture Coordinates conversion
	int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color, float kRange) {

	// Object Declaration (Point Cloud)
	cloud_pointer cloud(new point_cloud);

	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); i++)
	{
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		if ((Texture_Coord[i].u >= 0 &&
			Texture_Coord[i].u <= 1 &&
			Texture_Coord[i].v >= 0 &&
			Texture_Coord[i].v <= 1))
		{
			// Obtain color texture for specific point
			RGB_Color = RGB_Texture(color, Texture_Coord[i]);

			// Mapping Color (BGR due to Camera Model)
			cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
			cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
			cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
		}
		else
		{
			cloud->points[i].r = 255;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;
		}
	}

	cloud_pointer extracted_cloud_ptr(new point_cloud);

	for (int c = 0; c < cloud->size(); c++)
	{
		bool extract_flag = cloud->points[c].r == 255 && cloud->points[c].g == 0 && cloud->points[c].b == 0;
		bool distance_check = cloud->points[c].z >= 10.0f;
		if (!extract_flag)
		{
			if (!distance_check)
			{
				extracted_cloud_ptr->points.push_back(cloud->points[c]);
			}
		}
	}

	// Create the down sampling object
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	const cloud_pointer c_point_cloud_ptr = extracted_cloud_ptr;
	sor.setInputCloud(c_point_cloud_ptr);
	sor.filter(*extracted_cloud_ptr);

	return extracted_cloud_ptr; // PCL RGB Point Cloud generated
}
// --------------------------------------------------------------------------------
bool mkdir_flag = true;
std::string name, img_fname, points_fname;
void SavePointCloud(cv::Mat color, cloud_pointer cloud, int count)
{
	std::stringstream ss;
	ss << std::setw(5) << std::setfill('0') << count;

	if (mkdir_flag == true)
	{
		SYSTEMTIME st;
		GetLocalTime(&st);
		std::string folder_name = "Data\\"
			+ to_string(st.wYear) + "_"
			+ to_string(st.wMonth) + "_"
			+ to_string(st.wDay) + "_"
			+ to_string(st.wHour) + "_"
			+ to_string(st.wMinute);
		name = folder_name;
		img_fname = name + "\\images";
		points_fname = name + "\\PointCloud";
		_mkdir( name.c_str() );
		_mkdir(img_fname.c_str());
		_mkdir(points_fname.c_str());
		mkdir_flag = false;
	}
	cv::imwrite(img_fname + "\\" + ss.str() + ".png", color);
	//std::cout << points_fname + "\\" + ss.str() + ".png" << std::endl;
	
	cloud_pointer extracted_cloud_ptr(new point_cloud);

	for (int c = 0; c < cloud->size(); c++)
	{
		bool extract_flag = cloud->points[c].r == 255 && cloud->points[c].g == 0 && cloud->points[c].b == 0;
		if (!extract_flag)
		{
			extracted_cloud_ptr->points.push_back(cloud->points[c]);
		}
	}

	ss.clear();
	ss << "";
}
// --------------------------------------------------------------------------------
int main() {
	//======================
	// Variable Declaration
	//======================
	bool captureLoop = true; // Loop control for generating point clouds
	float kRange = 10.0f;

	//====================
	// Object Declaration
	//====================
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//======================
	// Stream configuration
	//======================
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

	rs2::pipeline_profile selection = pipe.start(cfg);

	rs2::device selected_device = selection.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	{
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	{
		// Query min and max values:
		auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	}

	// Create viewer object titled "Captured Frame"
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));
	viewer->addCoordinateSystem(3.0, "coordinate");
	viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer->removeCoordinateSystem("coordinate");

	pcl::visualization::PointCloudColorHandler<RGB_Cloud>::Ptr handler(new pcl::visualization::PointCloudColorHandlerRGBField<RGB_Cloud>());


	// Begin Stream with default configs

	// Loop and take frame captures upon user input
	while (!viewer->wasStopped()) 
	{
		viewer->spinOnce(); // Allow user to rotate point cloud and view it

		// Capture a single frame and obtain depth + RGB values from it    
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		auto RGB = frames.get_color_frame();

		// Map Color texture to each point
		pc.map_to(RGB);

		// Generate Point Cloud
		auto points = pc.calculate(depth);

		// Color
		auto color = frames.get_color_frame();
		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		if (!color)
			color = frames.get_infrared_frame();
		// Get the camera image information
		cv::Mat m(color.get_height(), color.get_width(), CV_8UC3,
			const_cast<void*>(color.get_data()));
		cv::Mat img_depth(color.get_height(), color.get_width(), CV_16UC1,
			const_cast<void*>(depth.get_data()));

		// Convert generated Point Cloud to PCL Formatting
		cloud_pointer cloud = PCL_Conversion(points, RGB, kRange);
		std::cout << "a" << std::endl;

		handler->setInputCloud(cloud);
		if (!viewer->updatePointCloud(cloud, *handler, "cloud"))
		{
			viewer->addPointCloud(cloud, *handler, "cloud");
		}
		cloud = nullptr;
		viewer->removeAllShapes();

		cout << endl;
		cout << "Press [Q] in viewer to continue. " << endl;

		// Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
		viewer->spinOnce(1, true);

		//SavePointCloud(m, cloud, i);

		i++; // Increment File Name
	}//End-while


	cout << "Exiting Program... " << endl;
	return EXIT_SUCCESS;
}