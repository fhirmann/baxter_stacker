#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/common/common.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

//#include <pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_plotter.h>

#include "perception/GetScene.h"
#include "perception/Block.h"

#define GLOBAL_FRAME_ID "/world"
#define CAMERA_FRAME_ID "/camera_link"

struct Color
{
  int r;
  int g;
  int b;
};

/*============================================================================*/
/*==== Transform point cloud to baxter base  =================================*/
/*============================================================================*/

class Filter
{

  private:
    
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::ServiceServer service_;

    ros::Publisher  input_pc_pub_;
    ros::Publisher  transformed_pc_pub_;
    ros::Publisher  trimmed_pc_pub_;
    ros::Publisher  table_removed_pc_pub_;
    ros::Publisher  filtered_pc_pub_;

    tf::TransformListener tf_listener_;

    double table_x_low_, table_x_high_;
    double table_y_low_, table_y_high_;
    double table_z_low_, table_z_high_;

    double plane_seg_threshold_;
    int    plane_seg_max_iterations_;

    int    col_seg_min_cluster_size_;
    double col_seg_dist_threshold_;
    int    col_seg_point_col_threshold_;
    int    col_seg_region_col_threshold_;

    double outlier_remove_radius_;
    int    outlier_remove_min_neighb_;

    Color red, blue, yellow, green;

    pcl::visualization::PCLPlotter *plot1, *plot2, *plot3, *plot4;

    sensor_msgs::PointCloud2 latest_sensor_msg;
    boost::mutex latest_sensor_msg_lock;
    pcl::PointCloud<pcl::PointXYZRGB> *input_cloud;
    

  public:

    Filter(ros::NodeHandle nh):
      nh_(nh),
      table_x_low_(0.0), table_x_high_(0.0),
      table_y_low_(0.0), table_y_high_(0.0),
      table_z_low_(0.0), table_z_high_(0.0),
      plane_seg_threshold_(0.0),
      plane_seg_max_iterations_(0),
      col_seg_min_cluster_size_(100),
      col_seg_dist_threshold_(0.0),
      col_seg_point_col_threshold_(0.0),
      col_seg_region_col_threshold_(0.0),
      outlier_remove_radius_(0.0),
      outlier_remove_min_neighb_(0)
    {
      //ROS_INFO_STREAM( "FILTER: start constructor" );
      //latest_sensor_msg = new sensor_msgs::PointCloud2ConstPtr;
      //input_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;

      // Create a ROS subscriber for the input point cloud
      point_cloud_sub_ = nh_.subscribe ("/camera/depth_registered/points", 1, &Filter::sensor_msg_callback, this);
      //TODO: insert delay for around 1 second such that the camera exposure is fine

      // advertise new ros service GetScene
      service_ = nh_.advertiseService("get_scene", &Filter::get_scene_callback, this);
      ROS_INFO( "FILTER: Service get_scene activated!");
      
      // Create a ROS publisher for the output point cloud
      input_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/filter_input", 1);
      transformed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/filter_transformed", 1);
      trimmed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/filter_trimmed", 1);
      table_removed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/filter_table_removed", 1);
      filtered_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("/filter_output", 1);

      plot1 = new pcl::visualization::PCLPlotter();
      plot2 = new pcl::visualization::PCLPlotter();
      plot3 = new pcl::visualization::PCLPlotter();
      plot4 = new pcl::visualization::PCLPlotter();
      //ROS_INFO_STREAM( "FILTER: end constructor" );
    }

    /*============================================================================*/
    /*===== load parameters ======================================================*/
    /*============================================================================*/
    void load_parameters()
    {
      //ROS_INFO_STREAM( "FILTER:  load_param  " << nh_.getParam("/filter/table_x_low",  table_x_low_) );

      bool ok = true;
      if(!nh_.getParam("/filter/table_x_low",  table_x_low_))   ok = false;
      if(!nh_.getParam("/filter/table_x_high", table_x_high_))  ok = false;
      if(!nh_.getParam("/filter/table_y_low",  table_y_low_))   ok = false;
      if(!nh_.getParam("/filter/table_y_high", table_y_high_))  ok = false;
      if(!nh_.getParam("/filter/table_z_low",  table_z_low_))   ok = false;
      if(!nh_.getParam("/filter/table_z_high", table_z_high_))  ok = false;
      if(!nh_.getParam("/filter/plane_seg_threshold", plane_seg_threshold_))  ok = false;
      if(!nh_.getParam("/filter/plane_seg_max_iterations",  plane_seg_max_iterations_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_min_cluster_size",  col_seg_min_cluster_size_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_dist_threshold",  col_seg_dist_threshold_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_point_col_threshold",  col_seg_point_col_threshold_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_region_col_threshold",  col_seg_region_col_threshold_)) ok = false;
      if(!nh_.getParam("/filter/outlier_remove_radius",  outlier_remove_radius_)) ok = false;
      if(!nh_.getParam("/filter/outlier_remove_min_neighb",  outlier_remove_min_neighb_)) ok = false;

      std::vector<int> param_list;
      if( nh_.getParam("/filter/color_red", param_list))
      {  
        red.r = param_list[0];
        red.g = param_list[1];
        red.b = param_list[2];
      }
      else
        ok = false; 

      if( nh_.getParam("/filter/color_blue", param_list)) 
      {
        blue.r = param_list[0];
        blue.g = param_list[1];
        blue.b = param_list[2];
      }
      else
        ok = false;

      if( nh_.getParam("/filter/color_green", param_list))
      {
        green.r = param_list[0];
        green.g = param_list[1];
        green.b = param_list[2];
      }
      else
        ok = false;

      if( nh_.getParam("/filter/color_yellow", param_list))
      {
        yellow.r = param_list[0];
        yellow.g = param_list[1];
        yellow.b = param_list[2];
      }
      else
        ok = false;

      //std::cout << ok << "  " << nh_.getParam("/filter/table_x_low",  table_x_low_) );

      if( !ok)
        ROS_INFO_STREAM( "FILTER:  ERROR: was not able to load parameters!" );

    }

    /*============================================================================*/
    /*===== transform to baxter axis =============================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_to_baxters_axis( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {

	    //output cloud initialization
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
	    
	    ros::Time time_st = ros::Time(0);
      tf_listener_.waitForTransform( GLOBAL_FRAME_ID, cloud->header.frame_id, time_st, ros::Duration(1.0));
      
      //  if it could not convert into baxters frame return error
    	if(!pcl_ros::transformPointCloud( GLOBAL_FRAME_ID, *cloud, *cloud_transformed, tf_listener_))
    	{
      	ROS_ERROR("Error converting to desired frame");
        return cloud_transformed;
  	  }

      // remove nan values - leave this out if a organized pcl is needed in a later function
      std::vector<int> mapping;
	    pcl::removeNaNFromPointCloud(*cloud_transformed, *cloud_transformed, mapping);

	    // return the point cloud
	    return cloud_transformed;
    }

    /*============================================================================*/
    /*===== limit points to table ================================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr limit_points_to_table(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {
      // passthrough filter from pcl
      pcl::PassThrough<pcl::PointXYZRGB> pass;

      // filter in z axis
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredZ(new pcl::PointCloud<pcl::PointXYZRGB>);
      pass.setInputCloud( input_cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits( table_z_low_, table_z_high_);
      pass.filter( *cloud_filteredZ);

      // filter in x axis
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredX(new pcl::PointCloud<pcl::PointXYZRGB>);
      pass.setInputCloud( cloud_filteredZ);
      pass.setFilterFieldName("x");
      pass.setFilterLimits( table_x_low_, table_x_high_);
      pass.filter( *cloud_filteredX);

      // filter in y axis
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      pass.setInputCloud( cloud_filteredX);
      pass.setFilterFieldName("y");
      pass.setFilterLimits( table_y_low_, table_y_high_);
      pass.filter( *cloud_filtered);

      return cloud_filtered;
    }

    /*============================================================================*/
    /*===== remove table with segmentation =======================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Create the segmentation object for the planar model
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inlierInds(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        
        // set all parameters
        seg.setInputCloud( cloud);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC); 
        seg.setDistanceThreshold( plane_seg_threshold_);
        seg.setOptimizeCoefficients( true);
        seg.setMaxIterations( plane_seg_max_iterations_);

        // get indices of points lying in the plane of the table with the coefficients        
        seg.segment(*inlierInds, *coefficients); 

        if (inlierInds->indices.size() == 0){
		        ROS_INFO_STREAM( "FILTER: Could not find a plane in the scene." );
        }
	      else{
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud( cloud);
            extract.setIndices( inlierInds);

            // Extract the points in the plane            
            extract.setNegative(false);
            extract.filter( *cloud_plane);

            // Extract the points which don't belong to the plane
            extract.setNegative(true);
            extract.filter( *cloud_without_plane);
        }
      
        cloud_without_plane->header = cloud->header;

        return cloud_without_plane;	
    }

    /*============================================================================*/
    /*===== segmentation of the cloud by color clustering ========================*/
    /*============================================================================*/
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >* color_segmentation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // return a vector of point cloud pointers. Each pc includes points of one colored cluster

      // kd-tree object for searches. Color-based region growing clustering object.
	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree( new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
      std::vector <pcl::PointIndices> clusters;
      std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >* cloud_clusters( new std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >);

      // set parameters
	    kdtree->setInputCloud( cloud);

	    clustering.setInputCloud( cloud);
	    clustering.setSearchMethod( kdtree);
	    clustering.setMinClusterSize( col_seg_min_cluster_size_);
	    clustering.setDistanceThreshold( col_seg_dist_threshold_);
	    clustering.setPointColorThreshold( col_seg_point_col_threshold_);
	    clustering.setRegionColorThreshold( col_seg_region_col_threshold_);

      // extract clusters	    
	    clustering.extract( clusters);

      // For every cluster...
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;
      filter.setRadiusSearch( outlier_remove_radius_);
	    filter.setMinNeighborsInRadius( outlier_remove_min_neighb_);

	    int currentClusterNum = 1;   

      for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	    {
		    // ...add all its points to a new cloud...
		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_filtered( new pcl::PointCloud<pcl::PointXYZRGB>);

		    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			    cluster->points.push_back(cloud->points[*point]);
		    
        cluster->width = cluster->points.size();
		    cluster->height = 1;
		    cluster->is_dense = true;
        cluster->header = cloud->header;

		    // ... remove all outliers of the cloud
		    if (cluster->points.size() <= 0)
			    break;
	
	      filter.setInputCloud( cluster);
	      filter.filter( *cluster_filtered);
        
        // ...and add it to the return vector
        cloud_clusters->push_back( cluster_filtered);

		   // ROS_INFO_STREAM( "FILTER: Cluster " << currentClusterNum << " has " << cluster_filtered->points.size() << " points" );
		    currentClusterNum++;
	    }

      return cloud_clusters;
    }

    /*============================================================================*/
    /*====== extract position orientation and color of the objects ===============*/
    /*============================================================================*/
    void cloud_statistics( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
          double* x_min, double* x_max, double* x_mean, 
          double* y_min, double* y_max, double* y_mean,
          double* z_min, double* z_max, double* z_mean, 
          int* r_min, int* r_max, double* r_mean,
          int* g_min, int* g_max, double* g_mean, 
          int* b_min, int* b_max, double* b_mean)
    {
        int N = cloud->points.size();

        *x_min = cloud->points.at(0).x;
        *x_max = cloud->points.at(0).x;
        *x_mean = 0.0;

        *y_min = cloud->points.at(0).y;
        *y_max = cloud->points.at(0).y;
        *y_mean = 0.0;

        *z_min = cloud->points.at(0).z;
        *z_max = cloud->points.at(0).z;
        *z_mean = 0.0;

        *r_min = cloud->points.at(0).r;
        *r_max = cloud->points.at(0).r;
        *r_mean = 0.0;

        *g_min = cloud->points.at(0).g;
        *g_max = cloud->points.at(0).g;
        *g_mean = 0.0;

        *b_min = cloud->points.at(0).b;
        *b_max = cloud->points.at(0).b;
        *b_mean = 0.0;

        for(int j = 0; j < N; j++)
        {
          *x_min = std::min( *x_min, (double)cloud->points.at(j).x);          
          *x_max = std::max( *x_max, (double)cloud->points.at(j).x);
          *x_mean += cloud->points.at(j).x;
          
          *y_min = std::min( *y_min, (double)cloud->points.at(j).y);          
          *y_max = std::max( *y_max, (double)cloud->points.at(j).y);
          *y_mean += cloud->points.at(j).y;

          *z_min = std::min( *z_min, (double)cloud->points.at(j).z);          
          *z_max = std::max( *z_max, (double)cloud->points.at(j).z);
          *z_mean += cloud->points.at(j).z;

          *r_min = std::min( *r_min, (int)cloud->points.at(j).r);          
          *r_max = std::max( *r_max, (int)cloud->points.at(j).r);
          *r_mean += cloud->points.at(j).r;

          *g_min = std::min( *g_min, (int)cloud->points.at(j).g);          
          *g_max = std::max( *g_max, (int)cloud->points.at(j).g);
          *g_mean += cloud->points.at(j).g;

          *b_min = std::min( *b_min, (int)cloud->points.at(j).b);          
          *b_max = std::max( *b_max, (int)cloud->points.at(j).b);
          *b_mean += cloud->points.at(j).b;
        }

        *x_mean = *x_mean / N;
        *y_mean = *y_mean / N;
        *z_mean = *z_mean / N;
        *r_mean = *r_mean / N;
        *g_mean = *g_mean / N;
        *b_mean = *b_mean / N;
        
        ROS_INFO_STREAM( "FILTER:  x min: " << *x_min << " mean: " << *x_mean <<
                       " max: " << *x_max );
        ROS_INFO_STREAM( "FILTER:  y min: " << *y_min << " mean: " << *y_mean <<
                       " max: " << *y_max );
        ROS_INFO_STREAM( "FILTER:  z min: " << *z_min << " mean: " << *z_mean <<
                       " max: " << *z_max );
        ROS_INFO_STREAM( "FILTER:  r min: " << *r_min << " mean: " << *r_mean <<
                       " max: " << *r_max );
        ROS_INFO_STREAM( "FILTER:  g min: " << *g_min << " mean: " << *g_mean <<
                       " max: " << *g_max );
        ROS_INFO_STREAM( "FILTER:  b min: " << *b_min << " mean: " << *b_mean <<
                       " max: " << *b_max );
    }

    /*============================================================================*/
    int color_extraction(double r_mean, double g_mean, double b_mean)
    {
      double red_dist = std::sqrt(  std::pow( r_mean - red.r, 2) +
                                    std::pow( g_mean - red.g, 2) +
				                            std::pow( b_mean - red.b, 2) );
      double min_dist = red_dist;
      int color = perception::Block::RED;
      

      double blue_dist = std::sqrt( std::pow( r_mean - blue.r, 2) +
                                    std::pow( g_mean - blue.g, 2) +
                                    std::pow( b_mean - blue.b, 2) );

      if( blue_dist < min_dist)
      {
        min_dist = blue_dist;
        color = perception::Block::BLUE;
      }

      double green_dist = std::sqrt(  std::pow( r_mean - green.r, 2) +
                                      std::pow( g_mean - green.g, 2) +
                                      std::pow( b_mean - green.b, 2) );

      if( green_dist < min_dist)
      {
        min_dist = green_dist;
        color = perception::Block::GREEN;
      }

      double yellow_dist = std::sqrt( std::pow( r_mean - yellow.r, 2) +
                                      std::pow( g_mean - yellow.g, 2) +
                                      std::pow( b_mean - yellow.b, 2) );

      if( yellow_dist < min_dist)
      {
        min_dist = yellow_dist;
        color = perception::Block::YELLOW;
      }

      ROS_INFO_STREAM( "FILTER: red_dist " << red_dist << " blue_dist " << blue_dist << " green_dist " << green_dist << " yellow_dist " << yellow_dist );

      return color;
    }


    void object_extraction( std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >* color_clusters, bool* success, std::vector<perception::Block>* blocks)
    {
      ROS_INFO_STREAM( "FILTER: object_extraction " << color_clusters->size() );

      *success = false;      

      // for each cluster
      for(int i = 0; i < color_clusters->size(); i++)
	    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = color_clusters->at(i);
        perception::Block block;

        ROS_INFO_STREAM( "FILTER: cloud size " << cloud->points.size() );

        // get statistic values
        double x_min, x_max, x_mean;
        double y_min, y_max, y_mean;
        double z_min, z_max, z_mean;
        int r_min, r_max;
        int g_min, g_max;
        int b_min, b_max;
        double r_mean, g_mean, b_mean;

        cloud_statistics( cloud, 
          &x_min, &x_max, &x_mean, &y_min, &y_max, &y_mean, 
          &z_min, &z_max, &z_mean, &r_min, &r_max, &r_mean, 
          &g_min, &g_max, &g_mean, &b_min, &b_max, &b_mean);


        block.id = i;
        block.type = perception::Block::CUBOID;

        // extract size and check if plausible
        block.width = x_max - x_min;
        block.depth = y_max - y_min;
        block.height = z_max - z_min;

        // extract position and orientation
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = GLOBAL_FRAME_ID;

        poseStamped.pose.position.x = x_mean;
        poseStamped.pose.position.y = y_mean;
        poseStamped.pose.position.z = z_mean;
        
        double theta = 0.0;
        tf::Quaternion q = tf::createQuaternionFromRPY(0 , 0, theta);
        poseStamped.pose.orientation.w = q.getW();
        poseStamped.pose.orientation.x = q.getX();
        poseStamped.pose.orientation.y = q.getY();
        poseStamped.pose.orientation.z = q.getZ();

        block.pose = poseStamped;

        // extract color
        block.color = color_extraction( r_mean, g_mean, b_mean);

        blocks->push_back(block);
        *success = true;
      }      
    }


    /*============================================================================*/
    /*====== process the point cloud =============================================*/
    /*============================================================================*/
    void process_point_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, bool* success, std::vector<perception::Block>* blocks)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr trimmed_cloud;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_removed_cloud;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;
      std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr >* color_clusters;      

      // Load all parameters
      load_parameters();

      // Do data processing here...

      // Limit points corresponding to table
      trimmed_cloud = limit_points_to_table( input_cloud);
      ROS_INFO_STREAM( "FILTER: trimmed_cloud: " << trimmed_cloud->points.size() );
      if (trimmed_cloud->points.size() == 0){ 
        *success = false;
        return;
      }

      // remove table surface (planar model) from the cloud
      table_removed_cloud = plane_segmentation( trimmed_cloud);
      ROS_INFO_STREAM( "FILTER: table_removed_cloud: " << table_removed_cloud->points.size() );
      if (table_removed_cloud->points.size() == 0){
        *success = false; 
        return;
      }

      /*
      plot_color_distribution_rgb(table_removed_cloud);
      plot_x_distribution(table_removed_cloud);
      plot_y_distribution(table_removed_cloud);
      plot_z_distribution(table_removed_cloud);*/

      /*pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud (new pcl::PointCloud<pcl::PointXYZHSV>());
      pcl::PointCloudXYZRGBtoXYZHSV(*table_removed_cloud, *hsv_cloud); // convert to hsv
      plot_color_distribution_hsv(hsv_cloud);*/

      // segmentation by color
      color_clusters = color_segmentation( table_removed_cloud);
      ROS_INFO_STREAM( "FILTER: color_segementation nr of clusters: " << color_clusters->size() );
      if (color_clusters->size() == 0){ 
        *success = false;
        return;
      }

      // extract object information
      object_extraction( color_clusters, success, blocks );

      // Publish the data.
      transformed_pc_pub_.publish( *input_cloud);
      trimmed_pc_pub_.publish( *trimmed_cloud);
      table_removed_pc_pub_.publish( *table_removed_cloud);
      filtered_pc_pub_.publish( color_clusters->at(0));
    }

    /*============================================================================*/
    /*====== callbacks ===========================================================*/
    /*============================================================================*/
    void sensor_msg_callback (const sensor_msgs::PointCloud2ConstPtr& input)
    { 
      latest_sensor_msg_lock.lock();    
      //pcl::fromROSMsg( *input, *input_cloud);
      //latest_sensor_msg = *input;
      latest_sensor_msg_lock.unlock();
    }


    void process_msg ( const sensor_msgs::PointCloud2ConstPtr& input, bool* success, std::vector<perception::Block>* blocks)
    {
      //sensor_msgs::PointCloud2 current_input;// = new sensor_msgs::PointCloud2ConstPtr(); 
      //const sensor_msgs::PointCloud2ConstPtr& input = &current_input;     
      //pcl::PointCloud<pcl::PointXYZRGB>* current_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_ptr(current_cloud);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud;

      //grap current sensor message
      //latest_sensor_msg_lock.lock(); 
      //current_input = latest_sensor_msg;
      //current_cloud = input_cloud;
      //latest_sensor_msg_lock.unlock(); 

      //TODO: check if message is up to date

      // convert msgs to point cloud
      pcl::fromROSMsg( *input, *input_cloud);

      // transform to baxter axis
      transformed_cloud = transform_to_baxters_axis( input_cloud);
      ROS_INFO_STREAM( "FILTER: \ntransformed_cloud: " << transformed_cloud->points.size() );
      if (transformed_cloud->points.size() == 0){ 
        *success = false;
        return;
      }

      // save pointcloud to file for debug mode
	    //pcl::io::savePCDFileASCII("ascii.pcd", *transformed_cloud);
      pcl::io::savePCDFileBinary("binary.pcd", *transformed_cloud);

      process_point_cloud( transformed_cloud, success, blocks);

      // Publish the data.
      input_pc_pub_.publish( input_cloud);
    }



    bool get_scene_callback( perception::GetScene::Request  &req,
                             perception::GetScene::Response &res)
    {
      ROS_INFO_STREAM( "FILTER: requested get scene service" );
    
      //define output variables
      bool success = false;
      std::vector<perception::Block>* blocks = new std::vector<perception::Block>;      

      //get data from camera
      sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", nh_, ros::Duration(10));
      
      if (msg)
      {
        //process data
        process_msg( msg, &success, blocks);
      }      
      else
      {
        ROS_ERROR_STREAM("FILTER: run into timeout because did not receive a pointcloud message");
      }

      //return output
      res.success = success;
      res.blocks = *blocks;
      
      ROS_INFO_STREAM( "FILTER: sending back response " );
      return true;
    }


    /*============================================================================*/
    /*====== statistical plots ===================================================*/
    /*============================================================================*/
    void plot_color_distribution_rgb( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> r, g, b;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        r.push_back( cloud->points.at(i).r);
        g.push_back( cloud->points.at(i).g);
        b.push_back( cloud->points.at(i).b);
      }

      plot1->clearPlots();
      plot1->addHistogramData(r, 255, "red", std::vector<char>{-127,0,0,-127});
      plot1->addHistogramData(g, 255, "green", std::vector<char>{0,-127,0,-127});
      plot1->addHistogramData(b, 255, "blue", std::vector<char>{0,0,-127,-127});
      plot1->setShowLegend(true);
      plot1->plot();
    }

    void plot_color_distribution_hsv( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> h, s, v;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        h.push_back( cloud->points.at(i).h);
        s.push_back( cloud->points.at(i).s);
        v.push_back( cloud->points.at(i).v);
      }

      plot1->clearPlots();
      plot1->addHistogramData(h, 255, "hue", std::vector<char>{127,0,0,127});
      plot1->setShowLegend(true);
      plot1->plot();

      plot2->clearPlots();
      plot2->addHistogramData(s, 255, "saturation", std::vector<char>{0,127,0,127});
      plot2->setShowLegend(true);
      plot2->plot();

      plot3->clearPlots();
      plot3->addHistogramData(v, 255, "value", std::vector<char>{0,0,127,127});
      plot3->setShowLegend(true);
      plot3->plot();
    }

    void plot_x_distribution( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> x;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        x.push_back( cloud->points.at(i).x);
      }

      plot2->clearPlots();
      plot2->addHistogramData(x, 100, "x");
      plot2->setShowLegend(true);
      plot2->plot();
    }

    void plot_y_distribution( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> y;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        y.push_back( cloud->points.at(i).y);
      }

      plot3->clearPlots();
      plot3->addHistogramData(y, 100, "y");
      plot3->setShowLegend(true);
      plot3->plot();
    }

    void plot_z_distribution( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> z;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        z.push_back( cloud->points.at(i).z);
      }

      plot4->clearPlots();
      plot4->addHistogramData(z, 100, "z");
      plot4->setShowLegend(true);
      plot4->plot();
    }
};


/*============================================================================*/
/*==== main    ===============================================================*/
/*============================================================================*/


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "baxter_perception");
  ros::NodeHandle nh;
  ros::Rate rate(0.2);
  
  Filter* filter = new Filter(nh);
  ROS_INFO("FILTER: init done");

  //parse calling arguments
  ROS_INFO_STREAM( "FILTER: Number of arguments " << argc);
  std::string cmd;  
  std::string filename;
  
  if(argc >=2) cmd = argv[1];
  if(argc >=3) filename = argv[2];

  // running in debug mode --> load pcd file and execute the same point cloud processing on it as in the service call
  if( !cmd.empty() && cmd == "debug")
  {
    ROS_INFO("FILTER: entering debug mode with the file %s", filename.c_str());

    // load point cloud from file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }
    ROS_INFO( "FILTER: Loaded %u points", (cloud->width * cloud->height));

    cloud->header.frame_id = GLOBAL_FRAME_ID;

    // view point cloud
    //pcl::visualization::CloudViewer viewer("input cloud");
	  //viewer.showCloud(cloud);

    bool success = false;
    std::vector<perception::Block>* blocks = new std::vector<perception::Block>;      

    while (ros::ok())
    {
      blocks->clear();
      filter->process_point_cloud( cloud, &success, blocks);

      ROS_INFO( "FILTER: success of the process: %u", success);
      for(int i=0; i < blocks->size(); i++) {
        ROS_INFO_STREAM( "FILTER: block nr.: " << i << " - " << blocks->at(i) );//);
      }

      rate.sleep();
    }
  }


  // continius camera mode --> gets every 2 seconds a picture from the camera --> processes it the same way as it would like in the service --> outputs clouds for rviz and the blocks on the terminal as text
  if( !cmd.empty() && cmd == "cont")
  {
    ROS_INFO( "FILTER: entering continious mode");
    bool success = false;
    std::vector<perception::Block>* blocks = new std::vector<perception::Block>; 

    // if it is in continuos mode - get message --> extract object info --> publish info --> repeat at a 1Hz frequency
    while (ros::ok())
    {
      blocks->clear();    

      sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", nh, ros::Duration(10));
      filter->process_msg( msg, &success, blocks);

      ROS_INFO( "FILTER: success of the process: %u", success);
      for(int i=0; i < blocks->size(); i++) {
        ROS_INFO_STREAM( "FILTER: block nr.: " << i << " - " << blocks->at(i) );
      }

      rate.sleep();
    }
  }

  // Spin
  ros::spin();

  return 0;
}
