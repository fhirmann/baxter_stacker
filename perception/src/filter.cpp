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

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>

#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

//#include <pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_plotter.h>

#include "perception/GetScene.h"
#include "perception/Block.h"

#define GLOBAL_FRAME_ID "/table"
#define CAMERA_FRAME_ID "/camera_rgb_optical_frame"

#define UNKNOWN_COLOR 100

#define BLOCK_TYPE_BIG_SQUARE   1 //40x40x80
#define BLOCK_TYPE_BIG_SLIM     2 //20x40x80
#define BLOCK_TYPE_CUBE         3 //40x40x40
#define BLOCK_TYPE_SMALL_SQUARE 4 //25x25x50
#define BLOCK_TYPE_UNKNOWN      10

struct BLOCK_TYPE
{
  int id;
  double width;
  double depth;
  double height;
};

// global parameters because the static function is not able to use member variables
double hue_red_;
double hue_blue_;
double hue_green_;
double hue_yellow_;
double hue_tol_;


/*============================================================================*/
/*==== Transform point cloud to baxter base  =================================*/
/*============================================================================*/

class Filter
{

  private:
    
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::ServiceServer service_;
    ros::Timer timer_;

    ros::Publisher  input_pc_pub_;
    ros::Publisher  transformed_pc_pub_;
    ros::Publisher  smoothed_pc_pub_;
    ros::Publisher  trimmed_pc_pub_;
    ros::Publisher  table_removed_pc_pub_;
    ros::Publisher  block1_pc_pub_;
    ros::Publisher  block2_pc_pub_;
    ros::Publisher  block3_pc_pub_;
    ros::Publisher  block4_pc_pub_;
    ros::Publisher  block5_pc_pub_;

    ros::Publisher  block1_top_pc_pub_;
    ros::Publisher  block1_left_pc_pub_;
    ros::Publisher  block1_right_pc_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trimmed_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_removed_cloud_;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud_;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr b1_top_;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr b1_left_;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr b1_right_;
    std::vector< pcl::PointCloud<pcl::PointXYZHSV>::Ptr >* color_clusters_;

    tf::TransformListener tf_listener_;

    double table_x_low_, table_x_high_;
    double table_y_low_, table_y_high_;
    double table_z_low_, table_z_high_;

    double smoothing_radius_;

    double plane_seg_threshold_;
    int    plane_seg_max_iterations_;

    int    col_seg_min_cluster_size_;
    int    col_seg_max_cluster_size_;
    double col_seg_dist_threshold_;

    double block_plane_dist_threshold_;
    int    block_plane_max_iterations_;
    int    block_plane_min_points_;

    //Blocksize big_square_, big_slim_, cube_, small_square_;
    std::vector<BLOCK_TYPE> block_types_;
    double block_type_tol_;

    pcl::visualization::PCLPlotter *plot1_, *plot2_, *plot3_, *plot4_, *plot5_, *plot6_;

    sensor_msgs::PointCloud2 latest_sensor_msg_;

    bool success_;
    std::vector<perception::Block>* blocks_;
    boost::mutex blocks_lock_;

    int count_;
    bool debug_;

    /*============================================================================*/
    /*===== load parameters ======================================================*/
    /*============================================================================*/
    bool load_parameters()
    {
      //ROS_INFO_STREAM( "FILTER:  load_param  " << nh_.getParam("/filter/table_x_low",  table_x_low_) );

      bool ok = true;
      if(!nh_.getParam("/filter/table_x_low",  table_x_low_))   ok = false;
      if(!nh_.getParam("/filter/table_x_high", table_x_high_))  ok = false;
      if(!nh_.getParam("/filter/table_y_low",  table_y_low_))   ok = false;
      if(!nh_.getParam("/filter/table_y_high", table_y_high_))  ok = false;
      if(!nh_.getParam("/filter/table_z_low",  table_z_low_))   ok = false;
      if(!nh_.getParam("/filter/table_z_high", table_z_high_))  ok = false;

      //ROS_INFO_STREAM(" load_param: ok1 " << ok);

      if(!nh_.getParam("/filter/smoothing_radius",          smoothing_radius_))         ok = false;
      if(!nh_.getParam("/filter/plane_seg_threshold",       plane_seg_threshold_))      ok = false;
      if(!nh_.getParam("/filter/plane_seg_max_iterations",  plane_seg_max_iterations_)) ok = false;

      //ROS_INFO_STREAM(" load_param: ok2 " << ok);

      if(!nh_.getParam("/filter/col_seg_min_cluster_size",  col_seg_min_cluster_size_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_max_cluster_size",  col_seg_max_cluster_size_)) ok = false;
      if(!nh_.getParam("/filter/col_seg_dist_threshold",    col_seg_dist_threshold_))   ok = false;

      //ROS_INFO_STREAM(" load_param: ok3 " << ok);

      if(!nh_.getParam("/filter/hue_red",   hue_red_))   ok = false;
      if(!nh_.getParam("/filter/hue_blue",  hue_blue_))  ok = false;
      if(!nh_.getParam("/filter/hue_green", hue_green_)) ok = false;
      if(!nh_.getParam("/filter/hue_tol",   hue_tol_))   ok = false;

      //ROS_INFO_STREAM(" load_param: ok4 " << ok);

      if(!nh_.getParam("/filter/block_plane_dist_threshold",   block_plane_dist_threshold_))   ok = false;
      if(!nh_.getParam("/filter/block_plane_max_iterations",   block_plane_max_iterations_))   ok = false;
      if(!nh_.getParam("/filter/block_plane_min_points",       block_plane_min_points_))       ok = false;

      block_types_.clear();
      std::vector<double> param_list;
      BLOCK_TYPE big_square;
      if( nh_.getParam("/filter/block_type_big_square", param_list))
      {
        big_square.width  = param_list[0];
        big_square.depth  = param_list[1];
        big_square.height = param_list[2];
        big_square.id = BLOCK_TYPE_BIG_SQUARE;
        block_types_.push_back(big_square);
      }
      else
        ok = false;

      //ROS_INFO_STREAM(" load_param: ok5 " << ok);

      BLOCK_TYPE big_slim;
      if( nh_.getParam("/filter/block_type_big_slim", param_list))
      {
        big_slim.width  = param_list[0];
        big_slim.depth  = param_list[1];
        big_slim.height = param_list[2];
        big_slim.id = BLOCK_TYPE_BIG_SLIM;
        block_types_.push_back(big_slim);
      }
      else
        ok = false;

      //ROS_INFO_STREAM(" load_param: ok6 " << ok);

      BLOCK_TYPE cube;
      if( nh_.getParam("/filter/block_type_cube", param_list))
      {
        cube.width  = param_list[0];
        cube.depth  = param_list[1];
        cube.height = param_list[2];
        cube.id = BLOCK_TYPE_CUBE;
        block_types_.push_back(cube);
      }
      else
        ok = false;

      //ROS_INFO_STREAM(" load_param: ok7 " << ok);

      BLOCK_TYPE small_square;
      if( nh_.getParam("/filter/block_type_small_square", param_list))
      {
        small_square.width  = param_list[0];
        small_square.depth  = param_list[1];
        small_square.height = param_list[2];
        small_square.id = BLOCK_TYPE_SMALL_SQUARE;
        block_types_.push_back(small_square);
      }
      else
        ok = false;

      //ROS_INFO_STREAM(" load_param: ok8 " << ok);

      if(!nh_.getParam("/filter/block_type_tol",   block_type_tol_))   ok = false;


      if( !ok)
        ROS_ERROR_STREAM( "FILTER:  was not able to load parameters!" );

      return ok;
    }

    /*============================================================================*/
    /*===== transform to baxter axis =============================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_to_baxters_axis( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      int ret = 0;

      //output cloud initialization
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

      //  if it could not convert into baxters frame return error
      ros::Time time_st;
      pcl_conversions::fromPCL( cloud->header.stamp, time_st);
      tf_listener_.waitForTransform( GLOBAL_FRAME_ID, cloud->header.frame_id, time_st, ros::Duration(3.0));

      ret = pcl_ros::transformPointCloud( GLOBAL_FRAME_ID, *cloud, *cloud_transformed, tf_listener_);
      //ROS_ERROR_STREAM("Return val: " << ret << " now " << time_st << " header " << cloud->header.stamp);
      if( !ret )
      {
        ROS_ERROR("Error converting to desired frame");
        return cloud_transformed;
      }

      // return the point cloud
      return cloud_transformed;
    }

    /*============================================================================*/
    /*===== limit points to table ================================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr limit_points_to_table(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_nan_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

      // remove nan values
      std::vector<int> mapping;
      pcl::removeNaNFromPointCloud(*input_cloud, *removed_nan_cloud, mapping);

      // passthrough filter from pcl
      pcl::PassThrough<pcl::PointXYZRGB> pass;

      // filter in z axis
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredZ(new pcl::PointCloud<pcl::PointXYZRGB>);
      pass.setInputCloud( removed_nan_cloud);
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
    /*===== smooth the surfaces ==================================================*/
    /*============================================================================*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smooth_surfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {
      // Object for storing the point cloud.
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

      // Smoothing object (we choose what point types we want as input and output).
      pcl::MovingLeastSquares <pcl::PointXYZRGB, pcl::PointXYZRGB> filter;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;

      filter.setInputCloud(input_cloud);
      filter.setSearchRadius(smoothing_radius_);
      filter.setPolynomialFit(true);
      filter.setSearchMethod(kdtree);

      filter.process(*output_cloud);

      // remove nan values
     /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_nan_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
      boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
      pcl::removeNaNFromPointCloud(*output_cloud, *indices);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;

      extract.setInputCloud(output_cloud);
      extract.setIndices(indices);
      extract.setNegative( false);
      extract.filter(*removed_nan_cloud);*/

      //ROS_INFO_STREAM( input_cloud->width << " - " << output_cloud->width << " | " << input_cloud->height << " - " << output_cloud->height);

      return output_cloud;
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
    pcl::PointCloud< pcl::PointXYZHSV>::Ptr rgb_to_hsv( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud (new pcl::PointCloud<pcl::PointXYZHSV>());
      pcl::PointCloudXYZRGBtoXYZHSV(*input_cloud, *hsv_cloud); // converts the rgb value to the hsv value -> misses to copy all the other informations

      hsv_cloud->header = input_cloud->header;
      for( int i = 0; i < input_cloud->points.size(); i++)
      {
        //ROS_INFO_STREAM( "x " << input_cloud->points.at(i).x << " y " << input_cloud->points.at(i).y << " z " << input_cloud->points.at(i).z);
        hsv_cloud->points.at(i).x = input_cloud->points.at(i).x;
        hsv_cloud->points.at(i).y = input_cloud->points.at(i).y;
        hsv_cloud->points.at(i).z = input_cloud->points.at(i).z;
      }

      hsv_cloud->width                = input_cloud->width;
      hsv_cloud->height               = input_cloud->height;
      hsv_cloud->is_dense             = input_cloud->is_dense;
      hsv_cloud->sensor_origin_       = input_cloud->sensor_origin_ ;
      hsv_cloud->sensor_orientation_  = input_cloud->sensor_orientation_ ;

      //ROS_INFO_STREAM(" RGB size " << input_cloud->points.size() << " HSV size " << hsv_cloud->points.size());
      /*ROS_INFO_STREAM(" RGB to HSV point 0 -  x " << input_cloud->points.at(0).x << " = " << hsv_cloud->points.at(0).x <<
                                                  " y " << input_cloud->points.at(0).y << " = " << hsv_cloud->points.at(0).y <<
                                                  " z " << input_cloud->points.at(0).z << " = " << hsv_cloud->points.at(0).z <<
                                                  " r " << input_cloud->points.at(0).r << " h " << hsv_cloud->points.at(0).h );*/

      return hsv_cloud;
    }

    pcl::PointCloud< pcl::PointXYZRGB>::Ptr hsv_to_rgb( pcl::PointCloud<pcl::PointXYZHSV>::Ptr input_cloud)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

      rgb_cloud->header = input_cloud->header;

      for( int i = 0; i < input_cloud->points.size(); i++)
      {
        pcl::PointXYZRGB rgb_point;
        pcl::PointXYZHSVtoXYZRGB( input_cloud->points.at(i), rgb_point);

        rgb_cloud->points.push_back( rgb_point);
      }

      rgb_cloud->width                = input_cloud->width;
      rgb_cloud->height               = input_cloud->height;
      rgb_cloud->is_dense             = input_cloud->is_dense;
      rgb_cloud->sensor_origin_       = input_cloud->sensor_origin_ ;
      rgb_cloud->sensor_orientation_  = input_cloud->sensor_orientation_ ;

      //ROS_INFO_STREAM(" hsv_to_rgb header " << rgb_cloud->header);

      return rgb_cloud;
    }

    static int get_hue_color( double hue)
    {

      int color = UNKNOWN_COLOR;

      if( hue < hue_tol_) hue += 360;

      if( hue > (hue_red_ - hue_tol_) && hue < (hue_red_ + hue_tol_))
        color = perception::Block::RED;

      if( hue > (hue_blue_ - hue_tol_) && hue < (hue_blue_ + hue_tol_))
        color = perception::Block::BLUE;

      if( hue > (hue_green_ - hue_tol_) && hue < (hue_green_ + hue_tol_))
        color = perception::Block::GREEN;

      if( hue > (hue_yellow_ - hue_tol_) && hue < (hue_yellow_ + hue_tol_))
        color = perception::Block::YELLOW;

      return color;
    }

    // If this function returns true, the candidate point will be added
    // to the cluster of the seed point.
    static bool color_condition(const pcl::PointXYZHSV& seedPoint, const pcl::PointXYZHSV& candidatePoint, float squaredDistance)
    {

      int seedColor = get_hue_color( seedPoint.h);
      int candidateColor = get_hue_color( candidatePoint.h);

      // Do whatever you want here.
      //ROS_INFO_STREAM( "Seed h = " << seedPoint.h << " color " << seedColor << " candidate h = " << candidatePoint.h << " color " << canditateColor);
      if ( seedColor == UNKNOWN_COLOR || seedColor != candidateColor ||
           candidatePoint.s < 0.3 || candidatePoint.v > 0.9 )
        return false;

      return true;
    }

    std::vector< pcl::PointCloud<pcl::PointXYZHSV>::Ptr >* color_segmentation(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
    {

      // return a vector of point cloud pointers. Each pc includes points of one colored cluster
      hsv_cloud_ = rgb_to_hsv( input_cloud);

      //ROS_INFO_STREAM("color_seg - removed nan and rgb to hsv - " << hsv_cloud_->header << " - " << hsv_cloud_->width << " - " << hsv_cloud_->height << " - " << hsv_cloud_->is_dense);

      //ROS_INFO_STREAM("index " << std::to_string(mapping.at(0)) << " length " << mapping.size());
      /*for( int i = 0; i < hsv_cloud_->points.size(); i++) {
        ROS_INFO_STREAM( "i " << i << " - " << mapping.at(i) << " x " << hsv_cloud_->points.at(i).x << " y " << hsv_cloud_->points.at(i).y << " z " << hsv_cloud_->points.at(i).z);
      }*/

      std::vector <pcl::PointIndices> clusters_hsv;
      std::vector< pcl::PointCloud<pcl::PointXYZHSV>::Ptr >* cloud_clusters_hsv( new std::vector< pcl::PointCloud<pcl::PointXYZHSV>::Ptr >);

      // Conditional Euclidean clustering object.
      pcl::ConditionalEuclideanClustering<pcl::PointXYZHSV> clustering_hsv;

      clustering_hsv.setInputCloud( hsv_cloud_);
      clustering_hsv.setClusterTolerance( col_seg_dist_threshold_);
      clustering_hsv.setMinClusterSize(   col_seg_min_cluster_size_);
      clustering_hsv.setMaxClusterSize(   col_seg_max_cluster_size_);
      clustering_hsv.setConditionFunction( &color_condition);

      clustering_hsv.segment( clusters_hsv);

      //ROS_INFO_STREAM("color_seg - clustered");

      // For every cluster...
      int currentClusterNum = 1;
      for (std::vector<pcl::PointIndices>::const_iterator i = clusters_hsv.begin(); i != clusters_hsv.end(); ++i)
      {
        // ...add all its points to a new cloud...
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZHSV>);

        for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
          cluster->points.push_back( hsv_cloud_->points[*point]);

        cluster->header   = hsv_cloud_->header;
        cluster->width    = cluster->points.size();
        cluster->height   = 1;
        cluster->is_dense = true;

        // ...and add it to the return vector
        cloud_clusters_hsv->push_back( cluster);

        //ROS_INFO_STREAM( " cluster nr. " << currentClusterNum << " size " << cluster->points.size());

        currentClusterNum++;
      }

      return cloud_clusters_hsv;
    }

    /*============================================================================*/
    /*====== extract position orientation and color of the objects ===============*/
    /*============================================================================*/
    void cloud_statistics_size( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,
                           double* x_min, double* x_max, double* x_mean,
                           double* y_min, double* y_max, double* y_mean,
                           double* z_min, double* z_max, double* z_mean)
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
      }

      *x_mean = *x_mean / N;
      *y_mean = *y_mean / N;
      *z_mean = *z_mean / N;

      /*ROS_INFO_STREAM( "FILTER:  x min: " << *x_min << " mean: " << *x_mean <<
                                          " max: " << *x_max );
      ROS_INFO_STREAM( "FILTER:  y min: " << *y_min << " mean: " << *y_mean <<
                                          " max: " << *y_max );
      ROS_INFO_STREAM( "FILTER:  z min: " << *z_min << " mean: " << *z_mean <<
                                          " max: " << *z_max );
      ROS_INFO_STREAM( "FILTER:  hue min: " << *hue_min << " mean: " << *hue_mean <<
                                          " max: " << *hue_max );*/
    }

    void cloud_statistics_color( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,
                           double* hue_min, double* hue_max, double* hue_mean)
    {
      int N = cloud->points.size();

      *hue_min = cloud->points.at(0).h;
      *hue_max = cloud->points.at(0).h;
      *hue_mean = 0.0;

      for(int j = 0; j < N; j++)
      {
        *hue_min = std::min( *hue_min, (double)cloud->points.at(j).h);
        *hue_max = std::max( *hue_max, (double)cloud->points.at(j).h);

        if( cloud->points.at(j).h < hue_tol_)
          *hue_mean += cloud->points.at(j).h + 360;
        else
          *hue_mean += cloud->points.at(j).h;
      }

      *hue_mean = *hue_mean / N;

      /*ROS_INFO_STREAM( "FILTER:  x min: " << *x_min << " mean: " << *x_mean <<
                                          " max: " << *x_max );
      ROS_INFO_STREAM( "FILTER:  y min: " << *y_min << " mean: " << *y_mean <<
                                          " max: " << *y_max );
      ROS_INFO_STREAM( "FILTER:  z min: " << *z_min << " mean: " << *z_mean <<
                                          " max: " << *z_max );
      ROS_INFO_STREAM( "FILTER:  hue min: " << *hue_min << " mean: " << *hue_mean <<
                                          " max: " << *hue_max );*/
    }

    int get_block_type( double width, double height, double depth)
    {
      int type_id = BLOCK_TYPE_UNKNOWN;

      for (auto &block_type : block_types_) {
        if (((width  < (block_type.width + block_type_tol_)  && width  > (block_type.width - block_type_tol_) &&
              depth  < (block_type.depth + block_type_tol_)  && depth  > (block_type.depth - block_type_tol_)) ||
             (depth  < (block_type.width + block_type_tol_)  && depth  > (block_type.width - block_type_tol_) &&
              width  < (block_type.depth + block_type_tol_)  && width  > (block_type.depth - block_type_tol_)))&&
            height < (block_type.height + block_type_tol_) && height > (block_type.height - block_type_tol_))
          type_id = block_type.id;
      }

      ROS_INFO_STREAM( "FILTER: get block type  width " << width << " depth " << depth << " height " << height << " type " << type_id );
      return type_id;
    }

    int get_block_type_from_front( double width, double height, double* depth)
    {
      int type_id = BLOCK_TYPE_UNKNOWN;

      for (auto &block_type : block_types_) {

        //ROS_INFO_STREAM( "FILTER: get block type from front  width " << block_type.width << " - " << width << " depth " << block_type.depth << " - " << *depth << " height " << block_type.height << " - " << height );

        if((width  < (block_type.width + block_type_tol_)  && width  > (block_type.width - block_type_tol_) ||
            width  < (block_type.depth + block_type_tol_)  && width  > (block_type.depth - block_type_tol_))&&
            height < (block_type.height + block_type_tol_) && height > (block_type.height - block_type_tol_) &&
            type_id == BLOCK_TYPE_UNKNOWN) {
          type_id = block_type.id;
          *depth = block_type.depth;
        }
      }

      ROS_INFO_STREAM( "FILTER: get block type from front  width " << width << " depth " << *depth << " height " << height << " type " << type_id );
      return type_id;
    }

    void split_block_into_planes( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud, pcl::PointCloud<pcl::PointXYZHSV>::Ptr top,
                                  pcl::PointCloud<pcl::PointXYZHSV>::Ptr left,  pcl::PointCloud<pcl::PointXYZHSV>::Ptr right,
                                  pcl::ModelCoefficients::Ptr top_coefs,        pcl::ModelCoefficients::Ptr left_coefs,
                                  pcl::ModelCoefficients::Ptr right_coefs)
    {
      // extract planes of cuboid
      // Create the segmentation object for the planar model;
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_to_split( new pcl::PointCloud<pcl::PointXYZHSV>());
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inlierInds(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZHSV> seg;

      *cloud_to_split = *cloud;

      // set all parameters
      seg.setInputCloud( cloud_to_split);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold( block_plane_dist_threshold_);
      seg.setOptimizeCoefficients( true);
      seg.setMaxIterations( block_plane_max_iterations_);

      int dir = 0;

      do{

        // get indices of points lying in the current plane with the coefficients
        seg.segment(*inlierInds, *coefficients);

        if( coefficients->values.at(2) >= 0.9 )
          dir = 1; // top plane
        else if( coefficients->values.at(0) > coefficients->values.at(1))
          dir = 2;
        else
          dir = 3;

        //ROS_INFO_STREAM( "nr. of inliers " << inlierInds->indices.size() << " coeffs " << coefficients->values.at(0) << " | " << coefficients->values.at(1) << " | " <<  coefficients->values.at(2) << " | " <<  coefficients->values.at(3) << " dir " << std::to_string(dir));

        if (inlierInds->indices.size() > block_plane_min_points_){
          // Extract the planar inliers from the input cloud
          pcl::ExtractIndices<pcl::PointXYZHSV> extract;
          extract.setInputCloud( cloud_to_split);
          extract.setIndices( inlierInds);

          // Extract the points in the plane
          extract.setNegative(false);
          switch (dir) {
            case 1:
              if( top->points.size() < inlierInds->indices.size()) {
                extract.filter(*top);
                top_coefs = coefficients;
              }
              break;
            case 2:
              if( left->points.size() < inlierInds->indices.size()) {
                extract.filter(*left);
                left_coefs = coefficients;
              }
              break;
            case 3:
              if( right->points.size() < inlierInds->indices.size()){
                extract.filter(*right);
                right_coefs = coefficients;
              }
              break;
          }
          // Extract the points which don't belong to the plane
          extract.setNegative(true);
          extract.filter( *cloud_to_split);
        }

        //ROS_INFO_STREAM( "cloud to split size " << cloud_to_split->points.size() << " top sizes " << top->points.size() << " left sizes " << left->points.size() << " right sizes " << right->points.size());
      } while ( inlierInds->indices.size() > block_plane_min_points_ &&
                cloud_to_split->points.size() > block_plane_min_points_);
    }

    void object_extraction( std::vector< pcl::PointCloud<pcl::PointXYZHSV>::Ptr >* color_clusters, bool* success,
                            std::vector<perception::Block>* blocks)
    {
      ROS_INFO_STREAM( "FILTER: object_extraction " << color_clusters->size() );

      *success = false;

      // for each cluster
      for(int i = 0; i < color_clusters->size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud = color_clusters->at(i);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr top( new pcl::PointCloud<pcl::PointXYZHSV>());
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr left( new pcl::PointCloud<pcl::PointXYZHSV>());
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr right( new pcl::PointCloud<pcl::PointXYZHSV>());
        pcl::ModelCoefficients::Ptr top_coeffs(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr left_coeffs(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr right_coeffs(new pcl::ModelCoefficients);
        perception::Block block;

        split_block_into_planes( cloud, top, left, right, top_coeffs, left_coeffs, right_coeffs);

        if(i == 0)
        {
          b1_top_   = top;
          b1_left_  = left;
          b1_right_ = right;

          b1_top_->header = cloud->header;
          b1_left_->header = cloud->header;
          b1_right_->header = cloud->header;
        }

        //TODO evaluate each side of the block

        ROS_INFO_STREAM( "FILTER: cloud size " << cloud->points.size() << " top " << top->points.size() << " left " << left->points.size() << " right " << right->points.size() );


        /*plot_x_distribution( cloud);
        plot_y_distribution( cloud);
        plot_z_distribution( cloud);*/

        if( left->points.size() > 0 && top->points.size() == 0)
        {
          //only front plane available

          // get statistic values
          double x_min, x_max, x_mean;
          double y_min, y_max, y_mean;
          double z_min, z_max, z_mean;

          cloud_statistics_size(left, &x_min, &x_max, &x_mean, &y_min, &y_max, &y_mean, &z_min, &z_max, &z_mean);

          // extract size and check if plausible
          block.width = std::max( (x_max - x_min), (y_max - y_min));
          //block.depth = y_max - y_min;
          block.height = z_max - z_min;

          block.type = get_block_type_from_front(block.width, block.height, &block.depth);

          if (block.type == BLOCK_TYPE_UNKNOWN)
            continue;

          // extract position and orientation
          geometry_msgs::PoseStamped poseStamped;
          poseStamped.header.frame_id = GLOBAL_FRAME_ID;
          poseStamped.header.stamp = ros::Time::now();

          poseStamped.pose.position.x = x_mean;
          poseStamped.pose.position.y = y_min + block.depth/2;
          poseStamped.pose.position.z = z_mean;

          double theta = 0.0;
          tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, theta);
          poseStamped.pose.orientation.w = q.getW();
          poseStamped.pose.orientation.x = q.getX();
          poseStamped.pose.orientation.y = q.getY();
          poseStamped.pose.orientation.z = q.getZ();

          block.pose = poseStamped;

        }
        else {
          //front and top plane available

          // get statistic values
          double x_min, x_max, x_mean;
          double y_min, y_max, y_mean;
          double z_min, z_max, z_mean;

          cloud_statistics_size(cloud, &x_min, &x_max, &x_mean, &y_min, &y_max, &y_mean, &z_min, &z_max, &z_mean);

          // extract size and check if plausible
          block.width = x_max - x_min  -0.005;
          block.depth = y_max - y_min  -0.005;
          block.height = z_max - z_min -0.005;

          block.type = get_block_type(block.width, block.height, block.depth);

          if (block.type == BLOCK_TYPE_UNKNOWN)
            continue;

          // extract position and orientation
          geometry_msgs::PoseStamped poseStamped;
          poseStamped.header.frame_id = GLOBAL_FRAME_ID;
          poseStamped.header.stamp = ros::Time::now();

          poseStamped.pose.position.x = x_mean;
          poseStamped.pose.position.y = y_mean;
          poseStamped.pose.position.z = z_mean;

          double theta = 0.0;
          tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, theta);
          poseStamped.pose.orientation.w = q.getW();
          poseStamped.pose.orientation.x = q.getX();
          poseStamped.pose.orientation.y = q.getY();
          poseStamped.pose.orientation.z = q.getZ();

          block.pose = poseStamped;

        }

        // extract color
        double hue_min, hue_max, hue_mean;
        cloud_statistics_color(cloud, &hue_min, &hue_max, &hue_mean);
        block.color = get_hue_color( hue_mean);

        ROS_INFO_STREAM( "block color " << hue_mean << " nr " << std::to_string(block.color) );
        //plot_color_distribution_hsv( cloud);

        if(block.color == UNKNOWN_COLOR)
          continue;

        block.id = block.color*10 + block.type;

        blocks->push_back(block);
        *success = true;
      }
    }

    /*============================================================================*/
    /*====== process the point cloud =============================================*/
    /*============================================================================*/

    void process_latest_msg( bool *success,  std::vector<perception::Block>* blocks)
    {
      ROS_INFO_STREAM( " \nprocess latest msg\n\n");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      *success = false;

      // Load all parameters
      if( !load_parameters())
        return;

      //handle debug mode with rosbag (timestamps wrong!)
      if( debug_)
        latest_sensor_msg_.header.stamp = ros::Time::now();

      //check if message is up to date
      if( latest_sensor_msg_.header.frame_id.empty() ||
          (ros::Time::now().toSec() - latest_sensor_msg_.header.stamp.toSec() ) > 200)
      {
        ROS_ERROR_STREAM("FILTER: No current camera data is available  " << ros::Time::now().toSec() << " - " <<
                          latest_sensor_msg_.header.stamp.toSec() << " = " <<
                         (ros::Time::now().toSec() - latest_sensor_msg_.header.stamp.toSec()));
        return;
      }

      // convert msgs to point cloud
      pcl::fromROSMsg( latest_sensor_msg_, *input_cloud);
      input_pc_pub_.publish( *input_cloud);

      // transform to baxter axis
      transformed_cloud_ = transform_to_baxters_axis( input_cloud);
      ROS_INFO_STREAM( "FILTER: transformed_cloud: " << transformed_cloud_->points.size() );
      if (transformed_cloud_->points.size() == 0)   return;

      // Limit points corresponding to table
      trimmed_cloud_ = limit_points_to_table( transformed_cloud_);
      ROS_INFO_STREAM( "FILTER: trimmed_cloud: " << trimmed_cloud_->points.size() );
      if (trimmed_cloud_->points.size() == 0)  return;

      /*// surface smoothing
      smoothed_cloud_ = smooth_surfaces( trimmed_cloud_);
      ROS_INFO_STREAM( "FILTER: smoothed_cloud: " << smoothed_cloud_->points.size() );
      if (smoothed_cloud_->points.size() == 0)   return;*/

      // remove table surface (planar model) from the cloud
      table_removed_cloud_ = plane_segmentation( trimmed_cloud_);
      ROS_INFO_STREAM( "FILTER: table_removed_cloud: " << table_removed_cloud_->points.size() );
      if (table_removed_cloud_->points.size() == 0)   return;

      /*plot_color_distribution_rgb( table_removed_cloud_);
      plot_x_distribution( table_removed_cloud_);
      plot_x_distribution_hsv( hsv_cloud_);*/

      // segmentation by color
      color_clusters_ = color_segmentation( table_removed_cloud_);
      ROS_INFO_STREAM( "FILTER: color_segementation nr of clusters: " << color_clusters_->size() );
      if (color_clusters_->size() == 0)
        return;

      //plot_color_distribution_hsv( color_clusters_->at(0));
      //plot_color_distribution_hsv( color_clusters_->at(1));

      // extract object information
      object_extraction( color_clusters_, success, blocks );
    }


    /*============================================================================*/
    /*====== callbacks ===========================================================*/
    /*============================================================================*/
    void sensor_msg_callback (const sensor_msgs::PointCloud2ConstPtr& input)
    {
      int duration = 30;
      count_ ++;

      if( debug_) duration = 1;//00;

      if(count_ >= duration) {
        //ROS_INFO_STREAM("sensor_msg_callback: \n" << latest_sensor_msg.header);
        bool success = false;
        std::vector<perception::Block>* blocks = new std::vector<perception::Block>;

        latest_sensor_msg_ = *input;
        process_latest_msg( &success, blocks);

        // store data for service call
        blocks_lock_.lock();
        success_ = success;
        blocks_ = blocks;
        blocks_lock_.unlock();

        count_ = 0;
      }
    }

    bool get_scene_callback( perception::GetScene::Request  &req,
                             perception::GetScene::Response &res)
    {
      ROS_INFO_STREAM( "FILTER: requested get scene service" );

      //grab latest result
      blocks_lock_.lock();
      res.success = success_;
      res.blocks = *blocks_;
      blocks_lock_.unlock();

      //TODO check if data is not outdated (blocks timestamp)

      ROS_INFO_STREAM( "FILTER: sending back response \n");// << res.blocks);
      return true;
    }

    void timer_callback(const ros::TimerEvent& event)
    {
      /*ROS_INFO_STREAM( "FILTER: periodical request from timer" );

      bool success = false;
      std::vector<perception::Block>* blocks = new std::vector<perception::Block>;

      //process latest sensor message
      process_latest_msg( &success, blocks);*/

      if( transformed_cloud_ ) {
        transformed_pc_pub_.publish(*transformed_cloud_);
      }

      if( smoothed_cloud_) {
        smoothed_pc_pub_.publish(*smoothed_cloud_);
      }

      if( trimmed_cloud_ ) {
        trimmed_pc_pub_.publish(*trimmed_cloud_);
      }

      if( table_removed_cloud_) {
        table_removed_pc_pub_.publish(*table_removed_cloud_);
      }


      //rviz can only display RGB points -> therefore transfer it back to rgb color space
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());


      if( color_clusters_!= nullptr) {
        if (color_clusters_->size() >= 1) {
          rgb_cloud = hsv_to_rgb(color_clusters_->at(0));
          block1_pc_pub_.publish(rgb_cloud);
        }
        if (color_clusters_->size() >= 2) {
          rgb_cloud = hsv_to_rgb(color_clusters_->at(1));
          block2_pc_pub_.publish(rgb_cloud);
        }
        if (color_clusters_->size() >= 3) {
          rgb_cloud = hsv_to_rgb(color_clusters_->at(2));
          block3_pc_pub_.publish(rgb_cloud);
        }
        if (color_clusters_->size() >= 4) {
          rgb_cloud = hsv_to_rgb(color_clusters_->at(3));
          block4_pc_pub_.publish(rgb_cloud);
        }
        if (color_clusters_->size() >= 5) {
          rgb_cloud = hsv_to_rgb(color_clusters_->at(4));
          block5_pc_pub_.publish(rgb_cloud);
        }
      }

      if ( b1_top_) {
        rgb_cloud = hsv_to_rgb( b1_top_);
        block1_top_pc_pub_.publish(rgb_cloud);
      }

      if ( b1_left_) {
        rgb_cloud = hsv_to_rgb( b1_left_);
        block1_left_pc_pub_.publish(rgb_cloud);
      }

      if ( b1_right_) {
        rgb_cloud = hsv_to_rgb( b1_right_);
        block1_right_pc_pub_.publish(rgb_cloud);
      }
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

      plot1_->clearPlots();
      plot1_->addHistogramData(r, 255, "red");
      plot1_->setShowLegend(true);
      plot1_->plot();

      plot2_->clearPlots();
      plot2_->addHistogramData(g, 255, "green");
      plot2_->setShowLegend(true);
      plot2_->plot();

      plot3_->clearPlots();
      plot3_->addHistogramData(b, 255, "blue");
      plot3_->setShowLegend(true);
      plot3_->plot();
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

      plot4_->clearPlots();
      plot4_->addHistogramData(h, 255, "hue", std::vector<char>{127,0,0,127});
      plot4_->setShowLegend(true);
      plot4_->plot();

      plot5_->clearPlots();
      plot5_->addHistogramData(s, 255, "saturation", std::vector<char>{0,127,0,127});
      plot5_->setShowLegend(true);
      plot5_->plot();

      plot6_->clearPlots();
      plot6_->addHistogramData(v, 255, "value", std::vector<char>{0,0,127,127});
      plot6_->setShowLegend(true);
      plot6_->plot();
    }

    void plot_x_distribution( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
      std::vector<double> x;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        x.push_back( cloud->points.at(i).x);
      }

      plot1_->clearPlots();
      plot1_->addHistogramData(x, 100, "x");
      plot1_->setShowLegend(true);
      plot1_->plot();
    }

    void plot_y_distribution( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> y;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        y.push_back( cloud->points.at(i).y);
      }

      plot2_->clearPlots();
      plot2_->addHistogramData(y, 100, "y");
      plot2_->setShowLegend(true);
      plot2_->plot();
    }

    void plot_z_distribution( pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
      // plot red distribution of points
      std::vector<double> z;

      int N = cloud->points.size();

      for(int i = 0; i < N; i++)
      {
        z.push_back( cloud->points.at(i).z);
      }

      plot3_->clearPlots();
      plot3_->addHistogramData(z, 100, "z");
      plot3_->setShowLegend(true);
      plot3_->plot();
    }

  public:

    Filter(ros::NodeHandle nh, std::string cmd):
      nh_(nh),
      table_x_low_(0.44), table_x_high_(1.24),
      table_y_low_(-0.5), table_y_high_(0.5),
      table_z_low_(-0.225), table_z_high_(0.1),
      smoothing_radius_(0.0025),
      plane_seg_threshold_(0.01),
      plane_seg_max_iterations_(10),
      col_seg_min_cluster_size_(100),
      col_seg_max_cluster_size_(2000),
      col_seg_dist_threshold_(0.015),
      block_plane_dist_threshold_(0.003),
      block_plane_max_iterations_(10),
      block_plane_min_points_(30),
      block_type_tol_(0.01)
    {
      color_clusters_ = nullptr;

      //ROS_INFO_STREAM( "FILTER: start constructor" );
      hue_red_   = 355;
      hue_blue_  = 220;
      hue_green_ = 85;
      hue_yellow_= 35;
      hue_tol_   = 15;

      if( !cmd.empty() && cmd == "debug")
      {
        ROS_INFO("FILTER: entering debug mode");
        debug_ = true;
      }else
        debug_ = false;

      // Get the transform listener before the first sensor message can arrive to avoid extrapolation into the past error
      tf_listener_.waitForTransform( GLOBAL_FRAME_ID, CAMERA_FRAME_ID, ros::Time::now(), ros::Duration(3.0));
      ros::Duration(0.25).sleep();

      // Create a ROS subscriber for the input point cloud and sleep for a second so the camera publishes good data
      point_cloud_sub_ = nh_.subscribe ("/camera/depth_registered/points", 1, &Filter::sensor_msg_callback, this);
      ros::Duration(0.75).sleep();

      // Create a timer to periodically process the latest scene
      timer_ = nh_.createTimer(ros::Duration(0.5), &Filter::timer_callback, this);

      // advertise new ros service GetScene
      service_ = nh_.advertiseService("get_scene", &Filter::get_scene_callback, this);
      ROS_INFO( "FILTER: Service get_scene activated!");
      
      // Create a ROS publisher for the output point cloud
      input_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_input", 1);
      transformed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_transformed", 1);
      smoothed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_smoothed", 1);
      trimmed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_trimmed", 1);
      table_removed_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_table_removed", 1);
      block1_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_block1", 1);
      block2_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_block2", 1);
      block3_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_block3", 1);
      block4_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_block4", 1);
      block5_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_block5", 1);
      block1_top_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_b1_top", 1);
      block1_left_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_b1_left", 1);
      block1_right_pc_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB>>("/filter_b1_right", 1);

      plot1_ = new pcl::visualization::PCLPlotter();
      plot2_ = new pcl::visualization::PCLPlotter();
      plot3_ = new pcl::visualization::PCLPlotter();
      plot4_ = new pcl::visualization::PCLPlotter();
      plot5_ = new pcl::visualization::PCLPlotter();
      plot6_ = new pcl::visualization::PCLPlotter();
      //ROS_INFO_STREAM( "FILTER: end constructor" );

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

  //parse calling arguments
  std::string cmd;
  if(argc >=2) cmd = argv[1];

  Filter* filter = new Filter( nh, cmd);
  ROS_INFO("FILTER: init done");

  // Spin
  ros::spin();

  return 0;
}
