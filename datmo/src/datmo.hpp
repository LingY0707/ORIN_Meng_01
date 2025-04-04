#include <ros/ros.h>
#include <math.h>      
#include <time.h> 
#include <omp.h>      
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <datmo/TrackArray.h>
#include <datmo/Track.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>

#include "cluster.hpp"

#include "dkt_msgs/DetectedObjectArray.h"
#include "dkt_msgs/DetectedObject.h"

typedef std::pair<double, double> Point;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;

using namespace std;

class Datmo
{
public:
  Datmo();
  ~Datmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void segObj_callback(const dkt_msgs::DetectedObjectArray &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  void transformPointList(const pointList& , pointList& );

  tf::TransformListener tf_listener;
private:
  ros::Publisher pub_marker_array; 
  ros::Publisher pub_tracks_box_kf;
  ros::Publisher pub_object_array_;
  ros::Subscriber sub_scan;
  sensor_msgs::LaserScan scan;
  vector<Cluster> clusters;
  dkt_msgs::DetectedObjectArray final_objects;


  
  double dt;
  tf::StampedTransform ego_pose;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers

  //Parameters
  float roi_m_;
  float pic_scale_;
  double dth;
  double euclidean_distance;
  int max_cluster_size;
  bool p_marker_pub;
  bool w_exec_times;
  string lidar_frame;
  string world_frame;
  string in_topic;
  string out_topic;
  std::vector<std::pair<double, double>> object_size;
};
