#pragma once
#include <ros/ros.h>
#include "l_shape_tracker.hpp"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "datmo/Track.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class Cluster {
public:

  Cluster(unsigned long int id, const pointList&, const double&, const std::string&, const tf::Transform& ego_pose);


  std::string frame_name;
  Point ego_coordinates;

  datmo::Track msg_track_box_kf;

  unsigned long int id; //identifier for the cluster 
  unsigned long int age; //age of the cluster 
  float r, g, b, a; //color of the cluster

  visualization_msgs::Marker getBoundingBoxCenterVisualisationMessage();
  visualization_msgs::Marker getClosestCornerPointVisualisationMessage();
  visualization_msgs::Marker getClusterVisualisationMessage();
  visualization_msgs::Marker getLineVisualisationMessage();
  visualization_msgs::Marker getArrowVisualisationMessage();
  visualization_msgs::Marker getThetaL2VisualisationMessage();
  visualization_msgs::Marker getThetaL1VisualisationMessage();
  visualization_msgs::Marker getThetaBoxVisualisationMessage();
  visualization_msgs::Marker getBoundingBoxVisualisationMessage();
  visualization_msgs::Marker getBoxModelKFVisualisationMessage();
  visualization_msgs::Marker getLShapeVisualisationMessage();
  visualization_msgs::Marker getBoxSolidVisualisationMessage();

  void update(const pointList&, const double dt, const tf::Transform& ego_pose);

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.
  double meanX() { return mean_values.first; };
  double meanY() { return mean_values.second;};

  LshapeTracker Lshape; 

  double old_thetaL1;
  double L1, L2, thetaL1, thetaL2;
  double cx, cy, cvx, cvy, L1_box, L2_box, th, psi, comega, length_box, width_box; 
  double x_ukf,  y_ukf, vx_ukf,  vy_ukf, omega_ukf;

private:

  pointList new_cluster;
  std::vector<Point> corner_list;
  tf2::Quaternion quaternion; //used for transformations between quaternions and angles
  std::vector<Point> l1l2; //save coordinates of the three points that define the lines

  // mean value of the cluster
  std::pair<double, double> mean_values;
  std::pair<double, double> previous_mean_values;

  Point closest_corner_point;
  

  visualization_msgs::Marker boxcenter_marker_;
  void populateTrackingMsgs(const double& dt);
  void calcMean(const pointList& ); //Find the mean value of the cluster
  void rectangleFitting(const pointList& ); //Search-Based Rectangle Fitting 
  double areaCriterion(const VectorXd&, const VectorXd& );
  double closenessCriterion(const VectorXd& ,const VectorXd&, const double& );
  Point lineIntersection(double& , double& , double& , double& , double& , double& );
  double perpendicularDistance(const Point&, const Point&, const Point&);
  void ramerDouglasPeucker(const std::vector<Point>&, double, std::vector<Point>&);
};
