#include "datmo.hpp"


Datmo::Datmo(){
  ros::NodeHandle n; 
  ros::NodeHandle n_private("~");
  ROS_INFO("Starting Detection And Tracking of Moving Objects");

  n_private.param("max_cluster_size", max_cluster_size, 360);
  n_private.param("euclidean_distance", euclidean_distance, 10.0);
  n_private.param("in_topic", in_topic, string("/detection/lidar_detector/objects"));
  n_private.param("out_topic", out_topic, string("/detection/l_shaped/objects"));

  object_size.resize(6);
  object_size = {{1, 1}, {1.9, 4.0}, {3.9, 8.0}, {4, 7}, {4, 7}, {1, 1}};

  sub_scan = n.subscribe(in_topic, 1, &Datmo::segObj_callback, this);
  pub_object_array_ = n.advertise<dkt_msgs::DetectedObjectArray>(out_topic, 1);

  clusters.clear();
  final_objects.objects.clear();

  dt = 0.08;

  ego_pose.setIdentity();
}

Datmo::~Datmo(){
}

void Datmo::segObj_callback(const dkt_msgs::DetectedObjectArray &in_object_array){

  vector<pointList> point_clusters;

  //Get Point_Clusters
  for (const auto& in_object : in_object_array.objects)
  {
    if(in_object.label == "person"){
      // std::cout<<"detected person! continue!"<<std::endl;
      continue;
    }
      
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert from ros msg to PCL::pic_scalePointCloud data type
    pcl::fromROSMsg(in_object.pointcloud, cloud);

    int num_points = cloud.size();
    pointList point_vec(num_points);

    for (int i_point = 0; i_point < num_points; i_point++)
    {
      Point offset_point_tmp;
      offset_point_tmp.first = cloud[i_point].x;
      offset_point_tmp.second = cloud[i_point].y;

      point_vec[i_point] = offset_point_tmp;
    }
    point_clusters.push_back(point_vec);
  }

  // Cluster Association based on the Euclidean distance

  vector<bool> g_matched(point_clusters.size(),false);   // The Group has been matched with a Cluster
  vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group

  double euclidean[point_clusters.size()][clusters.size()]; // Matrix object to save the euclidean distances

  //Finding mean coordinates of group and associating with cluster Objects
  double mean_x = 0, mean_y = 0;


  for(unsigned int g = 0; g<point_clusters.size();++g){
    double sum_x = 0, sum_y = 0;
      
    for(unsigned int l =0; l<point_clusters[g].size(); l++){
      sum_x = sum_x + point_clusters[g][l].first;
      sum_y = sum_y + point_clusters[g][l].second;
    }
    mean_x = sum_x / point_clusters[g].size();
    mean_y = sum_y / point_clusters[g].size();

    for(unsigned int c=0;c<clusters.size();++c){
      // euclidean[g][c] = abs( mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY()); 
      euclidean[g][c] = sqrt((mean_x - clusters[c].meanX())*(mean_x - clusters[c].meanX()) + (mean_y - clusters[c].meanY())*(mean_y - clusters[c].meanY())); 
    }
  }
  //Find the smallest euclidean distance and associate if smaller than the threshold 
  vector<pair <int,int>> pairs;
  for(unsigned int c=0; c<clusters.size();++c){
    unsigned int position;
    double min_distance = euclidean_distance;
    for(unsigned int g=0; g<point_clusters.size();++g){
      if(euclidean[g][c] < min_distance && in_object_array.objects[g].classId == final_objects.objects[c].classId){
        min_distance = euclidean[g][c];
        position = g;
      }
    }
    if(min_distance < euclidean_distance){
      g_matched[position] = true, c_matched[c] = true;
      pairs.push_back(pair<int,int>(c,position));
    }
  }
  //Update Tracked Clusters
  #pragma omp parallel for
  for(unsigned int p=0; p<pairs.size();++p){
    clusters[pairs[p].first].update(point_clusters[pairs[p].second], dt, ego_pose);
    final_objects.objects[pairs[p].first] = in_object_array.objects[pairs[p].second];
  }
      
  //Delete Not Associated Clusters
  unsigned int o=0;
  unsigned int p = clusters.size();
  while(o<p){
    if(c_matched[o] == false){

      std::swap(clusters[o], clusters.back());
      clusters.pop_back();

      std::swap(c_matched[o], c_matched.back());
      c_matched.pop_back();

      std::swap(final_objects.objects[o], final_objects.objects.back());
      final_objects.objects.pop_back();

      o--;
      p--;
    }
  o++;
  }

  // Initialisation of new Cluster Objects
  for(unsigned int i=0; i<point_clusters.size();++i){
    if(g_matched[i] == false && point_clusters[i].size()< max_cluster_size){
      Cluster cl(cclusters, point_clusters[i], dt, world_frame, ego_pose);
      cclusters++;
      clusters.push_back(cl);
      final_objects.objects.push_back(in_object_array.objects[i]);
    } 
  }
  //Publish Clusters
  for(int i=0; i<clusters.size(); i++){
    double object_width = object_size[final_objects.objects[i].classId].first;
    double object_length = object_size[final_objects.objects[i].classId].second;
    Cluster cl = clusters[i];
    bool l_w = false;
    // if(cl.msg_track_box_kf.length > cl.msg_track_box_kf.width){
    //   l_w = true;
    // }
    if(abs(cl.msg_track_box_kf.length - object_length) < abs(cl.msg_track_box_kf.width - object_length)){
      l_w = true;
    }
    // final_objects.objects[i].dimensions.x = l_w ? object_length : object_width;
    // final_objects.objects[i].dimensions.y = l_w ? object_width : object_length;
    final_objects.objects[i].dimensions.x = cl.msg_track_box_kf.length;
    final_objects.objects[i].dimensions.y = cl.msg_track_box_kf.width;
    final_objects.objects[i].dimensions.z = 2;
    final_objects.objects[i].pose.position = cl.msg_track_box_kf.odom.pose.pose.position;
    final_objects.objects[i].pose.orientation = cl.msg_track_box_kf.odom.pose.pose.orientation;
    final_objects.objects[i].theta = cl.msg_track_box_kf.odom.twist.twist.angular.z;
    std::cout<<final_objects.objects[i].label<<"     orientation      "<<final_objects.objects[i].pose.orientation<<std::endl;
    // if(final_objects.objects[i].label == "car"){
    //   std::cout<<"     theta      "<<final_objects.objects[i].theta<<std::endl;
    // }
  }
  std::cout<<"-------------"<<std::endl;
  //publish final object array
  final_objects.header = in_object_array.header;
  pub_object_array_.publish(final_objects);
}