#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num, id;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;
uniform_real_distribution<double> rand_ground_height;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<cloudMapT> PointcloudMap;
// pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;
// PointcloudMap::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);

void RandomMapGenerate() {
  //gengshuang begin-------------------------------------------------------------------------------------------
  std::cout << "randomapgenerate "<<std::endl;
  std::string path;
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("pcd_path", path, "/home/lll/flycar_sim/scene" + std::to_string(id) + ".pcd");
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloudMap )== -1)
  {
        PCL_ERROR("Couldn't read pcd file\n");
        return;
  }
  std::cout << "Loaded "
    << cloudMap->width*cloudMap->height
    << " data points from test_pcd.pcd with the following fields: "
    << std::endl;

  // pcl::visualization::CloudViewer viewer("viewer");
  //   viewer.showCloud(cloudMap);
  //   while (!viewer.wasStopped())
  //   {       
  //   }
  //   system("pause");
  //   return;

  //gengshuang end------------------------------------------------------------------------------------------------------------



  // double rd_num = rd();

  // default_random_engine eng(rd());
  // pcl::PointXYZ pt_random;


  // rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  // rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  // rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  // rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  // rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  // rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  // rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  // rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);
  // rand_ground_height = uniform_real_distribution<double>(-0.1, 0.1);

  // generate polar obs
  // for (int i = 0; i < _obs_num; i++) {
  //   double x, y, w, h;
  //   x = rand_x(eng);
  //   y = rand_y(eng);
  //   w = rand_w(eng);

  //   if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
  //     i--;
  //     continue;
  //   }

  //   if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
  //     i--;
  //     continue;
  //   }

  //   x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  //   y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  //   int widNum = ceil(w / _resolution);

  //   for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
  //     for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
  //       h = rand_h(eng);
  //       int heiNum = ceil(h / _resolution);
  //       for (int t = -0.2; t < heiNum; t++) {
  //         pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
  //         pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
  //         pt_random.z = (t + 0.5) * _resolution + 1e-2;
  //         cloudMap.points.push_back(pt_random);
  //       }
  //     }
  // }
  // generate barrier obs
  // double x, y, w_x, w_y, h;
  // x = 5, y = 5;
  // w_x = 0.5;
  // w_y = 5;
  // h = 0.5;
  // double widNum_x = (w_x / _resolution);
  // double widNum_y = (w_y / _resolution);
  //   for (int r = -widNum_x / 2.0; r <= widNum_x / 2.0; r++)
  //     for (int s = -widNum_y / 2.0; s <= widNum_y / 2.0; s++) {
  //       int heiNum = ceil(h / _resolution);
  //       for (int t = -0.2; t < heiNum; t++) {
  //         pt_random.x = x + (r + 0.1) * _resolution + 1e-2;
  //         pt_random.y = y + (s + 0.1) * _resolution + 1e-2;
  //         pt_random.z = (t + 0.1) * _resolution + 1e-2;
  //         cloudMap.points.push_back(pt_random);
  //       }
  // }
  
  // generate circle obs
  // for (int i = 0; i < circle_num_; ++i) {
  //   double x, y, z;
  //   x = rand_x(eng);
  //   y = rand_y(eng);
  //   z = rand_z_(eng);

  //   if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
  //     i--;
  //     continue;
  //   }

  //   if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
  //     i--;
  //     continue;
  //   }

  //   x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  //   y = floor(y / _resolution) * _resolution + _resolution / 2.0;
  //   z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    //Eigen::Vector3d translate(x, y, z);

  //   double theta = rand_theta_(eng);
  //   Eigen::Matrix3d rotate;
  //   rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
  //       1;

  //   double radius1 = z;
  //   double radius2 = z;

  //   // draw a circle centered at (x,y,z)
  //   Eigen::Vector3d cpt;
  //   for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
  //     cpt(0) = 0.0;
  //     cpt(1) = radius1 * cos(angle);
  //     cpt(2) = radius2 * sin(angle);

  //     // inflate
  //     Eigen::Vector3d cpt_if;
  //     for (int ifx = -0; ifx <= 0; ++ifx)
  //       for (int ify = -0; ify <= 0; ++ify)
  //         for (int ifz = -0; ifz <= 0; ++ifz) {
  //           cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
  //                                          ifz * _resolution);
  //           cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
  //           pt_random.x = cpt_if(0);
  //           pt_random.y = cpt_if(1);
  //           pt_random.z = cpt_if(2);
  //           cloudMap.push_back(pt_random);
  //         }
  //   }
  // }

  // gengshuang begin--------------------------------
  cloudMap->width = cloudMap->points.size();
  cloudMap->height = 1;
  cloudMap->is_dense = true;
  cout << "cloudMap: " << cloudMap->points.size() << endl;
  ROS_WARN("Finished generate random map ");
  kdtreeLocalMap.setInputCloud(cloudMap->makeShared());
  _map_ok = true;
  // gengshuang end------------------------------------------

  //zrb begin----------------------------------------
  // cloudMap.width = cloudMap.points.size();
  // cloudMap.height = 1;
  // cloudMap.is_dense = true;

  // ROS_WARN("Finished generate random map ");

  // kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  // _map_ok = true;
  // zrb end----------------------------------------
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  //gengshuang-------------------------------
  // if (i < 10) {
  pcl::toROSMsg(*cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;
  //------------------------------------------

  //zrb begin---------------------------------
  // // if (i < 10) {
  // pcl::toROSMsg(cloudMap, globalMap_pcd);
  // globalMap_pcd.header.frame_id = "world";
  // _all_map_pub.publish(globalMap_pcd);
  // // }

  // return;
  //zrb end ----------------------------------


  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      //gengshuang begin -------------------------------
      pt = cloudMap->points[pointIdxRadiusSearch[i]];
      // gengshuang end ---------------------------------
      // zrb begin-------------------------------------
      // pt = cloudMap.points[pointIdxRadiusSearch[i]];
      // zrb end---------------------------------------
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);
  n.param("pcd_id", id, 1);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
