/*
  First Trial, using cylinderical shape of Tunnel,
  Improve the accuracy of lego-LOAM Algorithm.

*/

#include "utility.h"


class correctTransform{

private:

// ROS nodeHandle
  ros::NodeHandle nh;

// ROS Publisher
  ros::Publisher pubSphereCloud;


/*
  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubKeyPoses;

// ROS Publisher for Frame

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;
*/
// ROS Subscriber
// for Susscriber
// ros::Subscriber subVar;
// set subVar = nh.subscribe<>(,,handler);
// in handler
// declaration at Private in Class double timeVar;
// in constructor, set timeVar = 0;
// declaration at Private in Class bool newVar;
// in constructor, set timeVar = false;

/*
  ros::Subscriber subLaserCloudCornerLast;
  ros::Subscriber subLaserCloudSurfLast;
  ros::Subscriber subOutlierCloudLast;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subImu;


*/
  ros::Subscriber subLaserCloudSurround;
  ros::Subscriber subKeyPoseOrigin;


  pcl::PointCloud<PointType>::Ptr cloudRoiFiltered;
  pcl::PointCloud<PointType>::Ptr laserCloudSurroundlast; //
  pcl::PointCloud<PointType>::Ptr keyPoseOriginlast;

  // for roi filter
  pcl::PassThrough<PointType> pass;



  double timelaserCloudSurround;
  double timeKeyPoseOrigin;


  bool newlaserCloudSurroundLast;
  bool newKeyPoseOrigin;




public:
    // Constructor
    correctTransform():
        nh("~")
    {
      /*
      pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
      pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
      pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
      */
      pubSphereCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_sphere", 2);


      subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 2, &correctTransform::laserCloudSurroundHandler, this);
      subKeyPoseOrigin = nh.subscribe<sensor_msgs::PointCloud2> ("/key_pose_origin", 2, &correctTransform::keyPoseOriginHandler, this);
      /*
      subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
      subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
      subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this);
      subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
      subImu = nh.subscribe<sensor_msgs::Imu> (imuTopic, 50, &mapOptimization::imuHandler, this);

      pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
      pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
      pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
            */
      allocateMemory();
    }


    void allocateMemory() {
      laserCloudSurroundlast.reset(new pcl::PointCloud<PointType>());
      cloudRoiFiltered.reset(new pcl::PointCloud<PointType>());
      keyPoseOriginlast.reset(new pcl::PointCloud<PointType>());

      timelaserCloudSurround = 0;
      timeKeyPoseOrigin = 0;

      newlaserCloudSurroundLast = false;
      newKeyPoseOrigin = false;
    }

    void keyPoseOriginHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
      timeKeyPoseOrigin = msg->header.stamp.toSec();
      keyPoseOriginlast->clear();
      pcl::fromROSMsg(*msg, *keyPoseOriginlast);
      newKeyPoseOrigin = true;

    }


    void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        timelaserCloudSurround = msg->header.stamp.toSec();
        laserCloudSurroundlast->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurroundlast);
        newlaserCloudSurroundLast = true;
    }


    void runMain(){

      // getCylinderParam from input data;
      //
      // Params related it following the utility.h
      // 1) get ROI Cloud around Pose3
      // 2) get

      getCylinderParam();


      // ?) Pulish all data.
      pulishAll();
    }

    void getCylinderParam(){
      if (keyPoseOriginlast->points.empty())
        return;

      // keyPoseOriginlast is trajectory.
      // last idx is the present.
      // robot positiong
      int idxKeyPoseOrigin = keyPoseOriginlast->points.size()-1;
      int valueRange = 5;

      pass.setInputCloud(laserCloudSurroundlast);
      pass.setFilterFieldName("x");
      pass.setFilterLimits (keyPoseOriginlast->points[idxKeyPoseOrigin].x-valueRange, keyPoseOriginlast->points[idxKeyPoseOrigin].x+valueRange);
      pass.setFilterFieldName("y");
      pass.setFilterLimits (keyPoseOriginlast->points[idxKeyPoseOrigin].y-valueRange, keyPoseOriginlast->points[idxKeyPoseOrigin].y+valueRange);
      pass.setFilterFieldName("z");
      pass.setFilterLimits (keyPoseOriginlast->points[idxKeyPoseOrigin].z-valueRange, keyPoseOriginlast->points[idxKeyPoseOrigin].z+valueRange);

      pass.filter (*cloudRoiFiltered);

    }

    void pulishAll(){

      if (pubSphereCloud.getNumSubscribers() == 0)
          return;

      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudRoiFiltered, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(timelaserCloudSurround);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubSphereCloud.publish(cloudMsgTemp);

      cloudRoiFiltered->clear();
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Correct Transfrom Started.");

    correctTransform CT;

/*
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
*/

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        /*
        CT.run();
        FA.runFeatureAssociation();
        */
        CT.runMain();

        rate.sleep();
    }
    ros::spin();
/*
    loopthread.join();
    visualizeMapThread.join();
*/
    return 0;
}
