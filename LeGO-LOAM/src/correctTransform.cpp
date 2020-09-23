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
  ros::Publisher pubEuclideanClustering;

  ros::Publisher pubGroundGMCloud;  // ground Cloud of Global map
  ros::Publisher pubGroundLCCloud;  // Local Cloud of Global map

  ros::Publisher pubNormalVector;
// ROS Publisher for Frame

// ROS Subscriber
// for Susscriber
// ros::Subscriber subVar;
// set subVar = nh.subscribe<>(,,handler);
// in handler
// declaration at Private in Class double timeVar;
// in constructor, set timeVar = 0;
// declaration at Private in Class bool newVar;
// in constructor, set timeVar = false;

  ros::Subscriber subLaserCloudSurround;
  ros::Subscriber subKeyPoseOrigin;
  ros::Subscriber subGroundCloud;
  ros::Subscriber subLaserOdometry;

  pcl::PointCloud<PointType>::Ptr laserCloudSurroundlast; //
  pcl::PointCloud<PointType>::Ptr groundCloudlast;

  pcl::PointCloud<PointType>::Ptr cloudRoiFiltered;
  pcl::PointCloud<PointType>::Ptr cloudRoiFilteredUS;
  pcl::PointCloud<PointType>::Ptr cloudRoiFilteredDS;

  pcl::PointCloud<PointType>::Ptr cloudRANSACFiltered;
  pcl::PointCloud<PointType>::Ptr cloudRANSACFilteredRest;

  pcl::PointCloud<PointType>::Ptr cloudRSGroundLocal;
  pcl::PointCloud<PointType>::Ptr cloudRSGroundGlobal;

  pcl::PointCloud<PointTypePose>::Ptr keyPoseOriginlast;


  //vector<int> inliersRansac;
  //Eigen::VectorXf normalVector;


  double timelaserCloudSurround;
  double timegroundCloud;
  double timeKeyPoseOrigin;
  //double timelaserOdometry;

  bool newlaserCloudSurroundLast;
  bool newgroundCloudlast;
  bool newKeyPoseOrigin;
//  bool newlaserOdometry;

  float rollGlobal;
  float pitchGlobal;
  float yawGlobal;

public:
    // Constructor
    correctTransform():
        nh("~")
    {
      /*
      pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
      */
      // pubSphereCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_sphere", 2);
      // pubQuaternionBaseRobot = nh.advertise<geometry_msgs::QuaternionStamped>("/Quaternion_BaseRobot", 4);
      // pubQuaternionEnvironment = nh.advertise<geometry_msgs::QuaternionStamped>("/Quaternion_Environment", 4);
      // pubEuclideanClustering = nh.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 2);

      subGroundCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/ground_cloud", 2, &correctTransform::groundCloudHandler, this);
      subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 2, &correctTransform::laserCloudSurroundHandler, this);
      subKeyPoseOrigin = nh.subscribe<sensor_msgs::PointCloud2> ("/key_pose_origin", 2, &correctTransform::keyPoseOriginHandler, this);
      // subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &correctTransform::laserOdometryHandler, this);

      pubGroundGMCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_global_cloud", 2);
      pubGroundLCCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_local_cloud", 2);

      pubNormalVector = nh.advertise<cloud_msgs::vector_msgs>("/vectors", 2);
      /*
      subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
      subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
      */

      allocateMemory();
    }


    void allocateMemory() {
      laserCloudSurroundlast.reset(new pcl::PointCloud<PointType>());
      groundCloudlast.reset(new pcl::PointCloud<PointType>());
      cloudRoiFiltered.reset(new pcl::PointCloud<PointType>());
      cloudRoiFilteredUS.reset(new pcl::PointCloud<PointType>());
      cloudRoiFilteredDS.reset(new pcl::PointCloud<PointType>());
      cloudRANSACFiltered.reset(new pcl::PointCloud<PointType>());
      cloudRANSACFilteredRest.reset(new pcl::PointCloud<PointType>());

      cloudRSGroundLocal.reset(new pcl::PointCloud<PointType>());
      cloudRSGroundGlobal.reset(new pcl::PointCloud<PointType>());

      keyPoseOriginlast.reset(new pcl::PointCloud<PointTypePose>());


      timelaserCloudSurround = 0;
      timeKeyPoseOrigin = 0;
      timegroundCloud = 0;
  //    timelaserOdometry = 0;

      newlaserCloudSurroundLast = false;
      newgroundCloudlast = false;
      newKeyPoseOrigin = false;

      rollGlobal = 0;
      pitchGlobal = 0;
      yawGlobal = 0;

    }

    void keyPoseOriginHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
      timeKeyPoseOrigin = msg->header.stamp.toSec();
      keyPoseOriginlast->clear();
      pcl::fromROSMsg(*msg, *keyPoseOriginlast);
      newKeyPoseOrigin = true;

    }

    void groundCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
      timegroundCloud = msg->header.stamp.toSec();
      groundCloudlast->clear();
      pcl::fromROSMsg(*msg, *groundCloudlast);
      newgroundCloudlast = true;
    }

    void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
      timelaserCloudSurround = msg->header.stamp.toSec();
      laserCloudSurroundlast->clear();
      pcl::fromROSMsg(*msg, *laserCloudSurroundlast);
      newlaserCloudSurroundLast = true;
    }
/*    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& msg){
      timelaserOdometry = msg->header.stamp.toSec();
      double roll, pitch, yaw;
      geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;

      tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

      rollGlobal = -pitch;
      pitchGlobal = -yaw;
      yawGlobal = roll;

  //    newlaserOdometry = true;
}*/



    void runMain(){

      // make correspondence with subscribe topics
      //if(newlaserCloudSurroundLast && newgroundCloudlast && newKeyPoseOrigin &&
        //std::abs(timelaserCloudSurround - timeKeyPoseOrigin) < 0.05 &&
        //std::abs(timeKeyPoseOrigin - timegroundCloud) < 0.05){
      if(newlaserCloudSurroundLast && newgroundCloudlast && newKeyPoseOrigin){
          newlaserCloudSurroundLast=0;
          newgroundCloudlast=0;
          newKeyPoseOrigin=0;

      }else{
        return ;
      }



      // Params related it following the utility.h

      // 1) set ROI Cloud around Pose3
      setCloudROI();

      setCloudVoxelization();

      getPlaneParam();
      // getCylinderParam();
      // getSphereParam();

      // ?) Publish all data.
      publishAll();
    }

    void setCloudROI(){
      // Set ROI FILTER
      // keyPoseOriginlast is trajectory.
      if (laserCloudSurroundlast->points.empty() || keyPoseOriginlast->points.empty())
        return;

      int idxKeyPoseOrigin = keyPoseOriginlast->points.size()-1;
      int distanceRoiRange = 5;

      float minXRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].x-distanceRoiRange;
      float maxXRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].x+distanceRoiRange;
      float minYRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].y-distanceRoiRange;
      float maxYRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].y+0.5;//+distanceRoiRange;
      float minZRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].z-distanceRoiRange;
      float maxZRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].z+distanceRoiRange;

      // for roi filter
      pcl::PassThrough<PointType> pass_x;
      pcl::PointCloud<PointType> xf_cloud;
      pass_x.setInputCloud(laserCloudSurroundlast);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits (minXRoi, maxXRoi);
      pass_x.filter (xf_cloud);

      pcl::PointCloud<PointType>::Ptr xf_cloud_ptr(new pcl::PointCloud<PointType>(xf_cloud));
      pcl::PassThrough<PointType> pass_y;
      pcl::PointCloud<PointType> yf_cloud;

      //ROS_INFO("xf_cloud_ptr Size is : %d", xf_cloud_ptr->points.size());

      pass_y.setInputCloud(xf_cloud_ptr);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits (minYRoi, maxYRoi);
      pass_y.filter (yf_cloud);

      pcl::PointCloud<PointType>::Ptr yf_cloud_ptr(new pcl::PointCloud<PointType>(yf_cloud));
      pcl::PassThrough<PointType> pass_z;

      pass_z.setInputCloud(yf_cloud_ptr);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits (minZRoi, maxZRoi);
      pass_z.filter (*cloudRoiFiltered);

    }

    void setCloudVoxelization(){
      pcl::PointCloud<PointType>::Ptr cloudFilterIn (new pcl::PointCloud<PointType>(*groundCloudlast));
      //copyPointCloud(*groundCloudlast, *cloudFilterIn);

      // DownSampling
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setInputCloud(cloudFilterIn);
      downSizeFilter.setLeafSize(0.5, 0.5, 0.5);
      downSizeFilter.filter(*groundCloudlast);
      cloudFilterIn->clear();
    }

    void getPlaneParam(){

      // keyPoseOriginlast is trajectory.
      // last idx is the present.
      // robot positiong

      if (cloudRoiFiltered->points.size() <= 30)
        return;

      // Vectors
      float gx = 0, gy = 0, gz = 0;   // global_ground
      float lx = 0, ly = 0, lz = 0;   // local_ground

      // int idxKeyPoseOrigin = keyPoseOriginlast->points.size()-1;


      // Set RANSAC : Global Ground Cloud
      // Create the segmentation object for the planar model and set all the parameters

      pcl::SACSegmentation<PointType> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (50);
      seg.setDistanceThreshold (0.1);

      //ROS_INFO("roi number of points : %d", cloudRoiFiltered->size());
      // Segment the largest planar component from the cropped cloud
      seg.setInputCloud (cloudRoiFiltered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointType> extract;
      extract.setInputCloud (cloudRoiFiltered);
      extract.setIndices(inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloudRSGroundGlobal);

      // Remove the planar inliers, extract the rest
      // extract.setNegative (true);
      // extract.filter (*cloudRANSACFilteredRest);

      //coefficients.value.resize(3);
      gx = -coefficients->values[0];
      gy = coefficients->values[2];
      //gz = coefficients->values[0];
      gz = -coefficients->values[1];
      //normVecg = sqrt(gx*gx+gy*gy+gz*gz);

      // ROS_INFO("global coefficients of %f, %f, %f", gx, gy, gz);

      if (groundCloudlast->points.empty())
        return ;

      int cloudSize = groundCloudlast->points.size();
      //ROS_INFO("number of ground points : %d", cloudSize);
      /*PointType point;

      for (int i = 0; i < cloudSize; i++) {
        point.x = groundCloudlast->points[i].y;
        point.y = groundCloudlast->points[i].z;
        point.z = groundCloudlast->points[i].x;
        point.intensity = groundCloudlast->points[i].intensity;

        groundCloudlast->points[i] = point;
      }*/


      // Set RANSAC : Part of local Map
      // Create the segmentation object for the planar model and set all the parameters
      //pcl::SACSegmentation<PointType> seg;
      //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (50);
      seg.setDistanceThreshold (0.1);

      // Segment the largest planar component from the cropped cloud
      seg.setInputCloud (groundCloudlast);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
      }

      // Extract the planar inliers from the input cloud
      //pcl::ExtractIndices<PointType> extract;
      extract.setInputCloud (groundCloudlast);
      extract.setIndices(inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloudRSGroundLocal);

      // Remove the planar inliers, extract the rest
      //extract.setNegative (true);
      //extract.filter (*cloudRSGroundLocal);

      //coefficients.value.resize(3);


      // get transformation in camera frame (because points are in camera frame)
      /*
      Eigen::Vector3f translation;
      translation << 0, 0, 0;
      float x,y,z;
      x = translation(0);
      y = translation(1);
      z = translation(2);

      //Euler angles
      float roll,pitch,yaw;
      roll = rollGlobal;
      pitch = pitchGlobal;
      yaw = yawGlobal;

      Eigen::Affine3f transformation = pcl::getTransformation(x, y, z, roll, pitch, yaw);
      Eigen::Vector3f coeff;
      coeff << coefficients->values[0], coefficients->values[1], coefficients->values[2];

      lx = transformation*coeff(0);
      ly = transformation*coeff(1);
      lz = transformation*coeff(2);
*/


      lx = coefficients->values[0];
      ly = coefficients->values[1];
      lz = coefficients->values[2];

      ROS_INFO("coefficients of %f, %f, %f", lx, ly, lz);

      cloud_msgs::vector_msgs vectorDiff;
      vectorDiff.gx = gx;
      vectorDiff.gy = gy;
      vectorDiff.gz = gz;
      vectorDiff.lx = lx;
      vectorDiff.ly = ly;
      vectorDiff.lz = lz;
      // vectorDiff.header.stamp = ros::Time().fromSec(timegroundCloud);
      vectorDiff.header.stamp = ros::Time::now();

      pubNormalVector.publish(vectorDiff);

    }

    void getCylinderParam(){
      if (cloudRoiFilteredDS->points.size() <= 100)
        return;

      // Set RANSAC
      // Create the segmentation object for the model and set all the parameters
      pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_CYLINDER);
      seg.setMethodType (pcl::SAC_RANSAC);

      seg.setNormalDistanceWeight ( 0.1 );
      seg.setMaxIterations ( 500 );
      seg.setDistanceThreshold ( 0.4 );
      seg.setRadiusLimits ( 1.7, 2.5 );
      seg.setInputCloud ( cloudRoiFilteredDS );
//      seg.setInputNormals ( cloudRoiFiltered );

      // Segment the largest component from the cropped cloud
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0) {
        ROS_WARN_STREAM ("Could not estimate a model for the given dataset.");
        //break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointType> extract;
      extract.setInputCloud (cloudRoiFiltered);
      extract.setIndices(inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloudRANSACFiltered);
      // ROS_INFO_STREAM("PointCloud representing the component: " << cloudRANSACFiltered->points.size () << " data points." );

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloudRANSACFilteredRest);

      ROS_INFO("======getCylinderParam=====");
      ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRoiFilteredDS->points.size());
      ROS_INFO("Size of cloudRANSACFiltered : %d", (int) cloudRANSACFiltered->points.size());
      ROS_INFO("Size of cloudRANSACFilteredRest : %d", (int) cloudRANSACFilteredRest->points.size());

    }

    void getSphereParam(){

      if (cloudRoiFilteredDS->points.size() <= 100)
        return;

      // Set RANSAC
      // Create the segmentation object for the planar model and set all the parameters
      ROS_INFO("======getSphereParam=====");

      size_t itrNum = 4;
      float xCenter[itrNum];
      float yCenter[itrNum];
      float zCenter[itrNum];
      float rCenter[itrNum];

      pcl::SACSegmentation<PointType> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_SPHERE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.005);
      seg.setRadiusLimits(0.5, 2);

      pcl::PointCloud<PointType>::Ptr cloudSphereFiltered (new pcl::PointCloud<PointType>());

      *cloudSphereFiltered = *cloudRoiFilteredDS;

      for (size_t i = 0; i < itrNum; i++){
        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud (cloudSphereFiltered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          ROS_WARN_STREAM ("Could not estimate a sphere model for the given dataset.") ;
          return;
        }

        // lidar to camera
        xCenter[i] = coefficients->values[2];
        yCenter[i] = coefficients->values[0];
        zCenter[i] = coefficients->values[1];
        rCenter[i] = coefficients->values[3];

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud (cloudSphereFiltered);
        extract.setIndices(inliers);
        extract.setNegative (false);

        // Get the points associated with the surface
        extract.filter (*cloudRANSACFiltered);
        ROS_INFO_STREAM("PointCloud representing the component: " << cloudRANSACFiltered->points.size () << " data points." );

        // Remove the  inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloudSphereFiltered);
      }


      *cloudRANSACFilteredRest = *cloudSphereFiltered;
      int iTmp;
      for (size_t i=0; i<itrNum; i++){
        iTmp = int (i);
        ROS_INFO("%d : %f, %f, %f, %f", iTmp, xCenter[i], yCenter[i], zCenter[i], rCenter[i]);
      }

    }
/*
    void RansacThread(){

        if (RansacFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok()){
            rate.sleep();

            // 1) set ROI Cloud around Pose3
            setCloudROI();

            //setCloudVoxelization();

            getPlaneParam();
        }
    }
*/
    void publishAll(){

      if (cloudRSGroundGlobal->points.empty())
        return;

      sensor_msgs::PointCloud2 cloudMsgGlobal;
      pcl::toROSMsg(*cloudRSGroundGlobal, cloudMsgGlobal);
      cloudMsgGlobal.header.stamp = ros::Time::now(); // ros::Time().fromSec(timegroundCloud);
      cloudMsgGlobal.header.frame_id = "/camera_init";


      pubGroundGMCloud.publish(cloudMsgGlobal);
      cloudRSGroundGlobal->clear();

      if (cloudRSGroundLocal->points.empty())
        return;

      sensor_msgs::PointCloud2 cloudMsgLocal;
      pcl::toROSMsg(*cloudRSGroundLocal, cloudMsgLocal);
      cloudMsgLocal.header.stamp = ros::Time::now(); //ros::Time().fromSec(timegroundCloud);
      cloudMsgLocal.header.frame_id = "/camera_init";

      pubGroundLCCloud.publish(cloudMsgLocal);

      cloudRSGroundLocal->clear();
      cloudRoiFiltered->clear();

      cloudRANSACFiltered->clear();
      cloudRANSACFilteredRest->clear();

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Correct Transfrom Started.");

    correctTransform CT;

    //std::thread loopthread(&correctTransform::RansacThread, &CT);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        CT.runMain();
        rate.sleep();
    }

    // ros::spin();
    //loopthread.join();


    return 0;
}
