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
  ros::Publisher pubQuaternionBaseRobot;
  ros::Publisher pubQuaternionEnvironment;
  ros::Publisher pubEuclideanClustering;

// ROS Publisher for Frame
/*
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

  ros::Subscriber subLaserCloudSurround;
  ros::Subscriber subKeyPoseOrigin;



  pcl::PointCloud<PointType>::Ptr laserCloudSurroundlast; //

  pcl::PointCloud<PointType>::Ptr cloudRoiFiltered;
  pcl::PointCloud<PointType>::Ptr cloudRoiFilteredUS;
  pcl::PointCloud<PointType>::Ptr cloudRoiFilteredDS;

  pcl::PointCloud<PointType>::Ptr cloudRANSACFiltered;
  pcl::PointCloud<PointType>::Ptr cloudRANSACFilteredRest;


  pcl::PointCloud<PointTypePose>::Ptr keyPoseOriginlast;


  vector<int> inliersRansac;
  Eigen::VectorXf normalVector;


  double timelaserCloudSurround;
  double timeKeyPoseOrigin;


  bool newlaserCloudSurroundLast;
  bool newKeyPoseOrigin;


  // for CorrectPose


  bool bThresholdGapAngle;

  float xGround, yGround, zGround;   // ground
  float xBase, yBase, zBase;   // base


public:
    // Constructor
    correctTransform():
        nh("~")
    {
      /*
      pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
      */
      pubSphereCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_sphere", 2);
      pubQuaternionBaseRobot = nh.advertise<geometry_msgs::QuaternionStamped>("/Quaternion_BaseRobot", 4);
      pubQuaternionEnvironment = nh.advertise<geometry_msgs::QuaternionStamped>("/Quaternion_Environment", 4);
      pubEuclideanClustering = nh.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 2);

      // subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2> ("/ground_cloud", 2, &correctTransform::laserCloudSurroundHandler, this);
      subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 2, &correctTransform::laserCloudSurroundHandler, this);
      //subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 2, &correctTransform::laserCloudSurroundHandler, this);
      subKeyPoseOrigin = nh.subscribe<sensor_msgs::PointCloud2> ("/key_pose_origin", 2, &correctTransform::keyPoseOriginHandler, this);
      /*
      subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
      subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
      */

      allocateMemory();
    }


    void allocateMemory() {
      laserCloudSurroundlast.reset(new pcl::PointCloud<PointType>());
      cloudRoiFiltered.reset(new pcl::PointCloud<PointType>());
      cloudRoiFilteredUS.reset(new pcl::PointCloud<PointType>());
      cloudRoiFilteredDS.reset(new pcl::PointCloud<PointType>());
      cloudRANSACFiltered.reset(new pcl::PointCloud<PointType>());
      cloudRANSACFilteredRest.reset(new pcl::PointCloud<PointType>());

      keyPoseOriginlast.reset(new pcl::PointCloud<PointTypePose>());


      timelaserCloudSurround = 0;
      timeKeyPoseOrigin = 0;

      newlaserCloudSurroundLast = false;
      newKeyPoseOrigin = false;

      bThresholdGapAngle = false;

      xGround=0; yGround=0; zGround=0;   // ground
      xBase=0; yBase=0; zBase=0;   // base


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


      if (keyPoseOriginlast->points.empty() == true)
        return;


      // Params related it following the utility.h
      // 1) set ROI Cloud around Pose3
      setCloudROI();

      //setCloudVoxelization();

      //getEuclideanCluster();

      getPlaneParam();

      // getCylinderParam();

      //getSphereParam();




      // ?) Pulish all data.
      pulishAll();
    }

    void setCloudROI(){
      // Set ROI FILTER
      // keyPoseOriginlast is trajectory.
      if (laserCloudSurroundlast->points.empty())
        return;


      int idxKeyPoseOrigin = keyPoseOriginlast->points.size()-1;
      int distanceRoiRange = 4;

      float minXRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].x-distanceRoiRange;
      float maxXRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].x+distanceRoiRange;
      float minYRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].y-distanceRoiRange;
      float maxYRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].y-0.5;//+distanceRoiRange;
      float minZRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].z-distanceRoiRange;
      float maxZRoi = keyPoseOriginlast->points[idxKeyPoseOrigin].z+distanceRoiRange;

      /*
      // cause tf z->x, x->y, y->z

      ROS_INFO("x: %f, %f", minZRoi, maxZRoi);
      ROS_INFO("y: %f, %f", minXRoi, maxXRoi);
      ROS_INFO("z: %f, %f", minYRoi, maxYRoi);
      */

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

      //ROS_INFO("yf_cloud_ptr Size is : %d", yf_cloud_ptr->points.size());
      pass_z.setInputCloud(yf_cloud_ptr);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits (minZRoi, maxZRoi);
      pass_z.filter (*cloudRoiFiltered);

      ROS_INFO("======setCloudROI=====");
      ROS_INFO("Size of laserCloudSurroundlast : %d", (int) laserCloudSurroundlast->points.size());
      ROS_INFO("Size of cloudRoiFiltered : %d", (int) cloudRoiFiltered->points.size());
    }

    void setCloudVoxelization(){

      // Upsampling
      if (cloudRoiFiltered->points.empty())
        return;

//      *cloudRoiFilteredUS = * cloudRoiFiltered;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilterIn (new pcl::PointCloud<pcl::PointXYZ>());
      copyPointCloud(*cloudRoiFiltered, *cloudFilterIn);

      // Object for searching.
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> upSizeFilter;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
      pcl::PointCloud<pcl::PointNormal>::Ptr mls_normalPoints (new pcl::PointCloud<pcl::PointNormal>);

      float search_radius = 0.3; //0.01
      float sampling_radius = 0.5; //0.005
      float step_size = 0.1; //0.005
      double gauss_param = (double)std::pow(search_radius,2);
      int pol_order = 3;
      //float voxel_size = 0.1;
      //int itr_Num = 1;

      upSizeFilter.setComputeNormals(true);

      upSizeFilter.setInputCloud(cloudFilterIn);
      upSizeFilter.setSearchMethod(kdtree);
      // Use all neighbors in a radius of 3cm.
      upSizeFilter.setSearchRadius(search_radius);
      // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
      // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
      //upSizeFilter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
      upSizeFilter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
      //upSizeFilter.setDilationVoxelSize(voxel_size);
      //upSizeFilter.setDilationIterations(itr_Num);

      // Radius around each point, where the local plane will be sampled.
      // Set the radius of the circle in the local point plane that will be sampled.
      upSizeFilter.setUpsamplingRadius(sampling_radius);

      // Sampling step size. Bigger values will yield less (if any) new points.
      upSizeFilter.setUpsamplingStepSize(step_size);
      upSizeFilter.setPolynomialOrder(pol_order);
      upSizeFilter.setSqrGaussParam(gauss_param);

      upSizeFilter.process(*mls_normalPoints);
      //ROS_INFO("Size of cloudFilterIn : %d", (int) cloudFilterIn->points.size());
      //ROS_INFO("Size of mls_normalPoints : %d", (int) mls_normalPoints->points.size());
      copyPointCloud(*mls_normalPoints, *cloudRoiFilteredUS);
      //ROS_INFO("Size of cloudRoiFilteredUS : %d", (int) cloudRoiFilteredUS->points.size());


      // DownSampling
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
      downSizeFilter.setInputCloud(cloudRoiFilteredUS);
      downSizeFilter.filter(*cloudRoiFilteredDS);


      ROS_INFO("======setCloudVoxelization=====");

      ROS_INFO("Size of cloudRoiFiltered : %d", (int) cloudRoiFiltered->points.size());
      ROS_INFO("Size of cloudRoiFilteredUS : %d", (int) cloudRoiFilteredUS->points.size());
      ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRoiFilteredDS->points.size());
    }

    void getPlaneParam(){

      // keyPoseOriginlast is trajectory.
      // last idx is the present.
      // robot positiong

      if (cloudRoiFiltered->points.size() <= 100)
        return;

      int idxKeyPoseOrigin = keyPoseOriginlast->points.size()-1;
      int valueRange = 2;

      float tfRoll = keyPoseOriginlast->points[idxKeyPoseOrigin].yaw;
      float tfPitch = keyPoseOriginlast->points[idxKeyPoseOrigin].roll;
      float tfYaw = keyPoseOriginlast->points[idxKeyPoseOrigin].pitch;

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(tfRoll, tfPitch, tfYaw);
                                    // from mapOptimization, rpy was yrp orders...
                                    // i want to find a xy plane vector of Base robot.
                                    /*
      cout << "roll : " << keyPoseOriginlast->points[idxKeyPoseOrigin].roll*180/PI << endl;
      cout << "pitch : " << keyPoseOriginlast->points[idxKeyPoseOrigin].pitch*180/PI << endl;
      cout << "yaw : " << keyPoseOriginlast->points[idxKeyPoseOrigin].yaw*180/PI << endl;
*/
      // get the angle between Vectors
      float gx = 0, gy = 0, gz = 0;   // ground
      float bx = 0, by = 0, bz = 0, bw = 0, term = 0;   // base
      float normVecb = 0, normVecg = 0;
      float angleBetweenVectors = 0;
      float angleDegree = 0;

      double gRoll = 0, gPitch = 0, gYaw = 0;
      double bRoll = 0, bPitch = 0, bYaw = 0;

      //
      bw = geoQuat.w;
      // v = q_xyz/sqrt(1-qw*qw)
      term = 1/sqrt(1-bw*bw);
      bx = geoQuat.x*term;
      by = geoQuat.y*term;
      bz = geoQuat.z*term;
      normVecb = sqrt(bx*bx+by*by+bz*bz);

      if (abs(1-normVecb) > 0.005)
        return;

      // Set RANSAC
      // Create the segmentation object for the planar model and set all the parameters
      pcl::SACSegmentation<PointType> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (150);
      seg.setDistanceThreshold (0.08);
      ROS_INFO("======getPlaneParam=====");

      ROS_INFO("before Size of cloudRoiFilteredDS : %d", (int) cloudRoiFiltered->points.size());




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


      ROS_INFO("after Size of cloudRoiFilteredDS : %d", (int) cloudRoiFiltered->points.size());


      // Get the points associated with the planar surface
      extract.filter (*cloudRANSACFiltered);


      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloudRANSACFilteredRest);

      ROS_INFO("Size of cloudRANSACFiltered : %d", (int) cloudRANSACFiltered->points.size());
      ROS_INFO("Size of cloudRANSACFilteredRest : %d", (int) cloudRANSACFilteredRest->points.size());


      /*
      pcl::SampleConsensusModelPlane<PointType>::Ptr modelTarget(new pcl::SampleConsensusModelPlane<PointType> (cloudRoiFiltered));
      pcl::RandomSampleConsensus<PointType> modelRansac(modelTarget);
      modelRansac.setDistanceThreshold(.15);
      modelRansac.computeModel();
      modelRansac.getInliers(inliersRansac);
      modelRansac.getModelCoefficients(normalVector);



      // VectorXf need a resize of memory size because it is dynamic.
      normalVector.resize(4);

      gx = normalVector(2);
      gy = normalVector(0);
      gz = normalVector(1);*/
      //coefficients.value.resize(3);
      gx = coefficients->values[2];
      gy = coefficients->values[0];
      gz = coefficients->values[1];

      normVecg = sqrt(gx*gx+gy*gy+gz*gz);

/*
      ROS_INFO("base plane vector");
      ROS_INFO("base plane vector bx : %f" , bx);
      ROS_INFO("base plane vector by : %f" , by);
      ROS_INFO("base plane vector bz : %f" , bz);
      ROS_INFO("Norm is : %f" , normVecb);

      ROS_INFO("ground vector");
      ROS_INFO("ground vector gx : %f", gx);
      ROS_INFO("ground vector gy : %f", gy);
      ROS_INFO("ground vector gz : %f", gz);
      ROS_INFO("Norm is : %f", normVecg);
*/
      // calculate the angle bet ground vector and base plane vector
      angleBetweenVectors = acos(gx*bx+gy*by+gz*bz);
      angleDegree = angleBetweenVectors*180/PI;// abs(angleBetweenVectors)*180/PI;
      //ROS_INFO("angle radian : %f", angleBetweenVectors);
      //ROS_INFO("angle degree : %f", angleDegree);


      //
      tf::Matrix3x3(tf::Quaternion(gx, gy, gz, 0)).getRPY(gRoll, gPitch, gYaw);
      tf::Matrix3x3(tf::Quaternion(bx, by, bz, 0)).getRPY(bRoll, bPitch, bYaw);

      /*
      if (angleDegree <= 30){
        ROS_INFO("TEST 30");
        bThresholdGapAngle = true;
        tf::quaternionTFToMsg(tf::Quaternion(bx, by, bz, 0), QuarternionBaseMsg.quaternion);
        tf::quaternionTFToMsg(tf::Quaternion(gx, gy, gz, 0), QuarternionEnvMsg.quaternion);
      }
      else{
        bThresholdGapAngle = false;
      }
      */

      xGround = gx;
      yGround = gy;
      zGround = gz;

      xBase = bx;
      yBase = by;
      zBase = bz;

      //tf::quaternionTFToMsg(tf::Quaternion(bx, by, bz, 0), QuarternionBaseMsg.quaternion);
      //tf::quaternionTFToMsg(tf::Quaternion(gx, gy, gz, 0), QuarternionEnvMsg.quaternion);


/*
      geometry_msgs::Quaternion betweenQuat = tf::createQuaternionMsgFromRollPitchYaw;

      // get the diff angle betweent tow vectors
      //rollAlpha = acos()
      tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]);
      Eigen::Vector3f sourceVector = tf::Vector3(bx, by, bz);
      Eigen::Vector3f targetVector = tf::Vector3(gx, gy, gz);
      Eigen::Quaternion midQuat = Eigen::FromTwoVector(sourceVector, targetVector);
      */
      // In this moment 200420, couldn't use a library fluently. So choose nest method.




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
      seg.setMaxIterations ( 2000 );
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

      ROS_INFO("Size of cloudRoiFiltered : %d", (int) cloudRoiFiltered->points.size());
      ROS_INFO("Size of cloudRoiFilteredUS : %d", (int) cloudSphereFiltered->points.size());
      ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRANSACFilteredRest->points.size());

    }
    //if (cloudSphereFiltered->points.empty())
    //  return;

    // void getEuclideanCluster(pcl::PointCloud<pcl::PointXYZ::Ptr> cloudArg){
    // pcl::PointCloud<pcl::PointXYZ::Ptr> cloudArg
    // had Tried to use an argument for function, but didn't
    // because couldn't be sure that which one is the better way for a performance yet.

    void getEuclideanCluster(){
      if (cloudRoiFilteredDS->points.empty())
        return;

      // Creating the KdTree object for the search method of the extraction
      pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
      pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

//      ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRoiFilteredDS->points.size());

      *cloud_filtered = *cloudRoiFilteredDS;
      tree->setInputCloud (cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointType> ec;
      ec.setClusterTolerance (0.4); // 2cm
      ec.setMinClusterSize (20);
      ec.setMaxClusterSize (10000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);

      //ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRoiFilteredDS->points.size());
      //ROS_INFO("Size of cloud_filtered : %d", (int) cloud_filtered->points.size());

      ec.extract (cluster_indices);

      std::vector<pcl::PointCloud<PointType>::Ptr> clusters;
      //std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
      int j = 0;
      ROS_INFO("Size of cluster_indices %d", (int) cluster_indices.size());
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        // ROS_INFO("In for struct {} %d", j);
        pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);

        int label = rand () % 8;
        ROS_INFO("label : %d, rand() : %d", label, rand());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
          cloud_filtered->points[*pit].intensity = label;
          cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

/*
        clusters.push_back(cloud_cluster);
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*cloudRANSACFiltered, cloudMsgTemp);

        cloudMsgTemp.header.stamp = ros::Time().fromSec(timelaserCloudSurround);
        cloudMsgTemp.header.frame_id = "/camera_init";
        pubSphereCloud.publish(cloudMsgTemp);
*/
        clusters.push_back(cloud_cluster);
        //sensor_msgs::PointCloud2::Ptr tempROSMsg; //(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2 tempROSMsg;
        //pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
        pcl::toROSMsg(*clusters[j], tempROSMsg);
        tempROSMsg.header.stamp = ros::Time::now();
        tempROSMsg.header.frame_id = "/camera_init";
        //pc2_clusters.push_back(tempROSMsg);
        pubEuclideanClustering.publish(tempROSMsg);

        j++;

        // pc2_clusters.push_back(tempROSMsg);
        // pubEuclideanClustering.publish(tempROSMsg);
      }
      /*

      // Convert To ROS Data PointType
      pcl::PCLPointCloud2 cloud_p;
      pcl::toPCLPointCloud2(pc2_cluster, cloud_p);

      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_p, output);
      */
      //output.header.frame_id = "/camera_init";

      ROS_INFO("======getEuclideanCluster=====");

      ROS_INFO("Size of cloudRoiFilteredDS : %d", (int) cloudRoiFilteredDS->points.size());
      // ROS_INFO("Size of cloudRoiFilteredUS : %d", (int) cloudRoiFilteredUS->points.size());
      ROS_INFO("Size of cluster_indices : %d", (int) cluster_indices.size());
    }




    void pulishAll(){

      if (pubSphereCloud.getNumSubscribers() == 0)
        return;
        /*
      if (cloudRANSACFiltered->points.empty())
        return;
        */

      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudRANSACFiltered, cloudMsgTemp);

      cloudMsgTemp.header.stamp = ros::Time().fromSec(timelaserCloudSurround);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubSphereCloud.publish(cloudMsgTemp);

      cloudRoiFiltered->clear();
      cloudRoiFilteredUS->clear();
      cloudRoiFilteredDS->clear();

      cloudRANSACFiltered->clear();
      cloudRANSACFilteredRest->clear();

      //
      /*
      if (!bThresholdGapAngle){
        return;
      }
*/

      geometry_msgs::QuaternionStamped QuarternionBaseMsg;
      tf::quaternionTFToMsg(tf::Quaternion(xBase, yBase, zBase, 0), QuarternionBaseMsg.quaternion);
      QuarternionBaseMsg.header.stamp = ros::Time().fromSec(timeKeyPoseOrigin);
      QuarternionBaseMsg.header.frame_id = "/camera_init";

      pubQuaternionBaseRobot.publish(QuarternionBaseMsg);

      geometry_msgs::QuaternionStamped QuarternionEnvMsg;
      tf::quaternionTFToMsg(tf::Quaternion(xGround, yGround, zGround, 0), QuarternionEnvMsg.quaternion);
      QuarternionEnvMsg.header.stamp = ros::Time().fromSec(timeKeyPoseOrigin);
      QuarternionEnvMsg.header.frame_id = "/camera_init";
      pubQuaternionEnvironment.publish(QuarternionEnvMsg);

      // bThresholdGapAngle = false;

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Correct Transfrom Started.");

    correctTransform CT;


    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        CT.runMain();
        rate.sleep();
    }
    //ros::spin();

    return 0;
}
