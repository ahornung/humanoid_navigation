/*
 * 6D localization for humanoid robots
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/humanoid_localization
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <humanoid_localization/HumanoidLocalization.h>
#include <iostream>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl_ros/transforms.h>

#if PCL_VERSION_COMPARE(>=,1,7,0)
  #include<pcl_conversions/pcl_conversions.h>
#endif

// simple timing benchmark output
#define _BENCH_TIME 0

namespace humanoid_localization{
HumanoidLocalization::HumanoidLocalization(unsigned randomSeed)
:
m_rngEngine(randomSeed),
m_rngNormal(m_rngEngine, NormalDistributionT(0.0, 1.0)),
m_rngUniform(m_rngEngine, UniformDistributionT(0.0, 1.0)),
m_nh(),m_privateNh("~"),
m_odomFrameId("odom"), m_targetFrameId("odom"), m_baseFrameId("torso"), m_baseFootprintId("base_footprint"), m_globalFrameId("map"),
m_useRaycasting(true), m_initFromTruepose(false), m_numParticles(500),
m_sensorSampleDist(0.2),
m_nEffFactor(1.0), m_minParticleWeight(0.0),
m_bestParticleIdx(-1), m_lastIMUMsgBuffer(5),
m_bestParticleAsMean(true),
m_receivedSensorData(false), m_initialized(false), m_initGlobal(false), m_paused(false),
m_syncedTruepose(false),
m_observationThresholdTrans(0.1), m_observationThresholdRot(M_PI/6),
m_observationThresholdHeadYawRot(0.5), m_observationThresholdHeadPitchRot(0.3),
m_temporalSamplingRange(0.1), m_transformTolerance(0.1),
m_groundFilterPointCloud(true), m_groundFilterDistance(0.04),
m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
m_sensorSampleDistGroundFactor(3),
m_headYawRotationLastScan(0.0), m_headPitchRotationLastScan(0.0),
m_useIMU(false),
m_constrainMotionZ (false), m_constrainMotionRP(false), m_useTimer(false), m_timerPeriod(0.1)
{

   m_latest_transform.setData (tf::Transform(tf::createIdentityQuaternion()) );
  // raycasting or endpoint model?
  m_privateNh.param("use_raycasting", m_useRaycasting, m_useRaycasting);

  m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  m_privateNh.param("target_frame_id", m_targetFrameId, m_targetFrameId);
  m_privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  m_privateNh.param("base_footprint_id", m_baseFootprintId, m_baseFootprintId);
  m_privateNh.param("global_frame_id", m_globalFrameId, m_globalFrameId);
  m_privateNh.param("init_from_truepose", m_initFromTruepose, m_initFromTruepose);
  m_privateNh.param("init_global", m_initGlobal, m_initGlobal);
  m_privateNh.param("best_particle_as_mean", m_bestParticleAsMean, m_bestParticleAsMean);
  m_privateNh.param("num_particles", m_numParticles, m_numParticles);
  m_privateNh.param("neff_factor", m_nEffFactor, m_nEffFactor);
  m_privateNh.param("min_particle_weight", m_minParticleWeight, m_minParticleWeight);

  m_privateNh.param("initial_pose/x", m_initPose(0), 0.0);
  m_privateNh.param("initial_pose/y", m_initPose(1), 0.0);
  m_privateNh.param("initial_pose/z", m_initPose(2), 0.32); // hip height when standing
  m_privateNh.param("initial_pose/roll", m_initPose(3), 0.0);
  m_privateNh.param("initial_pose/pitch", m_initPose(4), 0.0);
  m_privateNh.param("initial_pose/yaw", m_initPose(5), 0.0);
  m_privateNh.param("initial_pose_real_zrp", m_initPoseRealZRP, false);

  m_privateNh.param("initial_std/x", m_initNoiseStd(0), 0.1); // 0.1
  m_privateNh.param("initial_std/y", m_initNoiseStd(1), 0.1); // 0.1
  m_privateNh.param("initial_std/z", m_initNoiseStd(2), 0.02); // 0.02
  m_privateNh.param("initial_std/roll", m_initNoiseStd(3), 0.04); // 0.04
  m_privateNh.param("initial_std/pitch", m_initNoiseStd(4), 0.04); // 0.04
  m_privateNh.param("initial_std_yaw", m_initNoiseStd(5), M_PI/12); // M_PI/12

  if (m_privateNh.hasParam("num_sensor_beams"))
    ROS_WARN("Parameter \"num_sensor_beams\" is no longer used, use \"sensor_sampling_dist\" instead");

  // laser observation model parameters:
  m_privateNh.param("sensor_sampling_dist", m_sensorSampleDist, m_sensorSampleDist);
  m_privateNh.param("max_range", m_filterMaxRange, 30.0);
  m_privateNh.param("min_range", m_filterMinRange, 0.05);
  ROS_DEBUG("Using a range filter of %f to %f", m_filterMinRange, m_filterMaxRange);

  m_privateNh.param("update_min_trans", m_observationThresholdTrans, m_observationThresholdTrans);
  m_privateNh.param("update_min_rot", m_observationThresholdRot, m_observationThresholdRot);
  m_privateNh.param("update_min_head_yaw", m_observationThresholdHeadYawRot, m_observationThresholdHeadYawRot);
  m_privateNh.param("update_min_head_pitch", m_observationThresholdHeadPitchRot, m_observationThresholdHeadPitchRot);
  m_privateNh.param("temporal_sampling_range", m_temporalSamplingRange, m_temporalSamplingRange);
  m_privateNh.param("transform_tolerance", m_transformTolerance, m_transformTolerance);

  m_privateNh.param("use_imu", m_useIMU, m_useIMU);
  m_privateNh.param("constrain_motion_z", m_constrainMotionZ, m_constrainMotionZ);
  m_privateNh.param("constrain_motion_rp", m_constrainMotionRP, m_constrainMotionRP);

  // point cloud observation model parameters
  m_privateNh.param("ground_filter_point_cloud", m_groundFilterPointCloud, m_groundFilterPointCloud);
  m_privateNh.param("ground_filter_distance", m_groundFilterDistance, m_groundFilterDistance);
  m_privateNh.param("ground_filter_angle", m_groundFilterAngle, m_groundFilterAngle);
  m_privateNh.param("ground_filter_plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
  m_privateNh.param("sensor_sampling_dist_ground_factor", m_sensorSampleDistGroundFactor, m_sensorSampleDistGroundFactor);

  m_privateNh.param("use_timer", m_useTimer, m_useTimer);
  m_privateNh.param("timer_period", m_timerPeriod, m_timerPeriod);

  // motion model parameters

  m_motionModel = boost::shared_ptr<MotionModel>(new MotionModel(&m_privateNh, &m_rngEngine, &m_tfListener, m_odomFrameId, m_baseFrameId));

  if (m_useRaycasting){
    m_mapModel = boost::shared_ptr<MapModel>(new OccupancyMap(&m_privateNh));
    m_observationModel = boost::shared_ptr<ObservationModel>(new RaycastingModel(&m_privateNh, m_mapModel, &m_rngEngine));
  } else{
#ifndef SKIP_ENDPOINT_MODEL
    //m_mapModel = boost::shared_ptr<MapModel>(new DistanceMap(&m_privateNh));
    m_mapModel = boost::shared_ptr<MapModel>(new OccupancyMap(&m_privateNh));
    m_observationModel = boost::shared_ptr<ObservationModel>(new EndpointModel(&m_privateNh, m_mapModel, &m_rngEngine));
#else
    ROS_FATAL("EndpointModel not compiled due to missing dynamicEDT3D");
    exit(-1);
#endif
  }


  m_particles.resize(m_numParticles);
  m_poseArray.poses.resize(m_numParticles);
  m_poseArray.header.frame_id = m_globalFrameId;
  m_tfListener.clear();


  // publishers can be advertised first, before needed:
  m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  m_poseEvalPub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_eval", 10);
  m_poseOdomPub = m_privateNh.advertise<geometry_msgs::PoseStamped>("pose_odom_sync", 10);
  m_poseArrayPub = m_privateNh.advertise<geometry_msgs::PoseArray>("particlecloud", 10);
  m_bestPosePub = m_privateNh.advertise<geometry_msgs::PoseArray>("best_particle", 10);
  m_nEffPub = m_privateNh.advertise<std_msgs::Float32>("n_eff", 10);
  m_filteredPointCloudPub = m_privateNh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);


  //TODO Propagate particles independent of sensor callback
  reset();

  // ROS subscriptions last:
  m_globalLocSrv = m_nh.advertiseService("global_localization", &HumanoidLocalization::globalLocalizationCallback, this);

  // subscription on laser, tf message filter
  m_laserSub = new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nh, "scan", 100);
  m_laserFilter = new tf::MessageFilter<sensor_msgs::LaserScan>(*m_laserSub, m_tfListener, m_odomFrameId, 100);
  m_laserFilter->registerCallback(boost::bind(&HumanoidLocalization::laserCallback, this, _1));

  // subscription on point cloud, tf message filter
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "point_cloud", 100);
  m_pointCloudFilter = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, m_odomFrameId, 100);
  m_pointCloudFilter->registerCallback(boost::bind(&HumanoidLocalization::pointCloudCallback, this, _1));

  // subscription on init pose, tf message filter
  m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(m_nh, "initialpose", 2);
  m_initPoseFilter = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, m_tfListener, m_globalFrameId, 2);
  m_initPoseFilter->registerCallback(boost::bind(&HumanoidLocalization::initPoseCallback, this, _1));


  m_pauseIntegrationSub = m_privateNh.subscribe("pause_localization", 1, &HumanoidLocalization::pauseLocalizationCallback, this);
  m_pauseLocSrv = m_privateNh.advertiseService("pause_localization_srv", &HumanoidLocalization::pauseLocalizationSrvCallback, this);
  m_resumeLocSrv = m_privateNh.advertiseService("resume_localization_srv", &HumanoidLocalization::resumeLocalizationSrvCallback, this);

  if (m_useIMU)
    m_imuSub = m_nh.subscribe("imu", 5, &HumanoidLocalization::imuCallback, this);
  if (m_useTimer)
  {
     m_timer = m_nh.createTimer(ros::Duration(m_timerPeriod), &HumanoidLocalization::timerCallback, this);
     ROS_INFO("Using timer with a period of %4f s", m_timerPeriod);
  }

  ROS_INFO("NaoLocalization initialized with %d particles.", m_numParticles);
}

HumanoidLocalization::~HumanoidLocalization() {

  delete m_laserFilter;
  delete m_laserSub;

  delete m_pointCloudFilter;
  delete m_pointCloudSub;

  delete m_initPoseFilter;
  delete m_initPoseSub;

}

void HumanoidLocalization::timerCallback(const ros::TimerEvent & e){
   ros::Time transformExpiration = e.current_real + ros::Duration(m_transformTolerance);
   tf::StampedTransform tmp_tf_stamped(m_latest_transform, transformExpiration, m_globalFrameId, m_targetFrameId);
   m_tfBroadcaster.sendTransform(tmp_tf_stamped);
}


void HumanoidLocalization::reset(){

#if defined(_BENCH_TIME)
  ros::WallTime startTime = ros::WallTime::now();
#endif

  if (m_initGlobal){
    this->initGlobal();
  } else {
    geometry_msgs::PoseWithCovarianceStampedPtr posePtr(new geometry_msgs::PoseWithCovarianceStamped());

    if (m_initFromTruepose){ // useful for evaluation, when ground truth available:
      geometry_msgs::PoseStamped truePose;
      tf::Stamped<tf::Pose> truePoseTF;
      tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), ros::Time::now(), "torso_real"); // TODO: param

      ros::Time lookupTime = ros::Time::now();
      while(m_nh.ok() && !m_tfListener.waitForTransform(m_globalFrameId, ident.frame_id_, lookupTime, ros::Duration(1.0))){
        ROS_WARN("Waiting for transform %s --> %s for ground truth initialization failed, trying again...", m_globalFrameId.c_str(), ident.frame_id_.c_str());
        lookupTime = ros::Time::now();
      }
      ident.stamp_ = lookupTime;

      m_tfListener.transformPose(m_globalFrameId, ident, truePoseTF);
      tf::poseStampedTFToMsg(truePoseTF, truePose);
      tf::poseTFToMsg(truePoseTF, posePtr->pose.pose);
      posePtr->header = truePose.header;


      // initial covariance acc. to params
      for(int j=0; j < 6; ++j){
        for (int i = 0; i < 6; ++i){
          if (i == j)
            posePtr->pose.covariance.at(i*6 +j) = m_initNoiseStd(i) * m_initNoiseStd(i);
          else
            posePtr->pose.covariance.at(i*6 +j) = 0.0;
        }
      }

    } else{
      posePtr.reset(new geometry_msgs::PoseWithCovarianceStamped());
      for (int i = 0; i < 6; ++i){
        posePtr->pose.covariance.at(i*6 +i) = m_initNoiseStd(i) * m_initNoiseStd(i);
      }

      double roll, pitch, z;
      initZRP(z, roll, pitch);



      posePtr->pose.pose.position.x = m_initPose(0);
      posePtr->pose.pose.position.y = m_initPose(1);
      posePtr->pose.pose.position.z = z;
      tf::Quaternion quat;
      quat.setRPY(roll, pitch, m_initPose(5));
      tf::quaternionTFToMsg(quat, posePtr->pose.pose.orientation);

    }

    this->initPoseCallback(posePtr);

  }





#if defined(_BENCH_TIME)
  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Initialization of "<< m_numParticles << " particles took "
                  << dt << "s (="<<dt/m_numParticles<<"s/particle)");
#endif


}


void HumanoidLocalization::initZRP(double& z, double& roll, double& pitch){
  if(m_initPoseRealZRP) {
    // Get latest pose height
    tf::Stamped<tf::Pose> lastOdomPose;
    double poseHeight;
    if(m_motionModel->getLastOdomPose(lastOdomPose) &&
        lookupPoseHeight(lastOdomPose.stamp_, poseHeight)) {
      z = poseHeight;
    } else {
      ROS_WARN("Could not determine current pose height, falling back to init_pose_z");
      z = m_initPose(2);
    }

    // Get latest roll and pitch
    if(!m_lastIMUMsgBuffer.empty()) {
      getRP(m_lastIMUMsgBuffer.back().orientation, roll, pitch);
    } else {
      ROS_WARN("Could not determine current roll and pitch, falling back to init_pose_{roll,pitch}");
      roll = m_initPose(3);
      pitch = m_initPose(4);
    }
  } else {
    // Use pose height, roll and pitch from init_pose_{z,roll,pitch} parameters
    z = m_initPose(2);
    roll = m_initPose(3);
    pitch = m_initPose(4);
  }


}
void HumanoidLocalization::laserCallback(const sensor_msgs::LaserScanConstPtr& msg){
  ROS_DEBUG("Laser received (time: %f)", msg->header.stamp.toSec());
  
  if (!m_initialized){
    ROS_WARN("Localization not initialized yet, skipping laser callback.");
    return;
  }

  double timediff = (msg->header.stamp - m_lastLaserTime).toSec();
  if (m_receivedSensorData && timediff < 0){
    ROS_WARN("Ignoring received laser data that is %f s older than previous data!", timediff);
    return;
  }
  

  /// absolute, current odom pose
  tf::Stamped<tf::Pose> odomPose;
  // check if odometry available, skip scan if not.
  if (!m_motionModel->lookupOdomPose(msg->header.stamp, odomPose))
     return;


  bool sensor_integrated = false;
  if (!m_paused && (!m_receivedSensorData || isAboveMotionThreshold(odomPose))) {

     // convert laser to point cloud first:
     PointCloud pc_filtered;
     std::vector<float> laserRangesSparse;
     prepareLaserPointCloud(msg, pc_filtered, laserRangesSparse);

     sensor_integrated = localizeWithMeasurement(pc_filtered, laserRangesSparse, msg->range_max);

  } 

  if(!sensor_integrated){ // no laser integration: propagate particles forward by full interval

     // relative odom transform to last odomPose
     tf::Transform odomTransform = m_motionModel->computeOdomTransform(odomPose);
     m_motionModel->applyOdomTransform(m_particles, odomTransform);
     constrainMotion(odomPose);
  }
  else
  {
     m_lastLocalizedPose = odomPose;
  }

  m_motionModel->storeOdomPose(odomPose);
  publishPoseEstimate(msg->header.stamp, sensor_integrated);
  m_lastLaserTime = msg->header.stamp; 
}

void HumanoidLocalization::constrainMotion(const tf::Pose& odomPose){
  // skip if nothing to do:
  if (!m_constrainMotionZ && !m_constrainMotionRP)
    return;

  // reset z according to current odomPose:
  double z = odomPose.getOrigin().getZ();
  double odomRoll, odomPitch, uselessYaw;
  odomPose.getBasis().getRPY(odomRoll, odomPitch, uselessYaw);

#pragma omp parallel for
  for (unsigned i=0; i < m_particles.size(); ++i){
    if (m_constrainMotionZ){
      tf::Vector3 pos = m_particles[i].pose.getOrigin();
      double floor_z = m_mapModel->getFloorHeight(m_particles[i].pose);
      pos.setZ(z+floor_z);
      m_particles[i].pose.setOrigin(pos);
    }

    if (m_constrainMotionRP){
      double yaw =  tf::getYaw(m_particles[i].pose.getRotation());
      m_particles[i].pose.setRotation(tf::createQuaternionFromRPY(odomRoll, odomPitch, yaw));

    }
  }
}

bool HumanoidLocalization::isAboveMotionThreshold(const tf::Pose& odomPose){
  tf::Transform odomTransform = m_lastLocalizedPose.inverse() * odomPose;

  double yaw, pitch, roll;
  odomTransform.getBasis().getRPY(roll, pitch, yaw);

  return (odomTransform.getOrigin().length() >= m_observationThresholdTrans
      || std::abs(yaw) >= m_observationThresholdRot);
}

bool HumanoidLocalization::localizeWithMeasurement(const PointCloud& pc_filtered, const std::vector<float>& ranges, double max_range){
  ros::WallTime startTime = ros::WallTime::now();
#if PCL_VERSION_COMPARE(>=,1,7,0)
  ros::Time t = pcl_conversions::fromPCL(pc_filtered.header).stamp;
#else
  ros::Time t = pc_filtered.header.stamp;
#endif
  // apply motion model with temporal sampling:
  m_motionModel->applyOdomTransformTemporal(m_particles, t, m_temporalSamplingRange);
  
  // constrain to ground plane, if desired:
  tf::Stamped<tf::Transform> odomPose;
  if (!m_motionModel->lookupOdomPose(t, odomPose))
    return false;
  constrainMotion(odomPose);

  // transformation from torso frame to sensor
  // this takes the latest tf, assumes that torso to sensor did not change over temp. sampling!
  tf::StampedTransform localSensorFrame;
  if (!m_motionModel->lookupLocalTransform(pc_filtered.header.frame_id, t, localSensorFrame))
    return false;

  tf::Transform torsoToSensor(localSensorFrame.inverse());
  
//### Particles in log-form from here...
  toLogForm();

  // skip pose integration if z, roll and pitch constrained to floor by odometry
  if (!(m_constrainMotionRP && m_constrainMotionZ)){
    bool imuMsgOk = false;
    double angleX, angleY;
    if(m_useIMU) {
      ros::Time imuStamp;
      imuMsgOk = getImuMsg(t, imuStamp, angleX, angleY);
    } else {
      tf::Stamped<tf::Pose> lastOdomPose;
      if(m_motionModel->lookupOdomPose(t, lastOdomPose)) {
        double dropyaw;
        lastOdomPose.getBasis().getRPY(angleX, angleY, dropyaw);
        imuMsgOk = true;
      }
    }

    tf::StampedTransform footprintToTorso;
    // integrated pose (z, roll, pitch) meas. only if data OK:
    if(imuMsgOk) {
      if (!m_motionModel->lookupLocalTransform(m_baseFootprintId, t, footprintToTorso)) {
        ROS_WARN("Could not obtain pose height in localization, skipping Pose integration");
      } else {
        m_observationModel->integratePoseMeasurement(m_particles, angleX, angleY, footprintToTorso);
      }
    } else {
      ROS_WARN("Could not obtain roll and pitch measurement, skipping Pose integration");
    }
  }

  m_filteredPointCloudPub.publish(pc_filtered);
  m_observationModel->integrateMeasurement(m_particles, pc_filtered, ranges, max_range, torsoToSensor);

  // TODO: verify poses before measurements, ignore particles then
  m_mapModel->verifyPoses(m_particles);

  // normalize weights and transform back from log:
  normalizeWeights();
  //### Particles back in regular form now

  double nEffParticles = nEff();

  std_msgs::Float32 nEffMsg;
  nEffMsg.data = nEffParticles;
  m_nEffPub.publish(nEffMsg);

  if (nEffParticles <= m_nEffFactor*m_particles.size()){ // selective resampling
    ROS_INFO("Resampling, nEff=%f, numParticles=%zd", nEffParticles, m_particles.size());
    resample();
  } else {
    ROS_INFO("Skipped resampling, nEff=%f, numParticles=%zd", nEffParticles, m_particles.size());
  }

  m_receivedSensorData = true;

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Observations for "<< m_numParticles << " particles took "
                  << dt << "s (="<<dt/m_numParticles<<"s/particle)");

  return true;
}

void HumanoidLocalization::prepareLaserPointCloud(const sensor_msgs::LaserScanConstPtr& laser, PointCloud& pc, std::vector<float>& ranges) const{
  unsigned numBeams = laser->ranges.size();
  // skip every n-th scan:
  //unsigned step = computeBeamStep(numBeams);
  // build complete pointcloud:
  unsigned step = 1;



  // prepare laser message:
  unsigned int numBeamsSkipped = 0;

  // range_min of laser is also used to filter out wrong messages:
  double laserMin = std::max(double(laser->range_min), m_filterMinRange);

  // (range_max readings stay, will be used in the sensor model)

  ranges.reserve(50);

  // build a point cloud
#if PCL_VERSION_COMPARE(>=,1,7,0)
  pcl_conversions::toPCL(laser->header, pc.header);
#else
  pc.header = laser->header;
#endif
  pc.points.reserve(50);
  for (unsigned beam_idx = 0; beam_idx < numBeams; beam_idx+= step){
    float range = laser->ranges[beam_idx];
    if (range >= laserMin && range <= m_filterMaxRange){
      double laserAngle = laser->angle_min + beam_idx * laser->angle_increment;
      tf::Transform laserAngleRotation(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), laserAngle));
      tf::Vector3 laserEndpointTrans(range, 0.0, 0.0);
      tf::Vector3 pt(laserAngleRotation * laserEndpointTrans);

      pc.points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
      ranges.push_back(range);

    } else{
      numBeamsSkipped++;
    }

  }
  pc.height = 1;
  pc.width = pc.points.size();
  pc.is_dense = true;

  // uniform sampling:
  pcl::UniformSampling<pcl::PointXYZ> uniformSampling;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
  cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ> (pc));
  uniformSampling.setInputCloud(cloudPtr);
  uniformSampling.setRadiusSearch(m_sensorSampleDist);
  pcl::PointCloud<int> sampledIndices;
  uniformSampling.compute(sampledIndices);
  pcl::copyPointCloud(*cloudPtr, sampledIndices.points, pc);
  // adjust "ranges" array to contain the same points:
  std::vector<float> rangesSparse;
  rangesSparse.resize(sampledIndices.size());
  for (size_t i = 0; i < rangesSparse.size(); ++i){
    rangesSparse[i] = ranges[sampledIndices.points[i]];
  }
  ranges = rangesSparse;
  ROS_INFO("Laser PointCloud subsampled: %zu from %zu (%u out of valid range)", pc.size(), cloudPtr->size(), numBeamsSkipped);
}

int HumanoidLocalization::filterUniform(const PointCloud & cloud_in, PointCloud & cloud_out, int numSamples) const{
  int numPoints = static_cast<int>(cloud_in.size() );
  numSamples = std::min( numSamples, numPoints);
  std::vector<unsigned int> indices;
  indices.reserve( numPoints );
  for (int i=0; i<numPoints; ++i)
    indices.push_back(i);
  random_shuffle ( indices.begin(), indices.end());

  cloud_out.reserve( cloud_out.size() + numSamples );
  for ( int i = 0; i < numSamples; ++i)
  {
    cloud_out.push_back( cloud_in.at(indices[i]));
  }
  return numSamples;
}



void HumanoidLocalization::filterGroundPlane(const PointCloud& pc, PointCloud& ground, PointCloud& nonground, double groundFilterDistance, double groundFilterAngle, double groundFilterPlaneDistance){
   ground.header = pc.header;
   nonground.header = pc.header;

   if (pc.size() < 50){
      ROS_WARN("Pointcloud in HumanoidLocalization::filterGroundPlane too small, skipping ground plane extraction");
      nonground = pc;
   } else {
      // plane detection for ground plane removal:
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // Create the segmentation object and set up:
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients (true);
      // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
      seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(200);
      seg.setDistanceThreshold (groundFilterDistance);
      seg.setAxis(Eigen::Vector3f(0,0,1));
      seg.setEpsAngle(groundFilterAngle);


      PointCloud cloud_filtered(pc);
      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      bool groundPlaneFound = false;

      while(cloud_filtered.size() > 10 && !groundPlaneFound){
         seg.setInputCloud(cloud_filtered.makeShared());
         seg.segment (*inliers, *coefficients);
         if (inliers->indices.size () == 0){
            ROS_INFO("PCL segmentation did not find any plane.");

            break;
         }

         extract.setInputCloud(cloud_filtered.makeShared());
         extract.setIndices(inliers);

         if (std::abs(coefficients->values.at(3)) < groundFilterPlaneDistance){
            ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
            extract.setNegative (false);
            extract.filter (ground);

            // remove ground points from full pointcloud:
            // workaround for PCL bug:
            if(inliers->indices.size() != cloud_filtered.size()){
               extract.setNegative(true);
               PointCloud cloud_out;
               extract.filter(cloud_out);
               nonground += cloud_out;
               cloud_filtered = cloud_out;
            }

            groundPlaneFound = true;
         } else{
            ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
            pcl::PointCloud<pcl::PointXYZ> cloud_out;
            extract.setNegative (false);
            extract.filter(cloud_out);
            nonground +=cloud_out;
            // debug
            //            pcl::PCDWriter writer;
            //            writer.write<pcl::PointXYZ>("nonground_plane.pcd",cloud_out, false);

            // remove current plane from scan for next iteration:
            // workaround for PCL bug:
            if(inliers->indices.size() != cloud_filtered.size()){
               extract.setNegative(true);
               cloud_out.points.clear();
               extract.filter(cloud_out);
               cloud_filtered = cloud_out;
            } else{
               cloud_filtered.points.clear();
            }
         }

      }
      // TODO: also do this if overall starting pointcloud too small?
      if (!groundPlaneFound){ // no plane found or remaining points too small
         ROS_WARN("No ground plane found in scan");

         // do a rough fitlering on height to prevent spurious obstacles
         pcl::PassThrough<pcl::PointXYZ> second_pass;
         second_pass.setFilterFieldName("z");
         second_pass.setFilterLimits(-groundFilterPlaneDistance, groundFilterPlaneDistance);
         second_pass.setInputCloud(pc.makeShared());
         second_pass.filter(ground);

         second_pass.setFilterLimitsNegative (true);
         second_pass.filter(nonground);
      }

      // debug:
      //        pcl::PCDWriter writer;
      //        if (pc_ground.size() > 0)
      //          writer.write<pcl::PointXYZ>("ground.pcd",pc_ground, false);
      //        if (pc_nonground.size() > 0)
      //          writer.write<pcl::PointXYZ>("nonground.pcd",pc_nonground, false);
   }
}



void HumanoidLocalization::prepareGeneralPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloud& pc, std::vector<float>& ranges) const{

    pc.clear();
    // lookup Transfrom Sensor to BaseFootprint
    tf::StampedTransform sensorToBaseFootprint;
    try{
      m_tfListener.waitForTransform(m_baseFootprintId, msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFootprintId, msg->header.frame_id, msg->header.stamp, sensorToBaseFootprint);


    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for pointCloudCallback: " << ex.what() << ", quitting callback.\n");
      return;
    }

    /*** filter PointCloud and fill pc and ranges ***/

    // pass-through filter to get rid of near and far ranges
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_tmp(new pcl::PointCloud<pcl::PointXYZ>());
#if PCL_VERSION_COMPARE(>=,1,7,0)
    pcl::PCLPointCloud2 pcd2_tmp;
    pcl_conversions::toPCL(*msg, pcd2_tmp);
    pcl::fromPCLPointCloud2(pcd2_tmp, *pcd_tmp);
#else
    pcl::fromROSMsg(*msg, *pcd_tmp);
#endif
    pass.setInputCloud (pcd_tmp);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (m_filterMinRange, m_filterMaxRange);
    pass.filter (pc);

    // identify ground plane
    PointCloud ground, nonground;
    if (m_groundFilterPointCloud)
    {
        Eigen::Matrix4f matSensorToBaseFootprint, matBaseFootprintToSensor;
        pcl_ros::transformAsMatrix(sensorToBaseFootprint, matSensorToBaseFootprint);
        pcl_ros::transformAsMatrix(sensorToBaseFootprint.inverse(), matBaseFootprintToSensor);
        // TODO:Why transform the point cloud and not just the normal vector?
        pcl::transformPointCloud(pc, pc, matSensorToBaseFootprint );
        filterGroundPlane(pc, ground, nonground, m_groundFilterDistance, m_groundFilterAngle, m_groundFilterPlaneDistance);

        // clear pc again and refill it based on classification
        pc.clear();
        pcl::PointCloud<int> sampledIndices;

        int numFloorPoints = 0;
        if (ground.size() > 0){ // check for 0 size, otherwise PCL crashes
          // transform clouds back to sensor for integration
          pcl::transformPointCloud(ground, ground, matBaseFootprintToSensor);
          voxelGridSampling(ground, sampledIndices, m_sensorSampleDist*m_sensorSampleDistGroundFactor);
          pcl::copyPointCloud(ground, sampledIndices.points, pc);
          numFloorPoints = sampledIndices.size();
        }

        //int numNonFloorPoints = filterUniform( nonground, pc, m_numNonFloorPoints );
        int numNonFloorPoints = 0;
        if (nonground.size() > 0){ // check for 0 size, otherwise PCL crashes
          // transform clouds back to sensor for integration
          pcl::transformPointCloud(nonground, nonground, matBaseFootprintToSensor);
          voxelGridSampling( nonground, sampledIndices, m_sensorSampleDist);
          pcl::copyPointCloud( nonground, sampledIndices.points, nonground);
          numNonFloorPoints = sampledIndices.size();
          pc += nonground;
        }

        //TODO improve sampling?


        ROS_INFO("PointCloudGroundFiltering done. Added %d non-ground points and %d ground points (from %zu). Cloud size is %zu", numNonFloorPoints, numFloorPoints, ground.size(), pc.size());
        // create sparse ranges..
        ranges.resize(pc.size());
        for (unsigned int i=0; i<pc.size(); ++i)
        {
           pcl::PointXYZ p = pc.at(i);
           ranges[i] = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        }

    }
    else
    {
       ROS_INFO("Starting uniform sampling");
       //ROS_ERROR("No ground filtering is not implemented yet!");
       // uniform sampling:
       pcl::PointCloud<int> sampledIndices;
       voxelGridSampling(pc, sampledIndices,  m_sensorSampleDist);
       pcl::copyPointCloud(pc, sampledIndices.points, pc);

       // adjust "ranges" array to contain the same points:
       ranges.resize(sampledIndices.size());
       for (size_t i = 0; i < ranges.size(); ++i){
          pcl::PointXYZ p = pc[i]; 
          ranges[i] = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
          //rangesSparse[i] = ranges[sampledIndices.points[i]];
          //ranges[i] = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
       }
       ROS_INFO("Done.");


    }
    return;

}

void HumanoidLocalization::voxelGridSampling(const PointCloud & pc, pcl::PointCloud<int> & sampledIndices, double search_radius) const
{
   pcl::UniformSampling<pcl::PointXYZ> uniformSampling;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
   cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ> (pc)); // TODO: Check if this is a shallow copy..
   uniformSampling.setInputCloud(cloudPtr);
   uniformSampling.setRadiusSearch(search_radius);
   uniformSampling.compute(sampledIndices);
}

void HumanoidLocalization::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_DEBUG("PointCloud received (time: %f)", msg->header.stamp.toSec());

  if (!m_initialized){
    ROS_WARN("Loclization not initialized yet, skipping PointCloud callback.");
    return;
  }

  double timediff = (msg->header.stamp - m_lastPointCloudTime).toSec();
  if (m_receivedSensorData && timediff < 0){
    ROS_WARN("Ignoring received PointCloud data that is %f s older than previous data!", timediff);
    return;
  }


  /// absolute, current odom pose
  tf::Stamped<tf::Pose> odomPose;
  // check if odometry available, skip scan if not.
  if (!m_motionModel->lookupOdomPose(msg->header.stamp, odomPose))
    return;


  bool sensor_integrated = false;

  // TODO #1: Make this nicer: head rotations for integration check
  // TODO #2: Initialization of m_headYawRotationLastScan, etc needs to be set correctly
  bool isAboveHeadMotionThreshold = false;
  double headYaw, headPitch, headRoll;
  tf::StampedTransform torsoToSensor;
  if (!m_motionModel->lookupLocalTransform(msg->header.frame_id, msg->header.stamp, torsoToSensor))
      return; //TODO: should we apply applyOdomTransformTemporal, before returning

  // TODO #3: Invert transform?: tf::Transform torsoToSensor(localSensorFrame.inverse());

  torsoToSensor.getBasis().getRPY(headRoll, headPitch, headYaw);
  double headYawRotationSinceScan = std::abs(headYaw - m_headYawRotationLastScan);
  double headPitchRotationSinceScan = std::abs(headPitch - m_headPitchRotationLastScan);

  if (headYawRotationSinceScan>= m_observationThresholdHeadYawRot || headPitchRotationSinceScan >= m_observationThresholdHeadPitchRot)
      isAboveHeadMotionThreshold = true;
  // end #1

  if (!m_paused && (!m_receivedSensorData || isAboveHeadMotionThreshold || isAboveMotionThreshold(odomPose))) {

    // convert laser to point cloud first:
    PointCloud pc_filtered;
    std::vector<float> rangesSparse;
    prepareGeneralPointCloud(msg, pc_filtered, rangesSparse);

    double maxRange = 10.0; // TODO #4: What is a maxRange for pointClouds? NaN? maxRange is expected to be a double and integrateMeasurement checks rangesSparse[i] > maxRange
    ROS_DEBUG("Updating Pose Estimate from a PointCloud with %zu points and %zu ranges", pc_filtered.size(), rangesSparse.size());
    sensor_integrated = localizeWithMeasurement(pc_filtered, rangesSparse, maxRange);
   
  } 
  if(!sensor_integrated){ // no observation necessary: propagate particles forward by full interval
     // relative odom transform to last odomPose
     tf::Transform odomTransform = m_motionModel->computeOdomTransform(odomPose);
     m_motionModel->applyOdomTransform(m_particles, odomTransform);
     constrainMotion(odomPose);
  }
  else{
     m_lastLocalizedPose = odomPose;
     // TODO #1
     m_headYawRotationLastScan = headYaw;
     m_headPitchRotationLastScan = headPitch;
  }

  m_motionModel->storeOdomPose(odomPose);
  publishPoseEstimate(msg->header.stamp, sensor_integrated);
  m_lastPointCloudTime = msg->header.stamp;
  ROS_DEBUG("PointCloud callback complete.");
}

void HumanoidLocalization::imuCallback(const sensor_msgs::ImuConstPtr& msg){
  m_lastIMUMsgBuffer.push_back(*msg);
}

bool HumanoidLocalization::getImuMsg(const ros::Time& stamp, ros::Time& imuStamp, double& angleX, double& angleY) const {
  if(m_lastIMUMsgBuffer.empty())
    return false;

  typedef boost::circular_buffer<sensor_msgs::Imu>::const_iterator ItT;
  const double maxAge = 0.2;
  double closestOlderStamp = std::numeric_limits<double>::max();
  double closestNewerStamp = std::numeric_limits<double>::max();
  ItT closestOlder = m_lastIMUMsgBuffer.end(), closestNewer = m_lastIMUMsgBuffer.end();
  for(ItT it = m_lastIMUMsgBuffer.begin(); it != m_lastIMUMsgBuffer.end(); it++) {
    const double age = (stamp - it->header.stamp).toSec();
    if(age >= 0.0 && age < closestOlderStamp) {
      closestOlderStamp = age;
      closestOlder = it;
    } else if(age < 0.0 && -age < closestNewerStamp) {
      closestNewerStamp = -age;
      closestNewer = it;
    }
  }

  if(closestOlderStamp < maxAge && closestNewerStamp < maxAge && closestOlderStamp + closestNewerStamp > 0.0) {
    // Linear interpolation
    const double weightOlder = closestNewerStamp / (closestNewerStamp + closestOlderStamp);
    const double weightNewer = 1.0 - weightOlder;
    imuStamp = ros::Time(weightOlder * closestOlder->header.stamp.toSec()
                          + weightNewer * closestNewer->header.stamp.toSec());
    double olderX, olderY, newerX, newerY;
    getRP(closestOlder->orientation, olderX, olderY);
    getRP(closestNewer->orientation, newerX, newerY);
    angleX   = weightOlder * olderX  + weightNewer * newerX;
    angleY   = weightOlder * olderY + weightNewer * newerY;
    ROS_DEBUG("Msg: %.3f, Interpolate [%.3f .. %.3f .. %.3f]\n", stamp.toSec(), closestOlder->header.stamp.toSec(),
              imuStamp.toSec(), closestNewer->header.stamp.toSec());
    return true;
  } else if(closestOlderStamp < maxAge || closestNewerStamp < maxAge) {
    // Return closer one
    ItT it = (closestOlderStamp < closestNewerStamp) ? closestOlder : closestNewer;
    imuStamp = it->header.stamp;
    getRP(it->orientation, angleX, angleY);
    return true;
  } else {
    if(closestOlderStamp < closestNewerStamp)
      ROS_WARN("Closest IMU message is %.2f seconds too old, skipping pose integration", closestOlderStamp);
    else
      ROS_WARN("Closest IMU message is %.2f seconds too new, skipping pose integration", closestNewerStamp);
    return false;
  }
}

void HumanoidLocalization::initPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);

  if (msg->header.frame_id != m_globalFrameId){
    ROS_WARN("Frame ID of \"initialpose\" (%s) is different from the global frame %s", msg->header.frame_id.c_str(), m_globalFrameId.c_str());
  }

  std::vector<double> heights;
  double poseHeight = 0.0;
  if (std::abs(pose.getOrigin().getZ()) < 0.01){
    m_mapModel->getHeightlist(pose.getOrigin().getX(), pose.getOrigin().getY(), 0.6, heights);
    if (heights.size() == 0){
      ROS_WARN("No ground level to stand on found at map position, assuming 0");
      heights.push_back(0.0);
    }

    bool poseHeightOk = false;
    if(m_initPoseRealZRP) {
      ros::Time stamp(msg->header.stamp);
      if(stamp.isZero()) {
        // Header stamp is not set (e.g. RViz), use stamp from latest pose message instead
        tf::Stamped<tf::Pose> lastOdomPose;
        m_motionModel->getLastOdomPose(lastOdomPose);
        stamp = lastOdomPose.stamp_;
      }
      poseHeightOk = lookupPoseHeight(stamp, poseHeight);
      if(!poseHeightOk) {
        ROS_WARN("Could not determine current pose height, falling back to init_pose_z");
      }
    }
    if(!poseHeightOk) {
      ROS_INFO("Use pose height from init_pose_z");
      poseHeight = m_initPose(2);
    }
  }


  Matrix6d initCov;
  if ((std::abs(msg->pose.covariance.at(6*0+0) - 0.25) < 0.1) && (std::abs(msg->pose.covariance.at(6*1+1) -0.25) < 0.1)
      && (std::abs(msg->pose.covariance.at(6*3+3) - M_PI/12.0 * M_PI/12.0)<0.1)){
    ROS_INFO("Covariance originates from RViz, using default parameters instead");
    initCov = Matrix6d::Zero();
    initCov.diagonal() = m_initNoiseStd.cwiseProduct(m_initNoiseStd);

    // manually set r&p, rviz values are 0
    bool ok = false;
    const double yaw = tf::getYaw(pose.getRotation());
    if(m_initPoseRealZRP) {
      bool useOdometry = true;
      if(m_useIMU) {
        if(m_lastIMUMsgBuffer.empty()) {
          ROS_WARN("Could not determine current roll and pitch because IMU message buffer is empty.");
        } else {
          double roll, pitch;
          if(msg->header.stamp.isZero()) {
            // Header stamp is not set (e.g. RViz), use stamp from latest IMU message instead
            getRP(m_lastIMUMsgBuffer.back().orientation, roll, pitch);
            ok = true;
          } else {
            ros::Time imuStamp;
            ok = getImuMsg(msg->header.stamp, imuStamp, roll, pitch);
          }
          if(ok) {
            ROS_INFO("roll and pitch not set in initPoseCallback, use IMU values (roll = %f, pitch = %f) instead", roll, pitch);
            pose.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
            useOdometry = false;
          } else {
            ROS_WARN("Could not determine current roll and pitch from IMU, falling back to odometry roll and pitch");
            useOdometry = true;
          }
        }
      }

      if(useOdometry) {
        double roll, pitch, dropyaw;
        tf::Stamped<tf::Pose> lastOdomPose;
        ok = m_motionModel->getLastOdomPose(lastOdomPose);
        if(ok) {
          lastOdomPose.getBasis().getRPY(roll, pitch, dropyaw);
          pose.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
          ROS_INFO("roll and pitch not set in initPoseCallback, use odometry values (roll = %f, pitch = %f) instead", roll, pitch);
        } else {
          ROS_WARN("Could not determine current roll and pitch from odometry, falling back to init_pose_{roll,pitch} parameters");
        }
      }
    }

    if(!ok) {
      ROS_INFO("roll and pitch not set in initPoseCallback, use init_pose_{roll,pitch} parameters instead");
      pose.setRotation(tf::createQuaternionFromRPY(m_initPose(3), m_initPose(4), yaw));
    }
  } else{
    for(int j=0; j < initCov.cols(); ++j){
      for (int i = 0; i < initCov.rows(); ++i){
        initCov(i,j) = msg->pose.covariance.at(i*initCov.cols() +j);
      }
    }
  }

  // sample from initial pose covariance:
  Matrix6d initCovL = initCov.llt().matrixL();
  tf::Transform transformNoise; // transformation on original pose from noise
  unsigned idx = 0;
  for(Particles::iterator it = m_particles.begin(); it != m_particles.end(); ++it){
    Vector6d poseNoise;
    for (unsigned i = 0; i < 6; ++i){
      poseNoise(i) = m_rngNormal();
    }
    Vector6d poseCovNoise = initCovL * poseNoise; // is now drawn according to covariance noise
    // if a variance is set to 0 => no noise!
    for (unsigned i = 0; i < 6; ++i){
      if (std::abs(initCov(i,i)) < 0.00001)
        poseCovNoise(i) = 0.0;
    }


    transformNoise.setOrigin(tf::Vector3(poseCovNoise(0), poseCovNoise(1), poseCovNoise(2)));
    tf::Quaternion q;
    q.setRPY(poseCovNoise(3), poseCovNoise(4),poseCovNoise(5));

    transformNoise.setRotation(q);
    it->pose = pose;

    if (heights.size() > 0){
      // distribute particles evenly between levels:
      it->pose.getOrigin().setZ(heights.at(int(double(idx)/m_particles.size() * heights.size())) + poseHeight);
    }

    it->pose *= transformNoise;

    it->weight = 1.0/m_particles.size();
    idx++;
  }

  ROS_INFO("Pose reset around mean (%f %f %f)", pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ());

  // reset internal state:
  m_motionModel->reset();
  // force integration of next laser data:
  m_receivedSensorData = false;
  m_initialized = true;


  // Fix "0" time warning (when initializing pose from RViz)
  ros::Time stampPublish = msg->header.stamp;
  if (stampPublish.isZero()){
    tf::Stamped<tf::Pose> lastOdomPose;
    m_motionModel->getLastOdomPose(lastOdomPose);
    stampPublish = lastOdomPose.stamp_;
    if (stampPublish.isZero())
       stampPublish = ros::Time::now();
  
  }
  
  publishPoseEstimate(stampPublish, false);
}




bool HumanoidLocalization::globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

  initGlobal();

  return true;
}

void HumanoidLocalization::normalizeWeights() {

  double wmin = std::numeric_limits<double>::max();
  double wmax = -std::numeric_limits<double>::max();

  for (unsigned i=0; i < m_particles.size(); ++i){
    double weight = m_particles[i].weight;
    assert (!isnan(weight));
    if (weight < wmin)
      wmin = weight;
    if (weight > wmax){
      wmax = weight;
      m_bestParticleIdx = i;
    }
  }
  if (wmin > wmax){
    ROS_ERROR_STREAM("Error in weights: min=" << wmin <<", max="<<wmax<<", 1st particle weight="<< m_particles[1].weight<< std::endl);

  }

  double min_normalized_value;
  if (m_minParticleWeight > 0.0)
    min_normalized_value = std::max(log(m_minParticleWeight), wmin - wmax);
  else
    min_normalized_value = wmin - wmax;

  double max_normalized_value = 0.0; // = log(1.0);
  double dn = max_normalized_value-min_normalized_value;
  double dw = wmax-wmin;
  if (dw == 0.0) dw = 1;
  double scale = dn/dw;
  if (scale < 0.0){
    ROS_WARN("normalizeWeights: scale is %f < 0, dw=%f, dn=%f", scale, dw, dn );
  }
  double offset = -wmax*scale;
  double weights_sum = 0.0;

#pragma omp parallel
  {

#pragma omp for
    for (unsigned i = 0; i < m_particles.size(); ++i){
      double w = m_particles[i].weight;
      w = exp(scale*w+offset);
      assert(!isnan(w));
      m_particles[i].weight = w;
#pragma omp atomic
      weights_sum += w;
    }

    assert(weights_sum > 0.0);
    // normalize sum to 1:
#pragma omp for
    for (unsigned i = 0; i < m_particles.size(); ++i){
      m_particles[i].weight /= weights_sum;
    }

  }
}

double HumanoidLocalization::getCumParticleWeight() const{
  double cumWeight=0.0;

  //compute the cumulative weights
  for (Particles::const_iterator it = m_particles.begin(); it != m_particles.end(); ++it){
    cumWeight += it->weight;
  }

  return cumWeight;
}

void HumanoidLocalization::resample(unsigned numParticles){

  if (numParticles <= 0)
    numParticles = m_numParticles;

  //compute the interval
  double interval=getCumParticleWeight()/numParticles;

  //compute the initial target weight
  double target=interval*m_rngUniform();

  //compute the resampled indexes
  double cumWeight=0;
  std::vector<unsigned> indices(numParticles);

  unsigned n=0;
  for (unsigned i = 0; i < m_particles.size(); ++i){
    cumWeight += m_particles[i].weight;
    while(cumWeight > target && n < numParticles){
      if (m_bestParticleIdx >= 0 && i == unsigned(m_bestParticleIdx)){
        m_bestParticleIdx = n;
      }

      indices[n++]=i;
      target+=interval;
    }
  }
  // indices now contains the indices to draw from the particles distribution

  Particles oldParticles = m_particles;
  m_particles.resize(numParticles);
  m_poseArray.poses.resize(numParticles);
  double newWeight = 1.0/numParticles;
#pragma omp parallel for
  for (unsigned i = 0; i < numParticles; ++i){
    m_particles[i].pose = oldParticles[indices[i]].pose;
    m_particles[i].weight = newWeight;
  }
}

void HumanoidLocalization::initGlobal(){
  ROS_INFO("Initializing with uniform distribution");

  double roll, pitch, z;
  initZRP(z, roll, pitch);

  m_mapModel->initGlobal(m_particles, z, roll, pitch, m_initNoiseStd, m_rngUniform, m_rngNormal);


  ROS_INFO("Global localization done");
  m_motionModel->reset();
  m_receivedSensorData = false;
  m_initialized = true;

  publishPoseEstimate(ros::Time::now(), false);

}

void HumanoidLocalization::publishPoseEstimate(const ros::Time& time, bool publish_eval){

  ////
  // send all hypotheses as arrows:
  ////

  m_poseArray.header.stamp = time;

  if (m_poseArray.poses.size() != m_particles.size())
    m_poseArray.poses.resize(m_particles.size());

#pragma omp parallel for
  for (unsigned i = 0; i < m_particles.size(); ++i){
    tf::poseTFToMsg(m_particles[i].pose, m_poseArray.poses[i]);
  }

  m_poseArrayPub.publish(m_poseArray);

  ////
  // send best particle as pose and one array:
  ////
  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.stamp = time;
  p.header.frame_id = m_globalFrameId;

  tf::Pose bestParticlePose;
  if (m_bestParticleAsMean)
    bestParticlePose = getMeanParticlePose();
  else
    bestParticlePose = getBestParticlePose();

  tf::poseTFToMsg(bestParticlePose,p.pose.pose);
  m_posePub.publish(p);

  if (publish_eval){
    m_poseEvalPub.publish(p);
  }

  geometry_msgs::PoseArray bestPose;
  bestPose.header = p.header;
  bestPose.poses.resize(1);
  tf::poseTFToMsg(bestParticlePose, bestPose.poses[0]);
  m_bestPosePub.publish(bestPose);

  ////
  // send incremental odom pose (synced to localization)
  ////
  tf::Stamped<tf::Pose> lastOdomPose;
  if (m_motionModel->getLastOdomPose(lastOdomPose)){
    geometry_msgs::PoseStamped odomPoseMsg;
    tf::poseStampedTFToMsg(lastOdomPose, odomPoseMsg);
    m_poseOdomPub.publish(odomPoseMsg);
  }


  ////
  // Send tf target->map (where target is typically odom)
  tf::Stamped<tf::Pose> targetToMapTF;
  try{
    tf::Stamped<tf::Pose> baseToMapTF(bestParticlePose.inverse(),time, m_baseFrameId);
    m_tfListener.transformPose(m_targetFrameId, baseToMapTF, targetToMapTF); // typically target == odom
  } catch (const tf::TransformException& e){
    ROS_WARN("Failed to subtract base to %s transform, will not publish pose estimate: %s", m_targetFrameId.c_str(), e.what());
    return;
  }

  tf::Transform latestTF(tf::Quaternion(targetToMapTF.getRotation()), tf::Point(targetToMapTF.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used
  // see ROS amcl_node

  ros::Duration transformTolerance(m_transformTolerance);
  ros::Time transformExpiration = (time + transformTolerance);

  tf::StampedTransform tmp_tf_stamped(latestTF.inverse(), transformExpiration, m_globalFrameId, m_targetFrameId);
  m_latest_transform = tmp_tf_stamped;

  m_tfBroadcaster.sendTransform(tmp_tf_stamped);

}

unsigned HumanoidLocalization::getBestParticleIdx() const{
  if (m_bestParticleIdx < 0 || m_bestParticleIdx >= m_numParticles){
    ROS_WARN("Index (%d) of best particle not valid, using 0 instead", m_bestParticleIdx);
    return 0;
  }

  return m_bestParticleIdx;
}

tf::Pose HumanoidLocalization::getParticlePose(unsigned particleIdx) const{
  return m_particles.at(particleIdx).pose;
}

tf::Pose HumanoidLocalization::getBestParticlePose() const{
  return getParticlePose(getBestParticleIdx());
}

tf::Pose HumanoidLocalization::getMeanParticlePose() const{
  tf::Pose meanPose = tf::Pose::getIdentity();

  double totalWeight = 0.0;

  meanPose.setBasis(tf::Matrix3x3(0,0,0,0,0,0,0,0,0));
  for (Particles::const_iterator it = m_particles.begin(); it != m_particles.end(); ++it){
    meanPose.getOrigin() += it->pose.getOrigin() * it->weight;
    meanPose.getBasis()[0] += it->pose.getBasis()[0];
    meanPose.getBasis()[1] += it->pose.getBasis()[1];
    meanPose.getBasis()[2] += it->pose.getBasis()[2];
    totalWeight += it->weight;
  }
  assert(!isnan(totalWeight));

  //assert(totalWeight == 1.0);

  // just in case weights are not normalized:
  meanPose.getOrigin() /= totalWeight;
  // TODO: only rough estimate of mean rotation, asserts normalized weights!
  meanPose.getBasis() = meanPose.getBasis().scaled(tf::Vector3(1.0/m_numParticles, 1.0/m_numParticles, 1.0/m_numParticles));

  // Apparently we need to normalize again
  meanPose.setRotation(meanPose.getRotation().normalized());

  return meanPose;
}

double HumanoidLocalization::nEff() const{

  double sqrWeights=0.0;
  for (Particles::const_iterator it=m_particles.begin(); it!=m_particles.end(); ++it){
    sqrWeights+=(it->weight * it->weight);
  }

  if (sqrWeights > 0.0)
    return 1./sqrWeights;
  else
    return 0.0;
}

void HumanoidLocalization::toLogForm(){
  // TODO: linear offset needed?
#pragma omp parallel for
  for (unsigned i = 0; i < m_particles.size(); ++i){
    assert(m_particles[i].weight > 0.0);
    m_particles[i].weight = log(m_particles[i].weight);
  }
}

void HumanoidLocalization::pauseLocalizationCallback(const std_msgs::BoolConstPtr& msg){
  if (msg->data){
    if (!m_paused){
      m_paused = true;
      ROS_INFO("Localization paused");
    } else{
      ROS_WARN("Received a msg to pause localizatzion, but is already paused.");
    }
  } else{
    if (m_paused){
      m_paused = false;
      ROS_INFO("Localization resumed");
      // force laser integration:
      m_receivedSensorData = false;
    } else {
      ROS_WARN("Received a msg to resume localization, is not paused.");
    }

  }

}

bool HumanoidLocalization::pauseLocalizationSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!m_paused){
    m_paused = true;
    ROS_INFO("Localization paused");
  } else{
    ROS_WARN("Received a request to pause localizatzion, but is already paused.");
  }

  return true;
}

bool HumanoidLocalization::resumeLocalizationSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (m_paused){
    m_paused = false;
    ROS_INFO("Localization resumed");
    // force next laser integration:
    m_receivedSensorData = false;
  } else {
    ROS_WARN("Received a request to resume localization, but is not paused.");
  }

  return true;
}

bool HumanoidLocalization::lookupPoseHeight(const ros::Time& t, double& poseHeight) const{
  tf::StampedTransform tf;
  if (m_motionModel->lookupLocalTransform(m_baseFootprintId, t, tf)){
    poseHeight = tf.getOrigin().getZ();
    return true;
  } else
    return false;
}

}

