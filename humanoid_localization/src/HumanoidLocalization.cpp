// SVN $HeadURL$
// SVN $Id$

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

// simple timing benchmark output
#define _BENCH_TIME 0

namespace humanoid_localization{
HumanoidLocalization::HumanoidLocalization(unsigned randomSeed)
:
m_rngEngine(randomSeed),
m_rngNormal(m_rngEngine, NormalDistributionT(0.0, 1.0)),
m_rngUniform(m_rngEngine, UniformDistributionT(0.0, 1.0)),
m_nh(),m_privateNh("~"),
m_odomFrameId("odom"), m_baseFrameId("torso"), m_baseFootprintId("/base_footprint"), m_globalFrameId("/map"),
m_useRaycasting(false), m_initFromTruepose(false), m_numParticles(1000),
m_numSensorBeams(50),
m_maxOdomInterval(7.0),
m_nEffFactor(1.0), m_minParticleWeight(0.0),
m_bestParticleIdx(-1), m_lastIMUMsgBuffer(5),
m_bestParticleAsMean(true),
m_firstLaserReceived(false), m_initialized(false), m_initGlobal(false), m_paused(false),
m_syncedTruepose(false),
m_observationThresholdTrans(0.1), m_observationThresholdRot(M_PI/6),
m_observationThresholdHeadYawRot(0.1), m_observationThresholdHeadPitchRot(0.1),
m_temporalSamplingRange(0.1),
m_translationSinceScan(0.0), m_rotationSinceScan(0.0),
m_headYawRotationLastScan(0.0), m_headPitchRotationLastScan(0.0),
m_useIMU(false)
{

  // raycasting or endpoint model?
  m_privateNh.param("use_raycasting", m_useRaycasting, m_useRaycasting);

  m_motionModel = boost::shared_ptr<MotionModel>(new MotionModel(&m_privateNh, &m_rngEngine, &m_tfListener));

  m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
  m_privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  m_privateNh.param("base_footprint_id", m_baseFootprintId, m_baseFootprintId);
  m_privateNh.param("global_frame_id", m_globalFrameId, m_globalFrameId);
  m_privateNh.param("init_from_truepose", m_initFromTruepose, m_initFromTruepose);
  m_privateNh.param("init_global", m_initGlobal, m_initGlobal);
  m_privateNh.param("best_particle_as_mean", m_bestParticleAsMean, m_bestParticleAsMean);
  m_privateNh.param("sync_truepose", m_syncedTruepose, m_syncedTruepose);
  m_privateNh.param("num_particles", m_numParticles, m_numParticles);
  m_privateNh.param("max_odom_interval", m_maxOdomInterval, m_maxOdomInterval);
  m_privateNh.param("neff_factor", m_nEffFactor, m_nEffFactor);
  m_privateNh.param("min_particle_weight", m_minParticleWeight, m_minParticleWeight);

  m_privateNh.param("initial_pose_x", m_initPose(0), 0.0);
  m_privateNh.param("initial_pose_y", m_initPose(1), 0.0);
  m_privateNh.param("initial_pose_z", m_initPose(2), 0.32); // hip height when standing
  m_privateNh.param("initial_pose_roll", m_initPose(3), 0.0);
  m_privateNh.param("initial_pose_pitch", m_initPose(4), 0.0);
  m_privateNh.param("initial_pose_yaw", m_initPose(5), 0.0);
  m_privateNh.param("initial_pose_real_zrp", m_initPoseRealZRP, false);

  m_privateNh.param("initial_std_x", m_initNoiseStd(0), 0.1); // 0.1
  m_privateNh.param("initial_std_y", m_initNoiseStd(1), 0.1); // 0.1
  m_privateNh.param("initial_std_z", m_initNoiseStd(2), 0.02); // 0.02
  m_privateNh.param("initial_std_roll", m_initNoiseStd(3), 0.04); // 0.04
  m_privateNh.param("initial_std_pitch", m_initNoiseStd(4), 0.04); // 0.04
  m_privateNh.param("initial_std_yaw", m_initNoiseStd(5), M_PI/12); // M_PI/12

  // laser observation model parameters:
  m_privateNh.param("num_sensor_beams", m_numSensorBeams, m_numSensorBeams);
  m_privateNh.param("max_range", m_filterMaxRange, 30.0);
  m_privateNh.param("min_range", m_filterMinRange, 0.05);
  ROS_DEBUG("Using a range filter of %f to %f", m_filterMinRange, m_filterMaxRange);

  m_privateNh.param("laser_thres_trans", m_observationThresholdTrans, m_observationThresholdTrans);
  m_privateNh.param("laser_thres_rot", m_observationThresholdRot, m_observationThresholdRot);
  m_privateNh.param("head_thres_rot_yaw", m_observationThresholdHeadYawRot, m_observationThresholdHeadYawRot);
  m_privateNh.param("head_thres_rot_pitch", m_observationThresholdHeadPitchRot, m_observationThresholdHeadPitchRot);
  m_privateNh.param("temporal_sampling_range", m_temporalSamplingRange, m_temporalSamplingRange);

  m_privateNh.param("use_imu", m_useIMU, m_useIMU);

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
  m_poseTruePub = m_privateNh.advertise<geometry_msgs::PoseStamped>("pose_true_sync", 10);
  m_poseArrayPub = m_privateNh.advertise<geometry_msgs::PoseArray>("particlecloud", 10);
  m_bestPosePub = m_privateNh.advertise<geometry_msgs::PoseArray>("best_particle", 10);
  m_nEffPub = m_privateNh.advertise<std_msgs::Float32>("n_eff", 10);
  m_filteredPointCloudPub = m_privateNh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);


  reset();

  // ROS subscriptions last:
  m_globalLocSrv = m_nh.advertiseService("global_localization", &HumanoidLocalization::globalLocalizationCallback, this);

  // subscription on laser, tf message filter
  m_laserSub = new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nh, "scan", 100);
  m_laserFilter = new tf::MessageFilter<sensor_msgs::LaserScan>(*m_laserSub, m_tfListener, m_odomFrameId, 100);
  m_laserFilter->registerCallback(boost::bind(&HumanoidLocalization::laserCallback, this, _1));

  // subscription on point cloud, tf message filter
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2 >(m_nh, "point_cloud", 100);
  m_pointCloudFilter = new tf::MessageFilter<sensor_msgs::PointCloud2 >(*m_pointCloudSub, m_tfListener, m_odomFrameId, 100);
  m_pointCloudFilter->registerCallback(boost::bind(&HumanoidLocalization::pointCloudCallback, this, _1));

  // subscription on init pose, tf message filter
  m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(m_nh, "initialpose", 2);
  m_initPoseFilter = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, m_tfListener, m_globalFrameId, 2);
  m_initPoseFilter->registerCallback(boost::bind(&HumanoidLocalization::initPoseCallback, this, _1));


  m_pauseIntegrationSub = m_privateNh.subscribe("pause_localization", 1, &HumanoidLocalization::pauseLocalizationCallback, this);
  m_pauseLocSrv = m_privateNh.advertiseService("pause_localization_srv", &HumanoidLocalization::pauseLocalizationSrvCallback, this);
  m_resumeLocSrv = m_privateNh.advertiseService("resume_localization_srv", &HumanoidLocalization::resumeLocalizationSrvCallback, this);

  m_imuSub = m_nh.subscribe("imu", 5, &HumanoidLocalization::imuCallback, this);

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

void HumanoidLocalization::reset(){

#if defined(_BENCH_TIME)
  ros::WallTime startTime = ros::WallTime::now();
#endif

  if (m_initGlobal){
    this->initGlobal();
  } else {
    geometry_msgs::PoseWithCovarianceStampedPtr posePtr;

    if (m_initFromTruepose){
    	ROS_ERROR("Init from Truepose service call not implemented\n");

      // TODO: fix below: instead of service lookup!
      //		geometry_msgs::PoseStamped truePose;
      //		tf::Stamped<tf::Pose> truePoseTF;
      //		tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(), btVector3(0,0,0)), time, "true_torso_Link");
      //
      //		if (! m_tfListener.waitForTransform(m_globalFrameId, time, "true_torso_Link", 1.0))
      //			ROS_WARN("Waiting for Truepose transform failed, trying again...");
      //
      //		m_tfListener.transformPose(m_globalFrameId, ident, truePoseTF);
      //		tf::poseStampedTFToMsg(truePoseTF, truePose);
      //		m_poseTruePub.publish(truePose);



//      const static std::string servname = "simulator_truepose";
//      ROS_INFO("Requesting truepose from %s...", m_nh.resolveName(servname).c_str());
//      nao_msgs::GetTruepose::Request req;
//      nao_msgs::GetTruepose::Response resp;
//
//      while(m_nh.ok() && !ros::service::call(servname, req, resp))
//      {
//        ROS_WARN("Truepose for initialization request to %s failed; trying again...", m_nh.resolveName(servname).c_str());
//        usleep(1000000);
//      }
//
//      posePtr.reset(new geometry_msgs::PoseWithCovarianceStamped(resp.pose));
//      // initial covariance acc. to params (Truepose has cov. 0)
//      for(int j=0; j < 6; ++j){
//        for (int i = 0; i < 6; ++i){
//          if (i == j)
//            posePtr->pose.covariance.at(i*6 +j) = m_initNoiseStd(i) * m_initNoiseStd(i);
//          else
//            posePtr->pose.covariance.at(i*6 +j) = 0.0;
//        }
//      }

    } else{
      posePtr.reset(new geometry_msgs::PoseWithCovarianceStamped());
      for (int i = 0; i < 6; ++i){
        posePtr->pose.covariance.at(i*6 +i) = m_initNoiseStd(i) * m_initNoiseStd(i);
      }

      posePtr->pose.pose.position.x = m_initPose(0);
      posePtr->pose.pose.position.y = m_initPose(1);
      double roll, pitch;
      if(m_initPoseRealZRP) {
        // Get latest pose height
        tf::Stamped<tf::Pose> lastOdomPose;
        double poseHeight;
        if(m_motionModel->getLastOdomPose(lastOdomPose) &&
            m_motionModel->lookupPoseHeight(lastOdomPose.stamp_, poseHeight)) {
          posePtr->pose.pose.position.z = poseHeight;
        } else {
          ROS_WARN("Could not determine current pose height, falling back to init_pose_z");
          posePtr->pose.pose.position.z = m_initPose(2);
        }

        // Get latest roll and pitch
        if(!m_lastIMUMsgBuffer.empty()) {
          getRPY(m_lastIMUMsgBuffer.back().orientation, roll, pitch);
        } else {
          ROS_WARN("Could not determine current roll and pitch, falling back to init_pose_{roll,pitch}");
          roll = m_initPose(3);
          pitch = m_initPose(4);
        }
      } else {
        // Use pose height, roll and pitch from init_pose_{z,roll,pitch} parameters
        posePtr->pose.pose.position.z = m_initPose(2);
        roll = m_initPose(3);
        pitch = m_initPose(4);
      }

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


void HumanoidLocalization::laserCallback(const sensor_msgs::LaserScanConstPtr& msg){
  ROS_DEBUG("Laser received (time: %f)", msg->header.stamp.toSec());

#if defined(_BENCH_TIME)
  ros::WallTime startTime = ros::WallTime::now();
#endif

  if (!m_initialized){
    ROS_WARN("Loclization not initialized yet, skipping laser callback.");
    return;
  }

  ros::Duration timediff = msg->header.stamp - m_lastLaserTime;
  if (m_firstLaserReceived && timediff < ros::Duration(0.0)){
    ROS_WARN("Ignoring received laser data that is %f s older than previous data!", timediff.toSec());
    return;
  }

  // compute current odometry transform first, see if it would trigger the
  // integration of observations
  tf::Transform odomTransform;
  if (!m_motionModel->lookupOdomTransform(msg->header.stamp, odomTransform))
    return;

  // TODO: another lookup, not needed (combine w. above)?
  tf::Stamped<tf::Pose> odomPose;
  m_motionModel->lookupOdomPose(msg->header.stamp, odomPose);

  float length = odomTransform.getOrigin().length();
  if (length > 0.1){
    ROS_WARN("Length of odometry change unexpectedly high: %f", length);
    //return;
  }

  m_translationSinceScan += length;
  double yaw, pitch, roll;
  odomTransform.getBasis().getRPY(roll, pitch, yaw);
  if (std::abs(yaw) > 0.15){
    ROS_WARN("Yaw of odometry change unexpectedly high: %f", yaw);
    //return;
  }
  m_rotationSinceScan += std::abs(yaw);

  bool sensor_integrated = false;
  if (!m_paused
      && (m_translationSinceScan >= m_observationThresholdTrans || m_rotationSinceScan >= m_observationThresholdRot
          || !m_firstLaserReceived))
  {

    // apply motion model with temporal sampling:
    m_motionModel->applyOdomTransformTemporal(m_particles, msg->header.stamp, m_temporalSamplingRange);

    // transformation from torso frame to laser
    // TODO: this takes the latest tf, assumes it did not change over temp. sampling!
    tf::StampedTransform torsoToLaser;
    if (!m_motionModel->lookupLaserTransform(msg->header.frame_id, msg->header.stamp, torsoToLaser))
      return;


    //### Particles in log-form from here...
    toLogForm();

    // integrated pose (z, roll, pitch) meas. only if data OK:
    double poseHeight;
    bool imuMsgOk;
    double angleX, angleY;
    if(m_useIMU) {
      ros::Time imuStamp;
      imuMsgOk = getImuMsg(msg->header.stamp, imuStamp, angleX, angleY);
    } else {
      tf::Stamped<tf::Pose> lastOdomPose;
      if(m_motionModel->lookupOdomPose(msg->header.stamp, lastOdomPose)) {
        double dropyaw;
        lastOdomPose.getBasis().getRPY(angleX, angleY, dropyaw);
        imuMsgOk = true;
      }
    }

    if(imuMsgOk) {
      if (!m_motionModel->lookupPoseHeight(msg->header.stamp, poseHeight)) {
        ROS_WARN("Could not obtain pose height in localization, skipping Pose integration");
      } else {
        m_observationModel->integratePoseMeasurement(m_particles, angleX, angleY, poseHeight);
      }
    }

    PointCloud pc_filtered;
    std::vector<float> laserRangesSparse;
    prepareLaserPointCloud(msg, pc_filtered, laserRangesSparse);


    m_filteredPointCloudPub.publish(pc_filtered);

    m_observationModel->integrateMeasurement(m_particles, pc_filtered, laserRangesSparse, msg->range_max, torsoToLaser);

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

    m_firstLaserReceived = true;
    m_rotationSinceScan = 0.0;
    m_translationSinceScan = 0.0;
    sensor_integrated = true;

#if defined(_BENCH_TIME)
    double dt = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO_STREAM("Observations for "<< m_numParticles << " particles took "
                    << dt << "s (="<<dt/m_numParticles<<"s/particle)");
#endif
  } else{ // no observation necessary: propagate particles forward by full interval

    m_motionModel->applyOdomTransform(m_particles, odomTransform);
  }

  m_motionModel->storeOdomPose(odomPose);
  publishPoseEstimate(msg->header.stamp, sensor_integrated);
  m_lastLaserTime = msg->header.stamp;
}

void HumanoidLocalization::prepareLaserPointCloud(const sensor_msgs::LaserScanConstPtr& laser, PointCloud& pc, std::vector<float>& ranges) const{
  // prepare laser message:
  unsigned numBeams = laser->ranges.size();
  unsigned step = computeBeamStep(numBeams);

  unsigned int numBeamsSkipped = 0;

  // range_min of laser is also used to filter out wrong messages:
  double laserMin = std::max(double(laser->range_min), m_filterMinRange);

  // (range_max readings stay, will be used in the sensor model)

  ranges.reserve(m_numSensorBeams+3);

  // build a point cloud
  pc.header = laser->header;
  pc.points.reserve(m_numSensorBeams+3);
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
      std::cout << range << std::endl;
      numBeamsSkipped++;
    }

  }
  pc.height = 1;
  pc.width = pc.points.size();
  pc.is_dense = true;
  ROS_INFO("%u/%zu laser beams skipped (out of valid range)", numBeamsSkipped, ranges.size());
}

unsigned HumanoidLocalization::computeBeamStep(unsigned numBeams) const{
  unsigned step = 1;
  if (m_numSensorBeams > 1){
    // from "amcl_node"
    step = (numBeams -1) / (m_numSensorBeams - 1);
    if (step < 1)
      step = 1;
  } else if (m_numSensorBeams == 1){
    step = numBeams;
  }

  return step;
}

void HumanoidLocalization::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  //ROS_INFO("received point cloud. organized: %d dense: %d size %zu", (msg->isOrganized()) ? 1 : 0, (msg->is_dense) ? 1 : 0, msg->size());
  //it's organized and NOT dense, hence it might contains points with a value of NaN or Inf

  //tf::StampedTransform torsoToLaser;
  //m_observationModel->integratePointCloudMeasurement(m_particles, torsoToLaser, msg);

  if (!m_motionModel || !m_observationModel){
    ROS_ERROR("MotionModel or ObservationModel is NULL in localization!");
    return;
  }

  if (!m_initialized){
    ROS_WARN("Loclization not initialized yet, skipping point cloud callback.");
    return;
  }

  ros::Duration timediff = msg->header.stamp - m_lastLaserTime;
  if (m_firstLaserReceived && timediff < ros::Duration(0.0)){
    ROS_WARN("Ignoring received laser data that is %f s older than previous data!", timediff.toSec());
    return;
  }

  // compute current odometry transform first, see if it would trigger the
  // integration of observations
  tf::Transform odomTransform;
  if (!m_motionModel->lookupOdomTransform(msg->header.stamp, odomTransform))
    return;

  // TODO: another lookup, not needed (combine w. above)?
  tf::Stamped<tf::Pose> odomPose;
  m_motionModel->lookupOdomPose(msg->header.stamp, odomPose);

  float length = odomTransform.getOrigin().length();
  if (length > 0.1){
    ROS_WARN("Length of odometry change unexpectedly high: %f", length);
  }

  m_translationSinceScan += length;
  double yaw, pitch, roll;
  odomTransform.getBasis().getRPY(roll, pitch, yaw);
  if (std::abs(yaw) > 0.15){
    ROS_WARN("Yaw of odometry change unexpectedly high: %f", yaw);
  }
  m_rotationSinceScan += std::abs(yaw);

  // transformation from torso frame to laser
  // TODO: this takes the latest tf, assumes it did not change over temp. sampling!
  double headYaw, headPitch, headRoll;
  tf::StampedTransform torsoToLaser;
  if (!m_motionModel->lookupLaserTransform(msg->header.frame_id, msg->header.stamp, torsoToLaser))
    return; //TODO: should we apply applyOdomTransformTemporal, before returning

  // lookup Transfrom Sensor to BaseFootprint
  tf::StampedTransform sensorToBaseFootprint;
  try{
    m_tfListener.waitForTransform(m_baseFootprintId, msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
    m_tfListener.lookupTransform(m_baseFootprintId, msg->header.frame_id, msg->header.stamp, sensorToBaseFootprint);


  }catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error for pointCloudCallback: " << ex.what() << ", quitting callback.\n");
    return;
  }



  torsoToLaser.getBasis().getRPY(headRoll, headPitch, headYaw);
  double headYawRotationSinceScan = std::abs(headYaw - m_headYawRotationLastScan);
  double headPitchRotationSinceScan = std::abs(headPitch - m_headPitchRotationLastScan);

  //ROS_INFO("%f %f %f", headYawRotationSinceScan, yaw, m_headYawRotationLastScan);

  /*ROS_INFO("%d %d %d %d %d", m_translationSinceScan >= m_observationThresholdTrans, m_rotationSinceScan >= m_observationThresholdRot,
         headYawRotationSinceScan>= m_observationThresholdHeadYawRot, headPitchRotationSinceScan >= m_observationThresholdHeadPitchRot
         , !m_firstLaserReceived);
   */
  bool integration_happend = false;
  if (!m_paused
      && (m_translationSinceScan >= m_observationThresholdTrans || m_rotationSinceScan >= m_observationThresholdRot ||
          headYawRotationSinceScan>= m_observationThresholdHeadYawRot || headPitchRotationSinceScan >= m_observationThresholdHeadPitchRot
          || !m_firstLaserReceived))
  {
    ROS_INFO("integrating new measurement");

    // apply motion model with temporal sampling:
    m_motionModel->applyOdomTransformTemporal(m_particles, msg->header.stamp, m_temporalSamplingRange);


    //### Particles in log-form from here...
    toLogForm();

    // integrated pose (z, roll, pitch) meas. only if data OK:
    double poseHeight;
    bool imuMsgOk;
    double angleX, angleY;
    if(m_useIMU) {
      ros::Time imuStamp;
      imuMsgOk = getImuMsg(msg->header.stamp, imuStamp, angleX, angleY);
    } else {
      tf::Stamped<tf::Pose> lastOdomPose;
      if(m_motionModel->lookupOdomPose(msg->header.stamp, lastOdomPose)) {
        double dropyaw;
        lastOdomPose.getBasis().getRPY(angleX, angleY, dropyaw);
        imuMsgOk = true;
      }
    }

    if(imuMsgOk) {
      if (!m_motionModel->lookupPoseHeight(msg->header.stamp, poseHeight)) {
        ROS_WARN("Could not obtain pose height in localization, skipping Pose integration");
      } else {
        m_observationModel->integratePoseMeasurement(m_particles, angleX, angleY, poseHeight);
      }
    }

    m_observationModel->integratePointCloudMeasurement(m_particles, torsoToLaser, msg, sensorToBaseFootprint);
    integration_happend = true;

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

    m_firstLaserReceived = true;
    m_rotationSinceScan = 0.0;
    m_translationSinceScan = 0.0;
    m_headYawRotationLastScan = headYaw;
    m_headPitchRotationLastScan = headPitch;

  } else{ // no observation necessary: propagate particles forward by full interval
    m_motionModel->applyOdomTransform(m_particles, odomTransform);
  }

  m_motionModel->storeOdomPose(odomPose);
  publishPoseEstimate(msg->header.stamp,integration_happend);
  m_lastLaserTime = msg->header.stamp;
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
    getRPY(closestOlder->orientation, olderX, olderY);
    getRPY(closestNewer->orientation, newerX, newerY);
    angleX   = weightOlder * olderX  + weightNewer * newerX;
    angleY   = weightOlder * olderY + weightNewer * newerY;
    ROS_DEBUG("Msg: %.3f, Interpolate [%.3f .. %.3f .. %.3f]\n", stamp.toSec(), closestOlder->header.stamp.toSec(),
              imuStamp.toSec(), closestNewer->header.stamp.toSec());
    return true;
  } else if(closestOlderStamp < maxAge || closestNewerStamp < maxAge) {
    // Return closer one
    ItT it = (closestOlderStamp < closestNewerStamp) ? closestOlder : closestNewer;
    imuStamp = it->header.stamp;
    getRPY(it->orientation, angleX, angleY);
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
      poseHeightOk = m_motionModel->lookupPoseHeight(stamp, poseHeight);
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
            getRPY(m_lastIMUMsgBuffer.back().orientation, roll, pitch);
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

  // sample from intial pose covariance:
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
  m_translationSinceScan = 0.0;
  m_rotationSinceScan = 0.0;
  m_firstLaserReceived = false;
  m_initialized = true;

  publishPoseEstimate(msg->header.stamp, false);
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
  //assert(scale >= 0.0);
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

  for (unsigned i = 0; i < numParticles; ++i){
    m_particles[i].pose = oldParticles[indices[i]].pose;
    m_particles[i].weight = newWeight;
  }
}

void HumanoidLocalization::initGlobal(){
  ROS_INFO("Initializing with uniform distribution");

  m_mapModel->initGlobal(m_particles, m_initPose, m_initNoiseStd, m_rngUniform, m_rngNormal);


  ROS_INFO("Global localization done");
  m_motionModel->reset();
  m_translationSinceScan = 0.0;
  m_rotationSinceScan = 0.0;
  m_firstLaserReceived = false;
  m_initialized = true;

  publishPoseEstimate(ros::Time::now(), false);

}

void HumanoidLocalization::publishPoseEstimate(const ros::Time& time, bool publish_eval){

  //
  // send all hypotheses as arrows:
  //

  m_poseArray.header.stamp = time;

  if (m_poseArray.poses.size() != m_particles.size())
    m_poseArray.poses.resize(m_particles.size());

  for (unsigned i = 0; i < m_particles.size(); ++i){
    tf::poseTFToMsg(m_particles[i].pose, m_poseArray.poses[i]);
  }

  m_poseArrayPub.publish(m_poseArray);

  //
  // send best particle as pose and one array:
  //
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

  if (publish_eval)
    m_poseEvalPub.publish(p);

  geometry_msgs::PoseArray bestPose;
  bestPose.header = p.header;
  bestPose.poses.resize(1);
  tf::poseTFToMsg(bestParticlePose, bestPose.poses[0]);
  m_bestPosePub.publish(bestPose);


  // TODO: move to own node (eval)
  /**
  ///////////////////////////////////////////////////////
  // Send poses for evaluation synced to localization
  ///////////////////////////////////////////////////////

  // send incremental odom pose (synced to localization)
  tf::Stamped<tf::Pose> lastOdomPose;
  if (m_motionModel->getLastOdomPose(lastOdomPose)){
    geometry_msgs::PoseStamped odomPoseMsg;
    tf::poseStampedTFToMsg(lastOdomPose, odomPoseMsg);
    m_poseOdomPub.publish(odomPoseMsg);
  }


  // send truepose when available (and enabled)
  if (m_syncedTruepose){
    geometry_msgs::PoseStamped truePose;
    tf::Stamped<tf::Pose> truePoseTF;
#if ROS_VERSION_MINIMUM(1, 8, 7) // fuerte
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), time, "true_torso_Link");
#else
    tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(), btVector3(0,0,0)), time, "true_torso_Link");
#endif

    try {
      m_tfListener.waitForTransform(m_globalFrameId, "true_torso_Link", time, ros::Duration(0.1));
      m_tfListener.transformPose(m_globalFrameId, ident, truePoseTF);
      tf::poseStampedTFToMsg(truePoseTF, truePose);
      m_poseTruePub.publish(truePose);

    } catch (const tf::TransformException& e) {
      ROS_WARN("Failed to obtain truepose from tf (%s)", e.what());
    }
  }

  **/

  //
  // transform to odom frame in global map frame:
  //
  tf::Stamped<tf::Pose> odomToMapTF;
  try{
    tf::Stamped<tf::Pose> baseToMapTF(bestParticlePose.inverse(),time, m_baseFrameId);
    m_tfListener.transformPose(m_odomFrameId, baseToMapTF, odomToMapTF);
  } catch (const tf::TransformException& e){
    ROS_WARN("Failed to subtract base to odom transform: %s", e.what());
    return;
  }

  tf::Transform latestTF(tf::Quaternion(odomToMapTF.getRotation()), tf::Point(odomToMapTF.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used
  // see ROS amcl_node

  // TODO: param
  ros::Duration transformTolerance(0.1);
  ros::Time transformExpiration = (time + transformTolerance);

  tf::StampedTransform tmp_tf_stamped(latestTF.inverse(),	transformExpiration, m_globalFrameId, m_odomFrameId);

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

  meanPose.setBasis(btMatrix3x3(0,0,0,0,0,0,0,0,0));
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
  meanPose.getBasis() = meanPose.getBasis().scaled(btVector3(1.0/m_numParticles, 1.0/m_numParticles, 1.0/m_numParticles));

  // TODO: Why do we have to normalize the quaternion here?
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
      m_firstLaserReceived = false;
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
    m_firstLaserReceived = false;
  } else {
    ROS_WARN("Received a request to resume localization, but is not paused.");
  }

  return true;
}

}

