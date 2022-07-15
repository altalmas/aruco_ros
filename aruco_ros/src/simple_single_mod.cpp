/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <aruco_ros/DetectorConfig.h>

// ABD
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <aruco_ros/Marker.h>
#include <aruco_ros/MarkerArray.h>

#include <unordered_set>

using std::vector;
using cv::Mat;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher debug_pub;
  ros::Publisher transform_pub;
  ros::Publisher marker_pub; // rviz visualization marker
  ros::Publisher simple_pose_pub; // important
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;
  
  // ABD
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  std::shared_ptr<dynamic_reconfigure::Server<aruco_ros::DetectorConfig>> dyn_srv_;
  bool enabled_ = true;
  Mat camera_matrix_, dist_coeffs_;
  aruco_ros::MarkerArray array_;
  bool send_tf_;
  std::unordered_set<int> map_markers_ids_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::string frame_id_prefix_;
  geometry_msgs::PoseStamped simple_pose_;

public:
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    debug_pub = it.advertise("debug", 1);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
	simple_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("simple_pose", 1);
		
    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 6);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("ArUco node started with marker size of %f m and marker id to track: %d", marker_size, marker_id);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
             marker_frame.c_str());

    // TODO : check later for explanation
    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
    
    // ABD
    int dictionary;
	dictionary = nh.param("dictionary", 10);
    dictionary_ = cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary));
	parameters_ = cv::aruco::DetectorParameters::create();
	
	dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<aruco_ros::DetectorConfig>>(nh);
	dyn_srv_->setCallback(std::bind(&ArucoSimple::paramCallback, this, std::placeholders::_1, std::placeholders::_2));
	
	camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
	send_tf_ = nh.param("send_tf", true);
	br_.reset(new tf2_ros::TransformBroadcaster());
	frame_id_prefix_ = nh.param<std::string>("frame_id_prefix", "aruco_");
	
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((debug_pub.getNumSubscribers() == 0)
        && (transform_pub.getNumSubscribers() == 0)
        && (marker_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      try
      {
        Mat inImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        vector<int> ids;
		vector<vector<cv::Point2f>> corners, rejected;
		vector<cv::Vec3d> rvecs, tvecs;

        // detection results will go into "markers"

        // ok, let's detect
		cv::aruco::detectMarkers(inImage, dictionary_, corners, ids, parameters_, rejected);
		
		array_.header.stamp = msg->header.stamp;
		// array_.header.frame_id = msg->header.frame_id;
		array_.header.frame_id = marker_frame;
		array_.markers.clear();
		
		if (ids.size() != 0) {
			
			// estimate the pose for each detected marker
			cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix_, dist_coeffs_,
				                                     rvecs, tvecs);
				                                     
		    //ROS_WARN("corners[0][0].x = %f", corners[0][0].x);
	        //ROS_WARN("corners[0][0].y = %f", corners[0][0].y);
	        
	        //ROS_WARN("corners[0][1].x = %f", corners[0][1].x);
	        //ROS_WARN("corners[0][1].y = %f", corners[0][1].y);
	        
	        //ROS_WARN("corners[0][2].x = %f", corners[0][2].x);
	        //ROS_WARN("corners[0][2].y = %f", corners[0][2].y);
	        
	        //ROS_WARN("corners[0][3].x = %f", corners[0][3].x);
	        //ROS_WARN("corners[0][3].y = %f", corners[0][3].y);
		        
		    array_.markers.reserve(ids.size());
			aruco_ros::Marker marker;
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = msg->header.stamp;
			transform.header.frame_id = marker_frame;
			
			for (unsigned int i = 0; i < ids.size(); i++) {
			    ROS_WARN("id = %d", ids[i]);    // = 19
			    //ROS_WARN("i = %d", i);        // = 0
			    
				marker.id = ids[i];
				marker.length = getMarkerLength(marker.id);
				fillCorners(marker, corners[i]);
				
				fillPose(marker.pose, rvecs[i], tvecs[i]);
				
				if (send_tf_) {
					transform.child_frame_id = getChildFrameId(ids[i]);

					// check if such static transform is in the map
					if (map_markers_ids_.find(ids[i]) == map_markers_ids_.end()) {
						transform.transform.rotation = marker.pose.orientation;
						fillTranslation(transform.transform.translation, tvecs[i]);
						br_->sendTransform(transform);
					}
				}
				
				array_.markers.push_back(marker);
				
				// publish the pose of the wanted marker only
				if (ids[i] == marker_id){
				    simple_pose_.header = transform.header;
				    fillPose(simple_pose_.pose, rvecs[i], tvecs[i]);
				    simple_pose_pub.publish(simple_pose_);
				}
			
			}
			
        }
        
        
        // Publish debug image
		if (debug_pub.getNumSubscribers() != 0) {
			Mat debug = inImage.clone();
			cv::aruco::drawDetectedMarkers(debug, corners, ids); // draw markers
			for (unsigned int i = 0; i < ids.size(); i++)
				cv::aruco::drawAxis(debug, camera_matrix_, dist_coeffs_,
				                    rvecs[i], tvecs[i], getMarkerLength(ids[i]));

			cv_bridge::CvImage out_msg;
			out_msg.header.frame_id = marker_frame;
			out_msg.header.stamp = msg->header.stamp;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = debug;
			debug_pub.publish(out_msg.toImageMsg());
		}

        // TODO: check later for explanation
        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }


  inline void fillTranslation(geometry_msgs::Vector3& translation, const cv::Vec3d& tvec) const
  {
		translation.x = tvec[0];
		translation.y = tvec[1];
		translation.z = tvec[2];
  }
	
  inline std::string getChildFrameId(int id) const
  {
		return frame_id_prefix_ + std::to_string(id);
  }
	
  inline void fillPose(geometry_msgs::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec) const
  {
		pose.position.x = tvec[0];
		pose.position.y = tvec[1];
		pose.position.z = tvec[2];

		double angle = norm(rvec);
		cv::Vec3d axis = rvec / angle;

		tf2::Quaternion q;
		q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

		// change nan to 0.0 to avoid tf error
		if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z()))
		{
			pose.orientation.w = 1.0;
			pose.orientation.x = 0.0;
			pose.orientation.y = 0.0;
			pose.orientation.z = 0.0;
		}
		else
		{
			pose.orientation.w = q.w();
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
		}
  }
	
  inline double getMarkerLength(int id)
  {
	    return marker_size;
  }
  
  inline void fillCorners(aruco_ros::Marker& marker, const vector<cv::Point2f>& corners) const
  {
		marker.c1.x = corners[0].x;
		marker.c2.x = corners[1].x;
		marker.c3.x = corners[2].x;
		marker.c4.x = corners[3].x;
		marker.c1.y = corners[0].y;
		marker.c2.y = corners[1].y;
		marker.c3.y = corners[2].y;
		marker.c4.y = corners[3].y;
  }
	
	
  void parseCameraInfo(const sensor_msgs::CameraInfo& cinfo, cv::Mat& matrix, cv::Mat& dist)
  {
        for (unsigned int i = 0; i < 3; ++i)
	        for (unsigned int j = 0; j < 3; ++j)
		        matrix.at<double>(i, j) = cinfo.K[3 * i + j];
        dist = cv::Mat(cinfo.D, true);
        
  }
  
  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    parseCameraInfo(msg, camera_matrix_, dist_coeffs_);

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
  
  void paramCallback(aruco_ros::DetectorConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
		parameters_->adaptiveThreshConstant = config.adaptiveThreshConstant;
		parameters_->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
		parameters_->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
		parameters_->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
		parameters_->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
		parameters_->cornerRefinementMethod = config.cornerRefinementMethod;
		parameters_->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
		parameters_->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 4) && (CV_VERSION_REVISION >= 7)) || (CV_VERSION_MAJOR > 3)
		parameters_->detectInvertedMarker = config.detectInvertedMarker;
#endif
		parameters_->errorCorrectionRate = config.errorCorrectionRate;
		parameters_->minCornerDistanceRate = config.minCornerDistanceRate;
		parameters_->markerBorderBits = config.markerBorderBits;
		parameters_->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
		parameters_->minDistanceToBorder = config.minDistanceToBorder;
		parameters_->minMarkerDistanceRate = config.minMarkerDistanceRate;
		parameters_->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
		parameters_->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
		parameters_->minOtsuStdDev = config.minOtsuStdDev;
		parameters_->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
		parameters_->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
		parameters_->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 4) && (CV_VERSION_REVISION >= 2)) || (CV_VERSION_MAJOR > 3)
		parameters_->aprilTagQuadDecimate = config.aprilTagQuadDecimate;
		parameters_->aprilTagQuadSigma = config.aprilTagQuadSigma;
#endif
	}
	
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
