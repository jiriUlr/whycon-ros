#ifndef WHYCON_ROS_CWHYCONROSNODE_H
#define WHYCON_ROS_CWHYCONROSNODE_H

#include <string>
#include <vector>
#include <cstring>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include "whycon/whyconConfig.h"
#include "whycon/MarkerArray.h"
#include "whycon/Marker.h"

#include "whycon/SelectMarker.h"
#include "whycon/SetCalibMethod.h"
#include "whycon/SetCalibPath.h"
#include "whycon/SetCoords.h"
#include "whycon/SetDrawing.h"

#include "whycon/whycon.h"


namespace whycon_ros
{

class CWhyconROSNode
{

    public:
        bool setDrawingCallback(whycon::SetDrawing::Request& req, whycon::SetDrawing::Response& res);

        bool setCoordsCallback(whycon::SetCoords::Request& req, whycon::SetCoords::Response& res);

        bool setCalibMethodCallback(whycon::SetCalibMethod::Request& req, whycon::SetCalibMethod::Response& res);

        bool setCalibPathCallback(whycon::SetCalibPath::Request& req, whycon::SetCalibPath::Response& res);

        bool selectMarkerCallback(whycon::SelectMarker::Request& req, whycon::SelectMarker::Response& res);

        void reconfigureCallback(whycon::whyconConfig& config, uint32_t level);

        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        void start();

        CWhyconROSNode();

        ~CWhyconROSNode();

    private:
        
        ros::Subscriber cam_info_sub_;          // camera info subscriber
        image_transport::Subscriber img_sub_;   // camera image raw subscriber

        image_transport::Publisher img_pub_;    // image publisher for GUI
        ros::Publisher markers_pub_;            // publisher of MarkerArray
        ros::Publisher visual_pub_;             // publisher of MarkerArray for RVIZ

        bool draw_coords_;
        ros::ServiceServer drawing_srv_;
        ros::ServiceServer calib_path_srv_;
        ros::ServiceServer coord_system_srv_;
        ros::ServiceServer calib_method_srv_;
        ros::ServiceServer select_marker_srv_;
        
        bool publish_visual_;   // whether to publish visualization msgs
        bool use_gui_;          // generate images for graphic interface?
        whycon::CWhycon whycon_;        // WhyCon instance
        whycon::CRawImage *image_;      // image wrapper for WhyCon
        double circle_diameter_;  // marker diameter [m]

        std::vector<whycon::SMarker> whycon_detections_;    // array of detected markers
        
        std::vector<double> intrinsic_mat_;        // intrinsic matrix from camera_info topic
        std::vector<double> distortion_coeffs_;    // distortion parameters from camera_info topic

};

}


#endif
