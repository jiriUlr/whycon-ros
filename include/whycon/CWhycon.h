#ifndef WHYCON__CWHYCON_H
#define WHYCON__CWHYCON_H

#include <vector>
#include <deque>

#include <stdlib.h>
#include <string>
#include <cmath>
//#include <opencv2/opencv.hpp>

// WhyCon/WhyCode libs
#include "whycon/CTimer.h"
#include "whycon/CCircleDetect.h"
#include "whycon/CTransformation.h"
#include "whycon/CNecklace.h"
#include "whycon/CRawImage.h"

// ROS libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <whycon_ros/whyconConfig.h>
#include <whycon_ros/MarkerArray.h>
#include <whycon_ros/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

namespace whycon
{

class CWhycon {

    public:

        int image_width_ = 640;         // default camera resolution
        int image_height_ = 480;        // default camera resolution
        float circle_diameter_;         // default black circle diameter [m];
        float field_length_ = 1.0;      // X dimension of the coordinate system
        float field_width_ = 1.0;       // Y dimension of the coordinate system

        // marker detection variables
        bool identify_ = false;     // whether to identify ID
        int num_markers_;           // num of markers to track
        int num_found_ = 0;         // num of markers detected in the last step
        int num_static_ = 0;        // num of non-moving markers

        // marker identification
        int id_bits_;       // num of ID bits
        int id_samples_;    // num of samples to identify ID
        int hamming_dist_;  // hamming distance of ID code

        CWhycon();
        ~CWhycon();
        
        void init(float circle_diam, bool use_gui, int id_b, int id_s, int ham_dist, int markers, int img_w, int img_h);
        void setDrawing(bool draw_coords, bool draw_segments);
        void setCoordinates(ETransformType type);
        void autocalibration();
        void manualcalibration();
        void loadCalibration(std::string& path);
        void saveCalibration(std::string& path);
        void selectMarker(float x, float y);
        void updateConfiguration(bool id, float diam, int markers, int size, double fl, double fw, double ict, double fct, double art, double cdtr, double cdta);
        void updateCameraInfo(std::vector<double>& intrinsic_mat, std::vector<double>& distortion_coeffs);
        void processImage(CRawImage *image, std::vector<SMarker>& whycon_detections);

    private:

        // GUI-related stuff
        bool use_gui_;          // whether use graphic interface
        bool draw_coords_;      // draw coordinates at the marker's positions
        bool draw_segments_;    // draw segmentation outcome
        int eval_time_;         // time required to detect the patterns

        CTransformation *trans_;    // transformation instance
        CNecklace *decoder_;        // instance to decode marker's ID

        std::deque<SMarker> current_marker_array_;      // array of currently detected markers
        std::deque<SMarker> last_marker_array_;         // array of previously detected markers
        std::deque<CCircleDetect*> detector_array_;     // array of detector instances for each marker

        bool calibrated_coords_;




        // variables related to (auto) calibration
        const int calibration_steps_ = 20;              // how many measurements to average to estimate calibration pattern position (manual calib)
        const int auto_calibration_steps_ = 30;         // how many measurements to average to estimate calibration pattern position (automatic calib)  
        const int auto_calibration_pre_steps_ = 10;     // how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
        int calib_num_ = 5;                             // number of objects acquired for calibration (5 means calibration winished inactive)
        STrackedObject calib_[5];                       // array to store calibration patterns positions
        std::vector<STrackedObject> calib_tmp_;         // array to store several measurements of a given calibration pattern
        int calib_step_ = calibration_steps_ + 2;       // actual calibration step (num of measurements of the actual pattern)
        
        bool autocalibrate_ = false;                    // is the autocalibration in progress ?
        bool mancalibrate_ = false;

        ETransformType last_transform_type_ = TRANSFORM_2D;     // pre-calibration transform (used to preserve pre-calibation transform type)
        int was_markers_ = 1;                                   // pre-calibration number of makrers to track (used to preserve pre-calibation number of markers to track)

        
        

        /*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
        void manualCalib();

        /*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
        void autoCalib();
};

}

#endif
