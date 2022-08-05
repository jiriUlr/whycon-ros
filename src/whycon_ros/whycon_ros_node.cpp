#include "whycon_ros/whycon_ros_node.h"

#include <string>
#include <algorithm>

#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "whycon/MarkerArray.h"
#include "whycon/Marker.h"


namespace whycon_ros
{

bool CWhyconROSNode::getGuiSettingsCallback(whycon::GetGuiSettings::Request &req, whycon::GetGuiSettings::Response &res)
{
    ROS_INFO("getGuiSettingsCallback");
    res.draw_coords = whycon_.getDrawCoords();
    res.draw_segments = whycon_.getDrawSegments();
    res.coords = whycon_.getCoordinates();
    return true;
}

bool CWhyconROSNode::setDrawingCallback(whycon::SetDrawing::Request &req, whycon::SetDrawing::Response &res)
{
    ROS_INFO("setDrawingCallback coords %d segs %d", req.draw_coords, req.draw_segments);
    whycon_.setDrawing(req.draw_coords, req.draw_segments);
    res.success = true;
    return true;
}

bool CWhyconROSNode::setCoordsCallback(whycon::SetCoords::Request &req, whycon::SetCoords::Response &res)
{
    ROS_INFO("setCoordsCallback %d", req.coords);
    try
    {
        whycon_.setCoordinates(static_cast<whycon::ETransformType>(req.coords));
        res.success = true;
    }
    catch(const std::exception& e)
    {
        res.success = false;
        res.msg = e.what();
    }
    return true;
}

bool CWhyconROSNode::setCalibMethodCallback(whycon::SetCalibMethod::Request &req, whycon::SetCalibMethod::Response &res)
{
    ROS_INFO("setCalibMethodCallback %d", req.method);
    try
    {
        if(req.method == 0)
        {
            whycon_.autocalibration();
            res.success = true;
        }
        else if(req.method == 1)
        {
            whycon_.manualcalibration();
            res.success = true;
        }
        else
        {
            res.success = false;
            res.msg = "ERROR in setting calibration method : unkown method '" + std::to_string(req.method) + "'";
        }
    }
    catch(const std::exception& e)
    {
        res.success = false;
        res.msg = e.what();
    }
    return true;
}

bool CWhyconROSNode::setCalibPathCallback(whycon::SetCalibPath::Request &req, whycon::SetCalibPath::Response &res)
{
    ROS_INFO("setCalibPathCallback action %s path %s", req.action.c_str(), req.path.c_str());
    try
    {
        if(req.action == "load")
        {
            whycon_.loadCalibration(req.path);
            res.success = true;
        }
        else if(req.action == "save")
        {
            whycon_.saveCalibration(req.path);
            res.success = true;
        }
        else
        {
            res.success = false;
            res.msg = "ERROR in setting calibration path : unkown action '" + req.action + "'";
        }
    }
    catch(const std::exception& e)
    {
        res.success = false;
        res.msg = e.what();
    }
    return true;
}

bool CWhyconROSNode::selectMarkerCallback(whycon::SelectMarker::Request &req, whycon::SelectMarker::Response &res)
{
    ROS_INFO("selectMarkerCallback x %f y %f", req.point.x, req.point.y);
    whycon_.selectMarker(req.point.x, req.point.y);
    return true;
}

void CWhyconROSNode::reconfigureCallback(whycon::whyconConfig& config, uint32_t level)
{
    ROS_INFO("[Reconfigure Request]\n"
        "identify %s circle_diameter %lf num_markers %d\n"
        "min_size %d field_length %lf field_width %lf\n"
        "initial_circularity_tolerance %lf final_circularity_tolerance %lf\n"
        "area_ratio_tolerance %lf\n"
        "center_distance_tolerance_ratio %lf center_distance_tolerance_abs %lf\n",
        (config.identify) ? "True" : "False",
        config.circle_diameter, config.num_markers, config.min_size, config.field_length,
        config.field_width, config.initial_circularity_tolerance,
        config.final_circularity_tolerance, config.area_ratio_tolerance,
        config.center_distance_tolerance_ratio, config.center_distance_tolerance_abs
        );

    whycon_.updateConfiguration(
        config.identify, config.circle_diameter, config.num_markers, config.min_size,
        config.field_length, config.field_width, config.initial_circularity_tolerance,
        config.final_circularity_tolerance, config.area_ratio_tolerance,
        config.center_distance_tolerance_ratio, config.center_distance_tolerance_abs
        );

    identify_ = config.identify;
}

void CWhyconROSNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    if(msg->K[0] == 0)
    {
        ROS_ERROR_ONCE("ERROR: Camera is not calibrated!");
        return;
    }

    if(!std::equal(intrinsic_mat_.begin(), intrinsic_mat_.end(), msg->K.begin()))
        intrinsic_mat_.assign(msg->K.begin(), msg->K.end());

    if(!std::equal(distortion_coeffs_.begin(), distortion_coeffs_.end(), msg->D.begin()))
        distortion_coeffs_.assign(msg->D.begin(), msg->D.end());

    whycon_.updateCameraInfo(intrinsic_mat_, distortion_coeffs_);
}

void CWhyconROSNode::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    // convert sensor_msgs::Image msg to whycon CRawImage
    image_->updateImage((unsigned char*)&msg->data[0], msg->width, msg->height, msg->step / msg->width);

    whycon_.processImage(image_, whycon_detections_);

    // generate information about markers into msgs
    whycon::MarkerArray marker_array;
    marker_array.header.stamp = ros::Time::now();
    marker_array.header.frame_id = msg->header.frame_id;
    
    // tf vector
    std::vector<geometry_msgs::TransformStamped> transform_array;

    // Generate RVIZ visualization marker
    visualization_msgs::MarkerArray visual_array;

    for(const whycon::SMarker &detection : whycon_detections_)
    {
        whycon::Marker marker;

        marker.id = detection.seg.ID;
        marker.size = detection.seg.size;
        marker.u = detection.seg.x;
        marker.v = detection.seg.y;
        marker.angle = detection.obj.angle;

        // Convert to ROS standard Coordinate System
        marker.position.position.x = detection.obj.x;
        marker.position.position.y = detection.obj.y;
        marker.position.position.z = detection.obj.z;
        marker.position.orientation.x = detection.obj.qx;
        marker.position.orientation.y = detection.obj.qy;
        marker.position.orientation.z = detection.obj.qz;
        marker.position.orientation.w = detection.obj.qw;
        marker.rotation.x = detection.obj.roll;
        marker.rotation.y = detection.obj.pitch;
        marker.rotation.z = detection.obj.yaw;
        marker_array.markers.push_back(marker);

        if(identify_ && publish_tf_)
        {
            geometry_msgs::TransformStamped transform_stamped;

            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = msg->header.frame_id;
            transform_stamped.child_frame_id = "marker_" + std::to_string(detection.seg.ID);
            transform_stamped.transform.translation.x = detection.obj.x;
            transform_stamped.transform.translation.y = detection.obj.y;
            transform_stamped.transform.translation.z = detection.obj.z;
            transform_stamped.transform.rotation.x = detection.obj.qx;
            transform_stamped.transform.rotation.y = detection.obj.qy;
            transform_stamped.transform.rotation.z = detection.obj.qz;
            transform_stamped.transform.rotation.w = detection.obj.qw;

            transform_array.push_back(transform_stamped);
        }

        if(publish_visual_)
        {
            visualization_msgs::Marker visual_marker;
            visual_marker.header.stamp = ros::Time::now();
            visual_marker.header.frame_id = msg->header.frame_id;
            visual_marker.ns = "whycon";
            visual_marker.id = detection.seg.ID;
            visual_marker.type = visualization_msgs::Marker::SPHERE;
            visual_marker.action = visualization_msgs::Marker::MODIFY;

            visual_marker.pose.position.x = detection.obj.x;
            visual_marker.pose.position.y = detection.obj.y;
            visual_marker.pose.position.z = detection.obj.z;
            visual_marker.pose.orientation.x = detection.obj.qx;
            visual_marker.pose.orientation.y = detection.obj.qy;
            visual_marker.pose.orientation.z = detection.obj.qz;
            visual_marker.pose.orientation.w = detection.obj.qw;

            visual_marker.scale.x = 0.01;//circleDiameter;  // meters
            visual_marker.scale.y = circle_diameter_;//circleDiameter;
            visual_marker.scale.z = circle_diameter_;
            visual_marker.color.r = 0.0;
            visual_marker.color.g = 1.0;
            visual_marker.color.b = 0.0;
            visual_marker.color.a = 1.0;
            visual_marker.lifetime = ros::Duration(0.1);  // secs
            visual_marker.frame_locked = true;

            visual_array.markers.push_back(visual_marker);
        }
    }

    // publishing detected markers
    if(marker_array.markers.size() > 0)
    {
        markers_pub_.publish(marker_array);

        if(publish_visual_)
            visual_pub_.publish(visual_array);

        for(int i = 0; i < transform_array.size(); i++)
            tf_broad_.sendTransform(transform_array[i]);
    }

    if(use_gui_)
    {
        std::memcpy((void*)&msg->data[0], image_->data_, msg->step * msg->height);
        img_pub_.publish(msg);
    }

    whycon_detections_.clear();
}

void CWhyconROSNode::start()
{
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10000);
    }
}

CWhyconROSNode::CWhyconROSNode() :
    intrinsic_mat_(9)
    , distortion_coeffs_(5)
{
    ros::NodeHandle nh("~");        // ROS node handle
    
    int id_bits;                    // num of ID bits
    int id_samples;                 // num of samples to identify ID
    int hamming_dist;               // hamming distance of ID code
    int num_markers;                // initial number of markers
    std::string calib_path;
    int coords_method;

    // obtain parameters
    nh.param("use_gui", use_gui_, true);
    nh.param("pub_visual", publish_visual_, false);
    nh.param("pub_tf", publish_tf_, false);
    nh.param("circle_diameter", circle_diameter_, 0.122);
    nh.param("id_bits", id_bits, 6);
    nh.param("id_samples", id_samples, 360);
    nh.param("hamming_dist", hamming_dist, 1);
    nh.param("num_markers", num_markers, 10);
    nh.param("calib_file", calib_path, std::string(""));
    nh.param("coords_method", coords_method, 0);

    int default_width = 640;
    int default_height = 480;
    image_ = new whycon::CRawImage(default_width, default_height, 3);
    whycon_.init(circle_diameter_, use_gui_, id_bits, id_samples, hamming_dist, num_markers, default_width, default_height);
    
    // subscribe to camera topics
    image_transport::ImageTransport it(nh);
    cam_info_sub_ = nh.subscribe("/camera/camera_info", 1, &CWhyconROSNode::cameraInfoCallback, this);
    img_sub_ = it.subscribe("/camera/image_raw", 1, &CWhyconROSNode::imageCallback, this);
    
    // advertise topics with markers description, RVIZ visualization and GUI visualization
    img_pub_ = it.advertise("processed_image", 1);
    markers_pub_ = nh.advertise<whycon::MarkerArray>("markers", 1);
    visual_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualisation", 1);

    // advertise services
    drawing_srv_ = nh.advertiseService("set_drawing", &CWhyconROSNode::setDrawingCallback, this);
    coord_system_srv_ = nh.advertiseService("set_coords", &CWhyconROSNode::setCoordsCallback, this);
    calib_method_srv_ = nh.advertiseService("set_calib_method", &CWhyconROSNode::setCalibMethodCallback, this);
    calib_path_srv_ = nh.advertiseService("set_calib_path", &CWhyconROSNode::setCalibPathCallback, this);
    select_marker_srv_ = nh.advertiseService("select_marker", &CWhyconROSNode::selectMarkerCallback, this);
    gui_settings_srv_ = nh.advertiseService("get_gui_settings", &CWhyconROSNode::getGuiSettingsCallback, this);

    // create dynamic reconfigure server
    dyn_srv_cb_ = boost::bind(&CWhyconROSNode::reconfigureCallback, this, _1, _2);
    dyn_srv_.setCallback(dyn_srv_cb_);

    if(calib_path.size() > 0)
    {
        try
        {
            whycon_.loadCalibration(calib_path);
            whycon_.setCoordinates(static_cast<whycon::ETransformType>(coords_method));
        }
        catch(const std::exception& e)
        {
            ROS_WARN("Calibration file '%s' could not be loaded. Using camera centric coordinates.", calib_path.c_str());
        }
    }
    else
    {
        ROS_INFO("Calibration file path empty. Using camera centric coordinates.");
    }
}

CWhyconROSNode::~CWhyconROSNode()
{
    delete image_;
}

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "whycon");

    whycon_ros::CWhyconROSNode whycon_ros_node;
    whycon_ros_node.start();

    return 0;
}
