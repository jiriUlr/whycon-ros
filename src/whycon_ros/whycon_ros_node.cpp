#include "whycon_ros/whycon_ros_node.h"

#include <algorithm>

namespace whycon_ros
{

bool CWhyconROSNode::setDrawingCallback(whycon_ros::SetDrawing::Request& req, whycon_ros::SetDrawing::Response& res)
{
    whycon_.setDrawing(req.draw_coords, req.draw_segments);
    res.success = true;
    return true;
}

bool CWhyconROSNode::setCoordsCallback(whycon_ros::SetCoords::Request& req, whycon_ros::SetCoords::Response& res)
{
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

bool CWhyconROSNode::setCalibMethodCallback(whycon_ros::SetCalibMethod::Request& req, whycon_ros::SetCalibMethod::Response& res)
{
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

bool CWhyconROSNode::setCalibPathCallback(whycon_ros::SetCalibPath::Request& req, whycon_ros::SetCalibPath::Response& res)
{
    try
    {
        if(req.action.compare("load"))
        {
            whycon_.loadCalibration(req.path);
            res.success = true;
        }
        else if(req.action.compare("save"))
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

bool CWhyconROSNode::selectMarkerCallback(whycon_ros::SelectMarker::Request& req, whycon_ros::SelectMarker::Response& res)
{
    whycon_.selectMarker(req.point.x, req.point.y);
    return true;
}

void CWhyconROSNode::reconfigureCallback(whycon_ros::whyconConfig& config, uint32_t level)
{
    ROS_INFO("[Reconfigure Request]\n"
        "identify %s circleDiameter %lf numMarkers %d\n"
        "minSize %d fieldLength %lf fieldWidth %lf\n"
        "initialCircularityTolerance %lf finalCircularityTolerance %lf\n"
        "areaRatioTolerance %lf\n"
        "centerDistanceToleranceRatio %lf centerDistanceToleranceAbs %lf\n",
        (config.identify) ? "True" : "False",
        config.circleDiameter, config.numMarkers, config.minSize, config.fieldLength,
        config.fieldWidth, config.initialCircularityTolerance,
        config.finalCircularityTolerance, config.areaRatioTolerance,
        config.centerDistanceToleranceRatio, config.centerDistanceToleranceAbs
        );

    whycon_.updateConfiguration(
        config.identify, config.circleDiameter, config.numMarkers, config.minSize,
        config.fieldLength, config.fieldWidth, config.initialCircularityTolerance,
        config.finalCircularityTolerance, config.areaRatioTolerance,
        config.centerDistanceToleranceRatio, config.centerDistanceToleranceAbs
        );
}

void CWhyconROSNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
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

void CWhyconROSNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // convert sensor_msgs::Image msg to whycon CRawImage
    image_->updateImage((unsigned char*)&msg->data[0], msg->width, msg->height, msg->step / msg->width);

    whycon_.processImage(image_, whycon_detections_);

    // Generate information about markers into msgs
    whycon_ros::MarkerArray marker_array;
    marker_array.header = msg->header;

    for(const whycon::SMarker& detection : whycon_detections_)
    {
        whycon_ros::Marker marker;

        marker.id = detection.seg.ID;
        marker.size = detection.seg.size;
        marker.u = detection.seg.x;
        marker.v = detection.seg.y;
        marker.angle = detection.obj.angle;

        // Convert to ROS standard Coordinate System
        marker.position.position.x = -detection.obj.y;
        marker.position.position.y = -detection.obj.z;
        marker.position.position.z = detection.obj.x;

        geometry_msgs::Quaternion orientation;
        orientation.x = detection.obj.qx;
        orientation.y = detection.obj.qy;
        orientation.z = detection.obj.qz;
        orientation.w = detection.obj.qw;
        marker.position.orientation = orientation;

        // Euler angles
        marker.rotation.x = detection.obj.theta;
        marker.rotation.y = detection.obj.phi;
        marker.rotation.z = detection.obj.psi;

        marker_array.markers.push_back(marker);
    }

    // Generate RVIZ visualization marker
    visualization_msgs::MarkerArray visual_array;

    if(publish_visual_)
    {
        for(whycon_ros::Marker marker : marker_array.markers)
        {
            visualization_msgs::Marker visual_marker;
            visual_marker.header = msg->header;
            visual_marker.ns = "whycon_ros";
            visual_marker.id = marker.id;
            visual_marker.type = visualization_msgs::Marker::SPHERE;
            visual_marker.action = visualization_msgs::Marker::MODIFY;

            visual_marker.pose = marker.position;
            visual_marker.scale.x = 0.5;//circleDiameter;  // meters
            visual_marker.scale.y = 0.25;//circleDiameter;
            visual_marker.scale.z = 0.01;
            visual_marker.color.r = 0.0;
            visual_marker.color.g = 1.0;
            visual_marker.color.b = 0.0;
            visual_marker.color.a = 1.0;
            visual_marker.lifetime = ros::Duration(0.2);  // secs

            visual_array.markers.push_back(visual_marker);
        }
    }

    // publishing detected markers
    if(marker_array.markers.size() > 0)
        markers_pub_.publish(marker_array);

    if(publish_visual_ && visual_array.markers.size() > 0)
        visual_pub_.publish(visual_array);
    
    if(use_gui_)
    {
        std::memcpy((void*)&msg->data[0], image_->data_, msg->step * msg->height);
        img_pub_.publish(msg);
    }
}

void CWhyconROSNode::start()
{
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(30000);
    }
}

CWhyconROSNode::CWhyconROSNode()
{
    ros::NodeHandle nh("~");        // ROS node handle
    
    int id_bits;                    // num of ID bits
    int id_samples;                 // num of samples to identify ID
    int hamming_dist;               // hamming distance of ID code
    int num_markers;                // initial number of markers
    int default_width = 640;
    int default_height = 480;

    // obtain parameters
    nh.param("use_gui", use_gui_, true);
    nh.param("pub_visual", publish_visual_, false);
    nh.param("circle_diam", circle_diameter_, 0.122);

    nh.param("id_bits", id_bits, 6);
    nh.param("id_samples", id_samples, 360);
    nh.param("hamming_dist", hamming_dist, 1);
    nh.param("num_markers", num_markers, 10);

    intrinsic_mat_.resize(9);
    distortion_coeffs_.resize(5);
    image_ = new whycon::CRawImage(default_width, default_height, 3);
    whycon_.init(circle_diameter_, use_gui_, id_bits, id_samples, hamming_dist, num_markers, default_width, default_height);
    
    // subscribe to camera topics
    image_transport::ImageTransport it(nh);
    cam_info_sub_ = nh.subscribe("/camera/camera_info", 1, &CWhyconROSNode::cameraInfoCallback, this);
    img_sub_ = it.subscribe("/camera/image_raw", 1, &CWhyconROSNode::imageCallback, this);
    
    // advertise topics with markers description, RVIZ visualization and GUI visualization
    img_pub_ = it.advertise("processed_image", 1);
    markers_pub_ = nh.advertise<whycon_ros::MarkerArray>("markers", 1);
    visual_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualisation", 1);

    // advertise services
    drawing_srv_ = nh.advertiseService("set_drawing", &CWhyconROSNode::setDrawingCallback, this);
    coord_system_srv_ = nh.advertiseService("set_coords", &CWhyconROSNode::setCoordsCallback, this);
    calib_method_srv_ = nh.advertiseService("set_calib_method", &CWhyconROSNode::setCalibMethodCallback, this);
    calib_path_srv_ = nh.advertiseService("set_calib_path", &CWhyconROSNode::setCalibPathCallback, this);
    select_marker_srv_ = nh.advertiseService("select_marker", &CWhyconROSNode::selectMarkerCallback, this);

    // create dynamic reconfigure server
    dynamic_reconfigure::Server<whycon_ros::whyconConfig> dyn_srv;
    dynamic_reconfigure::Server<whycon_ros::whyconConfig>::CallbackType dyn_srv_cb;
    dyn_srv_cb = boost::bind(&CWhyconROSNode::reconfigureCallback, this, _1, _2);
    dyn_srv.setCallback(dyn_srv_cb);
}

CWhyconROSNode::~CWhyconROSNode()
{
    delete image_;
}

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "whycon_ros");

    whycon_ros::CWhyconROSNode whycon_ros_node;
    whycon_ros_node.start();

    return 0;
}