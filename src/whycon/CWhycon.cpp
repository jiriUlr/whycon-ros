#include <stdexcept>
#include <stdio.h>

#include "whycon/CWhycon.h"

namespace whycon
{

bool CWhycon::getDrawCoords()
{
    return draw_coords_;
}

bool CWhycon::getDrawSegments()
{
    return draw_segments_;
}

int CWhycon::getCoordinates()
{
    return trans_->getTransformType();
}

void CWhycon::setDrawing(bool draw_coords, bool draw_segments)
{
    draw_coords_ = draw_coords;
    draw_segments_ = draw_segments;

    for(int i = 0; i < num_markers_; i++)
    {
        detector_array_[i]->setDraw(draw_segments_);
    }
}

void CWhycon::setCoordinates(ETransformType trans_type)
{
    try
    {
        trans_->setTransformType(trans_type);
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void CWhycon::autocalibration()
{
    if(num_found_ < 4)
    {
        throw std::runtime_error("Autocalibration not possible. Cannot locate 4 markers in the scene.");
    }
    else
    {
        calib_step_ = 0;
        was_markers_ = num_markers_;

        last_transform_type_ = trans_->getTransformType();
        trans_->setTransformType(TRANSFORM_NONE);
        mancalibrate_ = false;
        autocalibrate_ = true;
    }
}

void CWhycon::manualcalibration()
{
    if(num_found_ < 4)
    {
        throw std::runtime_error("Manual calibration not possible. Cannot locate 4 markers in the scene.");
    }
    else
    {
        calib_num_ = 0;
        was_markers_ = num_markers_;
        num_markers_ = 1;

        last_transform_type_ = trans_->getTransformType();
        trans_->setTransformType(TRANSFORM_NONE);
        autocalibrate_ = false;
        mancalibrate_ = true;
    }
}

void CWhycon::loadCalibration(std::string& path)
{
    try
    {
        trans_->loadCalibration(path);
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void CWhycon::saveCalibration(std::string& path)
{
    try
    {
        trans_->saveCalibration(path);
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void CWhycon::selectMarker(float x, float y)
{
    // if(mancalibrate_)
    // {
        if (calib_num_ < 4 && calib_step_ > calibration_steps_)
        {
            calib_step_ = 0;
            trans_->setTransformType(TRANSFORM_NONE);
        }
        if (num_markers_ > 0)
        {
            current_marker_array_[0].seg.x = x; 
            current_marker_array_[0].seg.y = y;
            current_marker_array_[0].seg.valid = true;
            detector_array_[0]->localSearch = true;
        }
    // }
    // else
    // {

    // }
}

/*manual calibration can be initiated by pressing 'r'
  and then clicking circles at four positions (0,0)(field_length_,0)...*/
void CWhycon::manualCalib()
{
    if(current_marker_array_[0].valid)
    {
        STrackedObject o = current_marker_array_[0].obj;
        //moveOne = moveVal;

        //object found - add to a buffer
        if(calib_step_ < calibration_steps_) calib_tmp_[calib_step_++] = o;

        //does the buffer contain enough data to calculate the object position
        if(calib_step_ == calibration_steps_)
        {
            o.x = o.y = o.z = 0;

            for(int k = 0; k < calibration_steps_; k++)
            {
                o.x += calib_tmp_[k].x;
                o.y += calib_tmp_[k].y;
                o.z += calib_tmp_[k].z;
            }

            o.x = o.x/calibration_steps_;
            o.y = o.y/calibration_steps_;
            o.z = o.z/calibration_steps_;

            if(calib_num_ < 4) calib_[calib_num_++] = o;

            //was it the last object needed to establish the transform ?
            if(calib_num_ == 4)
            {
                //calculate and save transforms
                trans_->calibrate2D(calib_, field_length_, field_width_);
                trans_->calibrate3D(calib_, field_length_, field_width_);
                calib_num_++;
                num_markers_ = was_markers_;
                trans_->setTransformType(last_transform_type_);
                detector_array_[0]->localSearch = false;
                mancalibrate_ = false;
                printf("manualCalib done\n");
            }

            calib_step_++;
        }
    }
}

/*finds four outermost circles and uses them to set-up the coordinate system
  [0,0] is left-top, [0,field_length_] next in clockwise direction*/
void CWhycon::autoCalib()
{
    int ok_last_tracks = 0;
    for(int i = 0; i < num_markers_; i++)
    {
        if(detector_array_[i]->lastTrackOK)
        {
            ++ok_last_tracks;
        }
    }
    if(ok_last_tracks < 4)
    {
        return;
    }

    int index[] = {0, 0, 0, 0};
    int max_eval = 0;
    int eval = 0;
    int sX[] = {-1, +1, -1, +1};
    int sY[] = {+1, +1, -1, -1};
    for(int b = 0; b < 4; b++)
    {
        max_eval = -10000000;
        for(int i = 0; i < num_markers_; i++)
        {
            eval =  sX[b] * current_marker_array_[i].seg.x + sY[b] * current_marker_array_[i].seg.y;
            if(eval > max_eval)
            {
                max_eval = eval;
                index[b] = i;
            }
        }
    }
    printf("INDEX: %i %i %i %i\n", index[0], index[1], index[2], index[3]);

    for(int i = 0; i < 4; i++)
    {
        if (calib_step_ <= auto_calibration_pre_steps_)
        {
            calib_[i].x = calib_[i].y = calib_[i].z = 0;
        }
        calib_[i].x += current_marker_array_[index[i]].obj.x;
        calib_[i].y += current_marker_array_[index[i]].obj.y;
        calib_[i].z += current_marker_array_[index[i]].obj.z;
    }
    calib_step_++;
    if (calib_step_ == auto_calibration_steps_)
    {
        for(int i = 0; i < 4; i++)
        {
            calib_[i].x = calib_[i].x / (auto_calibration_steps_ - auto_calibration_pre_steps_);
            calib_[i].y = calib_[i].y / (auto_calibration_steps_ - auto_calibration_pre_steps_);
            calib_[i].z = calib_[i].z / (auto_calibration_steps_ - auto_calibration_pre_steps_);
        }
        trans_->calibrate2D(calib_, field_length_, field_width_);
        trans_->calibrate3D(calib_, field_length_, field_width_);
        calib_num_++;
        num_markers_ = was_markers_;
        trans_->setTransformType(last_transform_type_);
        autocalibrate_ = false;
        printf("autoCalib done\n");
    }
}

void CWhycon::processImage(CRawImage *image, std::vector<SMarker> &whycon_detections)
{
    if(image_height_ != image->height_ || image_width_ != image->width_)
    {
        image_height_ = image->height_;
        image_width_ = image->width_;
    }

    // setup timers to assess system performance
    CTimer timer;
    num_found_ = num_static_ = 0;
    timer.reset();
    timer.start();

    // track the markers found in the last attempt
    for(int i = 0; i < num_markers_; i++)
    {
        if(current_marker_array_[i].valid)
        {
            last_marker_array_[i] = current_marker_array_[i];
            current_marker_array_[i] = detector_array_[i]->findSegment(image, last_marker_array_[i].seg);
        }
    }

    // search for untracked (not detected in the last frame) markers
    for(int i = 0; i < num_markers_; i++)
    {
        if(current_marker_array_[i].valid == false)
        {
            last_marker_array_[i].valid = false;
            last_marker_array_[i].seg.valid = false;
            current_marker_array_[i] = detector_array_[i]->findSegment(image, last_marker_array_[i].seg);
        }
        if(current_marker_array_[i].seg.valid == false) break;  //does not make sense to search for more patterns if the last one was not found
    }

    for(int i = 0; i < num_markers_; i++)
    {
        if(current_marker_array_[i].valid)
        {
            if(identify_ && current_marker_array_[i].seg.ID <= -1)
            {
                current_marker_array_[i].seg.angle = last_marker_array_[i].seg.angle;
                current_marker_array_[i].seg.ID = last_marker_array_[i].seg.ID;
            }
            num_found_++;
            if(current_marker_array_[i].seg.x == last_marker_array_[i].seg.x) num_static_++;
        }
    }

    eval_time_ = timer.getTime();

    for(int i = 0; i < num_markers_; i++)
    {
        if(current_marker_array_[i].valid)
        {
            whycon_detections.push_back(current_marker_array_[i]);
        }
    }

    // draw stuff on the GUI
    if(use_gui_)
    {
        image->drawTimeStats(eval_time_, num_found_);

        if(mancalibrate_)
        {
            image->drawGuideCalibration(calib_num_, field_length_, field_width_);
        }

        for(int i = 0; i < num_markers_ && draw_coords_; i++)
        {
            if(current_marker_array_[i].valid)
            {
                image->drawStats(current_marker_array_[i], trans_->getTransformType() == TRANSFORM_2D);
            }
        }
    }

    // establishing the coordinate system by manual or autocalibration
    if (autocalibrate_ && num_found_ > 3) //num_found_ == num_markers_)
    {
        autoCalib();
    }
    if (calib_num_ < 4)
    {
        manualCalib();
    }
}

// cleaning up
CWhycon::~CWhycon()
{
    delete trans_;
    delete decoder_;
    for(int i = 0; i < num_markers_; i++) delete detector_array_[i];
}

CWhycon::CWhycon() :
    draw_coords_(true)
    , draw_segments_(false)
    , calibrated_coords_(false)
{
}

void CWhycon::updateConfiguration(bool id, float diam, int markers, int size, double fl, double fw, double ict, double fct, double art, double cdtr, double cdta)
{
    field_length_ = fl;
    field_width_ = fw;
    identify_ = id;

    trans_->setCircleDiameter(diam);

    if(num_markers_ != markers)
    {
        current_marker_array_.resize(markers);
        last_marker_array_.resize(markers);

        if(num_markers_ < markers)
        {
            for(int i = num_markers_; i < markers; i++)
                detector_array_.push_back(new CCircleDetect(image_width_, image_height_, identify_, id_bits_, id_samples_, draw_segments_, trans_, decoder_));
        }
        else
        {
            for(int i = num_markers_; i > markers; i--) delete detector_array_[i];
            detector_array_.resize(markers);
        }
    }

    num_markers_ = markers;

    for(int i = 0; i < num_markers_; i++) detector_array_[i]->reconfigure(ict, fct, art, cdtr, cdta, id, size);
}

void CWhycon::updateCameraInfo(std::vector<float> &intrinsic_mat, std::vector<float> &distortion_coeffs)
{
    trans_->updateCameraParams(intrinsic_mat, distortion_coeffs);
}

void CWhycon::init(float circle_diam, bool use_gui, int id_b, int id_s, int ham_dist, int markers, int img_w, int img_h)
{
    circle_diameter_ = circle_diam;
    use_gui_ = use_gui;
    id_bits_ = id_b;
    id_samples_ = id_s;
    hamming_dist_ = ham_dist;
    num_markers_ = markers;
    image_width_ = img_w;
    image_height_ = img_h;

    calib_tmp_.resize(calibration_steps_);
    current_marker_array_.resize(num_markers_);
    last_marker_array_.resize(num_markers_);

    trans_ = new CTransformation(circle_diameter_);
    decoder_ = new CNecklace(id_bits_, id_samples_, hamming_dist_);

    // initialize the circle detectors - each circle has its own detector instance
    detector_array_.resize(num_markers_);
    for(int i = 0; i < num_markers_; i++)
        detector_array_[i] = new CCircleDetect(image_width_, image_height_, identify_, id_bits_, id_samples_, draw_segments_, trans_, decoder_);

}

}
