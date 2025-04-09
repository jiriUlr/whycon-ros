#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "whycon/CRawImage.h"

namespace whycon
{

CRawImage::CRawImage(int width, int height, int bpp) :
    width_(width),
    height_(height),
    bpp_(bpp),
    size_(width * height * bpp)
{
    data_ = (unsigned char*) std::malloc(sizeof(unsigned char) * size_);
}

CRawImage::~CRawImage()
{
    std::free(data_);
}

void CRawImage::updateImage(unsigned char* new_data, int width, int height, int bpp)
{
    if(width_ != width || height_ != height || bpp_ != bpp)
    {
        std::printf("Readjusting image format from %ix%i %ibpp to %ix%i %ibpp.\n", width_, height_, bpp_, width, height, bpp);
        width_ = width;
        height_ = height;
        bpp_ = bpp;
        size_ = width * height * bpp;
        data_ = (unsigned char*) std::realloc(data_, sizeof(unsigned char) * size_);
    }
    std::memcpy(data_, new_data, size_);
}

void CRawImage::drawTimeStats(int eval_time, int num_markers)
{
    cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
    char text[100];
    std::sprintf(text, "Found %i markers in %.3f ms", num_markers, eval_time / 1000.0);

    int font_face = cv::FONT_HERSHEY_SIMPLEX;//DUPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    cv::rectangle(img, cv::Point(0, text_size.height + 3), cv::Point(text_size.width, 0), cv::Scalar(0, 0, 0), cv::FILLED);

    cv::putText(img, text, cv::Point(0, text_size.height + 1), font_face, font_scale, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
}

void CRawImage::drawStats(SMarker &marker, bool trans_2D)
{
    cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
    char text[100];

    int font_face = cv::FONT_HERSHEY_SIMPLEX;//DUPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    int baseline = 0;
    cv::Scalar color(255, 0, 0);

    if(trans_2D)
        std::sprintf(text, "%03.0f %03.0f", 1000 * marker.obj.x, 1000 * marker.obj.y);
    else
        std::sprintf(text, "%.3f %.3f %.3f", marker.obj.x, marker.obj.y, marker.obj.z);

    cv::Size text_size0 = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point text_pos0(marker.seg.minx - 30, marker.seg.maxy + text_size0.height);
    cv::putText(img, text, text_pos0, font_face, font_scale, color, thickness, cv::LINE_AA);

    if(trans_2D)
    {
        // std::sprintf(text, "%02i %03i", marker.seg.ID, (int)(marker.obj.yaw / M_PI * 180));
    }
    else
    {
        std::sprintf(text, "%02i %.3f %.3f %.3f", marker.seg.ID, marker.obj.roll, marker.obj.pitch, marker.obj.yaw);
        cv::Size text_size1 = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        cv::Point text_pos1(marker.seg.minx - 30, marker.seg.maxy + 2 * text_size1.height + 5);
        cv::putText(img, text, text_pos1, font_face, font_scale, color, thickness, cv::LINE_AA);
    }
}

void CRawImage::drawGuideCalibration(int calib_num, float dim_x, float dim_y)
{
    cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
    char text[100];

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1;
    int thickness = 2;
    int baseline = 0;
    cv::Scalar text_color(0, 255, 0);
    cv::Scalar rect_color(0, 0, 0);

    switch(calib_num)
    {
        default:
        case 0:
            std::sprintf(text, "Click at [0.000, 0.000].");
            break;
        case 1:
            std::sprintf(text, "Click at [%.3f, 0.000].", dim_x);
            break;
        case 2:
            std::sprintf(text, "Click at [0.000, %.3f].", dim_y);
            break;
        case 3:
            std::sprintf(text, "Click at [%.3f, %.3f].", dim_x, dim_y);
            break;
    }

    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point text_pos(width_ / 2 - 130, height_ / 2);
    cv::Point text_pos_opp(text_pos.x + text_size.width, text_pos.y - text_size.height);

    cv::rectangle(img, text_pos, text_pos_opp, rect_color, cv::FILLED);
    cv::putText(img, text, text_pos, font_face, font_scale, text_color, thickness);
}

void CRawImage::plotCenter()
{
    int centerWidth = 20;
    unsigned char color[] = {255, 150, 150};
    for (int i = -centerWidth; i < centerWidth; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            data_[(width_ * (height_ / 2 + i) + width_ / 2 - centerWidth) * 3 + j] = color[j];
            data_[(width_ * (height_ / 2 + i) + width_ / 2 + centerWidth) * 3 + j] = color[j];
            data_[(width_ * (height_ / 2 - centerWidth) + width_ / 2 + i) * 3 + j] = color[j];
            data_[(width_ * (height_ / 2 + centerWidth) + width_ / 2 + i) * 3 + j] = color[j];
        }
    }
}

void CRawImage::plotLine(int x, int y)
{
    int base;
    if (y < 0 || y > height_ - 1)
        y = height_ / 2;

    if (x < 0 || x > width_ - 1)
        x = width_ / 2;

    for(int i=0; i < width_;i++)
    {
        if (i == width_ / 2)
            i++;
        base = (width_ * y + i) * 3;
        data_[base+0] = 255;
        data_[base+1] = 0;
        data_[base+2] = 255;
    }

    for(int j=0;j<height_;j++)
    {
        const int bidx = ((width_ * j) + x) * 3;
        if (j == height_ / 2)
            j++;
        data_[bidx+0] = 255;
        data_[bidx+1] = 255;
        data_[bidx+2] = 0;
    }
}


/** pocita jas obrazku:
  *  upperHalf == true, pocita se jen z horni poloviny obrazku
  *  upperHalf == false, pocita jen ze spodni poloviny obrazku
  */
double CRawImage::getOverallBrightness(bool upperHalf)
{
    int step = 5;
    int sum, num, satMax, satMin, pos;
    sum = num = satMax = satMin = 0;
    int limit = 0;

    if (upperHalf)
        limit = 0;
    else
        limit = height_ / 2;

    for (int i = limit; i < height_ / 2 + limit; i += step)
    {
        for (int j = 0; j < width_; j += step)
        {
            pos = (i * width_ + j) * bpp_;
            if (data_[pos] >= 250 && data_[pos + 1] >= 250 && data_[pos + 2] >= 250)
                satMax++;
            if (data_[pos] <= 25 && data_[pos + 1] <= 25 && data_[pos + 2] <= 25)
                satMin++;
            sum += data_[pos] + data_[pos + 1] + data_[pos + 2];
            num++;
        }
    }
    return (sum / num / bpp_) + (satMax - satMin) * 100.0 / num;
}

}
