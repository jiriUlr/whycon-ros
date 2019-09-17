/*
 * File name: CTransformation.h
 * Date:      2014
 * Author:   Tom Krajnik, Matias Nitsche
 * Description: Transforms detected ellipse position and dimensions to arbitrary 3D or 2D coordinate frame. The method is described in Chapter 4 of the article [1]. 
 * Licence: if you use this class for your research, please cite [1]. 
 * References: [1] Krajnik, Nitsche et al.: A practical multirobot localization system. Journal of Intelligent and Robotic Systems, 2014.
 */

#ifndef WHYCON_CTRANSFORMATION_H
#define WHYCON_CTRANSFORMATION_H

#include <cstdio>
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "whycon/SStructDefs.h"

namespace whycon
{

class CTransformation
{
    public:
        /*init: width & height of the image, diameter of the pattern, path to saved calibration file*/
        CTransformation(float circle_diam);
        ~CTransformation();

        // update of intrinsic and distortion camera params
        void updateCameraParams(cv::Mat intri, cv::Mat dist);
        void updateCameraParams(double *intri, double* dist);

        /* calculate marker 3D or 2D coordinates in user-defined coordinate system from the segment description provided by the CCircleDetector class, see 4.1-4.4 of [1] */
        SEllipseCenters calcSolutions(SSegment segment);

        /* calculate the pattern 3D position from its ellipse characteristic equation, see 4.3 of [1] */
        SEllipseCenters calcEigen(float *data);

        void transformAndAngles(STrackedObject &obj);

        /*supporting methods*/
        void saveCalibration(const char *str);
        void loadCalibration(const char *str);
        void saveCalibration(const std::string &str);
        void loadCalibration(const std::string &str);
        
        void calcQuaternion(STrackedObject &obj);
        void calcEulerFromQuat(STrackedObject &obj);


        /* DONE */
        void setTransformType(const ETransformType trans_type);
        ETransformType getTransformType();

        /* update circle diameter */
        void setCircleDiameter(const float circle_diam);

        /*image to canonical coordinates (unbarrel + focal center and length) */
        void transformXY(float &x,float &y);

        /* get back image coords from canonical coords */
        void reTransformXY(float &x, float &y, float &z);

        /* establish the user-defined coordinate system from four calibration patterns - see 4.4 of [1] */
        void calibrate2D(const STrackedObject *in, const float g_dim_x, const float g_dim_y, const float robot_radius = 0.0, const float robot_height = 0.0, const float camera_height = 1.0);
        void calibrate3D(const STrackedObject *o, const float g_dim_x, const float g_dim_y);
        S3DTransform calibrate3D(const STrackedObject &o0, const STrackedObject &o1, const STrackedObject &o2, const float g_dim_x, const float g_dim_y);

    private:
        
        void transform2D(STrackedObject &o);
        void transform3D(STrackedObject &o, const int num = 4);

        bool calibrated_;
        float grid_dim_x_;
        float grid_dim_y_;
        float circle_diameter_;

        float hom_[9];
        float trf4D_[16];
        S3DTransform D3transform_[4];

        cv::Mat intrinsic_mat_;
        cv::Mat distortion_coeffs_;
        ETransformType transform_type_;
};

}

#endif  /* end of CTransformation.h */