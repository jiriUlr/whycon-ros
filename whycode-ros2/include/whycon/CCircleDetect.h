#ifndef WHYCON__CCIRCLEDETECT_H
#define WHYCON__CCIRCLEDETECT_H

#include <math.h>
#include <memory>
#include "whycon/CRawImage.h"
#include "whycon/CTransformation.h"
#include "whycon/CNecklace.h"
#include "whycon/SStructDefs.h"

/*TODO note #07*/
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define INNER 0
#define OUTER 1
#define MAX_PATTERNS 50

namespace whycon
{

typedef struct{
    float x, y;
    float m0, m1;
    float v0, v1;
} SSegSmall;


class CCircleDetect {
    public:
        //constructor, wi and he correspond to the image dimensions 
        CCircleDetect(int wi, int he, bool id, int bits, int samples, bool draw, CTransformation *trans, CNecklace *decoder);

        //deallocate the detector's structures
        ~CCircleDetect();

        // dynamic reconfigure of parameters
        void reconfigure(float ict,float fct,float art,float cdtr,float cdta, bool id, int minS);

        //main detection method, implements Algorithm 2 of [1] 
        SMarker findSegment(CRawImage* image, SSegment init);

        //local pattern search - implements Algorithm 1 of [1]
        bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);

        //calculate the pattern dimensions by means of eigenvalue decomposition, see 3.3 of [1]
        SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

        //cleanup the shared buffers - see 3.6 of [1] 
        void bufferCleanup(SSegment init);

        //change threshold if circle not detected, see 3.2 of [1]
        bool changeThreshold();

        //normalise the angle
        float normalizeAngle(float a);

        // adjust the dimensions of the image, when the image size changes
        void adjustDimensions(int wi, int he);

        bool ambiguityAndObtainCode(CRawImage *image);
        void ambiguityPlain();
        void setDraw(bool draw);

        int debug;                  // debug level
        bool draw_, lastTrackOK;     // flags to draw results - used for debugging
        bool localSearch;           // used when selecting the circle by mouse click
        bool identify;              // attempt to identify segments

    private:
        CTransformation *trans_;
        CNecklace *decoder_;
        
        int idBits;
        int idSamples;
        int hammingDist;
        bool track;
        int maxFailed;
        int numFailed;
        int threshold; 

        int minSize; 
        int lastThreshold; 
        int thresholdBias; 
        int maxThreshold; 

        int thresholdStep;
        float circularTolerance;
        float circularityTolerance;
        float ratioTolerance;
        float centerDistanceToleranceRatio;
        int centerDistanceToleranceAbs;
        bool enableCorrections;

        int ID;
        int step;
        SSegment inner;
        SSegment outer;
        float outerAreaRatio, innerAreaRatio, areasRatio;
        int queueStart, queueEnd, queueOldStart, numSegments;
        int width, height, len;
        int expand[4];
        unsigned char *ptr;
        int tima, timb, timc, timd, sizer, sizerAll;
        float diameterRatio;
        bool ownBuffer;
        // static int *buffer;
        // static int *queue;
        static std::unique_ptr<int> buffer;
        static std::unique_ptr<int> queue;
        // static int *mask;
        // static int maskNum;
        float idx[MAX_PATTERNS];
        float idy[MAX_PATTERNS];
        int numberIDs;

        SEllipseCenters ellipse_centers;
        STrackedObject tracked_object;
};

}

#endif

/* end of CCircleDetect.h */
