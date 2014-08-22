#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <time.h>

using namespace std;
using namespace cv;
class FUNCTIONS
{
    public:
        FUNCTIONS();
        virtual ~FUNCTIONS();
        void computeCost(IplImage *img,double &mean,double &variance);
        double GetFocusNormalizedVariance(Mat image);
        double GetLaplacian(Mat image);
        double GetFocusBrenner(const IplImage * image);
        double GetFocusStandardDeviationCorrelation(const IplImage * image);
        double GetFocusAutoCorrelation(const IplImage * image);
        double GetFocusWavelet1(const IplImage * image);
        double GetFocusWavelet2(const IplImage * image);
        double GetFocusWavelet3(const IplImage * image);

    protected:
    private:
};

#endif // FUNCTIONS_H
