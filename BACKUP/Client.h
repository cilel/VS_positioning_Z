#ifndef CLIENT_H
#define CLIENT_H

#include "CLIENTFUNCS.h"
#include <visp/vpConfig.h>
#include <visp/vpPlot.h>

#include "SyncSerialComm.h"
#include "myLog.h"
#include "functions.h"
#include "controlserver.h"

#include <time.h>
#include <fstream>
#include <iomanip>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/** VISP include libraries   */
#include <visp/vpImage.h>
#include<visp/vpImageIo.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImageConvert.h>
//#include <visp/vpFeatureLuminance.h>


#include "tritor.h"
//#define Z 1
class Client
{
public:
    CLIENTFUNCS client;
    //CSyncSerialComm *myserial;
    Client();
    Client(char *ip);
    void process_array(IplImage *img,bool check,int &maxval, int &minval,double &std);
    virtual ~Client();
    void joinImages(Mat in,Mat &out, int firstrow, int firstcol);
    Mat Compare(vpMatrix i1,vpMatrix i2);
    vpMatrix MatToVpMatrix(Mat in);
    Mat VpMatrixToMat(vpMatrix in,bool ok);
    void Stuck(Mat in,Mat &out, int firstrow, int firstcol);
    void phase_correlation(Mat img1,Mat img2,Mat &res);
    void DeleteAllFiles(char* folderPath);
    Mat getMask(Mat);
    void Sharpen(const Mat& myImage,Mat& Result);
//    void onMouse( int event, int x, int y, int, void* );
    void drawGripper(Mat in, Mat out);
    void drawGripperMouse(Mat in, Mat out);
    vpColVector semPosCont(Mat curImage, Mat desImage, bool init);
    vpColVector new_v;

    //------------ LE CUI VISP VARIABLES --------------------
    // Variables for interaction, Hessian, combination, vectors, matrices etc.

private:
    clock_t t1,t2;
    float seconds;
    char imfile[50];
    IplImage *img;
    IplImage *grayim,*filtim,*subim;


    //int counter;

};

#endif // CLIENT_H
