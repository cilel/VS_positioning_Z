#ifndef CLIENT_H
#define CLIENT_H

#include "CLIENTFUNCS.h"

/** Project local libraries */
#include "SyncSerialComm.h"
#include "myLog.h"



/**  C/C++_Std Include libraries  */
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <iomanip>

/** OpenCV Include libraries */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/** VISP include libraries   */

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpPlot.h>

#include <visp/vpNoise.h>
#include <visp/vpImageConvert.h>
#include <visp/vpExponentialMap.h>

using namespace cv;

class Client
{
public:
    CLIENTFUNCS client;

    Client();           //! Optional constructor - not used
    Client(char *ip);   //! Main work constructor
    virtual ~Client();  //! Virtual destructor


//    vpColVector new_v;
protected:
    vpColVector semPosCont(Mat curImage, Mat desImage, bool init);
    void joinImages(Mat in,Mat &out, int firstrow, int firstcol);
    Mat Compare(vpMatrix i1,vpMatrix i2);
    vpMatrix MatToVpMatrix(Mat in);
    Mat VpMatrixToMat(vpMatrix in,bool ok);
    void Stuck(Mat in,Mat &out, int firstrow, int firstcol);
    void DeleteAllFiles(char* folderPath);
    void Stat(vpColVector src,double &mean,double &var,double &norm);

    //------------ LE CUI VISP VARIABLES --------------------
    // Variables for interaction, Hessian, combination, vectors, matrices etc.
private:
    clock_t t1,t2;
    float seconds;
    char imfile[50];
    IplImage *img;
    IplImage *grayim,*filtim,*subim;

};

#endif // CLIENT_H
