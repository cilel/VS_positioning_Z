#include "Client.h"
#include <time.h>
#include <visp/vpMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpImageConvert.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpHomogeneousMatrix.h>

#include <math.h>
#include "fftw3.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

/*-----*/

#include <stdlib.h>

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

#include "npFeatureLuminance.h"

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpPlot.h>

#include <visp/vpNoise.h>
#include <visp/vpImageConvert.h>
#include <visp/vpExponentialMap.h>

using namespace cv;

/* ------------------- Define projection type -------------------*/

typedef enum
{
    perspective,
    parallel, //donot control translation Z
    parallelZ // control translation Z
} projectionModel;

projectionModel pjModel;

/**=========================================================================
**                          GLOBAL VARIABLES
**=========================================================================*/
controlserver *control = new controlserver();   //! Control Object - OPTIONAL
tritor *stage = new tritor();                   //! Stage object
Point P;
vector<Point> G;
int counter =0,gCount=0;
void onMouse( int event, int x, int y, int, void* );
vpColVector vel;

Client::Client()
{
    //Optional constructor - not used
}
/**=========================================================================
**                  Main Constructor for Visual servoing
**=========================================================================*/
Client::Client(char *ip)
{

    /**================ Delete previous files and clear directories ===============*/
    DeleteAllFiles("IMAGES");
    DeleteAllFiles("RESULTS\\CURIM");
    DeleteAllFiles("RESULTS\\DIFIM");
    DeleteAllFiles("RESULTS\\TOTIMS");

    /**============ Results files ===========*/
    ofstream x_voltfile("RESULTS\\result_x_voltage.txt");
    ofstream y_voltfile("RESULTS\\result_y_voltage.txt");
    ofstream stats     ("RESULTS\\statvals.txt");
    ofstream errorvals( "RESULTS\\error.txt");
    ofstream motion(    "RESULTS\\motion.txt");

    /**======== Communication variables ======== */
    char rec[32]   = "",imfile[500]="";
    char fname[32] = "";         // Received filename from server
    int numbits;                 // Sent or Received data from server
    char write1_buffer[256] =""; // Buffer for serial communication

    /**============ VISP Variables ==============*/
    vpMatrix Lxy;       // Interaction matrix
    vpMatrix Ly;        // Interaction matrix
    vpMatrix Hsd;       // Hessian matrix
    vpMatrix H ;        // Hessien matrix for Levenberg-Maquardt
    vpMatrix dH;
    vpMatrix D;
    vpMatrix C;         // Combination matrix
    vpColVector V;
    vpImage<unsigned char> Id; // Desired image
    vpImage<unsigned char> I;  // Current image
    vpColVector ex;
    vpColVector ey;
    vpColVector e;

    /**============ Other computational variables ==============*/
    int iter =0;
    char saveimg[50]="";
    float mu=1;
    double temp_disp_x=0.0,temp_disp_y=0.0;
    double x_disp = 0.0,y_disp = 0.0,x_volt = 0.0,y_volt = 0.0;
    double TX = 0.0,TY = 0.0,TXSP=0.0, TYSP=0.0, DX=0.0, DY=0.0;
    double  minVal, maxVal;
    double var,norm,mean;
    double lambda = 2;
    Mat resIm;
    Mat resIm1;
    Point minLoc, maxLoc;

    vector<double>disp_x_act;   // X-voltage
    vector<double>disp_y_act;   // Y-voltage
    vector<double>vel_x;        // X-veleocity
    vector<double>vel_y;        // Y-velocity

    /**============ Build reference feature s* = 0 ==============*/
    vpRotationMatrix R;             //Rotation matrix
    vpTranslationVector T(0,0,0);   //Translation vector
    R[0][0] = 1.;
    R[0][1] = 0.;
    R[0][2] = 0.;
    R[1][0] = 0.;
    R[1][1] = 1.;
    R[1][2] = 0.;
    R[2][0] = 0.;
    R[2][1] = 0.;
    R[2][2] = 1.;

    vpHomogeneousMatrix cMcd;
    vpFeatureTranslation Sd(vpFeatureTranslation::cMcd);
    Sd.buildFrom(cMcd); // Reference feature


    /**=============================================================
    ***        INITIAL CONNECTION TO NV 40/3 for HOME POSITION
    ***============================================================*/
    memset(write1_buffer,0,50);

    CSyncSerialComm *myserial = new CSyncSerialComm("COM1"); // Serial class object
    myserial->Open();                   // Open port
    myserial->ConfigPort(19200,1);      // Configure the port with baud rate and timeout.

    //---- Set all channels of NV 40/3 in remote mode ----
    strcpy(write1_buffer,"setk,0,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,1,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,2,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));

    //---- Set all channels of NV 40/3 to zero displacement ----
    strcpy(write1_buffer,"setall,-20,-20,-20\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));

    //---- Set all channels of NV 40/3 to manual mode ----
    strcpy(write1_buffer,"setk,0,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,1,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,2,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    myserial->Close();

    /**=============================================================
    ***                 CONNECT TO THE SERVER
    ***============================================================*/
    client.connectServer(ip,27015);      // Connect To Server

    /**--------- Get Desire Image from the server -------*/
    cout<<"Select the desired position from 'APROS3' application and press 'ENTER'"<<endl;
    cout<<"NOTE: Close the application after selecting the desired position"<<endl;

    /**=============================================================
    ***                 ACQUIRE DESIRED IMAGE
    ***============================================================*/
    Mat refIm_temp;
    while(cvWaitKey(1)==-1)
    {
        numbits = client.sendData("READY2");
        if(numbits==-1)
        {
            cout<<"Cannot transfer data..."<<endl;
            exit(1);
        }
        client.recvData(rec,32);
        client.fileReceive(fname);

        client.sendData("RECV_SUCCESS");
        client.recvData(rec,32);


        sprintf(imfile,"IMAGES\\%s",fname);
        refIm_temp = imread(imfile,CV_LOAD_IMAGE_GRAYSCALE);
        drawGripper(refIm_temp,refIm_temp);
        imshow("DESIRED IMAGE",refIm_temp);
    }

    destroyAllWindows();
    Mat refIm = cvLoadImage(imfile,CV_LOAD_IMAGE_GRAYSCALE);
    imwrite("RESULTS\\REFIM.png",refIm);
    waitKey(-1);


    /**-------- Combine all images For display --------*/
    int r = refIm.rows;
    int c = refIm.cols;
    Mat totIm;
    totIm.create(2*r+20,2*c+20,CV_8U);   // BIG IMAGE FOR DISPLAY


    /**=============================================================
    ***     Open NV40/3 again in remote mode for visual servoing
    ***============================================================*/
    myserial->Open();
    myserial->ConfigPort(19200,1);
    strcpy(write1_buffer,"setk,0,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,1,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,2,1\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));

    //---- Set all channels of NV 40/3 to zero displacement ----
    strcpy(write1_buffer,"setall,-20,-20,-20\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));

    Sleep(300); // Wait for home position


    /**=============================================================
    ***     MAIN VISUAL SERVOING LOOP
    ***============================================================*/
    while(1)
    {
        /**============ Current image ===========*/
        numbits = client.sendData("READY2");
        if(numbits==-1)
        {
            cout<<"Cannot transfer data..."<<endl;
            exit(1);
        }
        client.recvData(rec,32);
        client.fileReceive(fname);

        client.sendData("RECV_SUCCESS");
        client.recvData(rec,32);

        sprintf(imfile,"IMAGES\\%s",fname);
        Mat curIm = cvLoadImage(imfile,CV_LOAD_IMAGE_GRAYSCALE);


        /**==========================================
        **              MAIN CONTROL FUNCTION
        ===========================================*/

        //!------- HERE IS THE FUNCTION OF "LE CUI"
        if(iter=0)
            semPosCont(curIm,refIm,true);
        else
            semPosCont(curIm,refIm,false);

        vel_x.push_back(new_v[1]);
        vel_y.push_back(new_v[0]);

        /**============ Compute displacements and voltages ============*/
        if(iter>1)
        {
            /**----------TX -------------*/

            x_disp = ((vel_x[0] + vel_x[iter-1])/2)*0.1;
            disp_x_act.push_back(x_disp);
            x_voltfile<<V[0]<<setw(15)<<x_disp<<setw(15);

            if(V[0]>=0)
            {
                x_disp = temp_disp_x + x_disp;
                x_volt = stage->move_inc_x(abs(x_disp));
            }
            if(V[0]<0)
            {
                x_disp = temp_disp_x - x_disp;
                x_volt = stage->move_dec_x(abs(x_disp));
            }
            if(V[0]<0.4&&V[0]>-0.4)
            {
                x_disp = temp_disp_x;
                x_volt = stage->move_inc_x(abs(x_disp));
            }
            if(x_volt>110||x_volt<-20)
            {
                cout<<"Error: Voltage overflow for - x axis"<<endl;
            }

            x_voltfile<<x_disp<<setw(15)<<x_volt<<"  ;"<<endl;;
            temp_disp_x = x_disp;


            /**----------TY -------------*/
            y_disp = ((vel_y[0] + vel_y[iter-1])/2)*0.1;
            disp_y_act.push_back(y_disp);
            y_voltfile<<V[1]<<setw(15)<<y_disp<<setw(15);

            if(V[1]>=0)
            {
                y_disp = temp_disp_y + y_disp;
                y_volt = stage->move_inc_y(abs(y_disp));
            }
            if(V[1]<0)
            {
                y_disp = temp_disp_y - y_disp;
                y_volt = stage->move_dec_y(abs(y_disp));
            }
            if(V[1]<0.3&&V[1]>-0.3)
            {
                y_disp = temp_disp_y;
                y_volt = stage->move_inc_y(abs(y_disp));
            }
            if(y_volt>110||y_volt<-20)
            {
                cout<<"Error: Voltage overflow for - y axis"<<endl;
            }

            y_voltfile<<y_disp<<setw(15)<<y_volt<<"  ;"<<endl;;
            temp_disp_y = y_disp;

        }

        //------- Write voltage to serial port ----
        if(x_volt<110)
        {
            sprintf(write1_buffer,"set,0,%f\r",x_volt);
            myserial->Write(write1_buffer,sizeof(write1_buffer));
        }

        if(y_volt<110)
        {
            sprintf(write1_buffer,"set,1,%f\r",y_volt);
            myserial->Write(write1_buffer,sizeof(write1_buffer));
        }



        vpMatrix::sub2Matrices(MatToVpMatrix(refIm),MatToVpMatrix(curIm),D);
        Mat diff = VpMatrixToMat(D,0);
        Mat cmp = Compare(MatToVpMatrix(refIm),MatToVpMatrix(curIm));

        sprintf(saveimg,"RESULTS\\TOTIMS\\TOTAL%d.png",iter);
        imwrite(saveimg,totIm);
        sprintf(saveimg,"RESULTS\\CURIM\\CURR%d.png",iter);
        imwrite(saveimg,curIm);
        sprintf(saveimg,"RESULTS\\DIFIM\\DIFF%d.png",iter);
        imwrite(saveimg,diff);

        //drawGripper(refIm1,refIm1);
        drawGripper(curIm,curIm);

        Stuck(refIm,totIm,0,0);
        Stuck(curIm,totIm,r+10,0);
        Stuck(diff,totIm,0,c+10);
        Stuck(cmp,totIm,c+10,c+10);
        imshow("TOTAL",totIm);
        //imshow("TEMP",resIm);

        waitKey(33);
        iter++;
    }

    /***---- Set all channels of NV 40/3 to Zero and manual mode ----*/
    strcpy(write1_buffer,"set,0,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"set,1,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"set,2,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));

    strcpy(write1_buffer,"setk,0,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,1,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    strcpy(write1_buffer,"setk,2,0\r");
    myserial->Write(write1_buffer,sizeof(write1_buffer));
    myserial->Close();

    client.sendData("END_CONNECTION");
    client.recvData(rec,32);
    printf("Connection ended......\n");
    x_voltfile.close();
    y_voltfile.close();

}

Client::~Client()
{
    char rec[32] = "";
    client.sendData("END_CONNECTION");
    client.recvData(rec,32);
    printf("Connection ended......\n");

}

/**=========================================================================
** By: Le CUI
** @function semPosCont
** input: current image, desired image and initialisation check (bool)
** Output: Column vector with velocities for DOF
**=========================================================================*/

vpColVector Client::semPosCont(Mat curImage, Mat desImage, bool init)
{
    // 1. Conver to Visp
    vpImage<unsigned char> I, Id;
    vpImageConvert::convert(curImage, I);
    vpImageConvert::convert(desImage, Id);

    pjModel = parallel;

    // 2. define variables - use the global variable vpColvector vel
    //vpHomogeneousMatrix cMo,cMod,wMe,eMo,cMw,wMcR,wMc,wMo,Tr,cMe; //Robot-camera reference
    //vpCameraParameters cam; // camera parameters

    double Zz ;  // Z position
    Zz= 0.2;
    double sigma; // sigma in Gauss PDF

    npFeatureLuminance sI; // current feature
    npFeatureLuminance sId; // desired feature

    vpMatrix Lsd;   // matrice d'interaction a la position desiree
    vpMatrix Hsd;  // hessien a la position desiree
    vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
    vpColVector error ; // Erreur I-I*, photometric information

    vpMatrix Lgsd; // interaction matrix (using image gradient) of the desired position
    vpMatrix Hgsd; // hessien of the desired position (using image gradient)
    vpMatrix Hg;  // Hessien for  levenberg-Marquartd (using image gradient)
    vpColVector sg_error; // error sg-sg*, image gradient

    vpColVector e ;// velocity to be multiply by lamda
    vpColVector v ; // camera velocity send to the robot
    vpColVector eg; // velocity of z axis
    vpColVector vg ; // camera velocity of z axis
    double vgd;// camera velocity of z axis, double

    vpVelocityTwistMatrix cVw; //spatial velocity transform matrix

    vpMatrix Js;// visual feature Jacobian
    vpMatrix Jn;// robot Jacobian
    vpMatrix diagHsd;// diag(Hsd)

    //For parallelZ, image gradient
    vpMatrix Jgs;// visual feature Jacobian
    vpMatrix Jgn;// robot Jacobian
    vpMatrix diagHgsd;// diag(Hsd)

    double lambda=100;
    double mu;

    if(init)
    {

        if(pjModel == parallel)
            sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallel) ;
        else if(pjModel == parallelZ)
            sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallelZ,npFeatureLuminance::ImageGradient) ;
        else
            sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
        //sId.setCameraParameters(cam) ;
        sId.set_sigma(10);//just a value, not useful
        sId.buildFrom(Id) ;


        if(pjModel == parallel)
            sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallel) ;
        else if(pjModel == parallelZ)
            sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallelZ,npFeatureLuminance::ImageGradient) ;
        else
            sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
        //sI.setCameraParameters(cam) ;
        sI.set_sigma(sigma);
        sI.buildFrom(I) ;

        sId.interaction(Lsd) ;
        // For Z
        Lgsd = sId.get_Lg();

        if(pjModel==parallel )
        {
            // Compute the Hessian H = L^TL
            Hsd = Lsd.AtA() ;

            //cout << "Hsd=\n" << Hsd <<endl;
            // Compute the Hessian diagonal for the Levenberg-Marquartd
            // optimization process
            unsigned int n = 5 ;
            diagHsd.resize(n,n) ;
            diagHsd.eye(n);
            for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];
        }
        else if (pjModel==parallelZ)
        {

            sI.interaction(Lsd) ; // here use Ls instead of Lsd to compute Js
            Lgsd = sId.get_Lg();

            // Compute the Hessian H = L^TL
            Hsd = Lsd.AtA() ;
            Hgsd = Lgsd.AtA();
            //cout << "Hgsd=\n" << Hgsd <<endl;

            // Compute the Hessian diagonal for the Levenberg-Marquartd
            // optimization process
            unsigned int n = 5 ;
            diagHsd.resize(n,n) ;
            diagHsd.eye(n);
            for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];

            diagHgsd.resize(1,1);
            diagHsd[0][0] = Hsd[0][0];

        }
        else //perspective
        {

            // Compute the Hessian H = L^TL
            Hsd = Lsd.AtA() ;
            // Compute the Hessian diagonal for the Levenberg-Marquartd
            // optimization process
            unsigned int n = 6 ;
            diagHsd.resize(n,n) ;
            diagHsd.eye(n);
            for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];
        }



    }
    else // iteration
    {
        // 3. Compute the control law and save velocities

        // Compute current visual feature
        sI.set_sigma(sigma);
        sI.buildFrom(I);
        // compute current error
        sI.error(sId,error);
        int S_g=0; // image laplacian

        if (pjModel==parallelZ)
        {
            vpColVector sg = sI.get_sg();
            int nbr_sg = sg.size();

            for (int m=0; m<nbr_sg; m++)
                S_g+= sg[m];

            sI.sg_error(sId.get_sg(),sg_error) ;
        }

        // Compute the levenberg Marquartd term
        H = ((mu * diagHsd) + Hsd).inverseByLU();
        //	compute the control law
        e = H * Lsd.t() *error ;
        // velocity
        v =  -lambda*e;

        if(pjModel==parallelZ)
        {
            /*
                      Jn.resize(6,5);
                      Jn[0][0]=1;
                      Jn[1][1]=1;
                      Jn[3][2]=1;
                      Jn[4][3]=1;
                      Jn[5][4]=1;

                      vpMatrix Lgs;//Hgs,diagHgs;
                      vpMatrix Ldgs, Ldg_inv;
                      vpMatrix Lg_temp; // temporar matrix for compute Lgs

                      vpMatrix Jdgs;

                      sI.interaction(Lg_temp);

                      Lgs = sI.get_Lg();
                      Ldgs = sI.get_Ldg();
                      //cout << "Ldg: " << Ldg << endl;

                      Jgn.resize(6,1);
                      Jgn[2][0]=1;

                      Jgs=-Lgs*cVw*Jgn;
                      Jdgs=-Ldgs*cVw*Jgn; // second derivative of visual feature jacobian

                      vpRowVector Jg, Jdg;
                      Jg.resize(1);
                      Jdg.resize(1);

                      for (int m=0; m<Jgs.getRows();m++)
                      {
            Jg[0]+=Jgs[m][0];
            Jdg[0]+=Jdgs[m][0];
                      }

                      if (iter> 100 && (cMod[2][3]-cMo[2][3] < 1e-6))
              vgd = -sign*lambda*0.5e-1/Jdg[0]*Jg[0];
                      else
              vgd = -sign*lambda*1e10/Jg[0];*/
        }
        if(pjModel==parallel)
        {
            vpColVector vc=v;
            v.resize(2);
            v[1]=vc[1];
            v[0]=vc[0];
        }
        else if (pjModel==parallelZ)
        {
            vpColVector vc=v;
            v.resize(3);
            v[2]=vgd;
            v[1]=vc[1];
            v[0]=vc[0];
        }

        new_v = v;



        // 4. return velocities
    }

}


void Client:: joinImages(Mat in,Mat &out, int firstrow, int firstcol)
{
    if (out.rows>firstrow && out.cols>firstcol)
    {
        int r=min(in.rows,out.rows-firstrow);
        int c=min(in.cols,out.cols-firstcol);
        for (int i=0; i<r; i++)
        {
            for (int j=0; j<c; j++)
            {
                out.data[(i+firstrow)*out.step+(j+firstcol)]=in.data[i*in.step+j];
            }
        }
    }
}

Mat Client::Compare(vpMatrix i1,vpMatrix i2)
{
    int l=0;
    int r=i1.getRows();
    int c=i1.getCols();
    Mat dst(r,c,CV_8UC1);
    if ( (unsigned)r==i2.getRows() && (unsigned)c==i2.getCols() )
    {
        for (int i=0; i<r; i++)
        {
            for (int j=0; j<c; j++)
            {
                if ( abs(i1.data[l]-i2.data[l])==0 )dst.data[l]=0;
                else dst.data[l]=255;
                l++;
            }
        }
    }
    return dst;
}

vpMatrix Client:: MatToVpMatrix(Mat in)
{
    vpMatrix out(in.rows,in.cols);
    for (int i=0; i<in.rows; i++)
        for (int j=0; j<in.cols; j++)
        {
            out[i][j]=(double)in.at<uchar>(i,j);
        }
    return out;
}

Mat Client::VpMatrixToMat(vpMatrix in,bool ok)
{
    Mat out(in.getRows(),in.getCols(),CV_8U);
    int l=0;
    int Min=-255;
    int Max=255;
    if (!ok)
    {
        for (unsigned int i=0; i<in.getRows(); i++)
        {
            for (unsigned int j=0; j<in.getCols(); j++)
            {
                Max=max(Max,(int)in[i][j]);
                Min=min(Min,(int)in[i][j]);
            }
        }
    }
    for (unsigned int i=0; i<in.getRows(); i++)
    {
        for (unsigned int j=0; j<in.getCols(); j++)
        {
            if (!ok)out.data[l]=(unsigned char)((in.data[l]-Min)*255/(Max-Min));
            else out.data[l]=(unsigned char)(in.data[l]);
            l++;
        }
    }

    return out;
}

void Client::Stuck(Mat in,Mat &out, int firstrow, int firstcol)
{
    if (out.rows>firstrow && out.cols>firstcol)
    {
        int r=min(in.rows,out.rows-firstrow);
        int c=min(in.cols,out.cols-firstcol);
        for (int i=0; i<r; i++)
        {
            for (int j=0; j<c; j++)
            {
                out.data[(i+firstrow)*out.step+(j+firstcol)]=in.data[i*in.step+j];
            }
        }
    }
}

/***********************************************
 *** Get Phase only correlation of the reference
 *** and current images.
 *** Input: REF Image, Cur Image (cv::Mat)
 *** Output: Correlation result image (cv::Mat)
***********************************************/
void Client::phase_correlation(Mat ref, Mat cur, Mat &resim)
{

    if(ref.cols!=cur.cols||ref.rows!=cur.rows)
    {
        cout<<"Images must be of same size...Quitting!"<<endl;
        exit(0);
    }
    int width     = ref.cols;
    int height    = cur.rows;
    int fft_size = width*height;
    int i, j, k;
    double tmp =0.0;
    resim.create(Size(width,height),CV_64F);

    /* allocate FFTW input and output arrays */
    fftw_complex *img1 = ( fftw_complex* )fftw_malloc( sizeof( fftw_complex ) * width * height );
    fftw_complex *img2 = ( fftw_complex* )fftw_malloc( sizeof( fftw_complex ) * width * height );
    fftw_complex *res  = ( fftw_complex* )fftw_malloc( sizeof( fftw_complex ) * width * height );


    /* setup FFTW plans */
    fftw_plan fft_img1 = fftw_plan_dft_1d( width * height, img1, img1, FFTW_FORWARD,  FFTW_ESTIMATE );
    fftw_plan fft_img2 = fftw_plan_dft_1d( width * height, img2, img2, FFTW_FORWARD,  FFTW_ESTIMATE );
    fftw_plan ifft_res = fftw_plan_dft_1d( width * height, res,  res,  FFTW_BACKWARD, FFTW_ESTIMATE );


    /* load images' data to FFTW input */
    for( i = 0, k = 0 ; i < height ; i++ )
    {
        for( j = 0 ; j < width ; j++, k++ )
        {
            img1[k][0] = ref.at<uchar>(i,j);
            img1[k][1] = (double)0.0;

            img2[k][0] = cur.at<uchar>(i,j);
            img2[k][1] = 0.0;
        }
    }

    /* obtain the FFT of img1 & img2 */
    fftw_execute( fft_img1 );
    fftw_execute( fft_img2 );

    /* obtain the cross power spectrum */
    for( i = 0; i < fft_size ; i++ )
    {
        res[i][0] = ( img2[i][0] * img1[i][0] ) - ( img2[i][1] * ( -img1[i][1] ) );
        res[i][1] = ( img2[i][0] * ( -img1[i][1] ) ) + ( img2[i][1] * img1[i][0] );

        tmp = sqrt( pow( res[i][0], 2.0 ) + pow( res[i][1], 2.0 ) );

        res[i][0] /= tmp;
        res[i][1] /= tmp;
    }

    /* obtain the phase correlation array */
    fftw_execute(ifft_res);

    /* normalize and copy to result image */
    for( i = 0, k = 0 ; i < height ; i++ )
    {
        for( j = 0 ; j < width ; j++, k++ )
        {
            resim.at<double>(i,j) = res[k][0] / ( double )fft_size;
        }
    }

    /* deallocate FFTW arrays and plans */
    fftw_destroy_plan( fft_img1 );
    fftw_destroy_plan( fft_img2 );
    fftw_destroy_plan( ifft_res );
    fftw_free( img1 );
    fftw_free( img2 );
    fftw_free( res );

}

void Client::DeleteAllFiles(char* folderPath)
{
    char fileFound[256];
    WIN32_FIND_DATA info;
    HANDLE hp;
    sprintf(fileFound, "%s\\*.*", folderPath);
    hp = FindFirstFile(fileFound, &info);
    do
    {
        sprintf(fileFound,"%s\\%s", folderPath, info.cFileName);
        cout<<fileFound<<endl;
        DeleteFile(fileFound);

    }
    while(FindNextFile(hp, &info));
    FindClose(hp);
}


/*****************************************************
 *** Get Mask image of the reference position
 *** Input: REF Image (cv::Mat)
 *** Output: Binary mask image (cv::Mat)
*****************************************************/

Mat Client::getMask(Mat I)
{
    vector<Point> R;
    vector<Point>RP;
    Point P1,P2,P3,P4;

    Mat binaryMat(I.size(), I.type());
    threshold(I, binaryMat, 100,255, THRESH_BINARY);

    Mat IRGB(I.size(), CV_8UC3);
    cvtColor(binaryMat, IRGB, CV_GRAY2RGB);

    namedWindow("Input",WINDOW_AUTOSIZE);
    setMouseCallback( "Input", onMouse, 0 );

    for(;;)
    {
        imshow("Input", IRGB);
        circle(IRGB,P,3,Scalar(0,0,255),-1,8,0);
        if(counter==1)
        {
            P1.x = P.x;
            P1.y = P.y;
        }
        if(counter==2)
        {
            P2.x = P.x;
            P2.y = P.y;
        }
        if(counter==3)
        {
            P3.x = P.x;
            P3.y = P.y;
        }
        if(counter==4)
        {
            P4.x = P.x;
            P4.y = P.y;
        }
        if( waitKey (30) >= 0||counter>4)
            break;
    }
    destroyAllWindows();

    R.push_back(P1);
    R.push_back(P2);
    R.push_back(P3);
    R.push_back(P4);

    Mat mask(I.size(), CV_8UC1);
    // Create black image with the same size as the original
    for(int i=0; i<mask.cols; i++)
        for(int j=0; j<mask.rows; j++)
            mask.at<uchar>(Point(i,j)) = 0;

    approxPolyDP(R, RP, 1.0, true);
    fillConvexPoly(mask, &RP[0], RP.size(), 255, 8, 0);

    // Create new image for result storage
    Mat imageDest = cvCreateMat(512, 512, I.type());

    // Cut out ROI and store it in imageDest
    binaryMat.copyTo(imageDest, mask);
    return imageDest;
}

/*****************************************************
 *** Get Phase only correlation of the reference
 *** and current images.
 *** Input: REF Image, Cur Image (cv::Mat)
 *** Output: Correlation result image (cv::Mat)
*****************************************************/
void Client::Sharpen(const Mat& myImage,Mat& Result)
{
    CV_Assert(myImage.depth() == CV_8U);  // accept only uchar images

    const int nChannels = myImage.channels();
    Result.create(myImage.size(),myImage.type());

    for(int j = 1 ; j < myImage.rows-1; ++j)
    {
        const uchar* previous = myImage.ptr<uchar>(j - 1);
        const uchar* current  = myImage.ptr<uchar>(j    );
        const uchar* next     = myImage.ptr<uchar>(j + 1);

        uchar* output = Result.ptr<uchar>(j);

        for(int i= nChannels; i < nChannels*(myImage.cols-1); ++i)
        {
            *output++ = saturate_cast<uchar>(5*current[i]
                                             -current[i-nChannels] - current[i+nChannels] - previous[i] - next[i]);
        }
    }

    Result.row(0).setTo(Scalar(0));
    Result.row(Result.rows-1).setTo(Scalar(0));
    Result.col(0).setTo(Scalar(0));
    Result.col(Result.cols-1).setTo(Scalar(0));
}


void onMouse( int event, int x, int y, int, void* )
{
    if( event != EVENT_LBUTTONDOWN )
        return;

    Point pt = Point(x,y);
    P = pt;
    G.push_back(pt);
    counter++;
}

/**----------- Draw gripper function -------*/
void Client::drawGripper(Mat in, Mat out)
{
    //----------------------------------
    vector<Point>LF;  // for polygon vertices
    vector<Point>RF;  // for polygon vertices
    out.create(in.size(),in.type());
    //------------- POINTS - left finger -------------
    vector<Point>L;
//    L.push_back(Point(310,0));
//    L.push_back(Point(310,86));
//    L.push_back(Point(319,95));
//    L.push_back(Point(319,107));
//    L.push_back(Point(309,109));
//    L.push_back(Point(295,94));
//    L.push_back(Point(276,1));
    L.push_back(Point(396,2));
    L.push_back(Point(355,93));
    L.push_back(Point(359,103));
    L.push_back(Point(354,114));
    L.push_back(Point(343,110));
    L.push_back(Point(338,94));
    L.push_back(Point(360,1));


    //------------- POINTS - right finger -------------
    vector<Point>R;
//    R.push_back(Point(368,0));
//    R.push_back(Point(368,86));
//    R.push_back(Point(359,93));
//    R.push_back(Point(359,108));
//    R.push_back(Point(372,107));
//    R.push_back(Point(384,95));
//    R.push_back(Point(402,0));
    R.push_back(Point(462,1));
    R.push_back(Point(408,116));
    R.push_back(Point(397,121));
    R.push_back(Point(392,131));
    R.push_back(Point(402,136));
    R.push_back(Point(419,130));
    R.push_back(Point(510,2));

//    for(unsigned int i=0; i<L.size()-1; i++)
//    {
//        line(in, L[i],L[i+1],Scalar(0,0,255),2,8,0);
//    }
//    for(unsigned int i=0; i<R.size()-1; i++)
//    {
//        line(in, R[i],R[i+1],Scalar(0,0,255),2,8,0);
//    }

    approxPolyDP(L, LF, 1.0, true);
    fillConvexPoly(in, &LF[0], LF.size(), Scalar(0,255,0), 8, 0);

    approxPolyDP(R, RF, 1.0, true);
    fillConvexPoly(in, &RF[0], RF.size(), Scalar(0,255,0), 8, 0);

    in.copyTo(out);
}



/**----------- Draw gripper function -------*/
void Client::drawGripperMouse (Mat in, Mat out)
{
    //----------------------------------
    vector<Point>GP;  // for polygon vertices
    out.create(in.size(),in.type());

    namedWindow("Input",WINDOW_AUTOSIZE);
    setMouseCallback( "Input", onMouse, 0 );
    while(1)
    {
        imshow("Input", in);
        if(waitKey(30)>=0)
            break;
    }
    destroyAllWindows();
//    for(unsigned int i=0; i<G.size()-1; i++)
//    {
//        line(in, G[i],G[i+1],Scalar(0,0,255),2,8,0);
//    }

    approxPolyDP(G, GP, 1.0, true);
    fillConvexPoly(in, &GP[0], GP.size(), 255, 8, 0);

    in.copyTo(out);
}
