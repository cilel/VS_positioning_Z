#include "Client.h"

#include <iostream>
/** Project local libraries */
#include "SyncSerialComm.h"
#include "myLog.h"
#include "tritor.h"
#include "npFeatureLuminance.h"

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
tritor *stage = new tritor();                   //! Stage object
Point P;
vector<Point> G;
int counter =0,gCount=0;
void onMouse( int event, int x, int y, int, void* );
vpColVector vel;


/** -------Global variables for controlling ------**/

// 2. define variables - use the global variable vpColvector vel
//vpHomogeneousMatrix cMo,cMod,wMe,eMo,cMw,wMcR,wMc,wMo,Tr,cMe; //Robot-camera reference
//vpCameraParameters cam; // camera parameters

double Zz = 0.007;  // Z position !!!! This should be the (inital) distance between sensor and object, in meter !!!!
double sigma=5; // sigma in Gauss PDF, not used in X-Y control

npFeatureLuminance sI; // current feature
npFeatureLuminance sId; // desired feature

vpMatrix Lsd;   // matrice d'interaction a la position desiree
vpMatrix Hsd;  // hessien a la position desiree
vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
vpColVector err ; // Erreur I-I*, photometric information

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

double lambda=100000;
double mu;




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
    winLog<<"Cleared all result directories..."<<endl;
    /**============ Results files ===========*/
    ofstream x_voltfile("RESULTS\\result_x_voltage.txt");
    ofstream y_voltfile("RESULTS\\result_y_voltage.txt");
    ofstream stats     ("RESULTS\\statvals.txt");
    ofstream motion(    "RESULTS\\motion.txt");

    /**======== Communication variables ======== */
    char rec[32]   = "",imfile[500]="";
    char fname[32] = "";         // Received filename from server
    int numbits;                 // Sent or Received data from server
    char write1_buffer[256] =""; // Buffer for serial communication

//    /**============ VISP Variables ==============*/
    vpMatrix D;
    vpColVector nV;

    /**============ Other computational variables ==============*/
    int iter =0;
    char saveimg[50]="";
    double temp_disp_x=0.0,temp_disp_y=0.0;
    double x_disp = 0.0,y_disp = 0.0,x_volt = 0.0,y_volt = 0.0;
    double var,norm,mean;

    Mat resIm;
    Mat resIm1;
    Point minLoc, maxLoc;

    vector<double>disp_x_act;   // X-voltage
    vector<double>disp_y_act;   // Y-voltage
    vector<double>vel_x;        // X-veleocity
    vector<double>vel_y;        // Y-velocity



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

        cout<<"CURRENT IMAGE ACQUIRED.... DEBUG 1"<<endl;


        /**==========================================
        **              MAIN CONTROL FUNCTION
        ===========================================*/

        //!------- HERE IS THE FUNCTION OF "LE CUI"
        if(iter==0)      // Modification by naresh
        {
            nV = semPosCont(curIm,refIm,true);
            cout<<"First image.... DEBUG 2"<<endl;
        }

        else
        {
            nV = semPosCont(curIm,refIm,false);
            cout<<"from second image.... DEBUG 3"<<endl;
        }

        cout<<"THE NEW VALUES ARE: "<<nV[1]<<"         "<<nV[0]<<endl;
        vel_x.push_back(nV[1]);
        vel_y.push_back(nV[0]);


        /**============ Compute displacements and voltages ============*/
        if(iter>0)
        {

            Stat(e,mean,var,norm);
            stats<<mean<<setw(20)<<var<<setw(20)<<norm<<"  ;"<<endl;

            /**----------TX -------------*/
            x_disp = ((vel_x[0] + vel_x[iter-1])/2)*0.1;
            disp_x_act.push_back(x_disp);
            x_voltfile<<nV[1]<<setw(15)<<x_disp<<setw(15);

            x_disp = temp_disp_x + x_disp;
            x_volt = stage->move_inc_x(abs(x_disp));


            if(x_volt>110||x_volt<-20)
            {
                cout<<"Error: Voltage overflow for - x axis"<<endl;
            }

            x_voltfile<<x_disp<<setw(15)<<x_volt<<"  ;"<<endl;;
            temp_disp_x = x_disp;


            /**----------TY -------------*/
            y_disp = ((vel_y[0] + vel_y[iter-1])/2)*0.08;
            disp_y_act.push_back(y_disp);
            y_voltfile<<nV[1]<<setw(15)<<y_disp<<setw(15);

            y_disp = temp_disp_y + y_disp;
            y_volt = stage->move_inc_y(abs(y_disp));

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

        Stuck(refIm,totIm,0,0);
        Stuck(curIm,totIm,r+10,0);
        Stuck(diff,totIm,0,c+10);
        Stuck(cmp,totIm,c+10,c+10);
        imshow("TOTAL",totIm);

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

vpColVector Client:: semPosCont(Mat curImage, Mat desImage, bool init)
{
    // 1. Conver to Visp
    vpImage<unsigned char> I, Id;
    vpImageConvert::convert(curImage, I);
    vpImageConvert::convert(desImage, Id);

    pjModel = parallel;

    if(init)
    {

        if(pjModel == parallel)
            sId.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::parallel) ;
        else if(pjModel == parallelZ)
            sId.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::parallelZ,npFeatureLuminance::ImageGradient) ;
        else
            sId.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::perspective) ;
        //sId.setCameraParameters(cam) ;
        sId.set_sigma(10);//just a value, not useful
        sId.buildFrom(Id) ;

        if(pjModel == parallel)
            sI.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::parallel) ;
        else if(pjModel == parallelZ)
            sI.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::parallelZ,npFeatureLuminance::ImageGradient) ;
        else
            sI.init( I.getHeight(), I.getWidth(), Zz, npFeatureLuminance::perspective) ;
        //sI.setCameraParameters(cam) ;
        sI.set_sigma(sigma);
        sI.buildFrom(I) ;

        sId.interaction(Lsd) ;
        // For Z
        // Lgsd = sId.get_Lg();

        if(pjModel==parallel )
        {
            // Compute the Hessian H = L^TL
            Jn.resize(6,5);
            Jn[0][0]=1;
            Jn[1][1]=1;
            Jn[3][2]=1;
            Jn[4][3]=1;
            Jn[5][4]=1;
            Js=-Lsd*Jn;

            Hsd = Js.AtA() ;

            cout << "Hsd=\n" << Hsd <<endl;
            // Compute the Hessian diagonal for the Levenberg-Marquartd
            // optimization process
            unsigned int n = 5 ;
            diagHsd.resize(n,n) ;
            diagHsd.eye(n);
            for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];

            cout << "diagHsd=\n" << diagHsd <<endl;
        }
        v.resize(6);

    }
    /*---------------------- iteration ------------------------*/
    else
    {
        // 3. Compute the control law and save velocities

        // Compute current visual feature
        sI.set_sigma(sigma);
        sI.buildFrom(I);
        // compute current error
        sI.error(sId,err);
        //int S_g=0; // image laplacian
        //cout<<"mu=" <<mu << "\t diagHsd:" <<diagHsd.getRows()<<"x"<<diagHsd.getRows()<< "\t Hsd:" <<Hsd.getRows()<<"x"<<Hsd.getRows()<< endl;
        // Compute the levenberg Marquartd term
        H = ((mu * diagHsd) + Hsd).inverseByLU();
        //	compute the control law
        e = H * Js.t() *err ;
        // velocity
        v = -lambda*e;

        cout << "v=" << v.t() << endl;

        // 4. return velocities
        if(pjModel==parallel)
        {
            vpColVector vc=v;
            v.resize(6);
            v[5]=0;//vc[4];
            v[4]=0;//vc[3];
            v[3]=0;//vc[2];
            v[2]=0;
            v[1]=-vc[1];
            v[0]=vc[0];
        }


    }
    return v;

}

/*****************************************************

*****************************************************/
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

/*****************************************************

*****************************************************/
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

/*****************************************************

*****************************************************/
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

/*****************************************************

*****************************************************/
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

/*****************************************************

*****************************************************/

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

/*****************************************************

*****************************************************/
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

*****************************************************/
void Client::Stat(vpColVector src,double &mean,double &var,double &norm)
{
    mean=0;
    var=0;
    norm=0;
    int N=src.getRows();
    for (int k=0; k<N; k++)
    {
        mean+=src[k];
        norm+=pow(src[k],2.);
    }
    mean/=N;
    var=norm/N-pow(mean,2.);
    norm=sqrt(norm);
}
