#include "controlserver.h"
#include <iostream>

using namespace std;
controlserver::controlserver()
{
    //ctor
}

controlserver::~controlserver()
{
    //dtor
}

/**===============================================
** Function to build the desired feature
**================================================*/
vpFeatureLuminance controlserver::getDesired(IplImage *im)
{
    vpFeatureLuminance sId;
    vpImage<unsigned char> Id; //desired image
    vpImageConvert::convert(im,Id);

    sId.init(Id.getHeight(), Id.getWidth(), Z) ;
    // sId.setCameraParameters(cam) ;
    sId.buildFrom(Id) ;
    sId.interaction(Lsd) ;

    cvReleaseImage(&im);
    return sId;
}

/**===============================================
** Function to build ther current feature
**================================================*/

vpFeatureLuminance controlserver::getCurrent(IplImage *im)
{
    vpFeatureLuminance sI;
    vpImage<unsigned char> I; //current image
    vpImageConvert::convert(im,I);
    sI.init( I.getHeight(), I.getWidth(), Z);
    sI.buildFrom(I) ;
    cvReleaseImage(&im);
    return sI;
}

/**===============================================
** Function to process the control law
** Outputs the velocity vector of translation and
** rotation
**================================================*/
vpColVector controlserver::computeControl(vpFeatureLuminance sId,vpFeatureLuminance sI)
{

    double var,norm,mean;
    float mu=0.0001;
    double lambda = 1000;

    Hsd = Lsd.AtA() ;
    dH=Diagonal(Hsd);
    C=(Hsd+mu*dH).pseudoInverse()*Lsd.t();

    sI.error(sId,error);
    Stat(error,mean,var,norm);
    cout<<mean<<"  " <<var<<"  "<<norm<<"  "<<endl;

    V = C*error ;
    V = V* lambda;

    return V;
}
/**===============================================
** Function to compute the statistics formulae
**================================================*/
void controlserver::Stat(vpColVector src,double &mean,double &var,double &norm)
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

/**===============================================
** Function to produce the diagonal matrix
**================================================*/
vpMatrix controlserver::Diagonal(vpMatrix H)
{
    vpMatrix dH;
    dH.eye(H.getCols());
    for(int i = 0 ; i < H.getCols() ; i++)
        dH[i][i] = H[i][i];
    return dH;
}
