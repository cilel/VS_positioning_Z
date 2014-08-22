/****************************************************************************
 *
 * Feature Luminance by perspective and parallel projection.
 *****************************************************************************/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>
#include <visp/vpException.h>

#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include "npFeatureLuminance.h"

using namespace std;
/*!
  \file vpFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/



/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
npFeatureLuminance::init()
{
    if (flags == NULL)
      flags = new bool[nbParameters];
    for (unsigned int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value Z (1 meters)
    Z_fl = 1;

    firstTimeIn =0 ;

}


void
npFeatureLuminance::init(unsigned int _nbr, unsigned int _nbc, double _Z, projectionModel projModel, visualFeature_Z vfeature_Z)
{
  init() ;

  nbr = _nbr ;
  nbc = _nbc ;

  if((nbr < 2*bord) || (nbc < 2*bord)){
    throw vpException(vpException::dimensionError, "border is too important compared to number of row or column.");
  }

  // number of feature = nb column x nb lines in the images
  dim_s = (nbr-2*bord)*(nbc-2*bord) ;

  imIx.resize(nbr,nbc) ;
  imIy.resize(nbr,nbc) ;
  imG.resize(nbr,nbc) ;
  imGxy.resize(nbr,nbc) ;//for display gradient

  s.resize(dim_s) ;
  sg.resize(dim_s);
  
  if (pixInfo != NULL)
    delete [] pixInfo;

  pixInfo = new npLuminance[dim_s] ;
  
  Z_fl = _Z ;
  pjModel = projModel;

  vf_Z = vfeature_Z;

  S_c = 1;
  S_p = 0;
}

/*! 
  Default constructor that build a visual feature.
*/
npFeatureLuminance::npFeatureLuminance() : vpBasicFeature()
{
    nbParameters = 1;
    dim_s = 0 ;
    bord = 10 ;
    flags = NULL;
    pixInfo = NULL;

    init() ;
}

/*! 
  Default destructor.
*/
npFeatureLuminance::~npFeatureLuminance()
{
  if (pixInfo != NULL) delete [] pixInfo ;
  if (flags != NULL) delete [] flags;
}



/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
npFeatureLuminance::set_Z(const double Z)
{
    this->Z_fl = Z ;
    flags[0] = true;
}

void
npFeatureLuminance::set_sigma(const double sigma)
{
    this->sigma = sigma ;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
npFeatureLuminance::get_Z() const
{
    return Z_fl ;
}

vpMatrix
npFeatureLuminance::get_Lg()
{
    return Lg;
}

vpMatrix
npFeatureLuminance::get_Ldg()
{
    return Ldg;
}

vpColVector
npFeatureLuminance::get_sg()
{
    return sg;
}

double
npFeatureLuminance::get_sg_sum()
{
    double S_g = 0;
    for (int m=0; m<sg.size();m++)
        S_g+= abs(sg[m]);

    return S_g;
}




void
npFeatureLuminance::setCameraParameters(vpCameraParameters &_cam)
{
  cam = _cam ;
}

/*!

  Build a luminance feature directly from the image
*/

void
npFeatureLuminance::matrixConvert(cv::Mat& src, vpMatrix& dest)
{
    cv::Size ksize;
    ksize = src.size();
    dest.resize(ksize.height,ksize.width);
    for(int i=0; i< ksize.height; i++)
        for(int j=0; j< ksize.width; j++)
            dest[i][j]=src.at<float>(i,j);

}

void
npFeatureLuminance::matrixConvert(vpMatrix& src, cv::Mat& dest)
{
    cv::Size ksize;
    ksize.height = src.getRows();
    ksize.width = src.getCols();
    dest.create(ksize,CV_32F);
    for(int i=0; i< ksize.height; i++)
        for(int j=0; j< ksize.width; j++)
            dest.at<float>(i,j)=src[i][j];
}

void
npFeatureLuminance::matrixConvert(vpImage<double> &src, cv::Mat& dest)
{
    cv::Size ksize;
    ksize.height = src.getRows();
    ksize.width = src.getCols();
    dest.create(ksize,CV_32F);
    for(int i=0; i< ksize.height; i++)
        for(int j=0; j< ksize.width; j++)
            dest.at<float>(i,j)=src[i][j];
}


void
npFeatureLuminance::buildFrom(vpImage<unsigned char> &I)
{
  unsigned int l = 0;
  double Ix,Iy,Ixx, Ixy, Iyx, Iyy ,Ixs, Iys, Ixss, Iyss;

  double px = cam.get_px() ;
  double py = cam.get_py() ;
  double a = -15;//6e5;

  if (firstTimeIn==0)
    { 
      firstTimeIn=1 ;
      l =0 ;
    for (unsigned int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (unsigned int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
          vpPixelMeterConversion::convertPointWithoutDistortion(cam,
						   i, j,
						   y, x)  ;
	    
	      pixInfo[l].x = x;
	      pixInfo[l].y = y;
          pixInfo[l].Z_l = Z_fl ;

	      l++;
	    }
	}
    }


  for (int i=3; i < nbr-3 ; i++)
  {
  //   cout << i << endl ;
    for (int j = 3 ; j < nbc-3; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      imIx[i][j] =  vpImageFilter::derivativeFilterX(I,i,j) ;//px *
      imIy[i][j] =  vpImageFilter::derivativeFilterY(I,i,j) ;//py *
      //cout << imIx[i][j] << endl;
    }
  }
  Ixs = 0;
  Iys = 0;
 vpMatrix Iuv; // derivative of gaussian kernel
 //vpMatrix I2uv; // second derivaive of gaussian kernel
 int filter_size = 25;
 Iuv.resize(filter_size,filter_size);
 //I2uv.resize(filter_size,filter_size);

 double sigma_2_inv = 0.5/(sigma*sigma);
 double sigma_3_inv = 1/(sigma*sigma*sigma);
 double sigma_8_inv = sigma_2_inv*sigma_3_inv*sigma_3_inv;
 double sigma_2 = sigma*sigma;
 const double pi_inv = 1/3.1415926;
 double Z_2_inv = 1/(Z_fl*Z_fl);

 //cout << sigma << endl;
 //cout << "sigma_inverse_sq=" << sigma_inverse_sq << endl;
 //cout << "sigma_3=" << sigma_3 << endl;

  for (int u=0; u < filter_size ; u++)
    for (int v = 0 ; v < filter_size; v++)
    {
        int u2=u*u;
        int v2=v*v;
        double exp_uv = exp(-(u2+v2)*sigma_2_inv);
        //cout << "exp_uv=" << exp_uv << endl;
        Iuv[u][v] = -a*Z_2_inv*((u2+v2)*sigma_2_inv-1)*pi_inv*sigma_3_inv*exp_uv;
   //     I2uv[u][v] = -a*Z_2_inv*((u2+v2-6*sigma_2)*(u2+v2-sigma_2)*pi_inv*sigma_8_inv*exp_uv*(-a)*Z_2_inv+((u2+v2)*sigma_2_inv-1)*pi_inv*sigma_3_inv*exp_uv)*(-(2-a*2*sigma_2_inv)*Z_2_inv);//

        //Iuv[u][v] = a*(u*u+v*v)*exp(-(u*u+v*v)*sigma_inverse_sq)*sigma_3;
        //cout << Iuv[u][v] << "\t" << u << "\t" << v << "\t" << sigma << endl;
    }

  cv::Mat cvimIx, cvimIy, cvIuv, cvI2uv, cvIxs, cvIys, cvIxss, cvIyss;
  matrixConvert(imIx,cvimIx);
  matrixConvert(imIy,cvimIy);
  matrixConvert(Iuv,cvIuv);
 // matrixConvert(I2uv,cvI2uv);


  double tm0 = vpTime::measureTimeSecond();

  l= 0 ;
  for (int i=bord; i < nbr-bord ; i++)
  {
  //   cout << i << endl ;
    for (int j = bord ; j < nbc-bord; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      Ix =  imIx[i][j] ;
      Iy =  imIy[i][j] ;

      sg[l] = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ;//sqrt
      //imGxy[i][j] = sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ;
      Ixx =  vpImageFilter::derivativeFilterX(imIx,i,j) ;
      Ixy =  vpImageFilter::derivativeFilterY(imIx,i,j) ;
      Iyx =  vpImageFilter::derivativeFilterX(imIy,i,j) ;
      Iyy =  vpImageFilter::derivativeFilterY(imIy,i,j) ;
/*
      if(l%20000==0)
          cout << "sg[" << l << "]=" << sg[l] << endl;
*/
      Ixs =0;
      Iys =0;
      Ixss =0;
      Iyss =0;

      /*----Convolution----*/
/*
      for (int u= -filter_size/2; u <= filter_size/2 ; u++)
      {
        for (int v = -filter_size/2 ; v < filter_size/2; v++)
        {
          if( (i+u >= 3) && (i+u < nbr-3) && (j+v >= 3) && (j+v < nbc-3))
          {
              Ixs += imIx[i+u][j+v]*Iuv[u+filter_size/2][v+filter_size/2];// i+/-u isn't the same thing
              Iys += imIy[i+u][j+v]*Iuv[u+filter_size/2][v+filter_size/2];

              Ixss += imIx[i+u][j+v]*I2uv[u+filter_size/2][v+filter_size/2];// i+/-u isn't the same thing
              Iyss += imIy[i+u][j+v]*I2uv[u+filter_size/2][v+filter_size/2];
          }
        }
     }
*/
      //cout << "Ixs, Iys = " << Ixs << "\t" << Iys <<endl;

      // Calcul de Z
      pixInfo[l].Ix_g  = Ix*px;
      pixInfo[l].Iy_g  = Iy*py;
      pixInfo[l].Ixx  = Ixx;
      pixInfo[l].Ixy  = Ixy;
      pixInfo[l].Iyx  = Iyx;
      pixInfo[l].Iyy  = Iyy;
 /*
      pixInfo[l].Ixs = Ixs;
      pixInfo[l].Iys = Iys;
      pixInfo[l].Ixss = Ixss;
      pixInfo[l].Iyss = Iyss;
*/
     // cout << "Ixs, Iys = " << pixInfo[l].Ixs << "\t" << pixInfo[l].Iys  <<endl;

      imGxy[i][j] = vpMath::sqr(Ix) + vpMath::sqr(Iy) ;

      l++;

    }
}
  //cout << "max[l]=" << l << endl;

  cv::filter2D(cvimIx,cvIxs,-1,cvIuv);
  cv::filter2D(cvimIy,cvIys,-1,cvIuv);
//  cv::filter2D(cvimIx,cvIxss,-1,cvI2uv);
//  cv::filter2D(cvimIy,cvIyss,-1,cvI2uv);


  vpMatrix mIxs,mIys,mIxss,mIyss;

  matrixConvert(cvIxs,mIxs);
  matrixConvert(cvIys,mIys);
  //matrixConvert(cvIxss,mIxss);
 // matrixConvert(cvIyss,mIyss);

  l= 0 ;
  for (int i=bord; i < nbr-bord ; i++)
  {
    for (int j = bord ; j < nbc-bord; j++)
    {
        pixInfo[l].Ixs = mIxs[i][j];
        pixInfo[l].Iys = mIys[i][j];
    //    pixInfo[l].Ixss = mIxss[i][j];
    //    pixInfo[l].Iyss = mIyss[i][j];

        l++;
    }

  }


  cout << "compute time=" <<  vpTime::measureTimeSecond()-tm0 <<endl;

  S_p = S_c;
  S_c = 0;

    for(int l = 0; l<dim_s; l++)
    {
        S_c +=sg[l];
    }
/*
  cout << "nbr=" << nbr << endl;
  cout << "nbc=" << nbc << endl;
*/

  if(vf_Z==NormalizedVariance)
  {
      //For Normalized variance
      double sum=0;
      double mean;
      for (unsigned int i=bord; i < nbr-bord ; i++)
          for (unsigned int j=bord; j < nbc-bord ; j++)
      {
            sum +=I[i][j];
      }

      mean = sum/(nbr*nbc);

      double mean_iv = 1/mean;

      l=0;

      for (unsigned int i=bord; i < nbr-bord ; i++)
          for (unsigned int j=bord; j < nbc-bord ; j++)
      {
            pixInfo[l].norVar = (I[i][j]-mean)*(I[i][j]-mean)*mean_iv;
            //sg[l] = fabs((I[i+1][j]- I[i][j]));//F-1 absolute gradiant
            //sg[l] = (I[i+1][j]- I[i][j])*(I[i+1][j]- I[i][j]);//F-2 Squared gradiant
            //sg[l] = (I[i+2][j]- I[i][j])*(I[i+2][j]- I[i][j]);//F-3 Brenner gradiant
            //sg[l] = (I[i][j]-mean)*(I[i][j]-mean);//F-10 variance
            sg[l] = (I[i][j]-mean)*(I[i][j]-mean)*mean_iv;//F-11 Normalized variance
            //sg[l] = I[i][j]*I[i+1][j]- I[i][j]*I[i+2][j];//F-12 AutoCorrelation
            //sg[l] = (I[i][j]*I[i+1][j])-mean*mean;//F-13 Standard-deviation-based correlation
            //sg[l] = I[i][j]*I[i][j];// F-18 image power

            //cout << "sg[l]=" << sg[l] << endl;
            l++;
      }
  }

  //For luminance
  l= 0 ;
  for (unsigned int i=bord; i < nbr-bord ; i++)
    {
      //   cout << i << endl ;
      for (unsigned int j = bord ; j < nbc-bord; j++)
	{
	  // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      Ix =  px * vpImageFilter::derivativeFilterX(I,i,j) ;
	  Iy =  py * vpImageFilter::derivativeFilterY(I,i,j) ;
	  
	  // Calcul de Z
	  
	  pixInfo[l].I  =  I[i][j] ;
	  s[l]  =  I[i][j] ;
      pixInfo[l].Ix  = Ix;
      pixInfo[l].Iy  = Iy;

	  l++;
	}
    }

}


/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
npFeatureLuminance::interaction(vpMatrix &L)
{
  double x,y,Ix,Iy,z,Zinv;

  double Ixx, Ixy, Iyx, Iyy, Ix_g, Iy_g, Ixs, Iys, Ixss, Iyss;

  if(pjModel==perspective)
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          Zinv =  1 / pixInfo[m].Z_l;

          {
            L[m][0] = Ix * Zinv;
            L[m][1] = Iy * Zinv;
            L[m][2] = -(x*Ix+y*Iy)*Zinv;
            L[m][3] = -Ix*x*y-(1+y*y)*Iy;
            L[m][4] = (1+x*x)*Ix + Iy*x*y;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
  }
  else if(pjModel==parallel)
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          z = pixInfo[m].Z_l ;

          {
            L[m][0] = Ix;
            L[m][1] = Iy ;
            /*L[m][2] = -Iy * z;
            L[m][3] = Ix * z;
            L[m][4]  = Iy*x-Ix*y;*/

            L[m][2] =  0 ;
            L[m][3] = -Iy * z;
            L[m][4] = Ix * z;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
    }
  else if(pjModel==parallelZ)
  {
        L.resize(dim_s,6) ;
        Lg.resize(dim_s,6);
        Ldg.resize(dim_s,6);

        //CostFunctionSg=0;

      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Zinv =  1 / pixInfo[m].Z_l;
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          z = pixInfo[m].Z_l;

          Ixx = pixInfo[m].Ixx;
          Ixy = pixInfo[m].Ixy;
          Iyx = pixInfo[m].Iyx;
          Iyy = pixInfo[m].Iyy;

          Ix_g = pixInfo[m].Ix_g;
          Iy_g = pixInfo[m].Iy_g;

          Ixs = pixInfo[m].Ixs;
          Iys = pixInfo[m].Iys;

          Ixss = pixInfo[m].Ixss;
          Iyss = pixInfo[m].Iyss;

          //cout << "Ix and Ix_g: "<<Ix << "  " << Ix_g << endl;

          double A = ( Ixx*Ix_g+Iyx*Iy_g );
          double B = ( Ixy*Ix_g+Iyy*Iy_g );
          double D = (Ix*Ixs +Iy*Iys); // maximize image gradient as cost function => mimimize derivative of image gradient
      //    double E = (Ixs*Ixs+Ix*Ixss +Iys*Iys+Iy*Iyss); // derivative of D;

/*
          if(m%20000==0)
          {
              cout << "D[" << m << "]=" << D <<endl;
              cout << "E[" << m << "]=" << E <<endl;
          }
*/
          //cout << "Ix,Ixs, Iy,Iys= " << Ix << "\t" << pixInfo[m].Ixs << "\t"<< Iy << "\t"<< Iys << "\t" << endl;

          double C= -( Ix+Iy )*pixInfo[m].norVar;

          {
            L[m][0] = Ix;
            L[m][1] = Iy ;
            L[m][2] = 0;//(A+B);
            L[m][3] = -Iy * z;
            L[m][4] = Ix * z;
            L[m][5]  = Iy*x-Ix*y;
          }

          if(vf_Z==ImageGradient)
          {
              Lg[m][0] = 0;
              Lg[m][1] = 0;
              Lg[m][2] = 2*D;//(A*x+B*y)*Zinv;//2*(A+B)//2*D;
              Lg[m][3] = 0;
              Lg[m][4] = 0;
              Lg[m][5] = 0;
/*
              Ldg[m][0] = 0;
              Ldg[m][1] = 0;
              Ldg[m][2] = 2*E;
              Ldg[m][3] = 0;
              Ldg[m][4] = 0;
              Ldg[m][5] = 0;
*/
              //Lz[m] = Lg[m][2];
              //CostFunctionSg += (A+B);
              //cout << "CostFunctionLz="<<CostFunctionSg << endl;
          } 
          else if(vf_Z==NormalizedVariance)
          {
              Lg[m][0] = 0;
              Lg[m][1] = 0;
              Lg[m][2] = C;
              Lg[m][3] = 0;
              Lg[m][4] = 0;
              Lg[m][5] = 0;
          }
      }
    }
  else
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          Zinv =  1 / pixInfo[m].Z_l;

          {
            L[m][0] = Ix * Zinv;
            L[m][1] = Iy * Zinv;
            L[m][2] = -(x*Ix+y*Iy)*Zinv;
            L[m][3] = -Ix*x*y-(1+y*y)*Iy;
            L[m][4] = (1+x*x)*Ix + Iy*x*y;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
  }
        //cout << "L=" << L << endl;
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/

vpMatrix  npFeatureLuminance::interaction(const unsigned int /* select */)
{
  /* static */ vpMatrix L  ; // warning C4640: 'L' : construction of local static object is not thread-safe
  interaction(L) ;
  return L ;
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void
npFeatureLuminance::error(const vpBasicFeature &s_star,
			  vpColVector &e)
{
  e.resize(dim_s) ;

  for (unsigned int i =0 ; i < dim_s ; i++)
    {
      e[i] = s[i] - s_star[i] ;
    }
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired image gradient

  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.
*/
void
npFeatureLuminance::sg_error(const vpColVector &sg_star,
              vpColVector &eg)
{
  eg.resize(dim_s) ;

  for (unsigned int i =0 ; i < dim_s ; i++)
    {
      eg[i] = sg[i] - sg_star[i] ;
    // eg[i] = sg[i] ; // use maximazing cost function C=delta(x)^2+delta(y)^2
    }
}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector
npFeatureLuminance::error(const vpBasicFeature &s_star,
			  const unsigned int /* select */)
{
  /* static */ vpColVector e ; // warning C4640: 'e' : construction of local static object is not thread-safe
  
  error(s_star, e) ;
  
  return e ;

}




/*!

  Not implemented.

 */
void
npFeatureLuminance::print(const unsigned int /* select */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
 }



/*!

  Not implemented.

 */
void
npFeatureLuminance::display(const vpCameraParameters & /* cam */,
                            const vpImage<unsigned char> & /* I */,
                            const vpColor &/* color */,
                            unsigned int /* thickness */) const
{
 static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void
npFeatureLuminance::display(const vpCameraParameters & /* cam */,
                            const vpImage<vpRGBa> & /* I */,
                            const vpColor &/* color */,
                            unsigned int /* thickness */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
npFeatureLuminance *npFeatureLuminance::duplicate() const
{
  npFeatureLuminance *feature = new npFeatureLuminance ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
