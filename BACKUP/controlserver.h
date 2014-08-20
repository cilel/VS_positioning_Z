#ifndef CONTROLSERVER_H
#define CONTROLSERVER_H

/**  C_Std Include libraries  */

#include <iostream>
#include <fstream>
#include <time.h>

/** VISP include libraries   */
#include <visp/vpImage.h>
#include<visp/vpImageIo.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImageConvert.h>
#include <visp/vpFeatureLuminance.h>

/** OpenCV Include libraries */
#include <opencv/cv.h>
#include <opencv/highgui.h>


#include "tritor.h"

#define Z 1

class controlserver
{
public:
    controlserver();
    virtual ~controlserver();
    vpFeatureLuminance getDesired(IplImage *im);
    vpFeatureLuminance getCurrent(IplImage *im);
    vpColVector computeControl(vpFeatureLuminance , vpFeatureLuminance );

    vpMatrix Lsd; // Interaction matrix
    vpMatrix Hsd; // hessien a la position desiree
    vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
    vpMatrix dH;
    vpMatrix D;
    vpMatrix C; //!< Matrice de combinaison \see getImageDesiree
    vpColVector error ;
    vpColVector V;

    void Stat(vpColVector src,double &mean,double &var,double &norm);
    vpMatrix Diagonal(vpMatrix H);
protected:


private:

};

#endif // CONTROLSERVER_H
