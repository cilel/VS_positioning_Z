#include "functions.h"
using namespace std;
using namespace cv;
FUNCTIONS::FUNCTIONS()
{
    //ctor
}

FUNCTIONS::~FUNCTIONS()
{
    //dtor
}
void FUNCTIONS::computeCost(IplImage *img,double &mean,double &variance)
{
    CvScalar s;

    int height = img->height;
    int width  = img->width;

    unsigned i;
    int** imArray;
    unsigned ROWS = height;
    unsigned COLUMNS = width;
    double mean1,total=0.0;
    double var = 0;

    imArray = new int*[ROWS];
    for (i = 0; i < ROWS; ++i)
    {
        imArray[i] = new int[COLUMNS];
    }

    for (int m = 0; m < height; m++)
    {
        for (int n = 0; n < width; n++)
        {
            s = cvGet2D(img,m,n);
            imArray[m][n]=s.val[0];
        }
    }
    cout<<"SIZE OF ARRAY IS: "<<sizeof(imArray)<<endl;


    /**Calculate the mean of the image*/
    for (int u = 0; u < height; u++)
    {
        for (int v = 0; v < width; v++)
        {
            total +=imArray[u][v];
        }
    }
    mean1 = (total/ (height * width));
    cout << "Mean: " << mean1  << endl;
    mean = mean1;

    /**Calculate the standard deviation of the image*/

    for (int a = 0; a < img->height; a++)
    {
        for (int b = 0; b < img->width; b++)
        {
            var += (imArray[a][b] - mean1) * (imArray[a][b] - mean1);
        }
    }
    var = var/(img->height * img->width*mean1);
    variance = var;

    for(unsigned int num=0; num<ROWS; ++num)
        delete[] imArray[num];
    delete[] imArray;

}

double FUNCTIONS::GetFocusNormalizedVariance(const Mat image)
{
    cv::Scalar mean,var;
    double normvar;
    //Mat igray;
    //cvtColor(image, igray, CV_RGB2GRAY);
    cv::meanStdDev(image,mean,var);
    var[0]=pow(var[0],2.);
    normvar=var[0]/mean[0];
    return normvar;
}


double FUNCTIONS::GetLaplacian(const Mat image)
{
    cv::Scalar cost;
    Mat igray,iL,square;
    //cvtColor(image, igray, CV_RGB2GRAY);
    Laplacian(image,iL,CV_32F,1,1,0);//<-Laplacien open_cv_2
    multiply(iL,iL,square);//<-
    cost=cv::sum(square);//<-
    return cost[0];
}

double FUNCTIONS::GetFocusBrenner(const IplImage * image)
{
    double focus = 0;

    CvScalar pixel;
    CvScalar pixelPlusTwo;

    for(int j=0; j<image->height; j++)
        for(int i=0; i<image->width-2; i++)
        {
            pixel = cvGet2D(image,j,i);
            pixelPlusTwo = cvGet2D(image,j,i+2);

            focus = focus + (pixel.val[0] - pixelPlusTwo.val[0]) * (pixel.val[0] - pixelPlusTwo.val[0]);
        }

    return focus;
}

double FUNCTIONS::GetFocusStandardDeviationCorrelation(const IplImage * image)
{

    double focus = 0;

    CvScalar pixel;
    CvScalar pixelPlusOne;

    CvScalar mean = cvAvg(image);

    double temp =  mean.val[0]*mean.val[0]*image->height*(image->width-1);

    for(int j=0; j<image->height; j++)
        for(int i=0; i<image->width-1; i++)
        {
            pixel = cvGet2D(image,j,i);
            pixelPlusOne = cvGet2D(image,j,i+1);

            focus = focus + pixel.val[0]*pixelPlusOne.val[0];
        }

    focus = focus -temp;

    return focus;

}

double FUNCTIONS::GetFocusAutoCorrelation(const IplImage * image)
{
    double focus = 0;
    double focus1 = 0;
    double focus2 = 0;

    CvScalar pixel;
    CvScalar pixelPlusOne;
    CvScalar pixelPlusTwo;

    for(int j=0; j<image->height; j++)
        for(int i=0; i<image->width-2; i++)
        {
            pixel = cvGet2D(image,j,i);
            pixelPlusOne = cvGet2D(image,j,i+1);
            pixelPlusTwo = cvGet2D(image,j,i+2);

            focus1 = focus1 + pixel.val[0]*pixelPlusOne.val[0];
            focus2 = focus2 + pixel.val[0]*pixelPlusTwo.val[0];
        }

    focus = focus1 - focus2;

    return focus;

}

//---------------------------------------------------------------------------
double FUNCTIONS::GetFocusWavelet1(const IplImage * image)
{

    const int half_height = image->height/2;
    const int half_width = image->width/2;

    double focus = 0;
    CvScalar pixel;

    int i,j;

    /* HL */
    for(j=0; j<half_height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + abs(pixel.val[0]);
        }

    /* LH */
    for(j=half_height; j<image->height; j++)
        for(i=0; i<half_width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + abs(pixel.val[0]);
        }

    /* HH */
    for(j=half_height; j<image->height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + abs(pixel.val[0]);
        }

    return focus;
}

//---------------------------------------------------------------------------
double FUNCTIONS::GetFocusWavelet2(const IplImage * image)
{

    const int half_height = image->height/2;
    const int half_width = image->width/2;

    double focus = 0;
    CvScalar pixel;

    int i,j;

    double meanHL = 0;
    double meanLH = 0;
    double meanHH = 0;

    /* HL */
    int nbPixel =0;
    for(j=0; j<half_height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanHL = meanHL + abs(pixel.val[0]);
            nbPixel++;
        }

    meanHL = meanHL/nbPixel;

    /* LH */
    nbPixel =0;
    for(j=half_height; j<image->height; j++)
        for(i=0; i<half_width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanLH = meanLH + abs(pixel.val[0]);
            nbPixel++;
        }

    meanLH = meanLH/nbPixel;

    /* HH */
    nbPixel = 0;
    for(j=half_height; j<image->height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanHH = meanHH + abs(pixel.val[0]);
            nbPixel++;
        }

    meanHH = meanHH/nbPixel;


    /* HL */
    for(j=0; j<half_height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanHL)*(abs(pixel.val[0])-meanHL);
        }

    /* LH */
    for(j=half_height; j<image->height; j++)
        for(i=0; i<half_width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanLH)*(abs(pixel.val[0])-meanLH);
        }

    /* HH */
    for(j=half_height; j<image->height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanHH)*(abs(pixel.val[0])-meanHH);
        }


    focus = focus/(half_height*half_width);

    return focus;
}
//---------------------------------------------------------------------------
double FUNCTIONS::GetFocusWavelet3(const IplImage * image)
{

    const int half_height = image->height/2;
    const int half_width = image->width/2;

    double focus = 0;
    CvScalar pixel;

    int i,j;

    double meanHL = 0;
    double meanLH = 0;
    double meanHH = 0;

    /* HL */
    int nbPixel =0;
    for(j=0; j<half_height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanHL = meanHL + pixel.val[0];
            nbPixel++;
        }

    meanHL = meanHL/nbPixel;

    /* LH */
    nbPixel =0;
    for(j=half_height; j<image->height; j++)
        for(i=0; i<half_width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanLH = meanLH + pixel.val[0];
            nbPixel++;
        }

    meanLH = meanLH/nbPixel;

    /* HH */
    nbPixel = 0;
    for(j=half_height; j<image->height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            meanHH = meanHH + pixel.val[0];
            nbPixel++;
        }

    meanHH = meanHH/nbPixel;


    /* HL */
    for(j=0; j<half_height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanHL)*(abs(pixel.val[0])-meanHL);
        }

    /* LH */
    for(j=half_height; j<image->height; j++)
        for(i=0; i<half_width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanLH)*(abs(pixel.val[0])-meanLH);
        }

    /* HH */
    for(j=half_height; j<image->height; j++)
        for(i=half_width; i<image->width; i++)
        {
            pixel = cvGet2D(image,j,i);
            focus = focus + (abs(pixel.val[0])-meanHH)*(abs(pixel.val[0])-meanHH);
        }


    focus = focus/(half_height*half_width);

    return focus;
}
