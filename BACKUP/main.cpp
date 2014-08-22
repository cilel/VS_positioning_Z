#include <iostream>
#include <math.h>
#include "fftw3.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "Client.h"
#include "myLog.h"
#include "configinfo.h"

using namespace cv;
using namespace std;

myLog winLog;
string serverIPAddress = "";

int main()
{

    Client info;                                                //! CLIENT Object
    configinfo uHostAddress;
    string localHostName = uHostAddress.getHostName();          //! Read Host Name
    string localHostAddr = uHostAddress.getHostIPAddress();     //! Read Host address
    uHostAddress.readServerConfig();                            //! Address Configuration


    Client myclient(uHostAddress.serverIPAddress);              //! connect to the server and perform visual servoing
    return 0;
}
