#include "tritor.h"

tritor::tritor()
{
    p1=0;
    p2=0;
    p3=0;
    p4=0;
    p5=0;
    volt = 0;
}

tritor::~tritor()
{
    p1=0;
    p2=0;
    p3=0;
    p4=0;
    p5=0;
}

double tritor::move_inc_x(double disp)
{
    p1 = -5.3343e-007;
    p2 = 0.00017528;
    p3 = -0.017184;
    p4 = 1.6457;
    p5 = -19.018;
    volt = p1*pow(disp,4)+p2*pow(disp,3)+p3*pow(disp,2)+p4*disp+p5;
    if(volt>-19 && volt<110)
        return volt;
    else
        return 0.0;
}

double tritor::move_inc_y(double disp)
{
    p1 = -5.6939e-007;
    p2 = 0.00018878;
    p3 = -0.020198;
    p4 = 1.8527;
    p5 = -19.171;
    volt = p1*pow(disp,4)+p2*pow(disp,3)+p3*pow(disp,2)+p4*disp+p5;
    if(volt>-19 && volt<110)
        return volt;
    else
        return 0.0;
}

double tritor::move_dec_x(double disp)
{
    p1 = 2.6118e-007;
    p2 = -2.2086e-005;
    p3 = 0.0032187;
    p4 = 0.73526;
    p5 = -21.086;
    volt = p1*pow(disp,4)+p2*pow(disp,3)+p3*pow(disp,2)+p4*disp+p5;
    if(volt>-19 && volt<110)
        return volt;
    else
        return 0.0;
}

double tritor::move_dec_y(double disp)
{
    p1 = 4.9101e-007;
    p2 = -7.3072e-005;
    p3 = 0.0056593;
    p4 = 0.77264;
    p5 = -22.073;
    volt = p1*pow(disp,4)+p2*pow(disp,3)+p3*pow(disp,2)+p4*disp+p5;
    if(volt>-19 && volt<110)
        return volt;
    else
        return 0.0;
}
