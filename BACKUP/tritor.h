#ifndef TRITOR_H
#define TRITOR_H

#include <math.h>

class tritor
{
public:
    /** Default constructor */
    tritor();
    /** Default destructor */
    virtual ~tritor();
    double move_inc_x(double disp);
    double move_dec_x(double disp);
    double move_inc_y(double disp);
    double move_dec_y(double disp);
protected:

private:
    double p1,p2,p3,p4,p5;
    double volt;
};

#endif // TRITOR_H
