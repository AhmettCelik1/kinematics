#include <iostream>
#include <cmath>

#define L1 1.0 // length of first link (base to arm)
#define L2 0.5 // length of second link (arm to gripper)
#define L3 0.3 // length of third link (gripper fingers)

using namespace std;

void inverseKinematics(double x, double y, double z)
{
    double r = sqrt(x*x + y*y);
    double s = z - L1;
    double D = (r*r + s*s - L2*L2 - L3*L3) / (2.0 * L2 * L3);

    double theta1 = atan2(y, x);
    double theta2 = atan2(s, r) - atan2(L3 * sqrt(1.0 - D*D), L2 + L3*D);
    double theta3 = acos(D);

    // convert to degrees
    theta1 = theta1 * 180.0 / M_PI;
    theta2 = theta2 * 180.0 / M_PI;
    theta3 = theta3 * 180.0 / M_PI;

    cout << "Joint angles: (" << theta1 << ", " << theta2 << ", " << theta3 << ")" << endl;
}

int main()
{
    double x, y, z;

    // set end effector position
    x = 0.5;
    y = 0.5;
    z = 1.0;

    inverseKinematics(x, y, z);

    return 0;
}

