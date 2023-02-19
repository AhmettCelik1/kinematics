#include <iostream>
#include <cmath>

#define L1 1.0 // length of first link (base to arm)
#define L2 0.5 // length of second link (arm to gripper)
#define L3 0.3 // length of third link (gripper fingers)

using namespace std;

int main()
{
    double theta1, theta2, theta3;
    double x, y, z;

    // set joint angles (in radians)
    theta1 = 0.0; // base joint
    theta2 = M_PI / 2.0; // arm joint
    theta3 = M_PI / 6.0; // gripper joint

    // calculate end effector position
    x = cos(theta1) * (L1 + L2 * cos(theta2) + L3 * cos(theta2 + theta3));
    y = sin(theta1) * (L1 + L2 * cos(theta2) + L3 * cos(theta2 + theta3));
    z = L2 * sin(theta2) + L3 * sin(theta2 + theta3);

    // print results
    cout << "End effector position: (" << x << ", " << y << ", " << z << ")" << endl;

    return 0;
}

