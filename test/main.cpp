#include <iostream>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace Eigen;

// Define robot parameters
const double pi = 3.14159265358979323846;
const double base_length = 1.0;    // Length of the base joint to arm joint
const double arm_length = 0.5;     // Length of the arm joint to gripper joint
const double gripper_length = 0.3; // Length of the gripper

// Forward kinematics function
Matrix4d forward_kinematics(double theta1, double theta2, double theta3)
{
    // Convert joint angles from degrees to radians
    double theta1_rad = theta1 * pi / 180.0;
    double theta2_rad = theta2 * pi / 180.0;
    double theta3_rad = theta3 * pi / 180.0;

    // Calculate transformation matrices
    Matrix4d T_base_to_arm;
    T_base_to_arm << cos(theta1_rad), 0, sin(theta1_rad), 0,
        0, 1, 0, 0,
        -sin(theta1_rad), 0, cos(theta1_rad), base_length,
        0, 0, 0, 1;

    Matrix4d T_arm_to_gripper;
    T_arm_to_gripper << cos(theta2_rad), -sin(theta2_rad), 0, arm_length * cos(theta2_rad),
        sin(theta2_rad), cos(theta2_rad), 0, arm_length * sin(theta2_rad),
        0, 0, 1, 0,
        0, 0, 0, 1;

    Matrix4d T_gripper;
    T_gripper << cos(theta3_rad), 0, -sin(theta3_rad), gripper_length * cos(theta3_rad),
        0, 1, 0, 0,
        sin(theta3_rad), 0, cos(theta3_rad), gripper_length * sin(theta3_rad),
        0, 0, 0, 1;

    // Calculate final transformation matrix
    Matrix4d T_base_to_gripper = T_base_to_arm * T_arm_to_gripper * T_gripper;

    return T_base_to_gripper;
}

// Inverse kinematics function
bool inverse_kinematics(const Matrix4d &T_base_to_gripper, double &theta1, double &theta2, double &theta3)
{
    // Extract position and orientation from transformation matrix
    Vector3d p(T_base_to_gripper(0, 3), T_base_to_gripper(1, 3), T_base_to_gripper(2, 3));
    Matrix3d R = T_base_to_gripper.block<3, 3>(0, 0);

    // Calculate theta1
    theta1 = atan2(p[2], p[0]);

    // Calculate theta2 and theta3
    double a = arm_length + gripper_length * cos(pi / 6);
    double b = gripper_length * sin(pi / 6);
    double c = sqrt(p[0] * p[0] + p[2] * p[2]) - base_length;
    double alpha = atan2(b, a);
    double beta = atan2(c, sqrt(a * a + b * b - c * c));
    double gamma = atan2(sqrt(a * a + b * b - c * c), c);

    double theta2a = pi / 2 - alpha - beta;
    double theta2b = pi / 2 - alpha + beta;
    double theta3a = pi / 2 - gamma;
    double theta3b = pi / 2 + gamma;

    // Check if solution is valid
    if (theta2a < 0 || theta2a > pi || theta3a < 0 || theta3a > pi)
    {
        if (theta2b < 0 || theta2b > pi || theta3b < 0 || theta3b > pi)
        {
            return false;
        }
        else
        {
            theta2 = theta2b;
            theta3 = theta3b;
        }
    }
    else
    {
        theta2 = theta2a;
        theta3 = theta3a;
    }

    return true;
}

int main()
{

    int choice;
    std::cout << "Enter 1 for forward kinematics or 2 for inverse kinematics: ";
    std::cin >> choice;

    if (choice == 1)
    {
        // Get joint angles from user
        double theta1, theta2, theta3;
        std::cout << "Enter joint angles in degrees: ";
        std::cin >> theta1 >> theta2 >> theta3;

        // Calculate transformation matrix
        Matrix4d T_base_to_gripper = forward_kinematics(theta1, theta2, theta3);

        // Print transformation matrix
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << T_base_to_gripper << std::endl;

        // Convert transformation matrix to pose message
        tf::Matrix3x3 R(T_base_to_gripper(0, 0), T_base_to_gripper(0, 1), T_base_to_gripper(0, 2),
                        T_base_to_gripper(1, 0), T_base_to_gripper(1, 1), T_base_to_gripper(1, 2),
                        T_base_to_gripper(2, 0), T_base_to_gripper(2, 1), T_base_to_gripper(2, 2));
        tf::Vector3 p(T_base_to_gripper(0, 3), T_base_to_gripper(1, 3), T_base_to_gripper(2, 3));
        tf::Quaternion q;
        R.getRotation(q);

        // Print pose message
        std::cout << "Pose message:" << std::endl;
        std::cout << "Position: " << p.x() << ", " << p.y() << ", " << p.z() << std::endl;
        std::cout << "Orientation: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
    }
    else if (choice == 2)
    {
        // Get pose message from user
        double x, y, z, roll, pitch, yaw;
        std::cout << "Enter pose message: ";
        std::cin >> x >> y >> z >> roll >> pitch >> yaw;

        // Convert pose message to transformation matrix
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf::Matrix3x3 R(q);
        tf::Vector3 p(x, y, z);
        Matrix4d T_base_to_gripper;
        T_base_to_gripper << R[0][0], R[0][1], R[0][2], p[0],
            R[1][0], R[1][1], R[1][2], p[1],
            R[2][0], R[2][1], R[2][2], p[2],
            0, 0, 0, 1;

        // Calculate joint angles
        double theta1, theta2, theta3;
        if (inverse_kinematics(T_base_to_gripper, theta1, theta2, theta3))
        {
            // Print joint angles
            std::cout << "Joint angles: " << theta1 << ", " << theta2 << ", " << theta3 << std::endl;
            // in degrees
            std::cout << "Joint angles: " << theta1 * 180 / pi << ", " << theta2 * 180 / pi << ", " << theta3 * 180 / pi << std::endl;
        }
        else
        {
            std::cout << "No solution found." << std::endl;
        }
    }
    else
    {
        std::cout << "Invalid choice." << std::endl;
    }

    return 0;
}